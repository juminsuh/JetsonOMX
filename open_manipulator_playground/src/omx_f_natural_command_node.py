# natural_command_node.py — Bridge mode (no LLM inside)

import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetCartesianPath, GetPositionFK
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import base64
import requests
import threading
from datetime import datetime

# 링크 길이들 (미터 단위)
L2 = 0.128
L3 = 0.124

# 연속 회전에 관련된 상수들
KEEP_ROTATE_SPEED_DEG_S = 10.0 # 회전 속도
KEEP_TIMER_HZ = 20.0 # 회전 타이머 주파수
KEEP_DT = 1.0 / KEEP_TIMER_HZ # 타이머 주기

JOINT_LIMITS = {
    "joint1": (math.radians(-270), math.radians(360)),
    "joint2": (math.radians(-120), math.radians(90)),
    "joint3": (math.radians(-120), math.radians(90)),
    "joint4": (math.radians(-100), math.radians(100)),
    "joint5": (math.radians(-270), math.radians(270)),
}
ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5"]

# 정리된 json을 받아 로봇에게 publish하는 노드
class NaturalCommandNode(Node):
    def __init__(self):
        # 노드 초기화 생성자
        super().__init__('natural_command_node')

        # 관절 명령 퍼블리셔 생성
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        # 그리퍼 제어 퍼블리셔 생성
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        # IK(역기구학) 계산과 카르테시안 경로 계산을 위한 서비스 클라이언트
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')

        # ik 서비스가 준비될 때까지 대기 (동기 로그 출력)
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')
        while not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_cartesian_path service...')
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_fk service...')
        # 현재 관절 상태(라디안) 초기값 => joint5 는 그리퍼 회전인데 적용 안 한 것같 음
        self.current_joint1_pos = 0.0
        self.current_joint2_pos = 0.0
        self.current_joint3_pos = 0.0
        self.current_joint4_pos = 0.0
        self.current_joint5_pos = 0.0
        self.current_z = L2 # 엔드이펙터 높이 추적용 (초기값으로 L2 사용)

        # 현재 엔드이펙터(로봇팔의 손부분) 포즈를 저장할 변수 (PoseStamped 메시지)
        self.current_ee_pose = PoseStamped()
        self.current_ee_pose.header.frame_id = "world"
        self.current_ee_pose.pose.position.x = 0.0
        self.current_ee_pose.pose.position.y = 0.0
        # z값은 엔드이펙터의 높이
        self.current_ee_pose.pose.position.z = self.current_z
        self.current_ee_pose.pose.orientation.w = 1.0

        # 브리지 모드 활성화 로그 출력
        self.get_logger().info("Bridge mode active (no LLM parser, JSON directly expected)")

        # --- VLA Configuration ---
        self.declare_parameter('vla_api_url', 'http://100.82.52.106:8080/api/vla/infer')
        self.declare_parameter('vla_api_timeout', 10.0)
        self.declare_parameter('vla_image_topic', '/camera/image_raw')
        self.declare_parameter('vla_prompt', 'Move the robot arm to pick up the object')
        self.declare_parameter('enable_image_compression', True)
        self.declare_parameter('image_quality', 85)

        self.vla_api_url = self.get_parameter('vla_api_url').value
        self.vla_api_timeout = self.get_parameter('vla_api_timeout').value
        self.vla_prompt = self.get_parameter('vla_prompt').value
        vla_image_topic = self.get_parameter('vla_image_topic').value
        self.vla_enable_compression = self.get_parameter('enable_image_compression').value
        self.vla_image_quality = self.get_parameter('image_quality').value

        self.cv_bridge = CvBridge()
        self.latest_image = None
        self.image_lock = threading.Lock()
        
        self.vla_image_sub = self.create_subscription(
            Image,
            vla_image_topic,
            self.vla_image_callback,
            10
        )
        
        self.vla_active = False
        self.vla_loop_active = False
        # -------------------------

        # 연속 회전 관련 내부 상태
        self._keep_timer = None
        self._keep_dir = None
        self._keep_w_rad_s = math.radians(KEEP_ROTATE_SPEED_DEG_S)

        # 홈 포즈 (관절 각도 라디안)
        self.home_pose = [0.0, -1.57, 1.57, 1.57, 0.0 ]

    def process_command(self, cmd):
        try:
            action = cmd.get("action")

            if action != "stop":
                if self._keep_timer is not None:
                    self._stop_keep()
                    self.get_logger().info("Keep mode auto-stopped before new command.")

            if action == "stop":
                self._stop_keep()
                self.get_logger().info("Stopped continuous rotation.")
                return

            if action == "move_xyz":
                x, y, z = cmd["xyz"]
                self.send_ik_request(x, y, z)
                return

            if action == "move":
                dx, dy, dz = cmd["xyz"]
                roll, pitch, yaw = cmd["rpy"]
                self.move_with_cartesian(dx, dy, dz, roll, pitch, yaw)
                return
            if action == "initialize":
                self.reset_pose()
                return

            if action == "home":
                self.go_home_pose()
                return

            if action == "joint5":
                pos_deg = float(cmd.get("value", 0.0)) # 값으로 움직이게 함

                if pos_deg is None:
                    self.get_logger().warn(f"Unknown gripper direction: {cmd.get('direction')}")
                    return

                pos_rad = math.radians(pos_deg)
                self.current_joint5_pos += pos_rad
                traj = JointTrajectory()
                traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', "joint5"]
                pt = JointTrajectoryPoint()
                pt.positions = [
                    self.current_joint1_pos,
                    self.current_joint2_pos,
                    self.current_joint3_pos,
                    self.current_joint4_pos,
                    self.current_joint5_pos
                ]
                pt.time_from_start.sec = 2
                traj.points.append(pt)
                self.arm_pub.publish(traj)
                self.get_logger().info(f"Rotate {direction} {value}{unit} executed")
                return
            
            if action == "gripper":
                # 그리퍼 값 미리 지정해두고 open, close만 처리했음
                # openVLA랑 연결할 땐 value 받아서 입력하면 될 것으로 추정
                # pos_map_deg = {"open": 57.0, "close": 0.0, "reset": 0.0}
                pos_deg = float(cmd.get("value", 0.0)) # 값으로 움직이게 함

                if pos_deg is None:
                    self.get_logger().warn(f"Unknown gripper direction: {cmd.get('direction')}")
                    return

                # 그리퍼 음수값 지정불가
                if pos_deg < 0:
                    pos_deg = 0

                pos_rad = math.radians(pos_deg)
                goal = GripperCommand.Goal()
                goal.command = GripperCommandMsg()
                goal.command.position = pos_rad
                goal.command.max_effort = 1.0 # 그리퍼를 사용할 때 최대힘을 지정함
                self.gripper_client.wait_for_server()
                self.gripper_client.send_goal_async(goal)
                self.get_logger().info(f"Gripper '{cmd['direction']}' executed")
                return
            
            if action == "look":
                direction = (cmd.get("direction") or "").lower()
                self.look_command(direction)
                return

            if action == "vla_step":
                prompt = cmd.get("prompt", self.vla_prompt)
                threading.Thread(target=self.execute_vla_step, args=(prompt,), daemon=True).start()
                return

            if action == "vla_loop":
                prompt = cmd.get("prompt", self.vla_prompt)
                self.vla_loop_active = True
                threading.Thread(target=self.vla_loop_thread, args=(prompt,), daemon=True).start()
                return

            if action == "vla_stop":
                self.vla_loop_active = False
                self.get_logger().info("VLA loop stop requested")
                return
            
            if action in ("rotate", "move"):
                direction = cmd.get("direction") # 방향
                value = cmd.get("value") # 값
                unit = cmd.get("unit") # 단위

                # 회전 중에서 움직임을 계속하고 싶을 때 'joint1', 'joint2', 'joint3', 'joint4'(ex 계속 왼쪽으로 움직여라)
                if (action == "rotate") and isinstance(value, str) and value.strip().lower() == "keep":
                    self._start_keep(direction)
                    return

                # 보통 회전
                if action == "rotate":
                    # 각도 변환
                    if unit and unit.lower() in ("degree", "deg"): 
                        delta = math.radians(float(value))
                    else:
                        delta = self.get_delta(value, unit, 1.0)

                    # 좌우 회전은 joint1 
                    if direction == "left":
                        self.current_joint1_pos += delta
                    elif direction == "right":
                        self.current_joint1_pos -= delta
                    
                    # 상하 회전은 joint 3
                    elif direction == "up":
                        self.current_joint3_pos -= delta
                    elif direction == "down":
                        self.current_joint3_pos += delta
                    else:
                    # 나머지 회전은 지원 X 우리가 원하는 3D 회전은 구현해야하는듯함
                        self.get_logger().warn(f"Unsupported rotate direction: {direction}")
                        return

                    # trajectory 생성해서 쏴주기
                    traj = JointTrajectory()
                    traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', "joint5"]
                    pt = JointTrajectoryPoint()
                    pt.positions = [
                        self.current_joint1_pos,
                        self.current_joint2_pos,
                        self.current_joint3_pos,
                        self.current_joint4_pos,
                        self.current_joint5_pos
                    ]
                    pt.time_from_start.sec = 2
                    traj.points.append(pt)
                    self.arm_pub.publish(traj)
                    self.get_logger().info(f"Rotate {direction} {value}{unit} executed")
                    return

                # 좌표 X 인 움직임일 때
                elif action == "move":  
                    # 값이나 단위가 안들어오면 그냥 5cm 움직임
                    if value is None and unit is None:
                        value, unit = 5, "cm"
                        self.get_logger().info("No value/unit provided → defaulting to 5 cm")

                    # delta값 구하기 - 로봇팔의 길이, 움직일 길이를 사용해 움직일 joint 값을 미리 구해놓는 과정
                    step = self.get_delta(value, unit, 1.0)
                    if step == 0.0: # 0 이면 안움직임
                        self.get_logger().warn(f"Invalid or zero distance: {value} {unit}")
                        return
                    # 앞뒤 움직임만 지원하고 있음
                    if direction == "forward":
                        self.move_forward_backward(+step)
                    elif direction == "backward":
                        self.move_forward_backward(-step)
                    else:
                        self.get_logger().warn(f"Unsupported move direction: {direction}")
                    return

            self.get_logger().warn(f"Unknown action: {action}")

        except Exception as e:
            self.get_logger().error(f"process_command error: {e}")

    # home pose로 돌아가기
    def go_home_pose(self):
        # joint trajectory에서 joint값 설정하기
        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        pt = JointTrajectoryPoint()
        pt.positions = self.home_pose # 미리 지정한거 설정하기
        pt.time_from_start.sec = 2
        traj.points.append(pt)
        self.arm_pub.publish(traj)
        
        # 내부 상태를 홈 포즈로 업데이트
        (self.current_joint1_pos,
         self.current_joint2_pos,
         self.current_joint3_pos,
         self.current_joint4_pos,
         self.current_joint5_pos) = self.home_pose

        # 그리퍼가 열려있으면 닫는 액션을 전송
        goal = GripperCommand.Goal()
        goal.command = GripperCommandMsg()
        goal.command.position = 0.0
        goal.command.max_effort = 1.0
        self.gripper_client.wait_for_server()
        self.gripper_client.send_goal_async(goal)

        self.get_logger().info("Moved to home pose (gripper closed)")

    def move_forward_backward(self, step_m):
        """간단화된 전진/후진 구현: 링크 길이 L2를 사용해 joint2/joint3를 보정.

        이 함수는 엔드이펙터의 순수 직선 이동을 엄밀히 계산하지 않고,
        joint2와 joint3의 델타를 L2에 비례해서 변경하는 단순화된 모델을 사용합니다.

        :param step_m: 이동 거리(미터), 양수는 전진, 음수는 후진
        """

        # 거리 / L2 를 각도(delta_theta)로 근사
        delta_theta = step_m / L2
        self.current_joint2_pos += delta_theta
        self.current_joint3_pos -= delta_theta
        
        # 업데이트한 관절각으로 Trajectory 발행
        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        pt = JointTrajectoryPoint() # 움직일 경로 확인
        pt.positions = [
            self.current_joint1_pos,
            self.current_joint2_pos,
            self.current_joint3_pos,
            self.current_joint4_pos
        ]
        pt.time_from_start.sec = 2 # 2초뒤에 실행
        traj.points.append(pt)
        self.arm_pub.publish(traj)

        self.get_logger().info(
            f"Simplified move step={step_m:+.3f} m "
            f"(joint2={self.current_joint2_pos:.2f}, joint3={self.current_joint3_pos:.2f})"
        )

    def look_command(self, direction: str):
        """엔드이펙터 시선(피치) 조절을 위해 joint4를 계산하여 이동.

        joint2 + joint3 의 합(theta23)을 기반으로 joint4 값을 설정하면
        엔드이펙터가 위/아래를 바라보는 효과를 얻을 수 있습니다.
        """
        theta23 = self.current_joint2_pos + self.current_joint3_pos

        # look은 집게 위치만 바꾸면 됨
        # 현재는 90도 기준 업 다운만 가능함
        # up도 지정된 관절로만 이동가능함
        if direction == "up":
            self.current_joint4_pos = -theta23
        elif direction == "down":
            self.current_joint4_pos = -theta23 + math.pi / 2
        else:
            self.get_logger().warn(f"Unsupported look direction: {direction}")
            return

        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        pt = JointTrajectoryPoint()
        pt.positions = [
            self.current_joint1_pos,
            self.current_joint2_pos,
            self.current_joint3_pos,
            self.current_joint4_pos
        ]
        pt.time_from_start.sec = 2
        traj.points.append(pt)
        self.arm_pub.publish(traj)

        self.get_logger().info(
            f"Look {direction} executed (joint4={self.current_joint4_pos:.2f} rad)"
        )

    """다양한 단위를 관절 회전(라디안) 혹은 거리->관절 델타로 변환하는 유틸 함수.

    - degree/deg: 라디안 변환
    - cm, mm, m, inch: 거리 단위를 미터로 바꿔 radius(예: 관절 반경)로 나눈 값
    """
    def get_delta(self, value, unit, radius):
        if unit and unit.lower() in ("degree", "deg"):
            return math.radians(float(value))
        elif unit and unit.lower() == "cm":
            return (float(value) / 100.0) / radius if radius != 0 else 0.0
        elif unit and unit.lower() in ("mm",):
            return (float(value) / 1000.0) / radius if radius != 0 else 0.0
        elif unit and unit.lower() in ("m",):
            return (float(value) / radius) if radius != 0 else 0.0
        elif unit and unit.lower() in ("inch", "in"):
            return ((float(value) * 0.0254) / radius) if radius != 0 else 0.0
        return 0.0

    # initial pose로 움직이기
    def reset_pose(self):
        self.current_joint1_pos = 0.0
        self.current_joint2_pos = 0.0
        self.current_joint3_pos = 0.0
        self.current_joint4_pos = 0.0
        self.current_z = L2
        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        pt = JointTrajectoryPoint()
        pt.positions = [0.0, 0.0, 0.0, 0.0]
        pt.time_from_start.sec = 2
        traj.points.append(pt)
        self.arm_pub.publish(traj)

        # 그리퍼 초기화(여기선 0 라디안)
        goal = GripperCommand.Goal()
        goal.command = GripperCommandMsg()
        goal.command.position = 0.00
        goal.command.max_effort = 1.0
        self.gripper_client.wait_for_server()
        self.gripper_client.send_goal_async(goal)
        self.get_logger().info("Initialization complete")

    def send_ik_request(self, x, y, z):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.ik_link_name = "end_effector_link"
        req.ik_request.pose_stamped = pose
        req.ik_request.timeout.sec = 2
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res and res.error_code.val == 1:
            j = res.solution.joint_state
            name_to_pos = dict(zip(j.name, j.position))
            self.current_joint1_pos = name_to_pos.get('joint1', 0.0)
            self.current_joint2_pos = name_to_pos.get('joint2', 0.0)
            self.current_joint3_pos = name_to_pos.get('joint3', 0.0)
            self.current_joint4_pos = name_to_pos.get('joint4', 0.0)
            self.current_z = L2 + L3 * math.sin(self.current_joint3_pos)
            traj = JointTrajectory()
            traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
            pt = JointTrajectoryPoint()
            pt.positions = [
                self.current_joint1_pos,
                self.current_joint2_pos,
                self.current_joint3_pos,
                self.current_joint4_pos
            ]
            pt.time_from_start.sec = 2
            traj.points.append(pt)
            self.arm_pub.publish(traj)
            self.get_logger().info(f"IK-based movement completed: {pt.positions}")
        else:
            code = res.error_code.val if res else -1
            self.get_logger().error(f"IK computation failed (code: {code})")

    def get_current_ee_pose(self):
        req = GetPositionFK.Request()
        req.header.frame_id = "world"
        req.fk_link_names = ["end_effector_link"]
        req.robot_state.joint_state.name = ARM_JOINTS
        req.robot_state.joint_state.position = [
            self.current_joint1_pos,
            self.current_joint2_pos,
            self.current_joint3_pos,
            self.current_joint4_pos,
            self.current_joint5_pos,
        ]

        future = self.fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        self.get_logger().info(f"current_x = {res.pose_stamped[0].pose.position.x}")
        self.get_logger().info(f"current_y = {res.pose_stamped[0].pose.position.y}")
        self.get_logger().info(f"current_z = {res.pose_stamped[0].pose.position.z}")
        self.get_logger().info(f"orientation = {res.pose_stamped[0].pose.orientation}")
        if res and res.error_code.val == 1:
            return res.pose_stamped[0]
        else:
            self.get_logger().error("❌ FK failed")
            return None


    def move_with_cartesian(self, dx, dy, dz, roll=0, pitch=0, yaw=0):
        current_pose = self.get_current_ee_pose()
        if current_pose is None:
            return False

        target_x = current_pose.pose.position.x + dx
        target_y = current_pose.pose.position.y + dy
        target_z = current_pose.pose.position.z + dz

        self.get_logger().info(
            f"🎯 Cartesian-like IK target = "
            f"({target_x:.3f}, {target_y:.3f}, {target_z:.3f})"
        )

        return self.send_ik_request(target_x, target_y, target_z)


    def check_joint_limits(self, joint_values: dict) -> bool:
        for name, value in joint_values.items():
            min_lim, max_lim = JOINT_LIMITS[name]
            if not (min_lim <= value <= max_lim):
                self.get_logger().error(
                    f"🚨 Joint limit exceeded: {name} = "
                    f"{math.degrees(value):.2f}° "
                    f"(limit: {math.degrees(min_lim):.1f}° ~ {math.degrees(max_lim):.1f}°)"
                )
                return False
        return True

    # 회전 상태를 유지하는 함수 (계속 왼쪽으로 돌아라 같은 연속적인 움직임을 수행)
    def _start_keep(self, direction: str):
        d = (direction or "").lower()
        if d not in ("left", "right", "up", "down"):
            self.get_logger().warn(f"Unsupported direction for continuous rotate: {direction}")
            return
        # 기존 타이머가 있으면 정지
        self._stop_keep()
        self._keep_dir = d
        # 일정 주기로 _on_keep_tick가 호출되도록 타이머 등록
        self._keep_timer = self.create_timer(KEEP_DT, self._on_keep_tick)
        self.get_logger().info(f"Continuous rotate '{d}' at {KEEP_ROTATE_SPEED_DEG_S:.1f} deg/s.")

    # 회전 상태를 종료하는 함수
    def _stop_keep(self):
        if self._keep_timer is not None:
            self._keep_timer.cancel()
            self._keep_timer = None
        self._keep_dir = None

    """연속 회전 타이머 콜백: 방향에 따라 관절 각도를 조금씩 변경하고 발행."""
    def _on_keep_tick(self):
        # 연속 모드 아니면 종료
        if not self._keep_dir:
            return
        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        pt = JointTrajectoryPoint()
        w = self._keep_w_rad_s
        d = self._keep_dir

        # 방향별 관절 각도 증가/감소
        if d == "left":
            self.current_joint1_pos += w * KEEP_DT
        elif d == "right":
            self.current_joint1_pos -= w * KEEP_DT
        elif d == "up":
            self.current_joint3_pos -= w * KEEP_DT
        elif d == "down":
            self.current_joint3_pos += w * KEEP_DT
        pt.positions = [
            self.current_joint1_pos,
            self.current_joint2_pos,
            self.current_joint3_pos,
            self.current_joint4_pos
        ]
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = int(KEEP_DT * 1e9)
        traj.points.append(pt)
        self.arm_pub.publish(traj)

    # --- VLA Bridge Methods ---
    def vla_image_callback(self, msg):
        """저장할 최신 이미지 업데이트"""
        with self.image_lock:
            self.latest_image = msg
        

    def image_to_base64(self, cv_image):
        """OpenCV 이미지를 Base64 문자열로 변환 (vla_bridge logic 복사)"""
        if self.vla_enable_compression:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.vla_image_quality]
            success, encoded_image = cv2.imencode('.jpg', cv_image, encode_param)
            if not success:
                self.get_logger().error("Failed to encode image")
                return None
            image_bytes = encoded_image.tobytes()
        else:
            success, encoded_image = cv2.imencode('.png', cv_image)
            if not success:
                self.get_logger().error("Failed to encode image")
                return None
            image_bytes = encoded_image.tobytes()

        return base64.b64encode(image_bytes).decode('utf-8')

    def execute_vla_step(self, prompt):
        """단일 VLA 스텝 실행: 이미지 캡처 -> API 요청 -> 움직임 실행"""
        if self.vla_active:
            self.get_logger().warn("VLA step already in progress...")
            return False
        
        self.vla_active = True
        try:
            with self.image_lock:
                if self.latest_image is None:
                    self.get_logger().warn("No image received yet for VLA")
                    return False
                image_msg = self.latest_image

            # 1. 이미지 변환
            cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            image_base64 = self.image_to_base64(cv_image)
            
            if image_base64 is None:
                return False

            # 2. VLA API 요청
            request_data = {
                "image": image_base64,
                "prompt": prompt,
                "unnorm_key": "bridge_orig"
            }
            
            self.get_logger().info(f"🚀 Sending VLA request with prompt: '{prompt}'")
            response = requests.post(
                self.vla_api_url,
                json=request_data,
                timeout=self.vla_api_timeout
            )

            if response.status_code == 200:
                result = response.json()
                self.get_logger().info(f"✅ VLA Response received: {result}")
                
                # 3. 움직임 실행
                # API 결과 형식에 따른 파싱 (vla_bridge logic 참고)
                joint_positions = result.get("joint_positions", [])
                gripper_val = result.get("gripper", "open")
                
                if joint_positions:
                    # 5DOF 로봇이라고 가정 (joint1~5)
                    traj = JointTrajectory()
                    traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
                    pt = JointTrajectoryPoint()
                    pt.positions = [float(p) for p in joint_positions[:5]]
                    pt.time_from_start.sec = 2
                    traj.points.append(pt)
                    self.arm_pub.publish(traj)
                    
                    # 내부 상태 업데이트
                    self.current_joint1_pos = pt.positions[0]
                    self.current_joint2_pos = pt.positions[1]
                    self.current_joint3_pos = pt.positions[2]
                    self.current_joint4_pos = pt.positions[3]
                    self.current_joint5_pos = pt.positions[4]
                
                if gripper_val:
                    # Gripper 처리
                    pos_deg = 57.0 if gripper_val == "open" else 0.0
                    if isinstance(gripper_val, (int, float)):
                        pos_deg = float(gripper_val)
                    
                    pos_rad = math.radians(pos_deg)
                    goal = GripperCommand.Goal()
                    goal.command = GripperCommandMsg()
                    goal.command.position = pos_rad
                    goal.command.max_effort = 1.0
                    self.gripper_client.wait_for_server()
                    self.gripper_client.send_goal_async(goal)
                
                # 움직임이 완료될 때까지 대기
                import time
                time.sleep(2.5) # 움직임 시간 대기
                return True
            else:
                self.get_logger().error(f"❌ VLA API failed: {response.status_code} - {response.text}")
                return False

        except Exception as e:
            self.get_logger().error(f"💥 VLA execution error: {str(e)}")
            return False
        finally:
            self.vla_active = False

    def vla_loop_thread(self, prompt):
        """VLA 루프 스레드"""
        self.get_logger().info("🔄 Starting VLA iterative loop...")
        while self.vla_loop_active and rclpy.ok():
            success = self.execute_vla_step(prompt)
            if not success:
                self.get_logger().warn("VLA step failed in loop, retrying in 1s...")
                import time
                time.sleep(1.0)
        self.get_logger().info("🛑 VLA loop finished.")
    # -------------------------


def main():
    rclpy.init()
    node = NaturalCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()