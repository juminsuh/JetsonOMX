# natural_command_node.py — Bridge mode (no LLM inside)

import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetCartesianPath
from kinematics.Kinematics import Kinematic

# 링크 길이들 (미터 단위)
L2 = 0.128
L3 = 0.124
L_GRIPPER = 0.126
L4 = 0.165 # 추정치
d5 = 0.09193 

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

        # ik 서비스가 준비될 때까지 대기 (동기 로그 출력)
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')
        while not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_cartesian_path service...')

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

        # 연속 회전 관련 내부 상태
        self._keep_timer = None
        self._keep_dir = None
        self._keep_w_rad_s = math.radians(KEEP_ROTATE_SPEED_DEG_S)

        # 홈 포즈 (관절 각도 라디안)
        self.home_pose = [0.0, -1.57, 1.57, 1.57, 0.0 ]
        # forward kinematics 모델 설정
        self.forward_kinematics = Kinematic()

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

            if action == "perfect_move":
                x, y, z = cmd["xyz"]
                roll, pitch, yaw = cmd["rpy"]
                self.move_with_ik(x, y, z, roll, pitch, yaw)
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
            
    def move_with_ik(self, dx, dy, dz, roll=0.0, pitch=0.0, yaw=0.0):
        
        # 현재 위치 가져오고 forward kinematics 해서 xyz 갱신
        current_joint = [self.current_joint1_pos,
                         self.current_joint2_pos,
                         self.current_joint3_pos,
                         self.current_joint4_pos,
                         self.current_joint5_pos,
                         0.0 # gripper 값 (생략)
                         ]
        
        """
        fk_position 구조
        [ ex.x  ey.x  ez.x px]
        [ ex.y  ey.y  ez.y py]
        [ ex.z  ey.z  ez.z pz]
        [  0     0     0   1 ]
        px py pz만 따서 사용하고 만약에 openVLA output이 ee(end effector) 기준이면
        ex.x ey.x ... 이거 사용해서 변환 후 더해야 함
        """
        fk_position = self.forward_kinematics.forward_kinematics(current_joint)
        current_x = fk_position[0][3] / 100.0 # cm -> m
        current_y = fk_position[1][3] / 100.0
        current_z = fk_position[2][3] / 100.0

        target_x = current_x + float(dx)
        target_y = current_y + float(dy)
        target_z = current_z + float(dz)

        self.get_logger().info(
            f"🎯 [FK Request] current =({current_x:.3f}, {current_y:.3f}, {current_z:.3f}), " 
            f"target x={target_x:.3f}, target y={target_y:.3f}, target z={target_z:.3f}"
        )

        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = target_x
        pose.pose.position.y = target_y
        pose.pose.position.z = target_z

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        
        self.get_logger().info(
            f"🎯 [IK Request] Pos=({dx:.3f}, {dy:.3f}, {dz:.3f}), " 
            f"Pitch={pitch:.3f}, Yaw={yaw:.3f}, Roll(separate)={roll:.3f}"
        )
        
        # -------------------------------------------------------------
        # 2) IK 요청 구성
        # ----------------------------------------------------------initialize---
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.ik_link_name = "end_effector_link"
        req.ik_request.pose_stamped = pose
        req.ik_request.timeout.sec = 2

        # -------------------------------------------------------------
        # 3) IK 호출
        # -------------------------------------------------------------
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        # -------------------------------------------------------------
        # 4) IK 결과 처리
        # -------------------------------------------------------------
        if res and res.error_code.val == 1:  # SUCCESS
            self.get_logger().info("✅ IK computation successful.")

            j = res.solution.joint_state
            name_to_pos = dict(zip(j.name, j.position))

            arm_joint_map = {
                name: name_to_pos[name]
                for name in ARM_JOINTS
                if name in name_to_pos
            }

            if not self.check_joint_limits(arm_joint_map):
                self.get_logger().error("❌ Trajectory aborted due to joint limit violation.")
                return False
            
            # IK 결과에서 joint1~4 사용
            self.current_joint1_pos = name_to_pos.get("joint1", self.current_joint1_pos)
            self.current_joint2_pos = name_to_pos.get("joint2", self.current_joint2_pos)
            self.current_joint3_pos = name_to_pos.get("joint3", self.current_joint3_pos)
            self.current_joint4_pos = name_to_pos.get("joint4", self.current_joint4_pos)

            # 🔑 roll은 joint5에 직접 추가
            # self.current_joint4_pos += pitch
            self.current_joint5_pos += roll

            joint_list = [
                self.current_joint1_pos,
                self.current_joint2_pos,
                self.current_joint3_pos,
                self.current_joint4_pos,
                self.current_joint5_pos,
            ]
            
            self.get_logger().info(
                f"📐 Joint angles: J1={joint_list[0]:.3f}, J2={joint_list[1]:.3f}, "
                f"J3={joint_list[2]:.3f}, J4={joint_list[3]:.3f}, J5={joint_list[4]:.3f}"
            )

            # ---------------------------------------------------------
            # 5) Trajectory 생성 & Publish
            # ---------------------------------------------------------
            traj = JointTrajectory()
            traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

            pt = JointTrajectoryPoint()
            pt.positions = joint_list
            pt.time_from_start.sec = 2
            traj.points.append(pt)

            self.arm_pub.publish(traj)
            
            return True

        else:
            code = res.error_code.val if res else -1
            self.get_logger().error(f"❌ IK computation failed (code: {code})")
            return False

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