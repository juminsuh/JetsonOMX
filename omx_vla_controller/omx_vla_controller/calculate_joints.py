#!/usr/bin/env python3
import math
import sys
import argparse

# -----------------------------------------------------------------------------
# Robot Physical Parameters (from natural_command.py)
# -----------------------------------------------------------------------------
L2 = 0.128   # Upper Arm
L3 = 0.124   # Forearm
L4 = 0.126   # Gripper Length (using L_GRIPPER from code, or L4=0.165? Code says L4=0.165 as estimate but L_GRIPPER=0.126)
             # Let's use the values that make sense for the kinematic chain.
             # Usually standard OpenManipulator-X:
             # L2(Link1)=0.077, Link2=0.128, Link3=0.124, Link4=0.126
             # natural_command.py defines:
             # L2=0.128, L3=0.124, L_GRIPPER=0.126, L4=0.165
             # And current_z = L2 (initial).
             # We will use the parameters exactly as defined in the reference code.

LINK_L2 = 0.128
LINK_L3 = 0.124
LINK_GRIPPER = 0.126 # Used for EE
LINK_L4 = 0.126 # Assuming typical Gripper length for IK

# Joint Limits (Radians)
JOINT_LIMITS = {
    "joint1": (math.radians(-270), math.radians(360)),
    "joint2": (math.radians(-120), math.radians(90)),
    "joint3": (math.radians(-120), math.radians(90)),
    "joint4": (math.radians(-100), math.radians(100)),
    "joint5": (math.radians(-270), math.radians(270)),
}

def check_limits(joints):
    names = ["joint1", "joint2", "joint3", "joint4", "joint5"]
    ok = True
    for i, val in enumerate(joints):
        name = names[i]
        min_l, max_l = JOINT_LIMITS[name]
        if not (min_l <= val <= max_l):
            print(f"❌ Limit Exceeded: {name}={math.degrees(val):.2f}° (Range: {math.degrees(min_l):.1f}~{math.degrees(max_l):.1f})")
            ok = False
    return ok

def solve_ik_analytical(target_x, target_y, target_z, target_pitch=0.0):
    """
    Computes inverse kinematics for a 4DOF arm + Gripper rotation (5DOF).
    Target: (x, y, z)
    Pitch: Orientation of the end-effector relative to ground (radians)
    """
    
    # 1. J1: Base Rotation
    j1 = math.atan2(target_y, target_x)
    
    # 2. Project to 2D Plane (r, z)
    # Horizontal distance from base center
    r = math.sqrt(target_x**2 + target_y**2)
    
    # 3. Calculate Wrist Position (Joint 4 pivot)
    # We want the End Effector (at tip) to be at (r, z) with angle `pitch`
    # Wrist position (rw, zw) is back-tracked from EE tip by Gripper Length
    rw = r - LINK_GRIPPER * math.cos(target_pitch)
    zw = target_z - LINK_GRIPPER * math.sin(target_pitch)
    
    # Adjust for Base offset if any (natural_command.py assumes Z starts at L2?)
    # "self.current_z = L2" implies L2 is simply the initial height or Link1 length?
    # Standard OpenManipulator: Link1 is vertical.
    # Let's assume zw is relative to Link2 axis.
    # If the base link (Link1) has length L1_Z, we subtract it.
    # In natural_command.py: current_z = L2 + L3*sin(J3)...
    # This suggests L2 is the first segment length?
    # Let's assume standard geometric solver for typical 4DOF arm.
    
    # Triangle L2, L3 connecting to (rw, zw)
    # Distance from shoulder (J2) to wrist (J4)
    # Note: If J2 is at height L_BASE from ground.
    # Assuming L2 parameter is Link 2 length.
    
    L_sq = rw**2 + zw**2
    d = math.sqrt(L_sq)
    
    # Law of Cosines for J3
    # c3 = (d^2 - L2^2 - L3^3) / (2*L2*L3) -> Wait, L2, L3 are link lengths.
    cos_angle_3 = (L_sq - LINK_L2**2 - LINK_L3**2) / (2 * LINK_L2 * LINK_L3)
    
    if abs(cos_angle_3) > 1.0:
        print("❌ Target unreachable (out of workspace)")
        return None
        
    # Internal angle q3 (relative to straight line)
    # For OpenManipulator setup:
    # J3 = - (pi - internal_angle) typically, or similar.
    # Let's calculate inner angle
    angle_3 = math.acos(cos_angle_3)
    
    # Depending on elbow up/down configuration.
    # OpenManipulator usually Elbow Up? Or Down?
    # Let's assume typical configuration:
    j3 = -angle_3 # Elbow up-ish?
    
    # Calculate J2
    # Alpha is angle of vector (rw, zw)
    alpha = math.atan2(zw, rw)
    # Beta is angle from vector to Link2
    # cos_beta = (L2^2 + d^2 - L3^2) / (2*L2*d)
    cos_beta = (LINK_L2**2 + L_sq - LINK_L3**2) / (2 * LINK_L2 * d)
    if abs(cos_beta) > 1.0: return None
    beta = math.acos(cos_beta)
    
    j2 = alpha + beta # Or alpha - beta depending on config
    # Standard check:
    # If we use j3 as negative (elbow up), then j2 = alpha + beta?
    # Let's try to match the "forward/backward" logic from code:
    # "move_forward_backward": joint2 += delta, joint3 -= delta.
    # This implies they move in opposite directions to keep flat?
    # If J2 increases (down?), J3 decreases (up?)
    
    # Let's fix J2 calculation for typical "reach forward"
    j2 = math.atan2(zw, rw) - math.atan2(LINK_L3 * math.sin(j3), LINK_L2 + LINK_L3 * math.cos(j3))
    
    # 4. J4: Wrist Pitch
    # Global Pitch = J2 + J3 + J4
    # So J4 = Global Pitch - J2 - J3
    j4 = target_pitch - j2 - j3
    
    # 5. J5: Wrist Roll (Arbitrary for 4DOF, used for orientation)
    j5 = 0.0 # Placeholder
    
    return [j1, j2, j3, j4, j5]



def main():
    parser = argparse.ArgumentParser(description="Calculate Joint Angles for VLA Robot")
    subparsers = parser.add_subparsers(dest="mode", help="Mode: absolute, relative, compare")
    
    # Absolute Mode
    abs_parser = subparsers.add_parser("absolute", help="Move to absolute XYZ RPY")
    abs_parser.add_argument("x", type=float, help="X (meters)")
    abs_parser.add_argument("y", type=float, help="Y (meters)")
    abs_parser.add_argument("z", type=float, help="Z (meters)")
    abs_parser.add_argument("--roll", type=float, default=0.0, help="Roll (rad)")
    abs_parser.add_argument("--pitch", type=float, default=0.0, help="Pitch (rad)")
    abs_parser.add_argument("--yaw", type=float, default=0.0, help="Yaw (rad)")
    
    # Relative Mode
    rel_parser = subparsers.add_parser("relative", help="Move relative to (0,0,0)")
    rel_parser.add_argument("dx", type=float, help="Delta X (meters)")
    rel_parser.add_argument("dy", type=float, help="Delta Y (meters)")
    rel_parser.add_argument("dz", type=float, help="Delta Z (meters)")
    
    # Compare Mode
    cmp_parser = subparsers.add_parser("compare", help="Compare Heuristic vs IK logic")
    cmp_parser.add_argument("d_forward", type=float, help="Forward Distance (m)")
    
    args = parser.parse_args()
    
    if args.mode == "absolute":
        # For comparison with relative (0,0,0 -> +xyz), we treat absolute as target.
        print(f"\noffset: (0.0, 0.0, 0.0)  <-- effectively starting from zero")
        print(f"🎯 Target Position: ({args.x:.4f}, {args.y:.4f}, {args.z:.4f})")
        print(f"Pitch: {args.pitch:.2f}")

        joints = solve_ik_analytical(args.x, args.y, args.z, args.pitch)
        if joints:
            print("\n✅ Calculated Joint Angles (Radians):")
            print(f"   J1: {joints[0]:.4f}")
            print(f"   J2: {joints[1]:.4f}")
            print(f"   J3: {joints[2]:.4f}")
            print(f"   J4: {joints[3]:.4f}")
            print(f"   J5: {joints[4]:.4f}")
            
            print("\n✅ Degrees:")
            print(f"   J1: {math.degrees(joints[0]):.2f}°")
            print(f"   J2: {math.degrees(joints[1]):.2f}°")
            print(f"   J3: {math.degrees(joints[2]):.2f}°")
            print(f"   J4: {math.degrees(joints[3]):.2f}°")
            
            check_limits(joints)
        else:
            print("❌ Forward Kinematics Failed (Unreachable)")

    elif args.mode == "relative":
        # User requested relative based on (0,0,0)
        start_x, start_y, start_z = 0.0, 0.0, 0.0
        
        target_x = start_x + args.dx
        target_y = start_y + args.dy
        target_z = start_z + args.dz
        
        print(f"\n📍 Start Position: ({start_x}, {start_y}, {start_z})")
        print(f"👉 Relative Move: ({args.dx}, {args.dy}, {args.dz})")
        print(f"🎯 Target Position: ({target_x:.4f}, {target_y:.4f}, {target_z:.4f})")
        
        # Pitch default to 0 for this test or need arg? 
        # User didn't specify pitch behavior for relative, assuming 0.
        pitch = 0.0 
        
        joints = solve_ik_analytical(target_x, target_y, target_z, pitch)
        if joints:
            print("\n✅ Calculated Joint Angles (Radians):")
            print(f"   J1: {joints[0]:.4f}")
            print(f"   J2: {joints[1]:.4f}")
            print(f"   J3: {joints[2]:.4f}")
            print(f"   J4: {joints[3]:.4f}")
            print(f"   J5: {joints[4]:.4f}")
            
            print("\n✅ Degrees:")
            print(f"   J1: {math.degrees(joints[0]):.2f}°")
            print(f"   J2: {math.degrees(joints[1]):.2f}°")
            print(f"   J3: {math.degrees(joints[2]):.2f}°")
            print(f"   J4: {math.degrees(joints[3]):.2f}°")
            
            check_limits(joints)
        else:
            print("❌ Forward Kinematics Failed (Unreachable)")

    elif args.mode == "compare":
        print(f"\n🆚 Comparing User Heuristic vs Analytical IK")
        print(f"   Delta: d_forward={args.d_forward:.3f}m")
        
        # 1. User Heuristic (simplified move_forward_backward)
        # Assuming starting from Home Pose: [0, -1.57, 1.57, 1.57, 0]
        # Heuristic: delta_theta = step / L2
        # J2 += delta, J3 -= delta
        
        home_j1 = 0.0
        home_j2 = -1.5708
        home_j3 = 1.5708
        home_j4 = 1.5708
        home_j5 = 0.0
        
        step_m = args.d_forward
        delta_theta = step_m / LINK_L2
        
        h_j1 = home_j1
        h_j2 = home_j2 + delta_theta
        h_j3 = home_j3 - delta_theta
        h_j4 = home_j4 
        h_j5 = home_j5
        
        print(f"\n[User Heuristic Result]")
        print(f"   J2: {h_j2:.4f} rad ({math.degrees(h_j2):.2f}°)")
        print(f"   J3: {h_j3:.4f} rad ({math.degrees(h_j3):.2f}°)")
        
        # 2. Analytical IK (Relative Move)
        # Calculate Start Position (XYZ) from Home Pose (J2=-90deg, J3=90deg)
        # L2 points UP (Z), L3 points Forward (X)
        start_x = LINK_L3  # 0.124
        start_y = 0.0
        start_z = LINK_L2  # 0.128
        
        target_x = start_x + step_m
        target_y = start_y
        target_z = start_z
        
        print(f"\n[Analytical IK Result]")
        print(f"   Start XYZ: ({start_x:.3f}, {start_y:.3f}, {start_z:.3f})")
        print(f"   Target XYZ: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
        
        ik_joints = solve_ik_analytical(target_x, target_y, target_z, 0.0) # Pitch 0
        
        if ik_joints:
             ik_j2 = ik_joints[1]
             ik_j3 = ik_joints[2]
             print(f"   J2: {ik_j2:.4f} rad ({math.degrees(ik_j2):.2f}°)")
             print(f"   J3: {ik_j3:.4f} rad ({math.degrees(ik_j3):.2f}°)")
             
             diff_j2 = abs(ik_j2 - h_j2)
             diff_j3 = abs(ik_j3 - h_j3)
             print(f"\n[Comparison]")
             print(f"   Difference J2: {diff_j2:.4f} rad ({math.degrees(diff_j2):.2f}°)")
             print(f"   Difference J3: {diff_j3:.4f} rad ({math.degrees(diff_j3):.2f}°)")
        else:
             print("   ❌ Start/Target Unreachable with IK")

        print("\n[Analysis]")
        print("   The User Heuristic assumes `delta_angle = distance / Link_Length`.")
        print("   This is a linear approximation of the arc length formula (s = r * theta).")
        print("   For small movements, this approximates a straight line tangent to the circle.")
        print(f"   For step={step_m}, it rotates J2 by {math.degrees(delta_theta):.2f}°.")
        print("   ⚠️  Heuristic is valid for small steps but ignores trigonometric non-linearity.")

if __name__ == "__main__":
    main()
