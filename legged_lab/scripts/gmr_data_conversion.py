import pickle
import numpy as np
import torch
import argparse
from scipy.spatial.transform import Rotation


def quat_conjugate(q):
    """Compute quaternion conjugate.
    
    Args:
        q: Quaternion tensor of shape (..., 4) in (w, x, y, z) format
    
    Returns:
        Conjugate quaternion of same shape
    """
    q_conj = q.clone()
    q_conj[..., 1:] *= -1
    return q_conj


def quat_mul(q1, q2):
    """Multiply two quaternions.
    
    Args:
        q1: First quaternion tensor of shape (..., 4) in (w, x, y, z) format
        q2: Second quaternion tensor of shape (..., 4) in (w, x, y, z) format
    
    Returns:
        Product quaternion of same shape
    """
    w1, x1, y1, z1 = q1[..., 0], q1[..., 1], q1[..., 2], q1[..., 3]
    w2, x2, y2, z2 = q2[..., 0], q2[..., 1], q2[..., 2], q2[..., 3]
    
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    
    return torch.stack([w, x, y, z], dim=-1)


def axis_angle_from_quat(q):
    """Convert quaternion to axis-angle representation.
    
    Args:
        q: Quaternion tensor of shape (..., 4) in (w, x, y, z) format
    
    Returns:
        Axis-angle tensor of shape (..., 3)
    """
    # Normalize quaternion
    q_norm = q / torch.norm(q, dim=-1, keepdim=True)
    
    w = q_norm[..., 0]
    xyz = q_norm[..., 1:]
    
    # Compute angle
    w_clamped = torch.clamp(w, -1.0, 1.0)
    angle = 2 * torch.acos(w_clamped)
    
    # Compute axis
    sin_half_angle = torch.sqrt(torch.clamp(1 - w_clamped * w_clamped, min=1e-8))
    
    # Handle near-zero rotation case (when sin_half_angle is very small)
    small_angle_mask = sin_half_angle < 1e-6
    sin_half_angle = torch.where(small_angle_mask, torch.ones_like(sin_half_angle), sin_half_angle)
    
    axis = xyz / sin_half_angle.unsqueeze(-1)
    axis_angle = axis * angle.unsqueeze(-1)
    
    # Set to zero for near-zero rotations
    axis_angle[small_angle_mask] = 0.0
    
    return axis_angle


def convert_pkl_to_custom(input_pkl, output_txt, fps):
    dt = 1.0 / fps

    with open(input_pkl, "rb") as f:
        motion_data = pickle.load(f)

    root_pos = motion_data["root_pos"]
    root_rot = motion_data["root_rot"][:, [3, 0, 1, 2]]  # xyzw → wxyz
    dof_pos = motion_data["dof_pos"]

    root_lin_vel = (root_pos[1:] - root_pos[:-1]) / dt
    root_rot_t = torch.tensor(root_rot, dtype=torch.float32)

    print(f"[GMRDataConversion] root_rot_t[0] = {root_rot_t[0]}")
    print(f"[GMRDataConversion] root_rot_t[1] = {root_rot_t[1]}")
    print(f"[GMRDataConversion] root_rot_t[2] = {root_rot_t[2]}")

    q1_conj = quat_conjugate(root_rot_t[:-1])         
    print(f"[GMRDataConversion] q1_conj[0] = {q1_conj[0]}")
    print(f"[GMRDataConversion] q1_conj[1] = {q1_conj[1]}")
    print(f"[GMRDataConversion] q1_conj[2] = {q1_conj[2]}")
    dq = quat_mul(q1_conj, root_rot_t[1:])            
    print(f"[GMRDataConversion] dq[0] = {dq[0]}")
    print(f"[GMRDataConversion] dq[1] = {dq[1]}")
    print(f"[GMRDataConversion] dq[2] = {dq[2]}")
    axis_angle = axis_angle_from_quat(dq)             
    print(f"[GMRDataConversion] axis_angle[0] = {axis_angle[0]}")
    print(f"[GMRDataConversion] axis_angle[1] = {axis_angle[1]}")
    print(f"[GMRDataConversion] axis_angle[2] = {axis_angle[2]}")
    root_ang_vel = (axis_angle / dt).numpy()

    print(f"[GMRDataConversion] root_ang_vel[0] = {root_ang_vel[0]}")
    print(f"[GMRDataConversion] root_ang_vel[1] = {root_ang_vel[1]}")
    print(f"[GMRDataConversion] root_ang_vel[2] = {root_ang_vel[2]}")
    input("Press Enter to continue...")

    dof_vel = (dof_pos[1:] - dof_pos[:-1]) / dt

    euler_angles = Rotation.from_quat(root_rot[:-1, [1, 2, 3, 0]]).as_euler('XYZ', degrees=False)
    euler_angles = np.unwrap(euler_angles, axis=0)

    data_output = np.concatenate(
        (root_pos[:-1], euler_angles, dof_pos[:-1],  
         root_lin_vel, root_ang_vel, dof_vel),
        axis=1
    )

    np.savetxt(output_txt, data_output, fmt='%f', delimiter=', ')
    with open(output_txt, 'r') as f:
        frames_data = f.readlines()

    frames_data_len = len(frames_data)
    with open(output_txt, 'w') as f:
        f.write('{\n')
        f.write('"LoopMode": "Wrap",\n')
        f.write(f'"FrameDuration": {1.0/fps:.3f},\n')
        f.write('"EnableCycleOffsetPosition": true,\n')
        f.write('"EnableCycleOffsetRotation": true,\n')
        f.write('"MotionWeight": 0.5,\n\n')
        f.write('"Frames":\n[\n')

        for i, line in enumerate(frames_data):
            line_start_str = '  ['
            if i == frames_data_len - 1:
                f.write(line_start_str + line.rstrip() + ']\n')
            else:
                f.write(line_start_str + line.rstrip() + '],\n')

        f.write(']\n}')
    print(f"✅ Successfully converted {input_pkl} to {output_txt}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_pkl", type=str, required=True)
    parser.add_argument("--output_txt", type=str, required=True)
    parser.add_argument("--fps", type=float, default=30.0)
    args = parser.parse_args()

    convert_pkl_to_custom(args.input_pkl, args.output_txt, args.fps)
