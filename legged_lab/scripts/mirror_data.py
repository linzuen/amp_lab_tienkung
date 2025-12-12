import json
import numpy as np
import os
import sys

# 66 = 3 + 3 + 27 + 3 + 3 + 27
N_ROOT_POS = 3
N_EULER = 3
N_DOF = 27
N_ROOT_LIN_VEL = 3
N_ROOT_ANG_VEL = 3
N_DOF_VEL = 27

# --------------------------------------------
# 定义 DOF index 映射 (pos 和 vel 一样)
# --------------------------------------------
idx = {}
idx["L_leg"] = list(range(0, 6))
idx["R_leg"] = list(range(6, 12))
idx["Waist"] = [12]
idx["L_arm"] = list(range(13, 20))
idx["R_arm"] = list(range(20, 27))


# --------------------------------------------
# 镜像 DOF：左右交换 + 关节符号反转
# --------------------------------------------
def mirror_dofs(dof):
    mirrored = np.zeros_like(dof)

    # 左右互换
    mirrored[idx["L_leg"]] = dof[idx["R_leg"]]
    mirrored[idx["R_leg"]] = dof[idx["L_leg"]]
    mirrored[idx["L_arm"]] = dof[idx["R_arm"]]
    mirrored[idx["R_arm"]] = dof[idx["L_arm"]]

    # 腰 Yaw 反号
    mirrored[idx["Waist"]] = -dof[idx["Waist"]]

    # roll / yaw 取反
    roll_yaw_indices = [
        1, 2, 5,      # 左腿
        7, 8, 11      # 右腿
    ]
    arm_roll_yaw = [1, 2, 4, 6]

    for base in [idx["L_arm"][0], idx["R_arm"][0]]:
        roll_yaw_indices.extend([base + i for i in arm_roll_yaw])

    mirrored[roll_yaw_indices] *= -1
    return mirrored


# --------------------------------------------
# Root 镜像
# --------------------------------------------
def mirror_root_pos(xyz):
    x, y, z = xyz
    return np.array([x, -y, z])


def mirror_euler(rpy):
    roll, pitch, yaw = rpy
    return np.array([-roll, pitch, -yaw])


def mirror_vec_y(vec3):
    return np.array([vec3[0], -vec3[1], vec3[2]])


# --------------------------------------------
# 主处理函数
# --------------------------------------------
def mirror_motion(data):
    frames = data["Frames"]
    mirrored_frames = []

    for f in frames:
        f = np.array(f)

        root_pos = f[0:3]
        euler = f[3:6]
        dof_pos = f[6:6 + 27]
        root_lin = f[33:36]
        root_ang = f[36:39]
        dof_vel = f[39:66]

        f_new = np.concatenate([
            mirror_root_pos(root_pos),
            mirror_euler(euler),
            mirror_dofs(dof_pos),
            mirror_vec_y(root_lin),
            mirror_vec_y(root_ang),
            mirror_dofs(dof_vel)
        ])

        mirrored_frames.append(f_new.tolist())

    data["Frames"] = mirrored_frames
    return data


# --------------------------------------------
# 主入口：读取参数输入路径
# --------------------------------------------
if __name__ == "__main__":

    if len(sys.argv) != 2:
        print("使用方法:")
        print("  python mirror_motion.py <输入txt/json文件路径>")
        print("示例:")
        print("  python mirror_motion.py /home/lze/motions/run_left.txt")
        sys.exit(1)

    input_path = sys.argv[1]

    # 读取 JSON txt
    with open(input_path) as f:
        data = json.load(f)

    # 处理
    mirrored = mirror_motion(data)

    # 生成输出路径
    input_dir = os.path.dirname(input_path)
    input_name = os.path.basename(input_path)
    name_no_ext = os.path.splitext(input_name)[0]

    output_path = os.path.join(input_dir, f"{name_no_ext}_mirrored.txt")

    # 写文件
    with open(output_path, "w") as f:
        json.dump(mirrored, f, indent=2)

    print(f"镜像完成 → {output_path}")
