# Copyright (c) 2021-2024, The RSL-RL Project Developers.
# All rights reserved.
# Original code is licensed under the BSD-3-Clause license.
#
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# Copyright (c) 2025-2026, The Legged Lab Project Developers.
# All rights reserved.
#
# Copyright (c) 2025-2026, The TienKung-Lab Project Developers.
# All rights reserved.
# Modifications are licensed under the BSD-3-Clause license.
#
# This file contains code derived from the RSL-RL, Isaac Lab, and Legged Lab Projects,
# with additional modifications by the TienKung-Lab Project,
# and is distributed under the BSD-3-Clause license.

"""Configuration for Unitree robots.

The following configurations are available:

* :obj:`G1_MINIMAL_CFG`: G1 humanoid robot with minimal collision bodies

Reference: https://github.com/unitreerobotics/unitree_ros
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from legged_lab.assets import ISAAC_ASSET_DIR

ARMATURE_HIP_PITCH = 0.10897986
ARMATURE_HIP_ROLL = 0.070622376
ARMATURE_HIP_YAW = 0.036700268
ARMATURE_KNEE = 0.0812586105
ARMATURE_ANKLE = 0.01536

ARMATURE_WAIST = 0.1123945708

ARMATURE_SHOULDER_PITCH = 0.624
ARMATURE_SHOULDER_ROLL = 0.24
ARMATURE_SHOULDER_YAW = 0.24
ARMATURE_ELBOW = 0.24
ARMATURE_WRIST_ROLL = 0.073
ARMATURE_WRIST_PITCH = 0.073
ARMATURE_WRIST_YAW = 0.073

NATURAL_FREQ_HZ = 6
NATURAL_FREQ = NATURAL_FREQ_HZ * 2.0 * 3.1415926535   # 转换为角频率 (rad/s)
DAMPING_RATIO = 2.0

# 计算刚度 (Stiffness = Armature * ω²)
STIFFNESS_HIP_PITCH = ARMATURE_HIP_PITCH * NATURAL_FREQ**2
STIFFNESS_HIP_ROLL = ARMATURE_HIP_ROLL * NATURAL_FREQ**2
STIFFNESS_HIP_YAW = ARMATURE_HIP_YAW * NATURAL_FREQ**2
STIFFNESS_KNEE = ARMATURE_KNEE * NATURAL_FREQ**2
STIFFNESS_ANKLE = ARMATURE_ANKLE * NATURAL_FREQ**2

STIFFNESS_WAIST = ARMATURE_WAIST * NATURAL_FREQ**2

STIFFNESS_SHOULDER_PITCH = ARMATURE_SHOULDER_PITCH * NATURAL_FREQ**2
STIFFNESS_SHOULDER_ROLL = ARMATURE_SHOULDER_ROLL * NATURAL_FREQ**2
STIFFNESS_SHOULDER_YAW = ARMATURE_SHOULDER_YAW * NATURAL_FREQ**2
STIFFNESS_ELBOW_PITCH = ARMATURE_ELBOW * NATURAL_FREQ**2
STIFFNESS_ELBOW_ROLL = ARMATURE_WRIST_ROLL * NATURAL_FREQ**2
STIFFNESS_WRIST_PITCH = ARMATURE_WRIST_PITCH * NATURAL_FREQ**2
STIFFNESS_WRIST_YAW = ARMATURE_WRIST_YAW * NATURAL_FREQ**2

# 计算阻尼 (Damping = 2 * ζ * Armature * ω)
DAMPING_HIP_PITCH = 2.0 * DAMPING_RATIO * ARMATURE_HIP_PITCH * NATURAL_FREQ
DAMPING_HIP_ROLL = 2.0 * DAMPING_RATIO * ARMATURE_HIP_ROLL * NATURAL_FREQ
DAMPING_HIP_YAW =  2.0 * DAMPING_RATIO * ARMATURE_HIP_YAW * NATURAL_FREQ
DAMPING_KNEE = 2.0 * DAMPING_RATIO * ARMATURE_KNEE * NATURAL_FREQ
DAMPING_ANKLE = 2.0 * DAMPING_RATIO * ARMATURE_ANKLE * NATURAL_FREQ

DAMPING_WAIST = 2.0 * DAMPING_RATIO * ARMATURE_WAIST * NATURAL_FREQ


DAMPING_SHOULDER_PITCH = 2.0 * DAMPING_RATIO * ARMATURE_SHOULDER_PITCH * NATURAL_FREQ
DAMPING_SHOULDER_ROLL = 2.0 * DAMPING_RATIO * ARMATURE_SHOULDER_ROLL * NATURAL_FREQ
DAMPING_SHOULDER_YAW = 2.0 * DAMPING_RATIO * ARMATURE_SHOULDER_YAW * NATURAL_FREQ
DAMPING_ELBOW = 2.0 * DAMPING_RATIO * ARMATURE_ELBOW * NATURAL_FREQ
DAMPING_WRIST_ROLL = 2.0 * DAMPING_RATIO * ARMATURE_WRIST_ROLL * NATURAL_FREQ
DAMPING_WRIST_PITCH = 2.0 * DAMPING_RATIO * ARMATURE_WRIST_PITCH * NATURAL_FREQ
DAMPING_WRIST_YAW = 2.0 * DAMPING_RATIO * ARMATURE_WRIST_YAW * NATURAL_FREQ


ACTION_SCALE_WAIST     = 0.25 * 182.28 / STIFFNESS_WAIST
ACTION_SCALE_HIP_PITCH = 0.25 * 207.76 / STIFFNESS_HIP_PITCH
ACTION_SCALE_HIP_ROLL  = 0.25 * 241.42 / STIFFNESS_HIP_ROLL
ACTION_SCALE_HIP_YAW   = 0.25 * 104.16 / STIFFNESS_HIP_YAW
ACTION_SCALE_KNEE      = 0.25 * 213.80 / STIFFNESS_KNEE 
ACTION_SCALE_ANKLE     = 0.25 * 89.90 / STIFFNESS_ANKLE


ATOM_P3_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        fix_base=False,
        replace_cylinders_with_capsules=True,
        asset_path=f"{ISAAC_ASSET_DIR}/atom_p3/atom.urdf",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=8, solver_velocity_iteration_count=4
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.0),
        joint_pos={
            ".*_hip_pitch_joint": -0.1,
            ".*_hip_roll_joint": 0.0,
            ".*_hip_yaw_joint": 0.0,
            ".*_knee_joint": 0.3,
            ".*_ankle_pitch_joint": -0.2,
            ".*_ankle_roll_joint": 0.0,
            
            "waist_yaw_joint": 0.0,
            
            ".*_shoulder_pitch_joint": 0.0,
            "left_shoulder_roll_joint": 0.3,
            "right_shoulder_roll_joint": -0.3,
            ".*_shoulder_yaw_joint": 0.0,
            ".*_elbow_joint": 1.57,
            ".*_wrist_roll_joint": 0.0,
            ".*_wrist_pitch_joint": 0.0,
            ".*_wrist_yaw_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_pitch_joint",
                ".*_hip_roll_joint",
                ".*_hip_yaw_joint",
                ".*_knee_joint",
            ],
            effort_limit_sim={
                ".*_hip_pitch_joint": 207.76,
                ".*_hip_roll_joint": 241.42,
                ".*_hip_yaw_joint": 104.16,
                ".*_knee_joint": 213.80,
            },
            velocity_limit_sim={
                ".*_hip_pitch_joint": 13.80,
                ".*_hip_roll_joint": 11.90,
                ".*_hip_yaw_joint": 11.36,
                ".*_knee_joint": 16.67,
            },
            stiffness={
                ".*_hip_pitch_joint": STIFFNESS_HIP_PITCH,
                ".*_hip_roll_joint": STIFFNESS_HIP_ROLL,
                ".*_hip_yaw_joint": STIFFNESS_HIP_YAW,
                ".*_knee_joint": STIFFNESS_KNEE,
            },
            damping={
                ".*_hip_pitch_joint": DAMPING_HIP_PITCH,
                ".*_hip_roll_joint": DAMPING_HIP_ROLL,
                ".*_hip_yaw_joint": DAMPING_HIP_YAW,
                ".*_knee_joint": DAMPING_KNEE,
            },
            armature={
                ".*_hip_pitch_joint": ARMATURE_HIP_PITCH,
                ".*_hip_roll_joint": ARMATURE_HIP_ROLL,
                ".*_hip_yaw_joint": ARMATURE_HIP_YAW,
                ".*_knee_joint": ARMATURE_KNEE,
            },
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_ankle_pitch_joint", 
                ".*_ankle_roll_joint",
            ],
            effort_limit_sim={
                ".*_ankle_pitch_joint": 89.90,
                ".*_ankle_roll_joint": 89.90,
            },
            velocity_limit_sim={
                ".*_ankle_pitch_joint": 15.71,
                ".*_ankle_roll_joint": 15.71,
            },
            stiffness=2.0 * STIFFNESS_ANKLE,
            damping=2.0 * DAMPING_ANKLE,
            armature=2.0 * ARMATURE_ANKLE,
        ),
        "waist_yaw": ImplicitActuatorCfg(
            effort_limit_sim=182.28,
            velocity_limit_sim=6.49,
            joint_names_expr=["waist_yaw_joint"],
            stiffness=STIFFNESS_WAIST,
            damping=DAMPING_WAIST,
            armature=ARMATURE_WAIST,
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_pitch_joint",
                ".*_shoulder_roll_joint",
                ".*_shoulder_yaw_joint",
                ".*_elbow_.*",
                ".*_wrist_.*",
            ],
            effort_limit_sim={
                ".*_shoulder_pitch_joint": 93,
                ".*_shoulder_roll_joint": 70,
                ".*_shoulder_yaw_joint": 70, 
                ".*_elbow_.*": 70,
                ".*_wrist_roll_joint": 34,
                ".*_wrist_pitch_joint": 34,
                ".*_wrist_yaw_joint": 34,
            },
            velocity_limit_sim = 6.0,
            stiffness={
                ".*_shoulder_pitch_joint": 120,
                ".*_shoulder_roll_joint": 100,
                ".*_shoulder_yaw_joint": 100, 
                ".*_elbow_.*": 100,
                ".*_wrist_roll_joint": 80,
                ".*_wrist_pitch_joint": 80,
                ".*_wrist_yaw_joint": 80,
            },
            damping={
                ".*_shoulder_pitch_joint": 2,
                ".*_shoulder_roll_joint": 2,
                ".*_shoulder_yaw_joint": 2, 
                ".*_elbow_.*": 2,
                ".*_wrist_roll_joint": 1,
                ".*_wrist_pitch_joint": 1,
                ".*_wrist_yaw_joint": 1,
            },
            armature={
                ".*_shoulder_pitch_joint": ARMATURE_SHOULDER_PITCH,
                ".*_shoulder_roll_joint": ARMATURE_SHOULDER_ROLL,
                ".*_shoulder_yaw_joint": ARMATURE_SHOULDER_YAW,
                ".*_elbow_.*": ARMATURE_ELBOW,
                ".*_wrist_roll_joint": ARMATURE_WRIST_ROLL,
                ".*_wrist_pitch_joint": ARMATURE_WRIST_PITCH,
                ".*_wrist_yaw_joint": ARMATURE_WRIST_YAW,
            }

        ),
    },
)
