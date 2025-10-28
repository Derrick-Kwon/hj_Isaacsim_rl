# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the dexterous hand from Shadow Robot.

The following configurations are available:

* :obj:`SHADOW_HAND_CFG`: Shadow Hand with implicit actuator model.

Reference:

* https://www.shadowrobot.com/dexterous-hand-series/

"""


import isaaclab.sim as sim_utils
from isaaclab.actuators.actuator_cfg import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from rirolab_assets import LOCAL_ASSETS_DIR
##
# Configuration
##
import os

shadowhand_path = os.path.join(LOCAL_ASSETS_DIR, 'robots','shadowhand_lite', 'shadowhand_lite_with_tendon_no_fingertip_driven.usd')

SHADOW_HAND_LITE_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=shadowhand_path,
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            retain_accelerations=True,
            max_depenetration_velocity=1000.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.0005,
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
        # joint_drive_props=sim_utils.JointDrivePropertiesCfg(drive_type="force"),
        joint_drive_props=sim_utils.JointDrivePropertiesCfg(drive_type="force", max_effort=2, damping=0.0, stiffness=0.0),
        fixed_tendons_props=sim_utils.FixedTendonPropertiesCfg(limit_stiffness=30.0, damping=0.1),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.5),
        rot=(0.0, 0.0, -0.7071, 0.7071),
        joint_pos={".*": 0.0},
    ),
    actuators={
        "fingers": ImplicitActuatorCfg(
            joint_names_expr=["rh_(FF|MF|RF)J(4|3|2)", "rh_THJ5", "rh_THJ4", "rh_THJ2","rh_THJ1"],
            effort_limit={
                "rh_(FF|MF|RF)J2": 0.7245,
                "rh_FFJ(4|3)": 0.9,
                "rh_MFJ(4|3)": 0.9,
                "rh_RFJ(4|3)": 0.9,
                "rh_THJ5": 2.3722,
                "rh_THJ4": 1.45,
                "rh_THJ2": 0.99,
                "rh_THJ1": 0.81,
            },
            stiffness={
                "rh_(FF|MF|RF)J(4|3|2)": 1.0,
                "rh_THJ5": 1.0,
                "rh_THJ4": 1.0,
                "rh_THJ2": 1.0,
                "rh_THJ1": 1.0,
            },
            damping={
                "rh_(FF|MF|RF)J(4|3|2)": 0.1,
                "rh_THJ5": 0.1,
                "rh_THJ4": 0.1,
                "rh_THJ2": 0.1,
                "rh_THJ1": 0.1,
            },
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of Shadow Hand robot."""
