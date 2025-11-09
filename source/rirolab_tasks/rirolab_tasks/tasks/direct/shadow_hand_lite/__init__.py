# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Shadow Hand environment.
"""

import gymnasium as gym

from . import agents

##
# Register Gym environments.
##

inhand_task_entry = "rirolab_tasks.tasks.direct.inhand_manipulation"

gym.register(
    id="Riro-Repose-Cube-Shadow-Lite-Direct-v0",
    entry_point=f"{inhand_task_entry}.inhand_manipulation_env:InHandManipulationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.shadow_hand_lite_env_cfg:ShadowHandLiteEnvCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:ShadowHandLitePPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)

###############################################################################################333

#Register Lemon Environment
gym.register(
    id="Riro-Repose-Cube-Shadow-Lite-Direct-Lemon-v0",
    entry_point=f"{inhand_task_entry}.inhand_manipulation_env:InHandManipulationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.shadow_hand_lite_env_cfg:ShadowHandLiteEnvLemonCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:ShadowHandLitePPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)

#Register Mug Environment
gym.register(
    id="Riro-Repose-Cube-Shadow-Lite-Direct-Mug-v0",
    entry_point=f"{inhand_task_entry}.inhand_manipulation_env:InHandManipulationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.shadow_hand_lite_env_cfg:ShadowHandLiteEnvMugCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:ShadowHandLitePPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)


#Register Glue Environment
gym.register(
    id="Riro-Repose-Cube-Shadow-Lite-Direct-Glue-v0",
    entry_point=f"{inhand_task_entry}.inhand_manipulation_env:InHandManipulationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.shadow_hand_lite_env_cfg:ShadowHandLiteEnvGlueCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:ShadowHandLitePPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)

#Register Pillcase Environment
gym.register(
    id="Riro-Repose-Cube-Shadow-Lite-Direct-Pillcase-v0",
    entry_point=f"{inhand_task_entry}.inhand_manipulation_env:InHandManipulationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.shadow_hand_lite_env_cfg:ShadowHandLiteEnvPillcaseCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:ShadowHandLitePPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)
#Register Redblock Environment
gym.register(
    id="Riro-Repose-Cube-Shadow-Lite-Direct-Redblock-v0",
    entry_point=f"{inhand_task_entry}.inhand_manipulation_env:InHandManipulationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.shadow_hand_lite_env_cfg:ShadowHandLiteEnvRedblockCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:ShadowHandLitePPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)
#Register Spam Environment
gym.register(
    id="Riro-Repose-Cube-Shadow-Lite-Direct-Spam-v0",
    entry_point=f"{inhand_task_entry}.inhand_manipulation_env:InHandManipulationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.shadow_hand_lite_env_cfg:ShadowHandLiteEnvSpamCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:ShadowHandLitePPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)
#Register Bowl Environment
gym.register(
    id="Riro-Repose-Cube-Shadow-Lite-Direct-Bowl-v0",
    entry_point=f"{inhand_task_entry}.inhand_manipulation_env:InHandManipulationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.shadow_hand_lite_env_cfg:ShadowHandLiteEnvBowlCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:ShadowHandLitePPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)
#Register Pitcher Environment
gym.register(
    id="Riro-Repose-Cube-Shadow-Lite-Direct-Pitcher-v0",
    entry_point=f"{inhand_task_entry}.inhand_manipulation_env:InHandManipulationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.shadow_hand_lite_env_cfg:ShadowHandLiteEnvPitcherCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:ShadowHandLitePPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)
#Register Teapot Environment
gym.register(
    id="Riro-Repose-Cube-Shadow-Lite-Direct-Teapot-v0",
    entry_point=f"{inhand_task_entry}.inhand_manipulation_env:InHandManipulationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.shadow_hand_lite_env_cfg:ShadowHandLiteEnvTeapotCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:ShadowHandLitePPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)

#Register Bulb Environment
gym.register(
    id="Riro-Repose-Cube-Shadow-Lite-Direct-Bulb-v0",
    entry_point=f"{inhand_task_entry}.inhand_manipulation_env:InHandManipulationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.shadow_hand_lite_env_cfg:ShadowHandLiteEnvBulbCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:ShadowHandLitePPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)


#Register Teddybear Environment
gym.register(
    id="Riro-Repose-Cube-Shadow-Lite-Direct-Teddybear-v0",
    entry_point=f"{inhand_task_entry}.inhand_manipulation_deform_env:InHandManipulationDeformEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.shadow_hand_lite_env_cfg:ShadowHandLiteEnvTeddybearCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:ShadowHandLitePPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)
###############################################################################################333

gym.register(
    id="Riro-Repose-Cube-Shadow-Lite-OpenAI-FF-Direct-v0",
    entry_point=f"{inhand_task_entry}.inhand_manipulation_env:InHandManipulationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.shadow_hand_lite_env_cfg:ShadowHandLiteOpenAIEnvCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_ff_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:ShadowHandLiteAsymFFPPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ff_ppo_cfg.yaml",
    },
)

# gym.register(
#     id="Riro-Repose-Cube-Shadow-Lite-OpenAI-LSTM-Direct-v0",
#     entry_point=f"{inhand_task_entry}.inhand_manipulation_env:InHandManipulationEnv",
#     disable_env_checker=True,
#     kwargs={
#         "env_cfg_entry_point": f"{__name__}.shadow_hand_lite_env_cfg:ShadowHandLiteOpenAIEnvCfg",
#         "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_lstm_cfg.yaml",
#     },
# )

# ### Vision

gym.register(
    id="Riro-Repose-Cube-Shadow-Lite-Vision-Direct-v0",
    entry_point=f"{__name__}.shadow_hand_lite_vision_env:ShadowHandLiteVisionEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.shadow_hand_lite_vision_env:ShadowHandLiteVisionEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:ShadowHandLiteVisionFFPPORunnerCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_vision_cfg.yaml",
    },
)



# gym.register(
#     id="Riro-Repose-Cube-Shadow-Lite-Vision-Direct-Play-v0",
#     entry_point=f"{__name__}.shadow_hand_lite_vision_env:ShadowHandLiteVisionEnv",
#     disable_env_checker=True,
#     kwargs={ri
#         "env_cfg_entry_point": f"{__name__}.shadow_hand_lite_vision_env:ShadowHandLiteVisionEnvPlayCfg",
#         "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:ShadowHandLiteVisionFFPPORunnerCfg",
#         "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_vision_cfg.yaml",
#     },
# )
