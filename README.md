# Extension for IsaacLab

## Installation

- Install Isaac Lab by following the [installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html).
  We recommend using the conda installation as it simplifies calling Python scripts from the terminal.

- Clone or copy this project/repository separately from the Isaac Lab installation (i.e. outside the `IsaacLab` directory):

- Using a python interpreter that has Isaac Lab installed, install the library in editable mode using:

    ```bash
    # use 'PATH_TO_isaaclab.sh|bat -p' instead of 'python' if Isaac Lab is not installed in Python venv or conda
    python -m pip install -e source/rirolab_tasks
    python -m pip install -e source/rirolab_assets

<!-- - Install usd.zip from https://drive.google.com/file/d/1KxNj20RFphDcOyvKhQLSTQ4I3Q2ottjQ/view?usp=sharing. unzip under inrdom_assets/usd folder
    <pre> 📁inrdom_assets
    ├── 📁robots/ 
    └── 📁usd/
    ├── 📁disentangle/
    ├── 📁insert/
    ├── 📁install/
    └── ... / </pre> -->

- Verify that the extension is correctly installed by:

    - Listing the available tasks:

        Shows registered tasks having `"Riro-"` in id
        ```bash
        # use 'FULL_PATH_TO_isaaclab.sh|bat -p' instead of 'python' if Isaac Lab is not installed in Python venv or conda
        python scripts/list_envs.py
        ```

    - Running a task:

        ```bash
        # use 'FULL_PATH_TO_isaaclab.sh|bat -p' instead of 'python' if Isaac Lab is not installed in Python venv or conda
        python scripts/<RL_LIBRARY>/train.py --task=<TASK_NAME>
        
        # example
        python scripts/rsl_rl/train.py --task Riro-Repose-Cube-Shadow-Lite-Direct-v0 --num_envs 128 --headless
        ```

    - Running a task with dummy agents:

        These include dummy agents that output zero or random agents. They are useful to ensure that the environments are configured correctly.

        - Zero-action agent

            ```bash
            # use 'FULL_PATH_TO_isaaclab.sh|bat -p' instead of 'python' if Isaac Lab is not installed in Python venv or conda
            python scripts/zero_agent.py --task=<TASK_NAME>
            ```
        - Random-action agent

            ```bash
            # use 'FULL_PATH_TO_isaaclab.sh|bat -p' instead of 'python' if Isaac Lab is not installed in Python venv or conda
            python scripts/random_agent.py --task=<TASK_NAME>
            ```

### Set up IDE (Optional)

To setup the IDE, please follow these instructions:

- Run VSCode Tasks, by pressing `Ctrl+Shift+P`, selecting `Tasks: Run Task` and running the `setup_python_env` in the drop down menu.
  When running this task, you will be prompted to add the absolute path to your Isaac Sim installation.

- Our vscode debuger find IsaacLab extension for isaacsim/../IsaacLab. Place IsaacLab and isaacsim in the same directory (e.g. ~/IsaacLab, ~/isaacsim). You can edit IsaacLab path in setting.json

If everything executes correctly, it should create a file .python.env in the `.vscode` directory.
The file contains the python paths to all the extensions provided by Isaac Sim and Omniverse.
This helps in indexing all the python modules for intelligent suggestions while writing code.

<!-- ### Setup as Omniverse Extension (Optional)

We provide an example UI extension that will load upon enabling your extension defined in `source/inrdom_sim/inrdom_sim/ui_extension_example.py`.

To enable your extension, follow these steps:

1. **Add the search path of this project/repository** to the extension manager:
    - Navigate to the extension manager using `Window` -> `Extensions`.
    - Click on the **Hamburger Icon**, then go to `Settings`.
    - In the `Extension Search Paths`, enter the absolute path to the `source` directory of this project/repository.
    - If not already present, in the `Extension Search Paths`, enter the path that leads to Isaac Lab's extension directory directory (`IsaacLab/source`)
    - Click on the **Hamburger Icon**, then click `Refresh`.

2. **Search and enable your extension**:
    - Find your extension under the `Third Party` category.
    - Toggle it to enable your extension. -->

<!-- ## Code formatting

We have a pre-commit template to automatically format your code.
To install pre-commit:

```bash
pip install pre-commit
```

Then you can run pre-commit with:

```bash
pre-commit run --all-files
``` -->

## Troubleshooting

### Pylance Missing Indexing of Extensions

In some VsCode versions, the indexing of part of the extensions is missing.
In this case, add the path to your extension in `.vscode/settings.json` under the key `"python.analysis.extraPaths"`.

```json
{
    "python.analysis.extraPaths": [
        "<path-to-ext-repo>/source/riro_tasks",
        "<path-to-ext-repo>/source/riro_assets"
    ]
}
```

### Pylance Crash

If you encounter a crash in `pylance`, it is probable that too many files are indexed and you run out of memory.
A possible solution is to exclude some of omniverse packages that are not used in your project.
To do so, modify `.vscode/settings.json` and comment out packages under the key `"python.analysis.extraPaths"`
Some examples of packages that can likely be excluded are:

```json
"<path-to-isaac-sim>/extscache/omni.anim.*"         // Animation packages
"<path-to-isaac-sim>/extscache/omni.kit.*"          // Kit UI tools
"<path-to-isaac-sim>/extscache/omni.graph.*"        // Graph UI tools
"<path-to-isaac-sim>/extscache/omni.services.*"     // Services tools
...
```


### Senseglove Teleop

- Senseglove teleoperation for demo collect:
    ```bash
    # first terminal
    source /opt/ros/foxy/setup.bash
    source /opt/ros/noetic/setup.bash
    ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
    ```
    ```bash
    # second terminal
    cd ~/senseglove_ws
    source devel/setup.bash
    # if you use shadowhand, change ROS_MASTER_URI
    # export ROS_MASTER_URI=http://server:11311
    roslaunch senseglove_launch senseglove_demo.launch
    ```
    ```bash
    # third terminal
    conda activate env_isaaclab # need to contain geort, rospy, rclpy lib
    cd ~/isaacsim_ws/src/isaacsim_rl
    python scripts/fingertip_to_qpos.py
    ```
    ```bash
    # fourth terminal
    conda activate env_isaaclab
    cd ~/isaacsim_ws/src/isaacsim_rl
    source ~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/setup.bash
    python scripts/fingertip_to_qpos.py
    ```

