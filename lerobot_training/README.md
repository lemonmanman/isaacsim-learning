# Lerobot Training Tips

## Scrip Activating
Use commands below to activate the environment and start:
```bash
conda activate lerobot-ros2
cd ~/lerobot_ros2/examples/IsaacSim_DobotCR5/
unset LD_PRELOAD
source /opt/ros/jazzy/setup.bash
source ~/IsaacSim-ros_workspaces/jazzy_ws/install/setup.bash

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/IsaacSim-ros_workspaces/jazzy_ws/install/isaac_ros2_messages/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/IsaacSim-ros_workspaces/jazzy_ws/install/simulation_interfaces/lib

export PYTHONPATH=$PYTHONPATH:~/IsaacSim-ros_workspaces/jazzy_ws/install/isaac_ros2_messages/lib/python3.12/site-packages
export SIM_PREFIX=$(ros2 pkg prefix simulation_interfaces)
export PYTHONPATH=$PYTHONPATH:$SIM_PREFIX/lib/python3.12/site-packages

# then run your scripts
```
Before you start your isaacsim,use commands below:
```bash
# 1. 系统 ROS 2
source /opt/ros/jazzy/setup.bash
source ~/IsaacSim-ros_workspaces/jazzy_ws/install/setup.bash
echo $AMENT_PREFIX_PATH | grep isaac_ros2_messages
```
Then use commands below to startup:
```bash
./isaac-sim.sh --/isaac/startup/ros_sim_control_extension=True
```

## Data Training
Use commands below to start:
```bash
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True

python scripts/train.py \
  --policy act \
  --batch-size 2 \
  --num-workers 1 \
  --device cuda
```