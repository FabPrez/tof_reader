Create a ROS workspace (if you already have one, you can skip this step):
```
mkdir -p ~/projects/merlin_ws/src
cd ~/projects/merlin_ws
catkin init
```
Then:
```
cd ~/projects/merlin_ws/src <--[use your workspace name] 
git clone https://github.com/FabPrez/tof_reader.git
cd ~/projects/merlin_ws
catkin build
```
