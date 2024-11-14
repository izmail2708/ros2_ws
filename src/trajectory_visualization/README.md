# Trajectory Visualization package

### Build and source packages
```bash
cd ~/ros2_ws && 
colcon build --symlink-install --packages-select abb_model trajectory_visualization && 
source ~/ros2_ws/install/setup.bash
```


### Launch trajectory visualization (JSP) + robot state publisher (RSP) + RViz2
```bash
ros2 launch trajectory_visualization trajectory_visualization.launch.xml
```
