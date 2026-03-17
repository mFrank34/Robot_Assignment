# ROS2 Humble Commands Reference

## Workspace Setup

### Create a workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### Clone packages
```bash
cd ~/ros2_ws/src
git clone <repository_url>
```

### Build workspace
```bash
cd ~/ros2_ws
colcon build
```

### Build specific package
```bash
colcon build --packages-select <package_name>
```

### Build with symlink install (for development)
```bash
colcon build --symlink-install
```

### Source workspace
```bash
source install/setup.bash
```

### Source with overlay
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Node Management

### Run a node
```bash
ros2 run <package_name> <executable_name>
```

### Run with arguments
```bash
ros2 run <package_name> <executable_name> --ros-args -p param_name:=value
```

### Run with remapping
```bash
ros2 run <package_name> <executable_name> --ros-args --remap old_topic_name:=new_topic_name
```

### Launch a launch file
```bash
ros2 launch <package_name> <launch_file.py>
```

### List running nodes
```bash
ros2 node list
```

### Get node info
```bash
ros2 node info <node_name>
```

## Topic Management

### List topics
```bash
ros2 topic list
```

### Echo topic data
```bash
ros2 topic echo <topic_name>
```

### Echo with type info
```bash
ros2 topic echo <topic_name> --include-lost-messages
```

### Publish to topic
```bash
ros2 topic pub <topic_name> <msg_type> '<args>'
```

### Publish at rate
```bash
ros2 topic pub --rate 10 <topic_name> <msg_type> '<args>'
```

### Get topic info
```bash
ros2 topic info <topic_name>
```

### Get topic type
```bash
ros2 topic type <topic_name>
```

### Check topic bandwidth
```bash
ros2 topic bw <topic_name>
```

### Check topic frequency
```bash
ros2 topic hz <topic_name>
```

## Service Management

### List services
```bash
ros2 service list
```

### Call a service
```bash
ros2 service call <service_name> <service_type> '<args>'
```

### Get service type
```bash
ros2 service type <service_name>
```

### Find service by type
```bash
ros2 service find <service_type>
```

## Parameter Management

### List parameters
```bash
ros2 param list
```

### Get parameter
```bash
ros2 param get <node_name> <param_name>
```

### Set parameter
```bash
ros2 param set <node_name> <param_name> <value>
```

### Load parameters from file
```bash
ros2 param load <node_name> <param_file.yaml>
```

### Dump parameters to file
```bash
ros2 param dump <node_name> > params.yaml
```

## Message & Service Inspection

### Show message definition
```bash
ros2 interface show <message_type>
```

### Show service definition
```bash
ros2 interface show <service_type>
```

### List message types
```bash
ros2 interface list -m
```

### List service types
```bash
ros2 interface list -s
```

## Visualization & Debugging Tools

### Launch RViz2
```bash
ros2 run rviz2 rviz2
```

### Launch RViz2 with config
```bash
ros2 run rviz2 rviz2 -d <config_file.rviz>
```

### Launch rqt_gui
```bash
ros2 run rqt_gui rqt_gui
```

### Launch rqt_graph
```bash
ros2 run rqt_graph rqt_graph
```

### Launch tf2_tools
```bash
ros2 run tf2_tools view_frames
```

### View TF tree
```bash
ros2 run tf2_tools view_frames.py
```

### Check TF transform
```bash
ros2 run tf2_ros tf2_echo <source_frame> <target_frame>
```

### Check TF static transform
```bash
ros2 service call /tf2_echo tf2_msgs/TFMessage <args>
```

## Package Management

### Create a package (Python)
```bash
ros2 pkg create --build-type ament_python <package_name>
```

### Create a package (C++)
```bash
ros2 pkg create --build-type ament_cmake <package_name>
```

### Create with dependencies
```bash
ros2 pkg create --build-type ament_python <package_name> --dependencies <dep1> <dep2>
```

### List all packages
```bash
ros2 pkg list
```

### Find package prefix
```bash
ros2 pkg prefix <package_name>
```

### Get package executables
```bash
ros2 pkg executables <package_name>
```

## Build & Dependency Management

### Check dependencies
```bash
rosdep check --all-platform-sources --rosdistro humble
```

### Install dependencies
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### List build types
```bash
ament_cmake --help
ament_python --help
```

### Clean build
```bash
rm -rf build/ install/ log/
colcon build
```

### Build only test
```bash
colcon build --cmake-args -DBUILD_TESTING=ON
```

### Run tests
```bash
colcon test
```

## Logging & System Info

### List logs
```bash
ros2 topic echo /rosout
```

### Get ROS2 version
```bash
ros2 --version
```

### Check environment
```bash
printenv | grep ROS
```

### Check ROS domain ID
```bash
echo $ROS_DOMAIN_ID
```

### Set ROS domain ID
```bash
export ROS_DOMAIN_ID=<id>
```

## Multi-Machine Communication

### Check localhost only
```bash
export ROS_LOCALHOST_ONLY=1
```

### Allow network communication
```bash
unset ROS_LOCALHOST_ONLY
```

### Set RMW implementation
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## Bag Files (Recording & Playback)

### Record topics
```bash
ros2 bag record <topic1> <topic2> ...
```

### Record all topics
```bash
ros2 bag record -a
```

### Playback bag
```bash
ros2 bag play <bag_file>
```

### List bag info
```bash
ros2 bag info <bag_file>
```

## Security

### Generate keys
```bash
ros2 security generate-keys
```

### Generate artifacts
```bash
ros2 security generate-artifacts
```

## Miscellaneous

### Daemon management
```bash
ros2 daemon start
ros2 daemon stop
ros2 daemon status
```

### Clear daemon cache
```bash
ros2 daemon kill
```

### Check connectivity
```bash
ros2 test_communication pubsub
```

### Lifecycle node transitions
```bash
ros2 lifecycle list <node_name>
ros2 lifecycle set <node_name> configure
ros2 lifecycle set <node_name> activate
```

### Quality of Service (QoS) info
```bash
ros2 topic info <topic_name> --verbose
```

## Common Workflow Examples

### Start minimal system
```bash
# Terminal 1: Source workspace
source ~/ros2_ws/install/setup.bash

# Terminal 2: Start ROS2 daemon
ros2 daemon start

# Terminal 3: Launch your application
ros2 launch <package_name> <launch_file.py>
```

### Debug with multiple terminals
```bash
# Terminal 1: Run main node
ros2 run <package> <node> --ros-args --log-level debug

# Terminal 2: Monitor topics
ros2 topic echo <topic_name>

# Terminal 3: View graph
ros2 run rqt_graph rqt_graph

# Terminal 4: Visualize
ros2 run rviz2 rviz2
```

### Typical development cycle
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select <your_package>
source install/setup.bash
ros2 run <package_name> <executable_name>
```
