

rsync -avz 192.168.1.51:/home/sameh/ros2_ws/src/afoo/src /home/sameh/ros2_ws/src/afoo/ \
 && colcon build --symlink-install --event-handlers console_cohesion+ \
      --base-paths /home/sameh/ros2_ws --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
 && ros2 launch afoo afoo.launch.py


ros2 launch afoo afoo.launch.sim.py


ros2 launch teleop_twist_joy teleop-launch.py joy_config:='ps3' publish_stamped_twist:='true'


ros2 topic pub --rate 4 /afoo_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "
twist:
  linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 2.5"

ros2 topic pub --rate 4 /afoo_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "
twist:
  linear:
    x: 1.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0"

ros2 topic pub --rate 30 /afoo_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "
twist:
  linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0"