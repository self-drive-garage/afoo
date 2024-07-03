

colcon build --symlink-install --event-handlers console_cohesion+ --base-paths /home/sameh/ros2_ws --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --symlink-install


ros2 launch afoo afoo.launch.sim.py


ros2 launch teleop_twist_joy teleop-launch.py joy_config:='ps3' publish_stamped_twist:='true'

