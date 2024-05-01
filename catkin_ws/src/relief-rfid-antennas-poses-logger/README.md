Params:
- `robot_type`: `rb1` or `turtlebot`
- `pose_filename`: the absolute path of the directory under which the antennas'
poses are to be logged + "/" + a prefix
- `velocity_filename`: the absolute path of the file in which the velocity of
the robot is to be logged into
- under directory `config`:
  - `reader_X`: where X is incremental
  - `sn`: The reader's MAC address in condensed form (without colons)
  - `active`: Whether the poses of the antennas connected to this reader are
  to be logged
  - `antenna_Y`: where Y in [1,4] the configuration per antenna;
  - `active`: if the pose of antenna Y is to be logged
  - `x`, `y`, `z`: the pose of antenna Y with respect to the robot's footprint


Subscribed topics:
- `/amcl_pose` for the robot's pose
- `velocity_topic` for the robot's velocity
