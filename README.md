# MoveIt! Sample of the Panda Robot

The repository to manipulate Panda robot powered by MoveIt!.

### Topics

* **/hiro/panda/error_recover**: recover the error mode
    * usage: `rostopic pub -1 /hiro/panda/error_recover std_msgs/Empty "{}"`

* **/hiro/panda/gripper_cmd**: make the gripper close/open
    * open: `rostopic pub -1 /hiro/panda/gripper_cmd std_msgs/String "data: 'open'"`
    * close: `rostopic pub -1 /hiro/panda/gripper_cmd std_msgs/String "data: 'open'"`

* **/hiro/panda/target**: move the robot to target position in Cartesian space
    * example: `rostopic pub -1 /hiro/panda/target geometry_msgs/Pose "{position:{ x: -0.48, y: -0.16, z: 0.40}, orientation:{ x: 1.0, y: 0.0, z: 0.0, w: 0.0}}"`

* **/hiro/panda/vel_cmd**: Send Cartesian Velocity command to the robot
    * example: `rostopic pub -1 /hiro/panda/cmd_vel     geometry_msgs/Twist "{linear:{ x:0.0, y: 0.0, z: 0.01}, angular:{ x: 0.0, y: 0.0, z: 0.0}}"`
