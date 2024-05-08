# rbcar_robot
RBCAR robot controllers.

The rbcar_robot repository contains all the package to control the real robot.

## curtis_motordrive
Traction motor controller. It is intended to control the motordrive of the vehicle.
## rbcar_steering_controller
Steering motor controller. It is intended to control the steering position of the vehicle.
## rbcar_controller
Ackermann kinematics controller. It controls and coordinates steering and traction motors. It calculates and publish robot’s odometry.
## rbcar_bringup
It contains several launch files in order to launch some or all the components of the robot.
