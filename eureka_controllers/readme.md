# Eureka_controllers

## Structure

Eureka_controllers package contains of several quasi independant modules. For now it's:

- eureka_ackermann_controller
- eureka_control_msgs
- eureka_steering_library

### eureka_ackermann_controller

eureka_ackermann_controller is a frontend which inherited from eureka_steering_library. It providing high level function to setup the controller.

### eureka_control_msgs

eureka_control_msgs is a package that provide description for custom message that used for debugging.

### eureka_steering_library

eureka_steering_library is one of the most complex part of the eureka_controller package. It provide function to calculate odometry
in several manner (open loop, directly from commands on dc motors).
