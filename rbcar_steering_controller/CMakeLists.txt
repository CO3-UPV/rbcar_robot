cmake_minimum_required(VERSION 2.8.3)
project(rbcar_steering_controller)


find_package(catkin REQUIRED COMPONENTS
  robotnik_msgs
  std_msgs
  roscpp 
  message_generation
)



## Generate messages in the 'msg' folder
add_message_files(
   FILES
   EpcEncoderStatus.msg
   SteeringControllerStatus.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   SetMotorStatus.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   robotnik_msgs#   std_msgs
)


catkin_package(

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



## Declare a cpp executable
# add_executable(rbcar_steering_controller_node src/rbcar_steering_controller_node.cpp)
add_executable(rbcar_steering_controller src/Component.cc src/PCan.cc src/Dzcante020l080.cc src/MotorDrive.cc  src/MotorPosition.cc src/EpcMaCanOpen.cc src/EpcEncoder.cc src/rbcar_steering_controller.cpp)


add_dependencies(rbcar_steering_controller robotnik_msgs_generate_messages_cpp rbcar_steering_controller_generate_messages_cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(rbcar_steering_controller
   pcan
   ${catkin_LIBRARIES}
)


