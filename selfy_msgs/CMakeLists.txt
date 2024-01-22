cmake_minimum_required(VERSION 2.8.3)
project(selfy_msgs)

add_compile_options(-std=c++14 -O2 -Wall -Wextra)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  geometry_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
	DIRECTORY msg
	FILES
	  Classification.msg
    FailedFusion.msg
    Object.msg 
    ObjectCorrelation.msg 
    ObjectsListing.msg 
    PostFusionObject.msg 
    Sensor.msg
    SensorCorrelation.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp message_runtime std_msgs geometry_msgs
)


install(DIRECTORY tools/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

#install(TARGETS
#            ${PROJECT_NAME}
#            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#            )
