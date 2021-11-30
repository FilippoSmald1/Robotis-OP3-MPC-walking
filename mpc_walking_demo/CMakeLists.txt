cmake_minimum_required(VERSION 3.0.2)
project(mpc_walking_demo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
  robotis_controller_msgs
  op3_action_module_msgs
  robotis_framework_common
  robotis_device
  kdl_parser
  cmake_modules
)

find_package(Boost REQUIRED)
FIND_PACKAGE (Eigen3 REQUIRED)
find_package(Eigen REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roslib std_msgs robotis_controller_msgs op3_action_module_msgs robotis_framework_common robotis_device
  DEPENDS system_lib
  DEPENDS Eigen
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

include_directories(blasfeo/include)
include_directories(hpipm/include)
add_subdirectory(blasfeo)
add_subdirectory(hpipm)
file(GLOB srcs "hpipm/include/*.h" "blasfeo/include/*.h")

#include_directories(../test_control_architecture/blasfeo/include)
#include_directories(../test_control_architecture/hpipm/include)
#add_subdirectory(../test_control_architecture/blasfeo blasfeo)
#add_subdirectory(../test_control_architecture/hpipm hpipm)
#file(GLOB srcs "../../test_control_architecture/hpipm/include/*.h" "../../test_control_architecture/blasfeo/include/*.h")


add_library(${PROJECT_NAME} src/mpc_walking_demo.cpp src/MPCSolver.cpp)
#add_executable(mpc_walking_module src/mpc_walking_module.cpp)
add_dependencies(mpc_walking_demo ${simple_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_walking_demo ${catkin_LIBRARIES} blasfeo hpipm)


