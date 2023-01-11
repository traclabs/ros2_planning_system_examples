cmake_minimum_required(VERSION 2.8.3)
project(ros2_planning_system_examples)


add_compile_options(-std=c++17)

set(DEPS
  roscpp
  std_srvs
#  plum_plan_msgs
  behaviortree_cpp
  plansys2_domain_expert
  plansys2_problem_expert
  plansys2_executor
  plansys2_planner
  plansys2_msgs
  )

find_package(
  catkin REQUIRED COMPONENTS
  ${DEPS}
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES #
  CATKIN_DEPENDS
  ${DEPS}
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## BT Simple Example

add_library(bt_simple_example_recharge_node SHARED src/simple_bt_example/Recharge.cpp)
list(APPEND plugin_libs bt_simple_example_recharge_node)

add_library(bt_simple_example_move_node SHARED src/simple_bt_example/Move.cpp)
list(APPEND plugin_libs bt_simple_example_move_node)

add_library(bt_simple_example_close_gripper_node SHARED src/simple_bt_example/CloseGripper.cpp)
list(APPEND plugin_libs bt_simple_example_close_gripper_node)

add_library(bt_simple_example_open_gripper_node SHARED src/simple_bt_example/OpenGripper.cpp)
list(APPEND plugin_libs bt_simple_example_open_gripper_node)

add_library(bt_simple_example_approach_object_node SHARED src/simple_bt_example/ApproachObject.cpp)
list(APPEND plugin_libs bt_simple_example_approach_object_node)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(assembly_action_node src/simple_bt_example/assembly_action_node.cpp)
target_link_libraries(assembly_action_node ${catkin_LIBRARIES})

### Controller node for bt_simple_example ###
add_executable(simple_bt_example_controller_node src/simple_bt_example/simple_bt_example_controller_node.cpp)
target_link_libraries(simple_bt_example_controller_node ${catkin_LIBRARIES})

#add_executable(move_to_inspect_action_node src/tests/move_to_inspect/move_to_inspect_action_node.cpp)
#target_link_libraries(move_to_inspect_action_node ${catkin_LIBRARIES} )



install(TARGETS 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


