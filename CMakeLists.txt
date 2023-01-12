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
  tf
  ff_msgs
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

######################################
## BT Simple Example
######################################
# Actions
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

add_executable(assembly_action_node src/simple_bt_example/assembly_action_node.cpp)
target_link_libraries(assembly_action_node ${catkin_LIBRARIES})

# Controller node for bt_simple_example
# (just for easy firing up of a problem and its subsequent execution. You could do this manually)
add_executable(simple_bt_example_controller_node src/simple_bt_example/simple_bt_example_controller_node.cpp)
target_link_libraries(simple_bt_example_controller_node ${catkin_LIBRARIES})


#########################################
## Fake Astrobee Examples
#########################################

# Action for move_inspect
add_executable(fake_astrobee_move_to_inspect_action_node src/fake_astrobee_examples/fake_astrobee_move_to_inspect_action_node.cpp)
target_link_libraries(fake_astrobee_move_to_inspect_action_node ${catkin_LIBRARIES} )

# Control node for astrobee examples
add_executable(astrobee_move_to_inspect_control_node src/fake_astrobee_examples/astrobee_move_to_inspect_control_node.cpp)
target_link_libraries(astrobee_move_to_inspect_control_node ${catkin_LIBRARIES} )

#########################################
## Astrobee Examples
#########################################

add_library(bt_astrobee_navigate_node SHARED src/astrobee_examples/navigate_node.cpp)
list(APPEND plugin_libs bt_astrobee_navigate_node)

# Debug node for astrobee examples
add_executable(debug_go_to_waypoint src/astrobee_examples/debug_go_to_waypoint.cpp)
target_link_libraries(debug_go_to_waypoint ${catkin_LIBRARIES} )

#########################################
# For all BT plugins
#########################################
foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()




install(TARGETS 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )



