// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <plansys2_pddl_parser/Utils.h>

#include <memory>

#include <plansys2_msgs/ActionExecutionInfo.h>
#include <plansys2_msgs/Plan.h>

#include <plansys2_domain_expert/DomainExpertClient.hpp>
#include <plansys2_executor/ExecutorClient.hpp>
#include <plansys2_planner/PlannerClient.hpp>
#include <plansys2_problem_expert/ProblemExpertClient.hpp>

#include <ros/ros.h>
//#include "rclcpp_action/rclcpp_action.hpp"

class Assemble 
{
public:
  Assemble()
  {
  }

  bool init()
  { 
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge();
    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);
    
    if (!plan.has_value()) {
      std::cout << "Could not find plan to reach goal " <<
        parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
      return false;
    }
    
    ROS_WARN_STREAM("Found plan!: \n");
    for(auto item : plan.value().items)
      ROS_WARN("\t * Time: %f action: %s duration: %f", item.time, item.action.c_str(), item.duration);

    if (!executor_client_->start_plan_execution(plan.value())) {
      ROS_ERROR("Error starting a new plan (first)");
    }

    return true;
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"r2d2", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"car_1", "car"});
    problem_expert_->addInstance(plansys2::Instance{"car_2", "car"});
    problem_expert_->addInstance(plansys2::Instance{"car_3", "car"});

    problem_expert_->addInstance(plansys2::Instance{"wheels_zone", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"steering_wheels_zone", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"body_car_zone", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"assembly_zone", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"recharge_zone", "zone"});


    problem_expert_->addInstance(plansys2::Instance{"wheel_1", "piece"});
    problem_expert_->addInstance(plansys2::Instance{"wheel_2", "piece"});
    problem_expert_->addInstance(plansys2::Instance{"wheel_3", "piece"});

    problem_expert_->addInstance(plansys2::Instance{"body_car_1", "piece"});
    problem_expert_->addInstance(plansys2::Instance{"body_car_2", "piece"});
    problem_expert_->addInstance(plansys2::Instance{"body_car_3", "piece"});

    problem_expert_->addInstance(plansys2::Instance{"steering_wheel_1", "piece"});
    problem_expert_->addInstance(plansys2::Instance{"steering_wheel_2", "piece"});
    problem_expert_->addInstance(plansys2::Instance{"steering_wheel_3", "piece"});

    problem_expert_->addPredicate(plansys2::Predicate("(is_assembly_zone assembly_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(is_recharge_zone recharge_zone)"));

    problem_expert_->addPredicate(plansys2::Predicate("(piece_is_wheel wheel_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_is_wheel wheel_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_is_wheel wheel_3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_at wheel_1 wheels_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_at wheel_2 wheels_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_at wheel_3 wheels_zone)"));

    problem_expert_->addPredicate(plansys2::Predicate("(piece_is_body_car body_car_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_is_body_car body_car_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_is_body_car body_car_3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_at body_car_1 body_car_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_at body_car_2 body_car_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_at body_car_3 body_car_zone)"));

    problem_expert_->addPredicate(
      plansys2::Predicate("(piece_is_steering_wheel steering_wheel_1)"));
    problem_expert_->addPredicate(
      plansys2::Predicate("(piece_is_steering_wheel steering_wheel_2)"));
    problem_expert_->addPredicate(
      plansys2::Predicate("(piece_is_steering_wheel steering_wheel_3)"));
    problem_expert_->addPredicate(
      plansys2::Predicate("(piece_at steering_wheel_1 steering_wheels_zone)"));
    problem_expert_->addPredicate(
      plansys2::Predicate("(piece_at steering_wheel_2 steering_wheels_zone)"));
    problem_expert_->addPredicate(
      plansys2::Predicate("(piece_at steering_wheel_3 steering_wheels_zone)"));

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 wheels_zone)"));
    problem_expert_->addPredicate(plansys2::Predicate("(battery_full r2d2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_available r2d2)"));

    problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used wheel_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used wheel_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used wheel_3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used body_car_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used body_car_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used body_car_3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used steering_wheel_1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used steering_wheel_2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(piece_not_used steering_wheel_3)"));
    
    problem_expert_->setGoal(
      plansys2::Goal(
        "(and(car_assembled car_1) (car_assembled car_2) (car_assembled car_3))"));
    /*
    problem_expert_->setGoal(
      plansys2::Goal(
        "(and(robot_at r2d2 steering_wheels_zone) )"));
*/
  }

  void step()
  {
    if (!executor_client_->execute_and_check_plan()) {  // Plan finished
      auto result = executor_client_->getResult();

      if (result.value().success) {
        ROS_INFO("Plan succesfully finished");
      } else {
        ROS_ERROR("Plan finished with error");
      }
      }
  }

private:
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "simple_bt_example_controller_node");
  Assemble node;

  if (!node.init()) {
    return 0;
  }

  ros::Rate rate(5);
  while (ros::ok()) {
    node.step();

    rate.sleep();
    ros::spinOnce();
  }

  ros::shutdown();

  return 0;
}
