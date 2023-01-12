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
#include <ros/package.h>
#include <fstream>

//#include "rclcpp_action/rclcpp_action.hpp"

class ControlNode 
{
public:
  ControlNode()
  {
  }

  bool init(std::string problem_pkg, std::string problem_path)
  { 
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge(problem_pkg, problem_path);
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
    } else
      ROS_WARN("Start plan and execution seems to go well \n");

    return true;
  }

  void init_knowledge(std::string problem_pkg, std::string problem_path)
  {
    std::string filepath = ros::package::getPath(problem_pkg) + "/" + problem_path;
    std::ifstream ifs(filepath);
    std::string problem( (std::istreambuf_iterator<char>(ifs) ), (std::istreambuf_iterator<char>()));
    if(problem_expert_->addProblem(problem))
       ROS_WARN("Successfully added problem! \n");


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
  ros::init(argc, argv, "astrobee_move_to_inspect_control_node");
  ControlNode node;
  
  // Defaults
  std::string problem_pkg = "ros2_planning_system_examples";
  std::string problem_path = std::string("common/astrobee_inspection_problem_bumble.pddl");
  
  ros::NodeHandle pnh("~");
  pnh.getParam("problem_pkg", problem_pkg);
  pnh.getParam("problem_path", problem_path);
  
  if (!node.init(problem_pkg, problem_path)) {
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
