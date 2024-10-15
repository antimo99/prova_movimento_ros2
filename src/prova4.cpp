#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "rclcpp/wait_for_message.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <chrono>
//#include <control_msgs/FollowJointTrajectoryAction.h>


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);


  std::vector<std::string> joint_names_ = {"panda_joint1",
                                             "panda_joint2",
                                             "panda_joint3",
                                             "panda_joint4",
                                             "panda_joint5",
                                             "panda_joint6",
                                             "panda_joint7"};
  
  auto node = std::make_shared<rclcpp::Node>("client_node");



  //inizializzo l'action client
  auto action_client = rclcpp_action::create_client<control_msgs::FollowJointTrajectoryAction>(node, "/position_joint_trajectory_controller/follow_joint_trajectory");
  
  //attendo che i server siano attivi
  RCLCPP_INFO(node->get_logger(), "Waiting for servers...");
  action_client->wait_for_action_server();
  RCLCPP_INFO(node->get_logger(), "Servers UP");

  /*auto msg=sensor_msgs::msg::JointState();
  auto ricevuto=rclcpp::wait_for_message<sensor_msgs::msg::JointState>(msg,node->get_node_base_interface(),"joint_states",std::chrono::seconds(1));
 

  if(ricevuto)
   {
      RCLCPP_INFO(node->get_logger(), "Messaggio ricevuto");
      q0.position=msg.position;
   }
   else
   {
      RCLCPP_INFO(node->get_logger(), "Messaggio non ricevuto");
   }*/

   sensor_msgs::msg::JointState q0;
   if (!rclcpp::wait_for_message<sensor_msgs::msg::JointState>(q0, node, "joint_states", std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(node->get_logger(), "Fail nell'ottenere lo stato dei giunti iniziale q0");
      return;
    }

  
    //costruisco il goal
    control_msgs::FollowJointTrajectoryActionGoal goal;
    goal.goal.trajectory.joint_names=joint_names_;


    goal.header.seq=1;
    rclcpp::Duration delta(2,0);
    goal.header.stamp=node->now()+delta;
    //goal.header.frame_id="ci vuole una stringa";
    goal.goal_id.stamp=node->now()+delta;
    //goal.goal_id.id="stringa";
    //goal.goal.trajectory.header.seq=1;
    goal.goal.trajectory.header.stamp=rclcpp::Time(0,0);
    //goal.goal.trajectory.header.frame_id="stringa";
    goal.goal.trajectory.joint_names=joint_names_;
    goal.goal.trajectory.points.resize(2);
    //goal.goal.trajectory.points.velocities=vettore di veelocità;
    //goal.goal.trajectory.points.accelerations=vettore di accelerazioni
    //goal.goal.trajectory.points.effort=vettore di forze;

    goal.goal.trajectory.points[0].positions.resize(7);
    goal.goal.trajectory.points[0].positions=q0.position;

    goal.goal.trajectory.points[1].positions.resize(7);
    goal.goal.trajectory.points[1].positions=q0.position;
    goal.goal.trajectory.points[1].positions[6]+=0.8;

    rclcpp::Duration iniziale(0,0);
    rclcpp::Duration finale(15,0);
    goal.goal.trajectory.points[0].time_from_start=iniziale;
    goal.goal.trajectory.points[1].time_from_start=finale;


    //goal.goal.path_tolerance.resize(1);
    //goal.goal.path_tolerance[0].name="tolleranza1";
    //goal.goal.trajectory.path_tolerance.position=1.0;
    //goal.goal.trajectory.path_tolerance.veelocity=1.0;
    //goal.goal.trajectory.path_tolerance.acceeleration=1.0;

    goal.goal.goal_tolerance.resize(1);
    //goal.goal.goal_tolerance[0].name="tolleranza2";
    //goal.goal.trajectory.goal_tolerance.position=1.
    //goal.goal.trajectory.goal_tolerance.velocity=1.0;
    //goal.goal.trajectory.goal_tolerance.acceleration=1.0;
    goal.goal.goal_time_tolerance=rclcpp::Duration(0, 500000000);


    // chiamo l'azione e aspetto che termini
    auto future_goal_handle1 = action_client->async_send_goal(goal);
       if (rclcpp::spin_until_future_complete(node, future_goal_handle1) != rclcpp::FutureReturnCode::SUCCESS)
          {
            RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, GOAL NOT SENT");
            return -1;
          }

     auto future_result1 = action_client->async_get_result(future_goal_handle1.get());
         if (rclcpp::spin_until_future_complete(node, future_result1) != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, NO RESULT");
                return -1;
            }
            // se arrivo quì l'azione è terminata, controllo se è terminata con successo

            // check dello stato dell'azione, se non ho errori lo stato deve essere SUCCEEDED
            if (future_result1.get().code != rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, JOINT TRAJECTORY NOT SUCCEEDED");
                return -1;
            }
  
  rclcpp::shutdown();
  return 0;
}
