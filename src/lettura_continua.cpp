#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) 
    {
        // Stampa il messaggio ricevuto
        RCLCPP_INFO(this->get_logger(), "Received joint states:");
        for (size_t i = 0; i < msg->name.size(); ++i) 
        {
            RCLCPP_INFO(this->get_logger(), "Joint: %s, Position: %f",msg->name[i].c_str(), msg->position[i]);
        }

    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}