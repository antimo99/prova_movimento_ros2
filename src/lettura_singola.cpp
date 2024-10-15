#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JointStateListener : public rclcpp::Node {
public:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    bool message_received_;

    JointStateListener() : Node("joint_state_listener")
     {
        // subscriber al topic "joint_states"
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&JointStateListener::topic_callback, this, std::placeholders::_1));

        // Inizializza il flag
        message_received_ = false;
    }

    void wait_for_message() 
    {
        // Aspetta che venga ricevuto un messaggio
        while (rclcpp::ok() && !message_received_) 
        {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Attesa breve
        }
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

        // Imposta il flag per indicare che il messaggio è stato ricevuto
        message_received_ = true;
     }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto listener = std::make_shared<JointStateListener>();

    // Aspetta di ricevere un messaggio
    listener->wait_for_message();

    rclcpp::shutdown();
    return 0;
}