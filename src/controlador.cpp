#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>


class Controlador: public rclcpp::Node{
    public:
        Controlador(void);

    private:
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twistSub_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub_;

        void twistCB(const geometry_msgs::msg::TwistStamped::SharedPtr twistMsg);
        void poseCB(const geometry_msgs::msg::PoseStamped::SharedPtr poseMsg);

};

Controlador::Controlador(void): Node("controlador")
{
    twistSub_=create_subscription<geometry_msgs::msg::TwistStamped>("twist",100,std::bind(&Controlador::twistCB,this,std::placeholders::_1));
    
    poseSub_=create_subscription<geometry_msgs::msg::PoseStamped>("pose",100,std::bind(&Controlador::poseCB,this,std::placeholders::_1));
}

void Controlador::twistCB(const geometry_msgs::msg::TwistStamped::SharedPtr twistMsg)
{
    RCLCPP_INFO(this->get_logger(),"Twist: linear=(%f,%f,%f) angular=(%f,%f,%f)",twistMsg->twist.linear.x,twistMsg->twist.linear.y,twistMsg->twist.linear.z,twistMsg->twist.angular.x,twistMsg->twist.angular.y,twistMsg->twist.angular.z);
}



int main (int argc,char* argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Controlador>());
    rclcpp::shutdown();

    return 0;
}