#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

class Controlador: public rclcpp::Node{
    public:
        Controlador(void);

    private:
        std::string robotDescription_;
        std::string frameId_="base_link";
        std::string childFrameId_="tool0";
        KDL::Chain chain_;
        KDL::JntArrayVel jointPosVelAtual_; // Posição das juntas e velocidades das juntas
        KDL::JntArrayVel jointPosVelRef_; // Posição das juntas e velocidades das juntas Referencia 

        KDL::Frame framePosReferencia;
        KDL::Frame framePosAtual;
        KDL::FrameVel frameVelAtual;

        KDL::Twist deltaX;

        double ganho_;
        double dqMax_;

        KDL::ChainFkSolverVel_recursive *fkSolverVel_;
        KDL::ChainFkSolverPos_recursive *fkSolverPos_;
    
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twistSub_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub_;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStatesSubscriber_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trajetoSub_;

        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr jointTrajectoryPointPublisher_;

        void robotDescriptionCB(const std_msgs::msg::String::SharedPtr robotDescription);
        void jointStatesCB(const sensor_msgs::msg::JointState::SharedPtr jointStates);
        void trajetoCB(const geometry_msgs::msg::PoseStamped::SharedPtr trajetoMsg);
        void pubQ();

        void calcDeltaX();
        void calcJacobianoInversa();
};

Controlador::Controlador(void): Node("controlador")
{   
    trajetoSub_=create_subscription<geometry_msgs::msg::PoseStamped>("trajeto",100,std::bind(&Controlador::trajetoCB,this,std::placeholders::_1));

    jointStatesSubscriber_=create_subscription<sensor_msgs::msg::JointState>("joint_states",100,std::bind(&Controlador::jointStatesCB,this,std::placeholders::_1));

    rclcpp::QoS qos(rclcpp::KeepLast(1));
	qos.transient_local();
	auto robotDescriptionSubscriber=create_subscription<std_msgs::msg::String>("robot_description",qos,std::bind(&Controlador::robotDescriptionCB,this,std::placeholders::_1));
	rclcpp::executors::SingleThreadedExecutor executor;
	while(robotDescription_.empty())
	{
		RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(),*get_clock(),1000,"Waiting for robot model on /robot_description.");
		executor.spin_node_some(get_node_base_interface());
	}

	KDL::Tree tree;
	if (!kdl_parser::treeFromString(robotDescription_,tree))
		RCLCPP_ERROR(get_logger(),"Failed to construct KDL tree.");

	if (!tree.getChain(frameId_,childFrameId_,chain_))
		RCLCPP_ERROR(get_logger(),"Failed to get chain from KDL tree.");


    jointTrajectoryPointPublisher_=create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("/joint_trajectory_point",100);

    fkSolverPos_=new KDL::ChainFkSolverPos_recursive(chain_);
    fkSolverVel_=new KDL::ChainFkSolverVel_recursive(chain_);
}

void Controlador::calcDeltaX()
{
    deltaX=ganho_*KDL::diff(framePosAtual,framePosReferencia);
}

void Controlador::calcJacobianoInversa()
{
    // Implementação do cálculo do Jacobiano Inverso
}


void Controlador::trajetoCB(const geometry_msgs::msg::PoseStamped::SharedPtr trajetoMsg)
{
    framePosReferencia.p.x(trajetoMsg->pose.position.x);
    framePosReferencia.p.y(trajetoMsg->pose.position.y);
    framePosReferencia.p.z(trajetoMsg->pose.position.z);
    framePosReferencia.M=KDL::Rotation::Quaternion(
        trajetoMsg->pose.orientation.x,
        trajetoMsg->pose.orientation.y,
        trajetoMsg->pose.orientation.z,
        trajetoMsg->pose.orientation.w);
}

void Controlador::pubQ()
{
    trajectory_msgs::msg::JointTrajectoryPoint jointTrajectoryPointMsg;
    jointTrajectoryPointMsg.positions.resize(jointPosVelRef_.q.rows());
    for(unsigned int i=0;i < jointPosVelRef_.q.rows();i++)
        jointTrajectoryPointMsg.positions[i]=jointPosVelRef_.q(i);
    
    jointTrajectoryPointPublisher_->publish(jointTrajectoryPointMsg);
}

void Controlador::robotDescriptionCB(const std_msgs::msg::String::SharedPtr robotDescription){
    robotDescription_=robotDescription->data;
}

void Controlador::jointStatesCB(const sensor_msgs::msg::JointState::SharedPtr jointStates)
{
	for(unsigned int i=0;i < jointPosVelAtual_.q.rows();i++)
	{
		jointPosVelAtual_.q(i)=jointStates->position[i];
		jointPosVelAtual_.qdot(i)=jointStates->velocity[i];
	}

    if (fkSolverPos_->JntToCart(jointPosVelAtual_.q,framePosAtual) < 0)
		RCLCPP_ERROR(get_logger(),"Failed to compute forward kinematics position.");

    if(fkSolverVel_->JntToCart(jointPosVelAtual_,frameVelAtual) < 0)
		RCLCPP_ERROR(get_logger(),"Failed to compute forward kinematics velocity.");
}



int main (int argc,char* argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Controlador>());
    rclcpp::shutdown();

    return 0;
}