/******************************************************************************
  pose_publisher: cinematica direta de posicao (bloco fk(q) do Resolved Motion
  Rate Control).

  Assina  joint_states (sensor_msgs/JointState)
  Publica pose         (geometry_msgs/PoseStamped) -> X = fk(q), expressa em
                                                     base_frame

  Uso: pose_publisher base_frame tool_frame
******************************************************************************/

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>

class PosePublisher: public rclcpp::Node
{
	public:
	PosePublisher(const char *name,const char *frameId,const char *childFrameId);

	private:
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStatesSubscriber_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robotDescriptionSubscriber_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePublisher_;

	std::string frameId_;
	std::string robotDescription_;

	KDL::Chain chain_;
	std::vector<std::string> jointNames_;	// juntas moveis da cadeia, na ordem do KDL
	KDL::JntArray q_;
	std::unique_ptr<KDL::ChainFkSolverPos_recursive> fkSolverPos_;

	void jointStatesCB(const sensor_msgs::msg::JointState::SharedPtr jointStates);
	void robotDescriptionCB(const std_msgs::msg::String::SharedPtr robotDescription);
	void publish(const rclcpp::Time &stamp) const;
};

PosePublisher::PosePublisher(const char *name,const char *frameId,const char *childFrameId)
	:Node(name), frameId_(frameId)
{
	posePublisher_=create_publisher<geometry_msgs::msg::PoseStamped>("pose",100);

	// 1) Espera o modelo do robo (publicado uma vez, QoS transient_local).
	//    A assinatura de joint_states so e criada no fim do construtor, para o
	//    callback nunca rodar antes de o solver existir.
	rclcpp::QoS qos(rclcpp::KeepLast(1));
	qos.transient_local();
	robotDescriptionSubscriber_=create_subscription<std_msgs::msg::String>("robot_description",qos,std::bind(&PosePublisher::robotDescriptionCB,this,std::placeholders::_1));

	rclcpp::executors::SingleThreadedExecutor executor;
	while(rclcpp::ok() && robotDescription_.empty())
	{
		RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(),*get_clock(),1000,"Waiting for robot model on /robot_description.");
		executor.spin_node_some(get_node_base_interface());
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	if(robotDescription_.empty())
		throw std::runtime_error("Interrupted while waiting for /robot_description.");

	// 2) Monta a cadeia base -> ferramenta.
	KDL::Tree tree;
	if(!kdl_parser::treeFromString(robotDescription_,tree))
		throw std::runtime_error("Failed to construct KDL tree.");

	if(!tree.getChain(frameId_,childFrameId,chain_))
		throw std::runtime_error(std::string("Failed to get chain from '")+frameId_+"' to '"+childFrameId+"'. Check the link names.");

	for(unsigned int i=0;i < chain_.getNrOfSegments();i++)
	{
		const KDL::Joint &joint=chain_.getSegment(i).getJoint();
		if(joint.getType() != KDL::Joint::None)
			jointNames_.push_back(joint.getName());
	}

	fkSolverPos_=std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
	q_.resize(chain_.getNrOfJoints());
	KDL::SetToZero(q_);

	RCLCPP_INFO(get_logger(),"Chain '%s' -> '%s' with %u joints.",frameId_.c_str(),childFrameId,chain_.getNrOfJoints());

	// 3) So agora comeca a receber joint_states.
	jointStatesSubscriber_=create_subscription<sensor_msgs::msg::JointState>("joint_states",100,std::bind(&PosePublisher::jointStatesCB,this,std::placeholders::_1));
}

void PosePublisher::jointStatesCB(const sensor_msgs::msg::JointState::SharedPtr jointStates)
{
	// Associa cada junta da cadeia pelo NOME (joint_states pode ter outras
	// juntas e outra ordem).
	for(size_t i=0;i < jointNames_.size();i++)
	{
		auto it=std::find(jointStates->name.begin(),jointStates->name.end(),jointNames_[i]);
		if(it == jointStates->name.end())
		{
			RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Joint '%s' not found in joint_states.",jointNames_[i].c_str());
			return;
		}

		size_t idx=std::distance(jointStates->name.begin(),it);
		if(idx >= jointStates->position.size())
		{
			RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"joint_states has no position for '%s'.",jointNames_[i].c_str());
			return;
		}

		q_(i)=jointStates->position[idx];
	}

	rclcpp::Time stamp(jointStates->header.stamp);
	if(stamp.nanoseconds() == 0)
		stamp=now();

	publish(stamp);
}

void PosePublisher::robotDescriptionCB(const std_msgs::msg::String::SharedPtr robotDescription)
{
	robotDescription_=robotDescription->data;
}

void PosePublisher::publish(const rclcpp::Time &stamp) const
{
	KDL::Frame frame;
	if(fkSolverPos_->JntToCart(q_,frame) < 0)
	{
		RCLCPP_ERROR(get_logger(),"Failed to compute forward kinematics.");
		return;
	}

	geometry_msgs::msg::PoseStamped poseMsg;
	poseMsg.header.stamp=stamp;
	poseMsg.header.frame_id=frameId_;

	// Posicao (m)
	poseMsg.pose.position.x=frame.p.x();
	poseMsg.pose.position.y=frame.p.y();
	poseMsg.pose.position.z=frame.p.z();

	// Orientacao (quaternio unitario)
	frame.M.GetQuaternion(poseMsg.pose.orientation.x,
	                      poseMsg.pose.orientation.y,
	                      poseMsg.pose.orientation.z,
	                      poseMsg.pose.orientation.w);

	posePublisher_->publish(poseMsg);
}

int main(int argc,char* argv[])
{
	if(argc < 3)
	{
		std::cerr << "Usage: pose_publisher base_frame tool_frame\n";
		return -1;
	}

	rclcpp::init(argc,argv);

	try
	{
		auto posePublisher=std::make_shared<PosePublisher>("pose_publisher",argv[1],argv[2]);
		rclcpp::spin(posePublisher);
	}
	catch(const std::exception &e)
	{
		RCLCPP_FATAL(rclcpp::get_logger("pose_publisher"),"%s",e.what());
		rclcpp::shutdown();
		return 1;
	}

	rclcpp::shutdown();
	return 0;
} 