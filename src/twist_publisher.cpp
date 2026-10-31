/******************************************************************************
	           ROS 2 Jacobian using KDL Example
          Copyright (C) 2019, 2026 Walter Fetter Lages <w.fetter@ieee.org>

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful, but
        WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
        General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.  If not, see
        <http://www.gnu.org/licenses/>.
        
*******************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <tf2_kdl/tf2_kdl.hpp>
#include <kdl_parser/kdl_parser.hpp>

class TwistPublisher: public rclcpp::Node
{
	public:
	TwistPublisher(const char *name,const char *frameId,const char *childFrameId);
	~TwistPublisher(void);
	void publish(void) const;
			
	private:
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStatesSubscriber_;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twistPublisher_;
	
	std::string frameId_;

	std::string robotDescription_;
	KDL::Chain chain_;
	KDL::JntArrayVel jointPosVel_;
	KDL::ChainFkSolverVel_recursive *fkSolverVel_;
		
	void jointStatesCB(const sensor_msgs::msg::JointState::SharedPtr jointStates);
	void robotDescriptionCB(const std_msgs::msg::String::SharedPtr robotDescription);
};

TwistPublisher::TwistPublisher(const char *name,const char *frameId,const char *childFrameId)
	:Node(name), jointPosVel_(0)
{
	frameId_=frameId;
	
	twistPublisher_=create_publisher<geometry_msgs::msg::TwistStamped>("twist",100);
	
	rclcpp::QoS qos(rclcpp::KeepLast(1));
	qos.transient_local();
	auto robotDescriptionSubscriber=create_subscription<std_msgs::msg::String>("robot_description",qos,std::bind(&TwistPublisher::robotDescriptionCB,this,std::placeholders::_1));
	rclcpp::executors::SingleThreadedExecutor executor;
	while(robotDescription_.empty())
	{
		RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(),*get_clock(),1000,"Waiting for robot model on /robot_description.");
		executor.spin_node_some(get_node_base_interface());
	}

	KDL::Tree tree;
	if (!kdl_parser::treeFromString(robotDescription_,tree))
		RCLCPP_ERROR(get_logger(),"Failed to construct KDL tree.");

	if (!tree.getChain(frameId_,childFrameId,chain_))
		RCLCPP_ERROR(get_logger(),"Failed to get chain from KDL tree.");

	fkSolverVel_=new KDL::ChainFkSolverVel_recursive(chain_);
 
	jointPosVel_.resize(chain_.getNrOfJoints());

	jointStatesSubscriber_=create_subscription<sensor_msgs::msg::JointState>("joint_states",100,std::bind(&TwistPublisher::jointStatesCB,this,std::placeholders::_1));

}

TwistPublisher::~TwistPublisher(void)
{
	delete fkSolverVel_;
}

void TwistPublisher::jointStatesCB(const sensor_msgs::msg::JointState::SharedPtr jointStates)
{
	for(unsigned int i=0;i < jointPosVel_.q.rows();i++)
	{
		jointPosVel_.q(i)=jointStates->position[i];
		jointPosVel_.qdot(i)=jointStates->velocity[i];
	}
	publish();
}

void TwistPublisher::robotDescriptionCB(const std_msgs::msg::String::SharedPtr robotDescription)
{
        robotDescription_=robotDescription->data;
}

void TwistPublisher::publish(void) const
{
	KDL::FrameVel frameVel;
	if(fkSolverVel_->JntToCart(jointPosVel_,frameVel) < 0)
		RCLCPP_ERROR(get_logger(),"Failed to compute forward kinematics velocity.");

	auto twistMsg=tf2::toMsg(tf2::Stamped<KDL::Twist>(frameVel.GetTwist(),tf2::get_now(),frameId_));
	twistPublisher_->publish(twistMsg);
}

int main(int argc,char* argv[])
{
	rclcpp::init(argc,argv);

	if(argc < 3)
	{
                std::cerr << "Usage: twist_publisher base_frame tool_frame\n";
                return -1;
	}

	TwistPublisher twistPublisher("twist_publisher",argv[1],argv[2]);

	rclcpp::spin(twistPublisher.get_node_base_interface());
	return 0;
}
