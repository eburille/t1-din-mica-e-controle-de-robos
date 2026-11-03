/******************************************************************************
  Controlador: Resolved Motion Rate Control

    X^d (/trajeto) --> ΔX = K*(X^d - X) --> Δq = J^-1(q)*ΔX --> q^d = q + Δq
                                                                   |
                                                      /command <---+

  Ao iniciar, leva as juntas a q_inicial (espaco das juntas, com velocidade
  limitada) e so depois liga o controle cartesiano, com a referencia inicial
  igual a fk(q_inicial).

  Assina  trajeto       (geometry_msgs/PoseStamped)   referencia X^d
          joint_states  (sensor_msgs/JointState)      q e qdot medidos
  Publica /command (JointTrajectoryPoint) com q^d em positions

  Parametros:
    base_frame   (string) Base da cadeia.                   Padrao: "wam_link_0"
    tool_frame   (string) Efetuador.                        Padrao: "wam_link_7"
    ganho        (double) Ganho K do erro cartesiano.       Padrao: 1.0
    lambda       (double) Amortecimento do WDLS (singul.).  Padrao: 0.01
    dq_max       (double) Passo maximo por junta por ciclo  Padrao: 0.02 [rad]
    q_inicial    (double[]) Juntas iniciais [rad]           Padrao: [0,-1,0,2.5,0,0,0]
    vel_inicial  (double) Vel. maxima ate q_inicial         Padrao: 0.3 [rad/s]
    tol_inicial  (double) Tolerancia para "chegou"          Padrao: 0.005 [rad]
******************************************************************************/

#include <algorithm>
#include <chrono>
#include <cmath>
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
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl_parser/kdl_parser.hpp>

class Controlador: public rclcpp::Node
{
    public:
        Controlador(void);

    private:
        std::string frameId_;
        std::string childFrameId_;
        KDL::Chain chain_;
        std::vector<std::string> jointNames_;   // juntas moveis da cadeia, na ordem do KDL

        KDL::JntArrayVel jointPosVelAtual_; // posicao e velocidade das juntas (medidas)
        KDL::JntArrayVel jointPosVelRef_;   // posicao das juntas desejada (q^d)
        KDL::JntArray dq_;                  // Δq

        KDL::Frame framePosReferencia;      // X^d
        KDL::Frame framePosAtual;           // X = fk(q)
        KDL::FrameVel frameVelAtual;
        KDL::Twist deltaX;                  // ΔX = K*(X^d - X)

        double ganho_;
        double dqMax_;
        double velInicial_;                 // [rad/s] ate a posicao inicial
        double tolInicial_;                 // [rad]
        bool indoParaInicial_;              // true ate as juntas chegarem em q_inicial
        KDL::JntArray qInicial_;            // juntas iniciais
        rclcpp::Time tAnterior_;            // para o dt da fase inicial

        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fkSolverPos_;
        std::unique_ptr<KDL::ChainFkSolverVel_recursive> fkSolverVel_;
        std::unique_ptr<KDL::ChainIkSolverVel_wdls> ikSolverVel_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trajetoSub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStatesSubscriber_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr jointTrajectoryPointPublisher_;

        void robotDescriptionCB(const std_msgs::msg::String::SharedPtr robotDescription);
        void jointStatesCB(const sensor_msgs::msg::JointState::SharedPtr jointStates);
        void trajetoCB(const geometry_msgs::msg::PoseStamped::SharedPtr trajetoMsg);
        void pubQ();

        void moveParaPosicaoInicial();
        void calcDeltaX();
        bool calcJacobianoInversa();

        std::string robotDescription_;
};

Controlador::Controlador(void)
    : Node("controlador"), jointPosVelAtual_(0), jointPosVelRef_(0),
      framePosReferencia(KDL::Rotation::Identity(),KDL::Vector::Zero()),
      ganho_(1.0), dqMax_(0.02), velInicial_(0.3), tolInicial_(0.005),
      indoParaInicial_(true)
{
    declare_parameter("base_frame","wam_link_0");
    declare_parameter("tool_frame","wam_link_7");
    declare_parameter("ganho",1.0);
    declare_parameter("lambda",0.01);
    declare_parameter("dq_max",0.02);
    declare_parameter("q_inicial",std::vector<double>{0.0,-1.0,0.0,2.5,0.0,0.0,0.0});
    declare_parameter("vel_inicial",0.3);
    declare_parameter("tol_inicial",0.005);

    frameId_=get_parameter("base_frame").as_string();
    childFrameId_=get_parameter("tool_frame").as_string();
    ganho_=get_parameter("ganho").as_double();
    dqMax_=get_parameter("dq_max").as_double();
    velInicial_=get_parameter("vel_inicial").as_double();
    tolInicial_=get_parameter("tol_inicial").as_double();
    const double lambda=get_parameter("lambda").as_double();
    const std::vector<double> qIni=get_parameter("q_inicial").as_double_array();

    jointTrajectoryPointPublisher_=create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("/command",100);

    // 1) Espera o modelo do robo (publicado uma vez, QoS transient_local).
    //    As assinaturas de trajeto e joint_states so sao criadas no fim, para
    //    os callbacks nunca rodarem antes de os solvers existirem.
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();
    auto robotDescriptionSubscriber=create_subscription<std_msgs::msg::String>("robot_description",qos,std::bind(&Controlador::robotDescriptionCB,this,std::placeholders::_1));

    rclcpp::executors::SingleThreadedExecutor executor;
    while(rclcpp::ok() && robotDescription_.empty())
    {
        RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(),*get_clock(),1000,"Waiting for robot model on /robot_description.");
        executor.spin_node_some(get_node_base_interface());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if(robotDescription_.empty())
        throw std::runtime_error("Interrupted while waiting for /robot_description.");

    // 2) Monta a cadeia. Falhas aqui sao fatais.
    KDL::Tree tree;
    if(!kdl_parser::treeFromString(robotDescription_,tree))
        throw std::runtime_error("Failed to construct KDL tree.");

    if(!tree.getChain(frameId_,childFrameId_,chain_))
        throw std::runtime_error("Failed to get chain from '"+frameId_+"' to '"+childFrameId_+"'. Check the link names.");

    for(unsigned int i=0;i < chain_.getNrOfSegments();i++)
    {
        const KDL::Joint &joint=chain_.getSegment(i).getJoint();
        if(joint.getType() != KDL::Joint::None)
            jointNames_.push_back(joint.getName());
    }
    if(jointNames_.empty())
        throw std::runtime_error("The chain has no movable joints.");

    // 3) Solvers e vetores com o tamanho correto
    const unsigned int n=chain_.getNrOfJoints();
    fkSolverPos_=std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    fkSolverVel_=std::make_unique<KDL::ChainFkSolverVel_recursive>(chain_);
    ikSolverVel_=std::make_unique<KDL::ChainIkSolverVel_wdls>(chain_);
    ikSolverVel_->setLambda(lambda);    // amortecimento perto de singularidades

    jointPosVelAtual_.resize(n);
    jointPosVelRef_.resize(n);
    dq_.resize(n);
    KDL::SetToZero(jointPosVelAtual_.q);
    KDL::SetToZero(jointPosVelAtual_.qdot);
    KDL::SetToZero(jointPosVelRef_.q);
    KDL::SetToZero(dq_);

    // 4) Posicao inicial das juntas
    if(qIni.size() != n)
        throw std::runtime_error("Parameter 'q_inicial' must have "+std::to_string(n)+" values (one per joint of the chain).");

    qInicial_.resize(n);
    for(unsigned int i=0;i < n;i++)
        qInicial_(i)=qIni[i];

    // A referencia cartesiana inicial e a pose correspondente a q_inicial
    if(fkSolverPos_->JntToCart(qInicial_,framePosReferencia) < 0)
        throw std::runtime_error("Failed to compute the pose for 'q_inicial'.");

    tAnterior_=get_clock()->now();

    RCLCPP_INFO(get_logger(),"Chain '%s' -> '%s' with %u joints (K=%.2f, lambda=%.3f, dq_max=%.3f).",
                frameId_.c_str(),childFrameId_.c_str(),n,ganho_,lambda,dqMax_);
    RCLCPP_INFO(get_logger(),"Moving to the initial joint position at up to %.2f rad/s.",velInicial_);

    // 5) So agora comeca a receber dados
    trajetoSub_=create_subscription<geometry_msgs::msg::PoseStamped>("trajeto",100,std::bind(&Controlador::trajetoCB,this,std::placeholders::_1));
    jointStatesSubscriber_=create_subscription<sensor_msgs::msg::JointState>("joint_states",100,std::bind(&Controlador::jointStatesCB,this,std::placeholders::_1));
}

// Fase inicial: leva as juntas a q_inicial no espaco das juntas, com velocidade
// maxima vel_inicial (mantendo a direcao do movimento). Ao chegar, o controle
// cartesiano assume, com a referencia ja em fk(q_inicial).
void Controlador::moveParaPosicaoInicial()
{
    const rclcpp::Time agora=get_clock()->now();
    const double dt=std::clamp((agora-tAnterior_).seconds(),0.0,0.1);
    tAnterior_=agora;

    double maxErro=0.0;
    for(unsigned int i=0;i < dq_.rows();i++)
    {
        dq_(i)=qInicial_(i)-jointPosVelAtual_.q(i);
        maxErro=std::max(maxErro,std::fabs(dq_(i)));
    }

    if(maxErro < tolInicial_)
    {
        indoParaInicial_=false;
        for(unsigned int i=0;i < dq_.rows();i++)
            jointPosVelRef_.q(i)=qInicial_(i);
        RCLCPP_INFO(get_logger(),"Initial joint position reached. Cartesian control enabled.");
        pubQ();
        return;
    }

    const double escala=std::min(1.0,velInicial_*dt/maxErro);
    for(unsigned int i=0;i < dq_.rows();i++)
        jointPosVelRef_.q(i)=jointPosVelAtual_.q(i)+escala*dq_(i);

    pubQ();
}

// ΔX = K*(X^d - X): erro de posicao e de orientacao (eixo*angulo) no frame da base
void Controlador::calcDeltaX()
{
    deltaX=ganho_*KDL::diff(framePosAtual,framePosReferencia);
}

// Δq = J^-1(q)*ΔX e q^d = q + Δq
bool Controlador::calcJacobianoInversa()
{
    if(ikSolverVel_->CartToJnt(jointPosVelAtual_.q,deltaX,dq_) < 0)
    {
        RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"Failed to compute the inverse Jacobian.");
        return false;
    }

    // Seguranca: limita o passo mantendo a direcao de Δq
    double maxAbs=0.0;
    for(unsigned int i=0;i < dq_.rows();i++)
        maxAbs=std::max(maxAbs,std::fabs(dq_(i)));

    if(maxAbs > dqMax_)
    {
        const double escala=dqMax_/maxAbs;
        for(unsigned int i=0;i < dq_.rows();i++)
            dq_(i)*=escala;
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Step limited by dq_max (largest |dq| was %.3f rad).",maxAbs);
    }

    for(unsigned int i=0;i < dq_.rows();i++)
        jointPosVelRef_.q(i)=jointPosVelAtual_.q(i)+dq_(i);

    return true;
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

void Controlador::robotDescriptionCB(const std_msgs::msg::String::SharedPtr robotDescription)
{
    robotDescription_=robotDescription->data;
}

void Controlador::jointStatesCB(const sensor_msgs::msg::JointState::SharedPtr jointStates)
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

        const size_t idx=std::distance(jointStates->name.begin(),it);
        if(idx >= jointStates->position.size())
        {
            RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"joint_states has no position for '%s'.",jointNames_[i].c_str());
            return;
        }

        jointPosVelAtual_.q(i)=jointStates->position[idx];
        // Muitos publishers de joint_states nao enviam velocity.
        jointPosVelAtual_.qdot(i)=(idx < jointStates->velocity.size()) ? jointStates->velocity[idx] : 0.0;
    }

    if(fkSolverPos_->JntToCart(jointPosVelAtual_.q,framePosAtual) < 0)
    {
        RCLCPP_ERROR(get_logger(),"Failed to compute forward kinematics position.");
        return;
    }

    if(fkSolverVel_->JntToCart(jointPosVelAtual_,frameVelAtual) < 0)
        RCLCPP_ERROR(get_logger(),"Failed to compute forward kinematics velocity.");

    // Primeiro leva o robo a q_inicial; so depois liga o controle cartesiano
    if(indoParaInicial_)
    {
        moveParaPosicaoInicial();
        return;
    }

    calcDeltaX();
    if(calcJacobianoInversa())
        pubQ();
}

int main(int argc,char* argv[])
{
    rclcpp::init(argc,argv);

    try
    {
        rclcpp::spin(std::make_shared<Controlador>());
    }
    catch(const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("controlador"),"%s",e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}