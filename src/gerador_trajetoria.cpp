#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kdl/frames.hpp>

class GeradorTrajetoria: public rclcpp::Node
{
public:
	GeradorTrajetoria(void);

private:
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseRefSub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr trajetoPub_;

	KDL::Frame frame_referencia; // frame de referencia recebido do topico /referencia
	KDL::Frame frame_atual;      // ultimo ponto da trajetoria (estado do gerador)

	// Limites do perfil trapezoidal
	float velMax;         // velocidade linear maxima            [m/s]
	float aceleracao;     // aceleracao linear                   [m/s^2]
	float velMaxAng;      // velocidade angular maxima           [rad/s]
	float aceleracaoAng;  // aceleracao angular                  [rad/s^2]
	float rate;           // frequencia do gerador               [Hz]

	// Estado do gerador
	double velAtual_;        // velocidade ao longo do caminho (m/s equivalentes)
	bool temReferencia_;     // ja recebeu alguma referencia?

	rclcpp::TimerBase::SharedPtr timer_;
	void poseRefCB(const geometry_msgs::msg::PoseStamped::SharedPtr poseRefMsg);
	void publicaTrajeto();
	void calculaProximoFrame();
};


GeradorTrajetoria::GeradorTrajetoria(void): Node("gerador_trajetoria"), velAtual_(0.0), temReferencia_(false)
{
	rate=100;

	declare_parameter("vel_max",0.1);
	declare_parameter("aceleracao",0.2);
	declare_parameter("vel_max_ang",0.5);
	declare_parameter("aceleracao_ang",1.0);
	velMax=get_parameter("vel_max").as_double();
	aceleracao=get_parameter("aceleracao").as_double();
	velMaxAng=get_parameter("vel_max_ang").as_double();
	aceleracaoAng=get_parameter("aceleracao_ang").as_double();

	trajetoPub_=create_publisher<geometry_msgs::msg::PoseStamped>("/trajeto",100);
	poseRefSub_=create_subscription<geometry_msgs::msg::PoseStamped>("referencia",100,std::bind(&GeradorTrajetoria::poseRefCB,this,std::placeholders::_1));

	timer_=rclcpp::create_timer(this,this->get_clock(),rclcpp::Duration::from_seconds(1.0/rate),std::bind(&GeradorTrajetoria::publicaTrajeto,this));
}

void GeradorTrajetoria::poseRefCB(const geometry_msgs::msg::PoseStamped::SharedPtr poseRefMsg)
{
	frame_referencia.p.x(poseRefMsg->pose.position.x);
	frame_referencia.p.y(poseRefMsg->pose.position.y);
	frame_referencia.p.z(poseRefMsg->pose.position.z);

	frame_referencia.M=KDL::Rotation::Quaternion(
		poseRefMsg->pose.orientation.x,
		poseRefMsg->pose.orientation.y,
		poseRefMsg->pose.orientation.z,
		poseRefMsg->pose.orientation.w);

	temReferencia_=true;
}

void GeradorTrajetoria::publicaTrajeto()
{
	if(!temReferencia_) return;

	calculaProximoFrame();

	geometry_msgs::msg::PoseStamped poseMsg;
	poseMsg.header.stamp=this->now();
	poseMsg.header.frame_id="base_link";

	// Posicao (m)
	poseMsg.pose.position.x=frame_atual.p.x();
	poseMsg.pose.position.y=frame_atual.p.y();
	poseMsg.pose.position.z=frame_atual.p.z();

	// Orientacao (quaternio unitario)
	frame_atual.M.GetQuaternion(poseMsg.pose.orientation.x,
	                            poseMsg.pose.orientation.y,
	                            poseMsg.pose.orientation.z,
	                            poseMsg.pose.orientation.w);

	trajetoPub_->publish(poseMsg);
}

// Avanca frame_atual um passo (dt=1/rate) em direcao a frame_referencia com
// perfil trapezoidal de velocidade: acelera ate velMax, cruza e freia a tempo
// de parar exatamente na referencia.
//
// Posicao e orientacao sao sincronizadas: ambas seguem um unico parametro de
// caminho "s" (em metros equivalentes), de modo que chegam juntas. A rotacao
// e convertida para metros equivalentes com k=velMax/velMaxAng, e a aceleracao
// efetiva respeita tanto o limite linear quanto o angular.
void GeradorTrajetoria::calculaProximoFrame()
{
	const double dt=1.0/rate;
	const double eps=1e-9;

	// Erro de posicao (vetor) e de orientacao (eixo-angulo, no frame atual)
	const KDL::Vector erroPos=frame_referencia.p-frame_atual.p;
	const double dPos=erroPos.Norm();

	KDL::Vector eixo;
	const double dAng=(frame_atual.M.Inverse()*frame_referencia.M).GetRotAngle(eixo);

	// Comprimento restante do caminho, em "metros equivalentes"
	const double k=velMax/velMaxAng;
	const double sRest=std::max(dPos,dAng*k);

	// Ja chegou: trava na referencia e zera a velocidade
	if(sRest < eps)
	{
		frame_atual=frame_referencia;
		velAtual_=0.0;
		return;
	}

	// Aceleracao efetiva: respeita o limite linear e o angular
	const double aMax=std::min<double>(aceleracao,aceleracaoAng*k);

	// Velocidade maxima da qual ainda e possivel frear no que resta: v=sqrt(2*a*s)
	const double vFreio=std::sqrt(2.0*aMax*sRest);

	// Trapezio: acelera (+a*dt), limita em velMax e em vFreio (fase de frenagem)
	const double vNovo=std::min({velAtual_+aMax*dt,static_cast<double>(velMax),vFreio});
	const double ds=vNovo*dt;

	// Chega neste passo
	if(ds >= sRest)
	{
		frame_atual=frame_referencia;
		velAtual_=0.0;
		return;
	}

	// Fracao do caminho restante percorrida neste passo (igual para pos. e rot.)
	const double f=ds/sRest;

	frame_atual.p+=erroPos*f;
	if(dAng > eps)
		frame_atual.M=frame_atual.M*KDL::Rotation::Rot(eixo,dAng*f);

	velAtual_=vNovo;
}

int main(int argc,char* argv[])
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<GeradorTrajetoria>());
	rclcpp::shutdown();

	return 0;
}