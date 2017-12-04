// Bibliotecas
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/math/Vector3.hh>
#include "XMLRead.h" // biblioteca para acesso a dados do xml
#include <update_timer.h>
#include "serial_asio.hpp"




namespace gazebo
{

	class HilServer : public ModelPlugin
	{
		public: 
			HilServer();
	  		virtual ~HilServer(); 
			virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
	  	 	virtual void Reset();  

					
		protected: 
			// função chamada a cada passo de simulação
			virtual void Update();
		private: 
			// Método para receber dados do sistema embarcado (Callback)
			void GetSerialData(const boost::system::error_code& e,std::size_t size);
			// Método para enviar dados para o sistema embarcado 
			void SetSerialData();
			// Método para Ler dados de simulação
			void GetStatesVANT20();
			// Método para escrever dados de simulação
			void SetInputVANT20(double Fr,double Fl,double Tr,double Tl);
			// Segurador de ordem zero
			void Zoh();
			// leitura de dados
			void thread();
			
			
		// Informações do arquivo model.sdf
		std::string NameOfJointR_; // nome da junta correspondente ao servo motor direito
		std::string NameOfJointL_; // nome da junta correspondente ao servo motor esquerdo
		std::string link_name_;	   // nome do elo correpondente ao corpo principal
		std::string link_right_;   // nome do elo correpondente ao motor brushless direito
		std::string link_left_;    // nome do elo correpondente ao motor brushless esquerdo
		physics::LinkPtr link;	   // ponteiro para acesso a dados do corpo principal
		physics::LinkPtr linkR;    // ponteiro para acesso a dados do corpo direito
		physics::LinkPtr linkL;    // ponteiro para acesso a dados do corpo esquerdo
		physics::WorldPtr world;   // ponteiro para acesso a dados do mundo
		physics::JointPtr juntaR;  // ponteiro para acesso a dados da junta direita
		physics::JointPtr juntaL;  // ponteiro para acesso a dados da junta esquerda
		
		// dados de simulação
		double x,y,z,roll,pitch,yaw,alphar,alphal,vx,vy,vz,wx,wy,wz,dalphar,dalphal;
		// sinais de entrada
		double Fr, Fl, Tr, Tl;
		
		// configuração de temporizador
		UpdateTimer updateTimer;
  		event::ConnectionPtr updateConnection;
  		
  		// Classe de comunicação serial 
  		SerialClass serial;	
  		
  		// Thread
  		boost::thread *t;				

	};
}

//#endif
