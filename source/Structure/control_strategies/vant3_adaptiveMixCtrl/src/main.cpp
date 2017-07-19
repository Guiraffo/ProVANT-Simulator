#include "Icontroller.hpp"
#include <iostream>
#include <Eigen/Eigen>
#include "simulator_msgs/Sensor.h"
#include <cmath>

class vant3_adaptiveMixCtrl : public Icontroller
{
	private: double e;
	private: double NumeroConjuntos; 
	private: double pi;

	//Integradores
	private: double T;

	//Mapeamento
	private: Eigen::MatrixXd RI_B;
	private: Eigen::MatrixXd Wn;

	//Controlador dinâmico
	private: Eigen::MatrixXd K;
	private: Eigen::MatrixXd K1;
	private: Eigen::MatrixXd K2;
	private: Eigen::MatrixXd K3;
	private: Eigen::MatrixXd K4;
	private: Eigen::MatrixXd K5;
	private: Eigen::VectorXd Uref;
	private: Eigen::VectorXd Xref;
	private: Eigen::VectorXd X;
	private: Eigen::VectorXd Erro;
	private: Eigen::VectorXd Input;

	//Mixing adaptativo
	private: Eigen::VectorXd center;
	private: Eigen::VectorXd cutoff;
	private: std::vector<double> psii;
	private: std::vector<double> mii;

	//Controlador cinemático
	private: Eigen::VectorXd Trajectory;
	private: Eigen::MatrixXd Kk;
	private: Eigen::VectorXd Xk;
	private: Eigen::VectorXd Uk;
	private: Eigen::VectorXd Dk;



	public: vant3_adaptiveMixCtrl(): Xref(18,1),     X(18,1), Erro(18,1), Input(6,1),   Uref(6,1),  Trajectory(20,1),
   									    K(6,18),    K1(6,18),   K2(6,18),   K3(6,18),    K4(6,18),          K5(6,18),
									  center(5),   cutoff(5),    psii(5),     mii(5),   RI_B(3,3),           Wn(3,3), 
                                        Kk(3,3),     Xk(3,1),    Uk(3,1),    Dk(3,1)
	{	e = 2.718281828459046;
		pi = 3.141592653589793;
		NumeroConjuntos = 5;
		T = 0.012;
		Uref << 8.46, 8.46, -1.74*pow(10,-5), -1.74*pow(10,-5), 0, 0;

		K1 << 0.6882,-17.797,-27.111,2.3716,14.76,7.2797,3.2209,1.7684,-47.505,44.007,177.49,1.7374,81.988,29.381,-17.864,-0.13725,-19.865,5.0673,
			  5.258,24.224,-24.796,-2.6374,17.788,-5.5154,2.5947,3.5973,-38.149,-47.496,147.58,3.7147,61.289,77.037,-12.963,2.3957,22.233,3.4525,
			  0.0041721,0.012851,0.085796,0.0043825,-0.052614,-0.054448,-0.022045,0.0011651,0.15532,-0.024172,-0.59503,-0.11881,-0.40454,0.06699,0.052412,0.0024005,0.014331,-0.072794,
			 -0.054653,-0.0084829,-0.022359,-0.0047923,-0.026768,0.050669,0.0027455,-0.01968,-0.046318,-0.0038277,-0.067601,0.12026,0.074586,-0.32586,-0.0074509,-0.024796,0.00098278,0.068787,
			 -1.6991,0.063483,-0.55465,-0.0087758,-2.2071,0.1054,-0.045105,-0.05445,-1.0232,-0.074476,-9.3369,0.32482,-4.0409,-4.8135,-0.10556,-0.72215,-0.03389,0.36026,
			  2.5466,-6.558,2.5667,-0.096004,0.16436,2.8917,0.20037,-0.034041,3.1586,11.064,9.6936,1.9455,10.253,-2.7275,0.73827,1.1151,-7.0454,0.88752;

		K2 << 0.53071,-1.4211,-13.849,2.994,1.7182,0.040077,0.28157,0.43161,-19.312,11.911,104.11,-9.9803,11.185,11.801,-7.1685,-0.8618,-0.21681,-2.4424,
			 -5.4395,4.9284,-10.965,-2.4136,0.88519,-0.58848,0.18377,0.10907,-6.7777,-16.041,67.188,15.517,2.998,-0.46719,-0.33769,-2.359,4.7176,9.9742,
			  0.032969,0.0076627,0.076285,0.0028915,-0.017347,-0.035597,-0.02091,0.0017461,0.059051,0.000978,-0.51961,-0.11709,-0.19398,0.16384,0.0119,0.015356,0.028956,-0.079161,
			 -0.037624,-0.022814,0.0037606,-0.0018514,-0.015376,0.02682,0.00044496,-0.021794,0.014918,0.014254,-0.16473,0.056158,0.019992,-0.20082,0.010341,-0.018219,-0.037997,0.03193,
			 -0.19136,-0.080448,1.0945,0.0050479,-1.2904,-0.049697,-0.087036,-0.0889,1.2855,0.075655,-12.004,-0.33482,-2.669,-1.9501,0.46329,-0.11434,-0.056074,-0.30977,
			  0.17328,-1.9826,1.2565,-0.081711,-0.23157,1.4456,0.05254,-0.11145,1.1418,2.687,-8.0279,0.00096509,1.9154,-3.4577,0.3441,0.15571,-2.9692,-0.13253;

		K3 << 2.0698,0.058051,-14.92,2.2977,1.1974,-1.1271,0.030437,0.56389,-22.033,8.308,151.82,-11.976,7.0389,10.107,-10.301,-0.71758,3.7318,-4.2477,
		     -5.2307,2.2988,-11.506,-2.61,1.0187,1.0153,0.44487,-0.09605,-1.6971,-14.914,103.94,25.353,9.6508,-5.9049,4.6905,-0.76645,-1.8363,17.422,
			 -0.016707,0.017108,0.061938,-0.0054079,-0.022007,-0.027559,-0.021371,0.00061262,0.071207,-0.032268,-0.72308,-0.019966,-0.14335,0.021001,0.026231,-0.010551,0.028913,-0.026728,
			 -0.026495,-0.029337,0.028816,0.0036048,-0.020801,0.022664,-0.00027843,-0.021667,0.029601,0.040535,-0.43255,-0.0089865,-0.011287,-0.11201,0.01311,-0.01646,-0.029789,-0.0094459,
			 -0.089538,-0.051731,0.87767,-0.011547,-0.8004,-0.033754,-0.057515,-0.055287,0.96296,0.082856,-11.142,-0.26106,-1.3174,-0.95265,0.34165,-0.07113,0.021822,-0.31075,
			  0.17227,-1.0838,1.0626,0.030578,-0.12964,0.78432,0.022908,-0.057742,0.74312,1.4838,-10.533,-0.94076,0.31432,-1.1737,0.06081,0.044582,-1.6227,-0.7375;
		
		K4 << 0.075413,-0.75561,-14.003,2.4755,0.99882,-1.0563,0.26541,0.43146,-18.451,9.5375,174.61,-17.149,2.9412,6.8636,-6.6147,-0.24189,-1.2782,-5.9323,
			  0.20169,-0.15291,-3.9786,-2.6518,0.067205,0.41756,0.022261,-0.030367,-3.5509,-6.3164,46.892,8.1468,2.4887,0.70378,-1.3895,-0.063252,-0.90389,4.2591,
			 -0.031862,0.037062,0.023462,0.0035945,-0.033013,-0.030471,-0.022599,-0.00054363,0.041662,-0.014912,-0.5563,-0.017955,-0.11611,-0.0068967,0.016318,-0.021255,0.058991,-0.017739,
			 -0.028705,-0.039034,0.059282,-0.0052633,-0.021107,0.030755,-0.00019052,-0.021808,0.09056,-0.0036379,-0.91836,0.063415,-0.0085918,-0.11972,0.03551,-0.017215,-0.047658,0.024199,
  			 -0.093282,-0.0065774,0.54995,-0.0072657,-0.5852,0.0084906,-0.043443,-0.04177,0.78985,-0.10369,-8.8916,0.24606,-0.67833,-0.74405,0.29144,-0.065617,0.056012,0.023065,
		      0.056863,-0.66686,0.32038,-0.056464,0.050904,0.57769,0.029114,-0.022607,0.35792,0.40381,-3.3065,0.18708,0.59067,-0.6163,0.13081,0.04555,-1.1085,0.076974;

		K5 << 0.032041,-0.94806,-16.051,2.6325,0.91691,-1.1093,0.40806,0.33141,-20.478,10.479,233.04,-20.795,3.8936,8.4986,-7.048,-0.30514,-0.51601,-7.3845,
			  0.7879,-0.12124,-2.6558,-3.3958,0.13801,0.69899,-0.10783,0.077578,-2.416,-8.3433,38.502,12.496,2.1348,1.2105,-1.1569,0.41121,-0.78124,5.8018,
			 -0.0402,0.046065,0.042327,-0.0024864,-0.034652,-0.033624,-0.023187,-0.00044221,0.070593,-0.031022,-0.97027,0.0093182,-0.12686,-0.030936,0.027875,-0.026916,0.079332,-0.0089871,
			 -0.033106,-0.047699,0.079861,-0.004113,-0.022621,0.03494,0.00025386,-0.022618,0.11581,-0.0077274,-1.4309,0.076115,-0.026519,-0.13709,0.040973,-0.02073,-0.060582,0.025612,
			 -0.1048,0.003487,0.58879,-0.024275,-0.45968,0.003515,-0.033321,-0.033216,0.80521,-0.16075,-10.343,0.34707,-0.62002,-0.70956,0.28699,-0.075346,0.077905,0.061776,
			  0.055499,-0.51619,0.23363,-0.011729,0.040248,0.45605,0.024906,-0.019579,0.24788,0.32396,-2.8682,0.062842,0.42798,-0.44909,0.0766,0.042272,-0.92851,0.023971;
	}
		
	public: ~vant3_adaptiveMixCtrl()
	{
	
	}
		
	public: void config()
	{
			
	}
	
	private: double xii(double u, double center, double cutoff)
	{
		double w = ( u - center ) / cutoff;
		
		if(abs(w) <= 1.0){
			return pow(e, ( -1 / ( 1 - pow(w,2) )));	
		}
		else{
			return 0.0;	
		}
	}
	private: Eigen::VectorXd TrajetoriaReferenciaCompleta(float Tempo)
	{
		Eigen::VectorXd Traj(20);


//		//Trajetória Ponto fixo
//		Traj << 0,0,0,0,     0,0,     0,      0, //uvwpqr arp alp
//				0,0,1,0,-0.079,0,0.0787, 0.0787, //xyzphithetapsi ar al
//                0,0,0,0;
		
		//Trajetória completa
		double X,Y,Z,Xp,Yp,Zp,Phi,Theta,Psi,Phip,Thetap,Psip, Ar, Al, Arp, Alp;
		Theta = 0.0806;
		Thetap = 0;
		Phi = 0;
		Phip = 0;
		Ar = -0.0804;
		Arp = 0;
		Al = -0.0804;
		Alp = 0;
		if(Tempo < 20)
		{
			X = 0;
			Y = 0;
			Z = 1+0.5*Tempo;
			Xp = 0;
			Yp = 0;
			Zp = 0.5;
			Psi = 0;
			Psip = 0;
		}
		else// if(Tempo >= 20 && Tempo < 20 + 19.2)
		{
			Tempo = Tempo - 20;    
			X = (0.7813/2)*pow(Tempo,2);
			Y = 0;
			Z = 11;
			Xp = (0.7813)*Tempo;
			if(Tempo > 19.2)
			{
				Xp = 15;
				X = 15*(Tempo-19.2) + 144.01;
			}
			Yp = 0;
			Zp = 0;
			Psi = 0;
			Psip = 0;    
		}
//		else if(Tempo >= 20 + 19.2 && Tempo < 20 + 19.2 + 45.2389)
//		{		
//			Tempo = Tempo - 20 - 19.2;
//			double T = 60.3186;
//			double RT = 144;
//			X = RT*cos(((15/144)*Tempo)-(pi/2)) + RT;
//			Y = RT*sin(((15/144)*Tempo)-(pi/2)) + RT;
//			Z = 11;// + 2*sin((2*pi/22.6195)*(Tempo));
//			Xp = -(5*RT*sin((5*Tempo)/48 - pi/2))/48;
//			Yp = (5*RT*cos((5*Tempo)/48 - pi/2))/48;
//			Zp = 0;
//			Psi = (Tempo / 45.2389) * (3/4)*2*pi;
//			Psip = (1 / 45.2389) * (3/4)*2*pi;
//		}
//		else if(Tempo >= 20 + 19.2 + 45.2389 && Tempo <= 20 + 19.2 + 45.2389 + 19.2)
//		{
//			Tempo = Tempo - 20 - 19.2 - 45.2389;    
//			X = 0;
//			Y = 144+(0.7826/2)*pow(Tempo,2) - 15*Tempo;
//			Z = 11;
//			Xp = 0;
//			Yp = -15+(0.7826)*Tempo;
//			Zp = 0;
//			Psi = (3/4)*2*pi;
//			Psip = 0;  
//		}
//		else{
//			X = 0;
//			Y = 0;
//			Z = 11;
//			Xp = 0;
//			Yp = 0;
//			Zp = 0;
//			Psi = (3/4)*2*pi;
//			Psip = 0;
//		}

		//Mapeia trajetória pro corpo
		Eigen::MatrixXd RIB(3,3);
		Eigen::MatrixXd W_n(3,3);
		Eigen::VectorXd XpYpZp(3);
		Eigen::VectorXd PhipThetapPsip(3);
		Eigen::VectorXd UVW(3);
		Eigen::VectorXd PQR(3);

		XpYpZp << Xp, Yp, Zp;
		PhipThetapPsip << Phip, Thetap, Psip;

		RIB <<  (cos(Psi)*cos(Theta)), (cos(Psi)*sin(Phi)*sin(Theta) - cos(Phi)*sin(Psi)), (sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)),
				(cos(Theta)*sin(Psi)), (cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)), (cos(Phi)*sin(Psi)*sin(Theta) - cos(Psi)*sin(Phi)), 
                        (-sin(Theta)),                              (cos(Theta)*sin(Phi)),                              (cos(Phi)*cos(Theta));
		W_n << 1,         0,          -sin(Theta), 
		       0,  cos(Phi),  cos(Theta)*sin(Phi),
               0, -sin(Phi),  cos(Phi)*cos(Theta);

		UVW = RIB.transpose() * XpYpZp;
		PQR = W_n * PhipThetapPsip;

		Traj << UVW(0), UVW(1), UVW(2),
                PQR(0), PQR(1), PQR(2),
	               Arp,    Alp, 
   		             X,      Y,      Z,
                   Phi,  Theta,    Psi,
					Ar,     Al, 
                     0,      0,      0,     0;

		return Traj;
	}
	private: Eigen::VectorXd MakeXref(Eigen::VectorXd Trajectory )
	{
		Eigen::VectorXd xRef(18);
		xRef << Trajectory(0), Trajectory(1), Trajectory(2), Trajectory(3), Trajectory(4), Trajectory(5), // uvw pqr
		Trajectory(6), Trajectory(7), //arp alp
		Trajectory(10), // z
		Trajectory(11),Trajectory(12),Trajectory(13),Trajectory(14), Trajectory(15), // phi theta psi ar al
		Trajectory(16), Trajectory(17), Trajectory(18), Trajectory(19); // integradores
		return xRef;
	}
	
	public: std::vector<double> execute(simulator_msgs::SensorArray arraymsg)
	{
		//Recebe mensagem dos sensores
		simulator_msgs::Sensor msg;
		msg = arraymsg.values.at(0);			 			
		//sequencia da mensagem: x y z phi theta psi alphar alphal xp yp zp phip thetap psip alpharp alphalp		


		//Mapeia velocidades para do corpo
		double phi = msg.values.at(3);
		double theta = msg.values.at(4);
		double psi = msg.values.at(5);

		RI_B << (cos(psi)*cos(theta)), (cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi)), (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)),
				(cos(theta)*sin(psi)), (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)), (cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi)), 
                (-sin(theta)), (cos(theta)*sin(phi)), (cos(phi)*cos(theta));
		Wn << 1,         0,          -sin(theta), 
		      0,  cos(phi),  cos(theta)*sin(phi),
              0, -sin(phi),  cos(phi)*cos(theta);
		
		Eigen::MatrixXd XpYpZp(3,1);
		Eigen::MatrixXd PhipThetapPsip(3,1);
		Eigen::MatrixXd uvw(3,1);
		Eigen::MatrixXd pqr(3,1);
		XpYpZp << msg.values.at(8), msg.values.at(9), msg.values.at(10);
		PhipThetapPsip << msg.values.at(11), msg.values.at(12), msg.values.at(13);
		uvw = RI_B.transpose()*XpYpZp;
		pqr = Wn*PhipThetapPsip;
		
		//Obtem valores de referência
		static double Tempo = 0;
		Trajectory = TrajetoriaReferenciaCompleta(Tempo);
		Tempo = Tempo + 0.012;
		Xref =  MakeXref(Trajectory);
		//Xref << 0,0,0,0,0,0,0,0,1,0,-0.079,0,0.0787, 0.0787, 0, 0, 0, 0;

		

		// constro vetor de estados sequencia: u v w p q r alpharp alphalp z phi theta psi alphar alphal intz intu intv intpsi
		X << uvw(0), uvw(1), uvw(2),
			 pqr(0), pqr(1), pqr(2),
			 msg.values.at(14),
             msg.values.at(15),
             msg.values.at(2),
             msg.values.at(3),
             msg.values.at(4), 
             msg.values.at(5), 
             msg.values.at(6),
             msg.values.at(7), 
			 0, 0, 0, 0;

		//Controlador cinemático
		Xk << Trajectory(8)-msg.values.at(0), Trajectory(9)-msg.values.at(1), 0; // x-xr, y-yr
		Uk << Trajectory(0), Trajectory(1), Trajectory(2);
		Kk << 0.2,    0,   0,
                 0, 0.05,   0,
                 0,    0, 0.2;
		Dk = Kk*RI_B.transpose()*Xk+Uk;
		Xref(0) = Dk(0);
		Xref(1) = Dk(1);
		Xref(2) = Dk(2);

		// Integrador Trapezoidal intz intu intv intpsi
		static double ubint, ub_ant = 0;
		static double vbint, vb_ant = 0;
		static double zint, z_ant = 0;
		static double yawint, yaw_ant = 0;

		double ub_atual = X(0) - Xref(0);
		ubint = ubint + (T/2)*(ub_atual + ub_ant);
		ub_ant = ub_atual;
		
		double vb_atual = X(1) - Xref(1);
		vbint = vbint + (T/2)*(vb_atual + vb_ant);
		vb_ant = vb_atual;
		
		double z_atual = X(8) - Xref(8);
		zint = zint + (T/2)*(z_atual + z_ant);
		z_ant = z_atual;
		
		double yaw_atual = X(11) - Xref(11);
		yawint = yawint + (T/2)*(yaw_atual + yaw_ant);
		yaw_ant = yaw_atual;
		//Adiciona valores integrados no vetor de estados
		X(14) = zint;
		X(15) = ubint;
		X(16) = vbint;
		X(17) = yawint;

		//Mixing adaptativo
		double ub;
	 	center << 3.5, 7.5, 10, 12.5, 14.5;
		cutoff << 3.51, 2.5, 2.0, 1.5, 1.5; 
		ub = uvw(0);

		if(ub < 0)
		{
			ub = 0;
		}
		else if(ub > 16){
			ub = 16;
		}						
		for(size_t i = 0; i < NumeroConjuntos; i++)
		{
			psii[i] = xii(ub, center[i], cutoff[i]); 
		}	
		
		for(size_t i = 0; i < NumeroConjuntos; i++)
		{
			mii[i] = psii[i]/(psii[0]+psii[1]+psii[2]+psii[3]+psii[4]);
		}	
		K = mii[0]*K1 + mii[1]*K2 + mii[2]*K3 + mii[3]*K4 + mii[4]*K5;
		//Cálculo de lei de controle				
		Erro = (X-Xref);
		Input = K*Erro + Uref;

		//Aplica a entrada de controle
		std::vector<double> out(Input.data(), Input.data() + Input.rows() * Input.cols());
		return out;
	}

	public: std::vector<double> Reference()
	{
		std::vector<double> out(Xref.data(), Xref.data() + Xref.rows() * Xref.cols());
		return out;
	}

	public: std::vector<double> Error()
	{
		std::vector<double> out(Erro.data(), Erro.data() + Erro.rows() * Erro.cols());
		return out;
	}

	public: std::vector<double> State()
	{
		std::vector<double> out(X.data(), X.data() + X.rows() * X.cols());
		return out;
	}
};


extern "C"
{
	Icontroller *create(void) {return new vant3_adaptiveMixCtrl;}
	void destroy(Icontroller *p) {delete p;}
}
