/*
 * Discovery.h
 *
 *  Created on: Jul 12, 2016
 *      Author: arthur
 */

#include <iostream>
#include "ros/ros.h"
#include "controller/XMLRead.h"
#include "std_msgs/String.h"
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include "simulator_msgs/Sensor.h"
#include "simulator_msgs/SensorArray.h"


#include <fstream>
#include <string>
#include <dlfcn.h>
#include "Icontroller.hpp"
#include "std_msgs/Float64.h"
#include "controller/MatlabData.h"

#include <map>



class Controller
{
	private: std::map<std::string, int> mapOfWords;


	boost::mutex mtx;
	private: ros::NodeHandle nh;
	private: std::vector<ros::Subscriber> subarray;
	private: int cont;
	private: simulator_msgs::SensorArray data;

	private: MatlabData outfile;
	private: MatlabData infile;
	private: MatlabData reffile;
	private: MatlabData erfile;
	private: std::vector<double> out;
	private: std::vector<ros::Publisher> pubarray;
	private: ros::Publisher Step_pub;
	private: ros::Subscriber X_sub;
	private:int decimador;
	private: void* teste = NULL;
	private: Icontroller* controller = NULL;
	private: int T;
	private: XMLRead docmem;
	

	public: static void init(int argc, char** argv)
	{
		ros::init(argc, argv, "discovery");
	}

	public: Controller()
	{
		cont = 0;
		decimador = 0;
		simulator_msgs::Sensor t;
		for(int i=0;i<20;i++) data.values.push_back(t);
	}
	public: void Start()
	{
		// FILE
		if(std::getenv("TILT_CONFIG")==NULL) exit(1); 
		XMLRead doc(std::getenv("TILT_CONFIG"));
		docmem = doc;
        	std::vector<std::string>  array_sensors = doc.GetAllItems("Sensors");
		
		//SENSORS
		int i = 0;
		std::cout << array_sensors.size() << std::endl;		
		while(i<array_sensors.size())
		{
			mapOfWords.insert(std::make_pair(array_sensors.at(i), i));
			ros::Subscriber sub  = nh.subscribe(array_sensors.at(i), 1, &Controller::Sensor, this);
			subarray.push_back(sub);	
			i++;
		}

		//ACTUATORS
		std::vector<std::string>  array_actuators = doc.GetAllItems("Actuators");
		std::cout << array_actuators.size() << std::endl;
		i = 0;		
		while(i<array_actuators.size())
		{
			ros::Publisher pub  = nh.advertise<std_msgs::Float64>(array_actuators.at(i), 1);
			pubarray.push_back(pub);	
			i++;
		}
			

		// Sensor Reader
		data.name = "allsensors";
		data.header.stamp = ros::Time::now();
		data.header.frame_id = "1";
		
		// Control
		Step_pub = nh.advertise<std_msgs::String>(doc.GetItem("TopicoStep"), 1);
		
		configPlugin();
		configPrint();
		T = atoi(doc.GetItem("Sampletime").c_str());
		
		ros::spin();
	}
	private: void Sensor(simulator_msgs::Sensor msg)
	{
		mtx.lock();
		cont++;
		data.values[mapOfWords[msg.name]]=msg;
		if(cont == subarray.size())
		{
			data.header.stamp = ros::Time::now();
			control_law(data);
			cont = 0;
		}
		mtx.unlock();	
	}

	void configPrint()
	{
		if(std::getenv("TILT_MATLAB")==NULL) 
		{
			std::cout << "Problemas com TILT_MATLAB" << std::endl;
			exit(1);
		}
		std::string OutputPath = docmem.GetItem("Outputfile");
		OutputPath = std::getenv("TILT_MATLAB") + OutputPath;
		std::string InputPath = docmem.GetItem("InputPath");
		InputPath = std::getenv("TILT_MATLAB") + InputPath;
		std::string RefPath = docmem.GetItem("RefPath");
		RefPath = std::getenv("TILT_MATLAB") + RefPath;		
		std::string ErroPath = docmem.GetItem("ErroPath");
		ErroPath = std::getenv("TILT_MATLAB") + ErroPath;		
		outfile.startFile(OutputPath,"OUT");
		infile.startFile(InputPath,"IN");
		reffile.startFile(RefPath,"REF");
		erfile.startFile(ErroPath,"Erro");
	}

	
	void control_law(simulator_msgs::SensorArray msg)
	{
		try
		{
			// Amostrando
			if (decimador % T == 0)
			{
				// se HIL
					// enviar dados via uart (se ainda n enviou resposta, cancelar resposta)
					// ativar thread para ler dados serial
					// se voltou e reenviou novos dados, otimo
					// se não enviar dados anteriores
				out.clear();
				out = controller->execute(msg);
				outfile.printFile(out);
				std::vector<double> ref = controller->Reference();
				reffile.printFile(ref);
				std::vector<double> err = controller->Error();
				erfile.printFile(err);
				std::vector<double> x = controller->State();
				infile.printFile(x);
				decimador = 0;
			}
			decimador++;
			//Segurador de Ordem Zero
			if(pubarray.size() != out.size())
			{
				std::cout << "Valor de sinais de controle diferente do arquivo xml" << std::endl;
				exit(1);
			}
			else
			{
				int i = 0;
				while(i < out.size())
				{
					std_msgs::Float64 msgout;
					msgout.data = out.at(i);
					pubarray.at(i).publish(msgout);
					i++;
				}
			}
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	void configPlugin()
	{
		destroy_t* destroy_obj = NULL;
		create_t* create_obj = NULL;

		std::string file = std::getenv("TILT_STRATEGIES")+docmem.GetItem("Strategy"); 
		teste = dlopen(file.c_str(), RTLD_LAZY);
		if (!teste) {
			std::cerr << "Cannot load library: " << dlerror() << '\n';
			exit(1);
		}
		dlerror();
		create_obj = (create_t*) dlsym(teste, "create");
		const char* dlsym_error = dlerror();
		if (dlsym_error) {
			std::cerr << "Cannot load symbol create: " << dlsym_error << '\n';
			exit(1);
		}
		destroy_obj = (destroy_t*) dlsym(teste, "destroy");
		dlsym_error = dlerror();
		if (dlsym_error) {
			std::cerr << "Cannot load symbol destroy: " << dlsym_error << '\n';
			exit(1);
		}
		controller = create_obj();
		controller->config();
	}

	public:~Controller()
	{
		outfile.endFile();
		infile.endFile();
		reffile.endFile();
		erfile.endFile();
		dlclose(teste);
	}
};


