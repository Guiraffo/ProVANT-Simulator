/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the implementation of the Controller2 class.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#include "controller/controller.h"

void ControllerNode::init(int argc, char** argv)
{
  ros::init(argc, argv, "controller");
}

ControllerNode::ControllerNode()
{
  cont = 0;
  decimador = 0;
  simulator_msgs::Sensor t;
  for (int i = 0; i < 20; i++)
    data.values.push_back(t);
}

ControllerNode::~ControllerNode()
{
  outfile.endFile();
  infile.endFile();
  reffile.endFile();
  erfile.endFile();
  dlclose(teste);
}

void ControllerNode::Start()
{
  // XML FILE with the control configurations
  if (std::getenv("TILT_CONFIG") == NULL)
    exit(1);
  XMLRead doc(std::getenv("TILT_CONFIG"));  // start read XML data at the address placed in an environment variable
  docmem = doc;
  std::vector<std::string> array_sensors = doc.GetAllItems("Sensors");  // read all XML data of sensors

  // Receive Sensors order and create callback for new data
  int i = 0;
  while (i < array_sensors.size())
  {
    mapOfWords.insert(std::make_pair(array_sensors.at(i), i));                               // save order
    ros::Subscriber sub = nh.subscribe(array_sensors.at(i), 1, &ControllerNode::Sensor, this);  // create callback
    subarray.push_back(sub);                                                                 // save callback
    i++;
  }

  // Receive Actuators
  std::vector<std::string> array_actuators = doc.GetAllItems("Actuators");  // read all XML data of sensors
  i = 0;
  while (i < array_actuators.size())
  {
    ros::Publisher pub = nh.advertise<std_msgs::Float64>(array_actuators.at(i), 1);  // create puclisher
    pubarray.push_back(pub);                                                         // save publisher
    i++;
  }

  // Sensor Reader
  data.name = "allsensors";              // any name
  data.header.stamp = ros::Time::now();  // insert Date time
  data.header.frame_id = "1";            // any data

  // Control
  Step_pub = nh.advertise<std_msgs::String>(doc.GetItem("TopicoStep"),
                                            1);  // name of Topic to inform Simulator run one more step time
  Step();                                        // Step command
  configPlugin();                                // setup of dynamic library with control law
  configPrint();                                 // setup of print the information of simulation
  T = atoi(doc.GetItem("Sampletime").c_str());   // Save value of sampled time

  ros::spin();  // put node in wait
}

// Sensor Callback
void ControllerNode::Sensor(simulator_msgs::Sensor msg)
{
  mtx.lock();  // lock mutex
  cont++;
  data.values[mapOfWords[msg.name]] = msg;  // put the received data in exactly order
  if (cont == subarray.size())
  {
    // if all received all sensors of one steptime, call control function
    data.header.stamp = ros::Time::now();
    control_law(data);
    cont = 0;
  }
  mtx.unlock();
}

// Printing setup
void ControllerNode::configPrint()
{
  // get localization of output file
  if (std::getenv("TILT_MATLAB") == NULL)
  {
    std::cout << "Problemas com TILT_MATLAB" << std::endl;
    exit(1);
  }

  // start files with input, output, error and reference data of simulation
  std::string OutputPath = docmem.GetItem("Outputfile");
  OutputPath = std::getenv("TILT_MATLAB") + OutputPath;
  std::string InputPath = docmem.GetItem("InputPath");
  InputPath = std::getenv("TILT_MATLAB") + InputPath;
  std::string RefPath = docmem.GetItem("RefPath");
  RefPath = std::getenv("TILT_MATLAB") + RefPath;
  std::string ErroPath = docmem.GetItem("ErroPath");
  ErroPath = std::getenv("TILT_MATLAB") + ErroPath;
  outfile.startFile(OutputPath, "OUT");
  infile.startFile(InputPath, "IN");
  reffile.startFile(RefPath, "REF");
  erfile.startFile(ErroPath, "Erro");
}

void ControllerNode::control_law(simulator_msgs::SensorArray msg)
{
  try
  {
    // Sampling - each T step times is a new sampled time
    if (decimador % T == 0)
    {
      out.clear();
      out = controller->execute(msg);  // run control law
      outfile.printFile(out);
      std::vector<double> ref = controller->Reference();  // read current reference
      reffile.printFile(ref);                             // printing current reference
      std::vector<double> err = controller->Error();      // read current error
      erfile.printFile(err);                              // printing currente error
      std::vector<double> x = controller->State();        // read curretn State
      infile.printFile(x);                                // printing current state
      decimador = 0;
    }
    decimador++;
    // Zero order hold
    if (pubarray.size() != out.size())
    {
      // checking if number of sensors received is the same configured in XML file
      std::cout << "Value of control signals other than the xml file" << std::endl;
      exit(1);
    }
    else
    {
      int i = 0;
      // send all actuators data to simulator
      while (i < out.size())
      {
        std_msgs::Float64 msgout;
        msgout.data = out.at(i);
        pubarray.at(i).publish(msgout);
        i++;
      }
    }
    // se simulação
    Step();
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

void ControllerNode::Step()
{
  std_msgs::String msgpub;
  std::stringstream ss;
  ss << "GO";
  msgpub.data = ss.str();
  Step_pub.publish(msgpub);
}

void ControllerNode::configPlugin()
{
  destroy_t* destroy_obj = NULL;
  create_t* create_obj = NULL;

  std::string file = std::getenv("TILT_STRATEGIES") + docmem.GetItem("Strategy");
  teste = dlopen(file.c_str(), RTLD_LAZY);
  if (!teste)
  {
    std::cerr << "Cannot load library: " << dlerror() << '\n';
    exit(1);
  }
  dlerror();
  create_obj = (create_t*)dlsym(teste, "create");
  const char* dlsym_error = dlerror();
  if (dlsym_error)
  {
    std::cerr << "Cannot load symbol create: " << dlsym_error << '\n';
    exit(1);
  }
  destroy_obj = (destroy_t*)dlsym(teste, "destroy");
  dlsym_error = dlerror();
  if (dlsym_error)
  {
    std::cerr << "Cannot load symbol destroy: " << dlsym_error << '\n';
    exit(1);
  }
  controller = create_obj();
  controller->config();
}
