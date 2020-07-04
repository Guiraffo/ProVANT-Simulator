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

#include <dlfcn.h>

void ControllerNode::init(int argc, char** argv)
{
  ros::init(argc, argv, "controller");
}

ControllerNode::ControllerNode() : sensorCounter(0), stepCounter(0)
{
  ROS_DEBUG_STREAM("ControllerNode instance constructed, starting node configuration");
  setupNode();
}

ControllerNode::~ControllerNode()
{
  // Close the log files
  controlInputsLog.endFile();
  stateLog.endFile();
  referenceLog.endFile();
  trackingErrorLog.endFile();

  // Close the DLL handle
  dlclose(dllHandle);
}

void ControllerNode::setupNode()
{
  const char* TILT_CONFIG = std::getenv("TILT_CONFIG");
  ROS_DEBUG("Checking the value of TILT_CONFIG environment variable: %s", TILT_CONFIG);
  if (TILT_CONFIG == NULL)
  {
    ROS_FATAL("Error while trying to read the value of the TILT_CONFIG environment variable, this environament "
              "variable has null value or does not exist, but should point to the config.xml file of the "
              "model used for this simulation. Please correct the TILT_CONFIG value and try again.");
    exit(-1);
  }
  std::string configFilePath(TILT_CONFIG);

  ROS_DEBUG("Config file %s opened with sucess.", TILT_CONFIG);

  // Create a XML file handle for the configuration file
  XMLRead xmlDoc(configFilePath);
  configFile = xmlDoc;
  /// @todo Add verification for the XML file.

  // Stores the name of the control strategy
  controlStrategy = configFile.GetItem("Strategy");
  ROS_DEBUG_STREAM("Verifying the control strategy name: " << controlStrategy);
  if (controlStrategy.empty())
  {
    ROS_FATAL("Error while reading the control strategy for the selected model. Please check the value of the "
              "\"Strategy\" tag on the config.xml file and try again.");
    exit(-1);
  }

  // Read the values of the configured sensors in the config file
  std::vector<std::string> listOfSensors = xmlDoc.GetAllItems("Sensors");
  ROS_DEBUG("Configuring the simulation sensors");
  if (listOfSensors.empty())
  {
    ROS_FATAL("At least one sensor is necessary in order to run the simulation. Please the value of the \"Sensors\" "
              "tag in the config.xml file of the selected model and try again.");
    exit(-1);
  }

  // Subscribe to a ROS topic for each of the sensors, and configure the
  // callback for the sensor update.

  // Resize the SensorArray
  simulator_msgs::Sensor emptySensor;
  sensorData.values.resize(listOfSensors.size());

  for (int i = 0; i < listOfSensors.size(); i++)
  {
    // Store the index of the sensor in order to guarantee that the sensor
    // array passed to the control law alwas has the same order.
    sensorIndexMap.insert(std::make_pair(listOfSensors.at(i), i));
    // Subscribe to the ROS node relative to the sensor and configure the
    // sensor update callback.
    ROS_DEBUG_STREAM("Subscribing to the " << listOfSensors.at(i) << " topic.");
    ros::Subscriber sub = nh.subscribe(listOfSensors.at(i), 1, &ControllerNode::stateUpdateCallback, this);
    sensorSubscribers.push_back(sub);

    // Reserve the value on the sensorData array
    sensorData.values.push_back(emptySensor);
  }

  // Read the list of actuators from the config file and setup a ROS publisher
  // for the topic relative to each actuator.
  std::vector<std::string> listOfActuators = configFile.GetAllItems("Actuators");
  ROS_DEBUG("Configuring the actuators for the simulation");
  if (listOfActuators.empty())
  {
    ROS_FATAL("At least one actuator is necessary in order to run the simulation. Please check the value of the "
              "\"Actuators\" tag of the config.xml file of the selected model and try again.");
    exit(-1);
  }

  for (int i = 0; i < listOfActuators.size(); i++)
  {
    ROS_DEBUG_STREAM("Advertising the " << listOfActuators.at(i) << " topic");
    ros::Publisher pub = nh.advertise<std_msgs::Float64>(listOfActuators.at(i), 1);
    actuatorPublishersArray.push_back(pub);
  }

  // Setup initial data to the sensorData message
  // This message is passed to the control law execute function with the last
  // values of the sensors.
  // So far these values are randon and do not serve any function.
  /// @todo Refactor the control law instance in order to remove the need of
  /// methods provided they do not fullfill any function.
  sensorData.name = "allsensors";
  sensorData.header.stamp = ros::Time::now();
  sensorData.header.frame_id = "1";

  // Read the value of the controlLawExectuionRatio
  controlLawExecutionRatio = atoi(configFile.GetItem("Sampletime").c_str());
  ROS_DEBUG("Checking the value of the control law execution ratio: %d", controlLawExecutionRatio);
  if (controlLawExecutionRatio < 1)
  {
    ROS_FATAL_STREAM("The value for the control law exectuion ratio (" << controlLawExecutionRatio
                                                                       << ") is invalid. Please check the value of the "
                                                                          "Sampletime parameter on the config file for "
                                                                          "the model used in this simulation and try "
                                                                          "again.");
    exit(-1);
  }

  // Initialize the ROS topic to advance the simulation step.
  ROS_DEBUG("Advertising the Step topic");
  stepPublisher = nh.advertise<std_msgs::String>("Step", 1);

  // Setup the control law DLL
  setupControlStrategy();

  // Setup the simulation log files
  setupLogging();
}

// Sensor Callback
void ControllerNode::stateUpdateCallback(simulator_msgs::Sensor msg)
{
  // Lock mutex, this program is single thread, but this was maintained here
  // beacuse I was not sure what the purpose of this mutex was.
  std::lock_guard<std::mutex> lock(_mutex);

  sensorCounter++;

  // Check if this sensor exists on the map
  try
  {
    int sensorIndex = sensorIndexMap.at(msg.name);
    sensorData.values[sensorIndex] = msg;

    // Check if all of the sensor are already updated
    if (sensorCounter == sensorSubscribers.size())
    {
      // Execute the control law
      sensorData.header.stamp = ros::Time::now();
      controlLaw();

      // Reset the sensor counter
      sensorCounter = 0;
    }
  }
  // If a sensor with the msg.name is not found in the sensorIndexMap, emit
  // a warn log message reporting the error.
  catch (const std::out_of_range& e)
  {
    ROS_WARN_STREAM_ONCE("The controller node received a sensor with name " << msg.name
                                                                            << ", but a sensor with this name was not "
                                                                               "configured under the"
                                                                               " sensors list of the model "
                                                                               "config.xml.");
  }
}

/**
 * @brief Open a MatlabData file, verify if the file was correctly opened and
 * emit a fatal log message and halts execution otherwise.
 *
 * @param file The file to be started.
 * @param dir The path to the directory containing the file.
 * @param filePath The path of the file in relation with the dir path.
 * @param fileName The name of the file.
 */
void openAndVerifyFile(MatlabData* file, std::string dir, std::string filePath, std::string fileName)
{
  if (!file->startFile(dir + filePath, fileName))
  {
    ROS_FATAL_STREAM("Error while trying to create the "
                     << fileName << " log file for the simulation. Please check that the configured " << fileName
                     << " on the model config.xml file and the TILT_MATLAB environemtn variable"
                     << " both point to valid locations.");
    exit(-1);
  }
}

// Printing setup
void ControllerNode::setupLogging()
{
  const char* TILT_MATLAB = std::getenv("TILT_MATLAB");
  ROS_DEBUG("Checking the value of the TILT_MATLAB environment variable: %s", TILT_MATLAB);
  if (TILT_MATLAB == NULL)
  {
    ROS_FATAL("Error while trying to read the value of the TILT_MATLAB environment variable, this environament "
              "variable has null value or does not exist, but should point to the a valid folder to write "
              "to simulation logs. Please correct the TILT_MATLAB value and try again.");
    exit(-1);
  }

  std::string logOutputFolder(TILT_MATLAB);
  // Ensure that TILT_MATLAB ends with a /
  if (logOutputFolder.back() != '/')
  {
    logOutputFolder += "/";
  }

  // Start the file
  openAndVerifyFile(&referenceLog, logOutputFolder, configFile.GetItem("RefPath"), "reference log");
  openAndVerifyFile(&stateLog, logOutputFolder, configFile.GetItem("InputPath"), "states log");
  openAndVerifyFile(&trackingErrorLog, logOutputFolder, configFile.GetItem("ErroPath"), "tracking error log");
  openAndVerifyFile(&controlInputsLog, logOutputFolder, configFile.GetItem("Outputfile"), "control inputs log");
}

void ControllerNode::controlLaw()
{
  try
  {
    // Check if the control law should be executed at this step
    if (stepCounter % controlLawExecutionRatio == 0)
    {
      // Run the control law
      controlInputs.clear();
      controlInputs = controller->execute(sensorData);

      // Log the current control inputs values
      controlInputsLog.printFile(controlInputs);
      // Log current references values
      referenceLog.printFile(controller->Reference());
      // Log current tracking error values
      trackingErrorLog.printFile(controller->Error());
      // Log current state values
      stateLog.printFile(controller->State());

      // Reset the step counter
      stepCounter = 0;
    }
    stepCounter++;

    // Check if the returned number of control inputs is equal to the number of
    // configured actuators
    if (actuatorPublishersArray.size() != controlInputs.size())
    {
      ROS_FATAL_STREAM("The number of generated control inputs ("
                       << controlInputs.size() << ") is different from the number of configured actuators ("
                       << actuatorPublishersArray.size()
                       << "). Please verify the configuration of your model and the selected control strategy.");
      exit(-1);
    }

    // Publish a message to each actuator with the updated control inputs
    for (int i = 0; i < controlInputs.size(); i++)
    {
      std_msgs::Float64 msgout;
      msgout.data = controlInputs.at(i);
      actuatorPublishersArray.at(i).publish(msgout);
    }

    // Advance the simulation step
    step();
  }
  // In case any exception is thrown by the control strategy.
  catch (std::exception& e)
  {
    ROS_ERROR_STREAM("Exception thrown by the control strategy instance: " << e.what());
  }
}

void ControllerNode::step()
{
  std_msgs::String msgpub;
  stepPublisher.publish(msgpub);
}

void ControllerNode::setupControlStrategy()
{
  ROS_DEBUG("Initializing the control strategy DLL");
  create_t* create_obj = NULL;

  const char* TILT_STRATEGIES = std::getenv("TILT_STRATEGIES");
  ROS_DEBUG("Checking the value of TILT_STRATEGIES environment variable: %s", TILT_STRATEGIES);
  if (TILT_STRATEGIES == NULL)
  {
    ROS_FATAL("Error while trying to read the value of the TILT_STRATEGIES environment variable, this environament "
              "variable has null value or does not exist, but should point to the lib folder under "
              "your catkin workspace build destination. Please correct the TILT_STRATEGIES value and try again.");
    exit(-1);
  }

  std::string strategiesLibFolder(TILT_STRATEGIES);
  // If the TILT_STRATEGIES does not end with a /, add one
  if (strategiesLibFolder.back() != '/')
    strategiesLibFolder += "/";

  std::string file = strategiesLibFolder + controlStrategy;
  ROS_DEBUG_STREAM("Creating a handle for the " << file << " DLL");
  dllHandle = dlopen(file.c_str(), RTLD_LAZY);
  if (dllHandle == NULL)
  {
    ROS_FATAL_STREAM("Error while trying to open the " << controlStrategy << " control strategy: " << dlerror());
    exit(-1);
  }
  // Clear any existing errors
  dlerror();

  // Loads the create function from the control strategy library
  create_obj = (create_t*)dlsym(dllHandle, "create");
  ROS_DEBUG_STREAM("Creating a handle for the create function");
  // Check if any error was returned while trying to locate the create function.
  const char* err = dlerror();
  if (err != NULL)
  {
    ROS_FATAL_STREAM("Error while trying to open the create function in the "
                     << controlStrategy << ". The control node cannot be loaded. dlerror response = " << err);
    exit(-1);
  }

  // Call the create function to create an instance of the control strategy
  // and configures the instance.
  ROS_DEBUG("Initializing the control strategy instance");
  controller = create_obj();
  controller->config();
}

void ControllerNode::startSimulation()
{
  step();
}
