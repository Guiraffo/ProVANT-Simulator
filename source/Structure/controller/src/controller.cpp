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
  // Check the value of the TILT_CONFIG environment variable
  std::string configFilePath = std::getenv("TILT_CONFIG");
  if (configFilePath.empty())
  {
    ROS_FATAL_STREAM("The value of the TILT_CONFIG environment variable is invalid. This variable should contain the "
                     "path to the config.xml file for the desired model for this simulation, but it is empty or the "
                     "variable does not exist.");
    exit(-1);
  }

  // Create a XML file handle for the configuration file
  XMLRead xmlDoc(configFilePath);
  configFile = xmlDoc;
  /// @todo Add verification for the XML file.

  // Stores the name of the control strategy
  controlStrategy = configFile.GetItem("Strategy");

  // Read the values of the configured sensors in the config file
  std::vector<std::string> listOfSensors = xmlDoc.GetAllItems("Sensors");

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
    ros::Subscriber sub = nh.subscribe(listOfSensors.at(i), 1, &ControllerNode::stateUpdateCallback, this);
    sensorSubscribers.push_back(sub);

    // Reserve the value on the sensorData array
    sensorData.values.push_back(emptySensor);
  }

  // Read the list of actuators from the config file and setup a ROS publisher
  // for the topic relative to each actuator.
  std::vector<std::string> listOfActuators = configFile.GetAllItems("Actuators");
  for (int i = 0; i < listOfActuators.size(); i++)
  {
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
  std::string logOutputFolder = std::getenv("TILT_MATLAB");
  if (logOutputFolder.empty())
  {
    ROS_FATAL_STREAM("Error while trying to configure the simulation logging."
                     << "The current value of the TILT_MATLAB function is " << logOutputFolder
                     << ". Please correct the value of this environment variable and try again.");
    exit(-1);
  }

  if (logOutputFolder.back() != '/')
  {
    logOutputFolder += "/";
  }

  // Start the file
  openAndVerifyFile(&referenceLog, logOutputFolder, configFile.GetItem("RefPath"), "reference log");
  openAndVerifyFile(&stateLog, logOutputFolder, configFile.GetItem("InputPath"), "states log");
  openAndVerifyFile(&trackingErrorLog, logOutputFolder, configFile.GetItem("ErroPath"), "tracking error log");
  openAndVerifyFile(&controlInputsLog, logOutputFolder, configFile.GetItem("OutputFile"), "control inputs log");
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
  create_t* create_obj = NULL;

  std::string file = std::getenv("TILT_STRATEGIES") + controlStrategy;
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
  // Check if any error was returned while trying to locate the create function.
  std::string err = dlerror();
  if (!err.empty())
  {
    ROS_FATAL_STREAM("Error while trying to open the create function in the "
                     << controlStrategy << ". The control node cannot be loaded. dlerror response = " << err);
    exit(-1);
  }

  // Call the create function to create an instance of the control strategy
  // and configures the instance.
  controller = create_obj();
  controller->config();
}

void ControllerNode::startSimulation() {
  step();
}
