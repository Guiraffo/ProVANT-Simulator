/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the implementation of the ControllerNode class.
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

ControllerNode::ControllerNode(std::string configFilePath)
  : _configFilePath(configFilePath), sensorCounter(0), stepCounter(0)
{
  ROS_DEBUG_STREAM(_logMsg << "ControllerNode instance constructed, starting node configuration");
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
  // Open and parse the config.xml file
  if (!configFile.open(_configFilePath))
  {
    ROS_FATAL_STREAM(_logMsg << "Error while opening the config.xml file with path \"" << _configFilePath
                             << "\". Parsing the file resulting in the following error message: "
                             << configFile.getErrorMsg());
  }

  // Stores the name of the control strategy
  bool controlStrategyFound = false;
  controlStrategy = configFile.getControlStrategy(&controlStrategyFound);
  if (!controlStrategyFound)
  {
    ROS_FATAL_STREAM(_logMsg << "Error. The \"<Strategy>\" element was not found in the config.xml file. Please add "
                                "this tag and "
                                "set its content to the name of the desired control strategy dynamic library and try "
                                "again.");
  }

  ROS_INFO_STREAM(_logMsg << "Verifying the control strategy name: " << controlStrategy);
  if (!controlStrategyFound || controlStrategy.empty())
  {
    ROS_FATAL("%sError while reading the control strategy for the selected model. Please check the value of the "
              "\"Strategy\" tag on the config.xml file and try again.",
              _logMsg.c_str());
    exit(-1);
  }

  // Read the values of the configured sensors in the config file
  bool sensorsFound = false;
  ConfigReader::stringlist listOfSensors = configFile.getSensors(&sensorsFound);
  ROS_DEBUG("%sConfiguring the simulation sensors", _logMsg.c_str());
  if (!sensorsFound || listOfSensors.empty())
  {
    ROS_FATAL("%sAt least one sensor is necessary in order to run the simulation. Please the value of the \"Sensors\" "
              "tag in the config.xml file of the selected model and try again.",
              _logMsg.c_str());
    exit(-1);
  }

  // Subscribe to a ROS topic for each of the sensors, and configure the
  // callback for the sensor update.

  // Resize the SensorArray
  simulator_msgs::Sensor emptySensor;
  sensorData.values.resize(listOfSensors.size());

  int listPos = 0;
  for (ConfigReader::stringlist::const_iterator i = listOfSensors.cbegin(); i != listOfSensors.cend(); ++i)
  {
    // Store the index of the sensor in order to guarantee that the sensor
    // array passed to the control law alwas has the same order.
    sensorIndexMap.insert(std::make_pair(*i, listPos));
    // Subscribe to the ROS node relative to the sensor and configure the
    // sensor update callback.
    ROS_DEBUG_STREAM(_logMsg << "Subscribing to the " << *i << " topic.");
    ros::Subscriber sub = nh.subscribe(*i, 1, &ControllerNode::stateUpdateCallback, this);
    sensorSubscribers.push_back(sub);

    // Reserve the value on the sensorData array
    sensorData.values.push_back(emptySensor);

    listPos++;
  }

  // Read the list of actuators from the config file and setup a ROS publisher
  // for the topic relative to each actuator.
  bool actuatorsFound = false;
  ConfigReader::stringlist listOfActuators = configFile.getActuators(&actuatorsFound);
  ROS_DEBUG("%sConfiguring the actuators for the simulation", _logMsg.c_str());
  if (listOfActuators.empty())
  {
    ROS_FATAL("%sAt least one actuator is necessary in order to run the simulation. Please check the value of the "
              "\"Actuators\" tag of the config.xml file of the selected model and try again.",
              _logMsg.c_str());
    exit(-1);
  }

  for (ConfigReader::stringlist::const_iterator i = listOfActuators.cbegin(); i != listOfActuators.cend(); ++i)
  {
    ROS_DEBUG_STREAM(_logMsg << "Advertising the " << *i << " topic");
    ros::Publisher pub = nh.advertise<std_msgs::Float64>(*i, 1);
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
  bool sampleTimeFound = false;
  controlLawExecutionRatio = configFile.getSampleTime(&sampleTimeFound);
  ROS_DEBUG("%sChecking the value of the control law execution ratio: %d", _logMsg.c_str(), controlLawExecutionRatio);
  if (!sampleTimeFound || controlLawExecutionRatio < 1)
  {
    ROS_FATAL_STREAM(_logMsg << "The value for the control law exectuion ratio (" << controlLawExecutionRatio
                             << ") is invalid. Please check the value of the "
                                "Sampletime parameter on the config file for "
                                "the model used in this simulation and try "
                                "again.");
    exit(-1);
  }

  // Read the start paused value from the config file.
  ROS_DEBUG_STREAM(_logMsg << "Reading the start paused parameter.");
  bool startPausedFound = false;
  _startPaused = configFile.getStartPaused(&startPausedFound);
  if (!startPausedFound)
  {
    ROS_WARN_STREAM(_logMsg << "The StartPaused element was not found or has an illegal value. The simulation will "
                               "start in paused mode as is the default behavior. If this is the intended behavior, "
                               "please define the StartPaused element with a true value in the config.xml file to "
                               "silence this warning.");
  }
  if (_startPaused)
  {
    ROS_INFO_STREAM(_logMsg << "The simulation will start in paused mode.");
    autoStepEnabled = false;
  }
  else
  {
    ROS_INFO_STREAM(_logMsg << "The controller node will automatically start the simulation.");
    autoStepEnabled = true;
  }

  // Initialize the ROS topic to advance the simulation step.
  ROS_DEBUG("%sAdvertising the Step topic", _logMsg.c_str());
  if (_startPaused)
  {
    stepPublisher = nh.advertise<std_msgs::String>("Step", 1);
  }
  else
  {
    stepPublisher =
        nh.advertise<std_msgs::String>("Step", 1, std::bind(&ControllerNode::onStepSubscriberConnect, this));
  }

  // Initialize the ROS service to control the auto stepping mode.
  ROS_INFO("%sAdvertising the enable_autostepping service", _logMsg.c_str());
  autoSteppingServer = nh.advertiseService("enable_autostepping", &ControllerNode::setAutostepping, this);

  // Advertising the /provant_simulator/simulation_state topic.
  ROS_INFO_STREAM(_logMsg << "Advertisign the /provant_simulator/simulation_state topic.");
  simulationStatePublisher = nh.advertise<std_msgs::String>("provant_simulator/simulation_state", 1, true);

  // Read the simulation mode, if hil or not
  bool hilSyncFound = false;
  hilSyncFound = configFile.getHilFlagSynchronous(&hilSyncFound);
  _hilSync = configFile.getHilFlagSynchronous();

  bool hilAsyncFound = false;
  hilAsyncFound = configFile.getHilFlagSynchronous(&hilAsyncFound);
  _hilAsync = configFile.getHilFlagAsynchronous();

  if(!hilSyncFound & !hilAsyncFound){
      ROS_INFO_STREAM(_logMsg << "The simulation is not Hardware In the Loop");
  }
  else if(_hilSync){
     ROS_INFO_STREAM(_logMsg << "The simulation is Hardware In the Loop Synchrounous");
  }
  else if(_hilAsync){
     ROS_INFO_STREAM(_logMsg << "The simulation is Hardware In the Loop Asynchrounous");
  }

  // Read the simulation duration from the config file.
  ROS_DEBUG_STREAM(_logMsg << "Reading the simulation duration");
  _simulationDuration = configFile.getSimulationDuration();
  if (_simulationDuration == 0)
  {
    _closeWhenFinished = false;
    ROS_INFO_STREAM(_logMsg << "The simulation will be manually controlled");
  }
  else
  {
    ROS_INFO_STREAM(_logMsg << "The simulation will last " << _simulationDuration << " steps.");
    ROS_DEBUG_STREAM(_logMsg << "Reading the value of the ShutdownWhenFinished element.");
    bool found = false;
    _closeWhenFinished = configFile.getShutdownWhenFinished(&found);
    if (!found)
    {
      ROS_WARN_STREAM(_logMsg << "The simulation has a specified duration, but the ShutdownWhenFinished element is "
                                 "not defined. This simulation will not close when it finishes. If this is the "
                                 "intended behavior, please define this element in the config.xml file with a false "
                                 "value to silence this warning.");
    }
    if (_closeWhenFinished)
    {
      ROS_INFO_STREAM(_logMsg << "The simulation will automatically shutdown after it finishes.");
    }
  }

  // Setup the control law DLL
  setupControlStrategy();

  // Setup the simulation log files
  setupLogging();

  // Update the simulation state
  publishSimulationState("ready");
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
    ROS_WARN_STREAM_ONCE(_logMsg << "The controller node received a sensor with name " << msg.name
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
    ROS_FATAL_STREAM("[ControllerNode] "
                     << "Error while trying to create the " << fileName
                     << " log file for the simulation. Please check that the configured " << fileName
                     << " tag on the model config.xml file and the TILT_MATLAB environemtn variable"
                     << " both point to valid locations.");
    exit(-1);
  }
}

// Printing setup
void ControllerNode::setupLogging()
{
  const char* TILT_MATLAB = std::getenv("TILT_MATLAB");
  ROS_DEBUG("%sChecking the value of the TILT_MATLAB environment variable: %s", _logMsg.c_str(), TILT_MATLAB);
  if (TILT_MATLAB == NULL)
  {
    ROS_FATAL("%sError while trying to read the value of the TILT_MATLAB environment variable, this environament "
              "variable has null value or does not exist, but should point to the a valid folder to write "
              "to simulation logs. Please correct the TILT_MATLAB value and try again.",
              _logMsg.c_str());
    exit(-1);
  }

  std::string logOutputFolder(TILT_MATLAB);
  // Ensure that TILT_MATLAB ends with a /
  if (logOutputFolder.back() != '/')
  {
    logOutputFolder += "/";
  }

  // Start the file
  openAndVerifyFile(&referenceLog, logOutputFolder, configFile.getReferenceFilePath(), "reference log");
  openAndVerifyFile(&stateLog, logOutputFolder, configFile.getInputFilePath(), "states log");
  openAndVerifyFile(&trackingErrorLog, logOutputFolder, configFile.getErrorFilePath(), "tracking error log");
  openAndVerifyFile(&controlInputsLog, logOutputFolder, configFile.getOutputFilePath(), "control inputs log");
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
    _totalStepCounter++;

    // Check if the returned number of control inputs is equal to the number of
    // configured actuators
    if (actuatorPublishersArray.size() != controlInputs.size())
    {
      ROS_FATAL_STREAM(_logMsg << "The number of generated control inputs (" << controlInputs.size()
                               << ") is different from the number of configured actuators ("
                               << actuatorPublishersArray.size()
                               << "). Please verify the configuration of your model and the selected control "
                                  "strategy.");
      exit(-1);
    }

    // Publish a message to each actuator with the updated control inputs
    for (int i = 0; i < controlInputs.size(); i++)
    {
      std_msgs::Float64 msgout;
      msgout.data = controlInputs.at(i);
      actuatorPublishersArray.at(i).publish(msgout);
    }

    // Check if the simulation finished
    if (_simulationDuration != 0 && _totalStepCounter >= _simulationDuration)
    {
      ROS_INFO_STREAM(_logMsg << "The simulation finished successfully.");
      if (_closeWhenFinished)
      {
        ROS_INFO_STREAM(_logMsg << "Shuting-down the controller node.");
        shutdown();
      }
    }

    // Advance the simulation step
    if (autoStepEnabled)
    {
      if ((_simulationDuration == 0) || (_totalStepCounter < _simulationDuration))
      {
        step();
      }
    }
  }
  // In case any exception is thrown by the control strategy.
  catch (std::exception& e)
  {
    ROS_ERROR_STREAM(_logMsg << "Exception thrown by the control strategy instance: " << e.what());
  }
}

void ControllerNode::step()
{
  std_msgs::String msgpub;
  stepPublisher.publish(msgpub);
}

void ControllerNode::setupControlStrategy()
{
  ROS_DEBUG("%sInitializing the control strategy DLL", _logMsg.c_str());
  create_t* create_obj = NULL;

  const char* TILT_STRATEGIES = std::getenv("TILT_STRATEGIES");
  ROS_DEBUG("%sChecking the value of TILT_STRATEGIES environment variable: %s", _logMsg.c_str(), TILT_STRATEGIES);
  if (TILT_STRATEGIES == NULL)
  {
    ROS_FATAL("%sError while trying to read the value of the TILT_STRATEGIES environment variable, this environament "
              "variable has null value or does not exist, but should point to the lib folder under "
              "your catkin workspace build destination. Please correct the TILT_STRATEGIES value and try again.",
              _logMsg.c_str());
    exit(-1);
  }

  std::string strategiesLibFolder(TILT_STRATEGIES);
  // If the TILT_STRATEGIES does not end with a /, add one
  if (strategiesLibFolder.back() != '/')
    strategiesLibFolder += "/";

  std::string file = strategiesLibFolder + controlStrategy;
  ROS_DEBUG_STREAM(_logMsg << "Creating a handle for the " << file << " DLL");
  dllHandle = dlopen(file.c_str(), RTLD_LAZY);
  if (dllHandle == NULL)
  {
    ROS_FATAL_STREAM(_logMsg << "Error while trying to open the " << controlStrategy
                             << " control strategy: " << dlerror());
    exit(-1);
  }
  // Clear any existing errors
  dlerror();

  // Loads the create function from the control strategy library
  create_obj = (create_t*)dlsym(dllHandle, "create");
  ROS_DEBUG_STREAM(_logMsg << "Creating a handle for the create function");
  // Check if any error was returned while trying to locate the create function.
  const char* err = dlerror();
  if (err != NULL)
  {
    ROS_FATAL_STREAM(_logMsg << "Error while trying to open the create function in the " << controlStrategy
                             << ". The control node cannot be loaded. dlerror response = " << err);
    exit(-1);
  }

  // Call the create function to create an instance of the control strategy
  // and configures the instance.
  ROS_DEBUG("%sInitializing the control strategy instance", _logMsg.c_str());
  controller = create_obj();
  controller->config();
}

bool ControllerNode::setAutostepping(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  bool reqData = static_cast<bool>(req.data);
  autoStepEnabled = reqData;

  ROS_DEBUG_STREAM(_logMsg << "Setting the autoStepEnabled variable to " << std::boolalpha << reqData);

  res.success = true;
  res.message = std::string("");

  if(autoStepEnabled)
  {
    publishSimulationState("running");
  }
  else
  {
    publishSimulationState("paused");
  }

  return true;
}

void ControllerNode::shutdown()
{
  controlInputsLog.endFile();
  stateLog.endFile();
  referenceLog.endFile();
  trackingErrorLog.endFile();
  nh.shutdown();
  ros::shutdown();
}

void ControllerNode::onStepSubscriberConnect()
{
  std::unique_lock<std::mutex> _updateLock(_startedMutex);
  if (!_started)
  {
    ROS_INFO_STREAM(_logMsg << "A subscriber connected to the Step node, starting the simulation in 5s.");
    // Sleep for 5s to give gazebo a chance to finish loading.
    ros::WallDuration(5.0).sleep();
    // Start the simulation.
    _started = true;
    publishSimulationState("running");
    step();
  }
}

void ControllerNode::publishSimulationState(const std::string& state)
{
  std_msgs::String msg;
  msg.data = state;
  simulationStatePublisher.publish(msg);
}
