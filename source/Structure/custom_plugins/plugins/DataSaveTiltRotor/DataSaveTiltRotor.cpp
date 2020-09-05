/*
 * File: QuadForces.cpp
 * Author: Jonatan Mota Campos
 * Project: ProVANT
 * Company: Federal University of Minas Gerais
 * Version: 1.0
 * Date: 10/12/19
 * Description:  This library is responsable to satured and save the control inputs for vant 4.0
 */

#include <DataSaveTiltRotor.h>

namespace gazebo
{
// constructor
DataSaveTiltRotor::DataSaveTiltRotor()
{
  Fr = 0;
  Fl = 0;
  Rotr = 0;
  Rotl = 0;
  DAr = 0;
  DAl = 0;
  DRr = 0;
  DRl = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// destructor
DataSaveTiltRotor::~DataSaveTiltRotor()
{
  FrFile.endFile();
  FlFile.endFile();
  RotrFile.endFile();
  RotlFile.endFile();
  DArFile.endFile();
  DAlFile.endFile();
  DRrFile.endFile();
  DRlFile.endFile();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// to load initial setup
void DataSaveTiltRotor::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  std::cout << "DataSavetiltRotor Inicializado" << std::endl;
  try
  {
    if (!ros::isInitialized())
    {
      std::cout << "DataSavetiltRotor not initialized!" << std::endl;
      return;
    }

    // Topics where data from the control strategy is being published
    topic_Fr = XMLRead::ReadXMLString("topic_Fr", _sdf);
    topic_Fl = XMLRead::ReadXMLString("topic_Fl", _sdf);
    topic_Rotr = XMLRead::ReadXMLString("topic_Rotr", _sdf);
    topic_Rotl = XMLRead::ReadXMLString("topic_Rotl", _sdf);
    topic_DAr = XMLRead::ReadXMLString("topic_DAr", _sdf);
    topic_DAl = XMLRead::ReadXMLString("topic_DAl", _sdf);
    topic_DRr = XMLRead::ReadXMLString("topic_DRr", _sdf);
    topic_DRl = XMLRead::ReadXMLString("topic_DRl", _sdf);

    // Values of the actuators saturation choosen in the model.sdf file
    Fr_sat = XMLRead::ReadXMLDouble("Fr_sat", _sdf);
    Fl_sat = XMLRead::ReadXMLDouble("Fl_sat", _sdf);
    Rotr_sat = XMLRead::ReadXMLDouble("Rotr_sat", _sdf);
    Rotl_sat = XMLRead::ReadXMLDouble("Rotl_sat", _sdf);
    DAr_sat = XMLRead::ReadXMLDouble("DAr_sat", _sdf);
    DAl_sat = XMLRead::ReadXMLDouble("DAl_sat", _sdf);
    DRr_sat = XMLRead::ReadXMLDouble("DRr_sat", _sdf);
    DRl_sat = XMLRead::ReadXMLDouble("DRl_sat", _sdf);

    char* tilt_matlab_env = std::getenv("TILT_MATLAB");
    std::string matlab_path;
    if (tilt_matlab_env != NULL)
    {
      matlab_path = tilt_matlab_env;
      // Ensure that the matlab path ends with a forward slash
      if (matlab_path.back() != '/')
      {
        matlab_path += "/";
      }

      // open file to save data
      std::string RelativeFileFr("Fr.txt");
      std::string ControlInput_Fr = matlab_path + RelativeFileFr;
      FrFile.startFile(ControlInput_Fr, "Fr");

      std::string RelativeFileFl("Fl.txt");
      std::string ControlInput_Fl = matlab_path + RelativeFileFl;
      FlFile.startFile(ControlInput_Fl, "Fl");

      std::string RelativeFileRotr("Rotr.txt");
      std::string ControlInput_Rotr = matlab_path + RelativeFileRotr;
      RotrFile.startFile(ControlInput_Rotr, "Rotr");

      std::string RelativeFileRotl("Rotl.txt");
      std::string ControlInput_Rotl = matlab_path + RelativeFileRotl;
      RotlFile.startFile(ControlInput_Rotl, "Rotl");

      std::string RelativeFileDAr("DAr.txt");
      std::string ControlInput_DAr = matlab_path + RelativeFileDAr;
      DArFile.startFile(ControlInput_DAr, "DAr");

      std::string RelativeFileDAl("DAl.txt");
      std::string ControlInput_DAl = matlab_path + RelativeFileDAl;
      DAlFile.startFile(ControlInput_DAl, "DAl");

      std::string RelativeFileDRr("DRr.txt");
      std::string ControlInput_DRr = matlab_path + RelativeFileDRr;
      DRrFile.startFile(ControlInput_DRr, "DRr");

      std::string RelativeFileDRl("DRl.txt");
      std::string ControlInput_DRl = matlab_path + RelativeFileDRl;
      DRlFile.startFile(ControlInput_DRl, "DRl");
    }
    else
    {
      // Logs the error and finishes execution
      ROS_FATAL_STREAM("Error while trying to read the TILT_MATLAB environment variable. The value is empty. Please "
                       "check that the TILT_MATLAB environment variable is set and that it points to a valid path and "
                       "try again. The DataSaveTiltRotor plugin cannot be loaded.");
    }

    // Topic subscribers used to perform saturation and save data coming from controllers on the Callback function
    subscriberFr_ = node_handle_.subscribe(topic_Fr, 1, &gazebo::DataSaveTiltRotor::CallbackFr, this);
    subscriberFl_ = node_handle_.subscribe(topic_Fl, 1, &gazebo::DataSaveTiltRotor::CallbackFl, this);
    subscriberRotr_ = node_handle_.subscribe(topic_Rotr, 1, &gazebo::DataSaveTiltRotor::CallbackRotr, this);
    subscriberRotl_ = node_handle_.subscribe(topic_Rotl, 1, &gazebo::DataSaveTiltRotor::CallbackRotl, this);
    subscriberDAr_ = node_handle_.subscribe(topic_DAr, 1, &gazebo::DataSaveTiltRotor::CallbackDAr, this);
    subscriberDAl_ = node_handle_.subscribe(topic_DAl, 1, &gazebo::DataSaveTiltRotor::CallbackDAl, this);
    subscriberDRr_ = node_handle_.subscribe(topic_DRr, 1, &gazebo::DataSaveTiltRotor::CallbackDRr, this);
    subscriberDRl_ = node_handle_.subscribe(topic_DRl, 1, &gazebo::DataSaveTiltRotor::CallbackDRl, this);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// when reset simulator
void DataSaveTiltRotor::Reset()
{
  try
  {
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// callback to satured and save forces from right brushless
void DataSaveTiltRotor::CallbackFr(std_msgs::Float64 msg)
{
  try
  {
    if (msg.data > Fr_sat)
    {
      msg.data = Fr_sat;
    }
    else if (msg.data < -Fr_sat)
    {
      msg.data = -Fr_sat;
    }
    Fr = msg.data;
    FrFile.printFile(Fr);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// callback to satured and save forces from left brushless
void DataSaveTiltRotor::CallbackFl(std_msgs::Float64 msg)
{
  try
  {
    if (msg.data > Fl_sat)
    {
      msg.data = Fl_sat;
    }
    else if (msg.data < -Fr_sat)
    {
      msg.data = -Fl_sat;
    }
    Fl = msg.data;
    FlFile.printFile(Fl);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// callback to satured and save forces from left brushless
void DataSaveTiltRotor::CallbackRotr(std_msgs::Float64 msg)
{
  try
  {
    if (msg.data > Rotr_sat)
    {
      msg.data = Rotr_sat;
    }
    else if (msg.data < -Rotr_sat)
    {
      msg.data = -Rotr_sat;
    }
    Rotr = msg.data;
    RotrFile.printFile(Rotr);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// callback to satured and save forces from left brushless
void DataSaveTiltRotor::CallbackRotl(std_msgs::Float64 msg)
{
  try
  {
    if (msg.data > Rotl_sat)
    {
      msg.data = Rotl_sat;
    }
    else if (msg.data < -Rotl_sat)
    {
      msg.data = -Rotl_sat;
    }
    Rotl = msg.data;
    RotlFile.printFile(Rotl);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// callback to satured and save deflection from right aileron
void DataSaveTiltRotor::CallbackDAr(std_msgs::Float64 msg)
{
  try
  {
    if (msg.data > DAr_sat)
    {
      msg.data = DAr_sat;
    }
    else if (msg.data < -DAr_sat)
    {
      msg.data = -DAr_sat;
    }
    DAr = msg.data;
    DArFile.printFile(DAr);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// callback to satured and save deflection from left aileron
void DataSaveTiltRotor::CallbackDAl(std_msgs::Float64 msg)
{
  try
  {
    if (msg.data > DAl_sat)
    {
      msg.data = DAl_sat;
    }
    else if (msg.data < -DAl_sat)
    {
      msg.data = -DAl_sat;
    }
    DAl = msg.data;
    DAlFile.printFile(DAl);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// callback to satured and save deflection from right ruddervator
void DataSaveTiltRotor::CallbackDRr(std_msgs::Float64 msg)
{
  try
  {
    if (msg.data > DRr_sat)
    {
      msg.data = DRr_sat;
    }
    else if (msg.data < -DRr_sat)
    {
      msg.data = -DRr_sat;
    }
    DRr = msg.data;
    DRrFile.printFile(DRr);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// callback to satured and save deflection from left ruddervator
void DataSaveTiltRotor::CallbackDRl(std_msgs::Float64 msg)
{
  try
  {
    if (msg.data > DRl_sat)
    {
      msg.data = DRl_sat;
    }
    else if (msg.data < -DRl_sat)
    {
      msg.data = -DRl_sat;
    }
    DRl = msg.data;
    DRlFile.printFile(DRl);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

GZ_REGISTER_MODEL_PLUGIN(DataSaveTiltRotor)
}  // namespace gazebo
