/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file Aerodinamica.h
 * @brief This file is responsable to implement the aerodynamic forces of Fuselage, Wings and Tail surfaces on
 * the UAV 4.0.The work was based on Daniel Neri phd thesis.
 *
 * @author Daniel Neri Cardoso
 * @author Jonatan Campos
 */

#include "Aerodinamica.h"

#include "XMLRead.h"

#include <cstdlib>

#define PI 3.1415926536

namespace gazebo
{
Aerodinamica::Aerodinamica()
  : RBFw(3, 3)
  , RBwRw(3, 3)
  , RBwLw(3, 3)
  , RBtRw(3, 3)
  , RBtLw(3, 3)
  , RBcGw(3, 3)
  , DARPRNr(3)
  , DALPLNr(3)
  , DARPR(3)
  , DALPL(3)
  , DBAR(3)
  , DBAL(3)
  , PosCG(3)
  , DBf(3)
  , DBwr(3)
  , DBwl(3)
  , DBtr(3)
  , DBtl(3)
  , PosCGNr(3)
  , DBfNr(3)
  , DBwrNr(3)
  , DBwlNr(3)
  , DBtrNr(3)
  , DBtlNr(3)
  , RI_B(3, 3)
  , Wn(3, 3)
  , WI_IB(3)
  , PhipThetapPsip(3)
  , XpYpZp(3)
  , EnvironmentWindI(3)
  , dPI_CG(3)
  , dPI_f(3)
  , dPI_wr(3)
  , dPI_wl(3)
  , dPI_tr(3)
  , dPI_tl(3)
  , UVWCG(3)
  , UVWf(3)
  , UVWwr(3)
  , UVWwl(3)
  , UVWtr(3)
  , UVWtl(3)
  , RAlphaf(3, 3)
  , RBetaf(3, 3)
  , Fxz(3)
  , Fxy(3)
  , Forca_F(3)
  , FWrxz(3)
  , FWlxz(3)
  , Forca_Wr(3)
  , Forca_Wl(3)
  , RAlphawr(3, 3)
  , RAlphawl(3, 3)
  , RGammatr(3, 3)
  , RGammatl(3, 3)
  , Forca_Tr(3)
  , Forca_Tl(3)
  , F_TailR(3)
  , F_TailL(3)
  , Ad(8, 8)
  , Bd(8, 3)
  , Cd(3, 8)
  , Uk(3)
  , Dk1(8)
  , Dk(8)
{
  // srand(0); //initialize random variable
  rho = 1.21;       // air density
  sw = 0.25 / 2.0;  // wing surface area
  T = 0.004;        // execution period (related to the simulator execution time)
  pi = 3.141592653589793;
  DiameterPropeller = 24.0 * (0.0254);  // converting from inches to meters
  WingSpan = 1.7835;
  MeanGeometricChord = 0.1366;
  B = 0.087266462599716;  // propeller's inclination angle
  // Environment wind-speed w.r.t the Inertial Frame
  EnvironmentWindI << 0.0, 0.0, 0.0;  // initialize environment wind
  Time = 0;                           // time variable

  // Initial states for the Von Karman Turbulence model
  Dk << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // Position of the Aerodinamic centers w.r.t the Body frame expressed in Body frame
  PosCG << -0.60, 0.0, 0.0;          // position center of gravity
  DBAR << -0.62, -1.025, 0.172;      // position of auxiliary frame with respect to body
  DBAL << -0.62, 1.025, 0.172;       // position of auxiliary frame with respect to body
  DBf << -0.454, 0.0, 0.0;           // Position aerodynamic center of fuselage
  DBwr << -0.644, -0.465, 0.1420;    // Position aerodynamic center of wing R
  DBwl << -0.644, 0.465, 0.1420;     // Position aerodynamic center of wing L
  DBtr << -1.1323, -0.2952, 0.2332;  // Position aerodynamic center of tail R
  DBtl << -1.1323, 0.2952, 0.2332;   // Position aerodynamic center of tail L
  DARPR << 0.0, 0.0, 0.1;            // Position propeller w.r.t the auxiliary frame
  DALPL << 0.0, 0.0, 0.1;            // Position propeller w.r.t the auxiliary frame

  Eigen::VectorXd PosCGGroupOfThrusters(3);
  PosCGGroupOfThrusters << 0.0, 0.0, 0.07;

  // Normalized positions to use in gazebo functions (position w.r.t. CG)
  PosCGNr = PosCG - PosCG;
  DBfNr = DBf - PosCG;
  DBwrNr = DBwr - PosCG;
  DBwlNr = DBwl - PosCG;
  DBtrNr = DBtr - PosCG;
  DBtlNr = DBtl - PosCG;
  DARPRNr = DARPR - PosCGGroupOfThrusters;
  DALPLNr = DALPL - PosCGGroupOfThrusters;

  // Von Karman Disturbance model matrices

  Ad << -17.4149, -32.7252, 0, 0, 0, 0, 0, 0, 1.0000, 0, 0, 0, 0, 0, 0, 0, 0, 0, -16.3654, -31.6442, -13.4677, 0, 0, 0,
      0, 0, 1.0000, 0, 0, 0, 0, 0, 0, 0, 0, 1.0000, 0, 0, 0, 0, 0, 0, 0, 0, 0, -16.3654, -31.6442, -13.4677, 0, 0, 0, 0,
      0, 1.0000, 0, 0, 0, 0, 0, 0, 0, 0, 1.0000, 0;

  Bd << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;

  Cd << 8.0153, 81.7565, 0, 0, 0, 0, 0, 0, 0, 0, 7.0329, 72.5115, 33.6459, 0, 0, 0, 0, 0, 0, 0, 0, 4.2197, 43.5069,
      20.1875;

  // Initialize file to save wind data

  std::string relativeFile("Wind.txt");
  const char* tilt_matlab_env = std::getenv("TILT_MATLAB");
  std::string tiltMatlabPath;
  if (tilt_matlab_env != NULL)
  {
    tiltMatlabPath = std::string(tilt_matlab_env);
    if (tiltMatlabPath.back() != '/')
    {
      tiltMatlabPath += "/";
    }
  }
  else
  {
    ROS_FATAL_STREAM("Error, the value of the TILT_MATLAB environment var is empty or this environment variable does "
                     "not exist. The aerodinamica2 plugin cannot continue loading.");
  }

  std::string file = tiltMatlabPath + relativeFile;

  Wind.startFile(file);
}

Aerodinamica::~Aerodinamica()
{
  Wind.endFile();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double Aerodinamica::AWGN_generator()
{
  /* Generates additive white Gaussian Noise samples with zero mean and a standard deviation of 1. */

  double temp1;
  double temp2;
  double result;
  int p;

  p = 1;

  while (p > 0)
  {
    temp2 = (rand() / ((double)RAND_MAX)); /*  rand() function generates an
                                               integer between 0 and  RAND_MAX,
                                               which is defined in stdlib.h.
                                           */

    if (temp2 == 0)
    {  // temp2 is >= (RAND_MAX / 2)
      p = 1;
    }  // end if
    else
    {  // temp2 is < (RAND_MAX / 2)
      p = -1;
    }  // end else

  }  // end while()

  temp1 = cos((2.0 * (double)PI) * rand() / ((double)RAND_MAX));
  result = sqrt(-2.0 * log(temp2)) * temp1;

  return result;  // return the generated random sample to the caller

}  // end AWGN_generator()

void Aerodinamica::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  std::cout << "Forcas Aerodinamicas Inicializadas" << std::endl;
  try
  {
    if (!ros::isInitialized())
    {
      std::cout << "Aerodinamica nao inicializado!" << std::endl;
      return;
    }

    topic_Fr = XMLRead::ReadXMLString("topic_Fr", _sdf);
    topic_Fl = XMLRead::ReadXMLString("topic_Fl", _sdf);

    NameOfLinkMainBody_ = XMLRead::ReadXMLString("MainBody", _sdf);
    NameOfLinkFr_ = XMLRead::ReadXMLString("LinkFr", _sdf);
    NameOfLinkFl_ = XMLRead::ReadXMLString("LinkFl", _sdf);

    // modificado para melhor desempenho de aplicação de força dos propellers
    NameOfJointL_ = XMLRead::ReadXMLString("NameOfJointL", _sdf);
    juntaL = _model->GetJoint(NameOfJointL_);
    NameOfJointR_ = XMLRead::ReadXMLString("NameOfJointR", _sdf);
    juntaR = _model->GetJoint(NameOfJointR_);

    NameOfTopicAileronR_ = XMLRead::ReadXMLString("TopicAileronR", _sdf);
    NameOfTopicAileronL_ = XMLRead::ReadXMLString("TopicAileronL", _sdf);
    NameOfTopicElevator_ = XMLRead::ReadXMLString("TopicElevator", _sdf);
    NameOfTopicRudder_ = XMLRead::ReadXMLString("TopicRudder", _sdf);

    world = _model->GetWorld();
    MainBody = _model->GetLink(NameOfLinkMainBody_);
    linkFr = _model->GetLink(NameOfLinkFr_);
    linkFl = _model->GetLink(NameOfLinkFl_);
    link = _model->GetLink(NameOfLinkMainBody_);

    updateConnection =
        event::Events::ConnectWorldUpdateBegin([this](const common::UpdateInfo& /*info*/) { this->Update(); });

    // subscribers
    motor_subscriberFR_ = node_handle_.subscribe(topic_Fr, 1, &gazebo::Aerodinamica::CallbackFR, this);
    motor_subscriberFL_ = node_handle_.subscribe(topic_Fl, 1, &gazebo::Aerodinamica::CallbackFL, this);
    motor_subscriberFAileronR_ =
        node_handle_.subscribe(NameOfTopicAileronR_, 1, &gazebo::Aerodinamica::CallbackFAileronR, this);
    motor_subscriberFAileronL_ =
        node_handle_.subscribe(NameOfTopicAileronL_, 1, &gazebo::Aerodinamica::CallbackFAileronL, this);
    motor_subscriberRudderR_ =
        node_handle_.subscribe(NameOfTopicElevator_, 1, &gazebo::Aerodinamica::CallbackElevator, this);
    motor_subscriberRudderL_ =
        node_handle_.subscribe(NameOfTopicRudder_, 1, &gazebo::Aerodinamica::CallbackRudder, this);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

void Aerodinamica::CallbackFR(std_msgs::Float64 msg)
{
  // This function computes the force and torque generated by the propellers right
  // The forces and torques are computed according to the Propulse model proposed by Sergio Esteban from University of
  // Seville

  if (msg.data < 1)
  {
    msg.data = 1;
  }
  else if (msg.data > 100)
  {
    msg.data = 100;
  }

  double ForceR, TorqueR, J, phi;
  Eigen::MatrixXd RIB(3, 3), Wn(3, 3), RBR(3, 3);
  ignition::math::Pose3d pose = link->WorldPose();
  ignition::math::Vector3d angular = link->WorldAngularVel();
  ignition::math::Vector3d linear = link->WorldLinearVel();

  Eigen::VectorXd ay(3), dPIpR(3), PhipThetapPsip(3), XpYpZp(3), UVWpR(3), WIIB(3), vetForceR(3), vetForceRI(3);

  double AlphaR = juntaR->Position(0);
  double Phi = pose.Rot().Euler().X();
  double Theta = pose.Rot().Euler().Y();
  double Psi = pose.Rot().Euler().Z();

  RIB = Rotz(Psi) * Roty(Theta) * Rotx(Phi);
  RBR = Roty(AlphaR) * Rotx(-B);
  Wn << 1, 0, -sin(Theta), 0, cos(Phi), sin(Phi) * cos(Theta), 0, -sin(Phi), cos(Phi) * cos(Theta);

  WIIB << angular.X(), angular.Y(), angular.Z();
  PhipThetapPsip = Wn.inverse() * RIB.transpose() * WIIB;

  XpYpZp << linear.X(), linear.Y(), linear.Z();

  ay << 0, 1, 0;
  dPIpR = -RIB * RBR * SkewSymmetricMatrix(DARPR) * Rotx(-B).transpose() * ay * AlphaR -
          RIB * SkewSymmetricMatrix(DBAR + RBR * DARPR) * Wn * PhipThetapPsip + XpYpZp;

  UVWpR = (RBR.transpose() * RIB.transpose()) * (dPIpR - EnvironmentWindI);
  double VpRxz = pow(pow(UVWpR(2), 2) + pow(UVWpR(0), 2), 0.5);  // problema aqui
  double AlphapR = atan2(UVWpR(0), UVWpR(2));

  J = VpRxz / (DiameterPropeller * msg.data);
  phi = AlphapR;
  ForceR = rho * pow(DiameterPropeller, 4) * pow(msg.data, 2) * GetCt(J, phi);
  TorqueR = rho * pow(DiameterPropeller, 5) * pow(msg.data, 2) * GetCq(J, phi);
  vetForceR << 0, 0, ForceR;
  vetForceRI = RIB * RBR * vetForceR;

  try
  {
    ignition::math::Vector3d forceR(vetForceRI(0), vetForceRI(1), vetForceRI(2));
    ignition::math::Vector3d PositionR(DARPRNr(0), DARPRNr(1), DARPRNr(2));
    linkFr->AddForceAtRelativePosition(forceR, PositionR);
    ignition::math::Vector3d torqueR(0, 0, TorqueR);  // drag torque
    linkFr->AddRelativeTorque(torqueR);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}
void Aerodinamica::CallbackFL(std_msgs::Float64 msg)
{
  // This function computes the force and torque generated by the propellers left
  // The forces and torques are computed according to the propulsive model proposed by Sergio Esteban from University of
  // Seville

  if (msg.data < 1)
  {
    msg.data = 1;
  }
  else if (msg.data > 100)
  {
    msg.data = 100;
  }

  double ForceL, TorqueL, J, phi;
  Eigen::MatrixXd RIB(3, 3), Wn(3, 3), RBL(3, 3);
  ignition::math::Pose3d pose = link->WorldPose();
  ignition::math::Vector3d angular = link->WorldAngularVel();
  ignition::math::Vector3d linear = link->WorldLinearVel();

  Eigen::VectorXd ay(3), dPIpL(3), PhipThetapPsip(3), XpYpZp(3), UVWpL(3), WIIB(3), vetForceL(3), vetForceLI(3);
  ;

  double AlphaL = juntaL->Position(0);
  double Phi = pose.Rot().Euler().X();
  double Theta = pose.Rot().Euler().Y();
  double Psi = pose.Rot().Euler().Z();

  RIB = Rotz(Psi) * Roty(Theta) * Rotx(Phi);
  RBL = Roty(AlphaL) * Rotx(B);

  Wn << 1, 0, -sin(Theta), 0, cos(Phi), sin(Phi) * cos(Theta), 0, -sin(Phi), cos(Phi) * cos(Theta);

  WIIB << angular.X(), angular.Y(), angular.Z();
  PhipThetapPsip = Wn.inverse() * RIB.transpose() * WIIB;

  XpYpZp << linear.X(), linear.Y(), linear.Z();

  ay << 0, 1, 0;
  dPIpL = -RIB * RBL * SkewSymmetricMatrix(DALPL) * Rotx(B).transpose() * ay * AlphaL -
          RIB * SkewSymmetricMatrix(DBAL + RBL * DALPL) * Wn * PhipThetapPsip + XpYpZp;

  UVWpL = RBL.transpose() * RIB.transpose() * (dPIpL - EnvironmentWindI);
  double VpLxz = pow(pow(UVWpL(2), 2) + pow(UVWpL(0), 2), 0.5);
  double AlphapL = atan2(UVWpL(0), UVWpL(2));

  J = VpLxz / (DiameterPropeller * msg.data);
  phi = AlphapL;

  ForceL = rho * pow(DiameterPropeller, 4) * pow(msg.data, 2) * GetCt(J, phi);
  TorqueL = -rho * pow(DiameterPropeller, 5) * pow(msg.data, 2) * GetCq(J, phi);
  vetForceL << 0, 0, ForceL;
  vetForceLI = RIB * RBL * vetForceL;
  try
  {
    ignition::math::Vector3d forceL(vetForceLI(0), vetForceLI(1), vetForceLI(2));
    ignition::math::Vector3d PositionL(DALPLNr(0), DALPLNr(1), DALPLNr(2));
    linkFl->AddForceAtRelativePosition(forceL, PositionL);
    ignition::math::Vector3d torqueL(0, 0, TorqueL);  // drag torque
    linkFl->AddRelativeTorque(torqueL);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

void Aerodinamica::CallbackFAileronR(std_msgs::Float64 DaR)
{
  try
  {
    AileronRDeflection = DaR.data;
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

void Aerodinamica::CallbackFAileronL(std_msgs::Float64 DaL)
{
  try
  {
    AileronLDeflection = DaL.data;
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}
void Aerodinamica::CallbackElevator(std_msgs::Float64 De)
{
  try
  {
    ElevatorDeflection = De.data;
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}
void Aerodinamica::CallbackRudder(std_msgs::Float64 Dr)
{
  try
  {
    RudderDeflection = Dr.data;
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::VectorXd DisturbanceHovering(double Tempo)
{
  // This function is used to the application of external forces disturbances in the helicopter-flight mode
  // It receives the instant of time and returns a force vector to be applied at the Tilt-rotor UAV
  Eigen::VectorXd DisturbanceI(3);
  double ForceX, ForceY, ForceZ;

  ForceX = 0;

  if (Tempo >= 20 && Tempo <= 30)
  {
    ForceY = -15;
  }
  else
  {
    ForceY = 0;
  }

  if (Tempo >= 40 && Tempo <= 50)
  {
    ForceZ = -15;
  }
  else
  {
    ForceZ = 0;
  }

  DisturbanceI << ForceX, ForceY, ForceZ;

  return DisturbanceI;
}

Eigen::VectorXd DisturbanceForwardCircularDeceleration(double Tempo)
{
  // This function is used to the application of external forces disturbances in the full-flight envelope mode
  // It receives the instant of time and returns a force vector to be applied at the Tilt-rotor UAV

  Eigen::VectorXd DisturbanceI(3);
  double ForceX, ForceY, ForceZ;

  ForceX = 0;

  if (Tempo >= 10 && Tempo <= 20)
  {
    ForceY = 10;
  }
  if (Tempo >= 120 && Tempo <= 130)
  {
    ForceY = -10;
  }
  else
  {
    ForceY = 0;
  }

  if (Tempo >= 150 && Tempo <= 160)
  {
    ForceZ = -10;
  }
  else if (Tempo >= 200 && Tempo <= 210)
  {
    ForceZ = 10;
  }
  else
  {
    ForceZ = 0;
  }

  DisturbanceI << ForceX, ForceY, ForceZ;

  return DisturbanceI;
}

Eigen::VectorXd GetEnvironmentWindI(double Tempo)
{
  // This function generates the environment wind-speed
  // It receives the instant of time and returns a environment wind vector, which is given w.r.t the Inertial frame

  Eigen::VectorXd EnvironmentWindI(3);
  double WindX, WindY, WindZ;

  WindX = -3;

  if (Tempo >= 20 && Tempo <= 23)
  {
    WindY = 5;
  }
  else if (Tempo >= 100 && Tempo <= 103)
  {
    WindY = 0;
  }
  else if (Tempo >= 180 && Tempo <= 183)
  {
    WindY = 5;
  }
  else if (Tempo >= 220 && Tempo <= 223)
  {
    WindY = -5;
  }
  else
  {
    WindY = 0;
  }

  if (Tempo >= 50 && Tempo <= 53)
  {
    WindZ = -3;
  }
  else if (Tempo >= 150 && Tempo <= 153)
  {
    WindZ = 0;
  }
  else if (Tempo >= 200 && Tempo <= 203)
  {
    WindZ = 3;
  }
  else
  {
    WindZ = 0;
  }

  EnvironmentWindI << WindX, WindY, WindZ;

  return EnvironmentWindI;
}

Eigen::VectorXd Aerodinamica::VonKarmanTurbulenceModel()
{
  // This function executes the Von Karman Turbulence Model

  Eigen::VectorXd EnvironmentWindI(3);

  Uk << 0.02 * AWGN_generator(), 0.04 * AWGN_generator(), 0.04 * AWGN_generator();
  Dk1 = (Ad * T + Eigen::MatrixXd::Identity(8, 8)) * Dk + Bd * Uk;
  Dk = Dk1;

  EnvironmentWindI << Cd * Dk;
  EnvironmentWindI(0) = EnvironmentWindI(0) - 3;

  return EnvironmentWindI;
}

void Aerodinamica::Update()
{
  Time = Time + T;              // Compute the instant of time
  EnvironmentWindI << 0, 0, 0;  // compute the environment wind with respect to the Inertial frame

  // Examples of Environment wind
  // EnvironmentWindI << 0, 3, 0;
  // EnvironmentWindI = GetEnvironmentWindI(Time);
  // EnvironmentWindI = VonKarmanTurbulenceModel();s

  // Save environment wind-speed data in the wind file
  std::vector<double> DataWind;
  DataWind.push_back(EnvironmentWindI(0));
  DataWind.push_back(EnvironmentWindI(1));
  DataWind.push_back(EnvironmentWindI(2));
  static int Contador = 0;
  if (Contador % 3 == 0)
  {
    Wind.printFile(DataWind);
  }

  try
  {
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Compute the relative wind-speed//
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Computing the wind properties
    ignition::math::Vector3d Linear = link->WorldLinearVel();
    ignition::math::Vector3d Angular = link->WorldAngularVel();
    ignition::math::Pose3d pose = link->WorldPose();

    // computing configuration variables
    Phi = pose.Rot().Euler().X();
    Theta = pose.Rot().Euler().Y();
    Psi = pose.Rot().Euler().Z();

    // computing transformation matrices
    RI_B << Rotz(Psi) * Roty(Theta) * Rotx(Phi);

    Wn << 1.0, 0.0, -sin(Theta), 0.0, cos(Phi), cos(Theta) * sin(Phi), 0.0, -sin(Phi), cos(Phi) * cos(Theta);

    //-----------Computing [phidot thetadot psidot]-----------------------%
    WI_IB << Angular.X(), Angular.Y(), Angular.Z();
    PhipThetapPsip = Wn.inverse() * RI_B.transpose() * WI_IB;

    //-----------Computing [Xdot Ydot Zdot]-------------------------------%
    XpYpZp << Linear.X(), Linear.Y(), Linear.Z();

    // Compute the velocity of the aerodynamic centers expressed in the Inertial frame
    dPI_CG << -RI_B * SkewSymmetricMatrix(PosCG) * Wn * PhipThetapPsip +
                  XpYpZp;  // velocity of the aerodynamic center of fuselage w.r.t I expressed in I
    dPI_f << -RI_B * SkewSymmetricMatrix(DBf) * Wn * PhipThetapPsip +
                 XpYpZp;  // velocity of the aerodynamic center of fuselage w.r.t I expressed in I
    dPI_wr << -RI_B * SkewSymmetricMatrix(DBwr) * Wn * PhipThetapPsip +
                  XpYpZp;  // velocity of the aerodynamic center of wing right w.r.t I expressed in I
    dPI_wl << -RI_B * SkewSymmetricMatrix(DBwl) * Wn * PhipThetapPsip +
                  XpYpZp;  // velocity of the aerodynamic center of wing left w.r.t I expressed in I
    dPI_tr << -RI_B * SkewSymmetricMatrix(DBtr) * Wn * PhipThetapPsip +
                  XpYpZp;  // velocity of the aerodynamic center of tail right w.r.t I expressed in I
    dPI_tl << -RI_B * SkewSymmetricMatrix(DBtl) * Wn * PhipThetapPsip +
                  XpYpZp;  // velocity of the aerodynamic center of tail left w.r.t I expressed in I

    //----------Computing Properties of Relative wind for center of gravity---------%

    UVWCG = RI_B.transpose() * (dPI_CG - EnvironmentWindI);  // Express the relative airspeed velocity on the frame
                                                             // positioned at the aerodynamic center of fuselage

    double VCG = pow(pow(UVWCG(2), 2) + pow(UVWCG(1), 2) + pow(UVWCG(0), 2), 0.5);  // Magnitude x-z axis

    double AlphaCG = atan2(UVWCG(2), UVWCG(0));  // Orientation - Angle of attack CG
    double BetaCG = asin2(UVWCG(1), VCG);        // Orientation - Side slip angle CG

    //----------Computing Properties of Relative wind for fuselage---------%

    UVWf = RI_B.transpose() * (dPI_f - EnvironmentWindI);  // Express the relative airspeed velocity on the frame
                                                           // positioned at the aerodynamic center of fuselage

    double Vf = pow(pow(UVWf(2), 2) + pow(UVWf(1), 2) + pow(UVWf(0), 2), 0.5);  // Magnitude x-z axis

    double Alphaf = atan2(UVWf(2), UVWf(0));  // Orientation - Angle of attack fuselage
    double Betaf = asin2(UVWf(1), Vf);        // Orientation - Side slip angle fuselage

    //----------Computing Properties of Relative wind for wings---------%

    // Wing right
    UVWwr = RI_B.transpose() * (dPI_wr - EnvironmentWindI);  // Express the velocity on the frame positioned at the
                                                             // aerodynamic center of wing R

    double Vwr = pow(pow(UVWwr(2), 2) + pow(UVWwr(1), 2) + pow(UVWwr(0), 2), 0.5);  // compute Magnitude
    double Alphawr = atan2(UVWwr(2), UVWwr(0));                                     // compute Orientation
    double Betawr = asin2(UVWwr(1), Vwr);

    // Wing left
    UVWwl = RI_B.transpose() * (dPI_wl - EnvironmentWindI);  // Express the velocity on the frame positioned at the
                                                             // aerodynamic center of wing L

    double Vwl = pow(pow(UVWwl(2), 2) + pow(UVWwl(1), 2) + pow(UVWwl(0), 2), 0.5);  // compute Magnitude
    double Alphawl = atan2(UVWwl(2), UVWwl(0));                                     // compute Orientation
    double Betawl = asin2(UVWwl(1), Vwl);

    //			//----------Computing Properties of Relative wind for V-tail---------%

    // Tail right
    UVWtr = RI_B.transpose() * (dPI_tr - EnvironmentWindI);  // Express the velocity on the frame positioned at the
                                                             // aerodynamic center of Tail R

    double Vtr = pow(pow(UVWtr(2), 2) + pow(UVWtr(1), 2) + pow(UVWtr(0), 2), 0.5);  // compute Magnitude
    double Alphatr = atan2(UVWtr(2), UVWtr(0));                                     // compute Orientation
    double Betatr = asin2(UVWtr(1), Vtr);
    // Tail left
    UVWtl = RI_B.transpose() * (dPI_tl - EnvironmentWindI);  // Express the velocity on the frame positioned at the
                                                             // aerodynamic center of Tail L

    double Vtl = pow(pow(UVWtl(2), 2) + pow(UVWtl(1), 2) + pow(UVWtl(0), 2), 0.5);  // compute Magnitude
    double Alphatl = atan2(UVWtl(2), UVWtl(0));                                     // compute Orientation
    double Betatl = asin2(UVWtl(1), Vtl);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Compute aerodynamic coefficients//
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    Alphaf = Saturation(-Alphaf, -1.55, 1.55);
    Betaf = Saturation(-Betaf, -1.55, 1.55);
    Alphawr = Saturation(-Alphawr, -1.55, 1.55);
    Betawr = Saturation(-Betawr, -1.55, 1.55);
    Alphawl = Saturation(-Alphawl, -1.55, 1.55);
    Betawl = Saturation(-Betawl, -1.55, 1.55);
    Alphatr = Saturation(-Alphatr, -1.55, 1.55);
    Betatr = Saturation(-Betatr, -1.55, 1.55);
    Alphatl = Saturation(-Alphatl, -1.55, 1.55);
    Betatl = Saturation(-Betatl, -1.55, 1.55);
    AlphaCG = Saturation(-AlphaCG, -1.55, 1.55);
    BetaCG = Saturation(-BetaCG, -1.55, 1.55);

    // The vector of aerodynamic coefficients are given in the range -90 to 90 deg with data for each 0.008726646259972
    // rad/s

    double DeltaCoefRad = 0.008726646259972;

    // calculo dos coef. da fuselagem
    double Vala = ((Alphaf + pi / 2.0) / DeltaCoefRad);
    int Indexa = floor(Vala);
    double Proporcaoa = Vala - Indexa;

    double Vals = ((Betaf + pi / 2.0) / DeltaCoefRad);
    int Indexs = floor(Vals);
    double Proporcaos = Vals - Indexs;

    double CDfxz = VetCDfxz[Indexa] + Proporcaoa * (VetCDfxz[Indexa + 1] - VetCDfxz[Indexa]);
    double CLfxz = VetCLf[Indexa] + Proporcaoa * (VetCLf[Indexa + 1] - VetCLf[Indexa]);
    double CMfxz = VetCMf[Indexa] + Proporcaoa * (VetCMf[Indexa + 1] - VetCMf[Indexa]);

    double CDfxy = VetCDfxy[Indexs] + Proporcaos * (VetCDfxy[Indexs + 1] - VetCDfxy[Indexs]);
    double CYfxy = VetCYf[Indexs] + Proporcaos * (VetCYf[Indexs + 1] - VetCYf[Indexs]);
    double CNfxy = VetCNf[Indexs] + Proporcaos * (VetCNf[Indexs + 1] - VetCNf[Indexs]);

    // Interpolating data from table | Computing the right wing coeficients
    Vala = ((Alphawr + pi / 2.0) / DeltaCoefRad);
    Indexa = floor(Vala);
    Proporcaoa = Vala - Indexa;

    Vals = ((Betawr + pi / 2.0) / DeltaCoefRad);
    Indexs = floor(Vals);
    Proporcaos = Vals - Indexs;

    double CDwRxz = VetCDwxz[Indexa] + Proporcaoa * (VetCDwxz[Indexa + 1] - VetCDwxz[Indexa]);
    double CLwRxz = VetCLw[Indexa] + Proporcaoa * (VetCLw[Indexa + 1] - VetCLw[Indexa]);
    double CMwRxz = VetCMw[Indexa] + Proporcaoa * (VetCMw[Indexa + 1] - VetCMw[Indexa]);

    double CDwRxy = VetCDwxy[Indexs] + Proporcaos * (VetCDwxy[Indexs + 1] - VetCDwxy[Indexs]);
    double CYwRxy = VetCYw[Indexs] + Proporcaos * (VetCYw[Indexs + 1] - VetCYw[Indexs]);
    double CNwRxy = VetCNw[Indexs] + Proporcaos * (VetCNw[Indexs + 1] - VetCNw[Indexs]);

    // Interpolating data from table | Computing the left wing coeficients
    Vala = ((Alphawl + pi / 2.0) / DeltaCoefRad);
    Indexa = floor(Vala);
    Proporcaoa = Vala - Indexa;

    Vals = ((Betawl + pi / 2.0) / DeltaCoefRad);
    Indexs = floor(Vals);
    Proporcaos = Vals - Indexs;

    double CDwLxz = VetCDwxz[Indexa] + Proporcaoa * (VetCDwxz[Indexa + 1] - VetCDwxz[Indexa]);
    double CLwLxz = VetCLw[Indexa] + Proporcaoa * (VetCLw[Indexa + 1] - VetCLw[Indexa]);
    double CMwLxz = VetCMw[Indexa] + Proporcaoa * (VetCMw[Indexa + 1] - VetCMw[Indexa]);

    double CDwLxy = VetCDwxy[Indexs] + Proporcaos * (VetCDwxy[Indexs + 1] - VetCDwxy[Indexs]);
    double CYwLxy = VetCYw[Indexs] + Proporcaos * (VetCYw[Indexs + 1] - VetCYw[Indexs]);
    double CNwLxy = VetCNw[Indexs] + Proporcaos * (VetCNw[Indexs + 1] - VetCNw[Indexs]);

    // Interpolating data from table | Computing the right tail coeficients
    Vala = ((Alphatr + pi / 2.0) / DeltaCoefRad);
    Indexa = floor(Vala);
    Proporcaoa = Vala - Indexa;

    Vals = ((Betatr + pi / 2.0) / DeltaCoefRad);
    Indexs = floor(Vals);
    Proporcaos = Vals - Indexs;

    double CDtRxz = VetCDtxz[Indexa] + Proporcaoa * (VetCDtxz[Indexa + 1] - VetCDtxz[Indexa]);
    double CLtRxz = VetCLt[Indexa] + Proporcaoa * (VetCLt[Indexa + 1] - VetCLt[Indexa]);
    double CMtRxz = VetCMt[Indexa] + Proporcaoa * (VetCMt[Indexa + 1] - VetCMt[Indexa]);

    double CDtRxy = VetCDtxy[Indexs] + Proporcaos * (VetCDtxy[Indexs + 1] - VetCDtxy[Indexs]);
    double CYtRxy = VetCYt[Indexs] + Proporcaos * (VetCYt[Indexs + 1] - VetCYt[Indexs]);
    double CNtRxy = VetCNt[Indexs] + Proporcaos * (VetCNt[Indexs + 1] - VetCNt[Indexs]);

    // Interpolating data from table | Computing the left tail coeficients
    Vala = ((Alphatl + pi / 2.0) / DeltaCoefRad);
    Indexa = floor(Vala);
    Proporcaoa = Vala - Indexa;

    Vals = ((Betatl + pi / 2.0) / DeltaCoefRad);
    Indexs = floor(Vals);
    Proporcaos = Vals - Indexs;

    double CDtLxz = VetCDtxz[Indexa] + Proporcaoa * (VetCDtxz[Indexa + 1] - VetCDtxz[Indexa]);
    double CLtLxz = VetCLt[Indexa] + Proporcaoa * (VetCLt[Indexa + 1] - VetCLt[Indexa]);
    double CMtLxz = VetCMt[Indexa] + Proporcaoa * (VetCMt[Indexa + 1] - VetCMt[Indexa]);

    double CDtLxy = VetCDtxy[Indexs] + Proporcaos * (VetCDtxy[Indexs + 1] - VetCDtxy[Indexs]);
    double CYtLxy = VetCYt[Indexs] + Proporcaos * (VetCYt[Indexs + 1] - VetCYt[Indexs]);
    double CNtLxy = VetCNt[Indexs] + Proporcaos * (VetCNt[Indexs + 1] - VetCNt[Indexs]);

    // Interpolating data from table | Computing the interference coeficients

    Vala = ((AlphaCG + pi / 2.0) / DeltaCoefRad);
    Indexa = floor(Vala);
    Proporcaoa = Vala - Indexa;

    Vals = ((BetaCG + pi / 2.0) / DeltaCoefRad);
    Indexs = floor(Vals);
    Proporcaos = Vals - Indexs;

    double CDixz = VetCDixz[Indexa] + Proporcaoa * (VetCDixz[Indexa + 1] - VetCDixz[Indexa]);
    double CLixz = VetCLi[Indexa] + Proporcaoa * (VetCLi[Indexa + 1] - VetCLi[Indexa]);
    double CMixz = VetCMi[Indexa] + Proporcaoa * (VetCMi[Indexa + 1] - VetCMi[Indexa]);

    double CDixy = VetCDixy[Indexs] + Proporcaos * (VetCDixy[Indexs + 1] - VetCDixy[Indexs]);
    double CYixy = VetCYi[Indexs] + Proporcaos * (VetCYi[Indexs + 1] - VetCYi[Indexs]);
    double CNixy = VetCNi[Indexs] + Proporcaos * (VetCNi[Indexs + 1] - VetCNi[Indexs]);

    double Cla = 0.7104;
    double Cle = 0.3759;
    double Cyr = -0.0730;

    Alphaf = -Alphaf;
    Betaf = -Betaf;
    Alphawr = -Alphawr;
    Betawr = -Betawr;
    Alphawl = -Alphawl;
    Betawl = -Betawl;
    Alphatr = -Alphatr;
    Betatr = -Betatr;
    Alphatl = -Alphatl;
    Betatl = -Betatl;
    AlphaCG = -AlphaCG;
    BetaCG = -BetaCG;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Compute aerodynamic forces and moments//
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Rotation matrix Wind frame to aerodynamic center
    RBFw = Roty(-Alphaf) * Rotz(-Betaf);
    RBwRw = Roty(-Alphawr) * Rotz(-Betawr);
    RBwLw = Roty(-Alphawl) * Rotz(-Betawl);
    RBtRw = Roty(-Alphatr) * Rotz(-Betatr);
    RBtLw = Roty(-Alphatl) * Rotz(-Betatl);
    RBcGw = Roty(-AlphaCG) * Rotz(-BetaCG);

    // dynamic pressure
    double kf = 0.5 * rho * sw * pow(Vf, 2);
    double kwR = 0.5 * rho * sw * pow(Vwr, 2);
    double kwL = 0.5 * rho * sw * pow(Vwl, 2);
    double ktR = 0.5 * rho * sw * pow(Vtr, 2);
    double ktL = 0.5 * rho * sw * pow(Vtl, 2);
    double kcG = 0.5 * rho * sw * pow(VCG, 2);
    Eigen::VectorXd FFuselage(3), FWingR(3), FWingL(3), FVTailR(3), FVTailL(3), FInterference(3), FuselageI(3),
        WingRI(3), WingLI(3), VTailRI(3), VTailLI(3), AerodynamicMoments(3), InterferenceI(3);

    FFuselage << -kf * (CDfxy + CDfxz), kf * CYfxy, kf * CLfxz;
    FuselageI = RI_B * RBFw * FFuselage;

    FWingR << -kwR * (CDwRxy + CDwRxz), kwR * CYwRxy, kwR * (CLwRxz + Cla * AileronRDeflection);
    WingRI = RI_B * RBwRw * FWingR;

    FWingL << -kwL * (CDwLxy + CDwLxz), kwL * CYwLxy, kwL * (CLwLxz + Cla * AileronLDeflection);
    WingLI = RI_B * RBwLw * FWingL;

    FVTailR << -ktR * (CDtRxy + CDtRxz), ktR * (CYtRxy + Cyr * RudderDeflection),
        ktR * (CLtRxz + Cle * ElevatorDeflection);
    VTailRI = RI_B * RBtRw * FVTailR;

    FVTailL << -ktL * (CDtLxy + CDtLxz), ktL * (CYtLxy + Cyr * RudderDeflection),
        ktL * (CLtLxz + Cle * ElevatorDeflection);
    VTailLI = RI_B * RBtLw * FVTailL;

    AerodynamicMoments << 0,
        -(kf * CMfxz + kwR * CMwRxz + kwL * CMwLxz + ktR * CMtRxz + ktL * CMtLxz) * MeanGeometricChord,
        -(kf * CNfxy + kwR * CNwRxy + kwL * CNwLxy + ktR * CNtRxy + ktL * CNtLxy) * WingSpan;

    FInterference << -kcG * (CDixy + CDixz), kcG * CYixy, kcG * CLixz;
    InterferenceI = RI_B * RBcGw * FInterference;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Apply aerodynamic forces and moments to the Tilt-rotor UAV//
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Apply to gazebo
    MainBody->AddForceAtRelativePosition(ignition::math::Vector3d(FuselageI(0), FuselageI(1), FuselageI(2)),
                                         ignition::math::Vector3d(DBfNr(0), DBfNr(1), DBfNr(2)));
    MainBody->AddForceAtRelativePosition(ignition::math::Vector3d(WingRI(0), WingRI(1), WingRI(2)),
                                         ignition::math::Vector3d(DBwrNr(0), DBwrNr(1), DBwrNr(2)));
    MainBody->AddForceAtRelativePosition(ignition::math::Vector3d(WingLI(0), WingLI(1), WingLI(2)),
                                         ignition::math::Vector3d(DBwlNr(0), DBwlNr(1), DBwlNr(2)));
    MainBody->AddForceAtRelativePosition(ignition::math::Vector3d(VTailRI(0), VTailRI(1), VTailRI(2)),
                                         ignition::math::Vector3d(DBtrNr(0), DBtrNr(1), DBtrNr(2)));
    MainBody->AddForceAtRelativePosition(ignition::math::Vector3d(VTailLI(0), VTailLI(1), VTailLI(2)),
                                         ignition::math::Vector3d(DBtlNr(0), DBtlNr(1), DBtlNr(2)));
    MainBody->AddForceAtRelativePosition(ignition::math::Vector3d(InterferenceI(0), InterferenceI(1), InterferenceI(2)),
                                         ignition::math::Vector3d(PosCGNr(0), PosCGNr(1), PosCGNr(2)));

    ignition::math::Vector3d torque(AerodynamicMoments(0), AerodynamicMoments(1),
                                    AerodynamicMoments(2));  // drag torque
    MainBody->AddRelativeTorque(torque);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Compute aerodynamic coefficients//
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Apply Disturbance At CG
    Eigen::VectorXd DisturbanceI(3);
    DisturbanceI << 0, 0, 0;  // Compute disturbances

    // Example of external disturbancess
    // DisturbanceI << DisturbanceHovering(Time);
    // DisturbanceI << DisturbanceForwardCircularDeceleration(Time);

    // The disturbance is applied at the position of the center of gravity of the Tilt-rotor UAV
    MainBody->AddForceAtRelativePosition(ignition::math::Vector3d(DisturbanceI(0), DisturbanceI(1), DisturbanceI(2)),
                                         ignition::math::Vector3d(PosCGNr(0), PosCGNr(1), PosCGNr(2)));
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double Aerodinamica::GetCt(double J, double phi)
{
  // Compute the thrust coefficient according to the propulsive model proposed by Sergio Esteban from University of
  // Seville
  const double J_min = 0.0;
  const double J_max = 0.75;
  const double phi_min = -pi / 2.0;
  const double phi_max = pi / 2.0;
  double val;

  if (J < J_min)
  {
    J = J_min;
  }
  else if (J > J_max)
  {
    J = J_max;
  }

  if (phi < phi_min)
  {
    phi = phi_min;
  }
  else if (phi > phi_max)
  {
    phi = phi_max;
  }

  phi = abs(phi);
  Eigen::VectorXd Ct(15);
  Eigen::VectorXd Coef(15);
  Ct << 1, J, phi, pow(J, 2), J * phi, pow(phi, 2), pow(J, 3), pow(J, 2) * phi, J * pow(phi, 2), pow(phi, 3), pow(J, 4),
      pow(J, 3) * phi, pow(J, 2) * pow(phi, 2), J * pow(phi, 3), pow(phi, 4);
  Coef << 0.0829, 0.0066, 0.0, -0.2197, -0.1504, 0.0, 0.0630, 0.2064, 0.2217, 0.0, 0.0302, -0.1115, 0.0173, -0.0856,
      0.0;
  val = Ct.transpose() * Coef;
  return val;
}

double Aerodinamica::GetCq(double J, double phi)
{
  // Compute the torque coefficient according to the propulsive model proposed by Sergio Esteban from University of
  // Seville
  const double J_min = 0.0;
  const double J_max = 0.75;
  const double phi_min = -pi / 2.0;
  const double phi_max = pi / 2.0;
  double val;

  if (J < J_min)
  {
    J = J_min;
  }
  else if (J > J_max)
  {
    J = J_max;
  }

  if (phi < phi_min)
  {
    phi = phi_min;
  }
  else if (phi > phi_max)
  {
    phi = phi_max;
  }

  phi = abs(phi);
  Eigen::VectorXd Cq(15);
  Eigen::VectorXd Coef(15);

  Cq << 1, J, phi, pow(J, 2), J * phi, pow(phi, 2), pow(J, 3), pow(J, 2) * phi, J * pow(phi, 2), pow(phi, 3), pow(J, 4),
      pow(J, 3) * phi, pow(J, 2) * pow(phi, 2), J * pow(phi, 3), pow(phi, 4);
  Coef << 0.0053, 0.0074, 0.0, -0.0078, -0.0190, 0.0, -0.0215, 0.0254, 0.0183, 0.0, 0.0112, -0.0024, -0.0049, -0.0068,
      0.0;
  val = Cq.transpose() * Coef;
  return val;
}

double Aerodinamica::asin2(double x, double y)
{
  // New asin function to avoid division by zero
  if (x == 0)
  {
    return 0;
  }
  else
  {
    return asin(x / y);
  }
}

double Aerodinamica::Saturation(double Val, double Valmin, double Valmax)
{
  if (Val < Valmin)
  {
    return Valmin;
  }
  else if (Val > Valmax)
  {
    return Valmax;
  }
  else
  {
    return Val;
  }
}

Eigen::MatrixXd Aerodinamica::SkewSymmetricMatrix(Eigen::VectorXd Vector)
{
  // Place Vet in the Skew Symmetric matrix S
  Eigen::MatrixXd SkewMatrix(3, 3);
  SkewMatrix << 0, -Vector(2), Vector(1), Vector(2), 0, -Vector(0), -Vector(1), Vector(0), 0;

  return SkewMatrix;
}

Eigen::MatrixXd Aerodinamica::Rotz(double AngleinRad)
{
  Eigen::MatrixXd Rz(3, 3);
  Rz << cos(AngleinRad), -sin(AngleinRad), 0, sin(AngleinRad), cos(AngleinRad), 0, 0, 0, 1;
  return Rz;
}

Eigen::MatrixXd Aerodinamica::Roty(double AngleinRad)
{
  Eigen::MatrixXd Ry(3, 3);
  Ry << cos(AngleinRad), 0, sin(AngleinRad), 0, 1, 0, -sin(AngleinRad), 0, cos(AngleinRad);
  return Ry;
}

Eigen::MatrixXd Aerodinamica::Rotx(double AngleinRad)
{
  Eigen::MatrixXd Rx(3, 3);
  Rx << 1, 0, 0, 0, cos(AngleinRad), -sin(AngleinRad), 0, sin(AngleinRad), cos(AngleinRad);
  return Rx;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

GZ_REGISTER_MODEL_PLUGIN(Aerodinamica)
}  // namespace gazebo
