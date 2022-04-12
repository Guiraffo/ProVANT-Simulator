/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file aerodinamica2.cpp
 * @brief This file contains the declaration of a Gazebo model plugin that
 * simulate aerodinamics.
 *
 * @author Jonatan Campos
 * @author Júnio Eduardo de Morais Aquino
 */

#include "aerodinamica2.h"

#include "XMLRead.h"

namespace gazebo
{
Aerodinamica2::Aerodinamica2() : DBf(3), DBh(3), DBv(3)  //:SaveDataWind(6,1)
{
  Ho = 1.21;      // densidade do ar
  sf = 0.0146;    // area da superficie da fuselagem
  sh = 0.01437;   // area da superficie do estabilizador horizontal
  sv = 0.013635;  // area da superficie do estabilizador vertical

  R_IB.setZero(3, 3);
  wind_B.setZero(3, 1);  // vento no corpo
  wind_I.setZero(3, 1);  // vento inercial
  T = 0.001;

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

  // Position of the Aerodinamic centers w.r.t the Body frame expressed in Body frame
  Eigen::VectorXd PosCG(3);

  PosCG << 0.008, 0, -0.043;
  DBf << -0.005, 0, 0.0326;       // Position aerodynamic center of fuselage
  DBh << -0.375547, 0, 0.030084;  // Position aerodynamic center of vertical stabilizer
  DBv << -0.42236, 0, 0.11569;    // Position aerodynamic center of horizontal stabilizer

  DBf = DBf - PosCG;
  DBh = DBh - PosCG;
  DBv = DBv - PosCG;
}

Aerodinamica2::~Aerodinamica2()
{
  Wind.endFile();
}

void Aerodinamica2::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    std::cout << "Aerodinamica2 nao inicializado!" << std::endl;
    return;
  }

  topic_Elev = XMLRead::ReadXMLString("topic_Elev", _sdf);
  topic_Rud = XMLRead::ReadXMLString("topic_Rud", _sdf);
  topic_Fr = XMLRead::ReadXMLString("topic_Fr", _sdf);
  topic_Fl = XMLRead::ReadXMLString("topic_Fl", _sdf);

  NameOfLinkBody_ = XMLRead::ReadXMLString("bodyName", _sdf);
  NameOfLinkFr_ = XMLRead::ReadXMLString("LinkFr", _sdf);
  NameOfLinkFl_ = XMLRead::ReadXMLString("LinkFl", _sdf);

  world = _model->GetWorld();

  linkFr = _model->GetLink(NameOfLinkFr_);
  linkFl = _model->GetLink(NameOfLinkFl_);
  link = _model->GetLink(NameOfLinkBody_);

  updateConnection = event::Events::ConnectWorldUpdateEnd([this]() { this->Update(); });

  // subscribers
  motor_subscriberFR_ = node_handle_.subscribe(topic_Fr, 1, &gazebo::Aerodinamica2::CallbackFR, this);
  motor_subscriberFL_ = node_handle_.subscribe(topic_Fl, 1, &gazebo::Aerodinamica2::CallbackFL, this);
  motor_subscriberFElev_ = node_handle_.subscribe(topic_Elev, 1, &gazebo::Aerodinamica2::CallbackFElev, this);
  motor_subscriberFRud_ = node_handle_.subscribe(topic_Rud, 1, &gazebo::Aerodinamica2::CallbackFRud, this);
}

void Aerodinamica2::CallbackFR(std_msgs::Float64 msg)
{
  ignition::math::Vector3d forceR(0, 0, msg.data);
  ignition::math::Vector3d torqueR(0, 0, 0.0178947368 * msg.data);  // drag torque
  linkFr->AddRelativeForce(forceR);
  linkFr->AddRelativeTorque(torqueR);
}

void Aerodinamica2::CallbackFL(std_msgs::Float64 msg)
{
  ignition::math::Vector3d forceL(0, 0, msg.data);
  ignition::math::Vector3d torqueL(0, 0, -0.0178947368 * msg.data);  // drag torque
  linkFl->AddRelativeForce(forceL);
  linkFl->AddRelativeTorque(torqueL);
}

void Aerodinamica2::CallbackFElev(std_msgs::Float64 De)
{
  ElevatorDeflection = De.data;
}

void Aerodinamica2::CallbackFRud(std_msgs::Float64 Dr)
{
  RudderDeflection = Dr.data;
}

void Aerodinamica2::Update()
{
  // Fuselagem
  common::Time sim_time = world->SimTime();
  boost::mutex::scoped_lock scoped_lock(lock);

  // Calculo do vento
  ignition::math::Vector3d linear = link->WorldLinearVel();
  double xp = linear.X();
  double yp = linear.Y();
  double zp = linear.Z();

  ignition::math::Pose3d pose = link->WorldPose();
  phi = pose.Rot().Euler().X();
  theta = pose.Rot().Euler().Y();
  psi = pose.Rot().Euler().Z();

  R_IB << (cos(psi) * cos(theta)), (cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi)),
      (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)), (cos(theta) * sin(psi)),
      (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)),
      (cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi)), (-sin(theta)), (cos(theta) * sin(phi)),
      (cos(phi) * cos(theta));

  Eigen::MatrixXd XpYpZp(3, 1);
  Eigen::MatrixXd uvw(3, 1);

  XpYpZp << xp, yp, zp;
  uvw = R_IB.transpose() * XpYpZp;

  ub = uvw(0);
  vb = uvw(1);
  wb = uvw(2);
  static double Tempo = 0;
  Tempo = Tempo + T;

  // Environment wind properties
  wind_I << 0, 0, 0;
  wind_B = R_IB.transpose() * wind_I;
  ua = wind_B(0, 0);
  va = wind_B(1, 0);
  wa = wind_B(2, 0);
  std::vector<double> DataWind;
  DataWind.push_back(ua);
  DataWind.push_back(va);
  DataWind.push_back(wa);
  DataWind.push_back(wind_I(0));
  DataWind.push_back(wind_I(1));
  DataWind.push_back(wind_I(2));
  static int Contador = 0;
  if (Contador % 120 == 0)
  {
    Wind.printFile(DataWind);
  }

  Alpha = getAlpha(wb, wa, ub, ua);
  Beta = getBeta(vb, va, ub, ua);

  Eigen::MatrixXd RBAlpha(3, 3), RBBeta(3, 3);
  RBAlpha << cos(-Alpha), 0, sin(-Alpha), 0, 1, 0, -sin(-Alpha), 0, cos(-Alpha);

  RBBeta << cos(-Beta), -sin(-Beta), 0, sin(-Beta), cos(-Beta), 0, 0, 0, 1;

  Vxy = getAirxy(vb, va, ub, ua);
  Vxz = getAirxz(wb, wa, ub, ua);

  double Val = ((-Alpha + 3.1416) / 0.1);
  int IndexA = floor(Val);
  double ProporcaoA = Val - IndexA;

  Val = ((-Beta + 3.1416) / 0.1);
  int IndexS = floor(Val);
  double ProporcaoS = Val - IndexS;

  double CDfxz = VetCDf[IndexA] + ProporcaoA * (VetCDf[IndexA + 1] - VetCDf[IndexA]);
  double CLfxz = VetCLf[IndexA] + ProporcaoA * (VetCLf[IndexA + 1] - VetCLf[IndexA]);
  double CDfxy = VetCDf[IndexS] + ProporcaoS * (VetCDf[IndexS + 1] - VetCDf[IndexS]);
  double CLfxy = VetCLf[IndexS] + ProporcaoS * (VetCLf[IndexS + 1] - VetCLf[IndexS]);
  double CDh = VetCDh[IndexA] + ProporcaoA * (VetCDh[IndexA + 1] - VetCDh[IndexA]);
  double CLh = VetCLh[IndexA] + ProporcaoA * (VetCLh[IndexA + 1] - VetCLh[IndexA]);
  double CDv = VetCDv[IndexS] + ProporcaoS * (VetCDv[IndexS + 1] - VetCDv[IndexS]);
  double CLv = VetCLv[IndexS] + ProporcaoS * (VetCLv[IndexS + 1] - VetCLv[IndexS]);

  // Aerodynamic forces applied by the fuselage
  Eigen::MatrixXd Ffxz(3, 1), Ffxy(3, 1), FIf(3, 1);

  double kfxy = 0.5 * Ho * sf * pow(Vxy, 2);
  double kfxz = 0.5 * Ho * sf * pow(Vxz, 2);

  Ffxz << -kfxz * CDfxz, 0, kfxz * CLfxz;
  Ffxy << -kfxy * CDfxy, kfxy * CLfxy, 0;

  FIf = R_IB * (RBAlpha * Ffxz + RBBeta * Ffxy);  // Expressa forças no inercial
  link->AddForceAtRelativePosition(ignition::math::Vector3d(FIf(0), FIf(1), FIf(2)),
                                   ignition::math::Vector3d(DBf(0), DBf(1), DBf(2)));  // new

  // Aerodynamic forces applied by the vertical stabilizer
  double kvxy = 0.5 * Ho * sv * pow(Vxy, 2);
  Eigen::MatrixXd FBv(3, 1), FIv(3, 1);
  FBv << -kvxy * CDv, kvxy * (CLv + c_r(RudderDeflection)), 0;
  FIv = R_IB * RBBeta * FBv;  // Expressa forças no inercial
  link->AddForceAtRelativePosition(ignition::math::Vector3d(FIv(0), FIv(1), FIv(2)),
                                   ignition::math::Vector3d(DBv(0), DBv(1), DBv(2)));  // new

  // Aerodynamic forces applied by the horizontal stabilizer
  double khxz = 0.5 * Ho * sh * pow(Vxz, 2);
  Eigen::MatrixXd FBh(3, 1), FIh(3, 1);
  FBh << -khxz * CDh, 0, khxz * (CLh + c_e(ElevatorDeflection));
  FIh = R_IB * RBAlpha * FBh;  // Expressa forças no inercial
  link->AddForceAtRelativePosition(ignition::math::Vector3d(FIh(0), FIh(1), FIh(2)),
                                   ignition::math::Vector3d(DBh(0), DBh(1), DBh(2)));  // new
}

double Aerodinamica2::getAlpha(double wb, double wa, double ub, double ua)
{
  double alpha;
  alpha = atan2((wb - wa), (ub - ua));
  if (alpha > 3.0584)
  {
    alpha = 3.0584;
  }
  else if (alpha < -3.1416)
  {
    alpha = -3.1416;
  }
  return alpha;
}

double Aerodinamica2::getBeta(double vb, double va, double ub, double ua)
{
  double beta;
  beta = atan2((vb - va), (ub - ua));
  if (beta > 3.0584)
  {
    beta = 3.0584;
  }
  else if (beta < -3.1416)
  {
    beta = -3.1416;
  }
  return beta;
}

double Aerodinamica2::getAirxz(double wb, double wa, double ub, double ua)
{
  return sqrt(pow(wb - wa, 2) + pow(ub - ua, 2));
}

double Aerodinamica2::getAirxy(double vb, double va, double ub, double ua)
{
  return sqrt(pow(vb - va, 2) + pow(ub - ua, 2));
}

double Aerodinamica2::c_r(double Dr)
{
  return 1.165 * Dr;
}

double Aerodinamica2::c_e(double De)
{
  return 2.1873375 * De;
}

GZ_REGISTER_MODEL_PLUGIN(Aerodinamica2)
}  // namespace gazebo
