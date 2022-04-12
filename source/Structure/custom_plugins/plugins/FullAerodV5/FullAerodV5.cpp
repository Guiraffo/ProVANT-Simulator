/*
 * File: FullAerodV5.cpp
 * Author: Jonatan Mota Campos
 * Project: ProVANT
 * Company: Federal University of Minas Gerais
 * Version: 1.0
 * Date: 03/03/2021
 * Description:  This library is responsable to implement aerodynamics forces in a UAV of type tilt rotor with four
 * rotors
 */

#include "FullAerodV5.h"

#include "XMLRead.h"

namespace gazebo
{
/**********************************************************************************************************************/
// constructor
FullAerodV5::FullAerodV5()
  : UAVCm(3)
  , RudderCm(3)
  , ACf(3)
  , ACwr(3)
  , ACwl(3)
  , AChr(3)
  , AChl(3)
  , ACrr(3)
  , ACrl(3)
  , dArr(3)
  , dArl(3)
  , dBA4(3)
  , dBA5(3)
  , pIfp(3)
  , pIwrp(3)
  , pIwlp(3)
  , pIhrp(3)
  , pIhlp(3)
  , pIrrp(3)
  , pIrlp(3)
  , RIB(3, 3)
  , Wn(3, 3)
  , RAlphaf(3, 3)
  , RBetaf(3, 3)
  , RAlphawr(3, 3)
  , RAlphawl(3, 3)
  , RAlphahr(3, 3)
  , RAlphahl(3, 3)
  , RBetarr(3, 3)
  , RBetarl(3, 3)
  , Fxz(3)
  , Fxy(3)
  , Force_F(3)
  , FWrxz(3)
  , FWlxz(3)
  , Force_Wr(3)
  , Force_Wl(3)
  , FHrxz(3)
  , FHlxz(3)
  , Force_Hr(3)
  , Force_Hl(3)
  , FRrxy(3)
  , FRlxy(3)
  , Force_Rr(3)
  , Force_Rl(3)
  , WIIB(3)
  , PhipThetapPsip(3)
  , XpYpZp(3)
  , EnvironmentWind(3)
  , UVWf(3)
  , UVWfa(3)
  , UVWwr(3)
  , UVWwra(3)
  , UVWwl(3)
  , UVWwla(3)
  , UVWhr(3)
  , UVWhra(3)
  , UVWhl(3)
  , UVWhla(3)
  , UVWrr(3)
  , UVWrra(3)
  , UVWrl(3)
  , UVWrla(3)
{
  // Environment wind-speed w.r.t the Inertial Frame
  EnvironmentWind << 0, 0, 0;
  pi = 3.141592653589793;
  // Position of the Aerodinamic centers w.r.t the Body frame expressed in Body frame
  ACf << -0.05343, 0, -0.07182;
  ACwr << 0.250436, -0.259292, -0.097038;
  ACwl << 0.250436, 0.259292, -0.097038;
  AChr << -0.331992, -0.360017, 0.004812;
  AChl << -0.331992, 0.360017, 0.004812;
  dArr << -0.12749, 0, -0.20169;
  dArl << -0.12749, 0, -0.20169;
  dBA4 << -0.312133, -0.57, 0.010512;
  dBA5 << -0.312133, 0.57, 0.010512;
  UAVCm << -0.13, 0, -0.15;
  RudderCm << -0.01769, 0, -0.08356;
  // Aerod center position wrt origin of sdf

  // variables
  rho = 1.29;
  SfFrontal = 0.1692;
  SfLateral = 0.1692;
  sw = 0.0775;
  sh = 0.1457;
  sr = 0.0500;
  sigma = 0.0873;  // rad
  beta = 0.0873;
}

/**********************************************************************************************************************/
void FullAerodV5::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  _logMsg = "[FullAerodV5: " + GetHandle() + "] ";
  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Initializing plugin.");

  try
  {
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "A ROS node for Gazebo has not been initialized, unable to load "
                                                   "plugin. Load the Gazebo system plugin "
                                                   "\'libgazebo_ros_api_plugin.so\' in the gazebo_ros package and "
                                                   "try again. Note: This plugin will be automatically loaded using "
                                                   "the \"roslaunch gazebo_ros empty_world\" launch command.");
      return;
    }

    topic_FR2 = XMLRead::ReadXMLString("topic_FR2", _sdf);  // name of brushless number 1
    topic_FL3 = XMLRead::ReadXMLString("topic_FL3", _sdf);  // name of brushless number 2
    topic_FR4 = XMLRead::ReadXMLString("topic_FR4", _sdf);  // name of brushless number 3
    topic_FL5 = XMLRead::ReadXMLString("topic_FL5", _sdf);  // name of brushless number 4
    topic_Ar = XMLRead::ReadXMLString("topic_Ar", _sdf);    // name of Aileron right frontal
    topic_Al = XMLRead::ReadXMLString("topic_Al", _sdf);    // name of Aileron left frontal
    topic_Hr = XMLRead::ReadXMLString("topic_Hr", _sdf);    // name of horizontal stab. right rear
    topic_Hl = XMLRead::ReadXMLString("topic_Hl", _sdf);    // name of horizontal stab. left rear
    topic_Rr = XMLRead::ReadXMLString("topic_Rr", _sdf);    // name of Rudder right rear
    topic_Rl = XMLRead::ReadXMLString("topic_Rl", _sdf);    // name of Rudder left rear
    NameOfLink2_ = XMLRead::ReadXMLString("Rotor2", _sdf);  // name of rotor2 link
    NameOfLink3_ = XMLRead::ReadXMLString("Rotor3", _sdf);  // name of rotor3 link
    NameOfLink4_ = XMLRead::ReadXMLString("Rotor4", _sdf);  // name of rotor4 link
    NameOfLink5_ = XMLRead::ReadXMLString("Rotor5", _sdf);  // name of rotor5 link
    DragConst = XMLRead::ReadXMLDouble("DragCte", _sdf);    // Drag constant
    NameOfLinkMainBody_ = XMLRead::ReadXMLString("MainBody", _sdf);

    NameOfJointRotorCm4_ = XMLRead::ReadXMLString("RotorCm4joint", _sdf);
    NameOfJointRotorCm5_ = XMLRead::ReadXMLString("RotorCm5joint", _sdf);

    // get elements of the simulation
    linkFr2 = _model->GetLink(NameOfLink2_);
    linkFl3 = _model->GetLink(NameOfLink3_);
    linkFr4 = _model->GetLink(NameOfLink4_);
    linkFl5 = _model->GetLink(NameOfLink5_);

    RotorCm4joint = _model->GetJoint(NameOfJointRotorCm4_);
    RotorCm5joint = _model->GetJoint(NameOfJointRotorCm5_);
    // get world pointer
    world = _model->GetWorld();
    MainBody = _model->GetLink(NameOfLinkMainBody_);

    // update timer
    updateConnection =
        event::Events::ConnectWorldUpdateBegin([this](const common::UpdateInfo /*info*/) { this->Update(); });

    // subscribers of data to apply in simulator
    motor_subscriberFR2_ = node_handle_.subscribe(topic_FR2, 1, &gazebo::FullAerodV5::CallbackFR2, this);
    motor_subscriberFL3_ = node_handle_.subscribe(topic_FL3, 1, &gazebo::FullAerodV5::CallbackFL3, this);
    motor_subscriberFR4_ = node_handle_.subscribe(topic_FR4, 1, &gazebo::FullAerodV5::CallbackFR4, this);
    motor_subscriberFL5_ = node_handle_.subscribe(topic_FL5, 1, &gazebo::FullAerodV5::CallbackFL5, this);

    motor_subscriberAr_ = node_handle_.subscribe(topic_Ar, 1, &gazebo::FullAerodV5::CallbackAr, this);
    motor_subscriberAl_ = node_handle_.subscribe(topic_Al, 1, &gazebo::FullAerodV5::CallbackAl, this);
    motor_subscriberHr_ = node_handle_.subscribe(topic_Hr, 1, &gazebo::FullAerodV5::CallbackHr, this);
    motor_subscriberHl_ = node_handle_.subscribe(topic_Hl, 1, &gazebo::FullAerodV5::CallbackHl, this);
    motor_subscriberRr_ = node_handle_.subscribe(topic_Rr, 1, &gazebo::FullAerodV5::CallbackRr, this);
    motor_subscriberRl_ = node_handle_.subscribe(topic_Rl, 1, &gazebo::FullAerodV5::CallbackRl, this);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

/*****************************************************************************************************************************/

// callback to get the force at brushless 1
void FullAerodV5::CallbackFR2(std_msgs::Float64 msg)
{
  try
  {
    ignition::math::Vector3d forceR2(0, 0, msg.data);
    linkFr2->AddRelativeForce(forceR2);

    ignition::math::Vector3d torqueR2(0, 0, 0.0179 * msg.data);  // drag torque
                                                                 // Applying
    linkFr2->AddRelativeTorque(torqueR2);
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

/**********************************************************************************************************************/

// callback to get the force at brushless of rotor 3
void FullAerodV5::CallbackFL3(std_msgs::Float64 msg)
{
  ignition::math::Vector3d forceL3(0, 0, msg.data);
  linkFl3->AddRelativeForce(forceL3);

  ignition::math::Vector3d torqueL3(0, 0, -0.0179 * msg.data);  // drag torque
  linkFl3->AddRelativeTorque(torqueL3);
}

/**********************************************************************************************************************/

// callback to get the force at brushless 3
void FullAerodV5::CallbackFR4(std_msgs::Float64 msg)
{
  ignition::math::Vector3d forceR4(0, 0, msg.data);
  linkFr4->AddRelativeForce(forceR4);
  ignition::math::Vector3d torqueR4(0, 0, -0.0179 * msg.data);  // drag torque
  // Applying
  linkFr4->AddRelativeTorque(torqueR4);
}

/**********************************************************************************************************************/

// callback to get the force at brushless 4
void FullAerodV5::CallbackFL5(std_msgs::Float64 msg)
{
  ignition::math::Vector3d forceL5(0, 0, msg.data);
  linkFl5->AddRelativeForce(forceL5);
  ignition::math::Vector3d torqueL5(0, 0, 0.0179 * msg.data);  // drag torque
  // Applying
  linkFl5->AddRelativeTorque(torqueL5);
}

/**********************************************************************************************************************/

void FullAerodV5::CallbackAr(std_msgs::Float64 DAr)
{
  deflec_Ar = DAr.data;
}

/**********************************************************************************************************************/

void FullAerodV5::CallbackAl(std_msgs::Float64 DAl)
{
  deflec_Al = DAl.data;
}
/**********************************************************************************************************************/

void FullAerodV5::CallbackHr(std_msgs::Float64 DHr)
{
  deflec_Hr = DHr.data;
}
/**********************************************************************************************************************/

void FullAerodV5::CallbackHl(std_msgs::Float64 DHl)
{
  deflec_Hl = DHl.data;
}

/**********************************************************************************************************************/

void FullAerodV5::CallbackRr(std_msgs::Float64 DRr)
{
  deflec_Rr = DRr.data;
}

/**********************************************************************************************************************/

void FullAerodV5::CallbackRl(std_msgs::Float64 DRl)
{
  deflec_Rl = DRl.data;
}

Eigen::VectorXd GetCoef(double Alphaf, double Betaf, double Alphawr, double Alphawl, double Alphahr, double Alphahl,
                        double Betarr, double Betarl)
{
  const double VetCDf[361] = {
    1.5574,  1.5559,  1.5545,  1.553,   1.5516,  1.5502,  1.549,   1.5477,  1.5466,  1.5456,  1.5446,  1.5438,  1.5431,
    1.5425,  1.542,   1.5416,  1.5412,  1.5408,  1.5404,  1.5399,  1.5393,  1.5387,  1.5379,  1.537,   1.5359,  1.5347,
    1.5334,  1.5319,  1.5302,  1.5284,  1.5264,  1.5242,  1.5218,  1.5192,  1.5165,  1.5136,  1.5106,  1.5074,  1.5041,
    1.5006,  1.4971,  1.4934,  1.4896,  1.4856,  1.4816,  1.4774,  1.4731,  1.4687,  1.4641,  1.4594,  1.4546,  1.4496,
    1.4445,  1.4392,  1.4336,  1.4279,  1.4218,  1.4156,  1.409,   1.4021,  1.3949,  1.3873,  1.3794,  1.3711,  1.3625,
    1.3536,  1.3444,  1.3348,  1.3249,  1.3148,  1.3043,  1.2936,  1.2825,  1.2713,  1.2597,  1.2479,  1.2359,  1.2236,
    1.2111,  1.1985,  1.1856,  1.1725,  1.1594,  1.1461,  1.1328,  1.1195,  1.1062,  1.0931,  1.0802,  1.0674,  1.055,
    1.0428,  1.0309,  1.0192,  1.0077,  0.99623, 0.98484, 0.97345, 0.96201, 0.95045, 0.93874, 0.92682, 0.91469, 0.90236,
    0.88984, 0.87713, 0.86424, 0.85119, 0.83796, 0.82458, 0.81105, 0.79737, 0.78358, 0.7697,  0.75574, 0.74175, 0.72773,
    0.71372, 0.69974, 0.68582, 0.67197, 0.65823, 0.6446,  0.63107, 0.61766, 0.60437, 0.5912,  0.57814, 0.56521, 0.55241,
    0.53974, 0.52719, 0.51478, 0.50251, 0.49036, 0.47835, 0.46648, 0.45474, 0.44314, 0.43168, 0.42035, 0.40917, 0.39814,
    0.38729, 0.37663, 0.36619, 0.35598, 0.34601, 0.33631, 0.3269,  0.3178,  0.30902, 0.30058, 0.2925,  0.28481, 0.27753,
    0.27068, 0.26427, 0.25834, 0.25289, 0.24796, 0.24355, 0.23966, 0.23626, 0.23333, 0.23086, 0.2288,  0.22713, 0.2258,
    0.22479, 0.22403, 0.22349, 0.2231,  0.22283, 0.22259, 0.22231, 0.22193, 0.22137, 0.22058, 0.21949, 0.21806, 0.21624,
    0.21405, 0.2115,  0.20863, 0.20547, 0.20207, 0.19847, 0.19474, 0.19091, 0.18704, 0.18317, 0.17936, 0.17565, 0.17209,
    0.16872, 0.16559, 0.16273, 0.16017, 0.15791, 0.15596, 0.15432, 0.15299, 0.15196, 0.15122, 0.15075, 0.15054, 0.15058,
    0.15086, 0.15137, 0.1521,  0.15303, 0.15417, 0.15553, 0.1571,  0.1589,  0.16092, 0.16318, 0.16567, 0.1684,  0.17138,
    0.17461, 0.17808, 0.1818,  0.18578, 0.19,    0.19448, 0.19921, 0.20418, 0.20941, 0.2149,  0.22063, 0.22661, 0.23283,
    0.2393,  0.24601, 0.25294, 0.26011, 0.2675,  0.27512, 0.28296, 0.29101, 0.29927, 0.30773, 0.3164,  0.32527, 0.33434,
    0.34359, 0.35303, 0.36265, 0.37245, 0.38242, 0.39256, 0.40286, 0.41332, 0.42393, 0.43469, 0.44559, 0.45663, 0.4678,
    0.4791,  0.49052, 0.50206, 0.51372, 0.5255,  0.53741, 0.54944, 0.5616,  0.57388, 0.58628, 0.59881, 0.61147, 0.62422,
    0.63706, 0.64995, 0.66287, 0.6758,  0.68872, 0.7016,  0.71441, 0.72714, 0.73976, 0.75226, 0.76464, 0.77689, 0.78899,
    0.80094, 0.81273, 0.82436, 0.83581, 0.84708, 0.85816, 0.86905, 0.87977, 0.89032, 0.90071, 0.91094, 0.92103, 0.93098,
    0.94079, 0.95048, 0.96005, 0.96949, 0.9788,  0.98797, 0.99698, 1.0058,  1.0145,  1.0231,  1.0314,  1.0395,  1.0475,
    1.0552,  1.0627,  1.07,    1.0771,  1.0838,  1.0903,  1.0965,  1.1024,  1.108,   1.1132,  1.1182,  1.1228,  1.1271,
    1.1312,  1.1349,  1.1384,  1.1417,  1.1447,  1.1475,  1.15,    1.1524,  1.1545,  1.1565,  1.1583,  1.1599,  1.1615,
    1.1629,  1.1642,  1.1654,  1.1665,  1.1676,  1.1686,  1.1695,  1.1704,  1.1712,  1.172,   1.1727,  1.1733,  1.1739,
    1.1745,  1.175,   1.1755,  1.1759,  1.1763,  1.1767,  1.1771,  1.1774,  1.1776,  1.1779
  };
  const double VetCLf[361] = {
    -0.11823,  -0.1288,  -0.13939,  -0.15002,  -0.16071,  -0.17149,  -0.18235,  -0.1933,   -0.20434,  -0.21548,
    -0.22671,  -0.23804, -0.24946,  -0.26096,  -0.27256,  -0.28425,  -0.29602,  -0.30788,  -0.31981,  -0.33182,
    -0.34389,  -0.35602, -0.3682,   -0.38044,  -0.39272,  -0.40506,  -0.41744,  -0.42986,  -0.44233,  -0.45484,
    -0.46739,  -0.47998, -0.49259,  -0.50522,  -0.51787,  -0.53052,  -0.54316,  -0.55579,  -0.56839,  -0.58096,
    -0.5935,   -0.60597, -0.61837,  -0.63066,  -0.64281,  -0.65479,  -0.66657,  -0.67812,  -0.6894,   -0.70039,
    -0.71106,  -0.72138, -0.73132,  -0.74084,  -0.74991,  -0.75851,  -0.7666,   -0.77416,  -0.78114,  -0.78753,
    -0.79328,  -0.79839, -0.80285,  -0.8067,   -0.80995,  -0.81264,  -0.81477,  -0.81638,  -0.81749,  -0.81812,
    -0.8183,   -0.81804, -0.81739,  -0.81638,  -0.81507,  -0.81349,  -0.81167,  -0.80967,  -0.80752,  -0.80526,
    -0.80294,  -0.80058, -0.79823,  -0.79592,  -0.79365,  -0.79148,  -0.78942,  -0.7875,   -0.78575,  -0.7842,
    -0.78287,  -0.78178, -0.78091,  -0.78019,  -0.77961,  -0.77909,  -0.77862,  -0.77813,  -0.77759,  -0.77696,
    -0.77618,  -0.77523, -0.77408,  -0.77273,  -0.77116,  -0.76936,  -0.76732,  -0.76504,  -0.76249,  -0.75968,
    -0.75659,  -0.7532,  -0.74953,  -0.74556,  -0.74131,  -0.73678,  -0.73197,  -0.72687,  -0.7215,   -0.71585,
    -0.70993,  -0.70373, -0.69725,  -0.69049,  -0.68343,  -0.67607,  -0.66841,  -0.66043,  -0.65212,  -0.64349,
    -0.63451,  -0.6252,  -0.61554,  -0.60556,  -0.59525,  -0.58463,  -0.5737,   -0.56246,  -0.55093,  -0.53912,
    -0.52702,  -0.51465, -0.50205,  -0.48925,  -0.47628,  -0.46319,  -0.45001,  -0.43678,  -0.42353,  -0.4103,
    -0.39713,  -0.38405, -0.37107,  -0.3582,   -0.34544,  -0.33279,  -0.32027,  -0.30788,  -0.29562,  -0.28351,
    -0.27154,  -0.25972, -0.24804,  -0.23649,  -0.22504,  -0.21368,  -0.20241,  -0.19122,  -0.1801,   -0.16904,
    -0.15807,  -0.1472,  -0.13645,  -0.12584,  -0.11539,  -0.10509,  -0.094968, -0.085023, -0.075275, -0.065745,
    -0.056451, -0.04741, -0.038625, -0.030096, -0.021823, -0.013804, -0.006035, 0.0014874, 0.0087682, 0.015814,
    0.022644,  0.029277, 0.035734,  0.042033,  0.048183,  0.05419,   0.06006,   0.065795,  0.071377,  0.076784,
    0.081992,  0.086985, 0.091769,  0.096356,  0.10076,   0.10499,   0.10907,   0.113,     0.11679,   0.12046,
    0.12403,   0.1275,   0.13091,   0.13429,   0.13768,   0.14112,   0.14463,   0.14825,   0.15202,   0.15597,
    0.16014,   0.16456,  0.16921,   0.17409,   0.17918,   0.18447,   0.18995,   0.19561,   0.20143,   0.2074,
    0.21351,   0.21975,  0.2261,    0.23257,   0.23914,   0.24581,   0.25258,   0.25944,   0.26637,   0.27339,
    0.28047,   0.28761,  0.29483,   0.30212,   0.30949,   0.31694,   0.32449,   0.33214,   0.33989,   0.34775,
    0.35573,   0.36383,  0.37205,   0.38039,   0.38884,   0.3974,    0.40607,   0.41485,   0.42373,   0.43271,
    0.44179,   0.45097,  0.46021,   0.46949,   0.47878,   0.48805,   0.49728,   0.50643,   0.51548,   0.52439,
    0.53315,   0.54171,  0.55006,   0.55816,   0.56598,   0.57349,   0.58067,   0.58748,   0.59389,   0.59987,
    0.6054,    0.61045,  0.615,     0.61906,   0.62261,   0.62565,   0.62817,   0.63017,   0.63164,   0.63258,
    0.63297,   0.63282,  0.63213,   0.63092,   0.62919,   0.62696,   0.62424,   0.62104,   0.61738,   0.61325,
    0.60868,   0.60368,  0.59827,   0.59246,   0.58629,   0.57977,   0.57291,   0.56575,   0.55831,   0.5506,
    0.54264,   0.53447,  0.5261,    0.51757,   0.50889,   0.50011,   0.49125,   0.48233,   0.47339,   0.46446,
    0.45555,   0.44671,  0.43793,   0.42923,   0.42062,   0.41211,   0.4037,    0.3954,    0.38723,   0.37919,
    0.37128,   0.36353,  0.35593,   0.34847,   0.34115,   0.33398,   0.32695,   0.32006,   0.3133,    0.30669,
    0.3002,    0.29385,  0.28762,   0.28151,   0.2755,    0.26959,   0.26376,   0.25802,   0.25236,   0.24678,
    0.24126,   0.23581,  0.23042,   0.22508,   0.2198,    0.21457,   0.20937,   0.20422,   0.19909,   0.19399,
    0.18889
  };

  const double VetCDfs[361] = {
    -0.11751,   -0.11813,   -0.11872,   -0.11928,  -0.11978,   -0.12021,  -0.12053,   -0.12071,  -0.12072,   -0.12054,
    -0.12014,   -0.11952,   -0.11866,   -0.11755,  -0.11618,   -0.11454,  -0.11262,   -0.11043,  -0.10794,   -0.10514,
    -0.10202,   -0.098583,  -0.09481,   -0.090702, -0.086258,  -0.081474, -0.076348,  -0.070878, -0.06506,   -0.058891,
    -0.05237,   -0.045495,  -0.038274,  -0.030717, -0.022834,  -0.014633, -0.0061257, 0.0026797, 0.011773,   0.021145,
    0.030785,   0.040677,   0.050767,   0.060996,  0.071304,   0.081631,  0.091916,   0.1021,    0.11212,    0.12192,
    0.13144,    0.14063,    0.14944,    0.15785,   0.16584,    0.17338,   0.18043,    0.18697,   0.19298,    0.19842,
    0.20326,    0.20749,    0.21112,    0.21418,   0.21669,    0.21866,   0.22014,    0.22114,   0.22168,    0.22178,
    0.22149,    0.2208,     0.21976,    0.21839,   0.2167,     0.21474,   0.21252,    0.21006,   0.2074,     0.20456,
    0.20156,    0.19843,    0.19518,    0.19184,   0.18841,    0.18491,   0.18136,    0.17778,   0.17417,    0.17055,
    0.16694,    0.16336,    0.1598,     0.15628,   0.15278,    0.14932,   0.1459,     0.14252,   0.13919,    0.1359,
    0.13266,    0.12947,    0.12632,    0.12322,   0.12017,    0.11714,   0.11415,    0.11119,   0.10826,    0.10534,
    0.10245,    0.099566,   0.096695,   0.093833,  0.090977,   0.088126,  0.085276,   0.082426,  0.079572,   0.076713,
    0.073847,   0.070971,   0.068089,   0.065207,  0.06233,    0.059461,  0.056605,   0.053769,  0.050956,   0.04817,
    0.045418,   0.042702,   0.040023,   0.037377,  0.034762,   0.032178,  0.029622,   0.027091,  0.024585,   0.0221,
    0.019636,   0.017191,   0.014766,   0.012365,  0.0099887,  0.0076411, 0.0053243,  0.0030408, 0.00079318, -0.0014161,
    -0.0035845, -0.0057083, -0.007779,  -0.009787, -0.011723,  -0.013576, -0.015338,  -0.016999, -0.018549,  -0.019978,
    -0.021277,  -0.022436,  -0.023451,  -0.024315, -0.025024,  -0.025575, -0.025974,  -0.02623,  -0.026352,  -0.026352,
    -0.026247,  -0.026057,  -0.025804,  -0.025508, -0.025194,  -0.024888, -0.024616,  -0.024401, -0.024246,  -0.024153,
    -0.024122,  -0.024153,  -0.024246,  -0.024401, -0.024616,  -0.024888, -0.025194,  -0.025508, -0.025804,  -0.026057,
    -0.026247,  -0.026352,  -0.026352,  -0.02623,  -0.025974,  -0.025575, -0.025024,  -0.024315, -0.023451,  -0.022436,
    -0.021277,  -0.019978,  -0.018549,  -0.016999, -0.015338,  -0.013576, -0.011723,  -0.009787, -0.007779,  -0.0057083,
    -0.0035845, -0.0014161, 0.00079318, 0.0030408, 0.0053243,  0.0076411, 0.0099887,  0.012365,  0.014766,   0.017191,
    0.019636,   0.0221,     0.024585,   0.027091,  0.029622,   0.032178,  0.034762,   0.037377,  0.040023,   0.042702,
    0.045418,   0.04817,    0.050956,   0.053769,  0.056605,   0.059461,  0.06233,    0.065207,  0.068089,   0.070971,
    0.073847,   0.076713,   0.079572,   0.082426,  0.085276,   0.088126,  0.090977,   0.093833,  0.096695,   0.099566,
    0.10245,    0.10534,    0.10826,    0.11119,   0.11415,    0.11714,   0.12017,    0.12322,   0.12632,    0.12947,
    0.13266,    0.1359,     0.13919,    0.14252,   0.1459,     0.14932,   0.15278,    0.15628,   0.1598,     0.16336,
    0.16694,    0.17055,    0.17417,    0.17778,   0.18136,    0.18491,   0.18841,    0.19184,   0.19518,    0.19843,
    0.20156,    0.20456,    0.2074,     0.21006,   0.21252,    0.21474,   0.2167,     0.21839,   0.21976,    0.2208,
    0.22149,    0.22178,    0.22168,    0.22114,   0.22014,    0.21866,   0.21669,    0.21418,   0.21112,    0.20749,
    0.20326,    0.19842,    0.19298,    0.18697,   0.18043,    0.17338,   0.16584,    0.15785,   0.14944,    0.14063,
    0.13144,    0.12192,    0.11212,    0.1021,    0.091916,   0.081631,  0.071304,   0.060996,  0.050767,   0.040677,
    0.030785,   0.021145,   0.011773,   0.0026797, -0.0061257, -0.014633, -0.022834,  -0.030717, -0.038274,  -0.045495,
    -0.05237,   -0.058891,  -0.06506,   -0.070878, -0.076348,  -0.081474, -0.086258,  -0.090702, -0.09481,   -0.098583,
    -0.10202,   -0.10514,   -0.10794,   -0.11043,  -0.11262,   -0.11454,  -0.11618,   -0.11755,  -0.11866,   -0.11952,
    -0.12014,   -0.12054,   -0.12072,   -0.12071,  -0.12053,   -0.12021,  -0.11978,   -0.11928,  -0.11872,   -0.11813,
    -0.11751
  };
  const double VetCLfs[361] = {
    -0.0049047,  -0.0036266, -0.0023585, -0.0011103, 0.00010803, 0.0012875,   0.002423,   0.0035102,  0.0045449,
    0.0055235,   0.0064448,  0.0073083,  0.0081136,  0.0088586,  0.0095354,   0.010135,   0.010647,   0.011063,
    0.01137,     0.011557,   0.011612,   0.011521,   0.011267,   0.010831,    0.010194,   0.0093367,  0.0082408,
    0.0068873,   0.0052573,  0.0033319,  0.0010922,  -0.0014775, -0.0043806,  -0.0076173, -0.011188,  -0.015092,
    -0.019331,   -0.023904,  -0.028811,  -0.034053,  -0.039631,  -0.045536,   -0.05173,   -0.058167,  -0.064803,
    -0.07159,    -0.078483,  -0.085437,  -0.092405,  -0.099341,  -0.1062,     -0.11294,   -0.11953,   -0.12596,
    -0.1322,     -0.13822,   -0.14401,   -0.14954,   -0.15479,   -0.15974,    -0.16437,   -0.16865,   -0.17258,
    -0.17617,    -0.17942,   -0.18232,   -0.18488,   -0.1871,    -0.18898,    -0.19053,   -0.19173,   -0.1926,
    -0.19314,    -0.19337,   -0.19331,   -0.19295,   -0.19232,   -0.19143,    -0.19029,   -0.18891,   -0.1873,
    -0.18548,    -0.18346,   -0.18125,   -0.17885,   -0.17628,   -0.17354,    -0.17064,   -0.1676,    -0.16443,
    -0.16112,    -0.1577,    -0.15419,   -0.1506,    -0.14696,   -0.14329,    -0.13962,   -0.13597,   -0.13236,
    -0.12881,    -0.12535,   -0.12199,   -0.11874,   -0.11559,   -0.11255,    -0.10962,   -0.10678,   -0.10405,
    -0.10142,    -0.098892,  -0.096461,  -0.094126,  -0.091881,  -0.089715,   -0.087618,  -0.085583,  -0.083599,
    -0.081658,   -0.079749,  -0.077864,  -0.075993,  -0.074128,  -0.072267,   -0.070409,  -0.068552,  -0.066696,
    -0.064839,   -0.062981,  -0.06112,   -0.059254,  -0.057384,  -0.05551,    -0.053643,  -0.051797,  -0.049984,
    -0.048217,   -0.046511,  -0.044879,  -0.043332,  -0.041886,  -0.040553,   -0.039342,  -0.038246,  -0.037255,
    -0.036357,   -0.035541,  -0.034796,  -0.034111,  -0.033475,  -0.032877,   -0.032306,  -0.031752,  -0.031204,
    -0.030654,   -0.030093,  -0.029513,  -0.028903,  -0.028256,  -0.027563,   -0.026813,  -0.026,     -0.025115,
    -0.024163,   -0.023151,  -0.022083,  -0.020966,  -0.019805,  -0.018605,   -0.017369,  -0.0161,    -0.014796,
    -0.013452,   -0.012064,  -0.010629,  -0.009156,  -0.0076546, -0.0061356,  -0.004608,  -0.003075,  -0.0015384,
    -3.8913e-18, 0.0015384,  0.003075,   0.004608,   0.0061356,  0.0076546,   0.009156,   0.010629,   0.012064,
    0.013452,    0.014796,   0.0161,     0.017369,   0.018605,   0.019805,    0.020966,   0.022083,   0.023151,
    0.024163,    0.025115,   0.026,      0.026813,   0.027563,   0.028256,    0.028903,   0.029513,   0.030093,
    0.030654,    0.031204,   0.031752,   0.032306,   0.032877,   0.033475,    0.034111,   0.034796,   0.035541,
    0.036357,    0.037255,   0.038246,   0.039342,   0.040553,   0.041886,    0.043332,   0.044879,   0.046511,
    0.048217,    0.049984,   0.051797,   0.053643,   0.05551,    0.057384,    0.059254,   0.06112,    0.062981,
    0.064839,    0.066696,   0.068552,   0.070409,   0.072267,   0.074128,    0.075993,   0.077864,   0.079749,
    0.081658,    0.083599,   0.085583,   0.087618,   0.089715,   0.091881,    0.094126,   0.096461,   0.098892,
    0.10142,     0.10405,    0.10678,    0.10962,    0.11255,    0.11559,     0.11874,    0.12199,    0.12535,
    0.12881,     0.13236,    0.13597,    0.13962,    0.14329,    0.14696,     0.1506,     0.15419,    0.1577,
    0.16112,     0.16443,    0.1676,     0.17064,    0.17354,    0.17628,     0.17885,    0.18125,    0.18346,
    0.18548,     0.1873,     0.18891,    0.19029,    0.19143,    0.19232,     0.19295,    0.19331,    0.19337,
    0.19314,     0.1926,     0.19173,    0.19053,    0.18898,    0.1871,      0.18488,    0.18232,    0.17942,
    0.17617,     0.17258,    0.16865,    0.16437,    0.15974,    0.15479,     0.14954,    0.14401,    0.13822,
    0.1322,      0.12596,    0.11953,    0.11294,    0.1062,     0.099341,    0.092405,   0.085437,   0.078483,
    0.07159,     0.064803,   0.058167,   0.05173,    0.045536,   0.039631,    0.034053,   0.028811,   0.023904,
    0.019331,    0.015092,   0.011188,   0.0076173,  0.0043806,  0.0014775,   -0.0010922, -0.0033319, -0.0052573,
    -0.0068873,  -0.0082408, -0.0093367, -0.010194,  -0.010831,  -0.011267,   -0.011521,  -0.011612,  -0.011557,
    -0.01137,    -0.011063,  -0.010647,  -0.010135,  -0.0095354, -0.0088586,  -0.0081136, -0.0073083, -0.0064448,
    -0.0055235,  -0.0045449, -0.0035102, -0.002423,  -0.0012875, -0.00010803, 0.0011103,  0.0023585,  0.0036266,
    0.0049047
  };

  const double VetCLW[63] = {
    -0.1,     -0.11168, 0.046158, 0.19012,  0.14949,  0.032696,  0.16352,  0.40322,  0.47168,  0.43987,  0.41031,
    0.39008,  0.36076,  0.29406,  0.19604,  0.084877, -0.036643, -0.17593, -0.34932, -0.52255, -0.62463, -0.64979,
    -0.64757, -0.75134, -0.92226, -0.8552,  -0.84943, -0.95235,  -1.1505,  -0.80517, -0.33892, 0.12202,  0.66824,
    1.1202,   1.5215,   1.3454,   1.1965,   1.2698,   1.4431,    1.3737,   1.1108,   0.97621,  0.88426,  0.68382,
    0.48775,  0.35297,  0.19672,  0.019917, -0.11727, -0.2102,   -0.27283, -0.31364, -0.34131, -0.36589, -0.39616,
    -0.43407, -0.44923, -0.42199, -0.38794, -0.36149, -0.31994,  -0.25637, -0.17593
  };
  const double VetCDW[63] = { 0.3,     0.34268, 0.42225,  0.51124,  0.58331, 0.63736, 0.69401, 0.75098, 0.79637,
                              0.82679, 0.84797, 0.87748,  0.9134,   0.93713, 0.95045, 0.96032, 0.9569,  0.93842,
                              0.92269, 0.91245, 0.89517,  0.85707,  0.80044, 0.78136, 0.75541, 0.53444, 0.40511,
                              0.25973, 0.14268, 0.060593, 0.025275, 0.01631, 0.03762, 0.08312, 0.1536,  0.25923,
                              0.43233, 0.5999,  0.69536,  0.81398,  0.89075, 0.95251, 1.0054,  1.0477,  1.0817,
                              1.1101,  1.135,   1.1496,   1.144,    1.1279,  1.1121,  1.0905,  1.0612,  1.0305,
                              1.0027,  0.97769, 0.94374,  0.89018,  0.81981, 0.73368, 0.62861, 0.51045, 0.39123 };

  const double VetCLH[63] = {
    -0.1,     -0.11168, 0.046158, 0.19012,  0.14949,  0.032696,  0.16352,  0.40322,  0.47168,  0.43987,  0.41031,
    0.39008,  0.36076,  0.29406,  0.19604,  0.084877, -0.036643, -0.17593, -0.34932, -0.52255, -0.62463, -0.64979,
    -0.64757, -0.75134, -0.92226, -0.8552,  -0.84943, -0.95235,  -1.1505,  -0.80517, -0.33892, 0.12202,  0.66824,
    1.1202,   1.5215,   1.3454,   1.1965,   1.2698,   1.4431,    1.3737,   1.1108,   0.97621,  0.88426,  0.68382,
    0.48775,  0.35297,  0.19672,  0.019917, -0.11727, -0.2102,   -0.27283, -0.31364, -0.34131, -0.36589, -0.39616,
    -0.43407, -0.44923, -0.42199, -0.38794, -0.36149, -0.31994,  -0.25637, -0.17593
  };
  const double VetCDH[63] = { 0.3,     0.34268, 0.42225,  0.51124,  0.58331, 0.63736, 0.69401, 0.75098, 0.79637,
                              0.82679, 0.84797, 0.87748,  0.9134,   0.93713, 0.95045, 0.96032, 0.9569,  0.93842,
                              0.92269, 0.91245, 0.89517,  0.85707,  0.80044, 0.78136, 0.75541, 0.53444, 0.40511,
                              0.25973, 0.14268, 0.060593, 0.025275, 0.01631, 0.03762, 0.08312, 0.1536,  0.25923,
                              0.43233, 0.5999,  0.69536,  0.81398,  0.89075, 0.95251, 1.0054,  1.0477,  1.0817,
                              1.1101,  1.135,   1.1496,   1.144,    1.1279,  1.1121,  1.0905,  1.0612,  1.0305,
                              1.0027,  0.97769, 0.94374,  0.89018,  0.81981, 0.73368, 0.62861, 0.51045, 0.39123 };

  const double VetCDV[63] = { 0.0071,  -0.11395, 0.10658, 0.45137,  0.711,    0.85029,  0.98929,  1.1324,   1.2382,
                              1.3224,  1.3995,   1.4579,  1.5009,   1.5443,   1.581,    1.5978,   1.5993,   1.5894,
                              1.5642,  1.5276,   1.4894,  1.4474,   1.3927,   1.3111,   1.222,    1.1889,   1.0564,
                              0.67454, 0.30154,  0.11121, 0.024927, 0.010853, 0.012969, 0.035073, 0.13043,  0.35371,
                              0.74531, 1.0998,   1.1925,  1.2344,   1.327,    1.4034,   1.4551,   1.4959,   1.5341,
                              1.5695,  1.592,    1.5998,  1.5963,   1.5761,   1.537,    1.494,    1.4497,   1.3875,
                              1.3087,  1.2227,   1.1104,  0.96413,  0.82972,  0.67893,  0.39398,  0.054892, -0.1234 };
  const double VetCLV[63] = {
    0,        0.3994,   0.55274,  0.58755,  0.62718,  0.70302,   0.76193,  0.7903,   0.79189,  0.75553,  0.68102,
    0.59354,  0.49502,  0.36827,  0.23541,  0.114,    -0.061171, -0.30075, -0.49385, -0.61787, -0.73228, -0.84611,
    -0.94327, -1.0036,  -1.0128,  -0.97063, -0.92622, -0.99894,  -1.0372,  -0.74074, -0.43079, -0.12844, 0.18012,
    0.48016,  0.79568,  1.0575,   0.97331,  0.92997,  0.98023,   1.0149,   0.99685,  0.92894,  0.82764,  0.71268,
    0.59908,  0.46781,  0.26059,  0.024942, -0.13593, -0.25643,  -0.39112, -0.51329, -0.60848, -0.69516, -0.76483,
    -0.79388, -0.78733, -0.75437, -0.69013, -0.61666, -0.58454,  -0.53872, -0.35292
  };

  /***************************************************************/
  /**************** CALCULATE AEROD. COEFF *****************/

  if (Alphaf < -1.55)
  {
    Alphaf = -1.55;
  }
  else if (Alphaf > 1.55)
  {
    Alphaf = 1.55;
  }
  // The vector of aerodynamic coefficients CD and CL are represented from -180 to 180 with data for each 0.1 rad/s
  // calculo dos coef. da fuselagem
  double Valfa =
      ((Alphaf + 3.141592653589793 / 2.0) / 0.008726646259972);  // start by computing the index of the coefficient
  int Indexfa = floor(Valfa);                                    // round it to the floor value
  double Proporcaofa = Valfa - Indexfa;  // compute the proportion between the floor value and the next one

  double CDfa =
      VetCDf[Indexfa] + Proporcaofa * (VetCDf[Indexfa + 1] - VetCDf[Indexfa]);  // make a linear interpolation between
                                                                                // the floor value and the next one
  double CLfa =
      VetCLf[Indexfa] + Proporcaofa * (VetCLf[Indexfa + 1] - VetCLf[Indexfa]);  // make a linear interpolation between
                                                                                // the floor value and the next one

  if (Betaf < -1.55)
  {
    Betaf = -1.55;
  }
  else if (Betaf > 1.55)
  {
    Betaf = 1.55;
  }

  double Valfs = ((Betaf + 3.141592653589793 / 2.0) / 0.008726646259972);
  int Indexfs = floor(Valfs);
  double Proporcaofs = Valfs - Indexfs;

  double CDfs = VetCDfs[Indexfs] + Proporcaofs * (VetCDfs[Indexfs + 1] - VetCDfs[Indexfs]);
  double CLfs = VetCLfs[Indexfs] + Proporcaofs * (VetCLfs[Indexfs + 1] - VetCLfs[Indexfs]);

  // calculo dos coef. da asa
  if (Alphawr <= -3.1415)
  {
    Alphawr = -3.1415;
  }
  else if (Alphawr >= 3.0584)
  {
    Alphawr = 3.0584;
  }
  double Valwr = ((Alphawr + 3.1416) / 0.1);
  int Indexwr = floor(Valwr);
  double Proporcaowr = Valwr - Indexwr;

  double CDWr = VetCDW[Indexwr] + Proporcaowr * (VetCDW[Indexwr + 1] - VetCDW[Indexwr]);
  double CLWr = VetCLW[Indexwr] + Proporcaowr * (VetCLW[Indexwr + 1] - VetCLW[Indexwr]);

  // Alphawl = 0.0873;
  if (Alphawl <= -3.1415)
  {
    Alphawl = -3.1415;
  }
  else if (Alphawl >= 3.0584)
  {
    Alphawl = 3.0584;
  }
  double Valwl = ((Alphawl + 3.1416) / 0.1);
  int Indexwl = floor(Valwl);
  double Proporcaowl = Valwl - Indexwl;

  double CDWl = VetCDW[Indexwl] + Proporcaowl * (VetCDW[Indexwl + 1] - VetCDW[Indexwl]);
  double CLWl = VetCLW[Indexwl] + Proporcaowl * (VetCLW[Indexwl + 1] - VetCLW[Indexwl]);

  // calculo dos coef. dos estabilizadores horizontais
  if (Alphahr <= -3.1415)
  {
    Alphahr = -3.1415;
  }
  else if (Alphahr >= 3.0584)
  {
    Alphahr = 3.0584;
  }
  double Valhr = ((Alphahr + 3.1416) / 0.1);
  int Indexhr = floor(Valhr);
  double Proporcaohr = Valhr - Indexhr;

  double CDHr = VetCDH[Indexhr] + Proporcaohr * (VetCDH[Indexhr + 1] - VetCDH[Indexhr]);
  double CLHr = VetCLH[Indexhr] + Proporcaohr * (VetCLH[Indexhr + 1] - VetCLH[Indexhr]);

  if (Alphahl <= -3.1415)
  {
    Alphahl = -3.1415;
  }
  else if (Alphahl >= 3.0584)
  {
    Alphahl = 3.0584;
  }
  double Valhl = ((Alphahl + 3.1416) / 0.1);
  int Indexhl = floor(Valhl);
  double Proporcaohl = Valhl - Indexhl;

  double CDHl = VetCDH[Indexhl] + Proporcaohl * (VetCDH[Indexhl + 1] - VetCDH[Indexhl]);
  double CLHl = VetCLH[Indexhl] + Proporcaohl * (VetCLH[Indexhl + 1] - VetCLH[Indexhl]);

  // Calculo dos coeficientes dos Ruddervators
  if (Betarr <= -3.1415)
  {
    Betarr = -3.1415;
  }
  else if (Betarr >= 3.0584)
  {
    Betarr = 3.0584;
  }
  double Valrr = ((Betarr + 3.1416) / 0.1);
  int Indexrr = floor(Valrr);
  double Proporcaorr = Valrr - Indexrr;

  double CDRr = VetCDV[Indexrr] + Proporcaorr * (VetCDV[Indexrr + 1] - VetCDV[Indexrr]);
  double CLRr = VetCLV[Indexrr] + Proporcaorr * (VetCLV[Indexrr + 1] - VetCLV[Indexrr]);

  if (Betarl <= -3.1415)
  {
    Betarl = -3.1415;
  }
  else if (Betarl >= 3.0584)
  {
    Betarl = 3.0584;
  }
  double Valrl = ((Betarl + 3.1416) / 0.1);
  int Indexrl = floor(Valrl);
  double Proporcaorl = Valrl - Indexrl;

  double CDRl = VetCDV[Indexrl] + Proporcaorl * (VetCDV[Indexrl + 1] - VetCDV[Indexrl]);
  double CLRl = VetCLV[Indexrl] + Proporcaorl * (VetCLV[Indexrl + 1] - VetCLV[Indexrl]);

  double Ca = 1.2110;
  double Ce = 1.2110;
  double Cr = 1.1500;

  Eigen::VectorXd VetCoef(19);

  VetCoef << CDfa, CLfa, CDfs, CLfs, CDWr, CLWr, CDWl, CLWl, CDHr, CLHr, CDHl, CLHl, CDRr, CLRr, CDRl, CLRl, Ca, Ce, Cr;
  return VetCoef;
}

/***************************************************************************************/
void FullAerodV5::Update()
{
  try
  {
    ignition::math::Vector3d Linear = MainBody->WorldLinearVel();
    ignition::math::Vector3d Angular = MainBody->WorldAngularVel();
    ignition::math::Pose3d pose = MainBody->WorldPose();
    double ayR = RotorCm4joint->Position();
    double ayL = RotorCm5joint->Position();
    double ayRp = RotorCm4joint->GetVelocity(0);
    double ayLp = RotorCm5joint->GetVelocity(0);
    Eigen::VectorXd ayRpvec(3);
    Eigen::VectorXd ayRpvec_(3);
    Eigen::VectorXd ayLpvec(3);
    Eigen::VectorXd ayLpvec_(3);
    ayRpvec << 0, ayRp, 0;
    ayRpvec_ = Rotx(-beta).transpose() * ayRpvec;
    ayLpvec << 0, ayLp, 0;
    ayLpvec_ = Rotx(beta).transpose() * ayLpvec;
    phi = pose.Rot().Euler().X();
    theta = pose.Rot().Euler().Y();
    psi = pose.Rot().Euler().Z();

    /********************* *********** **********************************************************/

    // computing transformation matrices

    RIB << (cos(psi) * cos(theta)), (cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi)),
        (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)), (cos(theta) * sin(psi)),
        (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)),
        (cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi)), (-sin(theta)), (cos(theta) * sin(phi)),
        (cos(phi) * cos(theta));

    Wn << 1.0, 0.0, -sin(theta), 0.0, cos(phi), cos(theta) * sin(phi), 0.0, -sin(phi), cos(phi) * cos(theta);

    // computing orientation matrices for rear rotors
    Eigen::MatrixXd RBC4(3, 3);
    RBC4 << Roty(ayR) * Rotx(-beta);
    Eigen::MatrixXd RBC5(3, 3);
    RBC5 << Roty(ayL) * Rotx(beta);
    Eigen::MatrixXd RIC4(3, 3);
    RIC4 << RIB * RBC4;
    Eigen::MatrixXd RIC5(3, 3);
    RIC5 << RIB * RBC5;

    ACrr << RBC4 * dArr + dBA4;
    ACrl << RBC5 * dArl + dBA5;

    //-----------Computing [phidot thetadot psidot]-----------------------%
    WIIB << Angular.X(), Angular.Y(), Angular.Z();
    PhipThetapPsip = Wn.inverse() * RIB.transpose() * WIIB;

    //-----------Computing [Xdot Ydot Zdot]-------------------------------%
    XpYpZp << Linear.X(), Linear.Y(), Linear.Z();

    // Compute the velocity of the aerodynamic centers expressed in the Inertial frame
    pIfp << -RIB * SkewSymmetricMatrix(ACf) * Wn * PhipThetapPsip +
                XpYpZp;  // velocity of the aerodynamic center of fuselage w.r.t I expressed in I
    pIwrp << -RIB * SkewSymmetricMatrix(ACwr) * Wn * PhipThetapPsip +
                 XpYpZp;  // velocity of the aerodynamic center of wing right w.r.t I expressed in I
    pIwlp << -RIB * SkewSymmetricMatrix(ACwl) * Wn * PhipThetapPsip +
                 XpYpZp;  // velocity of the aerodynamic center of wing left w.r.t I expressed in I
    pIhrp << -RIB * SkewSymmetricMatrix(AChr) * Wn * PhipThetapPsip +
                 XpYpZp;  // velocity of the aerodynamic center of horizontal stab right w.r.t I expressed in I
    pIhlp << -RIB * SkewSymmetricMatrix(AChl) * Wn * PhipThetapPsip +
                 XpYpZp;  // velocity of the aerodynamic center of horizontal stab left w.r.t I expressed in I
    pIrrp << -RIB * SkewSymmetricMatrix(RBC4 * dArr + dBA4) * Wn * PhipThetapPsip + XpYpZp -
                 RIC4 * SkewSymmetricMatrix(dArr) * ayRpvec_;
    pIrlp << -RIB * SkewSymmetricMatrix(RBC5 * dArl + dBA5) * Wn * PhipThetapPsip + XpYpZp -
                 RIC5 * SkewSymmetricMatrix(dArl) * ayLpvec_;
    //----------Computing Properties of Relative wind for fuselage---------%

    UVWf = RIB.transpose() * pIfp;  // Express the velocity on the frame positioned at the aerodynamic center of
                                    // fuselage
    UVWfa = RIB.transpose() * EnvironmentWind;  // Express the enviroment wind-speed on the frame positioned at the
                                                // aerodynamic center of fuselage

    double Vfxz = pow(pow(UVWf(2) - UVWfa(2), 2) + pow(UVWf(0) - UVWfa(0), 2), 0.5);  // Magnitude x-z axis
    double Vfxy = pow(pow(UVWf(1) - UVWfa(1), 2) + pow(UVWf(0) - UVWfa(0), 2), 0.5);  // Magnitude x-y axis

    double Alphaf = atan2(UVWf(2) - UVWfa(2), UVWf(0) - UVWfa(0));  // Orientation - Angle of attack fuselage
    double Betaf = atan2(UVWf(1) - UVWfa(1), UVWf(0) - UVWfa(0));   // Orientation - Side slip angle fuselage

    //----------Computing Properties of Relative wind for Wings---------%

    // Wing right
    UVWwr = RIB.transpose() * pIwrp;  // Express the velocity on the frame positioned at the aerodynamic center of wing
                                      // R
    UVWwra = RIB.transpose() * EnvironmentWind;  // Express the enviroment wind-speed on the frame positioned at the
                                                 // aerodynamic center of wing R
    double Vwr = pow(pow(UVWwr(2) - UVWwra(2), 2) + pow(UVWwr(0) - UVWwra(0), 2), 0.5);  // compute Magnitude
    double Alphawr = atan2(UVWwr(2) - UVWwra(2), UVWwr(0) - UVWwra(0));                  // compute Orientation

    // Wing left
    UVWwl = RIB.transpose() * pIwlp;  // Express the velocity on the frame positioned at the aerodynamic center of wing
                                      // L
    UVWwla = RIB.transpose() * EnvironmentWind;  // Express the enviroment wind-speed on the frame positioned at the
                                                 // aerodynamic center of wing L
    double Vwl = pow(pow(UVWwl(2) - UVWwla(2), 2) + pow(UVWwl(0) - UVWwla(0), 2), 0.5);  // compute Magnitude
    double Alphawl = atan2(UVWwl(2) - UVWwla(2), UVWwl(0) - UVWwla(0));                  // compute Orientation

    //----------Computing Properties of Relative wind for Horiz. Stab.---------%

    // Horz. Stab. right
    UVWhr = Rotx(-sigma).transpose() * RIB.transpose() *
            pIhrp;  // Express the velocity on the frame positioned at the aerodynamic center of wing R
    UVWhra =
        Rotx(-sigma).transpose() * RIB.transpose() * EnvironmentWind;  // Express the enviroment wind-speed on the frame
                                                                       // positioned at the aerodynamic center of wing R
    double Vhr = pow(pow(UVWhr(2) - UVWhra(2), 2) + pow(UVWhr(0) - UVWhra(0), 2), 0.5);  // compute Magnitude
    double Alphahr = atan2(UVWhr(2) - UVWhra(2), UVWhr(0) - UVWhra(0));                  // compute Orientation

    // Horz. Stab. left
    UVWhl = Rotx(sigma).transpose() * RIB.transpose() *
            pIhlp;  // Express the velocity on the frame positioned at the aerodynamic center of wing L
    UVWhla =
        Rotx(sigma).transpose() * RIB.transpose() * EnvironmentWind;  // Express the enviroment wind-speed on the frame
                                                                      // positioned at the aerodynamic center of wing L
    double Vhl = pow(pow(UVWhl(2) - UVWhla(2), 2) + pow(UVWhl(0) - UVWhla(0), 2), 0.5);  // compute Magnitude
    double Alphahl = atan2(UVWhl(2) - UVWhla(2), UVWhl(0) - UVWhla(0));                  // compute Orientation

    //----------Computing Properties of Relative wind for Vertical. Stab.---------%
    // Vert. Stab. Right
    UVWrr = Roty(-pi / 2.0).transpose() * RIC4.transpose() * pIrrp;
    UVWrra = Roty(-pi / 2.0).transpose() * RIC4.transpose() * EnvironmentWind;
    double Vrr = pow(pow(UVWrr(1) - UVWrra(1), 2) + pow(UVWrr(0) - UVWrra(0), 2), 0.5);  // compute Magnitude
    double Betarr = atan2(UVWrr(1) - UVWrra(1), UVWrr(0) - UVWrra(0));                   // compute Orientation

    // Vert. Stab. Left
    UVWrl = Roty(-pi / 2.0).transpose() * RIC5.transpose() * pIrlp;
    UVWrla = Roty(-pi / 2.0).transpose() * RIC5.transpose() * EnvironmentWind;
    double Vrl = pow(pow(UVWrl(1) - UVWrla(1), 2) + pow(UVWrl(0) - UVWrla(0), 2), 0.5);  // compute Magnitude
    double Betarl = atan2(UVWrl(1) - UVWrla(1), UVWrl(0) - UVWrla(0));                   // compute Orientation

    // std::cout << "[AEROD]DEBUG UPDATE4[AEROD]" << std::endl;

    /***************************************************************/
    /**************** CALCULATE AEROD. Coefficients ****************/

    Eigen::VectorXd VetCoef(19);
    VetCoef = GetCoef(-Alphaf, -Betaf, -Alphawr, -Alphawl, -Alphahr, -Alphahl, -Betarr, -Betarl);

    double CDfa = VetCoef(0);
    double CLfa = VetCoef(1);
    double CDfs = VetCoef(2);
    double CLfs = VetCoef(3);
    double CDWr = VetCoef(4);
    double CLWr = VetCoef(5);
    double CDWl = VetCoef(6);
    double CLWl = VetCoef(7);
    double CDHr = VetCoef(8);
    double CLHr = VetCoef(9);
    double CDHl = VetCoef(10);
    double CLHl = VetCoef(11);
    double CDRr = VetCoef(12);
    double CLRr = VetCoef(13);
    double CDRl = VetCoef(14);
    double CLRl = VetCoef(15);
    double Ca = VetCoef(16);
    double Ce = VetCoef(17);
    double Cr = VetCoef(18);

    /***************************************************************/
    /**************** CALCULATE AEROD. FORCES*****************/

    /*******Fuselage************/
    double Fd_xz_F = -0.5 * rho * (Vfxz * Vfxz) * SfFrontal * CDfa;
    double Fl_xz_F = 0.5 * rho * (Vfxz * Vfxz) * SfFrontal * CLfa;
    double Fd_xy_F = -0.5 * rho * (Vfxy * Vfxy) * SfLateral * CDfs;
    double Fl_xy_F = 0.5 * rho * (Vfxy * Vfxy) * SfLateral * CLfs;
    // Rotation matrices that map from wind to the aerodynamic center frame
    RAlphaf << cos(-Alphaf), 0, sin(-Alphaf), 0, 1, 0, -sin(-Alphaf), 0, cos(-Alphaf);

    RBetaf << cos(-Betaf), -sin(-Betaf), 0, sin(-Betaf), cos(-Betaf), 0, 0, 0, 1;

    // Make the vector of aerodynamic forces applied by the fuselage
    Fxz << Fd_xz_F, 0, Fl_xz_F;
    Fxy << Fd_xy_F, Fl_xy_F, 0;

    // map the forces from the wind frame to the inertial frame
    Force_F = RIB * (RAlphaf * Fxz + RBetaf * Fxy);

    /*******Wings************/
    // Aerodynamic forces applied by the wings
    double Fd_Wr = -0.5 * rho * (Vwr * Vwr) * sw * CDWr;
    double Fl_Wr = 0.5 * rho * (Vwr * Vwr) * sw * (CLWr + Ca * deflec_Ar);
    double Fd_Wl = -0.5 * rho * (Vwl * Vwl) * sw * CDWl;
    double Fl_Wl = 0.5 * rho * (Vwl * Vwl) * sw * (CLWl + Ca * deflec_Al);

    // Rotation matrices that map from wind to the aerodynamic center frame
    RAlphawr << cos(-Alphawr), 0, sin(-Alphawr), 0, 1, 0, -sin(-Alphawr), 0, cos(-Alphawr);

    RAlphawl << cos(-Alphawl), 0, sin(-Alphawl), 0, 1, 0, -sin(-Alphawl), 0, cos(-Alphawl);

    // Make the vector of aerodynamic forces applied by the wings
    FWrxz << Fd_Wr, 0, Fl_Wr;
    FWlxz << Fd_Wl, 0, Fl_Wl;

    // map the forces from the wind frame to the inertial frame
    Force_Wr = RIB * RAlphawr * FWrxz;
    Force_Wl = RIB * RAlphawl * FWlxz;

    /*******Horizontl Stab.************/
    // Aerodynamic forces applied by the horiz. stab.
    double Fd_Hr = -0.5 * rho * (Vhr * Vhr) * sh * CDHr;
    double Fl_Hr = 0.5 * rho * (Vhr * Vhr) * sh * (CLHr + Ce * deflec_Hr);
    double Fd_Hl = -0.5 * rho * (Vhl * Vhl) * sh * CDHl;
    double Fl_Hl = 0.5 * rho * (Vhl * Vhl) * sh * (CLHl + Ce * deflec_Hl);

    // Rotation matrices that map from wind to the aerodynamic center frame
    RAlphahr << cos(-Alphahr), 0, sin(-Alphahr), 0, 1, 0, -sin(-Alphahr), 0, cos(-Alphahr);

    RAlphahl << cos(-Alphahl), 0, sin(-Alphahl), 0, 1, 0, -sin(-Alphahl), 0, cos(-Alphahl);

    // Make the vector of aerodynamic forces applied by the Horizontl Stab.
    FHrxz << Fd_Hr, 0, Fl_Hr;
    FHlxz << Fd_Hl, 0, Fl_Hl;

    // map the forces from the wind frame to the inertial frame
    Force_Hr = RIB * Rotx(-sigma) * RAlphahr * FHrxz;
    Force_Hl = RIB * Rotx(sigma) * RAlphahl * FHlxz;

    /*******Vertical Stab.************/
    // Aerodynamic forces applied by the vert. stab.

    double Fd_Rr = -0.5 * rho * (Vrr * Vrr) * sr * CDRr;
    double Fl_Rr = 0.5 * rho * (Vrr * Vrr) * sr * (CLRr + Cr * deflec_Rr);
    double Fd_Rl = -0.5 * rho * (Vrl * Vrl) * sr * CDRl;
    double Fl_Rl = 0.5 * rho * (Vrl * Vrl) * sr * (CLRl + Cr * deflec_Rl);

    // Rotation matrices that map from wind to the aerodynamic center frame
    RBetarr << cos(-Betarr), -sin(-Betarr), 0, sin(-Betarr), cos(-Betarr), 0, 0, 0, 1;

    RBetarl << cos(-Betarl), -sin(-Betarl), 0, sin(-Betarl), cos(-Betarl), 0, 0, 0, 1;

    // Make the vector of aerodynamic forces applied by the Vertical Stab.
    FRrxy << Fd_Rr, Fl_Rr, 0;
    FRlxy << Fd_Rl, Fl_Rl, 0;

    // map the forces from the wind frame to the inertial frame
    Force_Rr = RIC4 * Roty(-pi / 2.0) * RBetarr * FRrxy;
    Force_Rl = RIC5 * Roty(-pi / 2.0) * RBetarl * FRlxy;

    /***************************************************************/
    /**************** APPLY AEROD. FORCES GAZEBO *****************/

    // Get the position of the aerod center wrt the center of mass
    Eigen::VectorXd dCmF(3);
    Eigen::VectorXd dCmWr(3);
    Eigen::VectorXd dCmWl(3);
    Eigen::VectorXd dCmHr(3);
    Eigen::VectorXd dCmHl(3);
    Eigen::VectorXd dRudCmRr(3);
    Eigen::VectorXd dRudCmRl(3);

    dCmF = ACf - UAVCm;
    dCmWr = ACwr - UAVCm;
    dCmWl = ACwl - UAVCm;
    dCmHr = AChr - UAVCm;
    dCmHl = AChl - UAVCm;
    dRudCmRr = dArr - RudderCm;
    dRudCmRl = dArl - RudderCm;

    /*
    static int PlotValues = 0;
    PlotValues = PlotValues + 1;

    if (PlotValues%50 == 0)
    {
      std::cout << "CaR: " << deflec_Ar << "CaL: " << deflec_Al << "CeR: " << deflec_Hr << "CeL: " << deflec_Hl << "CrR:
    " << deflec_Rr << "CrL: " << deflec_Rl << std::endl; std::cout << "FrR: " << Force_Rr  << std::endl << "FrL: " <<
    Force_Rl  << std::endl;
    }*/

    MainBody->AddForceAtRelativePosition(ignition::math::Vector3d(Force_F(0), Force_F(1), Force_F(2)),
                                         ignition::math::Vector3d(dCmF(0), dCmF(1), dCmF(2)));
    MainBody->AddForceAtRelativePosition(ignition::math::Vector3d(Force_Wr(0), Force_Wr(1), Force_Wr(2)),
                                         ignition::math::Vector3d(dCmWr(0), dCmWr(1), dCmWr(2)));
    MainBody->AddForceAtRelativePosition(ignition::math::Vector3d(Force_Wl(0), Force_Wl(1), Force_Wl(2)),
                                         ignition::math::Vector3d(dCmWl(0), dCmWl(1), dCmWl(2)));
    MainBody->AddForceAtRelativePosition(ignition::math::Vector3d(Force_Hr(0), Force_Hr(1), Force_Hr(2)),
                                         ignition::math::Vector3d(dCmHr(0), dCmHr(1), dCmHr(2)));
    MainBody->AddForceAtRelativePosition(ignition::math::Vector3d(Force_Hl(0), Force_Hl(1), Force_Hl(2)),
                                         ignition::math::Vector3d(dCmHl(0), dCmHl(1), dCmHl(2)));
    linkFr4->AddForceAtRelativePosition(ignition::math::Vector3d(Force_Rr(0), Force_Rr(1), Force_Rr(2)),
                                        ignition::math::Vector3d(dRudCmRr(0), dRudCmRr(1), dRudCmRr(2)));
    linkFl5->AddForceAtRelativePosition(ignition::math::Vector3d(Force_Rl(0), Force_Rl(1), Force_Rl(2)),
                                        ignition::math::Vector3d(dRudCmRl(0), dRudCmRl(1), dRudCmRl(2)));
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

Eigen::MatrixXd FullAerodV5::SkewSymmetricMatrix(Eigen::VectorXd Vector)
{
  // Place Vet in the Skew Symmetric matrix S
  Eigen::MatrixXd SkewMatrix(3, 3);
  SkewMatrix << 0, -Vector(2), Vector(1), Vector(2), 0, -Vector(0), -Vector(1), Vector(0), 0;

  return SkewMatrix;
}

Eigen::MatrixXd FullAerodV5::Rotx(double ang)
{
  Eigen::MatrixXd Rotx_(3, 3);
  Rotx_ << 1, 0, 0, 0, cos(ang), -sin(ang), 0, sin(ang), cos(ang);

  return Rotx_;
}

Eigen::MatrixXd FullAerodV5::Roty(double ang)
{
  Eigen::MatrixXd Roty_alfay(3, 3);
  Roty_alfay << cos(ang), 0, sin(ang), 0, 1, 0, -sin(ang), 0, cos(ang);

  return Roty_alfay;
}

GZ_REGISTER_MODEL_PLUGIN(FullAerodV5)
}  // namespace gazebo
