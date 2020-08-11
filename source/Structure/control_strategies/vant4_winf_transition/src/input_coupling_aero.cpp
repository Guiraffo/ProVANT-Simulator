#include "vant4_winf_transition/input_coupling.h"

#include <cmath>

#include "vant4_winf_transition/uav4_parameters.h"
#include "vant4_winf_transition/skew_symmetric_matrix.h"

Eigen::MatrixXd InputCouplingMatrixAero(Eigen::VectorXd q, Eigen::VectorXd qp, double* ub)
{
  // Variáveis de Configuração
  double AlphaR = q(0);
  double AlphaL = q(1);
  double Phi = q(2);
  double Theta = q(3);
  double Psi = q(4);
  double X = q(5);
  double Y = q(6);
  double Z = q(7);

  // Derivada Variáveis de Configuração
  double AlphaRp = qp(0);
  double AlphaLp = qp(1);
  double Phip = qp(2);
  double Thetap = qp(3);
  double Psip = qp(4);
  double Xp = qp(5);
  double Yp = qp(6);
  double Zp = qp(7);

  Eigen::MatrixXd BAeroFUa(8, 5);

  // Distances from B to aerodynamic center
  // f  -> fuselage
  // wr -> wing right
  // wl -> wing left
  // tr -> tail right
  // wl -> tail left
  Eigen::VectorXd DBf(3);
  Eigen::VectorXd DBwr(3);
  Eigen::VectorXd DBwl(3);
  Eigen::VectorXd DBtr(3);
  Eigen::VectorXd DBtl(3);
  // Position of the Aerodinamic centers w.r.t the Body frame expressed in Body frame
  DBf << 0.0512, 0, 0.1;           // Position aerodynamic center of fuselage
  DBwr << 0.0512, -0.31, 0.1;      // Position aerodynamic center of wing R
  DBwl << 0.0512, 0.31, 0.1;       // Position aerodynamic center of wing L
  DBtr << -0.3967, -0.168, 0.148;  // Position aerodynamic center of tail R
  DBtl << -0.3967, 0.168, 0.148;   // Position aerodynamic center of tail L
  double rho = 1.21;               // densidade do ar
  // double sf = 2*0.0946;    //area da superficie da fuselagem
  double sfFrontal = 2.0 * 0.0946;
  double sfLateral = 2.2 * 0.1323;
  double sw = 2.0 * 0.1125;  // area da superficie da asa
  double st = 2.0 * 0.0494;  // area da superficie do Rudder esquerdo
  double mi = 0.5236;        // tail deflection rad
  double wd = 0.0873;        // wing diedral rad

  // Transformation matrices and configuration variables
  Eigen::MatrixXd RI_B(3, 3);
  Eigen::MatrixXd Wn(3, 3);
  Eigen::VectorXd WI_IB(3);
  Eigen::VectorXd PhipThetapPsip(3);
  Eigen::VectorXd XpYpZp(3);
  Eigen::VectorXd EnvironmentWind(3);
  Eigen::MatrixXd RBmiL(3, 3);
  Eigen::MatrixXd RBmiR(3, 3);
  Eigen::MatrixXd RBwdL(3, 3);
  Eigen::MatrixXd RBwdR(3, 3);

  // Velocities Aerodinamic centers
  Eigen::VectorXd dPI_f(3);
  Eigen::VectorXd dPI_wr(3);
  Eigen::VectorXd dPI_wl(3);
  Eigen::VectorXd dPI_tr(3);
  Eigen::VectorXd dPI_tl(3);

  // aerodynamic center velocities and wind velocities
  Eigen::VectorXd UVWf(3);
  Eigen::VectorXd UVWfa(3);
  Eigen::VectorXd UVWwr(3);
  Eigen::VectorXd UVWwra(3);
  Eigen::VectorXd UVWwl(3);
  Eigen::VectorXd UVWwla(3);
  Eigen::VectorXd UVWtr(3);
  Eigen::VectorXd UVWtra(3);
  Eigen::VectorXd UVWtl(3);
  Eigen::VectorXd UVWtla(3);

  //--------aerodynamic forces------------

  // fuselagem
  Eigen::MatrixXd RAlphaf(3, 3);
  Eigen::MatrixXd RBetaf(3, 3);
  Eigen::VectorXd Fxz(3);
  Eigen::VectorXd Fxy(3);
  Eigen::VectorXd Forca_F(3);

  // wings
  Eigen::VectorXd FWrxz(3);
  Eigen::VectorXd FWlxz(3);
  Eigen::VectorXd Forca_Wr(3);
  Eigen::VectorXd Forca_Wl(3);
  Eigen::MatrixXd RAlphawr(3, 3);
  Eigen::MatrixXd RAlphawl(3, 3);

  // tail
  Eigen::MatrixXd RGammatr(3, 3);
  Eigen::MatrixXd RGammatl(3, 3);
  Eigen::VectorXd ForcaTailR(3);
  Eigen::VectorXd ForcaTailL(3);
  Eigen::VectorXd F_TailR(3);
  Eigen::VectorXd F_TailL(3);

  Eigen::VectorXd az(3);
  az << 0, 0, 1;

  // Diedral - Tail R and L - Rotation matrices in x-axis - mi = tail deflection
  RBmiL << 1, 0, 0, 0, cos(mi), -sin(mi), 0, sin(mi), cos(mi);

  RBmiR << 1, 0, 0, 0, cos(-mi), -sin(-mi), 0, sin(-mi), cos(-mi);

  // Diedral - Wings R and L w.r.t x-axis - wd = wing deflection
  RBwdL << 1, 0, 0, 0, cos(wd), -sin(wd), 0, sin(wd), cos(wd);

  RBwdR << 1, 0, 0, 0, cos(-wd), -sin(-wd), 0, sin(-wd), cos(-wd);

  // computing transformation matrices
  RI_B << (cos(Psi) * cos(Theta)), (cos(Psi) * sin(Phi) * sin(Theta) - cos(Phi) * sin(Psi)),
      (sin(Phi) * sin(Psi) + cos(Phi) * cos(Psi) * sin(Theta)), (cos(Theta) * sin(Psi)),
      (cos(Phi) * cos(Psi) + sin(Phi) * sin(Psi) * sin(Theta)),
      (cos(Phi) * sin(Psi) * sin(Theta) - cos(Psi) * sin(Phi)), (-sin(Theta)), (cos(Theta) * sin(Phi)),
      (cos(Phi) * cos(Theta));

  Wn << 1.0, 0.0, -sin(Theta), 0.0, cos(Phi), cos(Theta) * sin(Phi), 0.0, -sin(Phi), cos(Phi) * cos(Theta);

  EnvironmentWind << 0, 0, 0;
  XpYpZp << Xp, Yp, Zp;
  PhipThetapPsip << Phip, Thetap, Psi;

  // Compute the velocity of the aerodynamic centers expressed in the Inertial frame
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

  //----------Computing Properties of Relative wind for fuselage---------%
  UVWf = RI_B.transpose() * dPI_f;  // Express the velocity on the frame positioned at the aerodynamic center of
                                    // fuselage
  UVWfa = RI_B.transpose() * EnvironmentWind;  // Express the enviroment wind-speed on the frame positioned at the
                                               // aerodynamic center of fuselage
  // fprintf (stderr, "UVWf(2): %f UVWfa(2): %f UVWf(0): %f UVWfa(0): %f \n", UVWf(2), UVWfa(2), UVWf(0), UVWfa(0));
  double Vfxz = pow(pow(UVWf(2) - UVWfa(2), 2) + pow(UVWf(0) - UVWfa(0), 2), 0.5);  // Magnitude x-z axis
  double Vfxy = pow(pow(UVWf(1) - UVWfa(1), 2) + pow(UVWf(0) - UVWfa(0), 2), 0.5);  // Magnitude x-y axis
  double Alphaf = atan2(UVWf(2) - UVWfa(2), UVWf(0) - UVWfa(0));  // Orientation - Angle of attack fuselage
  double Betaf = atan2(UVWf(1) - UVWfa(1), UVWf(0) - UVWfa(0));   // Orientation - Side slip angle fuselage
  // fprintf (stderr, "Vfxz: %f Vfxy: %f Alphaf: %f Betaf: %f \n", Vfxz, Vfxy, Alphaf, Betaf);
  *ub = UVWf(0) - UVWfa(0);
  // plot everything in order to evaluate the results
  std::cout << std::endl << std::endl << std::endl << std::endl << std::endl;
  std::cout << std::endl << std::endl << std::endl << std::endl << std::endl;
  std::cout << std::endl << std::endl << std::endl << std::endl << std::endl;
  //	std::cout << std::endl << "Vfxz:" << Vfxz << "  Vfxy:" << Vfxy << std::endl;
  //	std::cout << "Alphaf:" << Alphaf << "  Betaf:" << Betaf << std::endl;

  //----------Computing Properties of Relative wind for wings---------%
  // Wing right
  UVWwr = RBwdR.transpose() * RI_B.transpose() *
          dPI_wr;  // Express the velocity on the frame positioned at the aerodynamic center of wing R
  UVWwra = RBwdR.transpose() * RI_B.transpose() * EnvironmentWind;  // Express the enviroment wind-speed on the frame
                                                                    // positioned at the aerodynamic center of wing R
  double Vwr = pow(pow(UVWwr(2) - UVWwra(2), 2) + pow(UVWwr(0) - UVWwra(0), 2), 0.5);  // compute Magnitude
  double Alphawr = atan2(UVWwr(2) - UVWwra(2), UVWwr(0) - UVWwra(0));                  // compute Orientation

  // Wing left
  UVWwl = RBwdL.transpose() * RI_B.transpose() *
          dPI_wl;  // Express the velocity on the frame positioned at the aerodynamic center of wing L
  UVWwla = RBwdL.transpose() * RI_B.transpose() * EnvironmentWind;  // Express the enviroment wind-speed on the frame
                                                                    // positioned at the aerodynamic center of wing L
  double Vwl = pow(pow(UVWwl(2) - UVWwla(2), 2) + pow(UVWwl(0) - UVWwla(0), 2), 0.5);  // compute Magnitude
  double Alphawl = atan2(UVWwl(2) - UVWwla(2), UVWwl(0) - UVWwla(0));                  // compute Orientation

  // fprintf (stderr, "Vwr: %f Alphawr: %f Vwl: %f Alphawl: %f \n", Vwr, Alphawr, Vwl, Alphawl);

  // plot everything in order to evaluate the results
  //	std::cout << std::endl << "Vwr:" << Vwr << "  Vwl:" << Vwl << std::endl;
  //	std::cout << "Alphawr:" << Alphawr << "  Alphawl:" << Alphawl << std::endl;
  //----------Computing Properties of Relative wind for V-tail---------%
  // Tail right
  UVWtr = RBmiR.transpose() * RI_B.transpose() *
          dPI_tr;  // Express the velocity on the frame positioned at the aerodynamic center of Tail R
  UVWtra = RBmiR.transpose() * RI_B.transpose() * EnvironmentWind;  // Express the enviroment wind-speed on the frame
                                                                    // positioned at the aerodynamic center of wing R
  double Vtr = pow(pow(UVWtr(2) - UVWtra(2), 2) + pow(UVWtr(0) - UVWtra(0), 2), 0.5);  // compute Magnitude
  double Gammatr = atan2(UVWtr(2) - UVWtra(2), UVWtr(0) - UVWtra(0));                  // compute Orientation
  // Tail left
  UVWtl = RBmiL.transpose() * RI_B.transpose() *
          dPI_tl;  // Express the velocity on the frame positioned at the aerodynamic center of Tail L
  UVWtla = RBmiL.transpose() * RI_B.transpose() * EnvironmentWind;  // Express the enviroment wind-speed on the frame
                                                                    // positioned at the aerodynamic center of wing L
  double Vtl = pow(pow(UVWtl(2) - UVWtla(2), 2) + pow(UVWtl(0) - UVWtla(0), 2), 0.5);  // compute Magnitude
  double Gammatl = atan2(UVWtl(2) - UVWtla(2), UVWtl(0) - UVWtla(0));                  // compute Orientation

  // fprintf (stderr, "Vtr: %f Gammatr: %f Vtl: %f Gammatl: %f \n", Vtr, Gammatr, Vtl, Gammatl);
  // plot everything in order to evaluate the results
  //	std::cout << std::endl << "Vtr:" << Vtr << "  Vtl:" << Vtl << std::endl;
  //	std::cout << "Gammatr:" << Gammatr << "  Gammatl:" << Gammatl << std::endl;

  double VetCDf[63] = { 0.025,   0.065452, 0.16654,  0.26951,  0.37366,   0.52852,   0.73279,  0.92854,  1.1099,
                        1.2652,  1.4051,   1.5216,   1.6162,   1.694,     1.7517,    1.7846,   1.8024,   1.7931,
                        1.7549,  1.6829,   1.5829,   1.4625,   1.318,     1.1666,    0.99974,  0.80772,  0.60564,
                        0.42784, 0.2875,   0.023346, 0.012045, 0.0068935, 0.0072707, 0.013511, 0.017643, 0.30916,
                        0.45295, 0.63915,  0.8418,   1.0291,   1.193,     1.3428,    1.4847,   1.6012,   1.6972,
                        1.7637,  1.7964,   1.8013,   1.7799,   1.7438,    1.6823,    1.601,    1.5045,   1.3823,
                        1.2406,  1.0812,   0.8966,   0.69802,  0.49702,   0.35513,   0.25209,  0.14907,  0.051889 };
  double VetCLf[63] = { 0,        0.74461,  0.81047,  0.64392,   0.70064,  0.8229,    0.90449,  0.94542,  0.94146,
                        0.89219,  0.79445,  0.6682,   0.5406,    0.3852,   0.21348,   0.043238, -0.11908, -0.29265,
                        -0.4615,  -0.62107, -0.75584, -0.88146,  -0.98183, -1.0626,   -1.0791,  -1.082,   -0.90438,
                        -0.92431, -0.65444, -0.91363, -0.89073,  -0.26214, 0.3679999, 0.9726,   0.74113,  0.69509,
                        0.97793,  0.92464,  1.0841,   1.0828,    1.0509,   0.96642,   0.86173,  0.73431,  0.59556,
                        0.43358,  0.26364,  0.090584, -0.070567, -0.24276, -0.41334,  -0.56309, -0.68936, -0.81442,
                        -0.90355, -0.94545, -0.94099, -0.89544,  -0.80124, -0.68545,  -0.65111, -0.83993, -0.67012 };
  double VetCLW[63] = { -0.1,     -0.11168, 0.046158, 0.19012,  0.14949,  0.032696, 0.16352,  0.40322,   0.47168,
                        0.43987,  0.41031,  0.39008,  0.36076,  0.29406,  0.19604,  0.084877, -0.036643, -0.17593,
                        -0.34932, -0.52255, -0.62463, -0.64979, -0.64757, -0.75134, -0.92226, -0.8552,   -0.84943,
                        -0.95235, -1.1505,  -0.80517, -0.33892, 0.12202,  0.66824,  1.1202,   1.5215,    1.3454,
                        1.1965,   1.2698,   1.4431,   1.3737,   1.1108,   0.97621,  0.88426,  0.68382,   0.48775,
                        0.35297,  0.19672,  0.019917, -0.11727, -0.2102,  -0.27283, -0.31364, -0.34131,  -0.36589,
                        -0.39616, -0.43407, -0.44923, -0.42199, -0.38794, -0.36149, -0.31994, -0.25637,  -0.17593 };
  double VetCDW[63] = { 0.3,     0.34268, 0.42225,  0.51124,  0.58331, 0.63736, 0.69401, 0.75098, 0.79637,
                        0.82679, 0.84797, 0.87748,  0.9134,   0.93713, 0.95045, 0.96032, 0.9569,  0.93842,
                        0.92269, 0.91245, 0.89517,  0.85707,  0.80044, 0.78136, 0.75541, 0.53444, 0.40511,
                        0.25973, 0.14268, 0.060593, 0.025275, 0.01631, 0.03762, 0.08312, 0.1536,  0.25923,
                        0.43233, 0.5999,  0.69536,  0.81398,  0.89075, 0.95251, 1.0054,  1.0477,  1.0817,
                        1.1101,  1.135,   1.1496,   1.144,    1.1279,  1.1121,  1.0905,  1.0612,  1.0305,
                        1.0027,  0.97769, 0.94374,  0.89018,  0.81981, 0.73368, 0.62861, 0.51045, 0.39123 };
  double VetCDt[63] = { 0.0071,  -0.11395, 0.10658, 0.45137,  0.711,    0.85029,  0.98929,  1.1324,   1.2382,
                        1.3224,  1.3995,   1.4579,  1.5009,   1.5443,   1.581,    1.5978,   1.5993,   1.5894,
                        1.5642,  1.5276,   1.4894,  1.4474,   1.3927,   1.3111,   1.222,    1.1889,   1.0564,
                        0.67454, 0.30154,  0.11121, 0.024927, 0.010853, 0.012969, 0.035073, 0.13043,  0.35371,
                        0.74531, 1.0998,   1.1925,  1.2344,   1.327,    1.4034,   1.4551,   1.4959,   1.5341,
                        1.5695,  1.592,    1.5998,  1.5963,   1.5761,   1.537,    1.494,    1.4497,   1.3875,
                        1.3087,  1.2227,   1.1104,  0.96413,  0.82972,  0.67893,  0.39398,  0.054892, -0.1234 };
  double VetCLt[63] = { 0,        0.3994,   0.55274,  0.58755,  0.62718,  0.70302,  0.76193,  0.7903,    0.79189,
                        0.75553,  0.68102,  0.59354,  0.49502,  0.36827,  0.23541,  0.114,    -0.061171, -0.30075,
                        -0.49385, -0.61787, -0.73228, -0.84611, -0.94327, -1.0036,  -1.0128,  -0.97063,  -0.92622,
                        -0.99894, -1.0372,  -0.74074, -0.43079, -0.12844, 0.18012,  0.48016,  0.79568,   1.0575,
                        0.97331,  0.92997,  0.98023,  1.0149,   0.99685,  0.92894,  0.82764,  0.71268,   0.59908,
                        0.46781,  0.26059,  0.024942, -0.13593, -0.25643, -0.39112, -0.51329, -0.60848,  -0.69516,
                        -0.76483, -0.79388, -0.78733, -0.75437, -0.69013, -0.61666, -0.58454, -0.53872,  -0.35292 };

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // The vector of aerodynamic coefficients CD and CL are represented from -180 to 180 with data for each 0.1 rad/s
  // calculo dos coef. da fuselagem
  double Val = ((-Alphaf + 3.1416) / 0.1);  // start by computing the index of the coefficient
  int Index = floor(Val);                   // round it to the floor value
  double Proporcao = Val - Index;           // compute the proportion between the floor value and the next one

  double CDfa = VetCDf[Index] + Proporcao * (VetCDf[Index + 1] - VetCDf[Index]);  // make a linear interpolation between
                                                                                  // the floor value and the next one
  double CLfa = VetCLf[Index] + Proporcao * (VetCLf[Index + 1] - VetCLf[Index]);  // make a linear interpolation between
                                                                                  // the floor value and the next one
  CDfa = CDfa + 0.1;

  Val = ((-Betaf + 3.1416) / 0.1);
  Index = floor(Val);
  Proporcao = Val - Index;

  double CDfs = VetCDf[Index] + Proporcao * (VetCDf[Index + 1] - VetCDf[Index]);
  double CLfs = VetCLf[Index] + Proporcao * (VetCLf[Index + 1] - VetCLf[Index]);
  CDfs = CDfs + 0.1;

  // calculo dos coef. da asa
  Val = ((-Alphawr + 3.1416) / 0.1);
  Index = floor(Val);
  Proporcao = Val - Index;

  double CDWr = VetCDW[Index] + Proporcao * (VetCDW[Index + 1] - VetCDW[Index]);
  double CLWr = VetCLW[Index] + Proporcao * (VetCLW[Index + 1] - VetCLW[Index]);

  Val = ((-Alphawl + 3.1416) / 0.1);
  Index = floor(Val);
  Proporcao = Val - Index;

  double CDWl = VetCDW[Index] + Proporcao * (VetCDW[Index + 1] - VetCDW[Index]);
  double CLWl = VetCLW[Index] + Proporcao * (VetCLW[Index + 1] - VetCLW[Index]);

  // Calculo dos coeficientes da Cauda
  Val = ((-Gammatr + 3.1416) / 0.1);
  Index = floor(Val);
  Proporcao = Val - Index;

  double CDTr = VetCDt[Index] + Proporcao * (VetCDt[Index + 1] - VetCDt[Index]);
  double CLTr = VetCLt[Index] + Proporcao * (VetCLt[Index + 1] - VetCLt[Index]);

  Val = ((-Gammatl + 3.1416) / 0.1);
  Index = floor(Val);
  Proporcao = Val - Index;

  double CDTl = VetCDt[Index] + Proporcao * (VetCDt[Index + 1] - VetCDt[Index]);
  double CLTl = VetCLt[Index] + Proporcao * (VetCLt[Index + 1] - VetCLt[Index]);

  // fprintf (stderr, "CDfa: %f CLfa: %f CDfs: %f CLfs: %f CDWr: %f CLWr: %f CDWl: %f CLWl: %f CDTr: %f CLTr: %f CDTl:
  // %f CLTl: %f\n", CDfa, CLfa, CDfs, CLfs, CDWr, CLWr, CDWl, CLWl, CDTr, CLTr, CDTl, CLTl);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  Eigen::MatrixXd JacobianFuselage(3, 8);
  JacobianFuselage << Eigen::MatrixXd::Zero(3, 2), -RI_B * SkewSymmetricMatrix(DBf) * Wn,
      Eigen::MatrixXd::Identity(3, 3);

  double Clxz = CLfa;
  double Cdxz = CDfa;
  double Clxy = CLfs;
  double Cdxy = CDfs;

  // Rotation matrices that map from wind to the aerodynamic center frame of fuselagem
  RAlphaf << cos(-Alphaf), 0, sin(-Alphaf), 0, 1, 0, -sin(-Alphaf), 0, cos(-Alphaf);

  RBetaf << cos(-Betaf), -sin(-Betaf), 0, sin(-Betaf), cos(-Betaf), 0, 0, 0, 1;

  // aerodynamic forces actuating on the fuselagem expressed on the wind frame
  double Fd_xz_F = -0.5 * rho * (Vfxz * Vfxz) * sfFrontal * CDfa;
  double Fl_xz_F = 0.5 * rho * (Vfxz * Vfxz) * sfFrontal * CLfa;
  double Fd_xy_F = -0.5 * rho * (Vfxy * Vfxy) * sfLateral * CDfs;
  double Fl_xy_F = 0.5 * rho * (Vfxy * Vfxy) * sfLateral * CLfs;

  // Rotation matrices that map from wind frame to the aerodynamic center frame
  RAlphaf << cos(-Alphaf), 0, sin(-Alphaf), 0, 1, 0, -sin(-Alphaf), 0, cos(-Alphaf);

  RBetaf << cos(-Betaf), -sin(-Betaf), 0, sin(-Betaf), cos(-Betaf), 0, 0, 0, 1;

  // Make the vector of aerodynamic forces applied by the fuselage
  Fxz << Fd_xz_F, 0, Fl_xz_F;
  Fxy << Fd_xy_F, Fl_xy_F, 0;

  // map the forces from the wind frame to the body frame
  Forca_F = RAlphaf * Fxz + RBetaf * Fxy;

  // Computing the generalized forces vector due to the fuselagem
  Eigen::VectorXd FFuzelagem(8);
  FFuzelagem = JacobianFuselage.transpose() * RI_B * Forca_F;

  //%-----------Computing the generalized forces due to the V-tail-----------------

  Eigen::MatrixXd JacobianVTailR(3, 8);
  JacobianVTailR << Eigen::MatrixXd::Zero(3, 2), -RI_B * SkewSymmetricMatrix(DBtr) * Wn,
      Eigen::MatrixXd::Identity(3, 3);

  Eigen::MatrixXd JacobianVTailL(3, 8);
  JacobianVTailL << Eigen::MatrixXd::Zero(3, 2), -RI_B * SkewSymmetricMatrix(DBtl) * Wn,
      Eigen::MatrixXd::Identity(3, 3);

  double kvtr = 0.5 * rho * (Vtr * Vtr) * st;
  double kvtl = 0.5 * rho * (Vtl * Vtl) * st;

  // Aerodynamic forces applied by the V-tail surfaces
  double Fd_TR = -kvtr * CDTr;
  double Fl_TR = kvtr * CLTr;
  double Fd_TL = -kvtl * CDTl;
  double Fl_TL = kvtl * CLTl;

  // Rotation matrices that map from wind to the aerodynamic center frame
  RGammatr << cos(-Gammatr), 0, sin(-Gammatr), 0, 1, 0, -sin(-Gammatr), 0, cos(-Gammatr);

  RGammatl << cos(-Gammatl), 0, sin(-Gammatl), 0, 1, 0, -sin(-Gammatl), 0, cos(-Gammatl);

  // Make the vector of aerodynamic forces applied by the V-tail surfaces
  ForcaTailR << Fd_TR, 0, Fl_TR;
  ForcaTailL << Fd_TL, 0, Fl_TL;

  // map the forces from the wind frame to the body frame
  F_TailR = RBmiR * RGammatr * ForcaTailR;
  F_TailL = RBmiL * RGammatl * ForcaTailL;

  double Cr = 0.85;
  Eigen::VectorXd FVTailUa(8);
  Eigen::MatrixXd BAeroVT(8, 2);
  FVTailUa = JacobianVTailR.transpose() * RI_B * F_TailR + JacobianVTailL.transpose() * RI_B * F_TailL;
  BAeroVT << JacobianVTailR.transpose() * RI_B * RBmiR * RGammatr * az * kvtr * Cr,
      JacobianVTailL.transpose() * RI_B * RBmiL * RGammatl * az * kvtl * Cr;
  // std::cout << "BAeroVT - Rows: " << BAeroVT.rows() << " Cols: " <<  BAeroVT.cols()  << std::endl;

  //%---------Computing the generalized forces applied by wings-----------------

  Eigen::MatrixXd JacobianWingR(3, 8);
  JacobianWingR << Eigen::MatrixXd::Zero(3, 2), -RI_B * SkewSymmetricMatrix(DBwr) * Wn, Eigen::MatrixXd::Identity(3, 3);

  Eigen::MatrixXd JacobianWingL(3, 8);
  JacobianWingL << Eigen::MatrixXd::Zero(3, 2), -RI_B * SkewSymmetricMatrix(DBwl) * Wn, Eigen::MatrixXd::Identity(3, 3);

  double kwr = 0.5 * rho * (Vwr * Vwr) * sw;
  double kwl = 0.5 * rho * (Vwl * Vwl) * sw;
  // Aerodynamic forces applied by the wings
  double Fd_Wr = -kwr * CDWr;
  double Fl_Wr = kwr * CLWr;
  double Fd_Wl = -kwl * CDWl;
  double Fl_Wl = kwl * CLWl;

  // Rotation matrices that map from wind to the aerodynamic center frame
  RAlphawr << cos(-Alphawr), 0, sin(-Alphawr), 0, 1, 0, -sin(-Alphawr), 0, cos(-Alphawr);

  RAlphawl << cos(-Alphawl), 0, sin(-Alphawl), 0, 1, 0, -sin(-Alphawl), 0, cos(-Alphawl);

  // Make the vector of aerodynamic forces applied by the fuselage
  FWrxz << Fd_Wr, 0, Fl_Wr;
  FWlxz << Fd_Wl, 0, Fl_Wl;

  // map the forces from the wind frame to the body frame
  Forca_Wr = RBwdR * RAlphawr * FWrxz;
  Forca_Wl = RBwdL * RAlphawl * FWlxz;

  double Ca = 0.511;
  Eigen::VectorXd FWingsUa(8);
  Eigen::MatrixXd BAeroWings(8, 2);

  FWingsUa = JacobianWingR.transpose() * RI_B * Forca_Wr + JacobianWingL.transpose() * RI_B * Forca_Wl;
  BAeroWings << JacobianWingR.transpose() * RI_B * RBwdR * RAlphawr * az * kwr * Ca,
      JacobianWingL.transpose() * RI_B * RBwdL * RAlphawl * az * kwl * Ca;
  // std::cout << "BAeroWings - Rows: " << BAeroWings.rows() << " Cols: " <<  BAeroWings.cols()  << std::endl;
  BAeroFUa << BAeroWings, BAeroVT, FFuzelagem + FVTailUa + FWingsUa;  // Forçaa generalizada;
  // std::cout << "passei aqui!" << std::endl;
  //       BAero << 0, 0, 0, 0,
  //		0, 0, 0, 0,
  //		0, 0, 0, 0,
  //		0, 0, 0, 0,
  //		0, 0, 0, 0,
  //		0, 0, 0, 0,
  //		0, 0, 0, 0,
  //		0, 0, 0, 0;

  return BAeroFUa;
}
