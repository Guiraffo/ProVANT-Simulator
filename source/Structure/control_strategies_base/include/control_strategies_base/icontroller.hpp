/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the declaration of the interface Icontroller.
 * 
 * This interface is specialized by every control strategy developed within the
 * ProVANT Simulator package.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef ICONTROLLER_HPP
#define ICONTROLLER_HPP

#include "simulator_msgs/SensorArray.h"

/**
 * @brief The Icontroller is the interface for implementation of every control
 * strategy used in the ProVANT simulator.
 * 
 * This interface defines the methods that are used to initialize and execute
 * the control strategy internal loop, and methods that are used to determine
 * the references, state vector, error vector at every step time. These methods
 * are used to generate the log files that can be latter imported in Matlab to
 * generate graphs, calculate performance indexes or any other desired evalution
 * from the simulation resulsts.
 */
class Icontroller
{
public:
  /**
   * @brief Construct a new Icontroller object.
   * 
   * As this is an abstract class, no initialization is done here. However for
   * compatibility purposes with future versions. Please be sure to call this
   * constructor from your specialized class.
   * 
   * For example, if you have a class named NewController, do this in its
   * constructor:
   * @code{.cpp}
   * class NewController : public Icontroller {
   * 	NewController() : Icontroller() {}
   * };
   * @endcode
   */
  Icontroller(){};

  /**
   * @brief Destroy the Icontroller object.
   */
  virtual ~Icontroller(){};

  /**
   * @brief This method is called by the controller to initialize any necessary
   * configurations for the control strategy.
   * 
   * For example, this method could be used to initialize the gain parameters
   * of the implemented control strategy.
   */
  virtual void config() = 0;

	/**
	 * @brief The execute method is called every step time to execute the control
   * law implemented by derived classes.
   * 
   * This method receives an array of sensor messages containing the data read
   * from the sensors used in the simulation for a given control strategey
   * configuration, and must execute the control loop for the control strategy
   * and return a vector object with the desired control inputs to be applied
   * in the system.
	 * 
	 * @param arraymsg Array of simulator messages. This parameter contains the
   * messages sent by every sensor that is configured for the control strategy
   * implementation. (Note that this configuration can be changed used the
   * ProVANT Simulator GUI, or in the config.xml file for the model relative
   * to this control strategy.).
   *  
	 * @return std::vector<double> Vector of control inputs to be applied to the
   * system.
	 */
  virtual std::vector<double> execute(simulator_msgs::SensorArray arraymsg) = 0;
  
  /**
   * @brief The Reference method is called every step time to generate the 
   * reference values for the current step time.
   * 
   * This method must calculate and return a vector containing the vector of
   * references for a given step time of the system.
   * 
   * Please note that if you need to obtain the current simulation time t, the
   * following example code can be used:
   * @code{.cpp}
   *   ros::Time time = ros::Time::now();
   *   double t = time.toSec();
   * @endcode
   * 
   * In the above example, t will contain the current simulation time in 
   * seconds.
   * 
   * @return std::vector<double> Vector of references used in the system.
   */
  virtual std::vector<double> Reference() = 0;

  /**
   * @brief The Error method is called every step time to calculate and return
   * the error vector for the current step time.
   * 
   * This method must calculate and return the error vector for the current step
   * time. Usually this vector is calculated as the state vector minus the
   * reference vector, but if you control strategy uses another format, fell
   * free to use it.
   * 
   * @return std::vector<double> Vector of error at the current step time.
   */
  virtual std::vector<double> Error() = 0;
  // method built to allow user show state data to be printed
  /**
   * @brief The State method is called every step time to calculate and return
   * the state vector for the current step time.
   * 
   * This method must calculate and return the state vector of the system at
   * the current step time.
   * 
   * @return std::vector<double> State vector at the current step time.
   */
  virtual std::vector<double> State() = 0;
};

// to implement dynamics libraries
extern "C" {
typedef Icontroller* create_t();
typedef void destroy_t(Icontroller*);
}
#endif //ICONTROLLER_HPP
