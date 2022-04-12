/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file integrator.h
 * @brief This file contains the declaration of the Integrator class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_SIMULATOR_MATH_UTILS_INTEGRATOR_H
#define PROVANT_SIMULATOR_MATH_UTILS_INTEGRATOR_H

/**
 * @brief The Integrator class implements a numerical integrator based on the
 * first order trapezoidal rule, for a grid with a constant step.
 *
 * This integrator provides a first order approximation to the numerical integral
 * of a variable sampled with a fixed step time.
 * 
 * When a new sample is received trough the update method, the integral value
 * up to current step is updated according to the following rule:
 * \f[
 *  value := value + \Delta t \frac{f[k] + f[k - 1]}{2},
 * \f]
 * where \f$f[k]\f$ is the value of the variable at the current step time, 
 * \f$f[k - 1]\f$ is the value of the variable at the previous step time,
 * and \f$\Delta t\f$ is the value step time interval as indicated by the
 * step_time parameter.
 * 
 * The value of the sample at the previous step time is automatically updated
 * and does not need to be managed by the user.
 * 
 * To use this class, include the provant_simulator_math_utils/integrator.h header
 * and create a variable for the type you wish to use.
 * For example, for a integrator for double precision floating point variables,
 * use the following code:
 * @code{.cpp}
 * #include <provant_simulator_math_utils/integrator.h>
 * 
 * double integrate()
 * {
 *    Integrator<double> integrator(1.0);
 *    
 *    integrator.update(1.0);
 *    integrator.update(2.0);
 *    integrator.update(3.0);
 *    integrator.update(4.0);
 *    integrator.update(5.0);
 * 
 *    return integrator.value();
 * }
 * 
 * The above example implements the integral of the function f(x) = x 
 * evaluted at a constant step of 1.
 * 
 * @endcode
 * 
 * This class can also be used with Eigen to allow integration of a vector, as
 * shown in the following code example:
 * @code{.cpp}
 * #include <Eigen/Eigen>
 * ...
 * Eigen::VectorXd zero(5);
 * zero << 0, 0, 0, 0, 0;
 * Integrator<Eigen::VectorXd> integrator(1.0, zero, zero);
 *
 * Eigen::VectorXd sample(5);
 * sample << 1, 2, 3, 4, 5;
 * integrator.update(sample);
 * @endcode
 *
 */
template <typename T, typename ScalarType = double>
class Integrator
{
public:
  /**
   * @brief Construct a new Integrator object.
   *
   * @param step_time Value of the step_time used by the integrator. The samples
   * passed to the integrator must be sampled at a constant interval with
   * value equal to the step_time of the integrator.
   * @param initial_value Initial value of the integrator. The default is zero.
   * Note that for custom types you must explicitly set this parameter.
   * @param zero_value Indicates a value of zero. Used during the integrator
   * resseting procedure. For custom types such as vectors you must explicitly
   * set this value.
   */
  Integrator(ScalarType step_time, T initial_value = static_cast<T>(0.0), T zero_value = static_cast<T>(0.0))
    : _step_time(step_time)
    , _initial_value(initial_value)
    , _previousValue(zero_value)
    , _integral(initial_value)
    , _zero(zero_value)
  {
  }

  // Force creation of the copy and move constructors and operators.
  Integrator(const Integrator& other) = default;
  Integrator(Integrator&& other) = default;
  Integrator& operator=(const Integrator& other) = default;
  Integrator& operator=(Integrator&& other) = default;

  /**
   * @brief Destroy the Integrator object.
   */
  virtual ~Integrator()
  {
  }

  /**
   * @brief Returns the step time used by the integrator.
   *
   * @return const ScalarType&
   */
  const ScalarType& stepTime() const
  {
    return _step_time;
  }

  /**
   * @brief Returns the initial value of the integrator.
   *
   * @sa setInitialValue().
   *
   * @return const T&
   */
  const T& initialValue() const
  {
    return _initial_value;
  }

  /**
   * @brief Set the initial value of the integrator.
   *
   * This is the initial value of integral up to step zero.
   * Note that if you set the value of the initial value after the initialization
   * of the object, you must call the reset() method before the initial integrand
   * value is updated.
   *
   * @sa initialValue().
   * @sa reset().
   *
   * @param initialValue New value of the integral up to step zero.
   */
  virtual void setInitialValue(const T& initialValue)
  {
    _initial_value = initialValue;
  }

  /**
   * @brief Resets the integrator.
   *
   * Resets the integral value to the initial value of the integrator, and set
   * the value of the sample received in the previous step to zero.
   *
   * @sa setInitialValue().
   */
  virtual void reset()
  {
    _integral = _initial_value;
    _previousValue = _zero;
  }

  /**
   * @brief Returns the value of the sample received by the integrator in the
   * previous step.
   *
   * @return const T&
   */
  const T& previousValue() const
  {
    return _previousValue;
  }

  /**
   * @brief Returns the current value of the integral.
   *
   * @return const T&
   */
  const T& value() const
  {
    return _integral;
  }

  /**
   * @brief Updates the integrator with a new sample value and returns the value
   * of the integral up to the current step.
   *
   * @param sample New sample to include in the integrator.
   * @return const T& Integral value up to the current step.
   */
  const T& update(const T& sample)
  {
    updateIntegral(sample);
    return _integral;
  }

protected:
  /**
   * @brief Helper method used to calculate the value of the integral.
   *
   * Receives the value of a new sample, calculates the value of the integral for
   * the current step using the trapezoidal rule, and adds this value to the
   * integral value.
   *
   * @param sample New sample received by the integrator.
   */
  virtual void updateIntegral(const T& sample)
  {
    // Updates the integral summation
    _integral += _step_time * (_previousValue + sample) / static_cast<ScalarType>(2.0);
    // Updates the previous
    _previousValue = sample;
  }

private:
  //! Value of the step time used in the integrator.
  ScalarType _step_time;
  //! Initial value of the integrator.
  T _initial_value;

  //! Current value of the integral.
  T _integral;
  //! Value of the previous sample passed to the integrator.
  T _previousValue;

  //! Zero Value, used to reset the integral value and the previous step value for custom types such as Eigen::Vector.
  const T _zero;
};

#endif  // PROVANT_SIMULATOR_MATH_UTILS_INTEGRATOR_H
