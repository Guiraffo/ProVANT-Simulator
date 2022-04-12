/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file saturator.h
 * @brief This file contains the implementation of the Saturator class template.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_SATURATOR_H
#define PROVANT_SATURATOR_H

#include <algorithm>

namespace provant
{
namespace math_utils
{
template <typename T>
/**
 * @brief The Saturator class enables users to saturate a given value between a inferir and superior limit.
 *
 * The Saturator can also be enabled or disabled at run time.
 */
class Saturator
{
public:
  /**
   * @brief Construct a new Saturator object.
   *
   * @param limInf The saturation inferior limit.
   * @param limSup The saturation superior limit.
   * @param enabled Indicates if the saturation is enabled or not.
   */
  Saturator(T limInf, T limSup, bool enabled = true) : _limInf(limInf), _limSup(limSup), _enabled(enabled)
  {
  }

  /**
   * @brief Returns the saturation inferior limit.
   *
   * @return T
   */
  T getLimInf() const
  {
    return _limInf;
  }

  /**
   * @brief Returns the saturation superior limit.
   *
   * @return T
   */
  T getLimSup() const
  {
    return _limSup;
  }

  /**
   * @brief Indicates if the saturation is enabled or not.
   *
   * @return true If saturation is enabled.
   * @return false Otherwise.
   */
  bool isEnabled() const
  {
    return _enabled;
  }

  /**
   * @brief Enables or disables the saturation.
   *
   * @param enabled
   */
  void setEnabled(bool enabled)
  {
    _enabled = enabled;
  }

  /**
   * @brief Saturate the given value between the inferior and supperior limits of the class.
   *
   * If saturation is disabled, the value itself is returned.
   *
   * @param value Value to saturate.
   * @return T Saturated value.
   */
  T saturate(T value)
  {
    return _enabled ? std::min(std::max(value, _limInf), _limSup) : value;
  }

private:
  //! Stores the saturation lower limit.
  T _limInf;
  //! Stores the starution superior limit.
  T _limSup;
  //! Indicates if saturation is enabled or not.
  bool _enabled;
};

}  // namespace math_utils
}  // namespace provant

#endif  // PROVANT_SATURATOR_H
