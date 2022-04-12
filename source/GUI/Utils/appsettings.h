/*!
 * \file appsettings.h
 * @author Júnio Eduardo de Morais Aquino
 * @date 2019/02/21
 */

#ifndef APPSETTINGS_H
#define APPSETTINGS_H

#include <QObject>
#include <QDir>
#include <QProcessEnvironment>
#include <QString>
#include <QSettings>

//! The key used to access the GazeboModelPath.
static const QString GAZEBO_MODEL_PATH_KEY = "GAZEBO_MODEL_PATH";
//! The key used to access the ProvantRosPath.
static const QString PROVANT_ROS_KEY = "PROVANT_ROS";
//! The key used to access the TiltStrategiesPath.
static const QString TILT_STRATEGIES_KEY = "TILT_STRATEGIES";
//! The key used to access the TiltMatlabPath.
static const QString TILT_MATLAB_KEY = "TILT_MATLAB";
//! The key used to access the TiltProjectPath.
static const QString TILT_PROJECT_KEY = "TILT_PROJECT";
//! The key used to access the ProvantDatabasePath.
static const QString PROVANT_DATABASE_KEY = "PROVANT_DATABASE";
//! The key used to access the DirRosPath.
static const QString DIR_ROS_KEY = "DIR_ROS";
//! The key used to access the parameter ControlStrategiesPath
static const QString CONTROL_STRATEGIES_SOURCE_KEY = "CONTROL_STRATEGIES_"
                                                     "SOURCE";
//! The key used to access the ros version parameter
static const QString ROS_VERSION_KEY = "ROS_VERSION";
//! The path to the ros installation
static const QString ROS_PATH_KEY = "ROS_PATH";
//! The key used to access the path to the package containg the simulation
//! worlds
static const QString WORLDS_PACKAGE_KEY = "PROVANT_SIMULATOR_WORLDS";

/*!
 * \brief The AppSettings class is used to access all the paths used in the
 * application.
 *
 * This class was created as a single source of truth for the settings used in
 * the application.
 *
 * Each setting in this class should provide the following methods:
 *  - A method to access the parameter without checking if its configuration is
 * valid. This option is used to display the current value of the parameter to
 * the user.  Most existing parameters use the getDirectoryPath() method.
 *  - A method to access the parameter and check if its value is correct. In
 *  case of an error, this should be logged and reported to the user. Most
 *  existing parameters use the checkDirectoryPath() method.
 *  - A method to return the default value for the parameter. For parameters
 *  regarding the location of directories, this value is the original path on
 *  a fresh install.
 *  - A method for setting a new value for the parameter. Most existing
 * parameters use the setDirectoryPath() method.
 *
 * A new parameter should also define a static const QString containing the key
 * used to access the parameter value in the QSettings and, if needed,
 * in environment variables.
 *
 * Please note that an environment variable with the parameter value should
 * only be created if this value is necessary to at least one program started
 * by the GUI.
 *
 * Don't forget to update the methods checkAllParametersSet(),
 * applyValuesToEnvironmentVariables(), restoreDefaults(),
 * applyDefaultsToUndefinedParameters() and getEnvironmentVariables()
 * to include the new parameter.
 *
 * The access to settings first checks if the value is present in the
 * QSettings object; if the value isn't set, the environment variable with the
 * same name is queried. If both the previous attempts return an empty value
 * (meaning the value isn't set), the method returns the default.
 *
 * Error messages exhibited to the user should report what the parameter
 * represents in the application and the steps to fix the error.
 *
 * Eror messages logged should report a problem with the access of a value.
 *
 */
class AppSettings : public QObject
{
  Q_OBJECT
public:
  AppSettings(QObject* parent = nullptr);

  const QString getGazeboModelPathUncheked() const;
  const QString getGazeboModelPath() const;
  const QString getGazeboModelPathDefault() const;
  bool setGazeboModelPath(const QString& path);

  const QString getProvantRosPathUnchecked() const;
  const QString getProvantRosPath() const;
  const QString getProvantRosPathDefault() const;
  bool setProvantRosPath(const QString& path);

  const QString getTiltStrategiesPathUnchecked() const;
  const QString getTiltStrategiesPath() const;
  const QString getTiltStrategiesPathDefault() const;
  bool setTiltStrategiesPath(const QString& path);

  const QString getTiltMatlabPathUnchecked() const;
  const QString getTiltMatlabPath() const;
  const QString getTiltMatlabPathDefault() const;
  bool setTiltMatlabPath(const QString& path);

  const QString getTiltProjectPathUnchecked() const;
  const QString getTiltProjectPath() const;
  const QString getTiltProjectPathDefault() const;
  bool setTiltProjectPath(const QString& path);

  const QString getProvantDatabasePathUnchecked() const;
  const QString getProvantDatabasePath() const;
  const QString getProvantDatabasePathDefault() const;
  bool setProvantDatabasePath(const QString& path);

  const QString getCatkinWorkspacePathUncheked() const;
  const QString getCatkinWorkspacePath() const;
  const QString getCatkinWorkspacePathDefault() const;
  bool setCatkinWorkspacePath(const QString& path);

  const QString getControlStrategiesPathUnchecked() const;
  const QString getControlStrategiesPath() const;
  const QString getControlStrategiesPathDefault() const;
  bool setControlStrategiesPath(const QString& path);

  const QString getRosVersionUnchecked() const;
  const QString getRosVersion() const;
  const QString getRosVersionDefault() const;
  bool setRosVersion(const QString& version);

  const QString getRosPathUnchecked() const;
  const QString getRosPath() const;
  const QString getRosPathDefault() const;
  bool setRosPath(const QString& path);

  /**
   * @brief getWorldsPackagePathUnchecked
   * This function doesn't check if the currently configured value is valid.
   * Use this method only when extremamely necessary, such as in displaying
   * the current configured value for user modification, otherwise prefer
   * the getWorldsPackagePath() method.
   *
   * @return Path to the ROS package containg the Gazebo worlds
   */
  const QString getWorldsPackagePathUnchecked() const;
  /**
   * @brief getWorldsPackagePath Returns the path to the ROS package
   * containing the gazebo worlds contained in the ProVANT simulator.
   * @return
   */
  const QString getWorldsPackagePath() const;
  /**
   * @brief getWorldsPackagePathDefault Returns the default path to the
   * ROS package that contains the worlds.
   * @return Path to the ROS package containg the Gazebo worlds.
   */
  const QString getWorldsPackagePathDefault() const;
  /**
   * @brief setWorldsPackagePath Update the value for the path of the package
   * containg the simulation worlds.
   * @param path New path value to the worlds package.
   * @return True if the package was succesfully updated and false otherwise.
   */
  bool setWorldsPackagePath(const QString& path);

  bool checkAllParametersSet() const;
  void applyValuesToEnvironmentVariables();
  void applyDefaultsToUndefinedParameters();
  void restoreDefaults();
  QProcessEnvironment getEnvironmentVariables() const;

protected:
  QString getEnvironmentVariable(const QString& key,
                                 const QString& default_ = QString("")) const;
  QString getValueFromKey(const QString& key,
                          const QString& default_ = QString("")) const;
  QString getErrorCorrectionMessage(const QString& key) const;

  const QString getDirectoryPath(const QString& key,
                                 const QString& defaultValue) const;
  const QString checkDirectoryPath(const QString& path,
                                   const QString& defaultValue,
                                   const QString& errorMessage) const;
  bool setDirectoryPath(const QString& key, const QString& value,
                        bool setEnrionmentVariable = false);
  bool setEnvironmentVariable(const QString& key, const QString& value);

private:
  //! The QSettings object used to access and set the parameters.
  QSettings settings;

  //! The message shown in the title of the error dialog box.
  const QString _criticalMsgTitle = tr("Path Error");
};

#endif  // APPSETTINGS_H
