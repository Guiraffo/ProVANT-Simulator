/*!
 * @file appsettings.cpp
 * @author Júnio Eduardo de Morais Aquino
 * @date 2019/02/21
 */

#include "appsettings.h"

#include <QMessageBox>
#include <QTextCodec>

/**
 * @brief AppSettings::AppSettings
 * @param parent
 *
 * Initializes the AppSettings class and pass the parent parameter to the
 * base class for correct initialization.
 */
AppSettings::AppSettings(QObject *parent) : QObject(parent)
{

}

/**
 * @brief AppSettings::getGazeboModelPathUncheked
 * @return The path to GAZEBO_MODEL_PATH parameter.
 *
 * This function doesn't check if the currently configured value is valid.
 * Use this method only when extremamely necessary, such as in displaying
 * the current configured value for user modification, otherwise prefer
 * the getGazeboModelPath() method.
 */
const QString AppSettings::getGazeboModelPathUncheked() const
{
    return getDirectoryPath(GAZEBO_MODEL_PATH_KEY,
                            getGazeboModelPathDefault());
}

/**
 * @brief AppSettings::getGazeboModelPath
 * @return The path to GAZEBO_MODEL_PATH paramter.
 *
 * This function first tests if the returned value points to a valid system
 * path and if not, shows an error message informing the user about this error
 * and how to fix it.
 */
const QString AppSettings::getGazeboModelPath() const
{
    return checkDirectoryPath(GAZEBO_MODEL_PATH_KEY,
                            getGazeboModelPathDefault(),
                            tr("The directory containing the models used in "
                               "the simulator could not be opened."));
}

/**
 * @brief AppSettings::getGazeboModelPathDefault
 * @return The default value for the GAZEBO_MODEL_PATH parameter.
 */
const QString AppSettings::getGazeboModelPathDefault() const
{
    return QDir::home().absoluteFilePath("catkin_ws/src/"
                                         "ProVANT-Simulator_Developer/"
                                         "source/Database/models");
}

/**
 * @brief AppSettings::setGazeboModelPath
 * @param path The new path of the GAZEBO_MODEL_PATH parameter.
 * @return True if the informed value points to a valid system path and false
 * if the path is invalid.
 *
 *
 */
bool AppSettings::setGazeboModelPath(const QString &path)
{
    return setDirectoryPath(GAZEBO_MODEL_PATH_KEY, path, true);
}

/**
 * @brief AppSettings::getProvanRosPathUnchecked
 * @return The path to the PROVANT_ROS parameter.
 *
 * This function doesn't check if the currently configured value is valid.
 * Use this method only when extremamely necessary, such as in displaying
 * the current configured value for user modification, otherwise prefer
 * the getProvantRosPath() method.
 */
const QString AppSettings::getProvantRosPathUnchecked() const
{
    return getDirectoryPath(PROVANT_ROS_KEY,
                            getProvantRosPathDefault());
}

/**
 * @brief AppSettings::getProvantRosPath
 * @return The path to the PROVANT_ROS parameter.
 *
 * This function first tests if the returned value points to a valid system
 * path and if not, shows an error message informing the user about this error
 * and how to fix it.
 */
const QString AppSettings::getProvantRosPath() const
{
    return checkDirectoryPath(PROVANT_ROS_KEY,
                            getProvantRosPathDefault(),
                            tr("The path to the src directory of the catkin "
                               "workspace could not be opened."));
}

/**
 * @brief AppSettings::getProvantRosPathDefault
 * @return The default path of the PROVANT_ROS parameter.
 */
const QString AppSettings::getProvantRosPathDefault() const
{
    return QDir::home().absoluteFilePath("catkin_ws/src");
}

/**
 * @brief AppSettings::setProvantRosPath
 * @param path The new path to the PROVANT_ROS parameter.
 * @return True if the informed value points to a valid system path and false
 * if the path is invalid.
 */
bool AppSettings::setProvantRosPath(const QString &path)
{
    return setDirectoryPath(PROVANT_ROS_KEY, path, true);
}

/**
 * @brief AppSettings::getTiltStratigiesPathUnchecked
 * @return The path to TILT_STRATEGIES parameter.
 *
 * This function doesn't check if the currently configured value is valid.
 * Use this method only when extremamely necessary, such as in displaying
 * the current configured value for user modification, otherwise prefer
 * the getTiltStrategiesPath() method.
 */
const QString AppSettings::getTiltStrategiesPathUnchecked() const
{
    return getDirectoryPath(TILT_STRATEGIES_KEY,
                            getTiltStrategiesPathDefault());
}

/**
 * @brief AppSettings::getTiltStrategiesPath
 * @return The path to the TILT_STRATEGIES parameter.
 *
 * This function first tests if the returned value points to a valid system
 * path and if not, shows an error message informing the user about this error
 * and how to fix it.
 */
const QString AppSettings::getTiltStrategiesPath() const
{
    return checkDirectoryPath(TILT_STRATEGIES_KEY,
                            getTiltStrategiesPathDefault(),
                            tr("The path to the folder containing the compiled "
                               "binaries of the control strategies could not "
                               "be opened."));
}

/**
 * @brief AppSettings::getTiltStrategiesPathDefault
 * @return The default path to the TILT_STRATEGIES parameter.
 */
const QString AppSettings::getTiltStrategiesPathDefault() const
{
    return QDir::home().absoluteFilePath("catkin_ws/devel/lib");
}

/**
 * @brief AppSettings::setTiltStrategiesPath
 * @param path The new value for the TILT_STRATEGIES parameter.
 * @return True if the informed value points to a valid system path and false
 * if the path is invalid.
 */
bool AppSettings::setTiltStrategiesPath(const QString &path)
{
    return setDirectoryPath(TILT_STRATEGIES_KEY, path, true);
}

/**
 * @brief AppSettings::getTiltMatlabPathUnchecked
 * @return The path for the TITL_MATLAB parameter.
 *
 * This function doesn't check if the currently configured value is valid.
 * Use this method only when extremamely necessary, such as in displaying
 * the current configured value for user modification, otherwise prefer
 * the getTiltMatlabPath() method.
 */
const QString AppSettings::getTiltMatlabPathUnchecked() const
{
    return getDirectoryPath(TILT_MATLAB_KEY,
                            getTiltMatlabPathDefault());
}

/**
 * @brief AppSettings::getTiltMatlabPath
 * @return The value to the TILT_MATLAB parameter.
 *
 * This function first tests if the returned value points to a valid system
 * path and if not, shows an error message informing the user about this error
 * and how to fix it.
 */
const QString AppSettings::getTiltMatlabPath() const
{
    return checkDirectoryPath(TILT_MATLAB_KEY,
                            getTiltMatlabPathDefault(),
                            tr("The path to the folder containing the files "
                               "necessary to the Matlab logging of the files "
                               "could not be opened."));
}

/**
 * @brief AppSettings::getTiltMatlabPathDefault
 * @return The default value to the TILT_MATLAB parameter.
 */
const QString AppSettings::getTiltMatlabPathDefault() const
{
    return QDir::home().absoluteFilePath("catkin_ws/src/"
                                         "ProVANT-Simulator_Developer/"
                                         "source/Structure/Matlab/");
}

/**
 * @brief AppSettings::setTiltMatlabPath
 * @param path The new value for the TILT_MATLAB parameter.
 * @return True if the informed value points to a valid system path and false
 * if the path is invalid.
 */
bool AppSettings::setTiltMatlabPath(const QString &path)
{
    return setDirectoryPath(TILT_MATLAB_KEY, path, true);
}

/**
 * @brief AppSettings::getTiltProjectPathUnchecked
 * @return The path for the TILT_PROJECT parameter.
 *
 * This function doesn't check if the currently configured value is valid.
 * Use this method only when extremamely necessary, such as in displaying
 * the current configured value for user modification, otherwise prefer
 * the getTiltStrategiesPath() method.
 */
const QString AppSettings::getTiltProjectPathUnchecked() const
{
    return getDirectoryPath(TILT_PROJECT_KEY,
                            getTiltProjectPathDefault());
}

/**
 * @brief AppSettings::getTiltProjectPath
 * @return The path to the TILT_PROJECT parameter.
 *
 * This function first tests if the returned value points to a valid system
 * path and if not, shows an error message informing the user about this error
 * and how to fix it.
 */
const QString AppSettings::getTiltProjectPath() const
{
    return checkDirectoryPath(TILT_PROJECT_KEY,
                            getTiltProjectPathDefault(),
                            tr("The path to the root of the ProVANT source "
                               "files could not be opened."));
}

/**
 * @brief AppSettings::getTiltProjectPathDefault
 * @return The default path to the TILT_PROJECT parameter.
 */
const QString AppSettings::getTiltProjectPathDefault() const
{
    return QDir::home().absoluteFilePath("catkin_ws/src/"
                                         "ProVANT-Simulator_Developer");
}

/**
 * @brief AppSettings::setTiltProjectPath
 * @param path The new path to the TILT_PROJECT parameter.
 * @return True if the informed value points to a valid system path and false
 * if the path is invalid.
 */
bool AppSettings::setTiltProjectPath(const QString &path)
{
    return setDirectoryPath(TILT_PROJECT_KEY, path, true);
}

/**
 * @brief AppSettings::getProvantDatabsePathUnchecked
 * @return The path to the PROVANT_DATABSE path.
 *
 * This function doesn't check if the currently configured value is valid.
 * Use this method only when extremamely necessary, such as in displaying
 * the current configured value for user modification, otherwise prefer
 * the getProvantDatabasePath() method.
 */
const QString AppSettings::getProvantDatabasePathUnchecked() const
{
    return getDirectoryPath(PROVANT_DATABASE_KEY,
                            getProvantDatabasePathDefault());
}

/**
 * @brief AppSettings::getProvantDatabasePath
 * @return The path to the PROVANT_DATABSE parameter.
 *
 * This function first tests if the returned value points to a valid system
 * path and if not, shows an error message informing the user about this error
 * and how to fix it.
 */
const QString AppSettings::getProvantDatabasePath() const
{
    return checkDirectoryPath(PROVANT_DATABASE_KEY,
                            getProvantDatabasePathDefault(),
                            tr("The path to the Database ROS package of the "
                               "ProVANT simulator could not be opened."));
}

/**
 * @brief AppSettings::getProvantDatabasePathDefault
 * @return The default path to the PROVANT_DATABASE parameter.
 */
const QString AppSettings::getProvantDatabasePathDefault() const
{
    return QDir::home().absoluteFilePath("catkin_ws/src/"
                                         "ProVANT-Simulator_Developer/"
                                         "source/Database");
}

/**
 * @brief AppSettings::setProvantDatabasePath
 * @param path The new path to the PROVANT_DATABSE_KEY.
 * @return True if the informed value points to a valid system path and false
 * if the path is invalid.
 */
bool AppSettings::setProvantDatabasePath(const QString &path)
{
    return setDirectoryPath(PROVANT_DATABASE_KEY, path, true);
}

/**
 * @brief AppSettings::getRosPathUnchecked
 * @return
 *
 * This function doesn't check if the currently configured value is valid.
 * Use this method only when extremamely necessary, such as in displaying
 * the current configured value for user modification, otherwise prefer
 * the getRosPath() method.
 */
const QString AppSettings::getCatkinWorkspacePathUncheked() const
{
    return getDirectoryPath(DIR_ROS_KEY,
                            getCatkinWorkspacePathDefault());
}

/**
 * @brief AppSettings::getRosPath
 * @return The configured path to the DIR_ROS.
 *
 * This function first tests if the returned value points to a valid system
 * path and if not, shows an error message informing the user about this error
 * and how to fix it.
 */
const QString AppSettings::getCatkinWorkspacePath() const
{
    return checkDirectoryPath(DIR_ROS_KEY,
                            getCatkinWorkspacePathDefault(),
                            tr("The path to the root of your catkin workspace "
                               "could not be opened."));
}

/**
 * @brief AppSettings::getRosPathDefault
 * @return The default path to the DIR_ROS.
 */
const QString AppSettings::getCatkinWorkspacePathDefault() const
{
    return QDir::home().absoluteFilePath("catkin_ws");
}

/**
 * @brief AppSettings::setRosPath
 * @param path The new path to the DIR_ROS parameter.
 * @return True if the informed value points to a valid system path and false
 * if the path is invalid.
 */
bool AppSettings::setCatkinWorkspacePath(const QString &path)
{
    return setDirectoryPath(DIR_ROS_KEY, path, true);
}

/**
 * @brief AppSettings::getControlStrategiesPathUnchecked
 * @return The path to the control strategies directory.
 *
 * This function doesn't check if the currently configured value is valid.
 * Use this method only when extremamely necessary, such as in displaying
 * the current configured value for user modification, otherwise prefer
 * the getControlStrategiesPath() method.
 */
const QString AppSettings::getControlStrategiesPathUnchecked() const
{
    return getDirectoryPath(CONTROL_STRATEGIES_SOURCE_KEY,
                            getControlStrategiesPathDefault());
}

/**
 * @brief AppSettings::getControlStrategiesPath
 * @return The path to the directory containing the source files for the
 * control strategies contained in the simulator.
 *
 * This directory should contain all the source files for the control strategies
 * that can be used with the simulator.
 *
 * Each control strategy should be under a directory named with the control
 * strategy name.
 *
 * This function first tests if the returned value points to a valid system
 * path and if not, shows an error message informing the user about this error
 * and how to fix it.
 */
const QString AppSettings::getControlStrategiesPath() const
{
    return checkDirectoryPath(CONTROL_STRATEGIES_SOURCE_KEY,
                              getControlStrategiesPathDefault(),
                              tr("The path to the directory containing the "
                                 "source files for the control strategies "
                                 "used in the simulator could not be opened."));
}

/**
 * @brief AppSettings::getControlStrategiesPathDefault
 * @return The default path to the CONTROL_STRATEGIES_SOURCE.
 *
 * See also: getControlStrategiesPath().
 */
const QString AppSettings::getControlStrategiesPathDefault() const
{
    return QDir::cleanPath(getTiltProjectPathDefault() + QDir::separator() +
                           "/source/Structure/control_strategies");
}

/**
 * @brief AppSettings::setControlStrategiesPath Updates the path to the
 * controls strategies directory.
 * @param path The new path to the control strategies source directory.
 * @return True if the informed value points to a valid system path and false
 * if the path is invalid.
 *
 * See also: getControlStrategiesPath().
 */
bool AppSettings::setControlStrategiesPath(const QString &path)
{
    return setDirectoryPath(CONTROL_STRATEGIES_SOURCE_KEY, path);
}

/**
 * @brief AppSettings::getRosVersionUnchecked
 * @return The version of ROS used to run the simulator.
 */
const QString AppSettings::getRosVersionUnchecked() const
{
    return settings.value(ROS_VERSION_KEY, getRosVersionDefault()).toString();
}

/**
 * @brief AppSettings::getRosVersion
 * @return The ROS version configured in the settings.
 *
 * Checks to see if the ros version is already defined in QSettings, if it is
 * this value is returned, if not, the default version is returned.
 *
 * Currently two versions are allowed:
 *  - Melodic
 *  - Kinetic
 *
 * This function first tests if the returned value points to a valid system
 * path and if not, shows an error message informing the user about this error
 * and how to fix it.
 */
const QString AppSettings::getRosVersion() const
{
    if(!settings.contains(ROS_VERSION_KEY)) {
        qWarning("The value of the %s setting is not configured, returning "
                 "default value.",
                 qUtf8Printable(ROS_VERSION_KEY));
        return getRosVersionDefault();
    }
    return settings.value(ROS_VERSION_KEY).toString();
}

/**
 * @brief AppSettings::getRosVersionDefault
 * @return The default ROS version for the simulator.
 */
const QString AppSettings::getRosVersionDefault() const
{
    return "melodic";
}

/**
 * @brief AppSettings::setRosVersion
 * @param version The new ROS version used in the application.
 * @return True if the value is a valid option and the setting was updated,
 * and false otherwise.
 *
 * See getRosVersion() to check the allowed values.
 */
bool AppSettings::setRosVersion(const QString &version)
{
    if(version == "melodic" || version == "kinetic")
    {
        settings.setValue(ROS_VERSION_KEY, version);
        qInfo("Updated the value of %s to %s.",
              qUtf8Printable(ROS_VERSION_KEY),
              qUtf8Printable(version));
        return true;
    }
    else {
        qWarning("Trying to update the value of %s to %s. This operation is "
                 "not permited and no modification has been made.",
                 qUtf8Printable(ROS_VERSION_KEY),
                 qUtf8Printable(version));
        return false;
    }
}

/**
 * @brief AppSettings::getRosPathUnchecked
 * @return The path to the ROS installation directory.
 *
 * This function doesn't check if the currently configured value is valid.
 * Use this method only when extremamely necessary, such as in displaying
 * the current configured value for user modification, otherwise prefer
 * the getRosPath() method.
 */
const QString AppSettings::getRosPathUnchecked() const
{
    return getDirectoryPath(ROS_PATH_KEY,
                            getRosPathDefault());
}

/**
 * @brief AppSettings::getRosPath
 * @return The path to the ROS installation directory.
 *
 * This is the path to the directory where ROS keeps all of the files and
 * packages for the core ROS tools.
 *
 * It is used by the application when it is necessary to call an external ROS
 * tool.
 */
const QString AppSettings::getRosPath() const
{
    return checkDirectoryPath(ROS_PATH_KEY,
                              getRosPathDefault(),
                              tr(""));
}

/**
 * @brief AppSettings::getRosPathDefault
 * @return The default path to the ROS installation directory.
 */
const QString AppSettings::getRosPathDefault() const
{
    return QString("/opt/ros/%1/").arg(getRosVersionDefault());
}

/**
 * @brief AppSettings::setRosPath Defines a new path to the ROS installation
 * directory.
 * @param path The new path to the ROS installation directory.
 * @return True if the value points to a valid directory and the configuration
 * was updated, and false otherwise.
 */
bool AppSettings::setRosPath(const QString &path)
{
    return setDirectoryPath(ROS_PATH_KEY, path);
}

const QString AppSettings::getWorldsPackagePathUnchecked() const
{
    QString databasePath = getProvantDatabasePathUnchecked();
    return QDir::cleanPath(databasePath
                           +
                           QDir::separator()
                           +
                           "worlds"
                           +
                           QDir::separator()
                           +
                           "worlds");
}

const QString AppSettings::getWorldsPackagePath() const
{
    return checkDirectoryPath(WORLS_PACKAGE_KEY,
                              getWorldsPackagePathDefault(),
                              tr("The path to the directory containing the "
                                 "Gazebo worlds could bot be open. Please "
                                 "check the value of the PROVANT_DATABASE "
                                 "environment variable and try again."));
}

const QString AppSettings::getWorldsPackagePathDefault() const
{
    QString databasePath = getProvantDatabasePathDefault();
    return QDir::cleanPath(databasePath
                           +
                           QDir::separator()
                           +
                           "worlds"
                           +
                           QDir::separator()
                           +
                           "worlds");
}

bool AppSettings::setWorldsPackagePath(const QString &path)
{
    return setDirectoryPath(WORLS_PACKAGE_KEY, path);
}

/**
 * @brief AppSettings::checkAllParametersSet
 * @return True if all options are correctly configured and false otherwise.
 *
 * Verifies if all the parameters are correctly configured.
 * If one parameter is not configured, show the error message informing the
 * user about the need of correction.
 */
bool AppSettings::checkAllParametersSet() const
{
    if(getProvantRosPath().isEmpty())
    {
        return false;
    }
    if(getTiltStrategiesPath().isEmpty())
    {
        return false;
    }
    if(getTiltProjectPath().isEmpty())
    {
        return false;
    }
    if(getTiltMatlabPath().isEmpty())
    {
        return false;
    }
    if(getProvantDatabasePath().isEmpty())
    {
        return false;
    }
    if(getGazeboModelPath().isEmpty())
    {
        return false;
    }
    if(getCatkinWorkspacePath().isEmpty())
    {
        return false;
    }
    if(getControlStrategiesPath().isEmpty())
    {
        return false;
    }
    if(getRosVersion().isEmpty())
    {
        return false;
    }
    if(getRosPath().isEmpty())
    {
        return false;
    }
    return true;
}

/**
 * @brief AppSettings::applyValuesToEnvrionmentVariables
 *
 * Updates the values of all environment variables to the values from the
 * QSettings configurations.
 *
 * This function doesn't verify if the configurations are set and point to valid
 * locations.
 *
 * Its purpose is to ensure that any other ROS nodes spawned by the GUI that
 * use one or more of the environment variables also have access to the correct
 * values.
 *
 * This function should not contain the creation of an environemnt variable that
 * isn't used by at least one program called by the GUI.
 */
void AppSettings::applyValuesToEnvironmentVariables()
{
    setEnvironmentVariable(PROVANT_ROS_KEY, getProvantRosPathUnchecked());
    setEnvironmentVariable(TILT_STRATEGIES_KEY,
                           getTiltStrategiesPathUnchecked());
    setEnvironmentVariable(TILT_PROJECT_KEY, getTiltProjectPathUnchecked());
    setEnvironmentVariable(TILT_MATLAB_KEY, getTiltMatlabPathUnchecked());
    setEnvironmentVariable(PROVANT_DATABASE_KEY,
                           getProvantDatabasePathUnchecked());
    setEnvironmentVariable(GAZEBO_MODEL_PATH_KEY,
                           getGazeboModelPathUncheked());
    setEnvironmentVariable(DIR_ROS_KEY, getCatkinWorkspacePath());
}

/**
 * @brief AppSettings::applyDefaultsToUndefinedParameters
 *
 * Check if the QSettings object has a key corresponding to each application
 * parameter, if the option isn't set, the value is updated with the parameter
 * default.
 *
 * All options already configured aren't modified.
 */
void AppSettings::applyDefaultsToUndefinedParameters()
{
    if(!settings.contains(PROVANT_ROS_KEY))
    {
        setProvantRosPath(getProvantRosPathDefault());
    }
    if(!settings.contains(TILT_STRATEGIES_KEY))
    {
        setTiltStrategiesPath(getTiltStrategiesPathDefault());
    }
    if(!settings.contains(TILT_PROJECT_KEY))
    {
        setTiltProjectPath(getTiltProjectPathDefault());
    }
    if(!settings.contains(TILT_MATLAB_KEY))
    {
        setTiltMatlabPath(getTiltMatlabPathDefault());
    }
    if(!settings.contains(PROVANT_DATABASE_KEY))
    {
        setProvantDatabasePath(getProvantDatabasePathDefault());
    }
    if(!settings.contains(GAZEBO_MODEL_PATH_KEY))
    {
        setGazeboModelPath(getGazeboModelPathDefault());
    }
    if(!settings.contains(DIR_ROS_KEY))
    {
        setCatkinWorkspacePath(getCatkinWorkspacePathDefault());
    }
    if(!settings.contains(CONTROL_STRATEGIES_SOURCE_KEY))
    {
        setControlStrategiesPath(getControlStrategiesPathDefault());
    }
    if(!settings.contains(ROS_VERSION_KEY))
    {
        setRosVersion(getRosVersionDefault());
    }
    if(!settings.contains(ROS_PATH_KEY))
    {
        setRosPath(getRosPathDefault());
    }
}

/**
 * @brief AppSettings::restoreDefaults
 *
 * Sets all application settings to the respective default values.
 *
 * This function should be used with extreme care, since there is no way
 * to undo this action after its realization.
 *
 * A good example of when to call this function is in the installation of the
 * GUI in a new computer, where the configuration files don't exist yet.
 */
void AppSettings::restoreDefaults()
{
    qWarning("Restoring all configuration to default values");
    setProvantRosPath(getProvantRosPathDefault());
    setTiltStrategiesPath(getTiltStrategiesPathDefault());
    setTiltProjectPath(getTiltProjectPathDefault());
    setTiltMatlabPath(getTiltMatlabPathDefault());
    setProvantDatabasePath(getProvantDatabasePathDefault());
    setGazeboModelPath(getGazeboModelPathDefault());
    setCatkinWorkspacePath(getCatkinWorkspacePathDefault());
    setControlStrategiesPath(getControlStrategiesPathDefault());
    setRosVersion(getRosVersionDefault());
    setRosPath(getRosPathDefault());
}

/**
 * @brief AppSettings::getEnvironmentVariables Create a QProcessEnvironment
 * object with all the environment variables set to the values defined by
 * the user settings.
 * @return A QProcessEnvironment object containing the values of the
 * environment variables to be passed to other programas called by the GUI.
 */
QProcessEnvironment AppSettings::getEnvironmentVariables() const
{
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.insert(PROVANT_ROS_KEY, getProvantRosPath());
    env.insert(TILT_STRATEGIES_KEY, getTiltStrategiesPath());
    env.insert(TILT_PROJECT_KEY, getTiltProjectPath());
    env.insert(PROVANT_DATABASE_KEY, getProvantDatabasePath());
    env.insert(GAZEBO_MODEL_PATH_KEY, getGazeboModelPath());
    env.insert(DIR_ROS_KEY, getCatkinWorkspacePath());
    return env;
}

/**
 * @brief AppSettings::getEnvironmentVariable
 * @param key The key to the desired environment variable (the environment
 * variable name)
 * @param default_ The default value to return in the case that the value isn't
 * set.
 * @return The value contained in the envionment variable or the default value
 * if the variable isn't set or can't be accessed.
 */
QString AppSettings::getEnvironmentVariable(const QString &key,
                                            const QString &default_) const
{
    QString env = QString::fromUtf8(qgetenv(key.toStdString().c_str()));

    return env.isEmpty() ? default_ : env;
}

/**
 * @brief AppSettings::getValueFromKey
 * @param key The key to access the desired parameter.
 * @param default_ The default value to this parameter.
 * @return An string containing the configured value in QSettings or the
 * default value if the desired paramter isn't set.
 */
QString AppSettings::getValueFromKey(const QString &key,
                                     const QString &default_) const
{
    QString def = getEnvironmentVariable(key, default_);
    return settings.value(key, def).toString();
}

/**
 * @brief AppSettings::getErrorCorrectionMessage
 * @param key The key to the parameter in which an error ocurred.
 * @return A message informing the user about the steps needed to correct
 * the error.
 */
QString AppSettings::getErrorCorrectionMessage(const QString &key) const
{
    return tr("\n\nTo correct this issue, go to Tools -> Options in the main "
              "application menu bar and set the correct value under the option "
              "%1.").arg(key);
}

/**
 * @brief AppSettings::getDirectoryPath
 * @param key The key to access the desired parameter path.
 * @param defaultValue The default value to the parameter.
 * @return The value of the configured parameter or an empty string if no value
 * is set.
 */
const QString AppSettings::getDirectoryPath(const QString &key,
                                            const QString &defaultValue) const
{
    return getValueFromKey(key, defaultValue);
}

/**
 * @brief AppSettings::checkDirectoryPath
 * @param key The key to the parameter the should be acceseed.
 * @param defaultValue The default value to the parameter.
 * @param errorMessage An error message that will be displayed if the parameter
 * and its default value doesnt point to a valid system path.
 * @return A string containing the path configured in this parameter.
 *
 * Reads the value of the parameter informed in a key.
 * If the value of the parameter is already configured in QSettings or in an
 * environment variable, this value is returned.
 *
 * If the value of the parameter wasn't set, the value in the defaultValue
 * parameter is chcked for validity.
 *
 * If at least of the three options above points to a valid system path, this
 * value is returned.
 * Otherwise, an empty string is returned and a dialog box informing the user
 * about this error is shown on screen.
 */
const QString AppSettings::checkDirectoryPath(const QString &key,
                                              const QString &defaultValue,
                                              const QString &errorMessage) const
{
    QDir dir(getValueFromKey(key, defaultValue));

    if(!dir.exists()) {
        QMessageBox::critical(
                    nullptr,
                    _criticalMsgTitle,
                    errorMessage +
                    getErrorCorrectionMessage(key));
        return QString();
    }

    return dir.absolutePath();
}

/**
 * @brief AppSettings::setDirectoryPath
 * @param key The key to the parameter that will be updated.
 * @param value The new value to the parameter.
 * @param setEnvironmentVariable_ Indicates if the parameter should also be
 * persisted in an enviornment variable.
 * @return True if the path is valid and the operation was concluded and false
 * otherwise.
 *
 * Verifies if the path in the value parameter is valid meaning that it
 * points to a valid system path.
 *
 * If the path is valid, the QSettings value is updated along with the value
 * in the environment variables.
 */
bool AppSettings::setDirectoryPath(const QString &key, const QString &value,
                                   bool setEnrionmentVariable_)
{
    QDir dir(value);

    if(dir.exists()) {
        settings.setValue(key, value);
        if(setEnrionmentVariable_)
        {
            setEnvironmentVariable(key, value);
        }
        return true;
    }
    return false;
}

/**
 * @brief AppSettings::setEnvironmentVariable
 * @param key The key to the environment variable that will be updated.
 * @param value The new value to the environment variable.
 * @return True if the variable is set, and false otherwise.
 */
bool AppSettings::setEnvironmentVariable(const QString &key,
                                         const QString &value)
{
    return qputenv(key.toStdString().c_str(), value.toStdString().c_str());
}
