#include "appsettings.h"

#include <QMessageBox>
#include <QTextCodec>

AppSettings::AppSettings(QObject *parent) : QObject(parent)
{

}

const QString AppSettings::getGazeboModelPathUncheked() const
{
    return getDirectoryPath(GAZEBO_MODEL_PATH_KEY,
                            getGazeboModelPathDefault());
}

const QString AppSettings::getGazeboModelPath() const
{
    return checkDirectoryPath(GAZEBO_MODEL_PATH_KEY,
                            getGazeboModelPathDefault(),
                            tr("The directory containing the models used in "
                               "the simulator could not be opened."));
}

const QString AppSettings::getGazeboModelPathDefault() const
{
    return QDir::home().absoluteFilePath("catkin_ws/src/"
                                         "ProVANT-Simulator_Developer/"
                                         "source/Database/models");
}

bool AppSettings::setGazeboModelPath(const QString &path)
{
    return setDirectoryPath(GAZEBO_MODEL_PATH_KEY, path);
}

const QString AppSettings::getProvanRosPathUnchecked() const
{
    return getDirectoryPath(PROVANT_ROS_KEY,
                            getProvantRosPathDefault());
}

const QString AppSettings::getProvantRosPath() const
{
    return checkDirectoryPath(PROVANT_ROS_KEY,
                            getProvantRosPathDefault(),
                            tr("The path to the src directory of the catkin "
                               "workspace could not be opened."));
}

const QString AppSettings::getProvantRosPathDefault() const
{
    return QDir::home().absoluteFilePath("catkin_ws/src");
}

bool AppSettings::setProvantRosPath(const QString &path)
{
    return setDirectoryPath(PROVANT_ROS_KEY, path);
}

const QString AppSettings::getTiltStratigiesPathUnchecked() const
{
    return getDirectoryPath(TILT_STRATEGIES_KEY,
                            getTiltStrategiesPathDefault());
}

const QString AppSettings::getTiltStrategiesPath() const
{
    return checkDirectoryPath(TILT_STRATEGIES_KEY,
                            getTiltStrategiesPathDefault(),
                            tr("The path to the folder containing the compiled "
                               "binaries of the control strategies could not "
                               "be opened."));
}

const QString AppSettings::getTiltStrategiesPathDefault() const
{
    return QDir::home().absoluteFilePath("catkin_ws/devel/lib");
}

bool AppSettings::setTiltStrategiesPath(const QString &path)
{
    return setDirectoryPath(TILT_STRATEGIES_KEY, path);
}

const QString AppSettings::getTiltMatlabPathUnchecked() const
{
    return getDirectoryPath(TILT_MATLAB_KEY,
                            getTiltMatlabPathDefault());
}

const QString AppSettings::getTiltMatlabPath() const
{
    return checkDirectoryPath(TILT_MATLAB_KEY,
                            getTiltMatlabPathDefault(),
                            tr("The path to the folder containing the files "
                               "necessary to the Matlab logging of the files "
                               "could not be opened."));
}

const QString AppSettings::getTiltMatlabPathDefault() const
{
    return QDir::home().absoluteFilePath("catkin_ws/src/"
                                         "ProVANT-Simulator_Developer/"
                                         "source/Structure/Matlab/");
}

bool AppSettings::setTiltMatlabPath(const QString &path)
{
    return setDirectoryPath(TILT_MATLAB_KEY, path);
}

const QString AppSettings::getTiltProjectPathUnchecked() const
{
    return getDirectoryPath(TILT_PROJECT_KEY,
                            getTiltProjectPathDefault());
}

const QString AppSettings::getTiltProjectPath() const
{
    return checkDirectoryPath(TILT_PROJECT_KEY,
                            getTiltProjectPathDefault(),
                            tr("The path to the root of the ProVANT source "
                               "files could not be opened."));
}

const QString AppSettings::getTiltProjectPathDefault() const
{
    return QDir::home().absoluteFilePath("catkin_ws/src/"
                                         "ProVANT-Simulator_Developer");
}

bool AppSettings::setTiltProjectPath(const QString &path)
{
    return setDirectoryPath(TILT_PROJECT_KEY, path);
}

const QString AppSettings::getProvantDatabsePathUnchecked() const
{
    return getDirectoryPath(PROVANT_DATABASE_KEY,
                            getProvantDatabasePathDefault());
}

const QString AppSettings::getProvantDatabasePath() const
{
    return checkDirectoryPath(PROVANT_DATABASE_KEY,
                            getProvantDatabasePathDefault(),
                            tr("The path to the Database ROS package of the "
                               "ProVANT simulator could not be opened."));
}

const QString AppSettings::getProvantDatabasePathDefault() const
{
    return QDir::home().absoluteFilePath("catkin_ws/src/"
                                         "ProVANT-Simulator_Developer/"
                                         "source/Database");
}

bool AppSettings::setProvantDatabasePath(const QString &path)
{
    return setDirectoryPath(PROVANT_DATABASE_KEY, path);
}

const QString AppSettings::getRosPathUnchecked() const
{
    return getDirectoryPath(DIR_ROS_KEY,
                            getRosPathDefault());
}

const QString AppSettings::getRosPath() const
{
    return checkDirectoryPath(DIR_ROS_KEY,
                            getRosPathDefault(),
                            tr("The path to the root of your catkin workspace "
                               "could not be opened."));
}

const QString AppSettings::getRosPathDefault() const
{
    return QDir::home().absoluteFilePath("catkin_ws");
}

bool AppSettings::setRosPath(const QString &path)
{
    return setDirectoryPath(DIR_ROS_KEY, path);
}

QString AppSettings::getEnvironmentVariable(const QString &key,
                                            const QString &default_) const
{
    QString env = QString::fromUtf8(qgetenv(key.toStdString().c_str()));

    return env.isEmpty() ? default_ : env;
}

QString AppSettings::getValueFromKey(const QString &key,
                                     const QString &default_) const
{
    QString def = getEnvironmentVariable(key, default_);
    return settings.value(key, def).toString();
}

QString AppSettings::getErrorCorrectionMessage(const QString &key) const
{
    return tr("\n\nTo correct this issue, go to Tools -> Options in the main "
              "application menu bar and set the correct value under the option "
              "%1.").arg(key);
}

const QString AppSettings::getDirectoryPath(const QString &key,
                                            const QString &defaultValue) const
{
    return getValueFromKey(key, defaultValue);
}

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
    }

    return dir.absolutePath();
}

bool AppSettings::setDirectoryPath(const QString &key, const QString &value)
{
    QDir dir(value);

    if(dir.exists()) {
        settings.setValue(key, value);
        setEnvironmentVariable(key, value);
        return true;
    }
    return false;
}

bool AppSettings::setEnvironmentVariable(const QString &key, const QString &value)
{
    return qputenv(key.toStdString().c_str(), value.toStdString().c_str());
}
