/*!
 * \file appsettings.h
 * @author Júnio Eduardo de Morais Aquino
 * @date 2019/02/21
 */

#ifndef APPSETTINGS_H
#define APPSETTINGS_H

#include <QObject>
#include <QDir>
#include <QString>
#include <QSettings>

//! The key used to access the GazeboModelPath.
static const QString GAZEBO_MODEL_PATH_KEY = "GAZEBO_MODEL_PATH";
//! The key used to access the ProvantRosPath.
static const QString PROVANT_ROS_KEY = "PROVANT_ROS";
//! The key used to access the TiltStrategiesPath.
static const QString TILT_STRATEGIES_KEY =  "TILT_STRATEGIES";
//! The key used to access the TiltMatlabPath.
static const QString TILT_MATLAB_KEY = "TILT_MATLAB";
//! The key used to access the TiltProjectPath.
static const QString TILT_PROJECT_KEY = "TILT_PROJECT";
//! The key used to access the ProvantDatabasePath.
static const QString PROVANT_DATABASE_KEY = "PROVANT_DATABASE";
//! The key used to access the DirRosPath.
static const QString DIR_ROS_KEY = "DIR_ROS";

/*!
 * \brief The AppSettings class is used to access all the paths used in the
 * application.
 *
 * This class was created to be a single source of thruth for the paths of the
 * used in the application.
 *
 * Two kinds of access methods are provided for each parameter.
 * The first one is checked, and verifies if the path is valid, if it isn't
 * an error message is show to the user in a dialog box, informing which
 * parameter is incorrectly configured and explaining to the user which actions
 * should be taken to fix the error.
 *
 * The second one is unchecked and is intended to be used when the check is
 * redundant or unecessary, such as displaying the current value of the
 * parameter for user modification.
 *
 * When reading the values, first the function tries to read the value from
 * the QSettings object. This is the primary option because it is a more
 * portable option across diferent operational systems and more robust than
 * using environment variables, since the application can always read its
 * own configurations, but it needs to the opened from a previously configured
 * terminal session to have access to the user environment variables.
 *
 * If the QSettings option hasn't been configured, the value of the environment
 * variable is then read. If this value is a valid path, it is returned,
 * otherwise a default path is returned.
 *
 * The dafault paths were created based on the path of the Folders contained in
 * the simulator source files.
 *
 * The methods used for setting the values in the QSettings object also sets
 * the values in the respective environment variables to maintain compatibility
 * with previous versions.
 */
class AppSettings : public QObject
{
    Q_OBJECT
public:
    AppSettings(QObject *parent = nullptr);

    const QString getGazeboModelPathUncheked() const;
    const QString getGazeboModelPath() const;
    const QString getGazeboModelPathDefault() const;
    bool setGazeboModelPath(const QString &path);

    const QString getProvantRosPathUnchecked() const;
    const QString getProvantRosPath() const;
    const QString getProvantRosPathDefault() const;
    bool setProvantRosPath(const QString &path);

    const QString getTiltStratigiesPathUnchecked() const;
    const QString getTiltStrategiesPath() const;
    const QString getTiltStrategiesPathDefault() const;
    bool setTiltStrategiesPath(const QString &path);

    const QString getTiltMatlabPathUnchecked() const;
    const QString getTiltMatlabPath() const;
    const QString getTiltMatlabPathDefault() const;
    bool setTiltMatlabPath(const QString &path);

    const QString getTiltProjectPathUnchecked() const;
    const QString getTiltProjectPath() const;
    const QString getTiltProjectPathDefault() const;
    bool setTiltProjectPath(const QString &path);

    const QString getProvantDatabsePathUnchecked() const;
    const QString getProvantDatabasePath() const;
    const QString getProvantDatabasePathDefault() const;
    bool setProvantDatabasePath(const QString &path);

    const QString getRosPathUnchecked() const;
    const QString getRosPath() const;
    const QString getRosPathDefault() const;
    bool setRosPath(const QString &path);

    bool checkAllParametersSet() const;
    void applyValuesToEnvrionmentVariables();

protected:
    QString getEnvironmentVariable(const QString &key,
                                   const QString &default_=QString("")) const;
    QString getValueFromKey(const QString &key,
                            const QString &default_=QString("")) const;
    QString getErrorCorrectionMessage(const QString &key) const;

    const QString getDirectoryPath(const QString &key,
                                   const QString &defaultValue) const;
    const QString checkDirectoryPath(const QString &path,
                                     const QString &defaultValue,
                                     const QString &errorMessage) const;
    bool setDirectoryPath(const QString &key, const QString &value);
    bool setEnvironmentVariable(const QString &key, const QString &value);

private:
    //! The QSettings object used to access and set the parameters.
    QSettings settings;

    //! The message shown in the title of the error dialog box.
    const QString _criticalMsgTitle = tr("Path Error");
};

#endif // APPSETTINGS_H
