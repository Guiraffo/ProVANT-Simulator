#ifndef APPSETTINGS_H
#define APPSETTINGS_H

#include <QObject>
#include <QDir>
#include <QString>
#include <QSettings>

static const QString GAZEBO_MODEL_PATH_KEY = "GAZEBO_MODEL_PATH";
static const QString PROVANT_ROS_KEY = "PROVANT_ROS";
static const QString TILT_STRATEGIES_KEY =  "TILT_STRATEGIES";
static const QString TILT_MATLAB_KEY = "TILT_MATLAB";
static const QString TILT_PROJECT_KEY = "TILT_PROJECT";
static const QString PROVANT_DATABASE_KEY = "PROVANT_DATABASE";
static const QString DIR_ROS_KEY = "DIR_ROS";

class AppSettings : public QObject
{
    Q_OBJECT
public:
    AppSettings(QObject *parent = nullptr);

    const QString getGazeboModelPathUncheked() const;
    const QString getGazeboModelPath() const;
    const QString getGazeboModelPathDefault() const;
    bool setGazeboModelPath(const QString &path);

    const QString getProvanRosPathUnchecked() const;
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

protected:
    QString getEnvironmentVariable(const QString &key,
                                   const QString &default_=QString("")) const;
    QString getValueFromKey(const QString &key,
                            const QString &default_=QString("")) const;
    QString getErrorCorrectionMessage(const QString &key) const;

    const QString getDirectoryPath(const QString &key,
                                   const QString &defaultValue) const;
    const QString checkDirectoryPath(const QString &path, const QString &defaultValue,
                                     const QString &errorMessage) const;
    bool setDirectoryPath(const QString &key, const QString &value);
    bool setEnvironmentVariable(const QString &key, const QString &value);

private:
    QSettings settings;

    QString _criticalMsgTitle = tr("Path Error");
};

#endif // APPSETTINGS_H
