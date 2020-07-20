#ifndef CONFIGFILE_H
#define CONFIGFILE_H

#include <QStringList>
#include <QtXml/QtXml>

/**
 * @brief The ConfigFile class is responsible for manipulation the config XML
 * file.
 *
 * This class provides an interface for the access and manipulation of the
 * contents of the config file for a simulator model.
 *
 * @todo Question Arthur about the purpose of this class.
 */
class ConfigFile
{
public:
    ConfigFile(const QString &filename);
    void setFilename(const std::string &filename);
    void print() const;

    void clearSensorsAndActuators();

    const QStringList &getSensors() const;
    void setSensors(const QStringList &sensors);
    void addSensor(const QString &sensor);
    void deleteSensor(int pos);

    const QStringList getActuators() const;
    void setActuators(const QStringList &actuators);
    void addActuator(const QString &actuator);
    void deleteActuator(int pos);

    const QString  &getControlStrategy() const;
    void setStrategy(const QString &strategy);

    const QString  &getTurbulanceModel() const;
    void setTurbulanceModel(const QString &turbulance);

    const QString &getSampleTime() const;
    void setSampleTime(const QString &sampleTime);

    const QString &getCommunication() const;
    void setCommunication(const QString &comm);

    const QString &getErrorLogFilename() const;
    const QString &getRefLogFilename() const;
    const QString &getOutLogFilename() const;
    const QString &getInLog() const;
    void setLog(const QString &erro,
                const QString &ref,
                const QString &out,
                const QString &in);

    const QString &getStepTopic() const;
    void setStepTopic(const QString &topic);

    bool createFile();
    void readFile();
    bool writeFile();

    QString readItem(const QString &tag);
    QStringList readAllItems(const QString &tag);

private:
    QString _filename;
    QDomDocument _document;
    QFile _file;
    QStringList _sensors;
    QStringList _actuators;
    QString _controlStrategy;
    QString _turbulanceModel;
    QString _sampleTime;
    QString _communication;
    QString _errorFilename;
    QString _refFilename;
    QString _outFilename;
    QString _inFilename;
    QString _stepTopic;
};

#endif // CONFIGFILE_H
