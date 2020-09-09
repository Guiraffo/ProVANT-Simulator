#include "configfile.h"

#include <QtGlobal>
#include <QDebug>

/**
 * @brief ConfigFile::setSensors Override the list of sensors.
 * @param sensors
 */
void ConfigFile::setSensors(const QStringList &sensors)
{
    _sensors = sensors;
}

/**
 * @brief ConfigFile::addSensor Add a new sensor to the list.
 * @param sensor Name of the sensor that will be added to the list.
 */
void ConfigFile::addSensor(const QString &sensor)
{
    _sensors.push_back(sensor);
}

/**
 * @brief ConfigFile::deleteSensor
 * @param pos The position of the sensor to be removed.
 *
 * Removes a sensor from the sensors list by the index
 */
void ConfigFile::deleteSensor(int pos)
{
    _sensors.removeAt(pos);
}

/**
 * @brief ConfigFile::getActuators
 * @return The list of actuators.
 */
const QStringList ConfigFile::getActuators() const
{
    return _actuators;
}

/**
 * @brief ConfigFile::setActuators Overrides the list of actuators.
 * @param actuators The new list of actuators.
 */
void ConfigFile::setActuators(const QStringList &actuators)
{
    _actuators = actuators;
}

/**
 * @brief ConfigFile::addActuator Add a new actuator to the actuator list.
 * @param actuator The name of the new actuator.
 */
void ConfigFile::addActuator(const QString &actuator)
{
    _actuators.push_back(actuator);
}

/**
 * @brief ConfigFile::deleteActuator Remove a new acutator of the list by index.
 * @param pos The index of the actuator to be removed.
 */
void ConfigFile::deleteActuator(int pos)
{
    _actuators.removeAt(pos);
}

/**
 * @brief ConfigFile::getControlStrategy
 * @return The name of the control strategy selected by this model.
 */
const QString &ConfigFile::getControlStrategy() const
{
    return _controlStrategy;
}

/**
 * @brief ConfigFile::setStrategy
 * @param strategy The name of the new control strategy.
 */
void ConfigFile::setStrategy(const QString &strategy)
{
    _controlStrategy = strategy;
}

/**
 * @brief ConfigFile::getTurbulanceModel
 * @return The name of the turbulance model selected by this model.
 */
const QString &ConfigFile::getTurbulanceModel() const
{
    return _turbulanceModel;
}

/**
 * @brief ConfigFile::setTurbulanceModel
 * @param turbulance The name of the turbulance model.
 */
void ConfigFile::setTurbulanceModel(const QString &turbulance)
{
    _turbulanceModel = turbulance;
}

/**
 * @brief ConfigFile::getSampleTime
 * @return The sample time to this controller.
 */
const QString &ConfigFile::getSampleTime() const
{
    return _sampleTime;
}

/**
 * @brief ConfigFile::setSampleTime Defines the new sample time for the
 * controller.
 * @param sampleTime The sample time.
 * @todo Identificar em que a unidade está o sample time e adicionar métodos
 * com o tipo númerico correto.
 */
void ConfigFile::setSampleTime(const QString &sampleTime)
{
    _sampleTime = sampleTime;
}

/**
 * @brief ConfigFile::getCommunication
 * @return The communication.
 * @todo Identificar o que é o objeto de comunicação.
 */
const QString &ConfigFile::getCommunication() const
{
    return _communication;
}

/**
 * @brief ConfigFile::setCommunication Defines the new communication for the
 * controller.
 * @param comm
 */
void ConfigFile::setCommunication(const QString &comm)
{
    _communication = comm;
}

/**
 * @brief ConfigFile::getErrorLogFilename
 * @return The name of the file that should contain the error log.
 */
const QString &ConfigFile::getErrorLogFilename() const
{
    return _errorFilename;
}

/**
 * @brief ConfigFile::getRefLogFilename
 * @return The name of the file that should contain the reference log.
 */
const QString &ConfigFile::getRefLogFilename() const
{
    return _refFilename;
}

/**
 * @brief ConfigFile::getOutLogFilename
 * @return The name of the file that should contain the output log.
 */
const QString &ConfigFile::getOutLogFilename() const
{
    return _outFilename;
}

/**
 * @brief ConfigFile::getInLog
 * @return The name of the file that should contain the input log.
 */
const QString &ConfigFile::getInLog() const
{
    return _inFilename;
}

/**
 * @brief ConfigFile::setLog Defines the filenames of the simulation logs.
 * @param erro The filename of the error log.
 * @param ref The filename of the reference log.
 * @param out The filename of the output log.
 * @param in The filename of the input log.
 */
void ConfigFile::setLog(const QString &erro,
                        const QString &ref,
                        const QString &out,
                        const QString &in)
{
    _errorFilename = erro;
    _refFilename = ref;
    _outFilename = out;
    _inFilename = in;
}

/**
 * @brief ConfigFile::getStepTopic
 * @return The name of the ROS topic used to increment
 * the step of the simulation.
 */
const QString &ConfigFile::getStepTopic() const
{
    return _stepTopic;
}

/**
 * @brief ConfigFile::setStepTopic Defines the name of the ROS topic used
 * to increment the simulation step.
 * @param topic The new topic name.
 */
void ConfigFile::setStepTopic(const QString &topic)
{
    _stepTopic = topic;
}

/**
 * @brief ConfigFile::readFile Reads the content of the config file.
 */
void ConfigFile::readFile()
{
     _controlStrategy = readItem("Strategy");
     _turbulanceModel = readItem("Turbulance");
     _sampleTime = readItem("Sampletime");
     _communication = readItem("topicdata");
     _errorFilename = readItem("ErroPath");
     _refFilename = readItem("RefPath");
     _outFilename = readItem("Outputfile");
     _inFilename = readItem("InputPath");
     _stepTopic = readItem("TopicoStep");
     _sensors = readAllItems("Sensors");
     _actuators = readAllItems("Actuators");
}

/**
 * @brief ConfigFile::createFile Creates a new config file with empty values in
 * all parameters.
 * @return True if the file was sucessfully created and false otherwise.
 */
bool ConfigFile::createFile()
{
    if ( _file.open(QIODevice::ReadWrite | QIODevice::Text) )
    {
        QXmlStreamWriter xml;
        xml.setAutoFormatting(true);
        xml.setDevice(&_file);
        xml.writeStartDocument();
        xml.writeStartElement("config");
        xml.writeTextElement("topicdata", "");
        xml.writeTextElement("TopicoStep", "");
        xml.writeTextElement("Sampletime", "");
        xml.writeTextElement("Strategy", "");
        xml.writeTextElement("Turbulance", "");
        xml.writeTextElement("RefPath", "");
        xml.writeTextElement("Outputfile", "");
        xml.writeTextElement("InputPath", "");
        xml.writeTextElement("ErroPath", "");
        xml.writeStartElement("Sensors");
        xml.writeEndElement(); //End Sensors element
        xml.writeStartElement("Actuators");
        xml.writeEndElement(); // End Actuators element
        xml.writeEndDocument(); // End config element
        _file.close();
        return true;
    }
    else{
        qCritical("Error when trying to create the file %s.\nThe file could not"
                  "be opened.",
                  qUtf8Printable(_filename));
        return false;
    }
}

/**
 * @brief ConfigFile::writeFile Writes the configured options to a xml file.
 * @return
 */bool ConfigFile::writeFile()
{
    if(_file.open(QIODevice::ReadWrite|QIODevice::Truncate))
    {
        QXmlStreamWriter xml;
        xml.setAutoFormatting(true);
        xml.setDevice(&_file);
        xml.writeStartDocument();

        xml.writeStartElement("config");
        xml.writeTextElement("topicdata", _communication);
        xml.writeTextElement("TopicoStep", _stepTopic);
        xml.writeTextElement("Sampletime", _sampleTime);
        xml.writeTextElement("Strategy", _controlStrategy);
        xml.writeTextElement("Turbulance", _turbulanceModel);
        xml.writeTextElement("RefPath", _refFilename);
        xml.writeTextElement("Outputfile", _outFilename);
        xml.writeTextElement("InputPath", _inFilename);
        xml.writeTextElement("ErroPath", _errorFilename);

        xml.writeStartElement("Sensors");
        for(int i=0; i < _sensors.size(); i++)
        {
            xml.writeTextElement("Device", _sensors.at(i));
        }
        xml.writeEndElement(); // End element Sensors

        xml.writeStartElement("Actuators");
        for(int i = 0; i < _actuators.size(); i++)
        {
            xml.writeTextElement("Device", _actuators.at(i));
        }
        xml.writeEndElement(); // End element Actuators

        xml.writeEndElement(); // End element config
        xml.writeEndDocument();

        _file.close();
        return true;
    }
    else
    {
        /*
         * Logs the error and finishes the application.
         */
        //! @todo Create exception
        qFatal("An error ocurred when trying to write the file %s.",
               qUtf8Printable(_filename));
        _file.close();
        QCoreApplication::exit(1);
    }
    return false; // Just to please the compiler and avoid a "not all paths
    // return a value warning.
}

/**
 * @brief ConfigFile::readItem Read the contents of a single XML tag.
 * @param tag The XML tag to read.
 * @return The contents of the specified tag.
 */
QString ConfigFile::readItem(const QString &tag)
{
    /*
     * The previous line contained here shouldn't do anything, so i updated
     * the code to check if the file is opened and report the occurence of
     * this error.
     */
    if(_file.isOpen()) {
        _file.close();
        qWarning("The _file %s was not properly closed in a "
                 "previous instance.",
                 qUtf8Printable(_file.fileName()));
    }

    if(_file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QString erro;
        int line = 0, column = 0;
        if(_document.setContent(&_file, &erro, &line, &column))
        {
            _file.close();
            return  _document.firstChildElement("config")
                            .firstChildElement(tag)
                            .text();
        }
        else
        {
            _file.close();
            qFatal("An error (%s) ocurred when reading the %s file on line"
                   " %d and column %d.",
                   qUtf8Printable(_filename),
                   qUtf8Printable(erro),
                   line,
                   column);
            QCoreApplication::exit(1);
        }
    }
    else
    {
        //! @todo Create exception
        _file.close();
        qFatal("An error ocurred when tyring to open the file %s.",
               qUtf8Printable(_filename));

        _file.close();
    }
    _file.close();
    return "";
}

/**
 * @brief ConfigFile::readAllItems Reads the contents of all the tags
 * under a specified XML tag.
 * @param tag The tag whose elements should be read.
 * @return The readed elements of the specified tag.
 */
QStringList ConfigFile::readAllItems(const QString &tag)
{
    QStringList output;
    if(_file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QString erro;
        int line, column;
        if(_document.setContent(&_file,&erro,&line,&column))
        {
            _file.close();
            QDomNodeList devices = _document.firstChildElement("config")
                                           .firstChildElement(tag)
                                           .elementsByTagName("Device");
            for(int i=0; i< devices.count(); i++)
            {
                QDomNode device = devices.at(i);
                if(device.isElement())
                {
                    output.push_back(device.toElement().text());
                }
            }
        }
        else
        {
            if (erro == "unexpected end of file")
            {
                qDebug() << "Arquivo Vazio";
            }
            else
            {
                qDebug() << "Problemas com conteudo" << erro;
                qDebug("Linha %d", line);
                qDebug("Coluna %d", column);
                _file.close();
                QCoreApplication::exit(1);
            }
        }
    }
    else
    {
        QFileInfo finfo(_file);
        qFatal("%s%s%s",
               qUtf8Printable(
                   QObject::tr("An error ocurred while trying to read the "
                               "config.xml file for this model. Please "
                               "make sure that the file with path ")),
               qUtf8Printable(finfo.absoluteFilePath()),
               qUtf8Printable(
                       QObject::tr(" exists and is readable to the current "
                                   "user.")
        ));

        _file.close();
        QCoreApplication::exit(1);
    }
    _file.close();
    return output;
}

/**
 * @brief ConfigFile::ConfigFile
 * @param filename The path to the config file.
 *
 * Initializes the object, sets the filename and constructs the file object.
 */
ConfigFile::ConfigFile(const QString &filename) :
    _filename(filename),
    _file(filename)
{

}

/**
 * @brief ConfigFile::print
 *
 * Outputs the contents of the configfile to a debug window.
 */
void ConfigFile::print() const
{
    qDebug() << "Filename: " << qUtf8Printable(_filename);
    qDebug() << "Control Strategy" << qUtf8Printable(_controlStrategy);
    qDebug() << "Turbulance Model" << qUtf8Printable(_turbulanceModel);
    qDebug() << "Sample Time: " << qUtf8Printable(_sampleTime);
    qDebug() << "Communication: " << qUtf8Printable(_communication);
    qDebug() << "Error Filename: " << qUtf8Printable(_errorFilename);
    qDebug() << "Ref Filename: " << qUtf8Printable(_refFilename);
    qDebug() << "Out Filename: " << qUtf8Printable(_outFilename);
    qDebug() << "In Filename: " << qUtf8Printable(_inFilename);
    qDebug() << "Step Topic: " << qUtf8Printable(_stepTopic);

    qDebug() << "Sensors: ";
    for(int i = 0; i < _sensors.size(); i++)
    {
        qDebug() << qUtf8Printable(_sensors.at(i));
    }

    qDebug() << "Actuators: ";
    for(int i=0; i < _actuators.size(); i++)
    {
        qDebug() << qUtf8Printable(_actuators.at(i));
    }
}

/**
 * @brief ConfigFile::clearSensorsAndActuators
 * Clears the content of the lists of sensors and actuators.
 */
void ConfigFile::clearSensorsAndActuators()
{
    _actuators.clear();
    _sensors.clear();
}

/**
 * @brief ConfigFile::getSensors
 * @return The list of sensors.
 */
const QStringList &ConfigFile::getSensors() const
{
    return _sensors;
}
