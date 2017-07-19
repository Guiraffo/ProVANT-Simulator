#ifndef CONFIGFILE_H
#define CONFIGFILE_H
#include "vector"
#include "QtXml/QtXml"
class ConfigFile
{
public:
    ConfigFile(std::string);
    void SetFilename(std::string);
    void print();


    std::vector<std::string> GetSensors();
    void SetSensors(std::vector<std::string>);
    void AddSensor(std::string);
    void DeleteSensor(int pos);
    void Delete();

    std::vector<std::string> GetActuators();
    void SetActuators(std::vector<std::string>);
    void AddActuator(std::string);
    void DeleteActuator(int pos);

    std::string GetStrategy();
    void SetStrategy(std::string);

    std::string GetSampleTime();
    void SetSampleTime(std::string);

    std::string GetCommunication();
    void SetCommunication(std::string);

    std::string GetLogErro();
    std::string GetLogRef();
    std::string GetLogOut();
    std::string GetLogIn();
    void SetLog(std::string erro,std::string ref,std::string out,std::string in);

    std::string GetStepTopic();
    void SetStepTopic(std::string topic);

    void CreateFile();
    void ReadFile();
    void WriteFile();

    void ModifyParameter();
    void AddSensorToFile();
    void DeleteSensorToFile();
    void AddActuatorToFile();
    void DeleteActuatorToFile();

    std::string Readitem(std::string);
    std::vector<std::string> ReadAllItems(std::string);

private:
    std::string Filename;
    QDomDocument document;
    QFile file;
    std::vector<std::string> Sensors;
    std::vector<std::string> Actuators;
    std::string control_strategy;
    std::string sample_time;
    std::string communication;
    std::string erroFilename;
    std::string refFilename;
    std::string outFilename;
    std::string inFilename;
    std::string stepTopic;
};

#endif // CONFIGFILE_H
