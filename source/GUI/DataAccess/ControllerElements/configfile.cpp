#include "configfile.h"
#include "QDebug"

ConfigFile::ConfigFile(std::string filename):file(filename.c_str())
{
    Filename = filename;
}

std::vector<std::string> ConfigFile::GetSensors(){return Sensors;}

void ConfigFile::SetSensors(std::vector<std::string> listSensors){ Sensors = listSensors;}

void ConfigFile::AddSensor(std::string sensor){Sensors.push_back(sensor);}

void ConfigFile::DeleteSensor(int pos){Sensors.erase(Sensors.begin()+pos-1);}

std::vector<std::string> ConfigFile::GetActuators(){return Actuators;}

void ConfigFile::SetActuators(std::vector<std::string> listactuators){Actuators = listactuators;}

void ConfigFile::AddActuator(std::string actuator){Actuators.push_back(actuator);}

void ConfigFile::DeleteActuator(int pos){Actuators.erase(Actuators.begin()+pos-1);}

std::string ConfigFile::GetStrategy(){return control_strategy;}

void ConfigFile::SetStrategy(std::string strategy){control_strategy = strategy;}

std::string ConfigFile::GetSampleTime(){return sample_time;}

void ConfigFile::SetSampleTime(std::string sampletime){sample_time = sampletime;}

std::string ConfigFile::GetCommunication(){return communication;}

void ConfigFile::SetCommunication(std::string topicCommunication){communication = topicCommunication;}

std::string ConfigFile::GetLogErro(){return erroFilename;}

std::string ConfigFile::GetLogRef(){return refFilename;}

std::string ConfigFile::GetLogOut(){return outFilename;}

std::string ConfigFile::GetLogIn(){return inFilename;}

void ConfigFile::SetLog(std::string erro,std::string ref,std::string out,std::string in)
{
    erroFilename = erro;
    refFilename = ref;
    outFilename = out;
    inFilename = in;
}

std::string ConfigFile::GetStepTopic(){return stepTopic;}

void ConfigFile::SetStepTopic(std::string topic){stepTopic = topic;}

void ConfigFile::ReadFile()
{
     control_strategy = Readitem("Strategy");
     sample_time = Readitem("Sampletime");
     communication = Readitem("topicdata");
     erroFilename = Readitem("ErroPath");
     refFilename = Readitem("RefPath");
     outFilename = Readitem("Outputfile");
     inFilename = Readitem("InputPath");
     stepTopic = Readitem("TopicoStep");
     Sensors = ReadAllItems("Sensors");
     Actuators = ReadAllItems("Actuators");
}

std::string ConfigFile::Readitem(std::string tag)
{
    file.close();
    if(file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QString erro;
        int line, column;
        if(document.setContent(&file,&erro,&line,&column))
        {
            file.close();
            return  document.firstChildElement("config")
                            .firstChildElement(QString::fromStdString(tag))
                            .text()
                            .toStdString();
        }
        else
        {
            if (erro == "unexpected end of file")
            {
                qDebug() << "Arquivo Vazio";
            }
            else
            {
                qDebug()<< "Problemas com conteudo" << erro;
                qDebug("Linha %d",line);
                qDebug("Coluna %d",column);
                file.close();
                exit(1);
            }
        }
    }
    else
    {
        // TO DO: criar exceção
        std::string msg;
        msg = "2 - Problemas com o arquivo xml"+tag;
        qDebug(msg.c_str());
        file.close();
        exit(1);
    }
    file.close();
    return "";
}

std::vector<std::string> ConfigFile::ReadAllItems (std::string tag)
{
    std::vector<std::string> output;
    if(file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QString erro;
        int line, column;
        if(document.setContent(&file,&erro,&line,&column))
        {
            file.close();
            QDomNodeList devices = document.firstChildElement("config")
                                           .firstChildElement((QString::fromStdString(tag)))
                                           .elementsByTagName("Device");
            for(int i=0; i< devices.count(); i++)
            {
                QDomNode device = devices.at(i);
                if(device.isElement())
                {
                    output.push_back(device.toElement().text().toStdString());
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
                qDebug()<< "Problemas com conteudo" << erro;
                qDebug("Linha %d",line);
                qDebug("Coluna %d",column);
                file.close();
                exit(1);
            }
        }
    }
    else
    {
        // TO DO: criar exceção
        std::string msg;
        msg = "1 - Problemas com o arquivo xml"+tag;
        qDebug(msg.c_str());
        file.close();
        exit(1);
    }
    file.close();
    return output;
}

void ConfigFile::CreateFile()
{
    if ( file.open(QIODevice::ReadWrite) )
    {
        QXmlStreamWriter xml;
        xml.setAutoFormatting(true);
        xml.setDevice(&file);
        xml.writeStartDocument();
        xml.writeStartElement("config");
        xml.writeTextElement("topicdata","");
        xml.writeTextElement("TopicoStep","");
        xml.writeTextElement("Sampletime","");
        xml.writeTextElement("Strategy","");
        xml.writeTextElement("RefPath","");
        xml.writeTextElement("Outputfile","");
        xml.writeTextElement("InputPath","");
        xml.writeTextElement("ErroPath","");
        xml.writeStartElement("Sensors");
        xml.writeEndElement();
        xml.writeStartElement("Actuators");
        xml.writeEndElement();
        xml.writeEndDocument();
        file.close();
    }
}

void ConfigFile::WriteFile()
{
    if(file.open(QIODevice::ReadWrite|QIODevice::Truncate))
    {
        QXmlStreamWriter xml;
        xml.setAutoFormatting(true);
        xml.setDevice(&file);
        xml.writeStartDocument();
        xml.writeStartElement("config");
        xml.writeTextElement("topicdata",this->communication.c_str());
        xml.writeTextElement("TopicoStep",this->stepTopic.c_str());
        xml.writeTextElement("Sampletime",this->sample_time.c_str());
        xml.writeTextElement("Strategy",this->control_strategy.c_str());
        xml.writeTextElement("RefPath",this->refFilename.c_str());
        xml.writeTextElement("Outputfile",this->outFilename.c_str());
        xml.writeTextElement("InputPath",this->inFilename.c_str());
        xml.writeTextElement("ErroPath",this->erroFilename.c_str());
        xml.writeStartElement("Sensors");
        for(uint i=0;i<Sensors.size();i++)xml.writeTextElement("Device",this->Sensors.at(i).c_str());
        xml.writeEndElement();
        xml.writeStartElement("Actuators");
        for(uint i=0;i<Actuators.size();i++)xml.writeTextElement("Device",this->Actuators.at(i).c_str());
        xml.writeEndElement();
        xml.writeEndDocument();
    }
    else
    {
        // TO DO: criar exceção
        qDebug("3- Problemas com o arquivo xml");
        file.close();
        exit(1);
    }
    file.close();
}

void ConfigFile::print()
{
    qDebug() << Filename.c_str();
    qDebug() << control_strategy.c_str();
    qDebug() << sample_time.c_str();
    qDebug() << communication.c_str();
    qDebug() << erroFilename.c_str();
    qDebug() << refFilename.c_str();
    qDebug() << outFilename.c_str();
    qDebug() << inFilename.c_str();
    qDebug() << stepTopic.c_str();
    qDebug() << "Sensors:";
    for(uint i=0; i < Sensors.size();i++) qDebug() << Sensors.at(i).c_str();
    qDebug() << "Actuators:";
    for(uint i=0; i < Actuators.size();i++) qDebug() << Actuators.at(i).c_str();
}


void ConfigFile::Delete()
{
    this->Actuators.clear();
    this->Sensors.clear();
}
