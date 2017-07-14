#include "physics_da.h"

physics_DA::physics_DA()
{

}

std::string physics_DA::GetType(){return type;}
void physics_DA::SetType(std::string value){type = value;}
std::string physics_DA::GetStep(){return step;}
void physics_DA::SetStep(std::string value){step = value;}
std::string physics_DA::GetRealTimeFactor(){return real_time_factor;}
void physics_DA::SetRealTimeFactor(std::string value){real_time_factor = value;;}
std::string physics_DA::GetRealTimeUpdaterate(){return real_time_update_rate;}
void physics_DA::SetRealTimeUpdaterate(std::string value){real_time_update_rate = value;}


void physics_DA::Write(QXmlStreamWriter* xml)
{
    xml->writeStartElement("physics");
    xml->writeAttribute("type",type.c_str());
    xml->writeTextElement("max_step_size",step.c_str());
    xml->writeTextElement("real_time_factor",real_time_factor.c_str());
    if(real_time_update_rate.size()!=0)xml->writeTextElement("real_time_update_rate",real_time_update_rate.c_str());
    xml->writeEndElement();
}

void physics_DA::Read(QDomNode document)
{
    type = document.firstChildElement("physics")
                   .attribute("type").toStdString();
    step = document.firstChildElement("physics")
                   .firstChildElement("max_step_size")
                   .text().toStdString();
    real_time_factor = document.firstChildElement("physics")
                       .firstChildElement("real_time_factor")
                       .text().toStdString();
    real_time_update_rate = document.firstChildElement("physics")
                  .firstChildElement("real_time_update_rate")
                  .text().toStdString();
}

void physics_DA::print()
{
    qDebug()<<"Physics";
    qDebug()<<"type " << type.c_str();
    qDebug()<<"step "  << step.c_str();
    qDebug()<<"real_time_factor "  << real_time_factor.c_str();
    qDebug()<<"real_time_update_rate " << real_time_update_rate.c_str();
}
