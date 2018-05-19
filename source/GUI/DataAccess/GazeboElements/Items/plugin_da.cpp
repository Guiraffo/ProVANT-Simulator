#include "plugin_da.h"

plugin_DA::plugin_DA()
{

}

std::string plugin_DA::GetName(){return name;}
void plugin_DA::SetName(std::string value){name = value;}
std::string plugin_DA::GetFilename(){return filename;}
void plugin_DA::SetFilename(std::string value){filename = value;}

void plugin_DA::Write(QXmlStreamWriter* xml)
{
    /*xml->writeStartElement("plugin");
    xml->writeAttribute("name",name.c_str());
    xml->writeAttribute("filename",filename.c_str());
    xml->writeEndElement();*/
    /*xml->writeStartElement("plugin");
    xml->writeAttribute("name",name.c_str());
    xml->writeAttribute("filename",filename.c_str());
    for(uint i = 0;i<parameters.size();i++)
    {
        xml->writeTextElement(parameters.at(i).c_str(),values.at(i).c_str());
    }
    xml->writeEndElement();*/
}
void plugin_DA::print()
{
    qDebug() << "plugin";
    qDebug() << "name " << name.c_str();
    qDebug() << "filename " << filename.c_str();
    for(uint i = 0;i<parameters.size();i++)
    {
        qDebug() << parameters.at(i).c_str() << "  " << values.at(i).c_str();
    }
}











