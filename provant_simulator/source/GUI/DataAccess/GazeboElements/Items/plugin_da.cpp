#include "plugin_da.h"

plugin_DA::plugin_DA()
{

}

std::string plugin_DA::GetName(){return name;}
void plugin_DA::SetName(std::string value){name = value;}
std::string plugin_DA::GetFilename(){return filename;}
void plugin_DA::SetFilename(std::string value){filename = value;}
bool plugin_DA::Read(QDomNode* node)
{
     name = node->toElement().attribute("name").toStdString();
     filename = node->toElement().attribute("filename").toStdString();
     return true;
}
void plugin_DA::Write(QXmlStreamWriter* xml)
{
    xml->writeStartElement("plugin");
    xml->writeAttribute("name",name.c_str());
    xml->writeAttribute("filename",filename.c_str());
    xml->writeEndElement();
}
void plugin_DA::print()
{
    qDebug() << "plugin";
    qDebug() << "name " << name.c_str();
    qDebug() << "filename " << filename.c_str();
}





