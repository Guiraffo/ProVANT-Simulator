#include "modelplugin.h"

ModelPlugin::ModelPlugin()
{

}




void ModelPlugin::Write(QXmlStreamWriter xml)
{
    /*xml->writeStartElement("plugin");
    xml->writeAttribute("name",name.c_str());
    xml->writeAttribute("filename",filename.c_str());
    for(uint i = 0;i<parameters.size();i++)
    {
        xml->writeTextElement(parameters.at(i).c_str(),values.at(i).c_str());
    }
    xml->writeEndElement();*/

}

void ModelPlugin::print()
{
    plugin_DA::print();
    for(uint i = 0;i<parameters.size();i++)
    {
        qDebug() << parameters.at(i).c_str() << "  " << values.at(i).c_str();
    }
}

