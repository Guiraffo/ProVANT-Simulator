#include "modelplugin.h"

ModelPlugin::ModelPlugin()
{

}

bool ModelPlugin::Read(QDomNode* document)
{
    static bool i = 1;
    if(i==1){
        i = 0;
        *document = document->firstChildElement("plugin");
    }
    else
    {
        *document = document->nextSiblingElement("plugin");
    }
    if(document->isNull())
    {
        i = 1;
        return true;
    }

    plugin_DA::Read(document);
    QDomNodeList list = document->childNodes();
    parameters.clear();
    values.clear();
    for(uint i = 0; i<list.count();i++)
    {
        parameters.push_back(list.at(i).toElement().tagName().toStdString());
        values.push_back(list.at(i).toElement().text().toStdString());
    }
    return false;
}

void ModelPlugin::Write(QXmlStreamWriter*xml)
{
    xml->writeStartElement("plugin");
    xml->writeAttribute("name",name.c_str());
    xml->writeAttribute("filename",filename.c_str());
    for(uint i = 0;i<parameters.size();i++)
    {
        xml->writeTextElement(parameters.at(i).c_str(),values.at(i).c_str());
    }
    xml->writeEndElement();

}

void ModelPlugin::print()
{
    plugin_DA::print();
    for(uint i = 0;i<parameters.size();i++)
    {
        qDebug() << parameters.at(i).c_str() << "  " << values.at(i).c_str();
    }
}

