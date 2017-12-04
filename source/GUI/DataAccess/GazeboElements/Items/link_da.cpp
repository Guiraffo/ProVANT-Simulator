#include "link_da.h"

link_DA::link_DA()
{

}

bool link_DA::Read(QDomNode* document)
{
    static bool i = 1;
    if(i==1){
        i = 0;
        *document = document->firstChildElement("link");
    }
    else
    {
        *document = document->nextSiblingElement("link");
    }
    if(document->isNull())
    {
        i = 1;
        return true;
    }
    name = document->toElement().attribute("name")
                   .toStdString();

    pose = document->firstChildElement("pose")
                   .text()
                   .toStdString();

    inertialValues.Read(*document);
    collision.Read(*document);
    visual.Read(*document);
    return false;

}

void link_DA::Write(QXmlStreamWriter* xml,int n,std::vector<sensor> sensors)
{
    xml->writeStartElement("link");
    xml->writeAttribute("name",name.c_str());
    xml->writeTextElement("pose",pose.c_str());
    inertialValues.Write(xml);
    collision.Write(xml);
    visual.Write(xml);
    for(uint i=0;i<sensors.size();i++)
    {
        if(sensors.at(i).link == n)
        {
            sensors.at(i).Write(xml);
        }
    }
    xml->writeEndElement();
}

void link_DA::print()
{
    qDebug() << "Link";
    qDebug() << "Name: " << name.c_str();
    qDebug() << "Pose: " << pose.c_str();
    inertialValues.print();
    collision.print();
    visual.print();
}

