#include "joint_da.h"

joint_DA::joint_DA()
{
    one = new Axis("axis");
    two = new Axis("axis2");

}

bool joint_DA::Read(QDomNode* document)
{
    static bool i = 1;
    if(i==1){
        i = 0;
        *document = document->firstChildElement("joint");
    }
    else
    {
        *document = document->nextSiblingElement("joint");
    }
    if(document->isNull())
    {
        i = 1;
        return true;
    }
    name = document->toElement().attribute("name")
                   .toStdString();
    type = document->toElement().attribute("type")
                   .toStdString();

    pose = document->firstChildElement("pose")
                   .text()
                   .toStdString();
    parent = document->firstChildElement("parent")
                   .text()
                   .toStdString();
    child = document->firstChildElement("child")
                   .text()
                   .toStdString();



    one->Read(*document);
    if(!document->firstChildElement("axis2").isNull()) two->Read(*document);
    else two = NULL;
    return false;
}

void joint_DA::Write(QXmlStreamWriter* xml)
{
    xml->writeStartElement("joint");
    xml->writeAttribute("name",name.c_str());
    xml->writeAttribute("type",type.c_str());
    xml->writeTextElement("pose",pose.c_str());
    xml->writeTextElement("parent",parent.c_str());
    xml->writeTextElement("child",child.c_str());
    if(one!=NULL)one->Write(xml);
    if(two!=NULL)two->Write(xml);
    xml->writeEndElement();
}
void joint_DA::print()
{
    qDebug() << "Joint";
    qDebug() << "name " << name.c_str();
    qDebug() << "type " << type.c_str();
    qDebug() << "pose " << pose.c_str();
    qDebug() << "parent " << parent.c_str();
    qDebug() << "child " << child.c_str();
    if(one!=NULL) one->print();
    if(two!=NULL) two->print();
}

