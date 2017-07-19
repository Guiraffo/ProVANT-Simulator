#include "include_da.h"

Include_DA::Include_DA()
{

}

std::string Include_DA::GetUri(){return uri;}
void Include_DA::SetUri(std::string value){uri = value;}
std::string Include_DA::GetName(){return name;}
void Include_DA::SetName(std::string value){name = value;}
std::string Include_DA::GetIsStatic(){return isStatic;}
void Include_DA::SetIsStatic(std::string value){isStatic = value;}
std::string Include_DA::GetPose(){return pose;}
void Include_DA::SetPose(std::string value){pose = value;}

void Include_DA::Write(QXmlStreamWriter* xml)
{
    xml->writeStartElement("include");
    if(uri!="")xml->writeTextElement("uri",uri.c_str());
    if(name!="")xml->writeTextElement("name",name.c_str());
    if(isStatic!="")xml->writeTextElement("static",isStatic.c_str());
    if(pose!="")xml->writeTextElement("pose",pose.c_str());
    xml->writeEndElement();
}

bool Include_DA::Read(QDomNode* document)
{
    static bool i = 1;
    if(i==1){
        i = 0;
        *document = document->firstChildElement("include");
    }
    else
    {
        *document = document->nextSiblingElement("include");
    }
    if(document->isNull())
    {
        i = 1;
        return true;
    }
    uri = document->firstChildElement("uri")
                  .text().toStdString();
    name= document->firstChildElement("name")
                  .text().toStdString();
    isStatic = document->firstChildElement("static")
                       .text().toStdString();
    if(isStatic=="")isStatic="false";
    pose = document->firstChildElement("pose")
                   .text().toStdString();
    return false;
}

void Include_DA::print()
{
    qDebug() << "include";
    qDebug() << "Uri " << uri.c_str();
    qDebug() << "Name" << name.c_str();
    qDebug() << "Static " << isStatic.c_str();
    qDebug() << "Pose " << pose.c_str();
}

