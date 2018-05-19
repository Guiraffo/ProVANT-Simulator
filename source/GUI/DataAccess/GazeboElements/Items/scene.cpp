#include "scene.h"

scene::scene()
{

}


void scene::Write(QXmlStreamWriter xml)
{
    /*xml->writeStartElement("scene");
    xml->writeStartElement("sky");
    xml->writeTextElement("time","18");
    xml->writeStartElement("clouds");
    xml->writeTextElement("speed","0");
    xml->writeEndElement();
    xml->writeEndElement();
    xml->writeEndElement();*/
}

void scene::Read(QDomNode document)
{
     document = document.firstChildElement("scene");
}

void scene::print()
{

}


