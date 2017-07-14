#include "sensor.h"

void sensor::ReadSonar(QDomNode* document)
{
    min = document->firstChildElement("sonar").
                    firstChildElement("min").text();
    max = document->firstChildElement("sonar").
                    firstChildElement("max").text();
    radius = document->firstChildElement("sonar").
                       firstChildElement("radius").text();
}
void sensor::WriteSonar(QXmlStreamWriter* xml)
{
    xml->writeStartElement("sonar");
    xml->writeTextElement("min",min);
    xml->writeTextElement("max",max);
    xml->writeTextElement("radius",radius);
    xml->writeEndElement();
}
void sensor::printSonar()
{
    qDebug() << "Sensor Sonar";
    qDebug() << "link " << link; // ordem que se encontra no vetor de links
    qDebug() << "name " << name;
    qDebug() << "type " << type;
    qDebug() << "always_on " << always_on;
    qDebug() << "update_rate " << update_rate;
    qDebug() << "visualize " << visualize;
    qDebug() << "topic " << topic;
    qDebug() << "pose " <<pose;
    qDebug() << "min " <<min;
    qDebug() << "max " <<max;
    qDebug() << "radius " <<radius;
}

