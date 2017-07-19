#include "sensor.h"

void sensor::ReadMagnetometer(QDomNode* document)
{
    this->x.type = document->firstChildElement("magnetometer").
                                 firstChildElement("x").
                                 firstChildElement("noise").
                                 attribute("type");

    this->x.mean = document->firstChildElement("magnetometer").
                       firstChildElement("x").
                       firstChildElement("noise").
                       firstChildElement("mean").text();

    this->x.stddev = document->firstChildElement("magnetometer").
                       firstChildElement("x").
                       firstChildElement("noise").
                       firstChildElement("stddev").text();

    this->x.bias_mean = document->firstChildElement("magnetometer").
                       firstChildElement("x").
                       firstChildElement("noise").
                       firstChildElement("bias_mean").text();

    this->x.bias_stddev = document->firstChildElement("magnetometer").
                       firstChildElement("x").
                       firstChildElement("noise").
                       firstChildElement("bias_stddev").text();

    this->x.precision = document->firstChildElement("magnetometer").
                       firstChildElement("x").
                       firstChildElement("noise").
                       firstChildElement("precision").text();



    this->y.type = document->firstChildElement("magnetometer").
                                 firstChildElement("y").
                                 firstChildElement("noise").
                                 attribute("type");

    this->y.mean = document->firstChildElement("magnetometer").
                       firstChildElement("y").
                       firstChildElement("noise").
                       firstChildElement("mean").text();

    this->y.stddev = document->firstChildElement("magnetometer").
                       firstChildElement("y").
                       firstChildElement("noise").
                       firstChildElement("stddev").text();

    this->y.bias_mean = document->firstChildElement("magnetometer").
                       firstChildElement("y").
                       firstChildElement("noise").
                       firstChildElement("bias_mean").text();

    this->y.bias_stddev = document->firstChildElement("magnetometer").
                       firstChildElement("y").
                       firstChildElement("noise").
                       firstChildElement("bias_stddev").text();

    this->y.precision = document->firstChildElement("magnetometer").
                       firstChildElement("y").
                       firstChildElement("noise").
                       firstChildElement("precision").text();


    this->z.type = document->firstChildElement("magnetometer").
                                 firstChildElement("z").
                                 firstChildElement("noise").
                                 attribute("type");

    this->z.mean = document->firstChildElement("magnetometer").
                       firstChildElement("z").
                       firstChildElement("noise").
                       firstChildElement("mean").text();

    this->z.stddev = document->firstChildElement("magnetometer").
                       firstChildElement("z").
                       firstChildElement("noise").
                       firstChildElement("stddev").text();

    this->z.bias_mean = document->firstChildElement("magnetometer").
                       firstChildElement("z").
                       firstChildElement("noise").
                       firstChildElement("bias_mean").text();

    this->z.bias_stddev = document->firstChildElement("magnetometer").
                       firstChildElement("z").
                       firstChildElement("noise").
                       firstChildElement("bias_stddev").text();

    this->z.precision = document->firstChildElement("magnetometer").
                       firstChildElement("z").
                       firstChildElement("noise").
                       firstChildElement("precision").text();


}
void sensor::WriteMagnetometer(QXmlStreamWriter* xml)
{
    xml->writeStartElement("magnetometer");
            xml->writeStartElement("x");
                xml->writeStartElement("noise");
                    xml->writeAttribute("type",this->x.type);
                    xml->writeTextElement("mean",this->x.mean);
                    xml->writeTextElement("stddev",this->x.stddev);
                    xml->writeTextElement("bias_mean",this->x.bias_mean);
                    xml->writeTextElement("bias_stddev",this->x.bias_stddev);
                    xml->writeTextElement("precision",this->x.precision);
                xml->writeEndElement();
            xml->writeEndElement();
            xml->writeStartElement("y");
                xml->writeStartElement("noise");
                    xml->writeAttribute("type",this->y.type);
                    xml->writeTextElement("mean",this->y.mean);
                    xml->writeTextElement("stddev",this->y.stddev);
                    xml->writeTextElement("bias_mean",this->y.bias_mean);
                    xml->writeTextElement("bias_stddev",this->y.bias_stddev);
                    xml->writeTextElement("precision",this->y.precision);
                xml->writeEndElement();
            xml->writeEndElement();
            xml->writeStartElement("z");
                xml->writeStartElement("noise");
                    xml->writeAttribute("type",this->z.type);
                    xml->writeTextElement("mean",this->z.mean);
                    xml->writeTextElement("stddev",this->z.stddev);
                    xml->writeTextElement("bias_mean",this->z.bias_mean);
                    xml->writeTextElement("bias_stddev",this->z.bias_stddev);
                    xml->writeTextElement("precision",this->z.precision);
                xml->writeEndElement();
            xml->writeEndElement();
      xml->writeEndElement();
}
void sensor::printMagnetometer()
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


    qDebug() << "magnetometer";
    qDebug() << "x";
    qDebug() << "type " << this->x.type;
    qDebug() << "mean " << this->x.mean;
    qDebug() << "stddev " << this->x.stddev;
    qDebug() << "bias_mean " << this->x.bias_mean;
    qDebug() << "bias_stddev " << this->x.bias_stddev;
    qDebug() << "precision " << this->x.precision;
    qDebug() << "y";
    qDebug() << "type " << this->y.type;
    qDebug() << "mean " << this->y.mean;
    qDebug() << "stddev " << this->y.stddev;
    qDebug() << "bias_mean " << this->y.bias_mean;
    qDebug() << "bias_stddev " << this->y.bias_stddev;
    qDebug() << "precision " << this->y.precision;
    qDebug() << "z";
    qDebug() << "type " << this->z.type;
    qDebug() << "mean " << this->z.mean;
    qDebug() << "stddev " << this->z.stddev;
    qDebug() << "bias_mean " << this->z.bias_mean;
    qDebug() << "bias_stddev " << this->z.bias_stddev;
    qDebug() << "precision " << this->z.precision;

}


