#include "sensor.h"

void sensor::ReadGps(QDomNode* document)
{
  ///////////////////////////////////////////////////////////////
  this->pos_vertical.type = document->firstChildElement("gps")
                                .firstChildElement("position_sensing")
                                .firstChildElement("vertical")
                                .firstChildElement("noise")
                                .attribute("type");

  this->pos_vertical.mean = document->firstChildElement("gps")
                                .firstChildElement("position_sensing")
                                .firstChildElement("vertical")
                                .firstChildElement("noise")
                                .firstChildElement("mean")
                                .text();

  this->pos_vertical.stddev = document->firstChildElement("gps")
                                  .firstChildElement("position_sensing")
                                  .firstChildElement("vertical")
                                  .firstChildElement("noise")
                                  .firstChildElement("stddev")
                                  .text();

  this->pos_vertical.bias_mean = document->firstChildElement("gps")
                                     .firstChildElement("position_sensing")
                                     .firstChildElement("vertical")
                                     .firstChildElement("noise")
                                     .firstChildElement("bias_mean")
                                     .text();

  this->pos_vertical.bias_stddev = document->firstChildElement("gps")
                                       .firstChildElement("position_sensing")
                                       .firstChildElement("vertical")
                                       .firstChildElement("noise")
                                       .firstChildElement("bias_stddev")
                                       .text();

  this->pos_vertical.precision = document->firstChildElement("gps")
                                     .firstChildElement("position_sensing")
                                     .firstChildElement("vertical")
                                     .firstChildElement("noise")
                                     .firstChildElement("precision")
                                     .text();

  ///////////////////////////////////////////////////////////////
  this->pos_horizontal.type = document->firstChildElement("gps")
                                  .firstChildElement("position_sensing")
                                  .firstChildElement("horizontal")
                                  .firstChildElement("noise")
                                  .attribute("type");

  this->pos_horizontal.mean = document->firstChildElement("gps")
                                  .firstChildElement("position_sensing")
                                  .firstChildElement("horizontal")
                                  .firstChildElement("noise")
                                  .firstChildElement("mean")
                                  .text();

  this->pos_horizontal.stddev = document->firstChildElement("gps")
                                    .firstChildElement("position_sensing")
                                    .firstChildElement("horizontal")
                                    .firstChildElement("noise")
                                    .firstChildElement("stddev")
                                    .text();

  this->pos_horizontal.bias_mean = document->firstChildElement("gps")
                                       .firstChildElement("position_sensing")
                                       .firstChildElement("horizontal")
                                       .firstChildElement("noise")
                                       .firstChildElement("bias_mean")
                                       .text();

  this->pos_horizontal.bias_stddev = document->firstChildElement("gps")
                                         .firstChildElement("position_sensing")
                                         .firstChildElement("horizontal")
                                         .firstChildElement("noise")
                                         .firstChildElement("bias_stddev")
                                         .text();

  this->pos_horizontal.precision = document->firstChildElement("gps")
                                       .firstChildElement("position_sensing")
                                       .firstChildElement("horizontal")
                                       .firstChildElement("noise")
                                       .firstChildElement("precision")
                                       .text();

  ///////////////////////////////////////////////////////////////
  this->vel_vertical.type = document->firstChildElement("gps")
                                .firstChildElement("velocity_sensing")
                                .firstChildElement("vertical")
                                .firstChildElement("noise")
                                .attribute("type");

  this->vel_vertical.mean = document->firstChildElement("gps")
                                .firstChildElement("velocity_sensing")
                                .firstChildElement("vertical")
                                .firstChildElement("noise")
                                .firstChildElement("mean")
                                .text();

  this->vel_vertical.stddev = document->firstChildElement("gps")
                                  .firstChildElement("velocity_sensing")
                                  .firstChildElement("vertical")
                                  .firstChildElement("noise")
                                  .firstChildElement("stddev")
                                  .text();

  this->vel_vertical.bias_mean = document->firstChildElement("gps")
                                     .firstChildElement("velocity_sensing")
                                     .firstChildElement("vertical")
                                     .firstChildElement("noise")
                                     .firstChildElement("bias_mean")
                                     .text();

  this->vel_vertical.bias_stddev = document->firstChildElement("gps")
                                       .firstChildElement("velocity_sensing")
                                       .firstChildElement("vertical")
                                       .firstChildElement("noise")
                                       .firstChildElement("bias_stddev")
                                       .text();

  this->vel_vertical.precision = document->firstChildElement("gps")
                                     .firstChildElement("velocity_sensing")
                                     .firstChildElement("vertical")
                                     .firstChildElement("noise")
                                     .firstChildElement("precision")
                                     .text();

  ///////////////////////////////////////////////////////////////
  this->vel_horizontal.type = document->firstChildElement("gps")
                                  .firstChildElement("velocity_sensing")
                                  .firstChildElement("horizontal")
                                  .firstChildElement("noise")
                                  .attribute("type");

  this->vel_horizontal.mean = document->firstChildElement("gps")
                                  .firstChildElement("velocity_sensing")
                                  .firstChildElement("horizontal")
                                  .firstChildElement("noise")
                                  .firstChildElement("mean")
                                  .text();

  this->vel_horizontal.stddev = document->firstChildElement("gps")
                                    .firstChildElement("velocity_sensing")
                                    .firstChildElement("horizontal")
                                    .firstChildElement("noise")
                                    .firstChildElement("stddev")
                                    .text();

  this->vel_horizontal.bias_mean = document->firstChildElement("gps")
                                       .firstChildElement("velocity_sensing")
                                       .firstChildElement("horizontal")
                                       .firstChildElement("noise")
                                       .firstChildElement("bias_mean")
                                       .text();

  this->vel_horizontal.bias_stddev = document->firstChildElement("gps")
                                         .firstChildElement("velocity_sensing")
                                         .firstChildElement("horizontal")
                                         .firstChildElement("noise")
                                         .firstChildElement("bias_stddev")
                                         .text();

  this->vel_horizontal.precision = document->firstChildElement("gps")
                                       .firstChildElement("velocity_sensing")
                                       .firstChildElement("horizontal")
                                       .firstChildElement("noise")
                                       .firstChildElement("precision")
                                       .text();
}
void sensor::WriteGps(QXmlStreamWriter* xml)
{
  xml->writeStartElement("gps");
  xml->writeStartElement("position_sensing");
  xml->writeStartElement("horizontal");
  xml->writeStartElement("noise");
  xml->writeAttribute("type", this->pos_horizontal.type);
  xml->writeTextElement("mean", this->pos_horizontal.mean);
  xml->writeTextElement("stddev", this->pos_horizontal.stddev);
  xml->writeTextElement("bias_mean", this->pos_horizontal.bias_mean);
  xml->writeTextElement("bias_stddev", this->pos_horizontal.bias_stddev);
  xml->writeTextElement("precision", this->pos_horizontal.precision);
  xml->writeEndElement();
  xml->writeEndElement();
  xml->writeStartElement("vertical");
  xml->writeStartElement("noise");
  xml->writeAttribute("type", this->pos_vertical.type);
  xml->writeTextElement("mean", this->pos_vertical.mean);
  xml->writeTextElement("stddev", this->pos_vertical.stddev);
  xml->writeTextElement("bias_mean", this->pos_vertical.bias_mean);
  xml->writeTextElement("bias_stddev", this->pos_vertical.bias_stddev);
  xml->writeTextElement("precision", this->pos_vertical.precision);
  xml->writeEndElement();
  xml->writeEndElement();
  xml->writeEndElement();
  xml->writeStartElement("velocity_sensing");
  xml->writeStartElement("horizontal");
  xml->writeStartElement("noise");
  xml->writeAttribute("type", this->vel_horizontal.type);
  xml->writeTextElement("mean", this->vel_horizontal.mean);
  xml->writeTextElement("stddev", this->vel_horizontal.stddev);
  xml->writeTextElement("bias_mean", this->vel_horizontal.bias_mean);
  xml->writeTextElement("bias_stddev", this->vel_horizontal.bias_stddev);
  xml->writeTextElement("precision", this->vel_horizontal.precision);
  xml->writeEndElement();
  xml->writeEndElement();
  xml->writeStartElement("vertical");
  xml->writeStartElement("noise");
  xml->writeAttribute("type", this->vel_vertical.type);
  xml->writeTextElement("mean", this->vel_vertical.mean);
  xml->writeTextElement("stddev", this->vel_vertical.stddev);
  xml->writeTextElement("bias_mean", this->vel_vertical.bias_mean);
  xml->writeTextElement("bias_stddev", this->vel_vertical.bias_stddev);
  xml->writeTextElement("precision", this->vel_vertical.precision);
  xml->writeEndElement();
  xml->writeEndElement();
  xml->writeEndElement();
  xml->writeEndElement();
}
void sensor::printGps()
{
  qDebug() << "Sensor Sonar";
  qDebug() << "link " << link;  // ordem que se encontra no vetor de links
  qDebug() << "name " << name;
  qDebug() << "type " << type;
  qDebug() << "always_on " << always_on;
  qDebug() << "update_rate " << update_rate;
  qDebug() << "visualize " << visualize;
  qDebug() << "topic " << topic;
  qDebug() << "pose " << pose;

  qDebug() << "position_sensing noise vertical";
  qDebug() << "type" << this->pos_vertical.type;
  qDebug() << "mean" << this->pos_vertical.mean;
  qDebug() << "stddev" << this->pos_vertical.stddev;
  qDebug() << "bias_mean" << this->pos_vertical.bias_mean;
  qDebug() << "bias_stddev" << this->pos_vertical.bias_stddev;
  qDebug() << "precision" << this->pos_vertical.precision;

  qDebug() << "position_sensing noise horizontal";
  qDebug() << "type" << this->pos_horizontal.type;
  qDebug() << "mean" << this->pos_horizontal.mean;
  qDebug() << "stddev" << this->pos_horizontal.stddev;
  qDebug() << "bias_mean" << this->pos_horizontal.bias_mean;
  qDebug() << "bias_stddev" << this->pos_horizontal.bias_stddev;
  qDebug() << "precision" << this->pos_horizontal.precision;

  qDebug() << "velocity_sensing noise vertical";
  qDebug() << "type" << this->vel_vertical.type;
  qDebug() << "mean" << this->vel_vertical.mean;
  qDebug() << "stddev" << this->vel_vertical.stddev;
  qDebug() << "bias_mean" << this->vel_vertical.bias_mean;
  qDebug() << "bias_stddev" << this->vel_vertical.bias_stddev;
  qDebug() << "precision" << this->vel_vertical.precision;

  qDebug() << "velocity_sensing noise horizontal";
  qDebug() << "type" << this->vel_horizontal.type;
  qDebug() << "mean" << this->vel_horizontal.mean;
  qDebug() << "stddev" << this->vel_horizontal.stddev;
  qDebug() << "bias_mean" << this->vel_horizontal.bias_mean;
  qDebug() << "bias_stddev" << this->vel_horizontal.bias_stddev;
  qDebug() << "precision" << this->vel_horizontal.precision;
}
