
#include "sensor.h"

void sensor::ReadImu(QDomNode* document)
{
    this->ang_x.type = document->firstChildElement("imu").
                                 firstChildElement("angular_velocity").
                                 firstChildElement("x").
                                 firstChildElement("noise").
                                 attribute("type");

    this->ang_x.mean = document->firstChildElement("imu").
                       firstChildElement("angular_velocity").
                       firstChildElement("x").
                       firstChildElement("noise").
                       firstChildElement("mean").text();

    this->ang_x.stddev = document->firstChildElement("imu").
                       firstChildElement("angular_velocity").
                       firstChildElement("x").
                       firstChildElement("noise").
                       firstChildElement("stddev").text();

    this->ang_x.bias_mean = document->firstChildElement("imu").
                       firstChildElement("angular_velocity").
                       firstChildElement("x").
                       firstChildElement("noise").
                       firstChildElement("bias_mean").text();

    this->ang_x.bias_stddev = document->firstChildElement("imu").
                       firstChildElement("angular_velocity").
                       firstChildElement("x").
                       firstChildElement("noise").
                       firstChildElement("bias_stddev").text();

    this->ang_x.precision = document->firstChildElement("imu").
                       firstChildElement("angular_velocity").
                       firstChildElement("x").
                       firstChildElement("noise").
                       firstChildElement("precision").text();


    this->ang_y.type = document->firstChildElement("imu").
                                 firstChildElement("angular_velocity").
                                 firstChildElement("y").
                                 firstChildElement("noise").
                                 attribute("type");

    this->ang_y.mean = document->firstChildElement("imu").
                       firstChildElement("angular_velocity").
                       firstChildElement("y").
                       firstChildElement("noise").
                       firstChildElement("mean").text();

    this->ang_y.stddev = document->firstChildElement("imu").
                       firstChildElement("angular_velocity").
                       firstChildElement("y").
                       firstChildElement("noise").
                       firstChildElement("stddev").text();

    this->ang_y.bias_mean = document->firstChildElement("imu").
                       firstChildElement("angular_velocity").
                       firstChildElement("y").
                       firstChildElement("noise").
                       firstChildElement("bias_mean").text();

    this->ang_y.bias_stddev = document->firstChildElement("imu").
                       firstChildElement("angular_velocity").
                       firstChildElement("y").
                       firstChildElement("noise").
                       firstChildElement("bias_stddev").text();

    this->ang_y.precision = document->firstChildElement("imu").
                       firstChildElement("angular_velocity").
                       firstChildElement("y").
                       firstChildElement("noise").
                       firstChildElement("precision").text();


    this->ang_z.type = document->firstChildElement("imu").
                                 firstChildElement("angular_velocity").
                                 firstChildElement("z").
                                 firstChildElement("noise").
                                 attribute("type");

    this->ang_z.mean = document->firstChildElement("imu").
                       firstChildElement("angular_velocity").
                       firstChildElement("z").
                       firstChildElement("noise").
                       firstChildElement("mean").text();

    this->ang_z.stddev = document->firstChildElement("imu").
                       firstChildElement("angular_velocity").
                       firstChildElement("z").
                       firstChildElement("noise").
                       firstChildElement("stddev").text();

    this->ang_z.bias_mean = document->firstChildElement("imu").
                       firstChildElement("angular_velocity").
                       firstChildElement("z").
                       firstChildElement("noise").
                       firstChildElement("bias_mean").text();

    this->ang_z.bias_stddev = document->firstChildElement("imu").
                       firstChildElement("angular_velocity").
                       firstChildElement("z").
                       firstChildElement("noise").
                       firstChildElement("bias_stddev").text();

    this->ang_z.precision = document->firstChildElement("imu").
                       firstChildElement("angular_velocity").
                       firstChildElement("z").
                       firstChildElement("noise").
                       firstChildElement("precision").text();

    ////////////////////////////////////////////////////////////////////////

    this->accel_x.type = document->firstChildElement("imu").
                                 firstChildElement("linear_acceleration").
                                 firstChildElement("x").
                                 firstChildElement("noise").
                                 attribute("type");

    this->accel_x.mean = document->firstChildElement("imu").
                       firstChildElement("linear_acceleration").
                       firstChildElement("x").
                       firstChildElement("noise").
                       firstChildElement("mean").text();

    this->accel_x.stddev = document->firstChildElement("imu").
                       firstChildElement("linear_acceleration").
                       firstChildElement("x").
                       firstChildElement("noise").
                       firstChildElement("stddev").text();

    this->accel_x.bias_mean = document->firstChildElement("imu").
                       firstChildElement("linear_acceleration").
                       firstChildElement("x").
                       firstChildElement("noise").
                       firstChildElement("bias_mean").text();

    this->accel_x.bias_stddev = document->firstChildElement("imu").
                       firstChildElement("linear_acceleration").
                       firstChildElement("x").
                       firstChildElement("noise").
                       firstChildElement("bias_stddev").text();

    this->accel_x.precision = document->firstChildElement("imu").
                       firstChildElement("linear_acceleration").
                       firstChildElement("x").
                       firstChildElement("noise").
                       firstChildElement("precision").text();


    this->accel_y.type = document->firstChildElement("imu").
                                 firstChildElement("linear_acceleration").
                                 firstChildElement("y").
                                 firstChildElement("noise").
                                 attribute("type");

    this->accel_y.mean = document->firstChildElement("imu").
                       firstChildElement("linear_acceleration").
                       firstChildElement("y").
                       firstChildElement("noise").
                       firstChildElement("mean").text();

    this->accel_y.stddev = document->firstChildElement("imu").
                       firstChildElement("linear_acceleration").
                       firstChildElement("y").
                       firstChildElement("noise").
                       firstChildElement("stddev").text();

    this->accel_y.bias_mean = document->firstChildElement("imu").
                       firstChildElement("linear_acceleration").
                       firstChildElement("y").
                       firstChildElement("noise").
                       firstChildElement("bias_mean").text();

    this->accel_y.bias_stddev = document->firstChildElement("imu").
                       firstChildElement("linear_acceleration").
                       firstChildElement("y").
                       firstChildElement("noise").
                       firstChildElement("bias_stddev").text();

    this->accel_y.precision = document->firstChildElement("imu").
                       firstChildElement("linear_acceleration").
                       firstChildElement("y").
                       firstChildElement("noise").
                       firstChildElement("precision").text();


    this->accel_z.type = document->firstChildElement("imu").
                                 firstChildElement("linear_acceleration").
                                 firstChildElement("z").
                                 firstChildElement("noise").
                                 attribute("type");

    this->accel_z.mean = document->firstChildElement("imu").
                       firstChildElement("linear_acceleration").
                       firstChildElement("z").
                       firstChildElement("noise").
                       firstChildElement("mean").text();

    this->accel_z.stddev = document->firstChildElement("imu").
                       firstChildElement("linear_acceleration").
                       firstChildElement("z").
                       firstChildElement("noise").
                       firstChildElement("stddev").text();

    this->accel_z.bias_mean = document->firstChildElement("imu").
                       firstChildElement("linear_acceleration").
                       firstChildElement("z").
                       firstChildElement("noise").
                       firstChildElement("bias_mean").text();

    this->accel_z.bias_stddev = document->firstChildElement("imu").
                       firstChildElement("linear_acceleration").
                       firstChildElement("z").
                       firstChildElement("noise").
                       firstChildElement("bias_stddev").text();

    this->accel_z.precision = document->firstChildElement("imu").
                       firstChildElement("linear_acceleration").
                       firstChildElement("z").
                       firstChildElement("noise").
                       firstChildElement("precision").text();


}
void sensor::WriteImu(QXmlStreamWriter* xml)
{
    xml->writeStartElement("imu");
        xml->writeStartElement("angular_velocity");
            xml->writeStartElement("x");
                xml->writeStartElement("noise");
                    xml->writeAttribute("type",this->ang_x.type);
                    xml->writeTextElement("mean",this->ang_x.mean);
                    xml->writeTextElement("stddev",this->ang_x.stddev);
                    xml->writeTextElement("bias_mean",this->ang_x.bias_mean);
                    xml->writeTextElement("bias_stddev",this->ang_x.bias_stddev);
                    xml->writeTextElement("precision",this->ang_x.precision);
                xml->writeEndElement();
            xml->writeEndElement();
            xml->writeStartElement("y");
                xml->writeStartElement("noise");
                    xml->writeAttribute("type",this->ang_y.type);
                    xml->writeTextElement("mean",this->ang_y.mean);
                    xml->writeTextElement("stddev",this->ang_y.stddev);
                    xml->writeTextElement("bias_mean",this->ang_y.bias_mean);
                    xml->writeTextElement("bias_stddev",this->ang_y.bias_stddev);
                    xml->writeTextElement("precision",this->ang_y.precision);
                xml->writeEndElement();
            xml->writeEndElement();
            xml->writeStartElement("z");
                xml->writeStartElement("noise");
                    xml->writeAttribute("type",this->ang_z.type);
                    xml->writeTextElement("mean",this->ang_z.mean);
                    xml->writeTextElement("stddev",this->ang_z.stddev);
                    xml->writeTextElement("bias_mean",this->ang_z.bias_mean);
                    xml->writeTextElement("bias_stddev",this->ang_z.bias_stddev);
                    xml->writeTextElement("precision",this->ang_z.precision);
                xml->writeEndElement();
            xml->writeEndElement();
        xml->writeEndElement();
        xml->writeStartElement("linear_acceleration");
            xml->writeStartElement("x");
                xml->writeStartElement("noise");
                    xml->writeAttribute("type",this->accel_x.type);
                    xml->writeTextElement("mean",this->accel_x.mean);
                    xml->writeTextElement("stddev",this->accel_x.stddev);
                    xml->writeTextElement("bias_mean",this->accel_x.bias_mean);
                    xml->writeTextElement("bias_stddev",this->accel_x.bias_stddev);
                    xml->writeTextElement("precision",this->accel_x.precision);
                xml->writeEndElement();
            xml->writeEndElement();
            xml->writeStartElement("y");
                xml->writeStartElement("noise");
                    xml->writeAttribute("type",this->accel_y.type);
                    xml->writeTextElement("mean",this->accel_y.mean);
                    xml->writeTextElement("stddev",this->accel_y.stddev);
                    xml->writeTextElement("bias_mean",this->accel_y.bias_mean);
                    xml->writeTextElement("bias_stddev",this->accel_y.bias_stddev);
                    xml->writeTextElement("precision",this->accel_y.precision);
                xml->writeEndElement();
            xml->writeEndElement();
            xml->writeStartElement("z");
                xml->writeStartElement("noise");
                    xml->writeAttribute("type",this->accel_z.type);
                    xml->writeTextElement("mean",this->accel_z.mean);
                    xml->writeTextElement("stddev",this->accel_z.stddev);
                    xml->writeTextElement("bias_mean",this->accel_z.bias_mean);
                    xml->writeTextElement("bias_stddev",this->accel_z.bias_stddev);
                    xml->writeTextElement("precision",this->accel_z.precision);
                xml->writeEndElement();
            xml->writeEndElement();
        xml->writeEndElement();
    xml->writeEndElement();
}
void sensor::printImu()
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

    qDebug() << "imu";
    qDebug() << "angular x";
    qDebug() << "type " << this->ang_x.type;
    qDebug() << "mean " << this->ang_x.mean;
    qDebug() << "stddev " << this->ang_x.stddev;
    qDebug() << "bias_mean " << this->ang_x.bias_mean;
    qDebug() << "bias_stddev " << this->ang_x.bias_stddev;
    qDebug() << "precision " << this->ang_x.precision;
    qDebug() << "angular y";
    qDebug() << "type " << this->ang_y.type;
    qDebug() << "mean " << this->ang_y.mean;
    qDebug() << "stddev " << this->ang_y.stddev;
    qDebug() << "bias_mean " << this->ang_y.bias_mean;
    qDebug() << "bias_stddev " << this->ang_y.bias_stddev;
    qDebug() << "precision " << this->ang_y.precision;
    qDebug() << "angular z";
    qDebug() << "type " << this->ang_z.type;
    qDebug() << "mean " << this->ang_z.mean;
    qDebug() << "stddev " << this->ang_z.stddev;
    qDebug() << "bias_mean " << this->ang_z.bias_mean;
    qDebug() << "bias_stddev " << this->ang_z.bias_stddev;
    qDebug() << "precision " << this->ang_z.precision;
    qDebug() << "acceleration x";
    qDebug() << "type " << this->accel_x.type;
    qDebug() << "mean " << this->accel_x.mean;
    qDebug() << "stddev " << this->accel_x.stddev;
    qDebug() << "bias_mean " << this->accel_x.bias_mean;
    qDebug() << "bias_stddev " << this->accel_x.bias_stddev;
    qDebug() << "precision " << this->accel_x.precision;
    qDebug() << "acceleration y";
    qDebug() << "type " << this->accel_y.type;
    qDebug() << "mean " << this->accel_y.mean;
    qDebug() << "stddev " << this->accel_y.stddev;
    qDebug() << "bias_mean " << this->accel_y.bias_mean;
    qDebug() << "bias_stddev " << this->accel_y.bias_stddev;
    qDebug() << "precision " << this->accel_y.precision;
    qDebug() << "acceleration z";
    qDebug() << "type " << this->accel_z.type;
    qDebug() << "mean " << this->accel_z.mean;
    qDebug() << "stddev " << this->accel_z.stddev;
    qDebug() << "bias_mean " << this->accel_z.bias_mean;
    qDebug() << "bias_stddev " << this->accel_z.bias_stddev;
    qDebug() << "precision " << this->accel_z.precision;

}

