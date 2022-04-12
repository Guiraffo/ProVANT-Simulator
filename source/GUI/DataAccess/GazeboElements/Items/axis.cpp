#include "axis.h"

Axis::Axis(std::string _type) : damping("0"), friction("0")
{
  type = _type;
}

void Axis::Read(QDomNode document)
{
  xyz = document.firstChildElement(type.c_str())
            .firstChildElement("xyz")
            .text()
            .toStdString();
  lower = document.firstChildElement(type.c_str())
              .firstChildElement("limit")
              .firstChildElement("lower")
              .text()
              .toStdString();
  upper = document.firstChildElement(type.c_str())
              .firstChildElement("limit")
              .firstChildElement("upper")
              .text()
              .toStdString();
  effort = document.firstChildElement(type.c_str())
               .firstChildElement("limit")
               .firstChildElement("effort")
               .text()
               .toStdString();
  velocity = document.firstChildElement(type.c_str())
                 .firstChildElement("limit")
                 .firstChildElement("velocity")
                 .text()
                 .toStdString();
  if (!document.firstChildElement(type.c_str())
           .firstChildElement("dynamics")
           .isNull())
  {
    damping = document.firstChildElement(type.c_str())
                  .firstChildElement("dynamics")
                  .firstChildElement("damping")
                  .text()
                  .toStdString();
    friction = document.firstChildElement(type.c_str())
                   .firstChildElement("dynamics")
                   .firstChildElement("friction")
                   .text()
                   .toStdString();
  }
  if (!document.firstChildElement(type.c_str()).isNull())
    has = true;
  else
    has = false;
}

void Axis::Write(QXmlStreamWriter* xml)
{
  if (has)
  {
    xml->writeStartElement(type.c_str());
    xml->writeTextElement("xyz", xyz.c_str());
    xml->writeStartElement("limit");
    xml->writeTextElement("lower", lower.c_str());
    xml->writeTextElement("upper", upper.c_str());
    xml->writeTextElement("effort", effort.c_str());
    xml->writeTextElement("velocity", velocity.c_str());
    xml->writeEndElement();
    xml->writeStartElement("dynamics");
    xml->writeTextElement("damping", damping.c_str());
    xml->writeTextElement("friction", friction.c_str());
    xml->writeEndElement();
    xml->writeEndElement();
  }
}

void Axis::print()
{
  if (has)
  {
    qDebug() << type.c_str();
    qDebug() << "xyz" << xyz.c_str();
    qDebug() << "lower" << lower.c_str();
    qDebug() << "upper" << upper.c_str();
    qDebug() << "effort" << effort.c_str();
    qDebug() << "velocity" << velocity.c_str();
    qDebug() << "damping" << damping.c_str();
    qDebug() << "friction" << friction.c_str();
  }
}
