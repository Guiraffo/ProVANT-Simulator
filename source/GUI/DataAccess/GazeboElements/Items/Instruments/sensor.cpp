#include "sensor.h"
#include "QDebug"

sensor::sensor()
{
}

bool sensor::Read(QDomNode* document, bool* out2)
{
  static bool i = 1;
  static bool j = 1;
  static int cont = 0;
  static bool endsensors = false;
  static QDomNode element;
  *out2 = false;

  if (i == 1)
  {
    i = 0;
    *document = document->firstChildElement("link");
  }
  else
  {
    if (endsensors)
    {
      cont++;
      *document = document->nextSiblingElement("link");
      endsensors = false;
      j = 1;
      if (document->isNull())
      {
        i = 1;
        cont = 0;
        j = 1;
        endsensors = false;
        return true;
      }
    }
  }
  if (!endsensors)
  {
    if (j == 1)
    {
      j = 0;
      element = document->firstChildElement("sensor");
      if (element.isNull())
      {
        endsensors = true;
        return false;
      }
    }
    else
    {
      element = element.nextSiblingElement("sensor");
      if (element.isNull())
      {
        endsensors = true;
        return false;
      }
    }
  }

  link = cont;
  name = element.toElement().attribute("name");
  type = element.toElement().attribute("type");
  always_on = element.firstChildElement("always_on").text();
  update_rate = element.firstChildElement("update_rate").text();
  visualize = element.firstChildElement("visualize").text();
  topic = element.firstChildElement("topic").text();
  pose = element.firstChildElement("pose").text();
  if (name == "sonar")
    ReadSonar(&element);
  if (name == "gps")
    ReadGps(&element);
  if (name == "imu")
    ReadImu(&element);
  if (name == "magnetometer")
    ReadMagnetometer(&element);
  *out2 = true;
  return false;
}

void sensor::Write(QXmlStreamWriter* xml)
{
  xml->writeStartElement("sensor");
  xml->writeAttribute("name", name);
  xml->writeAttribute("type", type);
  xml->writeTextElement("always_on", always_on);
  xml->writeTextElement("update_rate", update_rate);
  xml->writeTextElement("visualize", visualize);
  xml->writeTextElement("topic", topic);
  xml->writeTextElement("pose", pose);
  if (name == "sonar")
    WriteSonar(xml);
  if (name == "gps")
    WriteGps(xml);
  if (name == "imu")
    WriteImu(xml);
  if (name == "magnetometer")
    WriteMagnetometer(xml);
  xml->writeEndElement();
}

void sensor::print()
{
  if (name == "sonar")
    printSonar();
  if (name == "gps")
    printGps();
  if (name == "imu")
    printImu();
  if (name == "magnetometer")
    printMagnetometer();
}
