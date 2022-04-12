#include "visual.h"

Visual::Visual()
{
  geometry = new GeometryMesh;
}

void Visual::Read(QDomNode document)
{
  name = document.firstChildElement("visual").attribute("name").toStdString();

  pose = document.firstChildElement("visual")
             .firstChildElement("pose")
             .text()
             .toStdString();

  geometry->Read(document.firstChildElement("visual"));
  material.Read(document.firstChildElement("visual"));
}
void Visual::Write(QXmlStreamWriter* xml)
{
  if (name != "")
  {
    xml->writeStartElement("visual");
    xml->writeAttribute("name", name.c_str());
    if (pose != "")
      xml->writeTextElement("pose", pose.c_str());
    geometry->Write(xml);
    material.Write(xml);
    xml->writeEndElement();
  }
}
void Visual::print()
{
  qDebug() << "Visual";
  qDebug() << "Pose " << pose.c_str();
  geometry->print();
  material.print();
}
