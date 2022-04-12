#include "collision.h"

Collision::Collision()
{
  geometry = new GeometryMesh;
}

void Collision::Read(QDomNode document)
{
  name =
      document.firstChildElement("collision").attribute("name").toStdString();

  pose = document.firstChildElement("collision")
             .firstChildElement("pose")
             .text()
             .toStdString();

  geometry->Read(document.firstChildElement("collision"));
}
void Collision::Write(QXmlStreamWriter* xml)
{
  if (name != "")
  {
    xml->writeStartElement("collision");
    xml->writeAttribute("name", name.c_str());
    if (pose != "")
      xml->writeTextElement("pose", pose.c_str());
    geometry->Write(xml);
    xml->writeEndElement();
  }
}
void Collision::print()
{
  qDebug() << "Collision";
  qDebug() << "name " << name.c_str();
  qDebug() << "pose " << pose.c_str();
  geometry->print();
}
