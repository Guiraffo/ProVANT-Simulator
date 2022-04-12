#include "material.h"

Material::Material()
{
}
std::string Material::GetEmissive()
{
  return emissive;
}
std::string Material::GetDiffuse()
{
  return diffuse;
}
std::string Material::GetSpecular()
{
  return specular;
}
std::string Material::GetAmbient()
{
  return ambient;
}
void Material::SetEmissive(std::string value)
{
  emissive = value;
}
void Material::SetDiffuse(std::string value)
{
  diffuse = value;
}
void Material::SetSpecular(std::string value)
{
  specular = value;
}
void Material::SetAmbient(std::string value)
{
  ambient = value;
}
void Material::Read(QDomNode document)
{
  ambient = document.firstChildElement("material")
                .firstChildElement("ambient")
                .text()
                .toStdString();
  diffuse = document.firstChildElement("material")
                .firstChildElement("diffuse")
                .text()
                .toStdString();
  specular = document.firstChildElement("material")
                 .firstChildElement("specular")
                 .text()
                 .toStdString();
  emissive = document.firstChildElement("material")
                 .firstChildElement("emissive")
                 .text()
                 .toStdString();
}
void Material::Write(QXmlStreamWriter* xml)
{
  if (ambient != "" && diffuse != "" && specular != "" && emissive != "")
  {
    xml->writeStartElement("material");
    xml->writeTextElement("ambient", ambient.c_str());
    xml->writeTextElement("diffuse", diffuse.c_str());
    xml->writeTextElement("specular", specular.c_str());
    xml->writeTextElement("emissive", emissive.c_str());
    xml->writeEndElement();
  }
}
void Material::print()
{
  qDebug() << "Material";
  qDebug() << "Emissive: " << emissive.c_str();
  qDebug() << "Diffuse: " << diffuse.c_str();
  qDebug() << "Specular: " << specular.c_str();
  qDebug() << "Ambient: " << ambient.c_str();
}
