#include "inertial.h"

Inertial::Inertial()
{
}

std::string Inertial::GetPose()
{
  return pose;
}
std::string Inertial::GetMass()
{
  return mass;
}
std::string Inertial::GetIxx()
{
  return ixx;
}
std::string Inertial::GetIxy()
{
  return ixy;
}
std::string Inertial::GetIxz()
{
  return ixz;
}
std::string Inertial::GetIyy()
{
  return iyy;
}
std::string Inertial::GetIyz()
{
  return iyz;
}
std::string Inertial::GetIzz()
{
  return izz;
}
void Inertial::SetPose(std::string value)
{
  pose = value;
}
void Inertial::SetMass(std::string value)
{
  mass = value;
}
void Inertial::SetIxx(std::string value)
{
  ixx = value;
}
void Inertial::SetIxy(std::string value)
{
  ixy = value;
}
void Inertial::SetIxz(std::string value)
{
  ixz = value;
}
void Inertial::SetIyy(std::string value)
{
  iyy = value;
}
void Inertial::SetIyz(std::string value)
{
  iyz = value;
}
void Inertial::SetIzz(std::string value)
{
  izz = value;
}
void Inertial::Read(QDomNode document)
{
  mass = document.firstChildElement("inertial")
             .firstChildElement("mass")
             .text()
             .toStdString();
  pose = document.firstChildElement("inertial")
             .firstChildElement("pose")
             .text()
             .toStdString();
  ixx = document.firstChildElement("inertial")
            .firstChildElement("inertia")
            .firstChildElement("ixx")
            .text()
            .toStdString();
  ixy = document.firstChildElement("inertial")
            .firstChildElement("inertia")
            .firstChildElement("ixy")
            .text()
            .toStdString();
  ixz = document.firstChildElement("inertial")
            .firstChildElement("inertia")
            .firstChildElement("ixz")
            .text()
            .toStdString();
  iyy = document.firstChildElement("inertial")
            .firstChildElement("inertia")
            .firstChildElement("iyy")
            .text()
            .toStdString();
  iyz = document.firstChildElement("inertial")
            .firstChildElement("inertia")
            .firstChildElement("iyz")
            .text()
            .toStdString();
  izz = document.firstChildElement("inertial")
            .firstChildElement("inertia")
            .firstChildElement("izz")
            .text()
            .toStdString();
}
void Inertial::Write(QXmlStreamWriter* xml)
{
  xml->writeStartElement("inertial");
  xml->writeTextElement("mass", mass.c_str());
  xml->writeTextElement("pose", pose.c_str());
  xml->writeStartElement("inertia");
  xml->writeTextElement("ixx", ixx.c_str());
  xml->writeTextElement("ixy", ixy.c_str());
  xml->writeTextElement("ixz", ixz.c_str());
  xml->writeTextElement("iyy", iyy.c_str());
  xml->writeTextElement("iyz", iyz.c_str());
  xml->writeTextElement("izz", izz.c_str());
  xml->writeEndElement();
  xml->writeEndElement();
}
void Inertial::print()
{
  qDebug() << "Inertial";
  qDebug() << "mass " << mass.c_str();
  qDebug() << "ixx " << ixx.c_str();
  qDebug() << "ixy " << ixy.c_str();
  qDebug() << "ixz " << ixz.c_str();
  qDebug() << "iyy " << iyy.c_str();
  qDebug() << "iyz " << iyz.c_str();
  qDebug() << "izz " << izz.c_str();
}
