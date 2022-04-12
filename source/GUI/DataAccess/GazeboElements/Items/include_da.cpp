#include "include_da.h"

#include <QDebug>

IncludeDA::IncludeDA()
{
}

std::string IncludeDA::GetUri()
{
  return getURI().toStdString();
}

void IncludeDA::SetUri(std::string value)
{
  setURI(QString::fromStdString(value));
}

const QString& IncludeDA::getURI() const
{
  return _uri;
}

void IncludeDA::setURI(const QString& value)
{
  _uri = value;
}

std::string IncludeDA::GetName()
{
  return getName().toStdString();
}

void IncludeDA::SetName(std::string value)
{
  setName(QString::fromStdString(value));
}

const QString& IncludeDA::getName() const
{
  return _name;
}

void IncludeDA::setName(const QString& value)
{
  _name = value;
}

std::string IncludeDA::GetIsStatic()
{
  return isStatic().toStdString();
}

void IncludeDA::SetIsStatic(std::string value)
{
  setStatic(QString::fromStdString(value));
}

const QString& IncludeDA::isStatic() const
{
  return _isStatic;
}

void IncludeDA::setStatic(const QString& value)
{
  _isStatic = value;
}

std::string IncludeDA::GetPose()
{
  return getPose().toStdString();
}

void IncludeDA::SetPose(std::string value)
{
  setPose(QString::fromStdString(value));
}

const QString& IncludeDA::getPose() const
{
  return _pose;
}

void IncludeDA::setPose(const QString& value)
{
  _pose = value;
}

bool IncludeDA::read(const QDomElement& includeElement)
{
  if (includeElement.isNull())
    return false;

  QDomElement uriElement = includeElement.firstChildElement("uri");
  if (uriElement.isNull())
    return false;
  setURI(uriElement.text());

  QDomElement nameElement = includeElement.firstChildElement("name");
  if (!nameElement.isNull())
    setName(nameElement.text());

  QDomElement staticElement = includeElement.firstChildElement("static");
  if (staticElement.isNull())
  {
    setStatic("0");
  }
  else
  {
    setStatic(staticElement.text());
  }

  QDomElement poseElement = includeElement.firstChildElement("pose");
  if (poseElement.isNull())
  {
    setPose("0 0 0 0 0 0");
  }
  else
  {
    setPose(poseElement.text());
  }

  return true;
}

void IncludeDA::write(QXmlStreamWriter* xml) const
{
  xml->writeStartElement("include");
  if (!_uri.isEmpty())
    xml->writeTextElement("uri", _uri);
  if (!_name.isEmpty())
    xml->writeTextElement("name", _name);
  if (!_isStatic.isEmpty())
    xml->writeTextElement("static", _isStatic);
  if (!_pose.isEmpty())
    xml->writeTextElement("pose", _pose);
  xml->writeEndElement();  // </include>
}

void IncludeDA::Write(QXmlStreamWriter xml)
{
  write(&xml);
}

void IncludeDA::print()
{
  qDebug() << "include"
           << " Uri: " << qUtf8Printable(_uri)
           << " Name: " << qUtf8Printable(_name)
           << " Static: " << qUtf8Printable(_isStatic)
           << " Pose: " << qUtf8Printable(_pose);
}
