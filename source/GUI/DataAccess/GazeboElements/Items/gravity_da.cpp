#include "gravity_da.h"

#include <QDebug>

GravityDA::GravityDA()
{
}

void GravityDA::Read(QDomNode document)
{
  read(document.toElement());
}

void GravityDA::Write(QXmlStreamWriter xml)
{
  write(&xml);
}

std::string GravityDA::getGravity()
{
  return gravity().toStdString();
}

const QString& GravityDA::gravity() const
{
  return _gravity;
}

void GravityDA::setGravity(std::string value)
{
  setGravity(QString::fromStdString(value));
}

void GravityDA::setGravity(const QString& value)
{
  _gravity = value;
}

bool GravityDA::read(const QDomElement& worldElement)
{
  QDomElement gravityElement = worldElement.firstChildElement(gravityTag);
  if (gravityElement.isNull())
    return false;

  _gravity = gravityElement.text();
  return true;
}

void GravityDA::write(QXmlStreamWriter* xml) const
{
  xml->writeTextElement(gravityTag, _gravity);
}

void GravityDA::print()
{
  qDebug() << "Gravity " << qUtf8Printable(_gravity);
}
