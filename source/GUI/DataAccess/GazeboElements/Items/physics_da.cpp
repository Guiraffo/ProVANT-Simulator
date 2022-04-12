#include "physics_da.h"

#include <QDebug>

PhysicsDA::PhysicsDA()
{
}

std::string PhysicsDA::GetType()
{
  return getType().toStdString();
}

void PhysicsDA::SetType(std::string value)
{
  setType(QString::fromStdString(value));
}

const QString& PhysicsDA::getType() const
{
  return _type;
}

void PhysicsDA::setType(const QString& type)
{
  this->_type = type;
}

std::string PhysicsDA::GetStep()
{
  return getStep().toStdString();
}

void PhysicsDA::SetStep(std::string value)
{
  setStep(QString::fromStdString(value));
}

const QString& PhysicsDA::getStep() const
{
  return _step;
}

void PhysicsDA::setStep(const QString& value)
{
  _step = value;
}

std::string PhysicsDA::GetRealTimeFactor()
{
  return getRealTimeFactor().toStdString();
}

void PhysicsDA::SetRealTimeFactor(std::string value)
{
  setRealTimeFactor(QString::fromStdString(value));
}

const QString& PhysicsDA::getRealTimeFactor() const
{
  return _realTimeFactor;
}

void PhysicsDA::setRealTimeFactor(const QString& value)
{
  _realTimeFactor = value;
}

std::string PhysicsDA::GetRealTimeUpdaterate()
{
  return getRealTimeUpdateRate().toStdString();
}

void PhysicsDA::SetRealTimeUpdaterate(std::string value)
{
  setRealTimeUpdateRate(QString::fromStdString(value));
}

const QString PhysicsDA::getRealTimeUpdateRate() const
{
  if (_realTimeUpdateRate.isEmpty())
    return QString("0");
  return _realTimeUpdateRate;
}

void PhysicsDA::setRealTimeUpdateRate(const QString& value)
{
  _realTimeUpdateRate = value;
}

bool PhysicsDA::read(const QDomElement& worldElement)
{
  if (worldElement.isNull())
    return false;

  QDomElement physicsElement = worldElement.firstChildElement("physics");
  if (physicsElement.isNull())
    return false;

  if (physicsElement.hasAttribute("type"))
    _type = physicsElement.attribute("type");
  else
    return false;

  QDomElement stepSizeElement = physicsElement.firstChildElement("max_step_"
                                                                 "size");
  if (stepSizeElement.isNull())
    return false;
  _step = stepSizeElement.text();

  QDomElement realTimeFactorElement = physicsElement.firstChildElement("real_"
                                                                       "time_"
                                                                       "facto"
                                                                       "r");
  if (realTimeFactorElement.isNull())
    return false;
  _realTimeFactor = realTimeFactorElement.text();

  QDomElement realTimeUpdateRateElement = physicsElement.firstChildElement("rea"
                                                                           "l_"
                                                                           "tim"
                                                                           "e_"
                                                                           "upd"
                                                                           "ate"
                                                                           "_ra"
                                                                           "t"
                                                                           "e");
  if (realTimeFactorElement.isNull())
    return false;
  _realTimeUpdateRate = realTimeUpdateRateElement.text();

  return true;
}

void PhysicsDA::write(QXmlStreamWriter* xml) const
{
  xml->writeStartElement("physics");
  xml->writeAttribute("type", _type);
  xml->writeTextElement("max_step_size", _step);
  xml->writeTextElement("real_time_factor", _realTimeFactor);
  if (!_realTimeUpdateRate.isEmpty())
    xml->writeTextElement("real_time_update_rate", _realTimeUpdateRate);

  xml->writeEndElement();  // </physics>
}

void PhysicsDA::Write(QXmlStreamWriter xml)
{
  write(&xml);
}

void PhysicsDA::Read(QDomNode document)
{
  read(document.toElement());
}

void PhysicsDA::print()
{
  qDebug() << "Physics";
  qDebug() << "type " << qUtf8Printable(_type);
  qDebug() << "step " << qUtf8Printable(_step);
  qDebug() << "real_time_factor " << qUtf8Printable(_realTimeFactor);
  qDebug() << "real_time_update_rate " << qUtf8Printable(_realTimeUpdateRate);
}
