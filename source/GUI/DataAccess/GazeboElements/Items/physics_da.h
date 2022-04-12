#ifndef PHYSICS_DA_H
#define PHYSICS_DA_H

#include <QDomElement>
#include <QXmlStreamWriter>
#include <QString>

#include <string>

class PhysicsDA
{
public:
  PhysicsDA();

  std::string GetType() __attribute__((deprecated));
  void SetType(std::string) __attribute__((deprecated));
  const QString& getType() const;
  void setType(const QString& _type);

  std::string GetStep() __attribute__((deprecated));
  void SetStep(std::string) __attribute__((deprecated));
  const QString& getStep() const;
  void setStep(const QString& value);

  std::string GetRealTimeFactor() __attribute__((deprecated));
  void SetRealTimeFactor(std::string) __attribute__((deprecated));
  const QString& getRealTimeFactor() const;
  void setRealTimeFactor(const QString& value);

  std::string GetRealTimeUpdaterate() __attribute__((deprecated));
  void SetRealTimeUpdaterate(std::string) __attribute__((deprecated));
  const QString getRealTimeUpdateRate() const;
  void setRealTimeUpdateRate(const QString& value);

  bool read(const QDomElement& worldElement);
  void write(QXmlStreamWriter* xml) const;

  void Write(QXmlStreamWriter) __attribute__((deprecated));
  void Read(QDomNode) __attribute__((deprecated));
  void print();

private:
  QString _type;
  QString _step;
  QString _realTimeFactor;
  QString _realTimeUpdateRate;
};

#endif  // PHYSICS_DA_H
