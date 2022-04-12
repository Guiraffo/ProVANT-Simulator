#ifndef INCLUDE_DA_H
#define INCLUDE_DA_H

#include <QDomElement>
#include <QString>
#include <QXmlStreamWriter>

//! @todo Remove along with deprecated methods
#include <string>

class IncludeDA
{
public:
  IncludeDA();

  std::string GetUri() __attribute__((deprecated));
  void SetUri(std::string) __attribute__((deprecated));
  const QString& getURI() const;
  void setURI(const QString& value);

  std::string GetName() __attribute__((deprecated));
  void SetName(std::string) __attribute__((deprecated));
  const QString& getName() const;
  void setName(const QString& value);

  std::string GetIsStatic() __attribute__((deprecated));
  void SetIsStatic(std::string) __attribute__((deprecated));
  const QString& isStatic() const;
  void setStatic(const QString& value);

  std::string GetPose() __attribute__((deprecated));
  void SetPose(std::string) __attribute__((deprecated));
  const QString& getPose() const;
  void setPose(const QString& value);

  bool read(const QDomElement& pluginElement);
  void write(QXmlStreamWriter* xml) const;

  void Write(QXmlStreamWriter);
  void print();

private:
  QString _uri;
  QString _name;
  QString _isStatic;
  QString _pose;
};

#endif  // INCLUDE_DA_H
