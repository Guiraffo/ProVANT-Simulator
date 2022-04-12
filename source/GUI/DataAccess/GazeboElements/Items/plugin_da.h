#ifndef PLUGIN_DA_H
#define PLUGIN_DA_H

#include <QDomElement>
#include <QMap>
#include <QString>
#include <QXmlStreamWriter>

class PluginDA
{
public:
  PluginDA();
  std::string GetName() __attribute__((deprecated));
  void SetName(std::string) __attribute__((deprecated));

  const QString& getName() const;
  void setName(const QString& value);

  std::string GetFilename() __attribute__((deprecated));
  void SetFilename(std::string) __attribute__((deprecated));
  const QString& getFilename() const;
  void setFilename(const QString& value);

  void Write(QXmlStreamWriter*) __attribute__((deprecated));
  void print();

  int getNumberOfParameters() const;
  const QString getParameter(const QString& name) const;
  void setParameter(const QString& name, const QString& value);

  const QMap<QString, QString>& getParameters() const;
  void setParameters(const QMap<QString, QString>& params);

  bool read(const QDomElement& pluginElement);
  void write(QXmlStreamWriter* xml) const;

private:
  QString _name;
  QString _filename;
  QMap<QString, QString> _parameterMap;
};

#endif  // PLUGIN_DA_H
