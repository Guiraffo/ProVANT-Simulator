#include "plugin_da.h"

#include <QDebug>

PluginDA::PluginDA()
{
}

std::string PluginDA::GetName()
{
  return getName().toStdString();
}

void PluginDA::SetName(std::string value)
{
  setName(QString::fromStdString(value));
}

const QString& PluginDA::getName() const
{
  return _name;
}

void PluginDA::setName(const QString& value)
{
  _name = value;
}

std::string PluginDA::GetFilename()
{
  return getFilename().toStdString();
}

void PluginDA::SetFilename(std::string value)
{
  setFilename(QString::fromStdString(value));
}

const QString& PluginDA::getFilename() const
{
  return _filename;
}

void PluginDA::setFilename(const QString& value)
{
  _filename = value;
}

void PluginDA::Write(QXmlStreamWriter* xml)
{
  write(xml);
}

void PluginDA::print()
{
  qDebug() << "plugin";
  qDebug() << "name " << qUtf8Printable(_name);
  qDebug() << "filename " << qUtf8Printable(_filename);
  for (QMap<QString, QString>::const_iterator i = _parameterMap.cbegin();
       i != _parameterMap.cend(); ++i)
  {
    qDebug() << qUtf8Printable(i.key()) << ": " << qUtf8Printable(i.value())
             << " ";
  }
}

int PluginDA::getNumberOfParameters() const
{
  return _parameterMap.size();
}

const QString PluginDA::getParameter(const QString& name) const
{
  return _parameterMap.value(name);
}

void PluginDA::setParameter(const QString& name, const QString& value)
{
  _parameterMap[name] = value;
}

const QMap<QString, QString>& PluginDA::getParameters() const
{
  return _parameterMap;
}

void PluginDA::setParameters(const QMap<QString, QString>& params)
{
  _parameterMap.clear();
  _parameterMap = params;
}

bool PluginDA::read(const QDomElement& pluginElement)
{
  if (pluginElement.isNull())
    return false;

  if (pluginElement.hasAttribute("name"))
  {
    setName(pluginElement.attribute("name"));
  }
  else
  {
    QDomElement nameElement = pluginElement.firstChildElement("name");
    if (nameElement.isNull())
      return false;
    setName(nameElement.text());
  }

  if (pluginElement.hasAttribute("filename"))
  {
    setFilename(pluginElement.attribute("filename"));
  }
  else
  {
    QDomElement fileNameElement = pluginElement.firstChildElement("filename");
    if (fileNameElement.isNull())
      return false;
    setName(fileNameElement.text());
  }

  QDomElement childElement = pluginElement.firstChildElement();
  while (!childElement.isNull())
  {
    if (childElement.tagName() != "name" && childElement.tagName() != "filenam"
                                                                      "e")
    {
      setParameter(childElement.tagName(), childElement.text());
    }

    childElement = childElement.nextSiblingElement();
  }

  return true;
}

void PluginDA::write(QXmlStreamWriter* xml) const
{
  xml->writeStartElement("plugin");

  xml->writeAttribute("name", _name);
  xml->writeAttribute("filename", _filename);

  for (auto it = _parameterMap.cbegin(); it != _parameterMap.cend(); it++)
  {
    xml->writeTextElement(it.key(), it.value());
  }

  xml->writeEndElement();  // </plugin>
}
