#ifndef MULTIPLEPLUGINS_H
#define MULTIPLEPLUGINS_H

#include <QDomElement>
#include <QList>
#include <QXmlStreamWriter>

#include <vector>

#include "plugin_da.h"

class MultiplePlugins
{
public:
  MultiplePlugins();

  void NewPlugin(PluginDA) __attribute__((deprecated));
  void Read(QDomNode) __attribute__((deprecated));
  void Clear() __attribute__((deprecated));
  void print() const;

  bool read(const QDomElement& element);
  void write(QXmlStreamWriter* xml) const;

  bool isEmpty() const;
  int numElements() const;

  const QList<PluginDA> getPlugins() const;
  void addPlugin(const PluginDA& plugin);
  void setPlugins(const QList<PluginDA>& plugins);
  void setPlugins(const std::vector<PluginDA>& pluginVector)
      __attribute__((deprecated));
  void clearPlugins();
  std::vector<PluginDA> getPluginsAsVector() const __attribute__((deprecated));

private:
  QList<PluginDA> _plugins;
};

#endif  // MULTIPLEPLUGINS_H
