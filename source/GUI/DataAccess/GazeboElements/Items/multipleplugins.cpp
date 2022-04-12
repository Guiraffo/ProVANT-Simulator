#include "multipleplugins.h"

MultiplePlugins::MultiplePlugins()
{
}

void MultiplePlugins::NewPlugin(PluginDA item)
{
  addPlugin(item);
}

void MultiplePlugins::Read(QDomNode document)
{
  Clear();

  // lendo primeiro include
  document = document.firstChildElement("plugin");
  // lendo demais include se houver
  while (true)
  {
    PluginDA item;
    if (!document.isNull())
    {
      item.setName(document.toElement().attribute("name"));
      item.setFilename(document.toElement().attribute("filename"));

      QDomNodeList list = document.childNodes();
      for (int i = 0; i < list.count(); i++)
      {
        item.setParameter(list.at(i).toElement().tagName(),
                          list.at(i).toElement().text());
      }
      NewPlugin(item);
      document = document.nextSiblingElement("plugin");
    }
    else
      break;
  }
}

void MultiplePlugins::Clear()
{
  clearPlugins();
}

void MultiplePlugins::print() const
{
  foreach (PluginDA var, _plugins)
  {
    var.print();
  }
}

bool MultiplePlugins::read(const QDomElement& element)
{
  if (element.isNull())
    return false;

  QDomElement pluginElement = element.firstChildElement("plugin");
  while (!pluginElement.isNull())
  {
    PluginDA newPlugin;
    if (!newPlugin.read(pluginElement))
    {
      _plugins.clear();
      return false;
    }
    addPlugin(newPlugin);

    pluginElement = pluginElement.nextSiblingElement("plugin");
  }

  return true;
}

void MultiplePlugins::write(QXmlStreamWriter* xml) const
{
  for (auto it = _plugins.cbegin(); it != _plugins.cend(); it++)
  {
    it->write(xml);
  }
}

bool MultiplePlugins::isEmpty() const
{
  return _plugins.isEmpty();
}

int MultiplePlugins::numElements() const
{
  return _plugins.size();
}

const QList<PluginDA> MultiplePlugins::getPlugins() const
{
  return _plugins;
}

void MultiplePlugins::addPlugin(const PluginDA& plugin)
{
  _plugins.push_back(plugin);
}

void MultiplePlugins::setPlugins(const QList<PluginDA>& plugins)
{
  _plugins = plugins;
}

void MultiplePlugins::setPlugins(const std::vector<PluginDA>& pluginVector)
{
  _plugins.clear();
  for (std::size_t i = 0; i < pluginVector.size(); i++)
  {
    _plugins.push_back(pluginVector.at(i));
  }
}

void MultiplePlugins::clearPlugins()
{
  _plugins.clear();
}

std::vector<PluginDA> MultiplePlugins::getPluginsAsVector() const
{
  std::vector<PluginDA> pluginVector;
  pluginVector.reserve(_plugins.size());

  for (QList<PluginDA>::const_iterator it = _plugins.cbegin();
       it != _plugins.end(); it++)
  {
    pluginVector.push_back(*it);
  }

  return pluginVector;
}
