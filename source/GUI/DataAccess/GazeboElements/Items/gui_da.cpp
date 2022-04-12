#include "gui_da.h"

GuiDA::GuiDA()
{
}

bool GuiDA::read(const QDomElement& world)
{
  if (world.isNull())
    return false;

  QDomElement guiElement = world.firstChildElement("gui");
  if (guiElement.isNull())
    return false;

  // Read the fullscreen element
  QDomElement fullScreenElement = guiElement.firstChildElement("fullscreen");
  if (fullScreenElement.isNull())
  {
    setFullscreen("false");
  }
  else
  {
    setFullscreen(fullScreenElement.text());
  }

  // Read the list of plugin elements
  if (!_plugins.read(guiElement))
    return false;

  return true;
}

void GuiDA::write(QXmlStreamWriter* xml) const
{
  if (!isEmpty())
  {
    xml->writeStartElement("gui");

    if (_fullscreen.isEmpty())
    {
      xml->writeTextElement("fullscreen", _fullscreen);
    }

    _plugins.write(xml);

    xml->writeEndElement();  // </gui>
  }
}

bool GuiDA::isEmpty() const
{
  return _fullscreen.isEmpty() && _plugins.isEmpty();
}

const QString& GuiDA::fullscreen() const
{
  return _fullscreen;
}

void GuiDA::setFullscreen(const QString& fullscren)
{
  _fullscreen = fullscren;
}

const MultiplePlugins& GuiDA::getPlugins() const
{
  return _plugins;
}

void GuiDA::setPlugins(const MultiplePlugins& plugins)
{
  _plugins = plugins;
}
