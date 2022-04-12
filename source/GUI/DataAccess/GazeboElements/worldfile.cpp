#include "worldfile.h"

#include <QFile>
#include <QFileInfo>
#include <QXmlStreamWriter>

WorldFile::WorldFile()
{
}

WorldFile::WorldFile(const QString& filename) : _filename(filename)
{
}

const QString& WorldFile::filename() const
{
  return _filename;
}

void WorldFile::setFilename(const QString& filename)
{
  _filename = filename;
}

const QString& WorldFile::sdfVersion() const
{
  return _sdfVersion;
}

void WorldFile::setSdfVersion(const QString& version)
{
  _sdfVersion = version;
}

const GravityDA& WorldFile::gravity() const
{
  return _gravity;
}

void WorldFile::setGravity(const GravityDA& gravity)
{
  _gravity = gravity;
}

const PhysicsDA& WorldFile::physics() const
{
  return _physics;
}

void WorldFile::setPhysics(const PhysicsDA& physics)
{
  _physics = physics;
}

const MultipleIncludes& WorldFile::getIncludeElements() const
{
  return _includeElements;
}

void WorldFile::setIncludeElements(const MultipleIncludes& includes)
{
  _includeElements = includes;
}

const MultiplePlugins& WorldFile::getPlugins() const
{
  return _pluginElements;
}

void WorldFile::setPlugins(const MultiplePlugins& plugins)
{
  _pluginElements = plugins;
}

const GuiDA& WorldFile::gui() const
{
  return _gui;
}

void WorldFile::setGui(const GuiDA& gui)
{
  _gui = gui;
}

bool WorldFile::read()
{
  // Verify if the file exists
  QFileInfo finfo(_filename);
  if (!finfo.exists())
    return false;

  QFile _file;

  _file.setFileName(_filename);
  if (!_file.open(QIODevice::ReadOnly | QIODevice::Text))
    return false;

  QString erro;
  int line, column;
  if (!doc.setContent(&_file, &erro, &line, &column))
  {
    _file.close();
    return false;
  }
  _file.close();

  QDomElement sdf = doc.firstChildElement("sdf");
  if (sdf.isNull())
    return false;

  if (!readSdfVersion(sdf))
    return false;

  QDomElement world = sdf.firstChildElement("world");
  if (world.isNull())
    return false;

  if (!_gravity.read(world))
    return false;
  if (!_physics.read(world))
    return false;
  if (!_includeElements.read(world))
    return false;
  if (!_pluginElements.read(world))
    return false;
  if (!sceneObj.read(world))
    return false;
  // Note. The GUI element is optional
  _gui.read(world);

  return true;
}

bool WorldFile::write() const
{
  QFile file(_filename);
  if (!file.open(QIODevice::ReadWrite | QIODevice::Truncate))
    return false;
  QXmlStreamWriter xml;
  xml.setAutoFormatting(true);
  xml.setDevice(&file);
  xml.setAutoFormattingIndent(2);

  xml.writeStartDocument();
  xml.writeComment("\nThis file is part of the ProVANT simulator project.\n"
                   "Licensed under the terms of the MIT open source "
                   "license. More details at\n"
                   "https://github.com/Guiraffo/ProVANT-Simulator/"
                   "blob/master/LICENSE.md\n");

  xml.writeStartElement("sdf");
  xml.writeAttribute("version", _sdfVersion);

  xml.writeStartElement("world");
  // Set the world name
  QFileInfo info(_filename);
  QString worldName = info.fileName().section(".", 0, 0);
  xml.writeAttribute("name", worldName);

  _gui.write(&xml);
  _gravity.write(&xml);
  _physics.write(&xml);
  _pluginElements.write(&xml);
  _includeElements.write(&xml);

  xml.writeStartElement("scene");
  xml.writeStartElement("sky");
  xml.writeTextElement("time", "18");
  xml.writeStartElement("clouds");
  xml.writeTextElement("speed", "0");
  xml.writeEndElement();  // </clouds>
  xml.writeEndElement();  // </sky>
  xml.writeEndElement();  // </scene>

  xml.writeEndElement();  // </world>

  xml.writeEndElement();  // </sdf>
  xml.writeEndDocument();

  file.close();
  return true;
}

bool WorldFile::Read()
{
  return read();
}

void WorldFile::Write()
{
  write();
}

void WorldFile::print()
{
  _gravity.print();
  _physics.print();
  _includeElements.print();
  _pluginElements.print();
}

bool WorldFile::readSdfVersion(const QDomElement& sdf)
{
  if (sdf.hasAttribute("version"))
  {
    _sdfVersion = sdf.attribute("version");
    return true;
  }
  return false;
}
