#ifndef WORLDFILE_H
#define WORLDFILE_H

#include <QString>

#include <string>

#include "Items/gravity_da.h"
#include "Items/include_da.h"
#include "Items/physics_da.h"
#include "Items/modelplugin.h"
#include "Items/scene.h"
#include "Items/multipleincludes.h"
#include "Items/multipleplugins.h"
#include "Items/gui_da.h"

class WorldFile
{
public:
  WorldFile();
  WorldFile(const QString& filename);

  const QString& filename() const;
  void setFilename(const QString& filename);

  const QString& sdfVersion() const;
  void setSdfVersion(const QString& version);

  const GravityDA& gravity() const;
  void setGravity(const GravityDA& gravity);

  const PhysicsDA& physics() const;
  void setPhysics(const PhysicsDA& physics);

  const MultipleIncludes& getIncludeElements() const;
  void setIncludeElements(const MultipleIncludes& includes);

  const MultiplePlugins& getPlugins() const;
  void setPlugins(const MultiplePlugins& plugins);

  const GuiDA& gui() const;
  void setGui(const GuiDA& gui);

  bool read();
  bool write() const;

  bool Read() __attribute__((deprecated));
  void Write() __attribute__((deprecated));
  void print();

private:
  QDomDocument doc;
  GravityDA _gravity;
  PhysicsDA _physics;
  Scene sceneObj;
  GuiDA _gui;

  MultipleIncludes _includeElements;
  MultiplePlugins _pluginElements;

  QString _filename;
  QString _sdfVersion;

  bool readSdfVersion(const QDomElement& sdf);
};

#endif  // WORLDFILE_H
