#include "world.h"
#include "QMessageBox"

#include <QDebug>

#include "Utils/appsettings.h"
#include "treeitens.h"

World::World()
{
}

void World::loadWorld(std::string first_filename, QTreeWidget* root)
{
  loadWorld(QString::fromStdString(first_filename), root);
}

void World::loadWorld(const QString& filename, QTreeWidget* root)
{
  _world = WorldFile(filename);
  if (_world.read())
    toTreeWidget(root);
}

bool World::write(QTreeWidget* root)
{
  WorldFile newMundo(_world.filename());
  newMundo.setSdfVersion(_world.sdfVersion());

  for (int i = 0; i < root->topLevelItemCount(); i++)
  {
    QTreeWidgetItem* item = root->topLevelItem(i);

    if (item->text(0).toLower() == "physics")
    {
      PhysicsDA _physics;
      if (!parsePhysics(item, &_physics))
        return false;
      newMundo.setPhysics(_physics);
    }
    else if (item->text(0).toLower() == "gravity")
    {
      GravityDA gravity;
      if (!parseGravity(item, &gravity))
        return false;
      newMundo.setGravity(gravity);
    }
    else if (item->text(0).toLower() == "gui")
    {
      GuiDA _gui;
      if (!parseGui(item, &_gui))
        return false;
      newMundo.setGui(_gui);
    }
    else if (item->text(0).toLower() == "plugin")
    {
      PluginDA newPlugin;
      if (!parsePlugin(item, &newPlugin))
        return false;

      MultiplePlugins plugins = newMundo.getPlugins();
      plugins.addPlugin(newPlugin);
      newMundo.setPlugins(plugins);
    }
    else if (item->text(0).toLower() == "include")
    {
      IncludeDA include;
      if (!parseInclude(item, &include))
        return false;

      MultipleIncludes includes = newMundo.getIncludeElements();
      includes.addIncludeElement(include);
      newMundo.setIncludeElements(includes);
    }
    else
    {
      QMessageBox::critical(nullptr, QObject::tr("Error"),
                            QObject::tr("Error while saving the world file. A "
                                        "tag "
                                        "with an invalid name was found. Plase "
                                        "check the tag values and try again."));
      qCritical() << "Error while parsing the World elements. An element"
                     " with the tag name "
                  << qUtf8Printable(item->text(0))
                  << "was found. But this tag is not recognized by the "
                     "parser.";
      return false;
    }
  }
  newMundo.setFilename(_world.filename());
  _world = newMundo;
  _world.write();

  return true;
}

void World::toTreeWidget(QTreeWidget* root)
{
  // Clear tree before inserting new elements
  root->clear();

  if (!_world.gui().isEmpty())
  {
    QTreeWidgetItem* guiItem = addRoot("GUI", "", root);
    GuiDA gui = _world.gui();

    QString fullscreenVal = gui.fullscreen();
    if (fullscreenVal.isEmpty())
    {
      fullscreenVal = "false";
    }
    addChild(guiItem, "Fullscreen", fullscreenVal);

    for (auto it = gui.getPlugins().getPlugins().cbegin();
         it != gui.getPlugins().getPlugins().cend(); it++)
    {
      QTreeWidgetItem* guiPlugin = addChild(guiItem, "Plugin", "", false);
      readPlugin(guiPlugin, *it);
    }
  }

  // Add elements related to the world's gravity
  QTreeWidgetItem* gravityElement = addRoot("Gravity", "", root);

  QString gravityStr = _world.gravity().gravity();
  QStringList gravityElements = gravityStr.split(" ");

  addChild(gravityElement, "X", gravityElements.at(0), true);
  addChild(gravityElement, "Y", gravityElements.at(1), true);
  addChild(gravityElement, "Z", gravityElements.at(2), true);

  // Add physics elements
  QTreeWidgetItem* physicsElement = addRoot("Physics", "", root);

  addChild(physicsElement, "Type", _world.physics().getType(), true);
  addChild(physicsElement, "Step time", _world.physics().getStep(), true);
  addChild(physicsElement, "Real time factor",
           _world.physics().getRealTimeFactor());
  addChild(physicsElement, "Real time update rate",
           _world.physics().getRealTimeUpdateRate());

  // Add the plugin elements
  readPlugins(root, _world.getPlugins());

  // Add the include elements
  QList<IncludeDA> includes = _world.getIncludeElements().getIncludes();
  for (auto it = includes.cbegin(); it != includes.cend(); it++)
  {
    QTreeWidgetItem* includeElement = addRoot("Include", "", root, false);

    addChild(includeElement, "Name", it->getName(), true);
    QTreeWidgetItem* poseItem = addChild(includeElement, "Pose", "", false);
    splitPose(it->getPose(), poseItem);

    addChild(includeElement, "isStatic", it->isStatic());
    addChild(includeElement, "URI", it->getURI());
  }
}

void World::readPlugin(QTreeWidgetItem* root, const PluginDA& plugin)
{
  addChild(root, "Name", plugin.getName(), true);
  addChild(root, "Filename", plugin.getFilename(), true);

  QMap<QString, QString> paramMap = plugin.getParameters();
  for (auto i = paramMap.cbegin(); i != paramMap.cend(); ++i)
  {
    addChild(root, i.key(), i.value(), true);
  }
}

void World::readPlugins(QTreeWidget* root, const MultiplePlugins& plugins)
{
  for (auto it = plugins.getPlugins().cbegin();
       it != plugins.getPlugins().cend(); it++)
  {
    QTreeWidgetItem* pluginElement = addRoot("Plugin", "", root, false);
    readPlugin(pluginElement, *it);
  }
}

bool World::parsePhysics(const QTreeWidgetItem* item, PhysicsDA* physics)
{
  if (item->childCount() < 3)
  {
    QMessageBox::critical(nullptr, QObject::tr("Error"),
                          QObject::tr("Error while parsing the physics "
                                      "element. The physics elements must "
                                      "have four child elements. Plase check "
                                      "the values and try again."));
    qCritical() << "Error while parsing the Physics element. An "
                   "object with "
                << item->childCount()
                << " child itens was found. But 4 were expected.";
    return false;
  }

  bool typeFound = false;
  bool stepTimeFound = false;
  bool realTimeFactorFound = false;
  bool realTimeUpdateRateFound = false;

  for (int i = 0; i < item->childCount(); i++)
  {
    QTreeWidgetItem* childItem = item->child(i);

    if (childItem->text(0).toLower() == "type")
    {
      QString value = childItem->text(1);

      if (value != "ode" && value != "bullet" && value != "simbody" &&
          value != "dart")
      {
        QMessageBox messageBox;
        messageBox.critical(nullptr, QObject::tr("Error"),
                            QObject::tr("Invalid physics type. "
                                        "Plase check this parameter "
                                        "value and try again."));
        qCritical() << "Error while parsing the Physics->Type "
                       "element. The value "
                    << qUtf8Printable(value)
                    << " was found. But the valid optins are ("
                       "ode, bullet, simbody, dart).";

        return false;
      }

      physics->setType(childItem->text(1));
      typeFound = true;
    }
    else if (childItem->text(0).toLower() == "step time")
    {
      bool ok;
      double value = childItem->text(1).toDouble(&ok);

      if (!ok || value < 0)
      {
        QMessageBox::critical(nullptr, QObject::tr("Error"),
                              QObject::tr("Invalid step time. Plase check"
                                          " the value and try again."));
        qCritical() << "Error while parsing the Physics -> "
                       "Step time element. The value "
                    << qUtf8Printable(childItem->text(1))
                    << " was found, but a positive numeric "
                       " value was expected.";
        return false;
      }

      physics->setStep(childItem->text(1));
      stepTimeFound = true;
    }
    else if (childItem->text(0).toLower() == "real time factor")
    {
      // Check the real time factor value
      bool ok;
      double value = childItem->text(1).toDouble(&ok);

      if (!ok || value < 0)
      {
        QMessageBox::critical(nullptr, QObject::tr("Error"),
                              QObject::tr("Invalid real time factor. "
                                          "Plase check the value and try "
                                          "again."));
        qCritical() << "Error while parsing the Physics -> Real"
                       " time factor value. The value "
                    << qUtf8Printable(childItem->text(1))
                    << " was found. But a positive numeric "
                       " value was expected.";
        return false;
      }

      physics->setRealTimeFactor(childItem->text(1));
      realTimeFactorFound = true;
    }
    else if (childItem->text(0).toLower() == "real time update rate")
    {
      bool ok;
      double value = childItem->text(1).toDouble(&ok);

      if (!ok || value < 0)
      {
        QMessageBox::critical(nullptr, QObject::tr("Error"),
                              QObject::tr("Invalid real time update rate."
                                          "Plase check the value and try "
                                          "again."));
        qCritical() << "Error while parsing the Physics -> Real"
                       " time update rate element. The value "
                    << qUtf8Printable(childItem->text(1))
                    << " was found, but a positive numeric "
                       "value was expected.";
        return false;
      }

      physics->setRealTimeUpdateRate(childItem->text(1));
      realTimeUpdateRateFound = true;
    }
    else
    {
      QMessageBox::critical(nullptr, QObject::tr("Error"),
                            QObject::tr("Unrecognized child element under "
                                        "the physics element. "
                                        "Plase check the values and try "
                                        "again."));
      qCritical() << "Error while parsing the Physics element. "
                     "A child element with the name \""
                  << qUtf8Printable(childItem->text(0)) << "\" and value \""
                  << qUtf8Printable(childItem->text(1))
                  << "\" was found. But this tag is not "
                     "recognized by the parser.";
      return false;
    }
  }

  if (!typeFound || !stepTimeFound || !realTimeFactorFound ||
      !realTimeUpdateRateFound)
  {
    QMessageBox::critical(nullptr, QObject::tr("Error"),
                          QObject::tr("The physics object must have a Type, "
                                      "Step time, Real time factor, and "
                                      "Real time update rate elements."
                                      "Plase check these values and try "
                                      "again."));
    qCritical() << "Error while parsing the Physics element. "
                   "The physics element must have a Type, Step "
                   "time, Real time factor, and Real time update "
                   "rate elements. But at least one of those was "
                   "not found by the parser.";
    return false;
  }

  return true;
}

bool World::parseGravity(const QTreeWidgetItem* item, GravityDA* gravity)
{
  if (item->childCount() != 3)
  {
    QMessageBox::critical(nullptr, QObject::tr("Error"),
                          QObject::tr("Invalid gravity element. The gravity "
                                      "element must have tree child itens."
                                      "Plase check the element values and try"
                                      "again."));
    qCritical() << "Error while parsing the Gravity element. An "
                   "element with "
                << item->childCount()
                << " was found. But the Gravity element must have "
                   "three child itens.";
    return false;
  }

  QString x, y, z;
  for (int i = 0; i < 3; i++)
  {
    QTreeWidgetItem* childItem = item->child(i);

    bool ok;
    childItem->text(1).toDouble(&ok);
    if (!ok)
    {
      QMessageBox::critical(nullptr, QObject::tr("Error"),
                            QObject::tr("Error while parsing a Gravity "
                                        "value. The gravity values must "
                                        "be numeric. Plase check the "
                                        "values and try again."));
      qCritical() << "Error while parsing the Gravity -> "
                  << qUtf8Printable(childItem->text(0))
                  << " element. The value "
                  << qUtf8Printable(childItem->text(1))
                  << " was found. But a numeric value was "
                     "expceted.";
      return false;
    }

    if (childItem->text(0).toLower() == "x")
    {
      x = childItem->text(1);
    }
    else if (childItem->text(0).toLower() == "y")
    {
      y = childItem->text(1);
    }
    else if (childItem->text(0).toLower() == "z")
    {
      z = childItem->text(1);
    }
    else
    {
      QMessageBox::critical(nullptr, QObject::tr("Error"),
                            QObject::tr("Invalid child element under the "
                                        "gravity element. The gravity "
                                        "element must have X, Y and Z "
                                        "values. Plase check these values "
                                        "and try again."));
      qCritical() << "Error while parsing the Gravity element. "
                     "A tag with name \""
                  << qUtf8Printable(childItem->text(0)) << "\" and value \""
                  << qUtf8Printable(childItem->text(1))
                  << "\" was found. But this tag is not "
                     "recognized by the parser.";
      return false;
    }
  }

  if (x.isEmpty() || y.isEmpty() || z.isEmpty())
  {
    QMessageBox::critical(nullptr, QObject::tr("Error"),
                          QObject::tr("Invalid child element under the "
                                      "gravity element. The gravity "
                                      "element must have X, Y and Z "
                                      "values. Plase check these values "
                                      "and try again."));
    qCritical() << "Error while parsing the Gravity element. "
                   "The Gravity element must have a X, Y and Z "
                   "child elements but at least one of these was "
                   "not found.";
    return false;
  }

  QString gravStr = QString("%1 %2 %3").arg(x).arg(y).arg(z);

  gravity->setGravity(gravStr);

  return true;
}

bool World::parsePlugin(const QTreeWidgetItem* item, PluginDA* plugin)
{
  if (item->childCount() < 2)
  {
    QMessageBox::critical(nullptr, QObject::tr("Error"),
                          QObject::tr("Eror while trying to parse "
                                      "a plugin element. Each plugin"
                                      "element must have at least "
                                      "a name and filename tags."
                                      "Please verify the world "
                                      "plugins and try again."));
    qCritical() << "Error while parsing a plugin element. A plugin "
                   "with "
                << item->childCount()
                << " was found. But each plugin element must have "
                   "at least two child elements.";
    return false;
  }

  bool nameFound = false;
  bool filenameFound = false;

  for (int i = 0; i < item->childCount(); i++)
  {
    QTreeWidgetItem* childItem = item->child(i);
    if (childItem->text(0).toLower() == "name")
    {
      plugin->setName(childItem->text(1));

      nameFound = true;
    }
    else if (childItem->text(0).toLower() == "filename")
    {
      plugin->setFilename(childItem->text(1));

      filenameFound = true;
    }
    else
    {
      // If it is not a name nor a filename element, it must be
      // a parameter
      plugin->setParameter(childItem->text(0), childItem->text(1));
    }
  }

  if (!nameFound || !filenameFound)
  {
    QMessageBox::critical(nullptr, QObject::tr("Error"),
                          QObject::tr("Error while parsing a "
                                      "plugin element. Every "
                                      "plugin must have a name and"
                                      " a filename tag. Plase "
                                      "check the world plugins and"
                                      " try again."));
    qCritical() << "Error while parsing a plugin element. Each "
                   "plugin must have at least name and filename "
                   "child elements, but at leat one of those was "
                   "not found.";
    return false;
  }

  return true;
}

bool World::parseInclude(const QTreeWidgetItem* item, IncludeDA* include)
{
  if (item->childCount() < 4)
  {
    QMessageBox::critical(nullptr, QObject::tr("Error"),
                          QObject::tr("Error while parsing a Include "
                                      "element. Each include element must "
                                      "have 4 child elements. Plase check "
                                      "this tag value and try again."));
    qCritical() << "Error while parsing an Include element. An "
                   "element with "
                << item->childCount()
                << " was found. But an element with 4 children was"
                   "expected.";
    return false;
  }

  bool nameFound = false;
  bool staticFound = false;
  bool poseFound = false;
  bool uriFound = false;

  for (int i = 0; i < item->childCount(); i++)
  {
    QTreeWidgetItem* childItem = item->child(i);

    if (childItem->text(0).toLower() == "name")
    {
      include->setName(childItem->text(1));
      nameFound = true;
    }
    else if (childItem->text(0).toLower() == "pose")
    {
      if (childItem->childCount() != 6)
      {
        QMessageBox::critical(nullptr, QObject::tr("Error"),
                              QObject::tr("Invalid pose element found. The "
                                          "Pose "
                                          "element must have six numeric "
                                          "elements"
                                          ". Plase check the pose values and "
                                          "try "
                                          "again."));
        qCritical() << "Error while parsing an Include -> Pose "
                       "element. A Pose with "
                    << childItem->childCount()
                    << " child itens was found. But a pose "
                       "element must have 6 child elements.";
        return false;
      }

      QString x, y, z, roll, pitch, yaw;

      for (int i = 0; i < childItem->childCount(); i++)
      {
        QTreeWidgetItem* poseItem = childItem->child(i);

        bool ok;
        poseItem->text(1).toDouble(&ok);

        if (!ok)
        {
          QMessageBox::critical(nullptr, QObject::tr("Error"),
                                QObject::tr("Invalid value found "
                                            "when parsing a Pose "
                                            "element. All pose "
                                            "elements "
                                            "must have a numeric value."
                                            ""));
          qCritical() << "Error while parsing an Include"
                         " -> Pose element. The "
                      << qUtf8Printable(poseItem->text(0))
                      << " was found with a value "
                      << qUtf8Printable(poseItem->text(1))
                      << " but a numric value was expected.";
          return false;
        }

        if (poseItem->text(0).toLower() == "x")
        {
          x = poseItem->text(1);
        }
        else if (poseItem->text(0).toLower() == "y")
        {
          y = poseItem->text(1);
        }
        else if (poseItem->text(0).toLower() == "z")
        {
          z = poseItem->text(1);
        }
        else if (poseItem->text(0).toLower() == "roll")
        {
          roll = poseItem->text(1);
        }
        else if (poseItem->text(0).toLower() == "pitch")
        {
          pitch = poseItem->text(1);
        }
        else if (poseItem->text(0).toLower() == "yaw")
        {
          yaw = poseItem->text(1);
        }
        else
        {
          QMessageBox::critical(nullptr, QObject::tr("Error"),
                                QObject::tr("An invalid tag was found while "
                                            "parsing a Pose element. Please "
                                            "check the values and try again."));
          qCritical() << "Error while parsing an Include -> "
                         "Pose element. A tag with name \""
                      << qUtf8Printable(poseItem->text(0)) << "\" and value \""
                      << qUtf8Printable(poseItem->text(1))
                      << "\" was found. But this tag is not "
                         "recognized by the parser.";
          return false;
        }
      }

      if (x.isEmpty() || y.isEmpty() || z.isEmpty() || roll.isEmpty() ||
          pitch.isEmpty() || yaw.isEmpty())
      {
        QMessageBox::critical(nullptr, QObject::tr("Error"),
                              QObject::tr("Invalid Pose element found. "
                                          "A Pose element must have X, Y"
                                          ", Z, Roll, Pitch and Yaw "
                                          "values. Please check these "
                                          "values and try again."));
        qCritical() << "Error while parsing an Include -> Pose "
                       "element. A Pose element must have X, Y,"
                       " Z, Roll, Pitch and Yaw child elements,"
                       " but at least one of those was not "
                       "found";
        return false;
      }

      QString poseStr = QString("%1 %2 %3 %4 %5 %6")
                            .arg(x)
                            .arg(y)
                            .arg(z)
                            .arg(roll)
                            .arg(pitch)
                            .arg(yaw);

      include->setPose(poseStr);
      poseFound = true;
    }
    else if (childItem->text(0).toLower() == "isstatic")
    {
      include->setStatic(childItem->text(1));
      staticFound = true;
    }
    else if (childItem->text(0).toLower() == "uri")
    {
      ///@todo Add URI validation
      include->setURI(childItem->text(1));
      uriFound = true;
    }
    else
    {
      QMessageBox::critical(nullptr, QObject::tr("Error"),
                            QObject::tr("An invalid child item was found "
                                        "under an Include item. Please "
                                        "check values and try again."));
      qCritical() << "Error while parsing an Include element. A "
                     "tag with name \""
                  << qUtf8Printable(childItem->text(0)) << "\" and value \""
                  << qUtf8Printable(childItem->text(1))
                  << "\" was found. But this tag is not "
                     "recognized by the parser.";
      return false;
    }
  }

  if (!nameFound || !poseFound || !staticFound || !uriFound)
  {
    QMessageBox::critical(nullptr, QObject::tr("Error"),
                          QObject::tr("Error wihle parsing an Include "
                                      "element. Each include element must "
                                      "have a Name, Pose, isStatic and URI "
                                      "elements. Please check these values "
                                      "and try again."));
    qCritical() << "Error while parsing an Include element. An "
                   "include element must have a Name, Pose, "
                   "isStatic and URI elements. But at least one of"
                   " those was not found.";
    return false;
  }

  return true;
}

bool World::parseGui(const QTreeWidgetItem* item, GuiDA* gui)
{
  MultiplePlugins plugins;

  for (int i = 0; i < item->childCount(); i++)
  {
    QTreeWidgetItem* childItem = item->child(i);

    if (childItem->text(0).toLower() == "fullscreen")
    {
      gui->setFullscreen(childItem->text(1));
    }
    else if (childItem->text(0).toLower() == "plugin")
    {
      PluginDA plugin;
      if (!parsePlugin(childItem, &plugin))
        return false;
      plugins.addPlugin(plugin);
    }
    else
    {
      QMessageBox::critical(nullptr, QObject::tr("Error"),
                            QObject::tr("An invalid child item was found "
                                        "under a GUI item. Please "
                                        "check these values and try again."));
      qCritical() << "Error while parsing a GUI element. An element with "
                     "tag name \""
                  << qUtf8Printable(childItem->text(0)) << "\" and value \""
                  << qUtf8Printable(childItem->text(1))
                  << "\" was found. But this tag is not "
                     "recognized by the parser.";
      return false;
    }
  }

  gui->setPlugins(plugins);
  return true;
}

bool parsePlugin(const QTreeWidgetItem* item, PluginDA* plugin)
{
  if (item->childCount() < 2)
  {
    QMessageBox::critical(nullptr, QObject::tr("Error"),
                          QObject::tr("Eror while trying to parse "
                                      "a plugin element. Each plugin"
                                      "element must have at least "
                                      "a name and filename tags."
                                      "Please verify the world "
                                      "plugins and try again."));
    qCritical() << "Error while parsing a plugin element. A plugin "
                   "with "
                << item->childCount()
                << " was found. But each plugin element must have "
                   "at least two child elements.";
    return false;
  }

  bool nameFound = false;
  bool filenameFound = false;

  for (int i = 0; i < item->childCount(); i++)
  {
    QTreeWidgetItem* childItem = item->child(i);
    if (childItem->text(0).toLower() == "name")
    {
      plugin->setName(childItem->text(1));

      nameFound = true;
    }
    else if (childItem->text(0).toLower() == "filename")
    {
      plugin->setFilename(childItem->text(1));

      filenameFound = true;
    }
    else
    {
      // If it is not a name nor a filename element, it must be
      // a parameter
      plugin->setParameter(childItem->text(0), childItem->text(1));
    }
  }

  if (!nameFound || !filenameFound)
  {
    QMessageBox::critical(nullptr, QObject::tr("Error"),
                          QObject::tr("Error while parsing a "
                                      "plugin element. Every "
                                      "plugin must have a name and"
                                      " a filename tag. Plase "
                                      "check the world plugins and"
                                      " try again."));
    qCritical() << "Error while parsing a plugin element. Each "
                   "plugin must have at least name and filename "
                   "child elements, but at leat one of those was "
                   "not found.";
    return false;
  }

  return true;
}

WorldFile World::getWorld() const
{
  return _world;
}

void World::setWorld(const WorldFile& world)
{
  _world = world;
}

void World::splitPose(const QString& pose, QTreeWidgetItem* element)
{
  QString x, y, z, roll, pitch, yaw;
  x = "0";
  y = "0";
  z = "0";
  roll = "0";
  pitch = "0";
  yaw = "0";
  if (!pose.isEmpty())
  {
    QStringList splitedPose = pose.split(" ");
    if (splitedPose.size() == 6)
    {
      x = splitedPose.at(0);
      y = splitedPose.at(1);
      z = splitedPose.at(2);
      roll = splitedPose.at(3);
      pitch = splitedPose.at(4);
      yaw = splitedPose.at(5);
    }
  }

  addChild(element, "X", x, true);
  addChild(element, "Y", y, true);
  addChild(element, "Z", z, true);
  addChild(element, "Roll", roll, true);
  addChild(element, "Pitch", pitch, true);
  addChild(element, "Yaw", yaw, true);
}
