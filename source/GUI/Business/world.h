#ifndef WORLD_H
#define WORLD_H

#include <string>

#include <QTreeWidgetItem>

#include "DataAccess/GazeboElements/worldfile.h"

/**
 * @brief The world class
 * @todo Document this class.
 */
class World
{
public:
  World();

  /**
   * @brief getFirst
   * @todo Remove this method.
   */
  void loadWorld(std::string, QTreeWidget*) __attribute__((deprecated));
  void loadWorld(const QString& filename, QTreeWidget* root);

  bool write(QTreeWidget* root);

  WorldFile getWorld() const;
  void setWorld(const WorldFile& world);

protected:
  WorldFile _world;

  void splitPose(const QString& pose, QTreeWidgetItem* element);
  void toTreeWidget(QTreeWidget* root);
  void readPlugin(QTreeWidgetItem* root, const PluginDA& plugin);
  void readPlugins(QTreeWidget* root, const MultiplePlugins& plugins);
  void readWorldPlugins(QTreeWidget* root, const MultiplePlugins& plugins);
  bool parsePhysics(const QTreeWidgetItem* item, PhysicsDA* physics);
  bool parseGravity(const QTreeWidgetItem* item, GravityDA* gravity);
  bool parsePlugin(const QTreeWidgetItem* item, PluginDA* plugin);
  bool parseInclude(const QTreeWidgetItem* item, IncludeDA* include);
  bool parseGui(const QTreeWidgetItem* item, GuiDA* gui);
};

#endif  // WORLD_H
