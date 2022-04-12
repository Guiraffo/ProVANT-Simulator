#ifndef GUI_DA_H
#define GUI_DA_H

#include <QDomDocument>
#include <QString>
#include <QXmlStreamWriter>

#include "multipleplugins.h"

/**
 * @brief The GuiDA class is a data access class that manages the GUI SDF
 * element.
 *
 * The GUI element is used to configure if the Gazebo client window is in
 * fullscreen or windowed mode, the initial position of the user camera, and
 * to add plugins to the Gazebo GUI.
 *
 * @todo Implement camera functionality.
 */
class GuiDA
{
public:
  /**
   * @brief GuiDA Default constructor of the GuiDA object. Creates an empty
   * object.
   */
  GuiDA();

  /**
   * @brief read Read a GUI element from a SDF file.
   * @param world XML element containing the SDF world. Must be a valid XMl
   * element and contain a GUI element.
   * @return true if an object could be succesfully read and false otherwise.
   */
  bool read(const QDomElement& world);
  /**
   * @brief write Writes the GUI element to a XML file.
   * @param xml The xml stream to write on.
   */
  void write(QXmlStreamWriter* xml) const;

  /**
   * @brief isEmpty Indicates if this element has any property set, or if it
   * is empty.
   * @return
   */
  bool isEmpty() const;

  /**
   * @brief fullscreen Indicates if the Gazebo window is in fullscreen mode
   * (true) or in windows mode (false).
   * The default value is false.
   * @return Status of the Gazebo windows fullscreen mode.
   */
  const QString& fullscreen() const;
  /**
   * @brief setFullscreen Updates the Gazebo window fullscreen status.
   * @param fullscren Indicates if the Gazebo window is in fullscreen or
   * windowed mode.
   * @sa fullscreen()
   */
  void setFullscreen(const QString& fullscren);

  /**
   * @brief getPlugins Returns the list of Gazebo GUI plugins.
   * @return
   */
  const MultiplePlugins& getPlugins() const;
  /**
   * @brief setPlugins Updates the list of Gazebo GUI plugins.
   * @param plugins List of GUI plugins.
   */
  void setPlugins(const MultiplePlugins& plugins);

private:
  //! Stores the list of gui plugins
  MultiplePlugins _plugins;
  //! Stores the fullscreen status of the Gazebo window (boolean)
  QString _fullscreen;
};

#endif  // GUI_DA_H
