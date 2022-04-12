#ifndef MODEL_H
#define MODEL_H

#include "DataAccess/GazeboElements/modelfile.h"

#include <QTreeWidgetItem>

class Model
{
public:
  Model();

  /**
   * @brief getFirst
   * @todo Remove in next version, maintained for compatibility.
   */
  void getFirst(std::string, QTreeWidget*) __attribute__((deprecated));
  void getFirst(const QString& filename, QTreeWidget* treeWidget);

  void toTreeWidget(QTreeWidget* tree);

  /**
   * @brief splitvector
   * @todo Remove in next version.
   */
  void splitvector(std::string, QTreeWidgetItem*, bool)
      __attribute__((deprecated));

  /**
   * @brief splitVector Receives a vector with a string with 6 elements
   * containing the pose of a model in the order x, y, z, roll, pitch and yaw
   * and creates tree widget itens filled with their respective values to
   * allow presentation in the GUI.
   *
   * @param data String containing the pose data.
   * @param item Parent tree item to add the newly created itens containg the
   * pose.
   */
  void splitVector(const QString& data, QTreeWidgetItem* item);

  ModelFile* model;
};

#endif  // MODEL_H
