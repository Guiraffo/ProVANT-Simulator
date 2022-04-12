#ifndef MODELFILE_H
#define MODELFILE_H

#include <QString>
#include "Items/model_da.h"

/**
 * @brief The ModelFile class
 *
 * @todo Document this class
 * @todo Make properties private.
 * @todo Create getters and setters
 */
class ModelFile
{
public:
  ModelFile(const QString& modelFilePath);
  void Read();
  void Write();
  void print();

  QString filename;
  Model_DA model;
  QDomDocument doc;
  QFile file;
  QString sdfVersion;
};

#endif  // MODELFILE_H
