#include "modellist.h"

#include "Utils/appsettings.h"

ModelList::ModelList()
{
  findModels();
}

void ModelList::update()
{
  findModels();
}

const QStringList& ModelList::models() const
{
  return _models;
}

bool ModelList::isAModel(const QString& name) const
{
  return _models.contains(name.toLower());
}

void ModelList::findModels()
{
  _models.clear();

  AppSettings settings;
  QString gazeboModelPath = settings.getGazeboModelPath();
  QDir gazeboModelsDir(gazeboModelPath);

  if (gazeboModelsDir.exists())
  {
    QFileInfoList files = gazeboModelsDir.entryInfoList(
        QDir::Dirs | QDir::NoDotAndDotDot | QDir::Readable);
    foreach (QFileInfo file, files)
    {
      QDir modelDir(file.absoluteFilePath());
      // Check that the model has a config.xml file
      QFileInfo configFile(modelDir.absoluteFilePath(QDir::cleanPath(
          QString("config") + QDir::separator() + QString("config.xml"))));
      if (!configFile.exists())
      {
        // Skip models that don't have a config file
        continue;
      }
      // Check that the model has a SDF file
      QFileInfo sdfFile(modelDir.absoluteFilePath(QDir::cleanPath(
          QString("robot") + QDir::separator() + QString("model.sdf"))));
      if (!sdfFile.exists())
      {
        // Skip models that don't have a SDF file
        continue;
      }

      _models.append(modelDir.dirName());
    }
  }
}
