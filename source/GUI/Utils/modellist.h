#ifndef MODELLIST_H
#define MODELLIST_H

#include <QStringList>

class ModelList
{
public:
  ModelList();

  void update();
  const QStringList& models() const;
  bool isAModel(const QString& name) const;

protected:
  void findModels();
  QStringList _models;
};

#endif // MODELLIST_H
