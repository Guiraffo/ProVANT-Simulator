#ifndef SCENE_H
#define SCENE_H

#include <QDomElement>
#include <QString>
#include <QXmlStreamWriter>

/**
 * @brief The Scene class
 * @todo This is an incomplete class. A full refactor is needed.
 */
class Scene
{
public:
  Scene();

  void write(QXmlStreamWriter* xml) const;
  bool read(const QDomElement& world);
  void print() const;
};

#endif  // SCENE_H
