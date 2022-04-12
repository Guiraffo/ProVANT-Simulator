#include "scene.h"

Scene::Scene()
{
}

void Scene::write(QXmlStreamWriter* xml) const
{
  Q_UNUSED(xml);
}

bool Scene::read(const QDomElement& world)
{
  Q_UNUSED(world);
  return true;
}

void Scene::print() const
{
}
