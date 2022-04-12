#ifndef COLLISION_H
#define COLLISION_H
#include "geometrymesh.h"
#include "QtXml"

class Collision
{
public:
  Collision();
  void Read(QDomNode);
  void Write(QXmlStreamWriter*);
  void print();

  std::string name;
  std::string pose;
  Geometry* geometry;
};

#endif  // COLLISION_H
