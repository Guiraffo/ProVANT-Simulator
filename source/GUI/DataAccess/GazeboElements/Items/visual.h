#ifndef VISUAL_H
#define VISUAL_H
#include "geometrymesh.h"
#include "material.h"

class Visual
{
public:
  Visual();
  void Read(QDomNode);
  void Write(QXmlStreamWriter*);
  void print();
  std::string name;
  std::string pose;
  Material material;
  Geometry* geometry;
};

#endif  // VISUAL_H
