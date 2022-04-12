#ifndef LINK_DA_H
#define LINK_DA_H
#include "string"
#include "inertial.h"
#include "collision.h"
#include "visual.h"
#include "Instruments/sensor.h"

class link_DA
{
public:
  link_DA();
  bool Read(QDomNode*);
  void Write(QXmlStreamWriter, int, std::vector<sensor>);
  void print();

  std::string name;
  std::string pose;
  Inertial inertialValues;
  Visual visual;
  Collision collision;
};

#endif  // LINK_DA_H
