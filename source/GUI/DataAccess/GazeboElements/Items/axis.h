#ifndef AXIS_H
#define AXIS_H
#include "string"
#include "QtXml"

class Axis
{
public:
  Axis(std::string);
  void Read(QDomNode);
  void Write(QXmlStreamWriter*);
  void print();

  bool has;
  std::string type;
  std::string xyz;
  std::string lower;
  std::string upper;
  std::string effort;
  std::string velocity;
  std::string damping;
  std::string friction;
};

#endif  // AXIS_H
