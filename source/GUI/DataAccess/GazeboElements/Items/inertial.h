#ifndef INERTIAL_H
#define INERTIAL_H
#include "string"
#include "QtXml"
class Inertial
{
public:
  Inertial();
  std::string GetPose();
  std::string GetMass();
  std::string GetIxx();
  std::string GetIxy();
  std::string GetIxz();
  std::string GetIyy();
  std::string GetIyz();
  std::string GetIzz();
  void SetPose(std::string);
  void SetMass(std::string);
  void SetIxx(std::string);
  void SetIxy(std::string);
  void SetIxz(std::string);
  void SetIyy(std::string);
  void SetIyz(std::string);
  void SetIzz(std::string);

  void Read(QDomNode);
  void Write(QXmlStreamWriter*);
  void print();

private:
  std::string pose;
  std::string mass;
  std::string ixx;
  std::string ixy;
  std::string ixz;
  std::string iyy;
  std::string iyz;
  std::string izz;
};

#endif  // INERTIAL_H
