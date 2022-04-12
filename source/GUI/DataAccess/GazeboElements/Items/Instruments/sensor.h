#ifndef SENSOR_H
#define SENSOR_H

#include "QString"
#include "QtXml"

// for imu
struct noise
{
  QString type;
  QString mean;
  QString stddev;
  QString bias_mean;
  QString bias_stddev;
  QString precision;
};

class sensor
{
public:
  int link;  // ordem que se encontra no vetor de links
public:
  QString name;
  QString type;
  QString always_on;
  QString update_rate;
  QString visualize;
  QString topic;
  QString pose;

  // sonar
  QString min;
  QString max;
  QString radius;

  // imu
  noise ang_x;
  noise ang_y;
  noise ang_z;
  noise accel_x;
  noise accel_y;
  noise accel_z;

  // gps
  noise pos_horizontal;
  noise pos_vertical;
  noise vel_horizontal;
  noise vel_vertical;

  // magnetometer
  noise x;
  noise y;
  noise z;

public:
  sensor();
  bool Read(QDomNode* document, bool* out2);
  void Write(QXmlStreamWriter* xml);
  void print();

  void ReadSonar(QDomNode* document);
  void WriteSonar(QXmlStreamWriter* xml);
  void printSonar();

  void ReadImu(QDomNode* document);
  void WriteImu(QXmlStreamWriter* xml);
  void printImu();

  void ReadGps(QDomNode* document);
  void WriteGps(QXmlStreamWriter* xml);
  void printGps();

  void ReadMagnetometer(QDomNode* document);
  void WriteMagnetometer(QXmlStreamWriter* xml);
  void printMagnetometer();
};

#endif  // SONAR_H
