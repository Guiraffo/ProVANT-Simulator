#ifndef ROSLAUNCH_H
#define ROSLAUNCH_H

#include "QString"
#include "qdebug.h"
#include "qfile.h"

class roslaunch
{
public:
  roslaunch();
  static void WriteNew(QString, QString, bool);
};

#endif  // ROSLAUNCH_H
