#ifndef MODELPLUGIN_H
#define MODELPLUGIN_H
#include "plugin_da.h"

class ModelPlugin : public PluginDA
{
public:
  ModelPlugin();
  QDomNode Read(QDomNode);

  void Write(QXmlStreamWriter) __attribute__((deprecated));
  void print();
};

#endif  // MODELPLUGIN_H
