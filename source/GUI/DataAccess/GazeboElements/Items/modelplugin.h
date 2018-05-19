#ifndef MODELPLUGIN_H
#define MODELPLUGIN_H
#include"plugin_da.h"

class ModelPlugin: public plugin_DA
{
public:
    ModelPlugin();
    QDomNode Read(QDomNode);
    void Write(QXmlStreamWriter);
    void print();


};

#endif // MODELPLUGIN_H
