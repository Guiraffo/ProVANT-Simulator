#ifndef MODELPLUGIN_H
#define MODELPLUGIN_H
#include"plugin_da.h"

class ModelPlugin: public plugin_DA
{
public:
    ModelPlugin();
    bool Read(QDomNode*);
    void Write(QXmlStreamWriter*);
    void print();

    std::vector<std::string> parameters;
    std::vector<std::string> values;
};

#endif // MODELPLUGIN_H
