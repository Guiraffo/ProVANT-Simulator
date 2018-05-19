#ifndef MULTIPLEPLUGINS_H
#define MULTIPLEPLUGINS_H

#include "DataAccess/GazeboElements/Items/plugin_da.h"

class multipleplugins
{
public:
    std::vector<plugin_DA> multipleItens;
    multipleplugins();
    void NewPlugin(plugin_DA);
    void Read(QDomNode);
    void Clear();
    void print();
};

#endif // MULTIPLEPLUGINS_H




