#ifndef MULTIPLEINCLUDES_H
#define MULTIPLEINCLUDES_H

#include "include_da.h"

class MultipleIncludes
{

public:
    std::vector<Include_DA> multipleItens;
    MultipleIncludes();
    void NewInclude(Include_DA);
    void Read(QDomNode);
    void Clear();
    void print();

};

#endif // MULTIPLEINCLUDES_H
