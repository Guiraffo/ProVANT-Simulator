#ifndef TOOLS_H
#define TOOLS_H

#include "string"
#include <cstdlib>
#include "qstring.h"
#include "Utils/environmentexception.h"


class Tools
{
public:
    Tools();
    static QString getEnv(char* name);
};

#endif // TOOLS_H