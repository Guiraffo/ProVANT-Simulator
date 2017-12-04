#ifndef JOINT_DA_H
#define JOINT_DA_H
#include "string"
#include "axis.h"

class joint_DA
{
public:
    joint_DA();
    bool Read(QDomNode*);
    void Write(QXmlStreamWriter*);
    void print();

    std::string type;
    std::string name;
    std::string pose;
    std::string parent;
    std::string child;
    Axis* one;
    Axis* two;

};

#endif // JOINT_DA_H
