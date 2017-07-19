#ifndef GEOMETRY_H
#define GEOMETRY_H
#include "QtXml"
#include"string"

class Geometry
{
public:
    Geometry(){}
    virtual std::vector<std::string> GetValues() = 0;
    virtual void SetValue(std::vector<std::string>) = 0;
    virtual void Read(QDomNode) = 0 ;
    virtual void Write(QXmlStreamWriter*)=0;
    virtual void print()=0;
};

#endif // GEOMETRY_H
