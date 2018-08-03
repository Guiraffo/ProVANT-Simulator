#ifndef GRAVITY_DA_H
#define GRAVITY_DA_H
#include "QtXml/QtXml"

class gravity_DA
{
public:
    gravity_DA();
    void SetGravity(std::string);
    std::string GetGravity();
    void Read(QDomNode);
    void Write(QXmlStreamWriter);
    void print();
private:
    std::string gravity;

};

#endif // GRAVITY_DA_H
