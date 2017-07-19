#ifndef GEOMETRYMESH_H
#define GEOMETRYMESH_H
#include"geometry.h"

class GeometryMesh:public Geometry
{
    std::vector<std::string> values;
public:
    GeometryMesh();
    std::vector<std::string> GetValues();
    void SetValue(std::vector<std::string>);
    void Read(QDomNode);
    void Write(QXmlStreamWriter*);
    void print();
};

#endif // GEOMETRYURI_H
