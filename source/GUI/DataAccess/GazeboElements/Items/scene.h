#ifndef SCENE_H
#define SCENE_H
#include "QtXml/QtXml"

class scene
{
public:
    scene();
    void Write(QXmlStreamWriter);
    void Read(QDomNode);
    void print();
};

#endif // SCENE_H
