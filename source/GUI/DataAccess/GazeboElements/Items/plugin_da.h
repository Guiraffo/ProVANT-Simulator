#ifndef PLUGIN_DA_H
#define PLUGIN_DA_H
#include "QtXml"

class plugin_DA
{
public:
    plugin_DA();
    std::string GetName();
    void SetName(std::string);
    std::string GetFilename();
    void SetFilename(std::string);
    bool Read(QDomNode*);
    void Write(QXmlStreamWriter* );
    void print();
protected:
    std::string name;
    std::string filename;
};

#endif // PLUGIN_DA_H
