#ifndef INCLUDE_DA_H
#define INCLUDE_DA_H
#include "QtXml/QtXml"


class Include_DA
{
public:
    Include_DA();
    std::string GetUri();
    void SetUri(std::string);
    std::string GetName();
    void SetName(std::string);
    std::string GetIsStatic();
    void SetIsStatic(std::string);
    std::string GetPose();
    void SetPose(std::string);

    void Write(QXmlStreamWriter*);
    bool Read(QDomNode*);
    void print();

private:
    std::string uri;
    std::string name;
    std::string isStatic;
    std::string pose;
};

#endif // INCLUDE_DA_H
