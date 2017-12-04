#ifndef MATERIAL_H
#define MATERIAL_H
#include"vector"
#include"string"
#include"QtXml"

class Material
{
    std::string ambient;
    std::string diffuse;
    std::string specular;
    std::string emissive;

public:
    std::string GetEmissive();
    std::string GetDiffuse();
    std::string GetSpecular();
    std::string GetAmbient();
    void SetEmissive(std::string );
    void SetDiffuse(std::string );
    void SetSpecular(std::string );
    void SetAmbient(std::string );
    Material();
    void Read(QDomNode);
    void Write(QXmlStreamWriter*);
    void print();
};

#endif // MATERIAL_H
