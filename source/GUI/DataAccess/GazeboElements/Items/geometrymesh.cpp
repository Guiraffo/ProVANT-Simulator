#include "geometrymesh.h"


GeometryMesh::GeometryMesh()
{

}

std::vector<std::string> GeometryMesh::GetValues()
{
    return values;
}

void GeometryMesh::SetValue(std::vector<std::string> values)
{
     this->values = values;
}

void GeometryMesh::Read(QDomNode document)
{
    values.clear();
    values.push_back(document.firstChildElement("geometry")
                             .firstChildElement("mesh")
                             .firstChildElement("uri")
                             .text()
                             .toStdString());
}

void GeometryMesh::Write(QXmlStreamWriter* xml)
{
    if(values.at(0)!="")
    {
        xml->writeStartElement("geometry");
        xml->writeStartElement("mesh");
        xml->writeTextElement("uri",values.at(0).c_str());
        xml->writeEndElement();
        xml->writeEndElement();
    }
}

void GeometryMesh::print()
{
    if(values.size()>0)
    qDebug() << "Mesh Uri: " << values.at(0).c_str();
}
