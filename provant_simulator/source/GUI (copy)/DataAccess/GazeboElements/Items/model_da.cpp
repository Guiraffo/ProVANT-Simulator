#include "model_da.h"

Model_DA::Model_DA(){}
std::string Model_DA::GetName(){return name;}
void Model_DA::SetName(std::string value){name = value;}
std::vector<link_DA> Model_DA::GetListsLinks(){return ListsLinks;}
void Model_DA::AddLink(link_DA data,int pos){ ListsLinks.insert(ListsLinks.begin()+pos-1,data);}
void Model_DA::AddLink(link_DA data){ ListsLinks.push_back(data);}
void Model_DA::DeleteLink(int i){ListsLinks.erase(ListsLinks.begin()+i-1);}
std::vector<joint_DA> Model_DA::GetListsJoints(){return ListsJoints;}
void Model_DA::AddJoint(joint_DA data,int pos){ ListsJoints.insert(ListsJoints.begin()+pos-1,data);}
void Model_DA::AddJoint(joint_DA data){ ListsJoints.push_back(data);}
void Model_DA::DeleteJoint(int i){ListsJoints.erase(ListsJoints.begin()+i-1);}
std::vector<ModelPlugin> Model_DA::GetListsPlugins(){return ListsPlugins;}
void Model_DA::AddPlugin(ModelPlugin data,int pos){ ListsPlugins.insert(ListsPlugins.begin()+pos-1,data);}
void Model_DA::AddPlugin(ModelPlugin data){ ListsPlugins.push_back(data);}
void Model_DA::DeletePlugin(int i){ListsPlugins.erase(ListsPlugins.begin()+i-1);}

void Model_DA::Write(QXmlStreamWriter* xml)
{
    xml->writeStartElement("model");
    xml->writeAttribute("name","modelo");

    for(uint i = 0; i< ListsLinks.size();i++) ListsLinks.at(i).Write(xml,i,ListsSensors);
    for(uint i = 0; i< ListsJoints.size();i++) ListsJoints.at(i).Write(xml);
    for(uint i = 0; i< ListsPlugins.size();i++) ListsPlugins.at(i).Write(xml);
    xml->writeEndElement();
}

void Model_DA::Read(QDomNode document)
{
    name = document.firstChildElement("model")
                   .attribute("name")
                   .toStdString();

    QDomNode itens = document.firstChildElement("model");
    ListsLinks.clear();
    while(true)
    {
        link_DA teste;
        if(teste.Read(&itens)) break;
        ListsLinks.push_back(teste);
    }
    QDomNode itens2 = document.firstChildElement("model");
    ListsJoints.clear();
    while(true)
    {
        joint_DA teste;
        if(teste.Read(&itens2)) break;
        ListsJoints.push_back(teste);
    }
    QDomNode itens3 = document.firstChildElement("model");
    ListsPlugins.clear();
    while(true)
    {
        ModelPlugin teste;
        if(teste.Read(&itens3)) break;
        ListsPlugins.push_back(teste);
    }
    QDomNode itens4 = document.firstChildElement("model");
    ListsSensors.clear();
    while(true)
    {
        sensor teste;
        bool out2;
        if(teste.Read(&itens4,&out2))break;
        if(out2) ListsSensors.push_back(teste);
    }
}

void Model_DA::print()
{
    qDebug() << "Model";
    qDebug() << "name " << name.c_str();
    for(uint i = 0; i< ListsLinks.size();i++) ListsLinks.at(i).print();
    for(uint i = 0; i< ListsJoints.size();i++) ListsJoints.at(i).print();
    for(uint i = 0; i< ListsPlugins.size();i++) ListsPlugins.at(i).print();
    for(uint i = 0; i< ListsSensors.size();i++) ListsSensors.at(i).print();
}

