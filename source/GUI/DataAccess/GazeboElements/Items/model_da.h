#ifndef MODELDA_H
#define MODELDA_H
#include"link_da.h"
#include"joint_da.h"
#include"modelplugin.h"
#include"Instruments/sensor.h"

class Model_DA
{
public:
    Model_DA();
    std::string GetName();
    void SetName(std::string);
    std::vector<link_DA> GetListsLinks();
    void AddLink(link_DA,int);
    void AddLink(link_DA);
    void DeleteLink(int);
    std::vector<joint_DA> GetListsJoints();
    void AddJoint(joint_DA,int);
    void DeleteJoint(int);
    std::vector<ModelPlugin> GetListsPlugins();
    void AddPlugin(ModelPlugin,int);
    void AddPlugin(ModelPlugin);
    void AddJoint(joint_DA data);
    void DeletePlugin(int);

    void Write(QXmlStreamWriter);
    void Read(QDomNode);
    void print();

private:
    std::string name;
    std::vector<link_DA> ListsLinks;
    std::vector<joint_DA> ListsJoints;
    std::vector<ModelPlugin> ListsPlugins;
public:
    std::vector<sensor> ListsSensors;
};

#endif // MODELDA_H
