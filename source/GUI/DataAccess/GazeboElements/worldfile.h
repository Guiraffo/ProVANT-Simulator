#ifndef WORLDFILE_H
#define WORLDFILE_H

#include"Items/gravity_da.h"
#include"Items/include_da.h"
#include"Items/physics_da.h"
#include"Items/worldplugin.h"
#include"Items/modelplugin.h"
#include"Items/scene.h"
#include"Items/multipleincludes.h"
#include"Items/multipleplugins.h"


class WorldFile
{
public:
    WorldFile(std::string);
    WorldFile();
    gravity_DA GetGravity();
    void SetGravity(gravity_DA);
    physics_DA GetPhysics();
    void SetPhysics(physics_DA);
    ModelPlugin GetPlugin();
    void SetPlugin(ModelPlugin);


    bool Read();
    void Write();
    void print();
    std::string Filename;
    QFile file;
    std::string sdfVersion;
    MultipleIncludes listIncludes;
    multipleplugins listPlugins;
private:
    QDomDocument doc;
    gravity_DA g;
    physics_DA physics;
    scene sceneObj;


};

#endif // WORLDFILE_H
