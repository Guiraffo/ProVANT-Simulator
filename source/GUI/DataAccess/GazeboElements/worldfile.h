#ifndef WORLDFILE_H
#define WORLDFILE_H

#include"Items/gravity_da.h"
#include"Items/include_da.h"
#include"Items/physics_da.h"
#include"Items/worldplugin.h"
#include"Items/scene.h"


class WorldFile
{
public:
    WorldFile(std::string);
    WorldFile();
    gravity_DA GetGravity();
    void SetGravity(gravity_DA);
    physics_DA GetPhysics();
    void SetPhysics(physics_DA);
    WorldPlugin GetPlugin();
    void SetPlugin(WorldPlugin);
    std::vector<Include_DA>GetListsInclude();
    void AddInclude(std::vector<Include_DA>);
    void DeleteInclude(int i);
    bool Read();
    void Write();
    void print();
    std::string Filename;
    QFile file;
    std::string sdfVersion;
private:
    QDomDocument doc;
    gravity_DA g;
    physics_DA physics;
    WorldPlugin plugin;
    scene sceneObj;
    std::vector<Include_DA> ListsInclude;
};

#endif // WORLDFILE_H
