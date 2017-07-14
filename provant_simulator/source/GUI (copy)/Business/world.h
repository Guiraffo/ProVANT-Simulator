#ifndef WORLD_H
#define WORLD_H
#include"DataAccess/GazeboElements/worldfile.h"
#include "QTreeWidgetItem"
#include "treeitens.h"

class world
{
public:
    world();
    void getTemplate(std::string,QTreeWidget*);
    void getLast(QTreeWidget*);
    void getFirst(std::string,QTreeWidget*);
    void getActual(QTreeWidget*);
    void Write(QTreeWidget* root);
    void ToTreeWidget(QTreeWidget*);
    void splitvector(std::string ,QTreeWidgetItem* );
//private:
    WorldFile* templateword;
    WorldFile* firstword;
    WorldFile* lastword;
    WorldFile* actualword;

};

#endif // WORLD_H
