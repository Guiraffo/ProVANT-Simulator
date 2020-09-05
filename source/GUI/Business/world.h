#ifndef WORLD_H
#define WORLD_H
#include"DataAccess/GazeboElements/worldfile.h"
#include "QTreeWidgetItem"
#include "treeitens.h"

/**
 * @brief The world class
 * @todo Refactor this class.
 */
class world
{
public:
    world();
    void getFirst(std::string,QTreeWidget*) __attribute__((deprecated));
    void getFirst(const QString &filename, QTreeWidget *root);

    void Write(QTreeWidget* root);
    void ToTreeWidget(QTreeWidget*);
    void splitvector(std::string ,QTreeWidgetItem* );
//private:
    WorldFile* templateword;
    WorldFile* word;


};

#endif // WORLD_H
