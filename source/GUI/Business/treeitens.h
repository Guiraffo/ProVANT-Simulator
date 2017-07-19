#ifndef TREEITENS_H
#define TREEITENS_H
#include "QTreeWidgetItem"

class TreeItens
{
public:
    TreeItens();
    static QTreeWidgetItem * AddRoot(std::string name, std::string description,QTreeWidget*);
    static QTreeWidgetItem * AddChild(QTreeWidgetItem * item,std::string name, std::string description);
};

#endif // TREEITENS_H
