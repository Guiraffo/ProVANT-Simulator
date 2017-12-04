#include "treeitens.h"

TreeItens::TreeItens()
{

}

QTreeWidgetItem * TreeItens::AddRoot(std::string name, std::string description,QTreeWidget* root)
{
    QTreeWidgetItem * item = new QTreeWidgetItem(root);
    item->setText(0, QString::fromStdString(name));
    item->setText(1, QString::fromStdString(description));
    root->addTopLevelItem(item);
    return item;
}


QTreeWidgetItem * TreeItens::AddChild(QTreeWidgetItem * item,std::string name, std::string description)
{
    QTreeWidgetItem * subitem = new QTreeWidgetItem(item);
    subitem->setText(0, QString::fromStdString(name));
    subitem->setText(1, QString::fromStdString(description));
    item->addChild(subitem);
    return subitem;
}
