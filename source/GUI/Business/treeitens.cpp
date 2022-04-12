#include "treeitens.h"

TreeItens::TreeItens()
{
}

QTreeWidgetItem* TreeItens::AddRoot(std::string name, std::string description,
                                    QTreeWidget* root)
{
  return addRoot(QString::fromStdString(name),
                 QString::fromStdString(description), root);
}

QTreeWidgetItem* TreeItens::AddChild(QTreeWidgetItem* item, std::string name,
                                     std::string description)
{
  return addChild(item, QString::fromStdString(name),
                  QString::fromStdString(description));
}

void makeItemEditable(QTreeWidgetItem* item)
{
  item->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled | Qt::ItemIsSelectable);
}

QTreeWidgetItem* addRoot(const QString& name, const QString& description,
                         QTreeWidget* tree, bool editable)
{
  QTreeWidgetItem* item = new QTreeWidgetItem(tree);
  item->setText(0, name);
  item->setText(1, description);

  if (editable)
    makeItemEditable(item);

  tree->addTopLevelItem(item);

  return item;
}

QTreeWidgetItem* addChild(QTreeWidgetItem* parent, const QString& name,
                          const QString& description, bool editable)
{
  QTreeWidgetItem* subitem = new QTreeWidgetItem(parent);
  subitem->setText(0, name);
  subitem->setText(1, description);

  if (editable)
    makeItemEditable(subitem);

  parent->addChild(subitem);
  return subitem;
}
