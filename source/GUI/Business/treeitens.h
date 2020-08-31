#ifndef TREEITENS_H
#define TREEITENS_H

#include <QTreeWidget>
#include <QTreeWidgetItem>

/**
 * @brief The TreeItens class
 * @todo Deprecate this class.
 * This class has only static methods, its a sign that it doesn't really model
 * any object and was created only to please to gods of Object Orientation.
 *
 * But. C++ is not Java, and there is no need to waste time in doing things like
 * creating unnecessary classes as groups of static methods. Just create
 * functions. If you need to group the functions in a namespace -> Create a
 * namespace.
 */
class TreeItens
{
public:
    TreeItens();
    static QTreeWidgetItem * AddRoot(std::string name,
                                     std::string description,
                                     QTreeWidget*) __attribute__((deprecated));
    static QTreeWidgetItem * AddChild(QTreeWidgetItem * item,
                                      std::string name,
                                      std::string description)
    __attribute__((deprecated));
};

/**
 * @brief addRoot Create a new top level tree widget item, fills its name
 * and description, add it to a tree and return a pointer to such item.
 *
 * @param name Name of the new tree widget item, this data will be put in column
 * 0.
 * @param description Description of the new tree widget item, this data will
 * be put in the column 1.
 * @param tree Tree to add the new widget item.
 * @return Pointer to the new item element.
 */
QTreeWidgetItem *addRoot(const QString &name,
                         const QString &description,
                         QTreeWidget *tree);
/**
 * @brief addChild Create a new child tree widget item, fill its name and
 * description, add it to the parent item and return a pointer to newly created
 * item.
 *
 * @param parent Tree widget item to add the new child in.
 * @param name Name of the item, this data will be put in column 0.
 * @param description Description of the item, this data will be put in column
 * 1.
 * @return Pointer to the newly created item.
 */
QTreeWidgetItem *addChild(QTreeWidgetItem *parent,
                          const QString &name,
                          const QString &description);


#endif // TREEITENS_H
