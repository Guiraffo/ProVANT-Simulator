#ifndef MODEL_H
#define MODEL_H

#include"DataAccess/GazeboElements/modelfile.h"
#include"DataAccess/ControllerElements/configfile.h"
#include "QTreeWidgetItem"
#include "treeitens.h"

class Model
{
public:
    Model();
    void getTemplate(std::string,QTreeWidget*);
    void getLast(QTreeWidget*);
    void getFirst(std::string,QTreeWidget*);
    void getActual(QTreeWidget*);
    void Write(QTreeWidget*);
    void ToTreeWidget(QTreeWidget*);
    void splitvector(std::string,QTreeWidgetItem*,bool);
    ModelFile* model;



};

#endif // MODEL_H
