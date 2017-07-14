#include "controller.h"

Controller::Controller()
{

}

void Controller::get(std::string filename,QListWidget* sensor,QListWidget* actuator)
{
    config = new ConfigFile(filename);
    config->ReadFile();

    std::vector<std::string> listsensors = config->GetSensors();
    std::vector<std::string> listactuators =  config->GetActuators();
    ToListWidget(sensor,listsensors);
    ToListWidget(actuator,listactuators);
}

void Controller::Write()
{

}

void Controller::ToListWidget(QListWidget* root,std::vector<std::string> list)
{
    for(uint i=0;i<list.size();i++)
    {
        QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(list.at(i)));
        root->addItem(item);
        item->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
    }
}

