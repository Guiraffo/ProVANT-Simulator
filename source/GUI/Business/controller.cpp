#include "controller.h"

/**
 * @brief Controller::Controller
 * Initializes the controller object.
 */
Controller::Controller()
{

}

/**
 * @brief Controller::~Controller
 * Frees the content of the config member variable.
 */
Controller::~Controller()
{
    if(config != nullptr) delete config;
}

/**
 * @brief Controller::get Read the contents of the controller and fills the
 * QListWidgets with the sensors and controllers.
 * @param filename The path to the file containing the controller that will
 * be read.
 * @param sensors The widget that displays the sensors contained in the
 * controller.
 * @param actuators The widget that displays the actuators contained in the
 * controller.
 */
void Controller::open(const QString &filename,
                     QListWidget *sensors,
                     QListWidget *actuators)
{
    config = new ConfigFile(filename);
    config->readFile();

    toListWidget(sensors, config->getSensors());
    toListWidget(actuators, config->getActuators());
}

/**
 * @brief Controller::toListWidget Reads the content of a string list and
 * inserts then into a QListWidget.
 * @param element The list widget in which the elements will be inserted.
 * @param items The list of items to insert.
 */
void Controller::toListWidget(QListWidget *element, const QStringList &items)
{
    for(QStringList::const_iterator i = items.constBegin();
        i != items.constEnd();
        i++)
    {
        QListWidgetItem *listItem = new QListWidgetItem(*i);
        listItem->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled);
        element->addItem(listItem);
    }
}
