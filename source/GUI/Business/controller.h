#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QListWidget>

#include"DataAccess/ControllerElements/configfile.h"

/**
 * @brief The Controller class handles the reading and writing of the
 * properties of a control strategy.
 * @todo Ask Arthur about the purpose of this class.
 */
class Controller
{
public:
    Controller();
    ~Controller();
    void open(const QString &filename,
             QListWidget *sensors,
             QListWidget *actuators);
    void toListWidget(QListWidget*, const QStringList &items);

public:
    ConfigFile* config = nullptr;
};

#endif // CONTROLLER_H
