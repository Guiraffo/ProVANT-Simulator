#ifndef CONTROLLER_H
#define CONTROLLER_H

#include"DataAccess/ControllerElements/configfile.h"
#include"QListWidget"

class Controller
{
public:
    Controller();
    void get(std::string, QListWidget * ,QListWidget *);
    void Write();
    void ToListWidget(QListWidget*, std::vector<std::string>);
    void ToListWidget(QListWidget*);

public:
    ConfigFile* config;
};

#endif // CONTROLLER_H
