#ifndef NEWSTRATEGY_H
#define NEWSTRATEGY_H

#include <QString>

/**
 * @brief The NewStrategy class creates a new control strategy.
 *
 * @todo Refactor to read the files from a base file inside a QResource insted
 * of direct writing it to an output file from strings.
 * @todo Refactor to separate the writing of the files to separate methods.
 * @todo Refactor to avoid the use of an unecessary static method.
 */
class NewStrategy
{
public:
    NewStrategy();
    static void createProject(const QString &name);
};

#endif // NEWSTRATEGY_H
