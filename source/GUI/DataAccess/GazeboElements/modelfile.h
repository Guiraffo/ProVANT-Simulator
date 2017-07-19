#ifndef MODELFILE_H
#define MODELFILE_H
#include "string"
#include "Items/model_da.h"

class ModelFile
{
public:
    ModelFile(std::string);
    void Read();
    void Write();
    void print();
    std::string filename;
    Model_DA model;
    QDomDocument doc;
    QFile file;
    std::string sdfVersion;
};

#endif // MODELFILE_H
