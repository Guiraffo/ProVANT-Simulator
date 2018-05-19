#ifndef PHYSICS_DA_H
#define PHYSICS_DA_H
#include "QtXml"

class physics_DA
{
public:
    physics_DA();
    std::string GetType();
    void SetType(std::string);
    std::string GetStep();
    void SetStep(std::string);
    std::string GetRealTimeFactor();
    void SetRealTimeFactor(std::string);
    std::string GetRealTimeUpdaterate();
    void SetRealTimeUpdaterate(std::string);
    void Write(QXmlStreamWriter);
    void Read(QDomNode);
    void print();
private:
    std::string type;
    std::string step;
    std::string real_time_factor;
    std::string real_time_update_rate;
};



#endif // PHYSICS_DA_H
