#include "gravity_da.h"

gravity_DA::gravity_DA()
{

}

void gravity_DA::Read(QDomNode document)
{
    gravity = document.firstChildElement("gravity").text().toStdString();
}

void gravity_DA::Write(QXmlStreamWriter xml)
{
    xml.writeTextElement("gravity",gravity.c_str());
}

std::string gravity_DA::GetGravity(){return gravity;}

void gravity_DA::SetGravity(std::string value){ gravity = value;}

void gravity_DA::print()
{
    qDebug() << "Gravity " << gravity.c_str();
}


