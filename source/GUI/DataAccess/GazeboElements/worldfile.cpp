#include "worldfile.h"

WorldFile::WorldFile(std::string filename):file(filename.c_str())
{
    Filename = filename;
}


WorldFile::WorldFile()
{

}

gravity_DA WorldFile::GetGravity(){return g;}
void WorldFile::SetGravity(gravity_DA value){g = value;}
physics_DA WorldFile::GetPhysics(){return physics;}
void WorldFile::SetPhysics(physics_DA value){physics = value;}
bool WorldFile::Read()
{
    // Abrindo Arquivo
    QFileInfo fileInfo(file);
    if (fileInfo.exists())
    {
        if(file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QString erro;
            int line, column;
            if(doc.setContent(&file,&erro,&line,&column))
            {
                file.close();
                // lendo informações do arquivo sdf
                sdfVersion = doc.firstChildElement("sdf")
                                .attribute("version")
                                .toStdString();
                QDomNode itens = doc.firstChildElement("sdf")
                                    .firstChildElement("world");
                qDebug() << "worlfile1";

                g.Read(itens);
                physics.Read(itens);
                listIncludes.Read(itens);
                listPlugins.Read(itens);
                sceneObj.Read(itens);
                qDebug() << "worlfile5";

            }
            else
            {
                if (erro == "unexpected end of file")
                {
                    qDebug() << "Arquivo Vazio";
                }
                else
                {
                    qDebug()<< "Problemas com conteudo" << erro;
                    qDebug("Linha %d",line);
                    qDebug("Coluna %d",column);
                    file.close();
                    exit(1);
                }
            }
        }
        else
        {
            // TO DO: criar exceção
            qDebug("Problemas com o arquivo xml");
            file.close();
            exit(1);
        }
        file.close();
        return true;
    }
    return false;
}

void WorldFile::Write()
{

    print();
    qDebug() << "ok";
    if(file.open(QIODevice::ReadWrite|QIODevice::Truncate))
    {
        QXmlStreamWriter xml;
        xml.setAutoFormatting(true);
        xml.setDevice(&file);
        xml.writeStartDocument();
        xml.writeStartElement("sdf");
        xml.writeAttribute("version",sdfVersion.c_str());
        xml.writeStartElement("world");
        xml.writeAttribute("name",Filename.c_str());

        //g.Write(xml);
        xml.writeTextElement("gravity",g.GetGravity().c_str());

        //physics.Write(xml);
        xml.writeStartElement("physics");
        xml.writeAttribute("type",physics.GetType().c_str());
        xml.writeTextElement("max_step_size",physics.GetStep().c_str());
        xml.writeTextElement("real_time_factor",physics.GetRealTimeFactor().c_str());
        if(physics.GetRealTimeUpdaterate().size()!=0)
            xml.writeTextElement("real_time_update_rate",physics.GetRealTimeUpdaterate().c_str());
        xml.writeEndElement();

        //plugin.Write(xml);
        xml.writeStartElement("plugin");
        xml.writeAttribute("name",listPlugins.multipleItens.at(0).GetName().c_str());
        xml.writeAttribute("filename",listPlugins.multipleItens.at(0).GetFilename().c_str());
        for(uint i = 0;i<listPlugins.multipleItens.at(0).parameters.size();i++)
        {
            xml.writeTextElement(listPlugins.multipleItens.at(0).parameters.at(i).c_str(),listPlugins.multipleItens.at(0).values.at(i).c_str());
        }
        xml.writeEndElement();

        for(uint i = 0; i<listIncludes.multipleItens.size();i++)
        {
            xml.writeStartElement("include");
            if(listIncludes.multipleItens.at(i).GetUri()!="")xml.writeTextElement("uri",listIncludes.multipleItens.at(i).GetUri().c_str());
            if(listIncludes.multipleItens.at(i).GetName()!="")xml.writeTextElement("name",listIncludes.multipleItens.at(i).GetName().c_str());
            if(listIncludes.multipleItens.at(i).GetIsStatic()!="")xml.writeTextElement("static",listIncludes.multipleItens.at(i).GetIsStatic().c_str());
            if(listIncludes.multipleItens.at(i).GetPose()!="")xml.writeTextElement("pose",listIncludes.multipleItens.at(i).GetPose().c_str());
            xml.writeEndElement();
        }

        //sceneObj.Write(xml);
        xml.writeStartElement("scene");
        xml.writeStartElement("sky");
        xml.writeTextElement("time","18");
        xml.writeStartElement("clouds");
        xml.writeTextElement("speed","0");
        xml.writeEndElement();
        xml.writeEndElement();
        xml.writeEndElement();
        xml.writeEndElement();
        xml.writeEndDocument();
    }
    else
    {
        // TO DO: criar exceção
        qDebug("Problemas com o arquivo xml");
        file.close();
        exit(1);
    }
    file.close();
}

void WorldFile::print(){
    g.print();
    physics.print();
    listIncludes.print();
    listPlugins.print();
}


