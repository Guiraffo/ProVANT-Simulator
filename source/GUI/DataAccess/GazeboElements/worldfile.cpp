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
WorldPlugin WorldFile::GetPlugin(){return plugin;}
void WorldFile::SetPlugin(WorldPlugin value){plugin = value;}
std::vector<Include_DA> WorldFile::GetListsInclude(){return ListsInclude;}
void WorldFile::AddInclude(std::vector<Include_DA> data){ListsInclude = data;}
void WorldFile::DeleteInclude(int i){ListsInclude.erase(ListsInclude.begin()+i-1);}
bool WorldFile::Read()
{
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

                sdfVersion = doc.firstChildElement("sdf")
                                .attribute("version")
                                .toStdString();

                QDomNode itens = doc.firstChildElement("sdf")
                                    .firstChildElement("world");

                g.Read(itens);
                physics.Read(itens);
                QDomNode itens2 = itens.firstChildElement("plugin");
                plugin.Read(&itens2);

                ListsInclude.clear();
                while(true)
                {
                    Include_DA teste;
                    if(teste.Read(&itens)) break;
                    ListsInclude.push_back(teste);
                }
                QDomNode itens3 = itens.firstChildElement("scene");
                sceneObj.Read(&itens3);
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
        g.Write(&xml);
        physics.Write(&xml);
        plugin.Write(&xml);
        for(uint i = 0; i<ListsInclude.size();i++)ListsInclude.at(i).Write(&xml);
        sceneObj.Write(&xml);
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
    plugin.print();
    for(uint i=0;i<ListsInclude.size();i++)ListsInclude.at(i).print();
    sceneObj.print();
}


