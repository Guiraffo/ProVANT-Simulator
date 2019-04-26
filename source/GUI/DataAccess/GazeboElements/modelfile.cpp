#include "modelfile.h"

ModelFile::ModelFile(std::string value):file(value.c_str())
{
    qDebug() << value.c_str();
    filename = value;
}

void ModelFile::Read()
{
    if(file.exists()) qDebug() << "existe";
    if(file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
         qDebug() << "teste 1";
        QString erro;
        int line, column;
        if(doc.setContent(&file,&erro,&line,&column))
        {

             qDebug() << "teste 2";
            file.close();

            sdfVersion = doc.firstChildElement("sdf")
                            .attribute("version")
                            .toStdString();

            QDomNode itens = doc.firstChildElement("sdf");

            model.Read(itens);
            //model.print();
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
        qDebug(" 41 Problemas com o arquivo xml");
        file.close();
        exit(1);
    }
    file.close();
}

void ModelFile::Write()
{
   if(file.open(QIODevice::ReadWrite|QIODevice::Truncate))
    {
        QXmlStreamWriter xml;
        xml.setAutoFormatting(true);
        xml.setDevice(&file);
        xml.writeStartDocument();
        xml.writeStartElement("sdf");
        xml.writeAttribute("version",sdfVersion.c_str());
        //model.Write(&xml);
        xml.writeEndDocument();
    }
    else
    {
        // TO DO: criar exceção
        qDebug(" 5 Problemas com o arquivo xml");
        file.close();
        exit(1);
    }
    file.close();
}

void ModelFile::print()
{
    qDebug() << "ModelFile";
    qDebug() << "filename " << filename.c_str();
    model.print();
}

