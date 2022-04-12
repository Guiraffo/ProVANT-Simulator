#include "modelfile.h"

#include <QDebug>

ModelFile::ModelFile(const QString& modelFilePath) : file(modelFilePath)
{
  filename = modelFilePath;
}

void ModelFile::Read()
{
  if (file.open(QIODevice::ReadOnly | QIODevice::Text))
  {
    QString erro;
    int line, column;
    if (doc.setContent(&file, &erro, &line, &column))
    {
      file.close();

      sdfVersion = doc.firstChildElement("sdf").attribute("version");

      QDomNode itens = doc.firstChildElement("sdf");

      model.Read(itens);
    }
    else
    {
      if (erro == "unexpected end of file")
      {
        qDebug() << "Arquivo Vazio";
      }
      else
      {
        qDebug() << "Problemas com conteudo" << erro;
        qDebug("Linha %d", line);
        qDebug("Coluna %d", column);
        file.close();
        exit(1);
      }
    }
  }
  else
  {
    QFileInfo finfo(file);
    qFatal("%s%s%s",
           qUtf8Printable(QObject::tr("Error while tyring to read the "
                                      "contents of SDF file with path ")),
           qUtf8Printable(finfo.absoluteFilePath()),
           qUtf8Printable(QObject::tr(". Please make sure that this file "
                                      "exists and is readable to the "
                                      "current user.")));
    file.close();
    QCoreApplication::exit(-1);
  }
  file.close();
}

void ModelFile::Write()
{
  if (file.open(QIODevice::ReadWrite | QIODevice::Truncate))
  {
    QXmlStreamWriter xml;
    xml.setAutoFormatting(true);
    xml.setDevice(&file);
    xml.writeStartDocument();
    xml.writeStartElement("sdf");
    xml.writeAttribute("version", sdfVersion);
    // model.Write(&xml);
    xml.writeEndDocument();
  }
  else
  {
    QFileInfo finfo(file);
    qFatal("%s%s%s",
           qUtf8Printable(QObject::tr("Error while trying to read the SDF file "
                                      "with "
                                      "path ")),
           qUtf8Printable(finfo.absoluteFilePath()),
           qUtf8Printable(QObject::tr(". Please make that this file exists"
                                      " and is writable to the current "
                                      "user.")));
    file.close();
    exit(1);
  }
  file.close();
}

void ModelFile::print()
{
  qDebug() << "ModelFile";
  qDebug() << "filename " << filename;
  model.print();
}
