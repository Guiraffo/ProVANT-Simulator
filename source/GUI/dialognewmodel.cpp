#include "dialognewmodel.h"
#include "ui_dialognewmodel.h"
#include "qdebug.h"
#include "qdir.h"
#include"Business/treeitens.h"
#include "Utils/appsettings.h"

Dialognewmodel::Dialognewmodel(Ui::MainWindow* last,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialognewmodel),
    parentUi(last)
{
    ui->setupUi(this);
}

Dialognewmodel::~Dialognewmodel()
{
    delete ui;
}

void Dialognewmodel::newModel()
{
    AppSettings settings;
    char const* tmp = settings.getGazeboModelPath().toStdString().c_str();
    if ( tmp == NULL ) {
        qDebug() << "Problemas com variavel de ambiente ";
    } else {
        std::string env(tmp);
        QDir dir(env.c_str());
        QFileInfoList files = dir.entryInfoList();
        foreach (QFileInfo file, files)
        {
            if (file.isDir())
            {
                if(file.fileName().size()>2)
                {
                    ui->comboBox->addItem(file.fileName());
                }
            }
        }
    }
}

void Dialognewmodel::on_buttonBox_accepted()
{
    for(int i = 0; i< parentUi->treeWidget->topLevelItemCount();i++)
    {
          QTreeWidgetItem* item = parentUi->treeWidget->topLevelItem(i);
          if(item->text(0)=="Include")
          {
              AppSettings settings;
              char const* tmp = settings.getGazeboModelPath()
                      .toStdString().c_str();
              if ( tmp == NULL ) {
                  qDebug() << "Problemas com variavel de ambiente ";
              } else {
                  std::string env(tmp);
                  QDir dir(env.c_str());
                  QFileInfoList files = dir.entryInfoList();
                  foreach (QFileInfo file, files)
                  {
                      if (file.isDir())
                      {
                          QStringList list;
                          list = item->child(3)->text(1).split("//");
                          if(list.at(1)==file.fileName())
                          {
                              item->child(0)->setText(1,"newmodel");
                              item->child(1)->child(0)->setText(1,"0");
                              item->child(1)->child(1)->setText(1,"0");
                              item->child(1)->child(2)->setText(1,"0");
                              item->child(1)->child(3)->setText(1,"0");
                              item->child(1)->child(4)->setText(1,"0");
                              item->child(1)->child(5)->setText(1,"0");
                              item->child(2)->setText(1,"false");
                              item->child(3)->setText(1,"model://"+ui->comboBox->currentText());
                              return;
                          }
                      }
                  }
              }
          }
    }
    QTreeWidgetItem *element, *elementPose, *edit;
    element = addRoot("Include", "", parentUi->treeWidget);
    edit = addChild(element,"name","newmodel");
    edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
    elementPose = addChild(element,"pose","");
    std::string vector("0 0 0 0 0 0");
    splitvector(vector,elementPose);
    addChild(element,"isStatic","false");
    addChild(element,"uri","model://"+ui->comboBox->currentText());
}

void Dialognewmodel::splitvector(std::string data,QTreeWidgetItem* Element)
{
    if(data != "")
    {
        QStringList splitvector;
        QString vector;
        vector = QString::fromStdString(data);
        QRegExp rx("(\\ |\\  |\\   |\\    |\\     |\\        |\\         |\\          |\\           |\\n|\\t)");
        splitvector = vector.split(rx);
        QStringList result;
        foreach (const QString &str, splitvector)
        {
            if (str.contains(" ")||str.size()==0)
            {
                //faz nada
            }
            else
            {
                result += str;
            }
        }
        QTreeWidgetItem* edit;
        edit = addChild(Element, "X", result.at(0));
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = addChild(Element, "Y", result.at(1));
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = addChild(Element, "Z", result.at(2));
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = addChild(Element, "Roll", result.at(3));
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = addChild(Element, "Pitch", result.at(4));
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = addChild(Element, "Yaw", result.at(5));
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
    }
}

