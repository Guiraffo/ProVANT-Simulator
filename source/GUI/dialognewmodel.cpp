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
    QTreeWidgetItem* element,*elementPose,*edit;
    element = TreeItens::AddRoot("Include","",parentUi->treeWidget);
    edit = TreeItens::AddChild(element,"name","newmodel");
    edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
    elementPose = TreeItens::AddChild(element,"pose","");
    std::string vector("0 0 0 0 0 0");
    splitvector(vector,elementPose);
    TreeItens::AddChild(element,"isStatic","false");
    TreeItens::AddChild(element,"uri","model://"+ui->comboBox->currentText().toStdString());
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
        edit = TreeItens::AddChild(Element,"X",result.at(0).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(Element,"Y",result.at(1).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(Element,"Z",result.at(2).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(Element,"Roll",result.at(3).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(Element,"Pitch",result.at(4).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        edit = TreeItens::AddChild(Element,"Yaw",result.at(5).toStdString());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
    }
}

