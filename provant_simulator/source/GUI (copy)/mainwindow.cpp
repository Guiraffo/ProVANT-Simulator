#include "mainwindow.h"
#include "dialog.h"
#include "dialognewmodel.h"
#include "DataAccess/RosElements/roslaunch.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->treeWidget->setColumnCount(2);
    ui->treeWidget->setEditTriggers(QTreeWidget::NoEditTriggers);
    ui->actionSave->setDisabled(true);
    ui->menuEdit->setDisabled(true);
    ui->pushButton->setDisabled(true);
    const QString name("Provant Simulator");
    setWindowTitle(name);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    /* modificar variável ambiente de comnfiguração */
    for(int i=0;i<ui->treeWidget->topLevelItemCount();i++)
    {
        if(ui->treeWidget->topLevelItem(i)->text(0)=="Include")
        {
            ui->treeWidget->topLevelItem(i)->child(3)->text(1).replace("model://","");

            char const* tmp = getenv( "GAZEBO_MODEL_PATH" );
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
                        if(file.fileName() == ui->treeWidget->topLevelItem(i)->child(3)->text(1).replace("model://",""))
                        {
                            QString qenv(env.c_str());
                            qenv = qenv + "/" +  ui->treeWidget->topLevelItem(i)->child(3)->text(1).replace("model://","");
                            qenv = qenv + "/config/config.xml";
                            qDebug() << qenv;
                            std::string command("export TILT_CONFIG="+qenv.toStdString());
                            std::system(command.c_str());
                        }
                    }
                }
            }
        }
    }

    if(istemplate)
    {
        if(SaveAs())
        {
            roslaunch::WriteNew(QString::fromStdString(mundo.actualword->Filename));
            std::system("roslaunch simulation_elements gazebo.launch");
        }
    }
    else
    {
        //Save
        Save();
        roslaunch::WriteNew(QString::fromStdString(mundo.actualword->Filename));
        std::system("roslaunch simulation_elements gazebo.launch");
    }
}

void MainWindow::on_actionNew_triggered()
{
    QString filename = QFileDialog::getOpenFileName(this
                                                    ,tr("New World")
                                                    , "/home/macro/catkin_ws/src/provant_simulator/source/Database"
                                                    , tr("Template Files (*.tpl)"));
    if(filename.isEmpty()) return;

    QString dir;
    QStringList splitvector;
    QRegExp rx("\\/");
    splitvector = filename.split(rx);
    splitvector.removeLast();
    // filter
    foreach (const QString &str, splitvector)
    {
        if (str.contains(" ")||str.size()==0)
        {
            //faz nada
        }
        else
        {
            dir = dir+"/"+str;
        }
    }


    QString imagefile(dir+"/imagem.gif");
    QFile ff(imagefile);
    QFileInfo fileInfo(ff);
    if (fileInfo.exists())
    {
        QGraphicsScene * scene = new QGraphicsScene(ui->graphicsView_2);
        QImage image(imagefile);
        scene->addPixmap(QPixmap::fromImage(image));
        ui->graphicsView_2->setScene(scene);
        ui->graphicsView_2->show();
    }

    ui->treeWidget->clear();
    mundo.getFirst(filename.toStdString(),ui->treeWidget);
    ui->actionSave->setEnabled(true);
    ui->menuEdit->setEnabled(true);

    ui->actionSave->setEnabled(true);
    ui->menuEdit->setEnabled(true);
    ui->pushButton->setEnabled(true);

    istemplate = true;
}


void MainWindow::on_treeWidget_itemDoubleClicked(QTreeWidgetItem *item, int column)
{
    if (column == 1 && item->text(0) != "uri") {
            ui->treeWidget->editItem(item, column);
        }
    if(item->text(0) == "uri")
    {
        char const* tmp = getenv("GAZEBO_MODEL_PATH");
        if ( tmp == NULL ) {
            qDebug() << "Problemas com variavel de ambiente ";
        } else {
            std::string env(tmp);
            QDir dir(env.c_str());
            QStringList list;
            list = item->text(1).split("//");
            if (!dir.cd(QString::fromStdString(list.at(1).toStdString()))) // "/tmp"
            {
                // não faz nada
            }
            else
            {
                std::string localmodel = env+"/"+list.at(1).toStdString()+"/robot/model.sdf";
                std::string localconfig = env+"/"+list.at(1).toStdString()+"/config/config.xml";
                Dialog modelsetup;
                modelsetup.setModel(localmodel,localconfig);
                modelsetup.setModal(true);
                modelsetup.exec();
            }
        }
    }
}



void MainWindow::on_treeWidget_doubleClicked(const QModelIndex &index)
{

}

void MainWindow::on_treeWidget_itemClicked(QTreeWidgetItem *item, int column)
{
    if(item->text(0) == "uri")
    {
        char const* tmp = getenv( "GAZEBO_MODEL_PATH" );
        if ( tmp == NULL ) {
            qDebug() << "Problemas com variavel de ambiente ";
        } else {
            std::string env(tmp);
            QDir dir(env.c_str());
            QStringList list;
            list = item->text(1).split("//");
            if (!dir.cd(QString::fromStdString(list.at(1).toStdString()))) // "/tmp"
            {
                QGraphicsScene * scene = new QGraphicsScene(ui->graphicsView);
                ui->graphicsView->setScene(scene);
                ui->graphicsView->show();
            }
            else
            {
                QString imagefile(QString::fromStdString(env)+"/"+list.at(1)+"/imagem.gif");
                QFile ff(imagefile);
                QFileInfo fileInfo(ff);
                if (fileInfo.exists())
                {
                    QGraphicsScene * scene = new QGraphicsScene(ui->graphicsView);
                    QImage image(imagefile);
                    scene->addPixmap(QPixmap::fromImage(image));
                    ui->graphicsView->setScene(scene);
                    ui->graphicsView->show();
                }
            }
        }
    }
}

void MainWindow::on_actionOpen_triggered()
{
    QString filename = QFileDialog::getOpenFileName(this
                                                    ,tr("Open World")
                                                    , "/home/macro/catkin_ws/src/provant_simulator/source/Database"
                                                    , tr("World Files (*.world)"));
    if(filename.isEmpty()) return;
    QString dir;
    QStringList splitvector;
    QRegExp rx("\\/");
    splitvector = filename.split(rx);
    splitvector.removeLast();
    // filter
    foreach (const QString &str, splitvector)
    {
        if (str.contains(" ")||str.size()==0)
        {
            //faz nada
        }
        else
        {
            dir = dir+"/"+str;
        }
    }


    QString imagefile(dir+"/imagem.gif");
    QFile ff(imagefile);
    QFileInfo fileInfo(ff);
    if (fileInfo.exists())
    {
        QGraphicsScene * scene = new QGraphicsScene(ui->graphicsView_2);
        QImage image(imagefile);
        scene->addPixmap(QPixmap::fromImage(image));
        ui->graphicsView_2->setScene(scene);
        ui->graphicsView_2->show();
    }
    ui->treeWidget->clear();
    mundo.getFirst(filename.toStdString(),ui->treeWidget);
    ui->actionSave->setEnabled(true);
    ui->menuEdit->setEnabled(true);

    ui->actionSave->setEnabled(true);
    ui->menuEdit->setEnabled(true);
    ui->pushButton->setEnabled(true);
    istemplate = false;
}


bool MainWindow::SaveAs()
{
    QString sf;
    QString filename = QFileDialog::getSaveFileName(this,
                                 tr("Save Word"),
                                 "/home/macro/catkin_ws/src/provant_simulator/source/Database",
                                 tr("World Files (*.world)"),
                                 &sf);
    if(filename.isEmpty()) return false;

    QFileInfo f( filename );
    if (f.suffix().isEmpty())
    {
        // http://www.qtcentre.org/forum/f-qt-programming-2/t-qfiledialoggetsavefilename-default-extension-8503.html
        filename += ".world";
    }
    else
    {
        if(f.suffix()!=".world")filename += ".world";
    }
    mundo.actualword->Filename = filename.toStdString();
    Save();
    istemplate = false;
    return true;
}

void MainWindow::on_actionSave_triggered()
{
    SaveAs();
}

void MainWindow::on_actionExit_triggered()
{
    this->close();
}

void MainWindow::Save()
{
    mundo.Write(ui->treeWidget);
}

void MainWindow::on_actionEdit_triggered()
{

}

void MainWindow::on_actionNew_2_triggered()
{
    Dialognewmodel newmodel(ui,this);
    newmodel.newModel();
    newmodel.setModal(true);
    newmodel.exec();
}
