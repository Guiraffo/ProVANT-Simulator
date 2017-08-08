#include "world.h"
#include "QMessageBox"

world::world()
{

}

void world::getTemplate(std::string template_filename,QTreeWidget* root)
{
    lastword = actualword;
    templateword = new WorldFile(template_filename);
    actualword = templateword;
    if(actualword->Read())ToTreeWidget(root);
}
void world::getLast(QTreeWidget* root)
{
    actualword = lastword;
    ToTreeWidget(root);
}
void world::getFirst(std::string first_filename,QTreeWidget*root)
{
    lastword = actualword;
    firstword = new WorldFile(first_filename);
    actualword = firstword;
    if(actualword->Read())ToTreeWidget(root);
}
void world::getActual(QTreeWidget* root)
{
    ToTreeWidget(root);
}
void world::Write(QTreeWidget* root)
{
    WorldFile* newMundo;
    std::vector<Include_DA> newList;
    Include_DA* include;
    newMundo = new WorldFile(this->actualword->Filename);
    newMundo->sdfVersion = actualword->sdfVersion;
    for(int i = 0; i< root->topLevelItemCount();i++)
    {
        QTreeWidgetItem* item = root->topLevelItem(i);
        if(item->text(0)=="Physics")
        {
            physics_DA physics;
            if (item->child(0)->text(1).toStdString()!="ode"
                &&item->child(0)->text(1).toStdString()!="bullet"
                &&item->child(0)->text(1).toStdString()!="simbody"
                &&item->child(0)->text(1).toStdString()!="dart")
            {
                    QMessageBox messageBox;
                    messageBox.about(0,"Error","The type of Engine is wrong!");
                    messageBox.setFixedSize(500,200);
                    return;
            }

            physics.SetType(item->child(0)->text(1).toStdString());
            physics.SetStep(item->child(1)->text(1).toStdString());
            bool ok;
            item->child(2)->text(1).toFloat(&ok);
            if(!ok){
                QMessageBox messageBox;
                messageBox.about(0,"Error","An error has occured verify the parameters of the World!");
                messageBox.setFixedSize(500,200);
                return;
            }
            physics.SetRealTimeFactor(item->child(2)->text(1).toStdString());
            physics.SetRealTimeUpdaterate(item->child(3)->text(1).toStdString());
            newMundo->SetPhysics(physics);
        }
        if(item->text(0)=="Gravity")
        {
            gravity_DA gravity;
            bool ok,ok2,ok3;
            std::string gravString = item->child(0)->text(1).toStdString()
                                    +" "+
                                     item->child(1)->text(1).toStdString()
                                    +" "+
                                     item->child(2)->text(1).toStdString();

            item->child(0)->text(1).toFloat(&ok);
            item->child(1)->text(1).toFloat(&ok2);
            item->child(2)->text(1).toFloat(&ok3);
            if(!ok||!ok2||!ok3){
                 QMessageBox messageBox;
                 messageBox.about(0,"Error","An error has occured verify the parameters of the World!");
                 messageBox.setFixedSize(500,200);
                 return;
            }
            gravity.SetGravity(gravString);
            newMundo->SetGravity(gravity);
        }
        if(item->text(0)=="Plugin")
        {
            WorldPlugin plugin;
            plugin.SetFilename(item->child(1)->text(1).toStdString());
            plugin.SetName(item->child(0)->text(1).toStdString());
            newMundo->SetPlugin(plugin);
        }

        if(item->text(0)=="Include")
        {
            include = new Include_DA;
            include->SetName(item->child(0)->text(1).toStdString());
            if(item->child(1)->childCount()!=0)
            {
                bool ok,ok2,ok3,ok4,ok5,ok6;
               item->child(1)->child(0)->text(1).toFloat(&ok);
               item->child(1)->child(1)->text(1).toFloat(&ok2);
               item->child(1)->child(2)->text(1).toFloat(&ok3);
               item->child(1)->child(3)->text(1).toFloat(&ok4);
               item->child(1)->child(4)->text(1).toFloat(&ok5);
               item->child(1)->child(5)->text(1).toFloat(&ok6);

               if(!ok||!ok2||!ok3||!ok4||!ok5||!ok6)
               {
                   QMessageBox messageBox;
                   messageBox.about(0,"Error","An error has occured verify the parameters of the World!");
                   messageBox.setFixedSize(500,200);
                   return;
               }

            std::string includeString = item->child(1)->child(0)->text(1).toStdString()+" "+
                                        item->child(1)->child(1)->text(1).toStdString()+" "+
                                        item->child(1)->child(2)->text(1).toStdString()+" "+
                                        item->child(1)->child(3)->text(1).toStdString()+" "+
                                        item->child(1)->child(4)->text(1).toStdString()+" "+
                                        item->child(1)->child(5)->text(1).toStdString();
            include->SetPose(includeString);
            }
            include->SetIsStatic(item->child(2)->text(1).toStdString());
            include->SetUri(item->child(3)->text(1).toStdString());
            newList.push_back(*include);
            newMundo->AddInclude(newList);
        }
    }
    newMundo->Filename = this->actualword->Filename;
    this->actualword = newMundo;
    this->actualword->Write();
}
void world::ToTreeWidget(QTreeWidget* root)
{
    QTreeWidgetItem* elementPose;
    QTreeWidgetItem* edit;

    root->clear();
    QTreeWidgetItem* element;
    //
    element = TreeItens::AddRoot("Gravity","",root);
    QStringList splitvector;
    QString vector;
    vector = QString::fromStdString(actualword->GetGravity().GetGravity());
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
    edit = TreeItens::AddChild(element,"X",result.at(0).toStdString());
    edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
    edit = TreeItens::AddChild(element,"Y",result.at(1).toStdString());
    edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
    edit = TreeItens::AddChild(element,"Z",result.at(2).toStdString());
    edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
    //
    element = TreeItens::AddRoot("Physics","",root);
    edit = TreeItens::AddChild(element,"Type",actualword->GetPhysics().GetType());
    edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
    TreeItens::AddChild(element,"Step time",actualword->GetPhysics().GetStep());
    edit = TreeItens::AddChild(element,"Real time factor",actualword->GetPhysics().GetRealTimeFactor());
    //edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
    edit = TreeItens::AddChild(element,"Real time update rate",actualword->GetPhysics().GetRealTimeUpdaterate());
    //edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
    //
    element = TreeItens::AddRoot("Plugin","",root);
    TreeItens::AddChild(element,"name",actualword->GetPlugin().GetName());
    TreeItens::AddChild(element,"filename",actualword->GetPlugin().GetFilename());
    //
    std::vector<Include_DA> list = actualword->GetListsInclude();
    for(uint i = 0; i< list.size();i++)
    {
        element = TreeItens::AddRoot("Include","",root);
        edit = TreeItens::AddChild(element,"name",list.at(i).GetName());
        edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
        elementPose = TreeItens::AddChild(element,"pose","");
        this->splitvector(list.at(i).GetPose(),elementPose);
        TreeItens::AddChild(element,"isStatic",list.at(i).GetIsStatic());
        TreeItens::AddChild(element,"uri",list.at(i).GetUri());

        char const* tmp = getenv( "GAZEBO_MODEL_PATH" );
        if ( tmp == NULL )
        {
                qDebug() << "Problemas com variavel de ambiente ";
        }
        else
        {
                std::string env(tmp);
                QDir dir(env.c_str());
                QStringList resultlist;
                resultlist  = QString::fromStdString(list.at(i).GetUri()).split("//");

                if (dir.cd(QString::fromStdString(resultlist.at(1).toStdString()))) // "/tmp"
                {
                       QString command(QString::fromStdString(env)
                                          +"/"
                                           +resultlist.at(1)
                                           +"/config/config.xml");
                       setenv("TILT_CONFIG",command.toStdString().c_str(),1);
                }

          }
    }
}

void world::splitvector(std::string data,QTreeWidgetItem* Element)
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