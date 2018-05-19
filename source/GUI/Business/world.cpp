#include "world.h"
#include "QMessageBox"

world::world()
{

}

void world::getTemplate(std::string template_filename,QTreeWidget* root)
{
    /*lastword = actualword;
    templateword = new WorldFile(template_filename);
    actualword = templateword;
    if(actualword->Read());//ToTreeWidget(root);*/
}
void world::getLast(QTreeWidget* root)
{
    /*actualword = lastword;
    ToTreeWidget(root);*/
}
void world::getFirst(std::string first_filename,QTreeWidget*root)
{
    word = new WorldFile(first_filename);
    if(word->Read()) ToTreeWidget(root);
}
void world::getActual(QTreeWidget* root)
{
    //ToTreeWidget(root);
}
void world::Write(QTreeWidget* root)
{

    qDebug() << "world::Write";

    Include_DA* include;
    plugin_DA* plugin;
    qDebug() << "world::Write";
    WorldFile* newMundo;
    newMundo = new WorldFile(this->word->Filename);
    newMundo->sdfVersion = word->sdfVersion;
    qDebug() << "world::Write";
    for(int i = 0; i< root->topLevelItemCount();i++)
    {
        QTreeWidgetItem* item = root->topLevelItem(i);
        if(item->text(0)=="Physics")
        {
             qDebug() << "world::Write";
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
             qDebug() << "world::Write";
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
             qDebug() << "world::Write";
            plugin = new plugin_DA();
            plugin->SetFilename(item->child(1)->text(1).toStdString());
            plugin->SetName(item->child(0)->text(1).toStdString());
            plugin->parameters.push_back(item->child(2)->text(0).toStdString());
            plugin->values.push_back(item->child(2)->text(1).toStdString());
            newMundo->listPlugins.multipleItens.push_back(*plugin);
            qDebug() << "TESTE -------------------------------";
            plugin->print();
        }

        if(item->text(0)=="Include")
        {
            qDebug() << "entrei 2";
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

            newMundo->listIncludes.NewInclude(*include);
            qDebug() << "entrei 2 fim";
        }
    }
    newMundo->Filename = this->word->Filename;
    this->word = newMundo;
    this->word->Write();
}
void world::ToTreeWidget(QTreeWidget* root)
{
    qDebug() << "world::ToTreeWidget";
    QTreeWidgetItem* elementPose;
    QTreeWidgetItem* edit;

    root->clear();
    QTreeWidgetItem* element;
    //
    element = TreeItens::AddRoot("Gravity","",root);
    QStringList splitvector;
    QString vector;
    vector = QString::fromStdString(word->GetGravity().GetGravity());
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
    qDebug() << "aqui 1 ";
    edit = TreeItens::AddChild(element,"X",result.at(0).toStdString());
    edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
    edit = TreeItens::AddChild(element,"Y",result.at(1).toStdString());
    edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
    edit = TreeItens::AddChild(element,"Z",result.at(2).toStdString());
    edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
    //
    element = TreeItens::AddRoot("Physics","",root);
    edit = TreeItens::AddChild(element,"Type",word->GetPhysics().GetType());
    edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
    TreeItens::AddChild(element,"Step time",word->GetPhysics().GetStep());
    edit = TreeItens::AddChild(element,"Real time factor",word->GetPhysics().GetRealTimeFactor());
    //edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
    edit = TreeItens::AddChild(element,"Real time update rate",word->GetPhysics().GetRealTimeUpdaterate());
    //edit->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
    //
     qDebug() << "aqui 2";


    std::vector<plugin_DA> listplugins = word->listPlugins.multipleItens;
    for(uint i = 0; i< listplugins.size();i++)
    {

        element = TreeItens::AddRoot("Plugin","",root);

        for(uint i = 0; i< listplugins.size();i++)
        {
            TreeItens::AddChild(element,"name",listplugins.at(i).GetName());
            TreeItens::AddChild(element,"filename",listplugins.at(i).GetFilename());
            std::vector<std::string> listpluginsparameters = listplugins.at(i).parameters;
            std::vector<std::string> listpluginsvalues = listplugins.at(i).values;
            for(uint j = 0; j< listpluginsparameters.size();j++)
            {
                TreeItens::AddChild(element,listpluginsparameters.at(j),
                                            listpluginsvalues.at(j));
            }
        }
    }
    //
    std::vector<Include_DA> list = word->listIncludes.multipleItens;
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
     qDebug() << "world::ToTreeWidget";
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
