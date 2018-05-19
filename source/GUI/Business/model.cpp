#include "model.h"
#include"QListWidgetItem"

Model::Model()
{

}

void Model::getTemplate(std::string template_filename,QTreeWidget* root)
{
    //lastmodel = actualmodel;
    //templatemodel = new ModelFile(template_filename);
    //actualmodel = templatemodel;
    //actualmodel->Read();
    //ToTreeWidget(root);
}
void Model::getLast(QTreeWidget* root)
{
    //actualmodel = lastmodel;
    //ToTreeWidget(root);
}
void Model::getFirst(std::string first_filename,QTreeWidget*root)
{
    //lastmodel = actualmodel;
    model = new ModelFile(first_filename);
    //actualmodel = firstmodel;
    model->Read();
    ToTreeWidget(root);
}
void Model::getActual(QTreeWidget* root)
{
    //ToTreeWidget(root);
}
void Model::Write(QTreeWidget* root)
{
    /*ModelFile* newModel;
    std::vector<link_DA> newListlink;
    std::vector<joint_DA> newListjoint;
    std::vector<ModelPlugin> newListplugin;

    link_DA* link;
    joint_DA* joint;
    ModelPlugin* plugin;
    sensor* newsensor;

    newModel = new ModelFile(this->actualmodel->filename);
    newModel->filename = this->actualmodel->filename;
    newModel->sdfVersion = this->actualmodel->sdfVersion;

    for(int i = 0; i< root->topLevelItemCount();i++)
    {
        QTreeWidgetItem* item = root->topLevelItem(i);
        if(item->text(0)=="Link")
        {
            link = new link_DA;
            link->name = item->child(0)->text(1).toStdString();
            std::string posevector = item->child(1)->child(0)->text(1).toStdString()+ " "+
                                     item->child(1)->child(1)->text(1).toStdString()+ " "+
                                     item->child(1)->child(2)->text(1).toStdString()+ " "+
                                     item->child(1)->child(3)->text(1).toStdString()+ " "+
                                     item->child(1)->child(4)->text(1).toStdString()+ " "+
                                     item->child(1)->child(5)->text(1).toStdString();
            link->pose = posevector;
            link->inertialValues.SetMass(item->child(2)->child(0)->text(1).toStdString());

            posevector = item->child(2)->child(1)->child(0)->text(1).toStdString()+ " "+
                         item->child(2)->child(1)->child(1)->text(1).toStdString()+ " "+
                         item->child(2)->child(1)->child(2)->text(1).toStdString()+ " "+
                         item->child(2)->child(1)->child(3)->text(1).toStdString()+ " "+
                         item->child(2)->child(1)->child(4)->text(1).toStdString()+ " "+
                         item->child(2)->child(1)->child(5)->text(1).toStdString();
            link->inertialValues.SetPose(posevector);
            link->inertialValues.SetIxx(item->child(2)->child(2)->child(0)->text(1).toStdString());
            link->inertialValues.SetIxy(item->child(2)->child(2)->child(1)->text(1).toStdString());
            link->inertialValues.SetIxz(item->child(2)->child(2)->child(2)->text(1).toStdString());
            link->inertialValues.SetIyy(item->child(2)->child(2)->child(3)->text(1).toStdString());
            link->inertialValues.SetIyz(item->child(2)->child(2)->child(4)->text(1).toStdString());
            link->inertialValues.SetIzz(item->child(2)->child(2)->child(5)->text(1).toStdString());

            link->collision.name = item->child(3)->child(0)->text(1).toStdString();

            if(item->child(3)->child(1)->childCount()>1)
            {
                posevector = item->child(3)->child(1)->child(0)->text(1).toStdString()+ " "+
                         item->child(3)->child(1)->child(1)->text(1).toStdString()+ " "+
                         item->child(3)->child(1)->child(2)->text(1).toStdString()+ " "+
                         item->child(3)->child(1)->child(3)->text(1).toStdString()+ " "+
                         item->child(3)->child(1)->child(4)->text(1).toStdString()+ " "+
                         item->child(3)->child(1)->child(5)->text(1).toStdString();
                link->collision.pose = posevector;
            }

            std::vector<std::string> list;
            list.push_back(item->child(3)->child(2)->child(0)->text(1).toStdString());
            link->collision.geometry->SetValue(list);

            link->visual.name = item->child(4)->child(0)->text(1).toStdString();

            if(item->child(3)->child(1)->childCount()>1)
            {
            posevector = item->child(4)->child(1)->child(0)->text(1).toStdString()+ " "+
                         item->child(4)->child(1)->child(1)->text(1).toStdString()+ " "+
                         item->child(4)->child(1)->child(2)->text(1).toStdString()+ " "+
                         item->child(4)->child(1)->child(3)->text(1).toStdString()+ " "+
                         item->child(4)->child(1)->child(4)->text(1).toStdString()+ " "+
                         item->child(4)->child(1)->child(5)->text(1).toStdString();

            link->visual.pose = posevector;
            }
            std::vector<std::string> list2;
            list2.push_back(item->child(4)->child(2)->child(0)->text(1).toStdString());
            link->visual.geometry->SetValue(list2);

            link->visual.material.SetAmbient(item->child(4)->child(3)->child(0)->text(1).toStdString());
            link->visual.material.SetDiffuse(item->child(4)->child(3)->child(1)->text(1).toStdString());
            link->visual.material.SetEmissive(item->child(4)->child(3)->child(2)->text(1).toStdString());
            link->visual.material.SetSpecular(item->child(4)->child(3)->child(3)->text(1).toStdString());

            //link->print();
            newModel->model.AddLink(*link);
        }
        if(item->text(0)=="Joint")
        {
            joint = new joint_DA;

            joint->name = item->child(0)->text(1).toStdString();
            joint->type = item->child(1)->text(1).toStdString();
            joint->parent = item->child(2)->text(1).toStdString();
            joint->child = item->child(3)->text(1).toStdString();

            std::string posevector = item->child(4)->child(0)->text(1).toStdString()+ " "+
                         item->child(4)->child(1)->text(1).toStdString()+ " "+
                         item->child(4)->child(2)->text(1).toStdString()+ " "+
                         item->child(4)->child(3)->text(1).toStdString()+ " "+
                         item->child(4)->child(4)->text(1).toStdString()+ " "+
                         item->child(4)->child(5)->text(1).toStdString();
            joint->pose = posevector;

            joint->one->xyz = item->child(5)->child(0)->text(1).toStdString();
            joint->one->damping = item->child(5)->child(1)->text(1).toStdString();
            joint->one->friction = item->child(5)->child(2)->text(1).toStdString();
            joint->one->upper = item->child(5)->child(3)->text(1).toStdString();
            joint->one->lower = item->child(5)->child(4)->text(1).toStdString();
            joint->one->effort = item->child(5)->child(5)->text(1).toStdString();
            joint->one->velocity = item->child(5)->child(6)->text(1).toStdString();



            if(item->childCount()>6)
            {
                joint->two->xyz = item->child(6)->child(0)->text(1).toStdString();
                joint->two->damping = item->child(6)->child(1)->text(1).toStdString();
                joint->two->friction = item->child(6)->child(2)->text(1).toStdString();
                joint->two->upper = item->child(6)->child(3)->text(1).toStdString();
                joint->two->lower = item->child(6)->child(4)->text(1).toStdString();
                joint->two->effort = item->child(6)->child(5)->text(1).toStdString();
                joint->two->velocity = item->child(6)->child(6)->text(1).toStdString();
            }
            else joint->two = NULL;

             newModel->model.AddJoint(*joint);
        }
        if(item->text(0)=="Plugin")
        {
             plugin = new ModelPlugin;

             plugin->SetName(item->child(0)->text(1).toStdString());
             plugin->SetFilename(item->child(1)->text(1).toStdString());
             for(int i = 2; i < item->childCount();i++)
             {
                 plugin->parameters.push_back(item->child(i)->text(0).toStdString());
                 plugin->values.push_back(item->child(i)->text(1).toStdString());
             }
             newModel->model.AddPlugin(*plugin);
        }
        if(item->text(0)=="Sensor")
        {
             newsensor = new sensor;
             newsensor->name = item->child(0)->text(1);
             newsensor->type = item->child(1)->text(1);
             newsensor->always_on = item->child(2)->text(1);
             newsensor->visualize = item->child(3)->text(1);
             newsensor->link = item->child(4)->text(1).toInt();
             std::string posevector = item->child(5)->child(0)->text(1).toStdString()+ " "+
                          item->child(5)->child(1)->text(1).toStdString()+ " "+
                          item->child(5)->child(2)->text(1).toStdString()+ " "+
                          item->child(5)->child(3)->text(1).toStdString()+ " "+
                          item->child(5)->child(4)->text(1).toStdString()+ " "+
                          item->child(5)->child(5)->text(1).toStdString();
             QString buff(posevector.c_str());
             newsensor->pose = buff;
             newsensor->update_rate = item->child(6)->text(1);
             newsensor->topic = item->child(7)->text(1);
             if(newsensor->name == "sonar")
             {
                newsensor->max =  item->child(8)->text(1);
                newsensor->min =  item->child(9)->text(1);
                newsensor->radius =  item->child(10)->text(1);
             }
             if(newsensor->name == "imu")
             {
                newsensor->accel_x.type = item->child(8)->child(0)->child(0)->text(1);
                newsensor->accel_x.mean = item->child(8)->child(0)->child(1)->text(1);
                newsensor->accel_x.stddev = item->child(8)->child(0)->child(2)->text(1);
                newsensor->accel_x.bias_mean = item->child(8)->child(0)->child(3)->text(1);
                newsensor->accel_x.bias_stddev = item->child(8)->child(0)->child(4)->text(1);
                newsensor->accel_x.precision = item->child(8)->child(0)->child(5)->text(1);

                newsensor->accel_y.type = item->child(8)->child(1)->child(0)->text(1);
                newsensor->accel_y.mean = item->child(8)->child(1)->child(1)->text(1);
                newsensor->accel_y.stddev = item->child(8)->child(1)->child(2)->text(1);
                newsensor->accel_y.bias_mean = item->child(8)->child(1)->child(3)->text(1);
                newsensor->accel_y.bias_stddev = item->child(8)->child(1)->child(4)->text(1);
                newsensor->accel_y.precision = item->child(8)->child(1)->child(5)->text(1);

                newsensor->accel_z.type = item->child(8)->child(2)->child(0)->text(1);
                newsensor->accel_z.mean = item->child(8)->child(2)->child(1)->text(1);
                newsensor->accel_z.stddev = item->child(8)->child(2)->child(2)->text(1);
                newsensor->accel_z.bias_mean = item->child(8)->child(2)->child(3)->text(1);
                newsensor->accel_z.bias_stddev = item->child(8)->child(2)->child(4)->text(1);
                newsensor->accel_z.precision = item->child(8)->child(2)->child(5)->text(1);

                newsensor->ang_x.type = item->child(9)->child(0)->child(0)->text(1);
                newsensor->ang_x.mean = item->child(9)->child(0)->child(1)->text(1);
                newsensor->ang_x.stddev = item->child(9)->child(0)->child(2)->text(1);
                newsensor->ang_x.bias_mean = item->child(9)->child(0)->child(3)->text(1);
                newsensor->ang_x.bias_stddev = item->child(9)->child(0)->child(4)->text(1);
                newsensor->ang_x.precision = item->child(9)->child(0)->child(5)->text(1);

                newsensor->ang_y.type = item->child(9)->child(1)->child(0)->text(1);
                newsensor->ang_y.mean = item->child(9)->child(1)->child(1)->text(1);
                newsensor->ang_y.stddev = item->child(9)->child(1)->child(2)->text(1);
                newsensor->ang_y.bias_mean = item->child(9)->child(1)->child(3)->text(1);
                newsensor->ang_y.bias_stddev = item->child(9)->child(1)->child(4)->text(1);
                newsensor->ang_y.precision = item->child(9)->child(1)->child(5)->text(1);

                newsensor->ang_z.type = item->child(9)->child(2)->child(0)->text(1);
                newsensor->ang_z.mean = item->child(9)->child(2)->child(1)->text(1);
                newsensor->ang_z.stddev = item->child(9)->child(2)->child(2)->text(1);
                newsensor->ang_z.bias_mean = item->child(9)->child(2)->child(3)->text(1);
                newsensor->ang_z.bias_stddev = item->child(9)->child(2)->child(4)->text(1);
                newsensor->ang_z.precision = item->child(9)->child(2)->child(5)->text(1);
             }
             if(newsensor->name == "gps")
             {
                 newsensor->pos_horizontal.type = item->child(8)->child(0)->child(0)->text(1);
                 newsensor->pos_horizontal.mean = item->child(8)->child(0)->child(1)->text(1);
                 newsensor->pos_horizontal.stddev = item->child(8)->child(0)->child(2)->text(1);
                 newsensor->pos_horizontal.bias_mean = item->child(8)->child(0)->child(3)->text(1);
                 newsensor->pos_horizontal.bias_stddev = item->child(8)->child(0)->child(4)->text(1);
                 newsensor->pos_horizontal.precision = item->child(8)->child(0)->child(5)->text(1);

                 newsensor->pos_vertical.type = item->child(8)->child(1)->child(0)->text(1);
                 newsensor->pos_vertical.mean = item->child(8)->child(1)->child(1)->text(1);
                 newsensor->pos_vertical.stddev = item->child(8)->child(1)->child(2)->text(1);
                 newsensor->pos_vertical.bias_mean = item->child(8)->child(1)->child(3)->text(1);
                 newsensor->pos_vertical.bias_stddev = item->child(8)->child(1)->child(4)->text(1);
                 newsensor->pos_vertical.precision = item->child(8)->child(1)->child(5)->text(1);

                 newsensor->vel_horizontal.type = item->child(9)->child(0)->child(0)->text(1);
                 newsensor->vel_horizontal.mean = item->child(9)->child(0)->child(1)->text(1);
                 newsensor->vel_horizontal.stddev = item->child(9)->child(0)->child(2)->text(1);
                 newsensor->vel_horizontal.bias_mean = item->child(9)->child(0)->child(3)->text(1);
                 newsensor->vel_horizontal.bias_stddev = item->child(9)->child(0)->child(4)->text(1);
                 newsensor->vel_horizontal.precision = item->child(9)->child(0)->child(5)->text(1);

                 newsensor->vel_vertical.type = item->child(9)->child(1)->child(0)->text(1);
                 newsensor->vel_vertical.mean = item->child(9)->child(1)->child(1)->text(1);
                 newsensor->vel_vertical.stddev = item->child(9)->child(1)->child(2)->text(1);
                 newsensor->vel_vertical.bias_mean = item->child(9)->child(1)->child(3)->text(1);
                 newsensor->vel_vertical.bias_stddev = item->child(9)->child(1)->child(4)->text(1);
                 newsensor->vel_vertical.precision = item->child(9)->child(1)->child(5)->text(1);
             }
             if(newsensor->name == "magnetometer")
             {
                 newsensor->x.type = item->child(8)->child(0)->text(1);
                 newsensor->x.mean = item->child(8)->child(1)->text(1);
                 newsensor->x.stddev = item->child(8)->child(2)->text(1);
                 newsensor->x.bias_mean = item->child(8)->child(3)->text(1);
                 newsensor->x.bias_stddev = item->child(8)->child(4)->text(1);
                 newsensor->x.precision = item->child(8)->child(5)->text(1);

                 newsensor->y.type = item->child(9)->child(0)->text(1);
                 newsensor->y.mean = item->child(9)->child(1)->text(1);
                 newsensor->y.stddev = item->child(9)->child(2)->text(1);
                 newsensor->y.bias_mean = item->child(9)->child(3)->text(1);
                 newsensor->y.bias_stddev = item->child(9)->child(4)->text(1);
                 newsensor->y.precision = item->child(9)->child(5)->text(1);

                 newsensor->z.type = item->child(10)->child(0)->text(1);
                 newsensor->z.mean = item->child(10)->child(1)->text(1);
                 newsensor->z.stddev = item->child(10)->child(2)->text(1);
                 newsensor->z.bias_mean = item->child(10)->child(3)->text(1);
                 newsensor->z.bias_stddev = item->child(10)->child(4)->text(1);
                 newsensor->z.precision = item->child(10)->child(5)->text(1);

             }
             newModel->model.ListsSensors.push_back(*newsensor);
        }
    }

    actualmodel = newModel;
    //actualmodel->print();*/

}
void Model::ToTreeWidget(QTreeWidget* root)
{
    root->clear();
    QTreeWidgetItem* edit;
    QTreeWidgetItem* element;
    QTreeWidgetItem* poseElement;

    element = TreeItens::AddRoot("Filename",model->filename.c_str(),root);
    std::vector<link_DA> links = model->model.GetListsLinks();
    for(uint i = 0; i < links.size(); i++)
    {
        QTreeWidgetItem* elementLink = TreeItens::AddRoot("Link","",root);
        TreeItens::AddChild(elementLink,"Name",links.at(i).name.c_str());
        poseElement = TreeItens::AddChild(elementLink,"Pose","");
        splitvector(links.at(i).pose,poseElement,false);
        element = TreeItens::AddChild(elementLink,"Inertial","");
        TreeItens::AddChild(element,"Mass [kg]",links.at(i).inertialValues.GetMass().c_str());
        poseElement = TreeItens::AddChild(element,"Pose [m]","");
        splitvector(links.at(i).inertialValues.GetPose(),poseElement,true);
        element = TreeItens::AddChild(element,"Inertia [kg.m^2]","");
        TreeItens::AddChild(element,"ixx",links.at(i).inertialValues.GetIxx().c_str());
        TreeItens::AddChild(element,"ixy",links.at(i).inertialValues.GetIxy().c_str());
        TreeItens::AddChild(element,"ixz",links.at(i).inertialValues.GetIxz().c_str());
        TreeItens::AddChild(element,"iyy",links.at(i).inertialValues.GetIyy().c_str());
        TreeItens::AddChild(element,"iyz",links.at(i).inertialValues.GetIyz().c_str());
        TreeItens::AddChild(element,"izz",links.at(i).inertialValues.GetIzz().c_str());
        element = TreeItens::AddChild(elementLink,"Collision","");
        TreeItens::AddChild(element,"Name",links.at(i).collision.name);
        poseElement = TreeItens::AddChild(element,"Pose","");
        splitvector(links.at(i).collision.pose,poseElement,false);
        element = TreeItens::AddChild(element,"Geometry","");
        element = TreeItens::AddChild(element,"Uri",links.at(i).collision.geometry->GetValues().at(0));
        QTreeWidgetItem* elementVisual = TreeItens::AddChild(elementLink,"Visual","");
        TreeItens::AddChild(elementVisual,"Name",links.at(i).visual.name);
        poseElement = TreeItens::AddChild(elementVisual,"Pose","");
        splitvector(links.at(i).visual.pose,poseElement,false);
        element = TreeItens::AddChild(elementVisual,"Geometry","");
        TreeItens::AddChild(element,"Uri",links.at(i).visual.geometry->GetValues().at(0));
        element = TreeItens::AddChild(elementVisual,"Material","");
        TreeItens::AddChild(element,"Ambient",links.at(i).visual.material.GetAmbient().c_str());
        TreeItens::AddChild(element,"Diffuse",links.at(i).visual.material.GetDiffuse().c_str());
        TreeItens::AddChild(element,"Emissive",links.at(i).visual.material.GetEmissive().c_str());
        TreeItens::AddChild(element,"Specular",links.at(i).visual.material.GetSpecular().c_str());
    }

    std::vector<joint_DA> joints = model->model.GetListsJoints();
    for(uint i = 0; i < joints.size(); i++)
    {
         element = TreeItens::AddRoot("Joint","",root);
         TreeItens::AddChild(element,"Name",joints.at(i).name);
         TreeItens::AddChild(element,"Type",joints.at(i).type);
         TreeItens::AddChild(element,"Parent",joints.at(i).parent);
         TreeItens::AddChild(element,"Child",joints.at(i).child);
         poseElement = TreeItens::AddChild(element,"Pose","");
         splitvector(joints.at(i).pose,poseElement,false);
         QTreeWidgetItem* AxisElement = TreeItens::AddChild(element,"Axis","");
         if(joints.at(i).one!=NULL)
         {
                TreeItens::AddChild(AxisElement,"XYZ",joints.at(i).one->xyz);
                TreeItens::AddChild(AxisElement,"Damping",joints.at(i).one->damping);
                TreeItens::AddChild(AxisElement,"Friction",joints.at(i).one->friction);
                TreeItens::AddChild(AxisElement,"Limit upper",joints.at(i).one->upper);
                TreeItens::AddChild(AxisElement,"Limit lower",joints.at(i).one->lower);
                TreeItens::AddChild(AxisElement,"Limit effort",joints.at(i).one->effort);
                TreeItens::AddChild(AxisElement,"Limit velocity",joints.at(i).one->velocity);
         }

         if(joints.at(i).two!=NULL)
         {
             QTreeWidgetItem* Axis2Element = TreeItens::AddChild(element,"Axis2","");
             TreeItens::AddChild(Axis2Element,"XYZ",joints.at(i).two->xyz);
             TreeItens::AddChild(Axis2Element,"Damping",joints.at(i).two->damping);
             TreeItens::AddChild(Axis2Element,"Friction",joints.at(i).two->friction);
             TreeItens::AddChild(Axis2Element,"Limit upper",joints.at(i).two->upper);
             TreeItens::AddChild(Axis2Element,"Limit lower",joints.at(i).two->lower);
             TreeItens::AddChild(Axis2Element,"Limit effort",joints.at(i).two->effort);
             TreeItens::AddChild(Axis2Element,"Limit velocity",joints.at(i).two->velocity);
         }
    }
    std::vector<ModelPlugin> plugins = model->model.GetListsPlugins();
    for(uint i = 0; i < plugins.size(); i++)
    {
        element = TreeItens::AddRoot("Plugin","",root);
        TreeItens::AddChild(element,"Name",plugins.at(i).GetName());
        TreeItens::AddChild(element,"Filename",plugins.at(i).GetFilename());
        for(uint j = 0; j < plugins.at(i).parameters.size(); j++)
        {
            TreeItens::AddChild(element,plugins.at(i).parameters.at(j),plugins.at(i).values.at(j));
        }
    }
    std::vector<sensor> sensors = model->model.ListsSensors;
    for(uint i = 0; i < sensors.size(); i++)
    {
        element = TreeItens::AddRoot("Sensor","",root);
        TreeItens::AddChild(element,"Name",sensors.at(i).name.toStdString());
        TreeItens::AddChild(element,"Type",sensors.at(i).type.toStdString());
        TreeItens::AddChild(element,"Always_on",sensors.at(i).always_on.toStdString());
        TreeItens::AddChild(element,"Visualize",sensors.at(i).visualize.toStdString());
        TreeItens::AddChild(element,"Link",QString::number(sensors.at(i).link).toStdString());
        TreeItens::AddChild(element,"Pose","");
        splitvector(sensors.at(i).pose.toStdString(),edit,true);
        TreeItens::AddChild(element,"Update_rate",sensors.at(i).update_rate.toStdString());
        TreeItens::AddChild(element,"Topic",sensors.at(i).topic.toStdString());

        if(sensors.at(i).name=="sonar")
        {
            TreeItens::AddChild(element,"Max",sensors.at(i).max.toStdString());
            TreeItens::AddChild(element,"Min",sensors.at(i).min.toStdString());
            TreeItens::AddChild(element,"Radius",sensors.at(i).radius.toStdString());
        }
        if(sensors.at(i).name=="imu")
        {
            QTreeWidgetItem* accel = TreeItens::AddChild(element,"Acceleration","");
            QTreeWidgetItem* accelx = TreeItens::AddChild(accel,"x","");
            TreeItens::AddChild(accelx,"type",sensors.at(i).accel_x.type.toStdString());
            TreeItens::AddChild(accelx,"mean",sensors.at(i).accel_x.mean.toStdString());
            TreeItens::AddChild(accelx,"stddev",sensors.at(i).accel_x.stddev.toStdString());
            TreeItens::AddChild(accelx,"bias mean",sensors.at(i).accel_x.bias_mean.toStdString());
            TreeItens::AddChild(accelx,"bias stddev",sensors.at(i).accel_x.bias_stddev.toStdString());
            TreeItens::AddChild(accelx,"precision",sensors.at(i).accel_x.precision.toStdString());

            QTreeWidgetItem* accely = TreeItens::AddChild(accel,"y","");
            TreeItens::AddChild(accely,"type",sensors.at(i).accel_y.type.toStdString());
            TreeItens::AddChild(accely,"mean",sensors.at(i).accel_y.mean.toStdString());
            TreeItens::AddChild(accely,"stddev",sensors.at(i).accel_y.stddev.toStdString());
            TreeItens::AddChild(accely,"bias mean",sensors.at(i).accel_y.bias_mean.toStdString());
            TreeItens::AddChild(accely,"bias stddev",sensors.at(i).accel_y.bias_stddev.toStdString());
            TreeItens::AddChild(accely,"precision",sensors.at(i).accel_y.precision.toStdString());

            QTreeWidgetItem* accelz = TreeItens::AddChild(accel,"z","");
           TreeItens::AddChild(accelz,"type",sensors.at(i).accel_z.type.toStdString());
            TreeItens::AddChild(accelz,"mean",sensors.at(i).accel_z.mean.toStdString());
            TreeItens::AddChild(accelz,"stddev",sensors.at(i).accel_z.stddev.toStdString());
            TreeItens::AddChild(accelz,"bias mean",sensors.at(i).accel_z.bias_mean.toStdString());
            TreeItens::AddChild(accelz,"bias stddev",sensors.at(i).accel_z.bias_stddev.toStdString());
            TreeItens::AddChild(accelz,"precision",sensors.at(i).accel_z.precision.toStdString());

            QTreeWidgetItem* ang = TreeItens::AddChild(element,"Angular","");
            QTreeWidgetItem* angx = TreeItens::AddChild(ang,"x","");
            TreeItens::AddChild(angx,"type",sensors.at(i).ang_x.type.toStdString());
            TreeItens::AddChild(angx,"mean",sensors.at(i).ang_x.mean.toStdString());
            TreeItens::AddChild(angx,"stddev",sensors.at(i).ang_x.stddev.toStdString());
            TreeItens::AddChild(angx,"bias mean",sensors.at(i).ang_x.bias_mean.toStdString());
            TreeItens::AddChild(angx,"bias stddev",sensors.at(i).ang_x.bias_stddev.toStdString());
            TreeItens::AddChild(angx,"precision",sensors.at(i).ang_x.precision.toStdString());

            QTreeWidgetItem* angy = TreeItens::AddChild(ang,"y","");
            TreeItens::AddChild(angy,"type",sensors.at(i).ang_y.type.toStdString());
            TreeItens::AddChild(angy,"mean",sensors.at(i).ang_y.mean.toStdString());
            TreeItens::AddChild(angy,"stddev",sensors.at(i).ang_y.stddev.toStdString());
            TreeItens::AddChild(angy,"bias mean",sensors.at(i).ang_y.bias_mean.toStdString());
            TreeItens::AddChild(angy,"bias stddev",sensors.at(i).ang_y.bias_stddev.toStdString());
            TreeItens::AddChild(angy,"precision",sensors.at(i).ang_y.precision.toStdString());

            QTreeWidgetItem* angz = TreeItens::AddChild(ang,"z","");
            TreeItens::AddChild(angz,"type",sensors.at(i).ang_z.type.toStdString());
            TreeItens::AddChild(angz,"mean",sensors.at(i).ang_z.mean.toStdString());
            TreeItens::AddChild(angz,"stddev",sensors.at(i).ang_z.stddev.toStdString());
            TreeItens::AddChild(angz,"bias mean",sensors.at(i).ang_z.bias_mean.toStdString());
            TreeItens::AddChild(angz,"bias stddev",sensors.at(i).ang_z.bias_stddev.toStdString());
            TreeItens::AddChild(angz,"precision",sensors.at(i).ang_z.precision.toStdString());
        }
        if(sensors.at(i).name=="gps")
        {
            QTreeWidgetItem* pos = TreeItens::AddChild(element,"position sensing","");
            QTreeWidgetItem* pos_hor = TreeItens::AddChild(pos,"horizontal","");
            TreeItens::AddChild(pos_hor,"type",sensors.at(i).pos_horizontal.type.toStdString());
            TreeItens::AddChild(pos_hor,"mean",sensors.at(i).pos_horizontal.mean.toStdString());
            TreeItens::AddChild(pos_hor,"stddev",sensors.at(i).pos_horizontal.stddev.toStdString());
            TreeItens::AddChild(pos_hor,"bias mean",sensors.at(i).pos_horizontal.bias_mean.toStdString());
            TreeItens::AddChild(pos_hor,"bias stddev",sensors.at(i).pos_horizontal.bias_stddev.toStdString());
            TreeItens::AddChild(pos_hor,"precision",sensors.at(i).pos_horizontal.precision.toStdString());

            QTreeWidgetItem* pos_ver = TreeItens::AddChild(pos,"vertical","");
            TreeItens::AddChild(pos_ver,"type",sensors.at(i).pos_vertical.type.toStdString());
            TreeItens::AddChild(pos_ver,"mean",sensors.at(i).pos_vertical.mean.toStdString());
            TreeItens::AddChild(pos_ver,"stddev",sensors.at(i).pos_vertical.stddev.toStdString());
            TreeItens::AddChild(pos_ver,"bias mean",sensors.at(i).pos_vertical.bias_mean.toStdString());
            TreeItens::AddChild(pos_ver,"bias stddev",sensors.at(i).pos_vertical.bias_stddev.toStdString());
            TreeItens::AddChild(pos_ver,"precision",sensors.at(i).pos_vertical.precision.toStdString());

            QTreeWidgetItem* vel = TreeItens::AddChild(element,"velocity sensing","");
            QTreeWidgetItem* vel_hor = TreeItens::AddChild(vel,"horizontal","");
            TreeItens::AddChild(vel_hor,"type",sensors.at(i).vel_horizontal.type.toStdString());
            TreeItens::AddChild(vel_hor,"mean",sensors.at(i).vel_horizontal.mean.toStdString());
            TreeItens::AddChild(vel_hor,"stddev",sensors.at(i).vel_horizontal.stddev.toStdString());
            TreeItens::AddChild(vel_hor,"bias mean",sensors.at(i).vel_horizontal.bias_mean.toStdString());
            TreeItens::AddChild(vel_hor,"bias stddev",sensors.at(i).vel_horizontal.bias_stddev.toStdString());
            TreeItens::AddChild(vel_hor,"precision",sensors.at(i).vel_horizontal.precision.toStdString());

            QTreeWidgetItem* vel_ver = TreeItens::AddChild(vel,"vertical","");
            TreeItens::AddChild(vel_ver,"type",sensors.at(i).vel_vertical.type.toStdString());
            TreeItens::AddChild(vel_ver,"mean",sensors.at(i).vel_vertical.mean.toStdString());
            TreeItens::AddChild(vel_ver,"stddev",sensors.at(i).vel_vertical.stddev.toStdString());
            TreeItens::AddChild(vel_ver,"bias mean",sensors.at(i).vel_vertical.bias_mean.toStdString());
            TreeItens::AddChild(vel_ver,"bias stddev",sensors.at(i).vel_vertical.bias_stddev.toStdString());
            TreeItens::AddChild(vel_ver,"precision",sensors.at(i).vel_vertical.precision.toStdString());
        }
        if(sensors.at(i).name=="magnetometer")
        {
            QTreeWidgetItem* X = TreeItens::AddChild(element,"x","");
            TreeItens::AddChild(X,"type",sensors.at(i).x.type.toStdString());
            TreeItens::AddChild(X,"mean",sensors.at(i).x.mean.toStdString());
            TreeItens::AddChild(X,"stddev",sensors.at(i).x.stddev.toStdString());
            TreeItens::AddChild(X,"bias mean",sensors.at(i).x.bias_mean.toStdString());
            TreeItens::AddChild(X,"bias stddev",sensors.at(i).x.bias_stddev.toStdString());
            TreeItens::AddChild(X,"precision",sensors.at(i).x.precision.toStdString());

            QTreeWidgetItem* Y = TreeItens::AddChild(element,"y","");
            TreeItens::AddChild(Y,"type",sensors.at(i).y.type.toStdString());
            TreeItens::AddChild(Y,"mean",sensors.at(i).y.mean.toStdString());
            TreeItens::AddChild(Y,"stddev",sensors.at(i).y.stddev.toStdString());
            TreeItens::AddChild(Y,"bias mean",sensors.at(i).y.bias_mean.toStdString());
            TreeItens::AddChild(Y,"bias stddev",sensors.at(i).y.bias_stddev.toStdString());
            TreeItens::AddChild(Y,"precision",sensors.at(i).y.precision.toStdString());

            QTreeWidgetItem* Z = TreeItens::AddChild(element,"z","");
            TreeItens::AddChild(Z,"type",sensors.at(i).z.type.toStdString());
            TreeItens::AddChild(Z,"mean",sensors.at(i).z.mean.toStdString());
            TreeItens::AddChild(Z,"stddev",sensors.at(i).z.stddev.toStdString());
            TreeItens::AddChild(Z,"bias mean",sensors.at(i).z.bias_mean.toStdString());
            TreeItens::AddChild(Z,"bias stddev",sensors.at(i).z.bias_stddev.toStdString());
            TreeItens::AddChild(Z,"precision",sensors.at(i).z.precision.toStdString());
        }
    }
}

void Model::splitvector(std::string data,QTreeWidgetItem* Element,bool editable)
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
            else result += str;
        }
        TreeItens::AddChild(Element,"X",result.at(0).toStdString());
        TreeItens::AddChild(Element,"Y",result.at(1).toStdString());
        TreeItens::AddChild(Element,"Z",result.at(2).toStdString());
        TreeItens::AddChild(Element,"Roll",result.at(3).toStdString());
        TreeItens::AddChild(Element,"Pitch",result.at(4).toStdString());
        TreeItens::AddChild(Element,"Yaw",result.at(5).toStdString());
    }
}
