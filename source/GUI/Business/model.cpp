#include "model.h"

#include <vector>

#include <QDebug>

#include "treeitens.h"

Model::Model()
{

}

void Model::getFirst(std::string first_filename, QTreeWidget*root)
{
    getFirst(QString::fromStdString(first_filename), root);
}

void Model::getFirst(const QString &filename, QTreeWidget *treeWidget)
{
    model = new ModelFile(filename);
    model->Read();
    toTreeWidget(treeWidget);
}

void Model::toTreeWidget(QTreeWidget *tree)
{
    tree->clear();
    QTreeWidgetItem* element;
    QTreeWidgetItem* poseElement;

    element = addRoot(QObject::tr("Filename"), model->filename, tree);
    std::vector<link_DA> links = model->model.GetListsLinks();

    for(std::size_t i = 0; i < links.size(); i++)
    {
        QTreeWidgetItem* elementLink = addRoot("Link","",tree);
        addChild(elementLink,
                 QObject::tr("Name"),
                 QString::fromStdString(links.at(i).name));
        // Link pose
        poseElement = addChild(elementLink, QObject::tr("Pose"), "");
        splitVector(QString::fromStdString(links.at(i).pose), poseElement);
        // Link inertial properties
        element = addChild(elementLink, QObject::tr("Inertial"), "");

        // Mass
        addChild(element,
                 QObject::tr("Mass [kg]"),
                 QString::fromStdString(links.at(i).inertialValues.GetMass()));

        poseElement = addChild(element, QObject::tr("Pose [m]"), "");
        splitVector(QString::fromStdString(
                        links.at(i).inertialValues.GetPose()
                    ),
                    poseElement);
        element = addChild(element, QObject::tr("Inertia [kg.m^2]"), "");
        addChild(element,
                 QObject::tr("ixx"),
                 QString::fromStdString(links.at(i).inertialValues.GetIxx()));
        addChild(element,
                 QObject::tr("ixy"),
                 QString::fromStdString(links.at(i).inertialValues.GetIxy()));
        addChild(element,
                 QObject::tr("ixz"),
                 QString::fromStdString(links.at(i).inertialValues.GetIxz()));
        addChild(element,
                 QObject::tr("iyy"),
                 QString::fromStdString(links.at(i).inertialValues.GetIyy()));
        addChild(element,
                 QObject::tr("iyz"),
                 QString::fromStdString(links.at(i).inertialValues.GetIyz()));
        addChild(element,
                 QObject::tr("izz"),
                 QString::fromStdString(links.at(i).inertialValues.GetIzz()));
        // Collision properties
        element = addChild(elementLink, QObject::tr("Collision"), "");
        addChild(element,
                 QObject::tr("Name"),
                 QString::fromStdString(links.at(i).collision.name));

        poseElement = addChild(element, QObject::tr("Pose"), "");
        splitVector(QString::fromStdString(links.at(i).collision.pose),
                    poseElement);
        element = addChild(element, QObject::tr("Geometry"), "");
        element = addChild(element,
                           QObject::tr("URI"),
                           QString::fromStdString(
                               links.at(i).collision.geometry->GetValues().at(0)
                           ));
        QTreeWidgetItem* elementVisual = addChild(elementLink,
                                                  QObject::tr("Visual"),
                                                  "");
        addChild(elementVisual,
                 QObject::tr("Name"),
                 QString::fromStdString(links.at(i).visual.name));
        poseElement = addChild(elementVisual, QObject::tr("Pose"), "");
        splitVector(QString::fromStdString(links.at(i).visual.pose),
                    poseElement);
        element = addChild(elementVisual, QObject::tr("Geometry"), "");
        addChild(element, QObject::tr("URI"),
                 QString::fromStdString(
                     links.at(i).visual.geometry->GetValues().at(0)
                ));

        element = addChild(elementVisual, QObject::tr("Material"), "");
        addChild(element,
                 QObject::tr("Ambient"),
                 QString::fromStdString(
                     links.at(i).visual.material.GetAmbient()
                ));
        addChild(element,
                 QObject::tr("Diffuse"),
                 QString::fromStdString(
                     links.at(i).visual.material.GetDiffuse()
                ));
        addChild(element,
                 QObject::tr("Emissive"),
                 QString::fromStdString(
                     links.at(i).visual.material.GetEmissive()
                ));
        addChild(element,
                 QObject::tr("Specular"),
                 QString::fromStdString(
                     links.at(i).visual.material.GetSpecular()
                ));
    }

    std::vector<joint_DA> joints = model->model.GetListsJoints();
    for(uint i = 0; i < joints.size(); i++)
    {
         element = addRoot(QObject::tr("Joint"), "", tree);
         addChild(element,
                  QObject::tr("Name"),
                  QString::fromStdString(joints.at(i).name));
         addChild(element,
                  QObject::tr("Type"),
                  QString::fromStdString(joints.at(i).type));
         addChild(element,
                  QObject::tr("Parent"),
                  QString::fromStdString(joints.at(i).parent));
         addChild(element,
                  QObject::tr("Child"),
                  QString::fromStdString(joints.at(i).child));

         poseElement = addChild(element, QObject::tr("Pose"), "");
         splitVector(QString::fromStdString(joints.at(i).pose), poseElement);

         QTreeWidgetItem *axisElement = addChild(element,
                                                 QObject::tr("Axis"),
                                                 "");
         if(joints.at(i).one != NULL)
         {
                addChild(axisElement,
                         QObject::tr("XYZ"),
                         QString::fromStdString(joints.at(i).one->xyz));
                addChild(axisElement,
                         QObject::tr("Damping"),
                         QString::fromStdString(joints.at(i).one->damping));
                addChild(axisElement,
                         QObject::tr("Friction"),
                         QString::fromStdString(joints.at(i).one->friction));
                addChild(axisElement,
                         QObject::tr("Upper limit"),
                         QString::fromStdString(joints.at(i).one->upper));
                addChild(axisElement,
                         QObject::tr("Lower limit"),
                         QString::fromStdString(joints.at(i).one->lower));
                addChild(axisElement,
                         QObject::tr("Effort limit"),
                         QString::fromStdString(joints.at(i).one->effort));
                addChild(axisElement,
                         QObject::tr("Limit velocity"),
                         QString::fromStdString(joints.at(i).one->velocity));
         }

         if(joints.at(i).two != NULL)
         {
             QTreeWidgetItem *axis2Element = addChild(element,
                                                      QObject::tr("Axis2"),
                                                      "");
             addChild(axis2Element,
                      QObject::tr("XYZ"),
                      QString::fromStdString(joints.at(i).two->xyz));
             addChild(axis2Element,
                      QObject::tr("Damping"),
                      QString::fromStdString(joints.at(i).two->damping));
             addChild(axis2Element,
                      QObject::tr("Friction"),
                      QString::fromStdString(joints.at(i).two->friction));
             addChild(axis2Element,
                      QObject::tr("Upper limit"),
                      QString::fromStdString(joints.at(i).two->upper));
             addChild(axis2Element,
                      QObject::tr("Lower limit"),
                      QString::fromStdString(joints.at(i).two->lower));
             addChild(axis2Element,
                      QObject::tr("Effort limit"),
                      QString::fromStdString(joints.at(i).two->effort));
             addChild(axis2Element,
                      QObject::tr("Limit velocity"),
                      QString::fromStdString(joints.at(i).two->velocity));
         }
    }

    std::vector<ModelPlugin> plugins = model->model.GetListsPlugins();
    for(std::size_t i = 0; i < plugins.size(); i++)
    {
        element = addRoot(QObject::tr("Plugin"), "", tree);
        addChild(element,
                 QObject::tr("Name"),
                 QString::fromStdString(plugins.at(i).GetName()));
        addChild(element,
                 QObject::tr("Filename"),
                 QString::fromStdString(plugins.at(i).GetFilename()));
        for(std::size_t j = 0; j < plugins.at(i).parameters.size(); j++)
        {
            addChild(element,
                     QString::fromStdString(plugins.at(i).parameters.at(j)),
                     QString::fromStdString(plugins.at(i).values.at(j)));
        }
    }
    std::vector<sensor> sensors = model->model.ListsSensors;
    for(std::size_t i = 0; i < sensors.size(); i++)
    {
        element = addRoot(QObject::tr("Sensor"), "", tree);
        addChild(element, QObject::tr("Name"), sensors.at(i).name);
        addChild(element, QObject::tr("Type"), sensors.at(i).type);
        addChild(element, QObject::tr("Always_on"), sensors.at(i).always_on);
        addChild(element, QObject::tr("Visualize"), sensors.at(i).visualize);
        addChild(element,
                 QObject::tr("Link"),
                 QString::number(sensors.at(i).link));
        QTreeWidgetItem *modelPose = addChild(element, QObject::tr("Pose"), "");
        splitVector(sensors.at(i).pose, modelPose);
        addChild(element,
                 QObject::tr("Update_rate"),
                 sensors.at(i).update_rate);
        addChild(element, QObject::tr("Topic"), sensors.at(i).topic);

        if(sensors.at(i).name == "sonar")
        {
            addChild(element, "Max", sensors.at(i).max);
            addChild(element, "Min", sensors.at(i).min);
            addChild(element, "Radius", sensors.at(i).radius);
        }
        if(sensors.at(i).name == "imu")
        {
            QTreeWidgetItem* accel = addChild(element, "Acceleration", "");
            QTreeWidgetItem* accelx = addChild(accel, "x", "");
            addChild(accelx, "type", sensors.at(i).accel_x.type);
            addChild(accelx, "mean", sensors.at(i).accel_x.mean);
            addChild(accelx, "stddev", sensors.at(i).accel_x.stddev);
            addChild(accelx, "bias mean", sensors.at(i).accel_x.bias_mean);
            addChild(accelx, "bias stddev", sensors.at(i).accel_x.bias_stddev);
            addChild(accelx, "precision", sensors.at(i).accel_x.precision);

            QTreeWidgetItem* accely = addChild(accel, "y", "");
            addChild(accely, "type", sensors.at(i).accel_y.type);
            addChild(accely, "mean", sensors.at(i).accel_y.mean);
            addChild(accely, "stddev", sensors.at(i).accel_y.stddev);
            addChild(accely, "bias mean", sensors.at(i).accel_y.bias_mean);
            addChild(accely, "bias stddev", sensors.at(i).accel_y.bias_stddev);
            addChild(accely, "precision", sensors.at(i).accel_y.precision);

            QTreeWidgetItem* accelz = addChild(accel, "z", "");
            addChild(accelz, "type", sensors.at(i).accel_z.type);
            addChild(accelz, "mean", sensors.at(i).accel_z.mean);
            addChild(accelz, "stddev", sensors.at(i).accel_z.stddev);
            addChild(accelz, "bias mean", sensors.at(i).accel_z.bias_mean);
            addChild(accelz, "bias stddev", sensors.at(i).accel_z.bias_stddev);
            addChild(accelz, "precision", sensors.at(i).accel_z.precision);

            QTreeWidgetItem *ang = addChild(element, "Angular", "");
            QTreeWidgetItem *angx = addChild(ang, "x", "");
            addChild(angx, "type", sensors.at(i).ang_x.type);
            addChild(angx, "mean", sensors.at(i).ang_x.mean);
            addChild(angx, "stddev", sensors.at(i).ang_x.stddev);
            addChild(angx, "bias mean", sensors.at(i).ang_x.bias_mean);
            addChild(angx, "bias stddev", sensors.at(i).ang_x.bias_stddev);
            addChild(angx, "precision", sensors.at(i).ang_x.precision);

            QTreeWidgetItem* angy = addChild(ang,"y","");
            addChild(angy,"type",sensors.at(i).ang_y.type);
            addChild(angy,"mean",sensors.at(i).ang_y.mean);
            addChild(angy,"stddev",sensors.at(i).ang_y.stddev);
            addChild(angy,"bias mean",sensors.at(i).ang_y.bias_mean);
            addChild(angy,"bias stddev",sensors.at(i).ang_y.bias_stddev);
            addChild(angy,"precision",sensors.at(i).ang_y.precision);

            QTreeWidgetItem* angz = addChild(ang,"z","");
            addChild(angz,"type",sensors.at(i).ang_z.type);
            addChild(angz,"mean",sensors.at(i).ang_z.mean);
            addChild(angz,"stddev",sensors.at(i).ang_z.stddev);
            addChild(angz,"bias mean",sensors.at(i).ang_z.bias_mean);
            addChild(angz,"bias stddev",sensors.at(i).ang_z.bias_stddev);
            addChild(angz,"precision",sensors.at(i).ang_z.precision);
        }
        if(sensors.at(i).name=="gps")
        {
            QTreeWidgetItem* pos = addChild(element,"position sensing","");
            QTreeWidgetItem* pos_hor = addChild(pos,"horizontal","");
            addChild(pos_hor,"type",sensors.at(i).pos_horizontal.type);
            addChild(pos_hor,"mean",sensors.at(i).pos_horizontal.mean);
            addChild(pos_hor,"stddev",sensors.at(i).pos_horizontal.stddev);
            addChild(pos_hor,"bias mean",sensors.at(i).pos_horizontal.bias_mean);
            addChild(pos_hor,"bias stddev",sensors.at(i).pos_horizontal.bias_stddev);
            addChild(pos_hor,"precision",sensors.at(i).pos_horizontal.precision);

            QTreeWidgetItem* pos_ver = addChild(pos,"vertical","");
            addChild(pos_ver,"type",sensors.at(i).pos_vertical.type);
            addChild(pos_ver,"mean",sensors.at(i).pos_vertical.mean);
            addChild(pos_ver,"stddev",sensors.at(i).pos_vertical.stddev);
            addChild(pos_ver,"bias mean",sensors.at(i).pos_vertical.bias_mean);
            addChild(pos_ver,"bias stddev",sensors.at(i).pos_vertical.bias_stddev);
            addChild(pos_ver,"precision",sensors.at(i).pos_vertical.precision);

            QTreeWidgetItem* vel = addChild(element,"velocity sensing","");
            QTreeWidgetItem* vel_hor = addChild(vel,"horizontal","");
            addChild(vel_hor,"type",sensors.at(i).vel_horizontal.type);
            addChild(vel_hor,"mean",sensors.at(i).vel_horizontal.mean);
            addChild(vel_hor,"stddev",sensors.at(i).vel_horizontal.stddev);
            addChild(vel_hor,"bias mean",sensors.at(i).vel_horizontal.bias_mean);
            addChild(vel_hor,"bias stddev",sensors.at(i).vel_horizontal.bias_stddev);
            addChild(vel_hor,"precision",sensors.at(i).vel_horizontal.precision);

            QTreeWidgetItem* vel_ver = addChild(vel,"vertical","");
            addChild(vel_ver,"type",sensors.at(i).vel_vertical.type);
            addChild(vel_ver,"mean",sensors.at(i).vel_vertical.mean);
            addChild(vel_ver,"stddev",sensors.at(i).vel_vertical.stddev);
            addChild(vel_ver,"bias mean",sensors.at(i).vel_vertical.bias_mean);
            addChild(vel_ver,"bias stddev",sensors.at(i).vel_vertical.bias_stddev);
            addChild(vel_ver,"precision",sensors.at(i).vel_vertical.precision);
        }
        if(sensors.at(i).name=="magnetometer")
        {
            QTreeWidgetItem* X = addChild(element,"x","");
            addChild(X,"type",sensors.at(i).x.type);
            addChild(X,"mean",sensors.at(i).x.mean);
            addChild(X,"stddev",sensors.at(i).x.stddev);
            addChild(X,"bias mean",sensors.at(i).x.bias_mean);
            addChild(X,"bias stddev",sensors.at(i).x.bias_stddev);
            addChild(X,"precision",sensors.at(i).x.precision);

            QTreeWidgetItem* Y = addChild(element,"y","");
            addChild(Y,"type",sensors.at(i).y.type);
            addChild(Y,"mean",sensors.at(i).y.mean);
            addChild(Y,"stddev",sensors.at(i).y.stddev);
            addChild(Y,"bias mean",sensors.at(i).y.bias_mean);
            addChild(Y,"bias stddev",sensors.at(i).y.bias_stddev);
            addChild(Y,"precision",sensors.at(i).y.precision);

            QTreeWidgetItem* Z = addChild(element,"z","");
            addChild(Z,"type",sensors.at(i).z.type);
            addChild(Z,"mean",sensors.at(i).z.mean);
            addChild(Z,"stddev",sensors.at(i).z.stddev);
            addChild(Z,"bias mean",sensors.at(i).z.bias_mean);
            addChild(Z,"bias stddev",sensors.at(i).z.bias_stddev);
            addChild(Z,"precision",sensors.at(i).z.precision);
        }
    }
}

void Model::splitvector(std::string data,
                        QTreeWidgetItem* Element,
                        bool editable)
{
    Q_UNUSED(editable) // Ignore warning about unused parameter
    splitVector(QString::fromStdString(data), Element);
}

void Model::splitVector(const QString &data, QTreeWidgetItem *item)
{
    if(!data.isEmpty())
    {
        QStringList splitvector;
        QString vector = data;
        QRegExp rx("(\\ |\\  |\\   |\\    |\\     |\\        |\\         "
                   "|\\          |\\           |\\n|\\t)");
        splitvector = vector.split(rx);

        QStringList result;
        foreach (const QString &str, splitvector)
        {
            if (str.isEmpty() || str.contains(" "))
            {
                //faz nada
            }
            else result += str;
        }
        if(result.length() < 5) {
            qCritical() << QObject::tr("Error. The splivector has less than 6 "
                                       "items, this procedure cannot finish "
                                       "execution.");
        }
        else {
            addChild(item, QObject::tr("X"), result.at(0));
            addChild(item, QObject::tr("Y"), result.at(1));
            addChild(item, QObject::tr("Z"), result.at(2));
            addChild(item, QObject::tr("Roll"), result.at(3));
            addChild(item, QObject::tr("Pitch"), result.at(4));
            addChild(item, QObject::tr("Yaw"), result.at(5));
        }
    }
}
