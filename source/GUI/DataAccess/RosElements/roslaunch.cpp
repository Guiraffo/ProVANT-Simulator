#include "roslaunch.h"

#include "Utils/appsettings.h"

roslaunch::roslaunch()
{
}

void roslaunch::WriteNew(QString worldname, QString worldbase, bool hil)
{
  AppSettings settings;
  char const* tmp = settings.getTiltProjectPath().toStdString().c_str();
  if (tmp == NULL)
  {
    qDebug() << "Problemas com variavel de ambiente ";
  }
  else
  {
    std::string env(tmp);
    qDebug() << worldbase;
    qDebug() << hil;
    // QFile
    // file(QString::fromStdString(env)+"/source/Database/launch/"+"gazebo.launch");
    QFile file(QString::fromStdString(env) + "/source/Database/launch/" +
               worldbase);
    if (file.exists())
      file.remove();
    if (file.open(QFile::ReadWrite))
    {
      QTextStream stream(&file);
      stream << "<launch>" << endl
             << "\t<arg name=\"paused\" default=\"false\"/>" << endl
             << "\t<arg name=\"use_sim_time\" default=\"true\"/>" << endl
             << "\t<arg name=\"gui\" default=\"true\"/>" << endl
             << "\t<arg name=\"headless\" default=\"false\"/>" << endl
             << "\t<arg name=\"debug\" default=\"false\"/>" << endl
             << endl
             << "\t<include file=\"$(find "
                "Database)/launch/empty_world.launch\">"
             << endl
             << "\t\t<arg name=\"world_name\" value=\"" + worldname + "\"/>"
             << endl
             << "\t\t<arg name=\"debug\" value=\"$(arg debug)\" />" << endl
             << "\t\t<arg name=\"gui\" value=\"$(arg gui)\" />" << endl
             << "\t\t<arg name=\"paused\" value=\"$(arg paused)\"/>" << endl
             << "\t\t<arg name=\"use_sim_time\" value=\"$(arg use_sim_time)\"/>"
             << endl
             << "\t\t<arg name=\"headless\" value=\"$(arg headless)\"/>" << endl
             << "\t</include>" << endl
             << endl;
      if (!hil)
      {
        stream << "\t<node name=\"controller\" pkg=\"controller\" "
                  "type=\"controller\" output=\"screen\" launch-prefix=\"xterm "
                  "-e\"/>"
               << endl
               << " </launch>" << endl;
      }
      else
      {
        stream << " </launch>" << endl;
      }
      file.close();
    }
  }
}
