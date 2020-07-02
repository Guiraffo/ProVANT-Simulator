#include "newstrategy.h"
#include "QDebug"

#include "Utils/appsettings.h"

newstrategy::newstrategy()
{
}

void newstrategy::CreateProject(QString name)
{
    AppSettings settings;
    char const* tmp = settings.getTiltProjectPath().toStdString().c_str();
    if ( tmp == NULL ) {
        qDebug() << "Problemas com variavel de ambiente ";
    } else {
        std::string env(tmp);
        QDir dir(QString::fromStdString(env)+"/source/Structure/control_strategies/");
        dir.mkdir(name);
        dir.cd(name);
        dir.mkdir("include");
        dir.mkdir("src");
        QString path = dir.absolutePath();
        QFile file(path+"/CMakeLists.txt");
        if(file.open(QFile::ReadWrite))
        {
            QTextStream stream( &file );
            stream << "cmake_minimum_required(VERSION 2.8.3)" << endl
            << "project("+name+")"<< endl
            << "find_package(catkin REQUIRED COMPONENTS"<< endl
            << "roscpp" << endl
            << "rospy" << endl
            << "std_msgs" << endl
            << "simulator_msgs)" << endl << endl
            << "catkin_package()" << endl
            << "include_directories(include)" << endl
            << "INCLUDE_DIRECTORIES (/usr/include/eigen3)" << endl
            << "include_directories( ${catkin_INCLUDE_DIRS})" << endl
            << "include_directories($ENV{TILT_PROJECT}/source/Structure/control_strategies/)" << endl << endl
            << "add_library("+name +" src/main.cpp)" << endl
            << "target_link_libraries("+name+" ${catkin_LIBRARIES})" << endl
            << "install(TARGETS" << endl
            << name << endl
            << "ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}" << endl
            << "LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}" << endl
            << " RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})" << endl;

            file.close();
        }

        QFile file2(path+"/package.xml");
        if(file2.open(QFile::ReadWrite))
        {
            QTextStream stream( &file2 );
            stream << "<?xml version=\"1.0\"?>" << endl
            << "<package>" << endl
            << "\t<name>"+name+"</name>"<< endl
            << "\t<version>0.0.0</version>" << endl
            << "\t<description> "+name +" package</description>" << endl
            << "\t<maintainer email=\"macro@todo.todo\">macro</maintainer>"<< endl
            << "\t<license>TODO</license>" << endl
            << "\t<buildtool_depend>catkin</buildtool_depend>" << endl
            << "\t<build_depend>roscpp</build_depend>" << endl
            << "\t<build_depend>rospy</build_depend>" << endl
            << "\t<build_depend>std_msgs</build_depend>" << endl
            << "\t<run_depend>roscpp</run_depend>"<< endl
            << "\t<run_depend>rospy</run_depend>" << endl
            << "\t<run_depend>std_msgs</run_depend>" << endl
            << "\t<build_depend>simulator_msgs</build_depend>" << endl
            << "\t<run_depend>simulator_msgs</run_depend>" << endl
            << "\t<build_depend>cmake_modules</build_depend>" << endl
            << "\t<run_depend>cmake_modules</run_depend>" << endl
            << "</package>" << endl;

            file2.close();
        }

        dir.cd("src");
        path = dir.absolutePath();
        QFile file3(path+"/main.cpp");
        if(file3.open(QFile::ReadWrite))
        {
            QTextStream stream( &file3 );
            stream << "#include \"Icontroller.hpp\"" << endl << endl << endl
            << "class "+name+" : public Icontroller" << endl
            << "{"<< endl
            << "\tpublic: "+name+"(){}"<< endl
            << "\tpublic: ~"+name+"(){}"<< endl
            << "\tpublic: void config(){}"<< endl
            << "\tpublic: std::vector<double> execute(simulator_msgs::SensorArray arraymsg)"<<endl
            << "\t{"<<endl<<"\t\tstd::vector<double> out;" << endl << "\t\treturn out;">> endl <<"\t}"<< endl
            << "\tpublic: std::vector<double> Reference()" << endl
            << "\t{"<<endl<<"\t\tstd::vector<double> out;" << endl << "\t\treturn out;">> endl <<"\t}"<< endl
            << "\tpublic: std::vector<double> Error()" << endl
            << "\t{"<<endl<<"\t\tstd::vector<double> out;" << endl << "\t\treturn out;">> endl <<"\t}"<< endl
            << "\tpublic: std::vector<double> State()" << endl << endl
            << "\t{"<<endl<<"\t\tstd::vector<double> out;" << endl << "\t\treturn out;">> endl <<"\t}"<< endl
            << "};"<< endl << endl << endl
            << "extern \"C\"" << endl
            << "{"<< endl
            << "\tIcontroller *create(void) {return new "+name+";}" << endl
            << "\tvoid destroy(Icontroller *p) {delete p;}"<< endl
            << "}" << endl;


            file3.close();

            QFileDialog teste;
            teste.viewMode();

            std::string command("nautilus "+env+"/source/Structure/control_strategies/"+name.toStdString());
            std::system(command.c_str());
        }
    }
}



