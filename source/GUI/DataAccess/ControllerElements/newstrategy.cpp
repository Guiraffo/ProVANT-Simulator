#include "newstrategy.h"

#include <QDir>
#include <QDesktopServices>
#include <QTextStream>
#include <QUrl>

#include "Utils/appsettings.h"

using std::endl;

NewStrategy::NewStrategy()
{

}

void NewStrategy::createProject(const QString &name)
{
    // Reads  the path to the control strategies folder
    AppSettings settings;
    QDir controlStrategiesDir(settings.getControlStrategiesPath());

    if(controlStrategiesDir.exists()) {
        // Create a new directory with the control strategy name
        controlStrategiesDir.mkdir(name);
        controlStrategiesDir.cd(name);
        controlStrategiesDir.mkdir("include");
        controlStrategiesDir.mkdir("src");

        // The path to the new control strategy folder
        QString path = controlStrategiesDir.absolutePath();

        // Write the CmakeLists.txt file
        QFile cmakeListsFile(QDir::cleanPath(path + "/CMakeLists.txt"));
        if(cmakeListsFile.open(QFile::ReadWrite | QFile::Text))
        {
            QTextStream stream( &cmakeListsFile );

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
                   << "include_directories($ENV{TILT_PROJECT}/source/"
                      "Structure/control_strategies/)" << endl << endl
                   << "add_library("+name +" src/main.cpp)" << endl
                   << "target_link_libraries("+name+" ${catkin_LIBRARIES})"
                   << endl
                   << "install(TARGETS" << endl
                   << name << endl
                   << "ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}"
                   << endl
                   << "LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}"
                   << endl
                   << "RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})"
                   << endl;

            // Forces the writing of the text buffer and closes the file
            stream.flush();
            cmakeListsFile.close();
        }
        else
        {
            qCritical("An error ocurrred while trying to create the "
                      "CmakeLists.txt file in the path: %s.",
                      qUtf8Printable(cmakeListsFile.fileName()));
        }

        // Write the package.xml file
        QFile packageXmlFile(path+"/package.xml");
        if(packageXmlFile.open(QFile::ReadWrite | QFile::Text))
        {
            QTextStream stream( &packageXmlFile );
            stream << "<?xml version=\"1.0\"?>" << endl
                   << "<package>" << endl
                   << "\t<name>"+name+"</name>"<< endl
                   << "\t<version>0.0.0</version>" << endl
                   << "\t<description> "+name +" package</description>" << endl
                   << "\t<maintainer email=\"macro@todo.todo\">"
                      "macro</maintainer>"<< endl
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

            // Forces the writing of the file buffer and closes the file
            stream.flush();
            packageXmlFile.close();
        }
        else
        {
            qCritical("An error ocurred while trying to create the "
                      "package.xml file with the path: %s.",
                      qUtf8Printable(packageXmlFile.fileName()));
        }

        controlStrategiesDir.cd("src");

        QString srcPath = controlStrategiesDir.absolutePath();

        // Writes the main.cpp file
        QFile mainCppFile(QDir::cleanPath(srcPath+"/main.cpp"));
        if(mainCppFile.open(QFile::ReadWrite))
        {
            QTextStream stream(&mainCppFile);

            stream << "#include \"Icontroller.hpp\"" << endl << endl << endl
                   << "class "+name+" : public Icontroller" << endl
                   << "{"<< endl
                   << "\tpublic: "+name+"(){}"<< endl
                   << "\tpublic: ~"+name+"(){}"<< endl
                   << "\tpublic: void config(){}"<< endl
                   << "\tpublic: std::vector<double> "
                      "execute(simulator_msgs::SensorArray arraymsg)"<<endl
                   << "\t{"<<endl<<"\t\tstd::vector<double> out;" << endl
                   << "\t\treturn out;">> endl <<"\t}"<< endl
                   << "\tpublic: std::vector<double> Reference()" << endl
                   << "\t{"<<endl<<"\t\tstd::vector<double> out;" << endl
                   << "\t\treturn out;">> endl <<"\t}"<< endl
                   << "\tpublic: std::vector<double> Error()" << endl
                   << "\t{"<<endl<<"\t\tstd::vector<double> out;" << endl
                   << "\t\treturn out;">> endl <<"\t}"<< endl
                   << "\tpublic: std::vector<double> State()" << endl << endl
                   << "\t{"<<endl<<"\t\tstd::vector<double> out;" << endl
                   << "\t\treturn out;">> endl <<"\t}"<< endl
                   << "};"<< endl << endl << endl
                   << "extern \"C\"" << endl
                   << "{"<< endl
                   << "\tIcontroller *create(void) {return new "+name+";}"
                   << endl
                   << "\tvoid destroy(Icontroller *p) {delete p;}"<< endl
                   << "}" << endl;

            // Force the writing of the file buffer and closes the file
            stream.flush();
            mainCppFile.close();
        }
        else
        {
            qCritical("An error ocurred while trying to create the "
                      "main.cpp file with the path: %s.",
                      qUtf8Printable(mainCppFile.fileName()));
        }

        // Opens the new control strategy in explorer
        QUrl destDir = QUrl::fromLocalFile(path);
        QDesktopServices::openUrl(destDir);
    }
}
