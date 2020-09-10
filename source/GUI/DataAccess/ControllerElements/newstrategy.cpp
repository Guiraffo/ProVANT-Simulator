#include "newstrategy.h"

#include <QDir>
#include <QDesktopServices>
#include <QTextCodec>
#include <QTextStream>
#include <QUrl>

#include "Utils/appsettings.h"

using std::endl;

NewStrategy::NewStrategy()
{

}

void NewStrategy::createProject(const QString &name)
{
    QString trimmedName = name.trimmed();
    // Reads  the path to the control strategies folder
    AppSettings settings;
    QDir controlStrategiesDir(settings.getControlStrategiesPath());

    if(controlStrategiesDir.exists()) {
        // Create a new directory with the control strategy name
        controlStrategiesDir.mkdir(trimmedName);
        controlStrategiesDir.cd(trimmedName);
        // Create the include/<project_name> folder
        controlStrategiesDir.mkdir("include");
        controlStrategiesDir.cd("include");
        controlStrategiesDir.mkdir(trimmedName);
        controlStrategiesDir.cdUp();
        // Create the src folder
        controlStrategiesDir.mkdir("src");

        // The path to the new control strategy folder
        QString path = controlStrategiesDir.absolutePath();

        // Write the CmakeLists.txt file
        QFile cmakeListsFile(QDir::cleanPath(path + "/CMakeLists.txt"));
        if(cmakeListsFile.open(QFile::ReadWrite | QFile::Text))
        {
            QTextStream stream( &cmakeListsFile );
            stream.setCodec(QTextCodec::codecForName("UTF-8"));

            stream << "# This file is part of the ProVANT simulator project.\n"
                   << "# Licensed under the terms of the MIT open source "
                      "license. More details at\n"
                   << "# https://github.com/Guiraffo/ProVANT-Simulator/blob/"
                      "master/LICENSE.md\n\n"
                   << "cmake_minimum_required(VERSION 3.0.2)\n"
                   << "project(" << trimmedName << ")\n\n"
                   << "find_package(catkin REQUIRED COMPONENTS\n"
                   << "\troscpp\n"
                   << "\tstd_msgs\n"
                   << "\tsimulator_msgs\n)\n"
                   << "find_package(Eigen3 3.3 REQUIRED NO_MODULE)\n\n"
                      // Catkin package section
                   << "catkin_package(\n"
                   << "\tINCLUDE_DIRS\n"
                   << "\t\tinclude/${PROJECT_NAME}\n"
                   << "\tLIBRARIES\n"
                   << "\t\t${PROJECT_NAME}\n"
                   << "\tCATKIN_DEPENDS\n"
                   << "\t\troscpp\n"
                   << "\t\tstd_msgs\n"
                   << "\t\tsimulator_msgs\n"
                   << "\t# DEPENDS\n"
                   << ")\n\n"
                   << "include_directories(include/${PROJECT_NAME})\n"
                   << "include_directories(${catkin_INCLUDE_DIRS})\n"
                   << "include_directories($ENV{TILT_PROJECT}/source/"
                      "Structure/control_strategies/)\n\n"
                      // Add library target and specify depencies
                   << "add_library(${PROJECT_NAME} src/main.cpp)\n"
                   << "add_dependencies(${PROJECT_NAME} "
                      "${catkin_EXPORTED_TARGETS})\n"
                   << "target_link_libraries(${PROJECT_NAME}\n"
                   << "\t${catkin_LIBRARIES}\n"
                   << "\tEigen3::Eigen\n"
                   << ")\n\n"
                   << "install(TARGETS\n"
                   << "\t${PROJECT_NAME}\n\n"
                   << "\tARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}"
                   << "\n"
                   << "\tLIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}"
                   << "\n"
                   << "\tRUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}"
                      "\n)"
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
            stream << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                   << "<!--\n"
                   << "This file is part of the ProVANT simulator project.\n"
                   << "Licensed under the terms of the MIT open source license."
                      " More details at\n"
                   << "https://github.com/Guiraffo/ProVANT-Simulator/blob/"
                      "master/LICENSE.md\n"
                   << "-->\n"
                   << "<package format=\"2\">\n"

                   << "\t<name>" << trimmedName << "</name>\n"
                   << "\t<version>0.0.1</version>\n"
                   << "\t<description> The " << trimmedName
                   << " package...</description>\n\n"

                   << "\t<maintainer email=\"jeduardo@ufmg.br\">"
                      "Junio Eduardo de Morais Aquino</maintainer>\n\n"

                   << "\t<license>MIT</license>\n\n"

                   << "\t<url type=\"website\">"
                      "http://provant.eng.ufmg.br/provantsimulator</url>\n"
                   << "\t<url type=\"repository\">"
                      "https://github.com/Guiraffo/ProVANT-Simulator.git"
                      "</url>\n"
                   << "\t<url type=\"bugtracker\">"
                      "https://github.com/Guiraffo/ProVANT-Simulator/issues"
                      "</url>\n\n"

                   << "\t<author email=\"youremail@todo.com\">"
                      "Please Insert Your Name Here</author>\n\n"

                   << "\t<buildtool_depend>catkin</buildtool_depend>\n\n"

                   << "\t<build_depend>cmake_modules</build_depend>\n"
                   << "\t<build_depend>roscpp</build_depend>\n"
                   << "\t<build_depend>std_msgs</build_depend>\n"
                   << "\t<build_depend>simulator_msgs</build_depend>\n"
                   << "\t<build_depend>eigen</build_depend>\n\n"

                   << "\t<exec_depend>roscpp</exec_depend>\n"
                   << "\t<exec_depend>std_msgs</exec_depend>\n"
                   << "\t<exec_depend>simulator_msgs</exec_depend>\n"
                   << "\t<exec_depend>eigen</exec_depend>\n\n"

                   << "\t<export>\n"
                   << "\n"
                   << "\t</export>\n"

                   << "</package>"
                   << endl;

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
        controlStrategiesDir.cdUp();

        // Writes the main.cpp file
        QFile mainCppFile(QDir::cleanPath(srcPath
                                          +
                                          QDir::separator()
                                          +
                                          "main.cpp"));
        // Assemble Class Name
        QString className = capitalizeClassName(name);

        if(mainCppFile.open(QFile::ReadWrite))
        {
            QTextStream stream(&mainCppFile);

            stream << "/*\n"
                   << " * This file is part of the ProVANT simulator project.\n"
                   << " * Licensed under the terms of the MIT open source "
                      "license. More details at\n"
                   << " * https://github.com/Guiraffo/ProVANT-Simulator/blob/"
                      "master/LICENSE.md\n"
                   << " */\n"
                   << "/**\n"
                   << " * @file This file contains the implementation of the "
                   << className
                   << " class.\n"
                   << " *\n"
                   << " * @author Insert Your Name Here\n"
                   << " */\n\n"

                   << "#include \"Icontroller.hpp\"\n"
                   << "#include <vector>\n\n"

                   << "class " << className << " : public Icontroller\n"
                   << "{\n"

                   << "public:\n"
                   << "\t" << className << "(){}\n"
                   << "\tvirtual ~" << className << "(){}\n\n"
                   << "\tvoid config(){}\n\n"

                   << "\tstd::vector<double> "
                      "execute(simulator_msgs::SensorArray arraymsg)\n"
                   << "\t{\n"
                   << "\t\tstd::vector<double> out;\n"
                   << "\t\treturn out;\n"
                   << "\t}\n\n"

                   << "\tstd::vector<double> Reference()\n"
                   << "\t{\n"
                   << "\t\tstd::vector<double> out;\n"
                   << "\t\treturn out;\n"
                   << "\t}\n\n"

                   << "\tstd::vector<double> Error()\n"
                   << "\t{\n"
                   << "\t\tstd::vector<double> out;\n"
                   << "\t\treturn out;\n"
                   << "\t}\n\n"

                   << "\tstd::vector<double> State()\n"
                   << "\t{\n"
                   << "\t\tstd::vector<double> out;\n"
                   << "\t\treturn out;\n"
                   << "\t}\n"

                   << "};\n\n"

                   << "extern \"C\"\n"
                   << "{\n"
                   << "\tIcontroller *create(void) {return new "
                   << className
                   << ";}\n"
                   << "\tvoid destroy(Icontroller *p) {delete p;}\n"
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

QString capitalizeClassName(const QString &name)
{
    if(name.isEmpty()) {
        return QString();
    }

    QString lowerCaseName = name.toLower();

    QString cameldCasedName = lowerCaseName.at(0).toUpper();
    // Indicate if the next letter should be uppercased
    bool upNext = false;
    for(int i = 1; i < name.length(); i++) {
        if(name.at(i) == "_") {
            upNext = true;
        }
        else if(upNext) {
            upNext = false;
            cameldCasedName += lowerCaseName.at(i).toUpper();
        }
        else {
            cameldCasedName += lowerCaseName.at(i);
        }
    }

    return cameldCasedName;
}
