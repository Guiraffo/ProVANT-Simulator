#include "multipleplugins.h"

multipleplugins::multipleplugins()
{

}


void multipleplugins::NewPlugin(plugin_DA item){
    multipleItens.push_back(item);
}

void multipleplugins::Read(QDomNode document){

    qDebug() << "multipleplugins::Read";

    Clear();

    // lendo primeiro include
    document = document.firstChildElement("plugin");
    // lendo demais include se houver
    while(true)
    {
        plugin_DA item;
        if(!document.isNull())
        {
            item.SetName(document.toElement().attribute("name").toStdString());
            item.SetFilename(document.toElement().attribute("filename").toStdString());

            QDomNodeList list = document.childNodes();
            for(uint i = 0; i<list.count();i++)
            {
                item.parameters.push_back(list.at(i).toElement().tagName().toStdString());
                item.values.push_back(list.at(i).toElement().text().toStdString());
            }
            NewPlugin(item);
            qDebug() << "MultipleIncludes::nextSiblingElement";
            document = document.nextSiblingElement("plugin");
        }else break;

    }
}

void multipleplugins::Clear(){
    multipleItens.clear();
}

void multipleplugins::print(){
    foreach (plugin_DA var, multipleItens) {
          var.print();
    }
}



