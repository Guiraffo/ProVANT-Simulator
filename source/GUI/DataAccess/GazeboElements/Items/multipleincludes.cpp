#include "multipleincludes.h"

MultipleIncludes::MultipleIncludes()
{

}
void MultipleIncludes::Read(QDomNode document)
{

        Clear();

        // lendo primeiro include
        document = document.firstChildElement("include");
        // lendo demais include se houver
        while(true)
        {
            Include_DA item;
            if(!document.isNull())
            {
                item.SetUri(document.firstChildElement("uri")
                              .text().toStdString());
                item.SetName(document.firstChildElement("name")
                              .text().toStdString());
                item.SetIsStatic(document.firstChildElement("static")
                                   .text().toStdString());
                if(item.GetIsStatic()=="")item.SetIsStatic("false");
                item.SetPose(document.firstChildElement("pose")
                               .text().toStdString());

                NewInclude(item);
                qDebug() << "MultipleIncludes::nextSiblingElement";
                document = document.nextSiblingElement("include");
            }else break;

        }
}

void MultipleIncludes::NewInclude(Include_DA Item)
{
    multipleItens.push_back(Item);
}

void MultipleIncludes::Clear()
{
    multipleItens.clear();
}

void MultipleIncludes::print()
{
    foreach (Include_DA var, multipleItens) {
          var.print();
    }
}
