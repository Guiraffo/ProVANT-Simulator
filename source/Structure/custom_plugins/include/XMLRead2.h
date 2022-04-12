/*
 * File: XMLRead.h
 * Author: Arthur Viana Lara
 * Project: ProVANT
 * Company: Federal University of Minas Gerais
 * Version: 1.0
 * Date: 29/01/18
 * Description: This library is responsable to implement code to read some data from XML file
 */

#include <tinyxml.h>

class XMLRead2
{
private:
  TiXmlDocument doc;

public:
  // constructor
  XMLRead2() __attribute__((deprecated))
  {
  }

  // destructor
  XMLRead2(std::string pathFile) __attribute__((deprecated)) : doc(pathFile)
  {
  }

  // get one item of XML file (here, it isn't used itens in hierarchy)
  std::string GetItem(std::string itemname) __attribute__((deprecated))
  {
    std::string buff = "";
    {
      if (doc.LoadFile())
      {
        TiXmlHandle dochandle(&doc);
        TiXmlElement* item;
        item = dochandle.FirstChild("config").FirstChild(itemname).ToElement();
        if (item != NULL)
        {
          buff = item->GetText();
        }
        else
        {
          std::cout << "Não se declarou o item <" + itemname + "> <" + itemname + ">" << std::endl;
          exit(0);
        }
      }
      return buff;
    }
  }

  // get all item of XML file (here, it isn't used itens in hierarchy)
  std::vector<std::string> GetAllItems(std::string itemname) __attribute__((deprecated))
  {
    std::vector<std::string> output;
    if (doc.LoadFile())
    {
      TiXmlHandle dochandle(&doc);
      TiXmlElement* child = dochandle.FirstChild("config").FirstChild(itemname).FirstChild("Device").ToElement();
      for (child; child; child = child->NextSiblingElement("Device"))
      {
        output.push_back(child->GetText());
      }
    }
    return output;
  }
};
