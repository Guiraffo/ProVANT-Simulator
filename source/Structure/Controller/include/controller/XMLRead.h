#include <tinyxml.h>

class XMLRead
{
	private: TiXmlDocument doc;

	public: XMLRead()
	{

	}

	public: XMLRead(std::string pathFile): doc(pathFile)
	{

	}

	public: std::string GetItem (std::string itemname )
	{
		std::string buff = "";
		{
			if(doc.LoadFile())
			{
				TiXmlHandle dochandle(&doc);
				TiXmlElement * item;
				item = dochandle.FirstChild("config").FirstChild(itemname).ToElement();
				if(item != NULL)
				{
					buff = item->GetText();
				}
				else
				{
					std::cout << "Não se declarou o item <"+itemname+"> <"+itemname+">" << std::endl;
					exit(0);
				}
			}
			return buff;
		}
	}
	public: std::vector<std::string> GetAllItems (std::string itemname)
	{
		std::vector<std::string> output;
		if(doc.LoadFile())
		{
			TiXmlHandle dochandle(&doc);
			TiXmlElement* child = dochandle.FirstChild("config").FirstChild(itemname).FirstChild("Device").ToElement();
			for( child; child; child=child->NextSiblingElement("Device") )
			{
				output.push_back(child->GetText());
			}
		}
		return output;	
	}
};
