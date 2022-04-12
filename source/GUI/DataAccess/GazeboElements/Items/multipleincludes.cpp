#include "multipleincludes.h"

MultipleIncludes::MultipleIncludes()
{
}

bool MultipleIncludes::read(const QDomElement& element)
{
  if (element.isNull())
    return false;
  _includes.clear();

  QDomElement includeElement = element.firstChildElement("include");
  while (!includeElement.isNull())
  {
    IncludeDA newInclude;
    if (!newInclude.read(includeElement))
    {
      _includes.clear();
      return false;
    }

    addIncludeElement(newInclude);
    includeElement = includeElement.nextSiblingElement("include");
  }

  return true;
}

void MultipleIncludes::write(QXmlStreamWriter* xml) const
{
  for (QList<IncludeDA>::const_iterator it = _includes.cbegin();
       it != _includes.end(); it++)
  {
    it->write(xml);
  }
}

bool MultipleIncludes::isEmpty() const
{
  return _includes.isEmpty();
}

int MultipleIncludes::numElements() const
{
  return _includes.size();
}

const QList<IncludeDA>& MultipleIncludes::getIncludes() const
{
  return _includes;
}

void MultipleIncludes::addIncludeElement(const IncludeDA& include)
{
  _includes.push_back(include);
}

void MultipleIncludes::setIncludeElements(const QList<IncludeDA>& include)
{
  _includes = include;
}

void MultipleIncludes::setIncludeElements(
    const std::vector<IncludeDA>& includes)
{
  _includes.clear();
  for (std::vector<IncludeDA>::const_iterator it = includes.begin();
       it != includes.end(); ++it)
  {
    _includes.push_back(*it);
  }
}

void MultipleIncludes::Read(QDomNode document)
{
  read(document.toElement());
}

void MultipleIncludes::NewInclude(IncludeDA Item)
{
  addIncludeElement(Item);
}

void MultipleIncludes::Clear()
{
  clearIncludeElements();
}

void MultipleIncludes::print() const
{
  foreach (IncludeDA var, _includes)
  {
    var.print();
  }
}

void MultipleIncludes::clearIncludeElements()
{
  _includes.clear();
}

std::vector<IncludeDA> MultipleIncludes::getIncludesAsVector() const
{
  std::vector<IncludeDA> includeVec;
  includeVec.reserve(_includes.size());

  for (QList<IncludeDA>::const_iterator it = _includes.cbegin();
       it != _includes.constEnd(); it++)
  {
    includeVec.push_back(*it);
  }

  return includeVec;
}
