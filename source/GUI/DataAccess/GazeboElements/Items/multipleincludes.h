#ifndef MULTIPLEINCLUDES_H
#define MULTIPLEINCLUDES_H

#include "include_da.h"

#include <QDomElement>
#include <QList>
#include <QXmlStreamWriter>

class MultipleIncludes
{
public:
  MultipleIncludes();

  void NewInclude(IncludeDA) __attribute__((deprecated));
  void Read(QDomNode) __attribute__((deprecated));
  void Clear() __attribute__((deprecated));
  void print() const;

  bool read(const QDomElement& element);
  void write(QXmlStreamWriter* xml) const;

  bool isEmpty() const;
  int numElements() const;

  const QList<IncludeDA>& getIncludes() const;
  void addIncludeElement(const IncludeDA& include);
  void setIncludeElements(const QList<IncludeDA>& include);
  void setIncludeElements(const std::vector<IncludeDA>& includes)
      __attribute__((deprecated));
  void clearIncludeElements();
  std::vector<IncludeDA> getIncludesAsVector() const
      __attribute__((deprecated));

private:
  QList<IncludeDA> _includes;
};

#endif  // MULTIPLEINCLUDES_H
