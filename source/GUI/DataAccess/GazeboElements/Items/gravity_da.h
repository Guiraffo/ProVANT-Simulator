#ifndef GRAVITY_DA_H
#define GRAVITY_DA_H

#include <QDomDocument>
#include <QString>
#include <QXmlStreamWriter>

class GravityDA
{
public:
  GravityDA();

  GravityDA(const GravityDA& other) = default;
  GravityDA& operator=(const GravityDA& other) = default;

  std::string getGravity() __attribute__((deprecated));
  const QString& gravity() const;

  void setGravity(std::string) __attribute__((deprecated));
  void setGravity(const QString& value);

  bool read(const QDomElement& worldElement);
  void write(QXmlStreamWriter* xml) const;

  void Read(QDomNode) __attribute__((deprecated));
  void Write(QXmlStreamWriter) __attribute__((deprecated));
  void print();

private:
  QString _gravity;

  constexpr static const char* gravityTag = "gravity";
};

#endif  // GRAVITY_DA_H
