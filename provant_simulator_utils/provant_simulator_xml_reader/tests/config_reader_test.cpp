#include <gtest/gtest.h>

#include "provant_simulator_xml_reader/config_reader.h"

TEST(ConfigReaderTest, OpenEmptyFile)
{
  // Using the default constructor with and specifying the open file path
  ConfigReader emptyReader1;
  ASSERT_FALSE(emptyReader1.open("data/empty.xml"));

  // Using the constructor with the file path and using the open method
  ConfigReader emptyReader2("data/empty.xml");
  ASSERT_FALSE(emptyReader2.open());

  ConfigReader emptyReader3("data/config.xml");
  ASSERT_FALSE(emptyReader3.open("data/empty.xml"));
}

TEST(ConfigReaderTest, OpenMalformedXMLFile)
{
  ConfigReader errorReader("data/invalid.xml");
  ASSERT_FALSE(errorReader.open());
}

TEST(ConfigReaderTest, CanOpenValidFile)
{
  ConfigReader reader1("data/config.xml");
  ASSERT_TRUE(reader1.open());

  ConfigReader reader2;
  ASSERT_TRUE(reader2.open("data/config.xml"));
}

TEST(ConfigReaderTest, CanOpenMinimalValidFile)
{
  ConfigReader reader("data/minimal.xml");
  ASSERT_TRUE(reader.open());
}

TEST(ConfigReaderTest, DefaultConstructorEmptyFilePath)
{
  ConfigReader def;
  ASSERT_TRUE(def.getFilePath().empty());
}

TEST(ConfigReaderTest, ConstructorSetsFilePath)
{
  ConfigReader reader("any.xml");
  ASSERT_EQ("any.xml", reader.getFilePath());
}

/*
 * Check if the open() method does not verify the file path set by the
 * constructor, even if it fails.
 */
TEST(ConfigReaderTest, OpenDoesNotModifieFilePath)
{
  std::string nonExistentPath = "doesnotexist.xml";
  ConfigReader reader1(nonExistentPath);
  ASSERT_FALSE(reader1.open());
  ASSERT_EQ(nonExistentPath, reader1.getFilePath());

  std::string minimalPath = "data/minimal.xml";
  ConfigReader minimalReader(minimalPath);
  ASSERT_TRUE(minimalReader.open());
  ASSERT_EQ(minimalPath, minimalReader.getFilePath());
}

TEST(ConfigReaderTest, OpenModifiesFilePath)
{
  std::string oldPath = "test.xml";
  std::string newPath = "data/minimal.xml";

  ConfigReader def;
  ASSERT_TRUE(def.open(newPath));
  ASSERT_EQ(newPath, def.getFilePath());

  std::string nonExistantPath = "doesnotexist.xml";
  ConfigReader reader1(oldPath);
  ASSERT_FALSE(reader1.open(nonExistantPath));
  ASSERT_EQ(nonExistantPath, reader1.getFilePath());
}

TEST(ConfigReaderTest, OpenFailsIfFilePathIsEmpty)
{
  ConfigReader def;
  ASSERT_TRUE(def.getFilePath().empty());
  ASSERT_FALSE(def.open());

  ConfigReader reader1("");
  ASSERT_EQ("", reader1.getFilePath());
  ASSERT_FALSE(reader1.open());

  ASSERT_FALSE(def.open(""));
}

TEST(ConfigReaderTest, LoadFromEmptyStringFails)
{
  ConfigReader def;
  ASSERT_FALSE(def.loadFromString(""));
}

TEST(ConfigReaderTest, LoadFromMalformXMLFails)
{
  ConfigReader def;
  const auto xmlstr = std::string("<?xml version=\"1.0\" encoding=\"UTF-8\"?><start></end>");
  ASSERT_FALSE(def.loadFromString(xmlstr));
}

TEST(ConfigReaderTest, LoadFromValidStringSucceeds)
{
  ConfigReader def;
  const auto xmlstr = std::string("<?xml version=\"1.0\" encoding=\"UTF-8\"?><!--This file is part of the ProVANT "
                                  "simulator project.Licensed under the terms of the MIT open source license. More "
                                  "details at "
                                  "https://github.com/Guiraffo/ProVANT-Simulator/blob/master/"
                                  "LICENSE.md--><config><empty></empty><oneChild><child>value</child></"
                                  "oneChild><twoChild><child>value1</child><child>value2</child></"
                                  "twoChild><twoChildDifferent><child1>value1</child1><child2>value2</child2></"
                                  "twoChildDifferent></config>");
  ASSERT_TRUE(def.loadFromString(xmlstr));
}

/*
 * When reading a value that is define, the getItem method must return the value
 * and set the found paramter to true.
 */
TEST(GetItemTest, ReadValid)
{
  ConfigReader configReader("data/config.xml");
  ASSERT_TRUE(configReader.open());

  ASSERT_EQ(configReader.getItem("topicdata"), "data");

  // Verifying if the found attribute is true
  bool found = false;
  ASSERT_EQ(configReader.getItem("topicdata", &found), "data");
  ASSERT_TRUE(found);
}

/*
 * When reading a value that is not defined, the getItem method must return
 * an emtpy string, and set the found parameter to false.
 */
TEST(GetItemTest, ReadNotDefined)
{
  ConfigReader configReader("data/config.xml");
  ASSERT_TRUE(configReader.open());

  ASSERT_EQ(configReader.getItem("any"), "");

  // Verifying if the found attribute is true
  bool found = false;
  ASSERT_EQ(configReader.getItem("any", &found), "");
  ASSERT_FALSE(found);
}

/*
 * WHen reading an element which is defined, but is empty, the getItem method
 * must return an empty string, and set the found parameter to true.
 */
TEST(GetItemTest, ReadEmpty)
{
  ConfigReader configReader("data/config3.xml");
  ASSERT_TRUE(configReader.open());

  ASSERT_EQ(configReader.getItem("Sampletime2"), "");

  // Verifying if the found attribute is true
  bool found = false;
  ASSERT_EQ(configReader.getItem("Sampletime2", &found), "");
  ASSERT_TRUE(found);
}

/*
 * When reading a numeric string, the getItem method must return the string,
 * and set the found parameter to true.
 */
TEST(GetItemTest, ReadNumeric)
{
  ConfigReader configReader("data/config.xml");
  ASSERT_TRUE(configReader.open());

  ASSERT_EQ(configReader.getItem("Sampletime"), "12");

  // Verifying if the found attribute is true
  bool found = false;
  ASSERT_EQ(configReader.getItem("Sampletime", &found), "12");
  ASSERT_TRUE(found);
}

/*
 * When reading an element which is defined, but is empty, the getItens method
 * must return an empty list, and set the found paramter to true.
 */
TEST(GetItensTest, ReadEmtpy)
{
  ConfigReader configReader("data/config5.xml");
  ASSERT_TRUE(configReader.open());

  ConfigReader::stringlist res;
  res = configReader.getItens("Sampletime", "child");
  ASSERT_TRUE(res.empty());

  bool found = false;
  res = configReader.getItens("Sampletime", "child", &found);
  ASSERT_TRUE(res.empty());
  ASSERT_TRUE(found);
}

/*
 * When reading an element which is defined, but has no children, the getItens
 * method must return an empty list, and set the found paramter to true.
 */
TEST(GetItensTest, ReadNoChildElements)
{
  ConfigReader configReader("data/config.xml");
  ASSERT_TRUE(configReader.open());

  ConfigReader::stringlist res;
  res = configReader.getItens("Sampletime", "child");
  ASSERT_TRUE(res.empty());

  bool found = false;
  res = configReader.getItens("Sampletime", "child", &found);
  ASSERT_TRUE(res.empty());
  ASSERT_TRUE(found);
}

/*
 * When reading an element which is defined, that has child elements, but none
 * with the specified child name, the getItens method must return an empty list, and set the found paramter to true.
 */
TEST(GetItensTest, ReadNoChildElementsWithSpecifiedName)
{
  ConfigReader configReader("data/config.xml");
  ASSERT_TRUE(configReader.open());

  ConfigReader::stringlist res;
  res = configReader.getItens("Sensors", "child");
  ASSERT_TRUE(res.empty());

  bool found = false;
  res = configReader.getItens("Sensors", "child", &found);
  ASSERT_TRUE(res.empty());
  ASSERT_TRUE(found);
}

/*
 * When reading an element which is defined, that has child elements with the
 * specified name, the getItens must return a list with the element values
 * and set the found parameter to true.
 */
TEST(GetItensTest, ReadOneChildElement)
{
  ConfigReader configReader("data/config.xml");
  ASSERT_TRUE(configReader.open());

  ConfigReader::stringlist res;
  res = configReader.getItens("Sensors", "Device");
  ASSERT_EQ(res.size(), 1);
  if (res.size() == 1)
    ASSERT_EQ(*res.begin(), "Estados");

  bool found = false;
  res = configReader.getItens("Sensors", "Device", &found);
  ASSERT_EQ(res.size(), 1);
  if (res.size() == 1)
    ASSERT_EQ(*res.begin(), "Estados");
  ASSERT_TRUE(found);
}

/*
 * When reading an element which is defined, that has child elements with the
 * specified name, the getItens must return a list with the element values
 * and set the found parameter to true.
 */
TEST(GetItensTest, ReadMultipleChildElements)
{
  ConfigReader configReader("data/config.xml");
  ASSERT_TRUE(configReader.open());

  ConfigReader::stringlist res;
  res = configReader.getItens("Actuators", "Device");
  ASSERT_EQ(res.size(), 4);
  if (res.size() == 4)
  {
    auto it = res.begin();
    ASSERT_EQ(*it, "Empuxoh1");
    ++it;
    ASSERT_EQ(*it, "Empuxoh2");
    ++it;
    ASSERT_EQ(*it, "Empuxoh3");
    ++it;
    ASSERT_EQ(*it, "Empuxoh4");
  }

  bool found = false;
  res = configReader.getItens("Actuators", "Device", &found);
  ASSERT_EQ(res.size(), 4);
  if (res.size() == 4)
  {
    auto it = res.begin();
    ASSERT_EQ(*it, "Empuxoh1");
    ++it;
    ASSERT_EQ(*it, "Empuxoh2");
    ++it;
    ASSERT_EQ(*it, "Empuxoh3");
    ++it;
    ASSERT_EQ(*it, "Empuxoh4");
  }
  ASSERT_TRUE(found);
}

/*
 * When reading an element which is defined, but is empty, the getItens method
 * must return an empty list, and set the found paramter to true.
 */
TEST(GetItensVectorTest, ReadEmtpy)
{
  ConfigReader configReader("data/config5.xml");
  ASSERT_TRUE(configReader.open());

  ConfigReader::stringvector res;
  res = configReader.getItensVector("Sampletime", "child");
  ASSERT_TRUE(res.empty());

  bool found = false;
  res = configReader.getItensVector("Sampletime", "child", &found);
  ASSERT_TRUE(res.empty());
  ASSERT_TRUE(found);
}

/*
 * When reading an element which is defined, but has no children, the getItens
 * method must return an empty list, and set the found paramter to true.
 */
TEST(GetItensVectorTest, ReadNoChildElements)
{
  ConfigReader configReader("data/config.xml");
  ASSERT_TRUE(configReader.open());

  ConfigReader::stringvector res;
  res = configReader.getItensVector("Sampletime", "child");
  ASSERT_TRUE(res.empty());

  bool found = false;
  res = configReader.getItensVector("Sampletime", "child", &found);
  ASSERT_TRUE(res.empty());
  ASSERT_TRUE(found);
}

/*
 * When reading an element which is defined, that has child elements, but none
 * with the specified child name, the getItens method must return an empty list, and set the found paramter to true.
 */
TEST(GetItensVectorTest, ReadNoChildElementsWithSpecifiedName)
{
  ConfigReader configReader("data/config.xml");
  ASSERT_TRUE(configReader.open());

  ConfigReader::stringvector res;
  res = configReader.getItensVector("Sensors", "child");
  ASSERT_TRUE(res.empty());

  bool found = false;
  res = configReader.getItensVector("Sensors", "child", &found);
  ASSERT_TRUE(res.empty());
  ASSERT_TRUE(found);
}

/*
 * When reading an element which is defined, that has child elements with the
 * specified name, the getItens must return a list with the element values
 * and set the found parameter to true.
 */
TEST(GetItensVectorTest, ReadOneChildElement)
{
  ConfigReader configReader("data/config.xml");
  ASSERT_TRUE(configReader.open());

  ConfigReader::stringvector res;
  res = configReader.getItensVector("Sensors", "Device");
  ASSERT_EQ(res.size(), 1);
  if (res.size() == 1)
    ASSERT_EQ(*res.begin(), "Estados");

  bool found = false;
  res = configReader.getItensVector("Sensors", "Device", &found);
  ASSERT_EQ(res.size(), 1);
  if (res.size() == 1)
    ASSERT_EQ(*res.begin(), "Estados");
  ASSERT_TRUE(found);
}

/*
 * When reading an element which is defined, that has child elements with the
 * specified name, the getItens must return a list with the element values
 * and set the found parameter to true.
 */
TEST(GetItensVectorTest, ReadMultipleChildElements)
{
  ConfigReader configReader("data/config.xml");
  ASSERT_TRUE(configReader.open());

  ConfigReader::stringvector res;
  res = configReader.getItensVector("Actuators", "Device");
  ASSERT_EQ(res.size(), 4);
  if (res.size() == 4)
  {
    auto it = res.begin();
    ASSERT_EQ(*it, "Empuxoh1");
    ++it;
    ASSERT_EQ(*it, "Empuxoh2");
    ++it;
    ASSERT_EQ(*it, "Empuxoh3");
    ++it;
    ASSERT_EQ(*it, "Empuxoh4");
  }

  bool found = false;
  res = configReader.getItensVector("Actuators", "Device", &found);
  ASSERT_EQ(res.size(), 4);
  if (res.size() == 4)
  {
    auto it = res.begin();
    ASSERT_EQ(*it, "Empuxoh1");
    ++it;
    ASSERT_EQ(*it, "Empuxoh2");
    ++it;
    ASSERT_EQ(*it, "Empuxoh3");
    ++it;
    ASSERT_EQ(*it, "Empuxoh4");
  }
  ASSERT_TRUE(found);
}

/*
 * If the file is valid, it must have no error message.
 */
TEST(CheckErrorTest, ValidFileHasNoError)
{
  ConfigReader configReader("data/config.xml");
  ASSERT_TRUE(configReader.open());

  ASSERT_FALSE(configReader.checkError());
}

/*
 * If the file xml has an error, it must have an error message.
 */
TEST(CheckErrorTest, InvalidFileHasError)
{
  ConfigReader configReader("data/invalid.xml");
  configReader.open();

  ASSERT_TRUE(configReader.checkError());
}

/*
 * If the file is valid, it must have no error message.
 */
TEST(GetErrorMessageTest, ValidFileHasNoError)
{
  ConfigReader configReader("data/config.xml");
  ASSERT_TRUE(configReader.open());

  ASSERT_EQ("", configReader.getErrorMsg());
}

/*
 * If the file xml has an error, it must have an error message.
 */
TEST(GetErrorMessageTest, InvalidFileHasError)
{
  ConfigReader configReader("data/invalid.xml");
  configReader.open();

  ASSERT_NE("", configReader.getErrorMsg());
}

/*
 * When reading from a valid file, the getDataTopic method must return the value
 * and set the found parameter to true.
 */
TEST(GetDataTopicTest, ReadValid)
{
  ConfigReader reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ("data", reader.getDataTopic());

  bool found = false;
  ASSERT_EQ("data", reader.getDataTopic(&found));
  ASSERT_TRUE(found);
}

/*
 * When reading from a valid file that does not define the topicdata, the
 * getDataTopic method must return an empty string and set the found parameter
 * to false.
 */
TEST(GetDataTopicTest, ReadNotFound)
{
  ConfigReader reader("data/minimal.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ("", reader.getDataTopic());

  bool found = false;
  ASSERT_EQ("", reader.getDataTopic(&found));
  ASSERT_FALSE(found);
}

/*
 * When reading from a valid file, the getStepTopic must return the value and
 * set the found parameter to true.
 */
TEST(GetStepTopicTest, ReadValid)
{
  ConfigReader reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ("Step", reader.getStepTopic());

  bool found = false;
  ASSERT_EQ("Step", reader.getStepTopic(&found));
  ASSERT_TRUE(found);
}

/*
 * When reading from a valid file that does not define the TopicoStep element, the
 * getStepTopic method must return an empty string and set the found parameter
 * to false.
 */
TEST(GetStepTopicTest, ReadNotFound)
{
  ConfigReader reader("data/minimal.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ("", reader.getStepTopic());

  bool found = false;
  ASSERT_EQ("", reader.getStepTopic(&found));
  ASSERT_FALSE(found);
}

/*
 * When reading from a valid file, the getControlStrategy must return the value and
 * set the found parameter to true.
 */
TEST(GetControlStrategyTest, ReadValid)
{
  ConfigReader reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ("liblqr_quadcopter.so", reader.getControlStrategy());

  bool found = false;
  ASSERT_EQ("liblqr_quadcopter.so", reader.getControlStrategy(&found));
  ASSERT_TRUE(found);
}

/*
 * When reading from a valid file that does not define the Strategy element, the
 * getControlStrategy method must return an empty string and set the found parameter
 * to false.
 */
TEST(GetControlStrategyTest, ReadNotFound)
{
  ConfigReader reader("data/minimal.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ("", reader.getControlStrategy());

  bool found = false;
  ASSERT_EQ("", reader.getControlStrategy(&found));
  ASSERT_FALSE(found);
}

/*
 * When reading from a valid file, the getTurbulenceModel must return the value and
 * set the found parameter to true.
 */
TEST(GetTurbulenceModelTest, ReadValid)
{
  ConfigReader reader("data/config2.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ("Von_Karman", reader.getTurbulenceModel());

  bool found = false;
  ASSERT_EQ("Von_Karman", reader.getTurbulenceModel(&found));
  ASSERT_TRUE(found);
}

/*
 * When reading from a valid file that does not define the Turbulance element, the
 * getTurbulenceModel method must return an empty string and set the found parameter
 * to false.
 */
TEST(GetTurbulenceModelTest, ReadNotFound)
{
  ConfigReader reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ("", reader.getTurbulenceModel());

  bool found = false;
  ASSERT_EQ("", reader.getTurbulenceModel(&found));
  ASSERT_FALSE(found);
}

/*
 * When reading from a valid file, the getReferenceFilePath must return the value and
 * set the found parameter to true.
 */
TEST(GetReferenceFilePathTest, ReadValid)
{
  ConfigReader reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ("ref.txt", reader.getReferenceFilePath());

  bool found = false;
  ASSERT_EQ("ref.txt", reader.getReferenceFilePath(&found));
  ASSERT_TRUE(found);
}

/*
 * When reading from a valid file that does not define the RefPath element, the
 * getReferenceFilePath method must return an empty string and set the found parameter
 * to false.
 */
TEST(GetReferenceFilePathTest, ReadNotFound)
{
  ConfigReader reader("data/minimal.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ("", reader.getReferenceFilePath());

  bool found = false;
  ASSERT_EQ("", reader.getReferenceFilePath(&found));
  ASSERT_FALSE(found);
}

/*
 * When reading from a valid file, the getOutputFilePath must return the value and
 * set the found parameter to true.
 */
TEST(GetOutputFilePathTest, ReadValid)
{
  ConfigReader reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ("out.txt", reader.getOutputFilePath());

  bool found = false;
  ASSERT_EQ("out.txt", reader.getOutputFilePath(&found));
  ASSERT_TRUE(found);
}

/*
 * When reading from a valid file that does not define the Outputfile element, the
 * getOutputFilePath method must return an empty string and set the found parameter
 * to false.
 */
TEST(GetOutputFilePathTest, ReadNotFound)
{
  ConfigReader reader("data/minimal.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ("", reader.getOutputFilePath());

  bool found = false;
  ASSERT_EQ("", reader.getOutputFilePath(&found));
  ASSERT_FALSE(found);
}

/*
 * When reading from a valid file, the getInputFilePath must return the value and
 * set the found parameter to true.
 */
TEST(GetInputFilePathTest, ReadValid)
{
  ConfigReader reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ("in.txt", reader.getInputFilePath());

  bool found = false;
  ASSERT_EQ("in.txt", reader.getInputFilePath(&found));
  ASSERT_TRUE(found);
}

/*
 * When reading from a valid file that does not define the InputPath element, the
 * getInputFilePath method must return an empty string and set the found parameter
 * to false.
 */
TEST(GetInputFilePathTest, ReadNotFound)
{
  ConfigReader reader("data/minimal.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ("", reader.getInputFilePath());

  bool found = false;
  ASSERT_EQ("", reader.getInputFilePath(&found));
  ASSERT_FALSE(found);
}

/*
 * When reading from a valid file, the getErrorFilePath must return the value and
 * set the found parameter to true.
 */
TEST(GetErrorFilePathTest, ReadValid)
{
  ConfigReader reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ("erro.txt", reader.getErrorFilePath());

  bool found = false;
  ASSERT_EQ("erro.txt", reader.getErrorFilePath(&found));
  ASSERT_TRUE(found);
}

/*
 * When reading from a valid file that does not define the ErroPath element, the
 * getErrorFilePath method must return an empty string and set the found parameter
 * to false.
 */
TEST(GetErrorFilePathTest, ReadNotFound)
{
  ConfigReader reader("data/minimal.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ("", reader.getErrorFilePath());

  bool found = false;
  ASSERT_EQ("", reader.getErrorFilePath(&found));
  ASSERT_FALSE(found);
}

/*
 * When reading from a valid file, the getSensors must return the list of values and
 * set the found parameter to true.
 */
TEST(GetSensorsTest, ReadValid)
{
  ConfigReader reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  ConfigReader::stringlist res;

  res = reader.getSensors();
  ASSERT_FALSE(res.empty());
  ASSERT_EQ(res.size(), 1);
  if (res.size() == 1)
    ASSERT_EQ(*res.begin(), "Estados");

  bool found = false;
  res = reader.getSensors(&found);
  ASSERT_FALSE(res.empty());
  ASSERT_EQ(res.size(), 1);
  ASSERT_TRUE(found);
  if (res.size() == 1)
    ASSERT_EQ(*res.begin(), "Estados");
}

/*
 * When reading from a valid file, the getSensors must return the list of values and
 * set the found parameter to true.
 */
TEST(GetSensorsTest, ReadMultiple)
{
  ConfigReader reader("data/config2.xml");
  ASSERT_TRUE(reader.open());

  ConfigReader::stringlist res;

  res = reader.getSensors();
  ASSERT_FALSE(res.empty());
  ASSERT_EQ(res.size(), 2);
  if (res.size() == 2)
  {
    auto it = res.begin();
    ASSERT_EQ(*it, "Estados1");
    ++it;
    ASSERT_EQ(*it, "Estados2");
  }

  bool found = false;
  res = reader.getSensors(&found);
  ASSERT_FALSE(res.empty());
  ASSERT_EQ(res.size(), 2);
  ASSERT_TRUE(found);
  if (res.size() == 2)
  {
    auto it = res.begin();
    ASSERT_EQ(*it, "Estados1");
    ++it;
    ASSERT_EQ(*it, "Estados2");
  }
}

/*
 * When reading from a valid file that does defines the Sensors element, but
 * has no Devices, the getSensors method must return an empty list and set the
 * found parameter to true.
 */
TEST(GetSensorsTest, ReadNoDevices)
{
  ConfigReader reader("data/config5.xml");
  ASSERT_TRUE(reader.open());

  ConfigReader::stringlist res;
  res = reader.getSensors();

  ASSERT_TRUE(res.empty());

  bool found = false;
  res = reader.getSensors(&found);
  ASSERT_TRUE(res.empty());
  ASSERT_TRUE(found);
}

/*
 * When reading from a valid file that does not define the Sensors element, the
 * getSensors method must return an empty string and set the found parameter
 * to false.
 */
TEST(GetSensorsTest, ReadNotFound)
{
  ConfigReader reader("data/minimal.xml");
  ASSERT_TRUE(reader.open());

  ConfigReader::stringlist res;
  res = reader.getSensors();

  ASSERT_TRUE(res.empty());

  bool found = false;
  res = reader.getSensors(&found);
  ASSERT_TRUE(res.empty());
  ASSERT_FALSE(found);
}

/*
 * When reading from a valid file, the getActuators must return the list of values and
 * set the found parameter to true.
 */
TEST(GetActuatorsTest, ReadValid)
{
  ConfigReader reader("data/config5.xml");
  ASSERT_TRUE(reader.open());

  ConfigReader::stringlist res;

  res = reader.getActuators();
  ASSERT_FALSE(res.empty());
  ASSERT_EQ(res.size(), 1);
  if (res.size() == 1)
    ASSERT_EQ(*res.begin(), "Empuxoh1");

  bool found = false;
  res = reader.getActuators(&found);
  ASSERT_FALSE(res.empty());
  ASSERT_EQ(res.size(), 1);
  ASSERT_TRUE(found);
  if (res.size() == 1)
    ASSERT_EQ(*res.begin(), "Empuxoh1");
}

/*
 * When reading from a valid file, the getActuators must return the list of values and
 * set the found parameter to true.
 */
TEST(GetActuatorsTest, ReadMultiple)
{
  ConfigReader reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  ConfigReader::stringlist res;

  res = reader.getActuators();
  ASSERT_FALSE(res.empty());
  ASSERT_EQ(res.size(), 4);
  if (res.size() == 4)
  {
    auto it = res.begin();
    ASSERT_EQ(*it, "Empuxoh1");
    ++it;
    ASSERT_EQ(*it, "Empuxoh2");
    ++it;
    ASSERT_EQ(*it, "Empuxoh3");
    ++it;
    ASSERT_EQ(*it, "Empuxoh4");
  }

  bool found = false;
  res = reader.getActuators(&found);
  ASSERT_FALSE(res.empty());
  ASSERT_EQ(res.size(), 4);
  ASSERT_TRUE(found);
  if (res.size() == 4)
  {
    auto it = res.begin();
    ASSERT_EQ(*it, "Empuxoh1");
    ++it;
    ASSERT_EQ(*it, "Empuxoh2");
    ++it;
    ASSERT_EQ(*it, "Empuxoh3");
    ++it;
    ASSERT_EQ(*it, "Empuxoh4");
  }
}

/*
 * When reading from a valid file that does defines the Actuators element, but
 * has no Devices, the getActuators method must return an empty list and set the
 * found parameter to true.
 */
TEST(GetActuatorsTest, ReadNoDevices)
{
  ConfigReader reader("data/config4.xml");
  ASSERT_TRUE(reader.open());

  ConfigReader::stringlist res;
  res = reader.getActuators();

  ASSERT_TRUE(res.empty());

  bool found = false;
  res = reader.getActuators(&found);
  ASSERT_TRUE(res.empty());
  ASSERT_TRUE(found);
}

/*
 * When reading from a valid file that does not define the Actuators element, the
 * getActuators method must return an empty string and set the found parameter
 * to false.
 */
TEST(GetActuatorsTest, ReadNotFound)
{
  ConfigReader reader("data/minimal.xml");
  ASSERT_TRUE(reader.open());

  ConfigReader::stringlist res;
  res = reader.getActuators();

  ASSERT_TRUE(res.empty());

  bool found = false;
  res = reader.getActuators(&found);
  ASSERT_TRUE(res.empty());
  ASSERT_FALSE(found);
}

/*
 * When reading from a valid file, the getSampleTime must return the list of values and
 * set the found parameter to true.
 */
TEST(GetSampleTime, ReadValid)
{
  ConfigReader reader("data/config5.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ(12, reader.getSampleTime());

  bool found = false;
  ASSERT_EQ(12, reader.getSampleTime(&found));
  ASSERT_TRUE(found);
}

/*
 * When reading from a valid file, that does not define the Sampletime element,
 * the getSampleTime method must return 0 and set the found parameter to false.
 */
TEST(GetSampleTime, ReadNotFound)
{
  ConfigReader reader("data/minimal.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ(0, reader.getSampleTime());

  bool found = false;
  ASSERT_EQ(0, reader.getSampleTime(&found));
  ASSERT_FALSE(found);
}

/*
 * When reading from a valid file that defines the StartPaused parameter as
 * true, the getStartPaused method must return true, and set the found parameter
 * to true.
 */
TEST(GetStartPausedTest, ReadTrue)
{
  ConfigReader reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_TRUE(reader.getStartPaused());

  bool found = false;
  ASSERT_TRUE(reader.getStartPaused(&found));
  ASSERT_TRUE(found);
}

/*
 * When reading from a valid file that defines the StartPaused parameter as
 * false, the getStartPaused method must return false, and set the found parameter
 * to true.
 */
TEST(GetStartPausedTest, ReadFalse)
{
  ConfigReader reader("data/config5.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_FALSE(reader.getStartPaused());

  bool found = false;
  ASSERT_FALSE(reader.getStartPaused(&found));
  ASSERT_TRUE(found);
}

/*
 * When reading from a valid file that does not define the StartPaused parameter,
 * the getStartPaused method must return true, and set the found parameter
 * to false.
 */
TEST(GetStartPausedTest, ReadNoutFound)
{
  ConfigReader reader("data/config1.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_TRUE(reader.getStartPaused());

  bool found = false;
  ASSERT_TRUE(reader.getStartPaused(&found));
  ASSERT_FALSE(found);
}

class TestProtectedMethods : public ConfigReader
{
public:
  TestProtectedMethods()
  {
  }
  TestProtectedMethods(const std::string& path) : ConfigReader(path)
  {
  }

  std::string toStdStringPublic(const char* str) const
  {
    return toStdString(str);
  }

  void setFoundResPublic(bool res, bool* found) const
  {
    setFoundRes(res, found);
  }

  ConfigReader::stringvector toStringvectorPublic(const ConfigReader::stringlist& list) const
  {
    return toStringvector(list);
  }

  const tinyxml2::XMLElement* getChildElementPublic(const std::string& name, bool* found) const
  {
    return getChildElement(name, found);
  }

  std::string getElementTextPublic(const std::string& name, bool* found) const
  {
    return getElementText(name, found);
  }

  ConfigReader::stringlist getChildElementValuesPublic(const std::string& name, const std::string& childName,
                                                       bool* found) const
  {
    return getChildElementValues(name, childName, found);
  }

  int getElementIntPublic(const std::string& name, bool* found) const
  {
    return getElementInt(name, found);
  }

  uint64_t getElementUint64Public(const std::string& name, bool* found) const
  {
    return getElementUint64(name, found);
  }

  std::string toLowerPublic(const std::string& str) const
  {
    return toLower(str);
  }

  std::string trimPublic(const std::string& str) const
  {
    return trim(str);
  }

  bool getElementBoolPublic(const std::string& name, bool* found) const
  {
    return getElementBool(name, found);
  }
};

/*
 * Test the behavior of the getElementBool when a true value is requested.
 */
TEST(ConfigReaderTest, GetElementBoolTrue)
{
  TestProtectedMethods reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_TRUE(reader.getElementBoolPublic("ShutdownWhenFinished", nullptr));

  bool res = false;
  ASSERT_TRUE(reader.getElementBoolPublic("ShutdownWhenFinished", &res));
  ASSERT_TRUE(res);

  ASSERT_TRUE(reader.getElementBoolPublic("ShutdownWhenFinished3", nullptr));
  ASSERT_TRUE(reader.getElementBoolPublic("ShutdownWhenFinished4", nullptr));
}

/*
 * Test the behavior of the getElementBool when a false value is requested.
 */
TEST(ConfigReaderTest, GetElementBoolFalse)
{
  TestProtectedMethods reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_FALSE(reader.getElementBoolPublic("ShutdownWhenFinished2", nullptr));

  bool res = false;
  ASSERT_FALSE(reader.getElementBoolPublic("ShutdownWhenFinished2", &res));
  ASSERT_TRUE(res);

  ASSERT_FALSE(reader.getElementBoolPublic("ShutdownWhenFinished5", nullptr));
}

/*
 * Test the behavior of the getElementBool when a false value is requested.
 */
TEST(ConfigReaderTest, GetElementBoolWrongTypes)
{
  TestProtectedMethods reader("data/wrongdatatype.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_FALSE(reader.getElementBoolPublic("ShutdownWhenFinished", nullptr));

  bool res = false;
  ASSERT_FALSE(reader.getElementBoolPublic("ShutdownWhenFinished", &res));
  ASSERT_FALSE(res);

  ASSERT_FALSE(reader.getElementBoolPublic("ShutdownWhenFinished2", nullptr));
  ASSERT_FALSE(reader.getElementBoolPublic("ShutdownWhenFinished3", nullptr));
  ASSERT_FALSE(reader.getElementBoolPublic("ShutdownWhenFinished4", nullptr));
}

TEST(ConfigReaderTest, ToLowerTest)
{
  std::string emptyString;
  std::string numericString = "12";
  std::string allSpacesString = "\t\n ";
  std::string lowerCaseString = "test";
  std::string upperCaseString = "TEST";
  std::string mixedCaseString = "TeSt";
  std::string mixedTypesString = "tesT 123";

  TestProtectedMethods reader;
  ASSERT_EQ("", reader.toLowerPublic(emptyString));
  ASSERT_EQ("12", reader.toLowerPublic(numericString));
  ASSERT_EQ(allSpacesString, reader.toLowerPublic(allSpacesString));
  ASSERT_EQ(lowerCaseString, reader.toLowerPublic(lowerCaseString));
  ASSERT_EQ(lowerCaseString, reader.toLowerPublic(upperCaseString));
  ASSERT_EQ(lowerCaseString, reader.toLowerPublic(mixedCaseString));
  ASSERT_EQ("test 123", reader.toLowerPublic(mixedTypesString));
}

TEST(ConfigReaderTest, TrimTest)
{
  std::string emptyString;
  std::string leftSpaceString = " test";
  std::string rightSpaceString = "test ";
  std::string numericString = "12";
  std::string bothSpacesString = " test ";
  std::string noSpacesString = "test";
  std::string tabString = "\ttest\t";
  std::string newLineString = "\n";
  std::string allSpacesString = "\n\t ";

  TestProtectedMethods reader;
  ASSERT_EQ("", reader.trimPublic(""));
  ASSERT_EQ("test", reader.trimPublic(leftSpaceString));
  ASSERT_EQ("test", reader.trimPublic(rightSpaceString));
  ASSERT_EQ("12", reader.trimPublic(numericString));
  ASSERT_EQ("test", reader.trimPublic(bothSpacesString));
  ASSERT_EQ("test", reader.trimPublic(noSpacesString));
  ASSERT_EQ("test", reader.trimPublic(tabString));
  ASSERT_EQ("", reader.trimPublic(newLineString));
  ASSERT_EQ("", reader.trimPublic(allSpacesString));
}

/*
 * Test the behavior of the getElementUint64 method when a valid number is requested.
 */
TEST(ConfigReaderTest, GetElementUint64Valid)
{
  TestProtectedMethods reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ(2500, reader.getElementUint64Public("Duration", nullptr));

  bool res = false;
  ASSERT_EQ(2500, reader.getElementUint64Public("Duration", &res));
  ASSERT_TRUE(res);
}

/*
 * Test the behavior of the getElementUint64 method when an element which does not
 * exist is requested.
 */
TEST(ConfigReaderTest, GetElementUint64NotFound)
{
  TestProtectedMethods reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ(0, reader.getElementUint64Public("any", nullptr));

  bool res = true;
  ASSERT_EQ(0, reader.getElementUint64Public("any", &res));
  ASSERT_FALSE(res);
}

/*
 * Test the behavior of the getElementUint64 method when a string valued element is requested.
 */
TEST(ConfigReaderTest, GetElementUint64WrongTypeStr)
{
  TestProtectedMethods reader("data/wrongdatatype.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ(0, reader.getElementUint64Public("Sampletime", nullptr));

  bool res = true;
  ASSERT_EQ(0, reader.getElementUint64Public("Sampletime", &res));
  ASSERT_FALSE(res);
}

/*
 * Test the behavior of the getElementUint64 method when a negative valued element is requested.
 */
TEST(ConfigReaderTest, GetElementUint64Negative)
{
  TestProtectedMethods reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ(0, reader.getElementUint64Public("Sampletime2", nullptr));

  bool res = true;
  ASSERT_EQ(0, reader.getElementUint64Public("Sampletime2", &res));
  ASSERT_FALSE(res);
}

/*
 * Test the behavior of the getElementUint64 method when a string valued element is requested.
 */
TEST(ConfigReaderTest, GetElementUint64Str)
{
  TestProtectedMethods reader("data/wrongdatatype.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ(0, reader.getElementUint64Public("Sampletime", nullptr));

  bool res = true;
  ASSERT_EQ(0, reader.getElementUint64Public("Sampletime", &res));
  ASSERT_FALSE(res);
}

/*
 * Test the behavior of the getElementUint64 method when a double valued element is requested.
 */
TEST(ConfigReaderTest, GetElementUint64Double)
{
  TestProtectedMethods reader("data/wrongdatatype.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ(0, reader.getElementUint64Public("Sampletime2", nullptr));

  bool res = true;
  ASSERT_EQ(0, reader.getElementUint64Public("Sampletime2", &res));
  ASSERT_FALSE(res);
}

/*
 * Test the behavior of the getElementInt method when a valid number is requested.
 */
TEST(ConfigReaderTest, GetElementIntValid)
{
  TestProtectedMethods reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ(12, reader.getElementIntPublic("Sampletime", nullptr));

  bool res = false;
  ASSERT_EQ(12, reader.getElementIntPublic("Sampletime", &res));
  ASSERT_TRUE(res);
}

/*
 * Test the behavior of the getElementInt method when a valid number is requested.
 */
TEST(ConfigReaderTest, GetElementIntNegative)
{
  TestProtectedMethods reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ(-12, reader.getElementIntPublic("Sampletime2", nullptr));

  bool res = false;
  ASSERT_EQ(-12, reader.getElementIntPublic("Sampletime2", &res));
  ASSERT_TRUE(res);
}

/*
 * Test the behavior of the getElementInt method when an element which does not
 * exist is passed.
 */
TEST(ConfigReaderTest, GetElementIntNotFound)
{
  TestProtectedMethods reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ(0, reader.getElementIntPublic("any", nullptr));

  bool res = true;
  ASSERT_EQ(0, reader.getElementIntPublic("any", &res));
  ASSERT_FALSE(res);
}

/*
 * Test what happens when the reader tries to convert a non numeric data type to an integer.
 */
TEST(ConfigReaderTest, GetElementIntWrongType)
{
  TestProtectedMethods reader("data/wrongdatatype.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ(0, reader.getElementIntPublic("Sampletime", nullptr));

  bool res = true;
  ASSERT_EQ(0, reader.getElementIntPublic("Sampletime", &res));
  ASSERT_FALSE(res);
}

/*
 * Test what happens when the reader tries to convert a floating point number to an integer.
 */
TEST(ConfigReaderTest, GetElementIntWrongTypeDouble)
{
  TestProtectedMethods reader("data/wrongdatatype.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ(0, reader.getElementIntPublic("Sampletime2", nullptr));

  bool res = true;
  ASSERT_EQ(0, reader.getElementIntPublic("Sampletime2", &res));
  ASSERT_FALSE(res);
}

TEST(ConfigReaderTest, GetChildElementValuesConfigTest)
{
  TestProtectedMethods reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  ConfigReader::stringlist resSensors, resActuators;

  resSensors = reader.getChildElementValuesPublic("Sensors", "Device", nullptr);
  ASSERT_EQ(resSensors.size(), 1);
  if (resSensors.size() == 1)
  {
    ASSERT_EQ("Estados", *(resSensors.cbegin()));
  }

  resActuators = reader.getChildElementValuesPublic("Actuators", "Device", nullptr);
  ASSERT_EQ(resActuators.size(), 4);
  int count = 0;
  for (auto it = resActuators.begin(); it != resActuators.end(); ++it)
  {
    switch (count)
    {
      case 0:
        ASSERT_EQ("Empuxoh1", *it);
        break;
      case 1:
        ASSERT_EQ("Empuxoh2", *it);
        break;
      case 2:
        ASSERT_EQ("Empuxoh3", *it);
        break;
      case 4:
        ASSERT_EQ("Empuxoh4", *it);
        break;
      default:
        break;
    }
    count++;
  }
}

TEST(ConfigReaderTest, GetChildValuesReadEmpty)
{
  TestProtectedMethods reader("data/childelements.xml");
  ASSERT_TRUE(reader.open());

  ConfigReader::stringlist emptyList;
  emptyList = reader.getChildElementValuesPublic("empty", "child", nullptr);

  ASSERT_TRUE(emptyList.empty());
}

TEST(ConfigReaderTest, GetChildValuesReadWrongChild)
{
  TestProtectedMethods reader("data/childelements.xml");
  ASSERT_TRUE(reader.open());

  ConfigReader::stringlist emptyList;
  emptyList = reader.getChildElementValuesPublic("oneChild", "child2", nullptr);

  ASSERT_TRUE(emptyList.empty());

  emptyList.clear();
  bool res = false;
  emptyList = reader.getChildElementValuesPublic("oneChild", "child2", &res);
  ASSERT_TRUE(res);
}

TEST(ConfigReaderTest, GetChildValuesReadCommon)
{
  TestProtectedMethods reader("data/childelements.xml");
  ASSERT_TRUE(reader.open());

  ConfigReader::stringlist res1, res2;
  bool res;
  res1 = reader.getChildElementValuesPublic("oneChild", "child", &res);

  ASSERT_TRUE(res);
  ASSERT_FALSE(res1.empty());
  ASSERT_EQ(res1.size(), 1);
  if (res1.size() == 1)
    ASSERT_EQ(*res1.begin(), "value");

  res = false;
  res2 = reader.getChildElementValuesPublic("twoChild", "child", &res);
  ASSERT_FALSE(res2.empty());
  ASSERT_EQ(res2.size(), 2);
  if (res2.size() == 2)
  {
    auto it = res2.begin();
    ASSERT_EQ(*it, "value1");
    ++it;
    ASSERT_EQ(*it, "value2");
  }
  ASSERT_TRUE(res);
}

TEST(ConfigReaderTest, GetChildValuesReadDiffChild)
{
  TestProtectedMethods reader("data/childelements.xml");
  ASSERT_TRUE(reader.open());

  ConfigReader::stringlist res1, res2;
  res1 = reader.getChildElementValuesPublic("twoChildDifferent", "child1", nullptr);

  ASSERT_FALSE(res1.empty());
  ASSERT_EQ(res1.size(), 1);
  if (res1.size() == 1)
    ASSERT_EQ(*res1.begin(), "value1");

  bool res = false;
  res2 = reader.getChildElementValuesPublic("twoChildDifferent", "child2", &res);
  ASSERT_FALSE(res2.empty());
  ASSERT_EQ(res2.size(), 1);
  if (res2.size() == 1)
    ASSERT_EQ(*res2.begin(), "value2");
  ASSERT_TRUE(res);
}

TEST(ConfigReaderTest, GetChildValuesSetFound)
{
  TestProtectedMethods reader("data/childelements.xml");
  ASSERT_TRUE(reader.open());

  ConfigReader::stringlist emptyList;
  bool res = false;
  emptyList = reader.getChildElementValuesPublic("oneChild", "child2", &res);

  ASSERT_TRUE(emptyList.empty());
  ASSERT_TRUE(res);
}

TEST(ConfigReaderTest, GetChildValuesNotFound)
{
  TestProtectedMethods reader("data/childelements.xml");
  ASSERT_TRUE(reader.open());

  ConfigReader::stringlist emptyList;
  bool res = true;
  emptyList = reader.getChildElementValuesPublic("any", "child", &res);

  ASSERT_TRUE(emptyList.empty());
  ASSERT_FALSE(res);
}

TEST(ConfigReaderTest, GetElementTextTest)
{
  TestProtectedMethods reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  // Valid element
  ASSERT_EQ("data", reader.getElementTextPublic("topicdata", nullptr));
  bool validRes = false;
  ASSERT_EQ("data", reader.getElementTextPublic("topicdata", &validRes));
  ASSERT_TRUE(validRes);

  // Invalid element
  ASSERT_EQ("", reader.getElementTextPublic("any", nullptr));
  bool invalidRes = true;
  ASSERT_EQ("", reader.getElementTextPublic("any", &invalidRes));
  ASSERT_FALSE(invalidRes);
}

/*
 * Check if the getChildElement method returns a valid pointer to the correct element when
 * a valid element and a valid file are passed.
 */
TEST(ConfigReaderTest, GetChildElementValidTest)
{
  TestProtectedMethods reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  bool found = false;
  const tinyxml2::XMLElement* element = reader.getChildElementPublic("topicdata", &found);

  ASSERT_TRUE(found);
  ASSERT_NE(element, nullptr);
  ASSERT_TRUE(element != NULL);
  if (element != nullptr)
  {
    const char* value = element->GetText();
    ASSERT_STREQ(value, "data");
  }
}

/*
 * Check the results of the getChildElement method when an
 */
TEST(ConfigReaderTest, GetChildElementInvalidTest)
{
  TestProtectedMethods reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  bool found = true;
  const tinyxml2::XMLElement* element = reader.getChildElementPublic("any", &found);

  ASSERT_FALSE(found);
  ASSERT_EQ(element, nullptr);
}

/*
 * The getChildElement method must not throw any exception if the found pointer is a nullptr.
 */
TEST(ConfigReaderTest, GetChildElementNullPtrTest)
{
  TestProtectedMethods reader("data/config.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_NO_THROW(const tinyxml2::XMLElement* element = reader.getChildElementPublic("any", nullptr));
  ASSERT_NO_THROW(const tinyxml2::XMLElement* element = reader.getChildElementPublic("topicdata", nullptr));
}

/*
 * Test if the toStdString converts an string correctly
 */
TEST(ConfigReaderTest, ToStdStringValid)
{
  TestProtectedMethods reader;

  const char* test = "TEST";
  ASSERT_EQ("TEST", reader.toStdStringPublic(test));
  ASSERT_EQ("", reader.toStdStringPublic(""));
}

/*
 * If the toStdString method receives a nullptr, it must return an empty string.
 */
TEST(ConfigReaderTest, ToSdtStringNull)
{
  TestProtectedMethods reader;

  const char* test = nullptr;
  ASSERT_EQ("", reader.toStdStringPublic(test));
}

TEST(ConfigReaderTest, SetFoundResTest)
{
  TestProtectedMethods reader;

  bool* testNull = nullptr;
  ASSERT_NO_THROW(reader.setFoundResPublic(true, testNull));
  ASSERT_NO_THROW(reader.setFoundResPublic(false, testNull));

  bool test = false;
  reader.setFoundResPublic(true, &test);
  ASSERT_TRUE(test);

  reader.setFoundResPublic(false, &test);
  ASSERT_FALSE(test);

  test = true;
  reader.setFoundResPublic(true, &test);
  ASSERT_TRUE(test);
  reader.setFoundResPublic(false, &test);
  ASSERT_FALSE(test);
}

TEST(ConfigReaderTest, ToStringVectorTest)
{
  ConfigReader::stringlist defList;
  ConfigReader::stringlist oneElementList;
  ConfigReader::stringlist twoElementList;

  oneElementList.push_back("test");

  twoElementList.push_back("test1");
  twoElementList.push_back("test2");

  TestProtectedMethods reader;

  ConfigReader::stringvector emptyVector, oneElementVector, twoElementVector;
  emptyVector = reader.toStringvectorPublic(defList);
  ASSERT_TRUE(emptyVector.empty());

  oneElementVector = reader.toStringvectorPublic(oneElementList);
  ASSERT_EQ(oneElementList.size(), oneElementVector.size());
  if (oneElementVector.size() == 1)
  {
    ASSERT_EQ(oneElementVector.at(0), "test");
  }

  twoElementVector = reader.toStringvectorPublic(twoElementList);
  ASSERT_EQ(twoElementVector.size(), 2);
  if (twoElementVector.size() == 2)
  {
    ASSERT_EQ(twoElementVector.at(0), "test1");
    ASSERT_EQ(twoElementVector.at(1), "test2");
  }
}

/*
 * When reading from a file which does not define the ShutdownWhenFinished element,
 * the getShutdownWhenFinished must return false.
 */
TEST(ShutdownWhenFinishedTest, ReadEmtpy)
{
  ConfigReader reader("data/config1.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_FALSE(reader.getShutdownWhenFinished());

  bool res = false;
  ASSERT_FALSE(reader.getShutdownWhenFinished(&res));
  ASSERT_FALSE(res);
}

/*
 * When reading from a file which defines the ShutdownWhenFinished element as true,
 * the getShutdownWhenFinished must return true.
 */
TEST(ShutdownWhenFinishedTest, ReadTrue)
{
  ConfigReader reader("data/config2.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_TRUE(reader.getShutdownWhenFinished());

  bool res = false;
  ASSERT_TRUE(reader.getShutdownWhenFinished(&res));
  ASSERT_TRUE(res);
}

/*
 * When reading from a file which defines the ShutdownWhenFinished element as false,
 * the getShutdownWhenFinished must return false.
 */
TEST(ShutdownWhenFinishedTest, ReadFalse)
{
  ConfigReader reader("data/config3.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_FALSE(reader.getShutdownWhenFinished());

  bool res = false;
  ASSERT_FALSE(reader.getShutdownWhenFinished(&res));
  ASSERT_TRUE(res);
}

/*
 * When reading from a file which defines the ShutdownWhenFinished element with a wrong data type, it must return false.
 */
TEST(ShutdownWhenFinishedTest, ReadWrongType)
{
  ConfigReader reader("data/config4.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_FALSE(reader.getShutdownWhenFinished());

  bool res = false;
  ASSERT_FALSE(reader.getShutdownWhenFinished(&res));
  ASSERT_FALSE(res);
}

/*
 * When reading from a file which does not defines the Duration element, the
 * method must return 0 and set the found parameter to false.
 */
TEST(DurationElementTest, ReadEmtpy)
{
  ConfigReader reader("data/config1.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ(0, reader.getSimulationDuration());

  bool res = true;
  ASSERT_EQ(0, reader.getSimulationDuration(&res));
  ASSERT_FALSE(res);
}

/*
 * When reading from a file which defines the Duration element to a value, the
 * method must return the value and set the found parameter to true.
 */
TEST(DurationElementTest, ReadValue)
{
  ConfigReader reader("data/config2.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ(2500, reader.getSimulationDuration());

  bool res = true;
  ASSERT_EQ(2500, reader.getSimulationDuration(&res));
  ASSERT_TRUE(res);
}

/*
 * When reading from a file which Duration element as zero, the
 * method must return 0 and set the found parameter to true.
 */
TEST(DurationElementTest, ReadZero)
{
  ConfigReader reader("data/config3.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ(0, reader.getSimulationDuration());

  bool res = true;
  ASSERT_EQ(0, reader.getSimulationDuration(&res));
  ASSERT_TRUE(res);
}

/*
 * When reading from a file which defines the Duration element as a negative number, the
 * method must return 0 and set the found parameter to false.
 */
TEST(DurationElementTest, ReadNegative)
{
  ConfigReader reader("data/config4.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ(0, reader.getSimulationDuration());

  bool res = true;
  ASSERT_EQ(0, reader.getSimulationDuration(&res));
  ASSERT_FALSE(res);
}

/*
 * When reading from a file which defines the Duration element as a floating point number, the
 * method must return 0 and set the found parameter to false.
 */
TEST(DurationElementTest, ReadDouble)
{
  ConfigReader reader("data/config5.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ(0, reader.getSimulationDuration());

  bool res = true;
  ASSERT_EQ(0, reader.getSimulationDuration(&res));
  ASSERT_FALSE(res);
}

TEST(BaudRateElementTest, TestValid)
{
  ConfigReader reader("data/config5.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ(921600, reader.getBaudRate());

  bool res = true;
  ASSERT_EQ(921600, reader.getBaudRate(&res));
  ASSERT_TRUE(res);
}

TEST(BaudRateElementTest, TestNotFound)
{
  ConfigReader reader("data/config1.xml");
  ASSERT_TRUE(reader.open());

  bool res = true;
  (void)reader.getBaudRate(&res);
  ASSERT_FALSE(res);
}

TEST(HilAsyncFlagTest, TestValid)
{
  ConfigReader reader("data/config5.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_TRUE(reader.getHilFlagAsynchronous());
}

TEST(HilSyncFlagTest, TestValid)
{
  ConfigReader reader("data/config5.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_TRUE(reader.getHilFlagSynchronous());
}

TEST(USART1Test, TestValid)
{
  ConfigReader reader("data/config5.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ("/dev/ttyUSB0", reader.getUsart1());
}

TEST(USART2Test, TestValid)
{
  ConfigReader reader("data/config5.xml");
  ASSERT_TRUE(reader.open());

  ASSERT_EQ("/dev/ttyUSB1", reader.getUsart2());
}

int main(int argc, char** argv)
{
  try
  {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
  }
  catch (const std::exception& e)
  {
    std::cerr << "Unhandled exception: " << e.what() << std::endl;
  }
}
