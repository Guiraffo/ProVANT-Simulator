/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file sdf_reader_test.cpp
 * @brief This file contains the tests of the SDFParser class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include <gtest/gtest.h>

#include <sdf/sdf.hh>

#include <iostream>
#include <memory>

#include "provant_simulator_sdf_parser/sdf_parser.h"

class SDFParserTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    _sdf.reset(new sdf::SDF());
    if (!sdf::init(_sdf))
    {
      throw std::runtime_error("Initializing the SDF pointer failed.");
    }

    if (!sdf::readFile("data/sdf_example.sdf", _sdf))
    {
      throw std::runtime_error("Reading the SDF file failed.");
    }

    _element = _sdf->Root();
    if (_element == nullptr)
    {
      throw std::runtime_error("Reading the SDF file failed");
    }
    _element = _element->GetElement("model")->GetElement("plugin");
    _testElement1 = _element->GetElement("testElement1");
    _reader1.reset(new SDFParser(_testElement1));

    _testElement2 = _element->GetElement("testElement2");
    _reader2.reset(new SDFParser(_testElement2));

    _testElement3 = _element->GetElement("testElement3");
    _reader3.reset(new SDFParser(_testElement3));
  }

  sdf::ElementPtr _element;
  sdf::ElementPtr _testElement1;
  sdf::ElementPtr _testElement2;
  sdf::ElementPtr _testElement3;

  std::unique_ptr<SDFParser> _reader1;
  std::unique_ptr<SDFParser> _reader2;
  std::unique_ptr<SDFParser> _reader3;

private:
  sdf::SDFPtr _sdf;
};

// Test if the element was actually read.
TEST_F(SDFParserTest, SanityTest)
{
  ASSERT_NE(_element, nullptr);
  ASSERT_EQ(_element->GetName(), "plugin");
}

// Check if the getElementPtr method is working
TEST_F(SDFParserTest, ElementPtr)
{
  ASSERT_EQ(_reader1->GetElementPtr(), _testElement1);
  ASSERT_EQ(_reader2->GetElementPtr(), _testElement2);
  ASSERT_EQ(_reader3->GetElementPtr(), _testElement3);
}

// Check that the HasAttribute method returns true for attributes that exist
TEST_F(SDFParserTest, TestHasAttributeTrue)
{
  ASSERT_TRUE(_reader1->HasAttribute("name"));
  ASSERT_TRUE(_reader2->HasAttribute("name"));
}

// Check that the HasAttribute method returns false for attributes that does not exist
TEST_F(SDFParserTest, TestHasAttributeFalse)
{
  ASSERT_FALSE(_reader1->HasAttribute("doesnotexist"));
  ASSERT_FALSE(_reader3->HasAttribute("testElement3"));
}

// Check that the HasElement method returns true for elements that exist
TEST_F(SDFParserTest, TestHasElementTrue)
{
  ASSERT_TRUE(_reader1->HasElement("name"));
  ASSERT_TRUE(_reader3->HasElement("name"));
}

// Check that the HasElement method returns true for elements that exist
TEST_F(SDFParserTest, TestHasElementFalse)
{
  ASSERT_FALSE(_reader1->HasElement("doesnotexist"));
  ASSERT_FALSE(_reader2->HasElement("name"));
}

// Check that the GetElementText method returns the correct value for elements that exist
TEST_F(SDFParserTest, TestGetElementTextTwoParamsExists)
{
  SDFStatus res;
  std::string value;
  ASSERT_NO_THROW(res = _reader1->GetElementText("name", &value));
  ASSERT_FALSE(res.isError());
  ASSERT_EQ(value, "nameElement");
}

// Check that the GetElementText method returns an error for elements that does not exist
TEST_F(SDFParserTest, TestGetElementTextTwoParamsNotExists)
{
  SDFStatus res;
  std::string value;
  ASSERT_NO_THROW(res = _reader2->GetElementText("name", &value));
  ASSERT_TRUE(res.isError());
  ASSERT_EQ(value, "");
}

// Check that the GetElementText clears the input parameter before returning the result
TEST_F(SDFParserTest, TestGetElementTextTwoParamsClear)
{
  SDFStatus res;
  std::string value = "initial_text";
  ASSERT_NO_THROW(res = _reader3->GetElementText("name", &value));
  ASSERT_FALSE(res.isError());
  ASSERT_EQ(value, "elementTest");
}

// Check that the GetElementText method returns the correct value for elements that exist
TEST_F(SDFParserTest, TestGetElementTextOneParamExists)
{
  std::string value;
  ASSERT_NO_THROW(value = _reader1->GetElementText("name"));
  ASSERT_EQ(value, "nameElement");
}

// Check that the GetElementText method throws an exception if the element does not exist
TEST_F(SDFParserTest, TestGetElementTextOneParamNotExists)
{
  std::string value;
  ASSERT_THROW(value = _reader2->GetElementText("name"), ElementNotFoundError);
  ASSERT_TRUE(value.empty());

  ASSERT_THROW(value = _reader3->GetElementText("doesnotexist"), ElementNotFoundError);
  ASSERT_TRUE(value.empty());
}

/*
 * Check if the methods are capable of retrieving the correct values from a SDF
 * element with contains an attribute and element with the same name, but with
 * different values.
 */
TEST_F(SDFParserTest, AttributeAndElementWithSameName)
{
  std::string attrRes, elementeRes;
  SDFStatus res;
  ASSERT_NO_THROW(res = _reader1->GetAttributeValue("name", &attrRes));
  ASSERT_FALSE(res.isError());
  ASSERT_EQ(attrRes, "nameAttr");

  attrRes.clear();
  ASSERT_TRUE(attrRes.empty());
  ASSERT_NO_THROW(attrRes = _reader1->GetAttributeValue("name"));
  ASSERT_EQ(attrRes, "nameAttr");

  ASSERT_NO_THROW(res = _reader1->GetElementText("name", &elementeRes));
  ASSERT_FALSE(res.isError());
  ASSERT_EQ(elementeRes, "nameElement");

  elementeRes.clear();
  ASSERT_TRUE(elementeRes.empty());
  ASSERT_NO_THROW(elementeRes = _reader1->GetElementText("name"));
  ASSERT_EQ(elementeRes, "nameElement");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  try
  {
    return RUN_ALL_TESTS();
  }
  catch (const std::exception& e)
  {
    std::cerr << "Unhandled exception: " << e.what() << '\n';
  }
}
