/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file sdf_status_tests.cpp
 * @brief This file contains the tests of the SDFStatus class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include <gtest/gtest.h>

#include <iostream>
#include <memory>

#include "provant_simulator_sdf_parser/sdf_status.h"

class SDFStatusTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    def.reset(new SDFStatus);
    isErrorNoMsg.reset(new SDFStatus(true));
    isErrorMsg.reset(new SDFStatus(true, "testMessage"));
    notError.reset(new SDFStatus(false));
    notErrorMsg.reset(new SDFStatus(false, "not an error"));
  }

  std::unique_ptr<SDFStatus> def;
  std::unique_ptr<SDFStatus> isErrorNoMsg;
  std::unique_ptr<SDFStatus> isErrorMsg;
  std::unique_ptr<SDFStatus> notError;
  std::unique_ptr<SDFStatus> notErrorMsg;
};

TEST_F(SDFStatusTest, DefaultConstructorTest)
{
  ASSERT_TRUE(def->isError());
  ASSERT_TRUE(def->errorMessage().empty());
}

TEST_F(SDFStatusTest, IsError)
{
  ASSERT_TRUE(isErrorNoMsg->isError());
  ASSERT_TRUE(isErrorMsg->isError());
}

TEST_F(SDFStatusTest, IsNotError)
{
  ASSERT_FALSE(notError->isError());
  ASSERT_FALSE(notErrorMsg->isError());
}

TEST_F(SDFStatusTest, HasMessage)
{
  ASSERT_FALSE(isErrorMsg->errorMessage().empty());
  ASSERT_FALSE(notErrorMsg->errorMessage().empty());
}

TEST_F(SDFStatusTest, HasEmptyMessage)
{
  ASSERT_TRUE(isErrorNoMsg->errorMessage().empty());
  ASSERT_TRUE(notError->errorMessage().empty());
}

TEST_F(SDFStatusTest, WhatEqualsErrorMessage)
{
  ASSERT_STREQ(def->errorMessage().c_str(), def->what());
  ASSERT_STREQ(isErrorNoMsg->errorMessage().c_str(), isErrorNoMsg->what());
  ASSERT_STREQ(isErrorMsg->errorMessage().c_str(), isErrorMsg->what());
  ASSERT_STREQ(notError->errorMessage().c_str(), notError->what());
  ASSERT_STREQ(notErrorMsg->errorMessage().c_str(), notErrorMsg->what());
}

class OkStatusTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ok.reset(new OkStatus);
  }

  std::unique_ptr<OkStatus> ok;
};

TEST_F(OkStatusTest, IsNotAnError)
{
  ASSERT_FALSE(ok->isError());
}

TEST_F(OkStatusTest, HasNoErrorMessage)
{
  ASSERT_TRUE(ok->errorMessage().empty());
}

class NotFoundErrorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    notFound.reset(new NotFoundError("testName", "testType"));
  }

  std::unique_ptr<NotFoundError> notFound;
};

TEST_F(NotFoundErrorTest, IsError)
{
  ASSERT_TRUE(notFound->isError());
}

TEST_F(NotFoundErrorTest, HasErrorMessage)
{
  ASSERT_FALSE(notFound->errorMessage().empty());
}

TEST_F(NotFoundErrorTest, ErrorMessageHasName)
{
  ASSERT_TRUE(notFound->errorMessage().find("testName") != std::string::npos);
}

TEST_F(NotFoundErrorTest, ErrorMessageHasType)
{
  ASSERT_TRUE(notFound->errorMessage().find("testType") != std::string::npos);
}

class ElementNotFoundErrorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    elementNotFound.reset(new ElementNotFoundError("elementName"));
  }

  std::unique_ptr<ElementNotFoundError> elementNotFound;
};

TEST_F(ElementNotFoundErrorTest, IsError)
{
  ASSERT_TRUE(elementNotFound->isError());
}

TEST_F(ElementNotFoundErrorTest, HasErrorMessage)
{
  ASSERT_FALSE(elementNotFound->errorMessage().empty());
}

TEST_F(ElementNotFoundErrorTest, ErrorMessageHasName)
{
  ASSERT_TRUE(elementNotFound->errorMessage().find("elementName") != std::string::npos);
}

TEST_F(ElementNotFoundErrorTest, ErrorMessageHasType)
{
  ASSERT_TRUE(elementNotFound->errorMessage().find("element") != std::string::npos);
}

class AttributeNotFoundErrorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    attrNotFound.reset(new AttributeNotFoundError("attrName"));
  }

  std::unique_ptr<AttributeNotFoundError> attrNotFound;
};

TEST_F(AttributeNotFoundErrorTest, IsError)
{
  ASSERT_TRUE(attrNotFound->isError());
}

TEST_F(AttributeNotFoundErrorTest, HasErrorMessage)
{
  ASSERT_FALSE(attrNotFound->errorMessage().empty());
}

TEST_F(AttributeNotFoundErrorTest, ErrorMessageHasName)
{
  ASSERT_TRUE(attrNotFound->errorMessage().find("attrName") != std::string::npos);
}

TEST_F(AttributeNotFoundErrorTest, ErrorMessageHasType)
{
  ASSERT_TRUE(attrNotFound->errorMessage().find("attribute") != std::string::npos);
}

class XMLSyntaxErrorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    xmlError.reset(new XMLSyntaxError("testMessage"));
  }

  std::unique_ptr<XMLSyntaxError> xmlError;
};

TEST_F(XMLSyntaxErrorTest, IsError)
{
  ASSERT_TRUE(xmlError->isError());
}

TEST_F(XMLSyntaxErrorTest, HasErrorMessage)
{
  ASSERT_FALSE(xmlError->errorMessage().empty());
}

TEST_F(XMLSyntaxErrorTest, ErrorMessageHasMsg)
{
  ASSERT_TRUE(xmlError->errorMessage().find("testMessage") != std::string::npos);
}

class ConversionErrorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    err.reset(new ConversionError("attr", "teste", "double"));
    boolErr.reset(new ConversionError("element", "teste", "bool"));
    extraMsgErr.reset(new ConversionError("element", "teste", "int", "extraMsg"));
  }

  std::unique_ptr<ConversionError> err;
  std::unique_ptr<ConversionError> boolErr;
  std::unique_ptr<ConversionError> extraMsgErr;
};

TEST_F(ConversionErrorTest, IsError)
{
  ASSERT_TRUE(err->isError());
  ASSERT_TRUE(boolErr->isError());
  ASSERT_TRUE(extraMsgErr->isError());
}

TEST_F(ConversionErrorTest, HasErrorMessage)
{
  ASSERT_FALSE(err->errorMessage().empty());
  ASSERT_FALSE(boolErr->errorMessage().empty());
  ASSERT_FALSE(extraMsgErr->errorMessage().empty());
}

TEST_F(ConversionErrorTest, ErrorMessageHasElementType)
{
  ASSERT_TRUE(err->errorMessage().find("attr") != std::string::npos);
  ASSERT_TRUE(boolErr->errorMessage().find("element") != std::string::npos);
  ASSERT_TRUE(extraMsgErr->errorMessage().find("element") != std::string::npos);
}

TEST_F(ConversionErrorTest, ErrorMessageHasType)
{
  ASSERT_TRUE(err->errorMessage().find("double") != std::string::npos);
  ASSERT_TRUE(boolErr->errorMessage().find("bool") != std::string::npos);
  ASSERT_TRUE(extraMsgErr->errorMessage().find("int") != std::string::npos);
}

TEST_F(ConversionErrorTest, ErrorMessageHasOriginalContent)
{
  ASSERT_TRUE(err->errorMessage().find("teste") != std::string::npos);
  ASSERT_TRUE(boolErr->errorMessage().find("teste") != std::string::npos);
  ASSERT_TRUE(extraMsgErr->errorMessage().find("teste") != std::string::npos);
}

TEST_F(ConversionErrorTest, ErrorMessageHasExtraMsg)
{
  ASSERT_TRUE(extraMsgErr->errorMessage().find("extraMsg") != std::string::npos);
}

class NullPointerErrorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    err.reset(new NullPointerError("methodName"));
  }

  std::unique_ptr<NullPointerError> err;
};

TEST_F(NullPointerErrorTest, IsError)
{
  ASSERT_TRUE(err->isError());
}

TEST_F(NullPointerErrorTest, HasErrorMessage)
{
  ASSERT_FALSE(err->errorMessage().empty());
}

TEST_F(NullPointerErrorTest, ErrorMessageHasMethodName)
{
  ASSERT_TRUE(err->errorMessage().find("methodName") != std::string::npos);
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
