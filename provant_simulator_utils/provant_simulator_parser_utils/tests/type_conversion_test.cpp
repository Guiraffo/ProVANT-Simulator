/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file type_conversion.cpp
 * @brief This files contains the tests of the functions defined in the
 * type_conversion.h header.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_simulator_parser_utils/type_conversion.h"

#include <gtest/gtest.h>

#include <boost/algorithm/string.hpp>

#include <cmath>
#include <set>

std::set<std::string> generatePermutations(const std::string& str)
{
  std::set<std::string> output;

  std::size_t length = str.length();
  std::size_t maxPermutations = 1 << length;

  std::string lowerCaseStr = boost::algorithm::to_lower_copy(str);
  output.insert(lowerCaseStr);

  for (std::size_t i = 0; i < maxPermutations; i++)
  {
    std::string combination = lowerCaseStr;
    for (std::size_t j = 0; j < length; j++)
    {
      if (((i >> j) & 1) == 1)
      {
        combination[j] = std::toupper(str.at(j));
      }
    }
    output.insert(combination);
  }

  return output;
}

TEST(PermutatinsTest, TrivialTest)
{
  std::set<std::string> permutations = generatePermutations("abc");

  ASSERT_EQ(1, permutations.count("abc"));
  ASSERT_EQ(1, permutations.count("aBc"));
  ASSERT_EQ(1, permutations.count("aBC"));
  ASSERT_EQ(1, permutations.count("abC"));
  ASSERT_EQ(1, permutations.count("Abc"));
  ASSERT_EQ(1, permutations.count("ABc"));
  ASSERT_EQ(1, permutations.count("ABC"));
  ASSERT_EQ(1, permutations.count("AbC"));
}

TEST(ParseBoolTest, ReturnsTrue)
{
  std::set<std::string> permutations = generatePermutations("true");

  for (auto it = permutations.cbegin(); it != permutations.cend(); ++it)
  {
    ASSERT_TRUE(ParseBool(*it));
  }

  ASSERT_TRUE(ParseBool("1"));
}

TEST(ParseBoolTest, ReturnsFalse)
{
  std::set<std::string> permutations = generatePermutations("false ");

  for (auto it = permutations.cbegin(); it != permutations.cend(); ++it)
  {
    ASSERT_FALSE(ParseBool(*it));
  }

  ASSERT_FALSE(ParseBool("0"));
}

TEST(ParseBoolTest, ThrowsExceptionOnString)
{
  std::set<std::string> permutations = generatePermutations("test");

  for (auto it = permutations.cbegin(); it != permutations.cend(); ++it)
  {
    ASSERT_THROW(ParseBool(*it), std::invalid_argument);
  }
}

TEST(ParseBoolTest, ThrowsExceptionOnInteger)
{
  ASSERT_THROW(ParseBool("5"), std::invalid_argument);
  ASSERT_THROW(ParseBool("-1"), std::invalid_argument);
}

TEST(ParseBoolNoThrowTest, CanParseTrue)
{
  bool value, res;
  std::exception e;
  std::set<std::string> permutations = generatePermutations("true");
  permutations.insert("1");

  for (auto it = permutations.cbegin(); it != permutations.cend(); ++it)
  {
    ASSERT_NO_THROW(res = ParseBool(*it, value, e));
    ASSERT_TRUE(res);
    ASSERT_TRUE(value);
  }
}

TEST(ParseBoolNoThrowTest, CanParseFalse)
{
  bool value, res;
  std::exception e;
  std::set<std::string> permutations = generatePermutations("false");
  permutations.insert("0");

  for (auto it = permutations.cbegin(); it != permutations.cend(); ++it)
  {
    ASSERT_NO_THROW(res = ParseBool(*it, value, e));
    ASSERT_TRUE(res);
    ASSERT_FALSE(value);
  }
}

TEST(ParseBoolNoThrowTest, ReturnsFalseOnError)
{
  bool value, res;
  std::exception e;
  ASSERT_NO_THROW(res = ParseBool("teste", value, e));

  ASSERT_FALSE(res);
}

TEST(ParseBoolNoThrowTest, ValueFalseOnError)
{
  bool value, res;
  std::exception e;
  ASSERT_NO_THROW(res = ParseBool("teste", value, e));

  ASSERT_FALSE(value);
}

TEST(ParseBoolNoThrowTest, HasExceptionMessage)
{
  bool value, res;
  std::exception e;
  ASSERT_NO_THROW(res = ParseBool("teste", value, e));

  ASSERT_STRNE("", e.what());
}

TEST(ParseIntTest, TrivialTest)
{
  ASSERT_EQ(0, ParseInt("0"));
  ASSERT_EQ(1, ParseInt("1"));
  ASSERT_EQ(2, ParseInt("2"));
  ASSERT_EQ(-2, ParseInt("-2"));
  ASSERT_EQ(-3, ParseInt("-3"));
}

TEST(ParseIntTest, CanConvertLimits)
{
  ASSERT_EQ(std::numeric_limits<int>::min(), ParseInt("-2147483648"));
  ASSERT_EQ(std::numeric_limits<int>::max(), ParseInt("2147483647"));
}

TEST(ParseIntTest, CanConvertHexadecimal)
{
  ASSERT_EQ(9, ParseInt("0x9"));
  ASSERT_EQ(0xA, ParseInt("0xA"));
  ASSERT_EQ(0xA, ParseInt("0XA"));
  ASSERT_EQ(-0xA, ParseInt("-0xA"));
  ASSERT_EQ(-0xA, ParseInt("-0XA"));
}

TEST(ParseIntTest, CanConvertOctal)
{
  ASSERT_EQ(9, ParseInt("011"));
  ASSERT_EQ(-9, ParseInt("-011"));
}

TEST(ParseIntTest, ThrowExceptionOnOverflow)
{
  ASSERT_THROW(ParseInt("2147483648"), std::logic_error);
  ASSERT_THROW(ParseInt("-2147483649"), std::logic_error);
}

TEST(ParseIntTest, ThrowExceptionOnString)
{
  ASSERT_THROW(ParseInt("str"), std::logic_error);
}

TEST(ParseIntTest, ThrowExceptionOnEmpty)
{
  ASSERT_THROW(ParseInt(""), std::logic_error);
}

TEST(ParseIntNoThrowTest, ReturnsTrue)
{
  std::exception e;
  int value;
  bool res;
  ASSERT_NO_THROW(res = ParseInt("1", value, e));

  ASSERT_EQ(1, value);
  ASSERT_TRUE(res);
}

TEST(ParseIntNoThrowTest, ReturnsFalse)
{
  std::exception e;
  int value;
  bool res;
  ASSERT_NO_THROW(res = ParseInt("str", value, e));

  ASSERT_FALSE(res);
}

TEST(ParseIntNoThrowTest, HasExceptionMessage)
{
  std::exception e;
  int value;
  bool res;
  ASSERT_NO_THROW(res = ParseInt("str", value, e));

  ASSERT_STRNE("", e.what());
}

TEST(ParseLongIntTest, TrivialTest)
{
  ASSERT_EQ(0, ParseLongInt("0"));
  ASSERT_EQ(1, ParseLongInt("1"));
  ASSERT_EQ(2, ParseLongInt("2"));
  ASSERT_EQ(-2, ParseLongInt("-2"));
  ASSERT_EQ(-3, ParseLongInt("-3"));
}

TEST(ParseLongIntTest, CanConvertLimits)
{
  ASSERT_EQ(std::numeric_limits<long int>::min(), ParseLongInt("-9223372036854775808"));
  ASSERT_EQ(std::numeric_limits<long int>::max(), ParseLongInt("9223372036854775807"));
}

TEST(ParseLongIntTest, CanConvertHexadecimal)
{
  ASSERT_EQ(9, ParseLongInt("0x9"));
  ASSERT_EQ(0xA, ParseLongInt("0xA"));
  ASSERT_EQ(0xA, ParseLongInt("0XA"));
  ASSERT_EQ(-0xA, ParseLongInt("-0xA"));
  ASSERT_EQ(-0xA, ParseLongInt("-0XA"));
}

TEST(ParseLongIntTest, CanConvertOctal)
{
  ASSERT_EQ(9, ParseLongInt("011"));
  ASSERT_EQ(-9, ParseLongInt("-011"));
}

TEST(ParseLongIntTest, ThrowExceptionOnOverflow)
{
  ASSERT_THROW(ParseLongInt("-9223372036854775809"), std::logic_error);
  ASSERT_THROW(ParseLongInt("9223372036854775808"), std::logic_error);
}

TEST(ParseLongIntTest, ThrowExceptionOnString)
{
  ASSERT_THROW(ParseLongInt("str"), std::logic_error);
}

TEST(ParseLongIntTest, ThrowExceptionOnEmpty)
{
  ASSERT_THROW(ParseLongInt(""), std::logic_error);
}

TEST(ParseLongIntNoThrowTest, ReturnsTrue)
{
  std::exception e;
  long int value;
  bool res;
  ASSERT_NO_THROW(res = ParseLongInt("1", value, e));

  ASSERT_EQ(1, value);
  ASSERT_TRUE(res);
}

TEST(ParseLongIntNoThrowTest, ReturnsFalse)
{
  std::exception e;
  long int value;
  bool res;
  ASSERT_NO_THROW(res = ParseLongInt("str", value, e));

  ASSERT_FALSE(res);
}

TEST(ParseLongIntNoThrowTest, HasExceptionMessage)
{
  std::exception e;
  long int value;
  bool res;
  ASSERT_NO_THROW(res = ParseLongInt("str", value, e));

  ASSERT_STRNE("", e.what());
}

TEST(ParseUnsignedIntTest, TrivialTest)
{
  ASSERT_EQ(0, ParseUnsignedInt("0"));
  ASSERT_EQ(1, ParseUnsignedInt("1"));
  ASSERT_EQ(2, ParseUnsignedInt("2"));
}

TEST(ParseUnsignedIntTest, CanConvertLimits)
{
  ASSERT_EQ(std::numeric_limits<unsigned int>::min(), ParseUnsignedInt("0"));
  ASSERT_EQ(std::numeric_limits<unsigned int>::max(), ParseUnsignedInt("4294967295"));
}

TEST(ParseUnsignedIntTest, CanConvertHexadecimal)
{
  ASSERT_EQ(9, ParseUnsignedInt("0x9"));
  ASSERT_EQ(0xA, ParseUnsignedInt("0xA"));
  ASSERT_EQ(0xA, ParseUnsignedInt("0XA"));
}

TEST(ParseUnsignedIntTest, CanConvertOctal)
{
  ASSERT_EQ(9, ParseUnsignedInt("011"));
}

TEST(ParseUnsignedIntTest, ThrowExceptionOnOverflow)
{
  ASSERT_THROW(ParseUnsignedInt("4294967296"), std::logic_error);
}

TEST(ParseUnsignedIntTest, ThrowExceptionOnString)
{
  ASSERT_THROW(ParseUnsignedInt("str"), std::logic_error);
}

TEST(ParseUnsignedIntTest, ThrowExceptionOnEmpty)
{
  ASSERT_THROW(ParseUnsignedInt(""), std::logic_error);
}

TEST(ParseUnsignedIntTest, ThrowExceptionOnNegative)
{
  ASSERT_THROW(ParseUnsignedInt("-1"), std::logic_error);
}

TEST(ParseUnsignedIntNoThrowTest, ReturnsTrue)
{
  std::exception e;
  unsigned int value;
  bool res;
  ASSERT_NO_THROW(res = ParseUnsignedInt("1", value, e));

  ASSERT_EQ(1, value);
  ASSERT_TRUE(res);
}

TEST(ParseUnsignedIntNoThrowTest, ReturnsFalse)
{
  std::exception e;
  unsigned int value;
  bool res;
  ASSERT_NO_THROW(res = ParseUnsignedInt("str", value, e));

  ASSERT_FALSE(res);
}

TEST(ParseUnsignedIntNoThrowTest, HasExceptionMessage)
{
  std::exception e;
  unsigned int value;
  bool res;
  ASSERT_NO_THROW(res = ParseUnsignedInt("str", value, e));

  ASSERT_STRNE("", e.what());
}

TEST(ParseFloatTest, TrivialTest)
{
  ASSERT_FLOAT_EQ(1.0, ParseFloat("1.0"));
  ASSERT_FLOAT_EQ(2.0, ParseFloat("2.0"));
  ASSERT_FLOAT_EQ(3.0, ParseFloat("3.0"));
  ASSERT_FLOAT_EQ(-1.0, ParseFloat("-1.0"));
}

TEST(ParseFloatTest, CanConvertExponents)
{
  ASSERT_FLOAT_EQ(1e-1, ParseFloat("1e-1"));
  ASSERT_FLOAT_EQ(-1e-1, ParseFloat("-1e-1"));
  ASSERT_FLOAT_EQ(1e1, ParseFloat("1e1"));
  ASSERT_FLOAT_EQ(-1e1, ParseFloat("-1e1"));
}

TEST(ParseFloatTest, CanConvertLimits)
{
  ASSERT_FLOAT_EQ(std::numeric_limits<float>::epsilon(), ParseFloat("1.192092896e-7"));
  ASSERT_FLOAT_EQ(std::numeric_limits<float>::lowest(), ParseFloat("-3.402823466e+38"));
  ASSERT_FLOAT_EQ(std::numeric_limits<float>::min(), ParseFloat("1.175494351e-38"));
  ASSERT_FLOAT_EQ(-std::numeric_limits<float>::min(), ParseFloat("-1.175494351e-38"));
  ASSERT_FLOAT_EQ(std::numeric_limits<float>::max(), ParseFloat("3.402823466e+38"));
  ASSERT_FLOAT_EQ(-std::numeric_limits<float>::max(), ParseFloat("-3.402823466e+38"));
}

TEST(ParseFloatTest, CanConvertInfinity)
{
  std::set<std::string> permutations = generatePermutations("inf");

  for (auto it = permutations.cbegin(); it != permutations.cend(); ++it)
  {
    ASSERT_TRUE(std::isinf(ParseFloat(*it)));
  }

  permutations = generatePermutations("infinity");

  for (auto it = permutations.cbegin(); it != permutations.cend(); ++it)
  {
    ASSERT_TRUE(std::isinf(ParseFloat(*it)));
  }

  permutations = generatePermutations("-inf");

  for (auto it = permutations.cbegin(); it != permutations.cend(); ++it)
  {
    ASSERT_TRUE(std::isinf(ParseFloat(*it)));
  }

  permutations = generatePermutations("-infinity");

  for (auto it = permutations.cbegin(); it != permutations.cend(); ++it)
  {
    ASSERT_TRUE(std::isinf(ParseFloat(*it)));
  }
}

TEST(ParseFloatTest, CanConvertNAN)
{
  std::set<std::string> permutations = generatePermutations("-nan");

  for (auto it = permutations.cbegin(); it != permutations.cend(); ++it)
  {
    ASSERT_TRUE(std::isnan(ParseFloat(*it)));
  }
}

TEST(ParseFloatTest, ThrowExceptionOnOverflow)
{
  ASSERT_THROW(ParseFloat("1.175494351e-39"), std::logic_error);
  ASSERT_THROW(ParseFloat("1.175494351e-39"), std::logic_error);
  ASSERT_THROW(ParseFloat("4.402823466e+38"), std::logic_error);
  ASSERT_THROW(ParseFloat("-4.402823466e+38"), std::logic_error);
}

TEST(ParseFloatTest, ThrowExceptionOnString)
{
  ASSERT_THROW(ParseFloat("str"), std::logic_error);
}

TEST(ParseFloatTest, ThrowExceptionOnEmpty)
{
  ASSERT_THROW(ParseFloat(""), std::logic_error);
}

TEST(ParseFloatNothrowTest, ReturnsTrue)
{
  std::exception e;
  float value;
  bool res;
  ASSERT_NO_THROW(res = ParseFloat("1", value, e));

  ASSERT_FLOAT_EQ(1.0, value);
  ASSERT_TRUE(res);
}

TEST(ParseFloatNothrowTest, ReturnsFalse)
{
  std::exception e;
  float value;
  bool res;
  ASSERT_NO_THROW(res = ParseFloat("str", value, e));

  ASSERT_FALSE(res);
}

TEST(ParseFloatNothrowTest, ReturnsNan)
{
  std::exception e;
  float value;
  bool res;
  ASSERT_NO_THROW(res = ParseFloat("str", value, e));

  ASSERT_TRUE(std::isnan(value));
}

TEST(ParseFloatNothrowTest, HasExceptionMessage)
{
  std::exception e;
  float value;
  bool res;
  ASSERT_NO_THROW(res = ParseFloat("str", value, e));

  ASSERT_STRNE("", e.what());
}

TEST(ParseDoubleTest, TrivialTest)
{
  ASSERT_DOUBLE_EQ(1.0, ParseDouble("1.0"));
  ASSERT_DOUBLE_EQ(2.0, ParseDouble("2.0"));
  ASSERT_DOUBLE_EQ(3.0, ParseDouble("3.0"));
  ASSERT_DOUBLE_EQ(-1.0, ParseDouble("-1.0"));
}

TEST(ParseDoubleTest, CanConvertExponents)
{
  ASSERT_DOUBLE_EQ(1e-1, ParseDouble("1e-1"));
  ASSERT_DOUBLE_EQ(-1e-1, ParseDouble("-1e-1"));
  ASSERT_DOUBLE_EQ(1e1, ParseDouble("1e1"));
  ASSERT_DOUBLE_EQ(-1e1, ParseDouble("-1e1"));
}

TEST(ParseDoubleTest, CanConvertLimits)
{
  ASSERT_DOUBLE_EQ(std::numeric_limits<double>::epsilon(), ParseDouble("2.2204460492503131e-016"));
  ASSERT_DOUBLE_EQ(std::numeric_limits<double>::lowest(), ParseDouble("-1.7976931348623158e+308"));
  ASSERT_DOUBLE_EQ(std::numeric_limits<double>::min(), ParseDouble("2.2250738585072014e-308"));
  ASSERT_DOUBLE_EQ(-std::numeric_limits<double>::min(), ParseDouble("-2.2250738585072014e-308"));
  ASSERT_DOUBLE_EQ(std::numeric_limits<double>::max(), ParseDouble("1.7976931348623158e+308"));
  ASSERT_DOUBLE_EQ(-std::numeric_limits<double>::max(), ParseDouble("-1.7976931348623158e+308"));
}

TEST(ParseDoubleTest, CanConvertInfinity)
{
  std::set<std::string> permutations = generatePermutations("inf");

  for (auto it = permutations.cbegin(); it != permutations.cend(); ++it)
  {
    ASSERT_TRUE(std::isinf(ParseDouble(*it)));
  }

  permutations = generatePermutations("infinity");

  for (auto it = permutations.cbegin(); it != permutations.cend(); ++it)
  {
    ASSERT_TRUE(std::isinf(ParseDouble(*it)));
  }

  permutations = generatePermutations("-inf");

  for (auto it = permutations.cbegin(); it != permutations.cend(); ++it)
  {
    ASSERT_TRUE(std::isinf(ParseDouble(*it)));
  }

  permutations = generatePermutations("-infinity");

  for (auto it = permutations.cbegin(); it != permutations.cend(); ++it)
  {
    ASSERT_TRUE(std::isinf(ParseDouble(*it)));
  }
}

TEST(ParseDoubleTest, CanConvertNAN)
{
  std::set<std::string> permutations = generatePermutations("-nan");

  for (auto it = permutations.cbegin(); it != permutations.cend(); ++it)
  {
    ASSERT_TRUE(std::isnan(ParseDouble(*it)));
  }
}

TEST(ParseDoubleTest, ThrowExceptionOnOverflow)
{
  ASSERT_THROW(ParseDouble("2.2250738585072014e-309"), std::logic_error);
  ASSERT_THROW(ParseDouble("1.7976931348623158e+309"), std::logic_error);
  ASSERT_THROW(ParseDouble("1.2250738585072014e-308"), std::logic_error);
  ASSERT_THROW(ParseDouble("2.7976931348623158e+308"), std::logic_error);
}

TEST(ParseDoubleTest, ThrowExceptionOnString)
{
  ASSERT_THROW(ParseDouble("str"), std::logic_error);
}

TEST(ParseDoubleTest, ThrowExceptionOnEmpty)
{
  ASSERT_THROW(ParseDouble(""), std::logic_error);
}

TEST(ParseDoubleNothrowTest, ReturnsTrue)
{
  std::exception e;
  double value;
  bool res;
  ASSERT_NO_THROW(res = ParseDouble("1", value, e));

  ASSERT_DOUBLE_EQ(1.0, value);
  ASSERT_TRUE(res);
}

TEST(ParseDoubleNothrowTest, ReturnsFalse)
{
  std::exception e;
  double value;
  bool res;
  ASSERT_NO_THROW(res = ParseDouble("str", value, e));

  ASSERT_FALSE(res);
}

TEST(ParseDoubleNothrowTest, ReturnsNan)
{
  std::exception e;
  double value;
  bool res;
  ASSERT_NO_THROW(res = ParseDouble("str", value, e));

  ASSERT_TRUE(std::isnan(value));
}

TEST(ParseDoubleNothrowTest, HasExceptionMessage)
{
  std::exception e;
  double value;
  bool res;
  ASSERT_NO_THROW(res = ParseDouble("str", value, e));

  ASSERT_STRNE("", e.what());
}

TEST(ParseLongDoubleTest, TrivialTest)
{
  ASSERT_DOUBLE_EQ(1.0, ParseLongDouble("1.0"));
  ASSERT_DOUBLE_EQ(2.0, ParseLongDouble("2.0"));
  ASSERT_DOUBLE_EQ(3.0, ParseLongDouble("3.0"));
  ASSERT_DOUBLE_EQ(-1.0, ParseLongDouble("-1.0"));
}

TEST(ParseLongDoubleTest, CanConvertExponents)
{
  ASSERT_DOUBLE_EQ(1e-1, ParseLongDouble("1e-1"));
  ASSERT_DOUBLE_EQ(-1e-1, ParseLongDouble("-1e-1"));
  ASSERT_DOUBLE_EQ(1e1, ParseLongDouble("1e1"));
  ASSERT_DOUBLE_EQ(-1e1, ParseLongDouble("-1e1"));
}

TEST(ParseLongDoubleTest, CanConvertLimits)
{
  ASSERT_DOUBLE_EQ(std::numeric_limits<long double>::epsilon(), ParseLongDouble("1.0842021724855044e-19"));
  ASSERT_DOUBLE_EQ(std::numeric_limits<long double>::lowest(), ParseLongDouble("-1.7976931348623158e+308"));
  ASSERT_DOUBLE_EQ(std::numeric_limits<long double>::min(), ParseLongDouble("0"));
  ASSERT_DOUBLE_EQ(-std::numeric_limits<long double>::min(), ParseLongDouble("-0"));
  ASSERT_DOUBLE_EQ(std::numeric_limits<long double>::max(), ParseLongDouble("1.7976931348623158e+308"));
  ASSERT_DOUBLE_EQ(-std::numeric_limits<long double>::max(), ParseLongDouble("-1.7976931348623158e+308"));
}

TEST(ParseLongDoubleTest, CanConvertInfinity)
{
  std::set<std::string> permutations = generatePermutations("inf");

  for (auto it = permutations.cbegin(); it != permutations.cend(); ++it)
  {
    ASSERT_TRUE(std::isinf(ParseLongDouble(*it)));
  }

  permutations = generatePermutations("infinity");

  for (auto it = permutations.cbegin(); it != permutations.cend(); ++it)
  {
    ASSERT_TRUE(std::isinf(ParseLongDouble(*it)));
  }

  permutations = generatePermutations("-inf");

  for (auto it = permutations.cbegin(); it != permutations.cend(); ++it)
  {
    ASSERT_TRUE(std::isinf(ParseLongDouble(*it)));
  }

  permutations = generatePermutations("-infinity");

  for (auto it = permutations.cbegin(); it != permutations.cend(); ++it)
  {
    ASSERT_TRUE(std::isinf(ParseLongDouble(*it)));
  }
}

TEST(ParseLongDoubleTest, CanConvertNAN)
{
  std::set<std::string> permutations = generatePermutations("-nan");

  for (auto it = permutations.cbegin(); it != permutations.cend(); ++it)
  {
    ASSERT_TRUE(std::isnan(ParseLongDouble(*it)));
  }
}

TEST(ParseLongDoubleTest, ThrowExceptionOnOverflow)
{
  ASSERT_THROW(ParseLongDouble("218973149535723176502126385303097020516906332229462420044032373389173700552297072261641"
                               "029033652888285354569780749557731442744315367028843419812557385374367867359320070697326"
                               "320191591828296152436552951064679108661431179063216977883889613478656060039914875343321"
                               "145491116008867984515486651285234014977303760000912547939396622315138362241783854274391"
                               "783813871780588948754057516822634765923557697480511372564902088485522249479139937758502"
                               "601177354918009979622602685950855888360815984690023564513234659447638493985927645628457"
                               "966177293040780660922910271504608538808795932778162298682754783076808004015069494230341"
                               "172895777710033571401055977524212405734700738625166011082837911962300846927720096515350"
                               "020847447079244384854591288672300061908512647211195136146752763351956292759795725027800"
                               "298079590419313960302147099703527646744553092202267965628099149823208332964124103850923"
                               "918473478612192169721054348428704835340811304257300221642134891734717423480071488075100"
                               "206439051723424765600472176809648610799494341570347632064355862420744350442438056613601"
                               "760883747816538902780957697597728686007148702828795556714140463261583262360276289631617"
                               "397848425448686060994827086796804807870251185893083854658422304090880599629459458620190"
                               "376604844679092600222541053077590106576067134720012584640695703025713896098375799892695"
                               "455305236856075868317922311363951946885088077187210470520395758748001314313144425494391"
                               "994017575316933939236688185618912993172910425292123683515992232205099800167710278403536"
                               "014082929639811512287776813570604578934353545169653956125404884644716978689321167108722"
                               "908808277835051822885764606221873970285165508372099234948333443522898475123275372663606"
                               "621390228126470623407535207172405866507951821730346378263135339370677490195019784169044"
                               "182473806316282858685774143258116536404021840272491339332094921949842244273042701987304"
                               "453662035026238695780468200360144729199712309553005720614186697485284685618651483271597"
                               "448120312194675168637934309618961510733006555242148519520176285859509105183947250286387"
                               "163249416761380499631979144187025430270675849519200883791516940158174004671147787720145"
                               "964446117520405945350476472180797576111172084627363927960033967047003761337450955318415"
                               "007379641260504792325166135484129188421134082301547330475406707281876350361733290800595"
                               "189632520707167390454777712968226520622565143991937680440029238090311243791261477625596"
                               "469422198137514696707944687035800439250765945161837981185939204954403611491531078225107"
                               "269148697980924094677214272701240437718740921675661363493890045123235166814608932240069"
                               "799317601780533819184998193300841098599393876029260139091141452600372028487213241195542"
                               "428210183120421610446740462163533690058366460659115629876474552506814500393294140413149"
                               "540067760295100596225302282300363147382468105964844244132486457313743759509641616804802"
                               "412935187620466813563687753281467553879887177183651289394719533506188500326760735438867"
                               "336800207438784965701457609034985757124304510203873049485425670247933932280911052604153"
                               "852899484920399109194612991249163328991799809438033787952209313146694614970593966415237"
                               "594928589096048991612194498998638483702248667224914892467841020618336462741696957630763"
                               "248023558797524525373703543388296086275342774001633343405508353704850737454481975472222"
                               "897528108302089868263302028525992308416805453968791141829762998896457648276528750456285"
                               "492426516521775079951625966922911497778896235667095662713848201819134832168799586365263"
                               "762097828507009933729439678463987902491451422274252700636394232799848397673998715441855"
                               "420156224415492665301451550468548925862027608576183712976335876121538256512963353814166"
                               "394951655600026415918655485005705261143195291991880795452239464962763563017858089669222"
                               "640623538289853586759599064700838568712381032959192649484625076899225841930548076362021"
                               "508902214922052806984201835084058693849381549890944546197789302911357651677540623227829"
                               "831403347327660395223160342282471752818181884430488092132193355086987339586127607367086"
                               "665237555567580317149010847732009642431878007000879734603290627894355374356444885190719"
                               "161645514115576193939969076741515640282654366402676009508752394550734155613586793306603"
                               "174472092444651353236664764973540085196704077110364053815007348689179836404957060618953"
                               "500508984091382686953509006678332447257871219660441528492484004185093281190896363417573"
                               "989716659600075948780061916409485433875852065711654107226099628815012314437794400874930"
                               "194474433078438899570184271000480830501217712356062289507626904285680004771889315808935"
                               "851559386317665294808903126774702966254511086154895839508779675546413794489596052797520"
                               "987481383976257859210575628440175934932416214833956535018919681138909184379573470326940"
                               "634289008780584694035245347939808067427323629788710086717580253156130235606487870925986"
                               "528841635097252953709111431720488774740553905400942537542411931794417513706468964386151"
                               "771884986701034153254238591108962471088538580868883777725864856414593426212108664758848"
                               "9260031762345960769508849149662444156604419552086811989770240.000000"),
               std::logic_error);
}

TEST(ParseLongDoubleTest, ThrowExceptionOnString)
{
  ASSERT_THROW(ParseLongDouble("str"), std::logic_error);
}

TEST(ParseLongDoubleTest, ThrowExceptionOnEmpty)
{
  ASSERT_THROW(ParseLongDouble(""), std::logic_error);
}

TEST(ParseLongDoubleNothrowTest, ReturnsTrue)
{
  std::exception e;
  long double value;
  bool res;
  ASSERT_NO_THROW(res = ParseLongDouble("1", value, e));

  ASSERT_DOUBLE_EQ(1.0, value);
  ASSERT_TRUE(res);
}

TEST(ParseLongDoubleNothrowTest, ReturnsFalse)
{
  std::exception e;
  long double value;
  bool res;
  ASSERT_NO_THROW(res = ParseLongDouble("str", value, e));

  ASSERT_FALSE(res);
}

TEST(ParseLongDoubleNothrowTest, ReturnsNan)
{
  std::exception e;
  long double value;
  bool res;
  ASSERT_NO_THROW(res = ParseLongDouble("str", value, e));

  ASSERT_TRUE(std::isnan(value));
}

TEST(ParseLongDoubleNothrowTest, HasExceptionMessage)
{
  std::exception e;
  long double value;
  bool res;
  ASSERT_NO_THROW(res = ParseLongDouble("str", value, e));

  ASSERT_STRNE("", e.what());
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
    std::cerr << "Unhandled exception with message: " << e.what() << '\n';
  }
}
