#include "provant_simulator_math_utils/saturator.h"

#include <gtest/gtest.h>

class SaturatorTest : public ::testing::Test
{
protected:
  provant::math_utils::Saturator<double> saturator{ -10.0, 10.0, true };
};

TEST_F(SaturatorTest, InitializesLimInf)
{
  ASSERT_DOUBLE_EQ(-10.0, saturator.getLimInf());
}

TEST_F(SaturatorTest, InitializesLimSup)
{
  ASSERT_DOUBLE_EQ(10.0, saturator.getLimSup());
}

TEST_F(SaturatorTest, SaturationEnabled)
{
  ASSERT_DOUBLE_EQ(-10.0, saturator.saturate(-20));
  ASSERT_DOUBLE_EQ(10.0, saturator.saturate(20));
  ASSERT_DOUBLE_EQ(5.0, saturator.saturate(5.0));
}

TEST_F(SaturatorTest, SaturationDisabled)
{
  saturator.setEnabled(false);

  ASSERT_DOUBLE_EQ(-20.0, saturator.saturate(-20.0));
  ASSERT_DOUBLE_EQ(20.0, saturator.saturate(20.0));
  ASSERT_DOUBLE_EQ(5.0, saturator.saturate(5.0));
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
    std::cerr << "An unexpected exception with message\"" << e.what() << "\" was catched during the test execution.";
  }
}
