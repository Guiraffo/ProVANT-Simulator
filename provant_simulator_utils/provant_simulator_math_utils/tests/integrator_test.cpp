/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file integrator_tests.cpp
 * @brief This file contains the tests of the Integrator class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_simulator_math_utils/integrator.h"

#include <gtest/gtest.h>

#include <Eigen/Eigen>

#include <memory>

class IntegratorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    defaultIntegrator.reset(new Integrator<double>(1.0));

    lineIntegrator.reset(new Integrator<double>(1.0));
    lineIntegrator->update(0.0);
    lineIntegrator->update(1.0);
    lineIntegrator->update(2.0);
    lineIntegrator->update(3.0);
    lineIntegrator->update(4.0);
    lineIntegrator->update(5.0);

    nonZeroInitialValueIntegrator.reset(new Integrator<double>(1.0, 10.0));

    zero << 0,0,0,0;
    vectorIntegrator.reset(new Integrator<Eigen::Vector4d>(1.0, zero, zero));
  }

  void TearDown() override
  {
    defaultIntegrator.release();
    lineIntegrator.release();
    nonZeroInitialValueIntegrator.release();
    vectorIntegrator.release();
  }

  std::unique_ptr<Integrator<double>> defaultIntegrator;
  std::unique_ptr<Integrator<double>> lineIntegrator;
  std::unique_ptr<Integrator<double>> nonZeroInitialValueIntegrator;
  std::unique_ptr<Integrator<Eigen::Vector4d>> vectorIntegrator;
  Eigen::Vector4d zero;
};

TEST_F(IntegratorTest, InitializesStepTime)
{
  ASSERT_DOUBLE_EQ(1.0, defaultIntegrator->stepTime());
  ASSERT_DOUBLE_EQ(1.0, vectorIntegrator->stepTime());
}

TEST_F(IntegratorTest, InitializesInitialValue)
{
  ASSERT_DOUBLE_EQ(0.0, defaultIntegrator->value());
  ASSERT_DOUBLE_EQ(10.0, nonZeroInitialValueIntegrator->value());

  ASSERT_TRUE(vectorIntegrator->value().isApprox(zero));
}

TEST_F(IntegratorTest, IsInitiallyZero)
{
  ASSERT_DOUBLE_EQ(0.0, defaultIntegrator->value());
  ASSERT_DOUBLE_EQ(0.0, defaultIntegrator->previousValue());

  Eigen::Vector4d zero;
  zero << 0, 0, 0, 0;
  ASSERT_TRUE(vectorIntegrator->value().isApprox(zero));
}

TEST_F(IntegratorTest, UpdatesInitialValue)
{
  defaultIntegrator->setInitialValue(10.0);
  ASSERT_DOUBLE_EQ(10.0, defaultIntegrator->initialValue());
}

TEST_F(IntegratorTest, CanReset)
{
  lineIntegrator->reset();

  ASSERT_DOUBLE_EQ(lineIntegrator->initialValue(), lineIntegrator->value());
  ASSERT_DOUBLE_EQ(0.0, lineIntegrator->previousValue());

  nonZeroInitialValueIntegrator->reset();
  ASSERT_DOUBLE_EQ(10.0, nonZeroInitialValueIntegrator->value());
  ASSERT_DOUBLE_EQ(0.0, nonZeroInitialValueIntegrator->previousValue());

  Eigen::Vector4d sample;
  sample << 1, 1, 1, 1;
  vectorIntegrator->update(sample);
  vectorIntegrator->reset();
  ASSERT_TRUE(vectorIntegrator->value().isApprox(zero));
}

TEST_F(IntegratorTest, UpdatesPreviousValue)
{
  ASSERT_DOUBLE_EQ(0.0, defaultIntegrator->previousValue());

  defaultIntegrator->update(1.0);
  ASSERT_DOUBLE_EQ(1.0, defaultIntegrator->previousValue());
  defaultIntegrator->update(2.0);
  ASSERT_DOUBLE_EQ(2.0, defaultIntegrator->previousValue());
  defaultIntegrator->update(3.0);
  ASSERT_DOUBLE_EQ(3.0, defaultIntegrator->previousValue());
  defaultIntegrator->update(4.0);
  ASSERT_DOUBLE_EQ(4.0, defaultIntegrator->previousValue());
  defaultIntegrator->update(5.0);
  ASSERT_DOUBLE_EQ(5.0, defaultIntegrator->previousValue());
}

TEST_F(IntegratorTest, VectorUpdatesPreviousValue)
{
  Eigen::Vector4d sample;
  sample << 1, 1, 1, 1;

  vectorIntegrator->update(sample);
  ASSERT_TRUE(vectorIntegrator->previousValue().isApprox(sample));
}

TEST_F(IntegratorTest, ReturnsIntegralValue)
{
  ASSERT_DOUBLE_EQ(0.5, defaultIntegrator->update(1.0));
  ASSERT_DOUBLE_EQ(2.0, defaultIntegrator->update(2.0));
  ASSERT_DOUBLE_EQ(4.5, defaultIntegrator->update(3.0));
  ASSERT_DOUBLE_EQ(8.0, defaultIntegrator->update(4.0));
  ASSERT_DOUBLE_EQ(12.5, defaultIntegrator->update(5.0));
}

TEST_F(IntegratorTest, VectorReturnsIntegralValue)
{
  Eigen::Vector4d sample, res;
  sample << 1, 1, 1, 1;
  res << 0.5, 0.5, 0.5, 0.5;

  ASSERT_TRUE(vectorIntegrator->update(sample).isApprox(res));
}

TEST_F(IntegratorTest, UpdatesIntegralValue)
{
  ASSERT_DOUBLE_EQ(0.5, defaultIntegrator->update(1.0));
  ASSERT_DOUBLE_EQ(0.5, defaultIntegrator->value());
  ASSERT_DOUBLE_EQ(2.0, defaultIntegrator->update(2.0));
  ASSERT_DOUBLE_EQ(2.0, defaultIntegrator->value());
  ASSERT_DOUBLE_EQ(4.5, defaultIntegrator->update(3.0));
  ASSERT_DOUBLE_EQ(4.5, defaultIntegrator->value());
  ASSERT_DOUBLE_EQ(8.0, defaultIntegrator->update(4.0));
  ASSERT_DOUBLE_EQ(8.0, defaultIntegrator->value());
  ASSERT_DOUBLE_EQ(12.5, defaultIntegrator->update(5.0));
  ASSERT_DOUBLE_EQ(12.5, defaultIntegrator->value());
}

TEST_F(IntegratorTest, VectorUpdatesIntegralValue)
{
  Eigen::Vector4d sample, res;
  sample << 1, 1, 1, 1;
  res << 0.5, 0.5, 0.5, 0.5;

  vectorIntegrator->update(sample);
  ASSERT_TRUE(vectorIntegrator->value().isApprox(res));
}

TEST_F(IntegratorTest, DecreasingLineIntegral)
{
  ASSERT_DOUBLE_EQ(0.0, defaultIntegrator->value());

  ASSERT_DOUBLE_EQ(-0.5, defaultIntegrator->update(-1.0));
  ASSERT_DOUBLE_EQ(-2.0, defaultIntegrator->update(-2.0));
  ASSERT_DOUBLE_EQ(-4.5, defaultIntegrator->update(-3.0));
  ASSERT_DOUBLE_EQ(-8.0, defaultIntegrator->update(-4.0));
  ASSERT_DOUBLE_EQ(-12.5, defaultIntegrator->update(-5.0));
}

TEST_F(IntegratorTest, NonZeroStartValueIntegralTest)
{
  ASSERT_DOUBLE_EQ(10.0, nonZeroInitialValueIntegrator->value());

  ASSERT_DOUBLE_EQ(9.5, nonZeroInitialValueIntegrator->update(-1.0));
  ASSERT_DOUBLE_EQ(8, nonZeroInitialValueIntegrator->update(-2.0));
  ASSERT_DOUBLE_EQ(5.5, nonZeroInitialValueIntegrator->update(-3.0));
  ASSERT_DOUBLE_EQ(2.0, nonZeroInitialValueIntegrator->update(-4.0));
  ASSERT_DOUBLE_EQ(-2.5, nonZeroInitialValueIntegrator->update(-5.0));
}

TEST_F(IntegratorTest, VariyingLineTest)
{
  ASSERT_DOUBLE_EQ(0.5, defaultIntegrator->update(1.0));
  ASSERT_DOUBLE_EQ(2.0, defaultIntegrator->update(2.0));
  ASSERT_DOUBLE_EQ(4.5, defaultIntegrator->update(3.0));
  ASSERT_DOUBLE_EQ(8.0, defaultIntegrator->update(4.0));
  ASSERT_DOUBLE_EQ(12.5, defaultIntegrator->update(5.0));
  ASSERT_DOUBLE_EQ(17.0, defaultIntegrator->update(4.0));
  ASSERT_DOUBLE_EQ(20.5, defaultIntegrator->update(3.0));
  ASSERT_DOUBLE_EQ(23.0, defaultIntegrator->update(2.0));
  ASSERT_DOUBLE_EQ(24.5, defaultIntegrator->update(1.0));
  ASSERT_DOUBLE_EQ(25.0, defaultIntegrator->update(0.0));
  ASSERT_DOUBLE_EQ(24.5, defaultIntegrator->update(-1.0));
  ASSERT_DOUBLE_EQ(23.0, defaultIntegrator->update(-2.0));
  ASSERT_DOUBLE_EQ(20.5, defaultIntegrator->update(-3.0));
  ASSERT_DOUBLE_EQ(17.0, defaultIntegrator->update(-4.0));
  ASSERT_DOUBLE_EQ(12.5, defaultIntegrator->update(-5.0));
  ASSERT_DOUBLE_EQ(7.0, defaultIntegrator->update(-6.0));
  ASSERT_DOUBLE_EQ(0.5, defaultIntegrator->update(-7.0));
  ASSERT_DOUBLE_EQ(-7.0, defaultIntegrator->update(-8.0));
}

TEST_F(IntegratorTest, ConstantZeroTest)
{
  ASSERT_DOUBLE_EQ(0.0, defaultIntegrator->update(0.0));
  ASSERT_DOUBLE_EQ(0.0, defaultIntegrator->update(0.0));
  ASSERT_DOUBLE_EQ(0.0, defaultIntegrator->update(0.0));
  ASSERT_DOUBLE_EQ(0.0, defaultIntegrator->update(0.0));
  ASSERT_DOUBLE_EQ(0.0, defaultIntegrator->update(0.0));
}

TEST_F(IntegratorTest, VectorIntegral)
{
  Eigen::Vector4d sample, res;
  sample << 1, 1, -1, -1;
  res << 0.5, 0.5, -0.5, -0.5;

  ASSERT_TRUE(vectorIntegrator->update(sample).isApprox(res));
  
  sample << 2.0, 2.0, -2.0, -2.0;
  res << 2.0, 2.0, -2.0, -2.0;
  ASSERT_TRUE(vectorIntegrator->update(sample).isApprox(res));

  sample << 3.0, 3.0, -3.0, -3.0;
  res << 4.5, 4.5, -4.5, -4.5;
  ASSERT_TRUE(vectorIntegrator->update(sample).isApprox(res));
    
  sample << 4.0, 4.0, -4.0, -4.0;
  res << 8.0, 8.0, -8.0, -8.0;
  ASSERT_TRUE(vectorIntegrator->update(sample).isApprox(res));
  
  sample << 5.0, 5.0, -5.0, -5.0;
  res << 12.5, 12.5, -12.5, -12.5;
  ASSERT_TRUE(vectorIntegrator->update(sample).isApprox(res));
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
    std::cerr << "Unhandled excpetion with message: " << e.what() << std::endl;
    return -1;
  }
}
