/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file tests_main.cpp
 * @brief This file is the entrypoint for execution of the ProVANT Simulator CSV Writer package.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include <gtest/gtest.h>

#include "csvfilewritertests.hpp"
#include "csvoptionstests.hpp"
#include "csvwritertests.hpp"

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
