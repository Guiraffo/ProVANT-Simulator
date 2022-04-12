/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file csvoptionstests.hpp
 * @brief This file contains the tests for the CSVOptions class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef CSV_OPTIONS_TESTS_H
#define CSV_OPTIONS_TESTS_H

#include <gtest/gtest.h>

#include <provant_simulator_csv_writer/csvwriter.h>

class CSVOptionsTests : public ::testing::Test
{
protected:
  const std::string delimiter{ "d" };
  const std::string newline{ "n" };
  const std::string quote{ "q" };
  const std::string escape{ "e" };

  provant::csv::CSVOptions obj{ delimiter, newline, quote, escape };
};

TEST_F(CSVOptionsTests, InitializesDelimiter)
{
  ASSERT_EQ(obj.delimiter, delimiter);
}

TEST_F(CSVOptionsTests, InitializesNewline)
{
  ASSERT_EQ(obj.newline, newline);
}

TEST_F(CSVOptionsTests, InitializesQuote)
{
  ASSERT_EQ(obj.quote, quote);
}

TEST_F(CSVOptionsTests, InitializesEscape)
{
  ASSERT_EQ(obj.escape, escape);
}

TEST_F(CSVOptionsTests, CSVOptions)
{
  const auto csv = provant::csv::CSVOptions::defaultCsv();

  ASSERT_EQ(csv.delimiter, ",");
  ASSERT_EQ(csv.newline, "\n");
  ASSERT_EQ(csv.quote, "\"");
  ASSERT_EQ(csv.escape, "\\");
}

TEST_F(CSVOptionsTests, ExcelOptions)
{
  const auto excel = provant::csv::CSVOptions::excel();

  ASSERT_EQ(excel.delimiter, ";");
  ASSERT_EQ(excel.newline, "\n");
  ASSERT_EQ(excel.quote, "\"");
  ASSERT_EQ(excel.escape, "\\");
}

TEST_F(CSVOptionsTests, TSVOptions)
{
  const auto tsv = provant::csv::CSVOptions::tabSeparatedValues();

  ASSERT_EQ(tsv.delimiter, "\t");
  ASSERT_EQ(tsv.newline, "\n");
  ASSERT_EQ(tsv.quote, "\"");
  ASSERT_EQ(tsv.escape, "\\");
}

#endif  // CSV_OPTIONS_TESTS_H
