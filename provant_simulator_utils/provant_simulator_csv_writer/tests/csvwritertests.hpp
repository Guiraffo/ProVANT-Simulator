/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file csvwritertests.hpp
 * @brief This file contains the tests for the CSVWriter class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_CSV_WRITER_TESTS_H
#define PROVANT_CSV_WRITER_TESTS_H

#include <gtest/gtest.h>

#include <provant_simulator_csv_writer/csvwriter.h>

#include <boost/algorithm/string.hpp>

#include <sstream>
#include <vector>

class CSVWriterTests : public ::testing::Test
{
protected:
  std::stringstream ss;
  provant::csv::CSVWriter<int, 2> defaultWriter{ ss, { "A", "B" } };
};

TEST_F(CSVWriterTests, ConstructorWritesColumnLabels)
{
  const auto res = ss.str();
  ASSERT_EQ(res, "\"A\",\"B\"\n");
}

TEST_F(CSVWriterTests, WriteLinesTest)
{
  std::vector<std::string> lines;

  defaultWriter.writeLine({ 1, 2 });

  const auto str = ss.str();
  boost::algorithm::split(lines, str, boost::algorithm::is_any_of(defaultWriter.options().newline));

  ASSERT_EQ(lines.size(), 3);
  ASSERT_EQ(lines.at(0), "\"A\",\"B\"");
  ASSERT_EQ(lines.at(1), "1,2");
  ASSERT_EQ(lines.at(2), "");
}

class ExcelCSVWriterTests : public ::testing::Test
{
protected:
  std::stringstream ss;
  provant::csv::CSVWriter<int, 2> excelWriter{ ss, { "A", "B" }, provant::csv::CSVOptions::excel() };
};

TEST_F(ExcelCSVWriterTests, InitializesOptions)
{
  const auto opts = excelWriter.options();
  const auto excelOpts = provant::csv::CSVOptions::excel();

  ASSERT_EQ(opts.delimiter, excelOpts.delimiter);
  ASSERT_EQ(opts.newline, excelOpts.newline);
  ASSERT_EQ(opts.quote, excelOpts.quote);
  ASSERT_EQ(opts.escape, excelOpts.escape);
}

TEST_F(ExcelCSVWriterTests, ConstructorWritesColumnNames)
{
  const auto str = ss.str();
  ASSERT_EQ(str, "\"A\";\"B\"\n");
}

#endif  // PROVANT_CSV_WRITER_TESTS_H
