/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file csvwriter.h
 * @brief This file contains the implementation of the CSVOptions class, and the
 * CSVWriter and CSVFileWriter class templates.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_CSVWRITER_H
#define PROVANT_CSVWRITER_H

#include <boost/algorithm/string/replace.hpp>

#include <array>

/*
 * @author Júnio Eduardo de Morais Aquino
 * These macros are created to fix compilation of the provant_simulator_csv_writer
 * library in Ubuntu 18.04.
 * The std::filesystem library is present in the C++17 standard, however, GCC does
 * not support the library until version 9.
 * As the default GCC version on Ubuntu 18.04 is GCC 7, we use these macros to
 * select for the adequate version of the filesystem library, as in GCC 7, it is
 * still an experimental feature.
 */
#if __GNUC__ > 8
#include <filesystem>
#define FS_NAMESPACE std::filesystem;
#else
#include <experimental/filesystem>
#define FS_NAMESPACE std::experimental::filesystem;
#endif

#include <fstream>
#include <memory>
#include <ostream>
#include <string>

namespace provant
{
namespace csv
{
struct CSVOptions
{
  CSVOptions(const std::string& delimiter_, const std::string& newline_, const std::string& quote_,
             const std::string& escape_)
    : delimiter(delimiter_), newline(newline_), quote(quote_), escape(escape_)
  {
  }

  static CSVOptions excel()
  {
    return CSVOptions{ ";", "\n", "\"", "\\" };
  }

  static CSVOptions defaultCsv()
  {
    return CSVOptions{ ",", "\n", "\"", "\\" };
  }

  static CSVOptions tabSeparatedValues()
  {
    return CSVOptions{ "\t", "\n", "\"", "\\" };
  }

  const std::string delimiter;
  const std::string newline;
  const std::string quote;
  const std::string escape;
};

template <typename T = double, int lineSize = 1>
class CSVWriter
{
public:
  CSVWriter(std::ostream& os, const std::array<std::string, lineSize>& columnLabels,
            const CSVOptions& options = CSVOptions::defaultCsv())
    : _output(os), _options(options)
  {
    for (int i = 0; i < lineSize - 1; ++i)
    {
      _output << escapeString(columnLabels.at(i)) << _options.delimiter;
    }
    _output << escapeString(columnLabels.at(lineSize - 1)) << _options.newline;
  }

  // Forbid copies and moves
  CSVWriter(const CSVWriter& other) = delete;
  CSVWriter(CSVWriter&& other) = delete;
  CSVWriter* operator=(const CSVWriter& other) = delete;
  CSVWriter* operator=(CSVWriter&& other) = delete;

  virtual ~CSVWriter()
  {
    _output.flush();
  }

  void writeLine(const std::array<T, lineSize>& values)
  {
    for (int i = 0; i < lineSize - 1; i++)
    {
      _output << values.at(i) << _options.delimiter;
    }
    _output << values.at(lineSize - 1) << _options.newline;
  }

  const CSVOptions& options() const
  {
    return _options;
  }

  void flush()
  {
    _output.flush();
  }

protected:
  std::string escapeString(const std::string& str) const
  {
    return _options.quote +
           boost::algorithm::replace_all_copy(str, _options.quote, _options.escape + _options.escape + _options.quote) +
           _options.quote;
  }

private:
  std::ostream& _output;

  const CSVOptions _options;
};

template <typename T, int lineSize>
class CSVFileWriter
{
public:
  CSVFileWriter(const std::string& path, const std::array<std::string, lineSize>& columnNames,
                const CSVOptions& options = CSVOptions::defaultCsv())
  {
    namespace fs = FS_NAMESPACE;

    fs::path filePath{ path };

    if (!filePath.has_extension())
    {
      filePath += ".csv";
    }

    _path = filePath.string();
    _os.open(_path, std::ios::out);
    if (!_os.good())
    {
      throw std::runtime_error("An error ocurred while trying to create the file with path " + _path);
    }

    _writer = std::make_unique<CSVWriter<T, lineSize>>(_os, columnNames, options);
    _os.flush();
  }

  static std::string relativeToMatlabData(const std::string& filename)
  {
    if (filename.empty())
    {
      throw std::invalid_argument("The filename cannot be empty");
    }

    const auto tilt_matlab = std::getenv("TILT_MATLAB");
    if (tilt_matlab != nullptr)
    {
      namespace fs = FS_NAMESPACE;
      const fs::path basePath{ tilt_matlab };
      const fs::path fullPath = fs::absolute(basePath / filename);

      return fullPath.string();
    }

    throw std::runtime_error{ "An error ocurred while reading the TILT_MATLAB environemnt variable." };
  }

  void writeLine(const std::array<T, lineSize>& values)
  {
    if (_writer)
    {
      _writer->writeLine(values);
    }
  }

  const CSVOptions& options() const
  {
    return _writer->options();
  }

  const std::string& path() const
  {
    return _path;
  }

  CSVWriter<T, lineSize>* getWriter()
  {
    return _writer.get();
  }

  void flush()
  {
    _os.flush();
  }

private:
  std::string _path;
  std::ofstream _os;
  std::unique_ptr<CSVWriter<T, lineSize>> _writer;
};

}  // namespace csv
}  // namespace provant

#endif  // PROVANT_CSVWRITER_H
