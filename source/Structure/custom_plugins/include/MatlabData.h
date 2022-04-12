/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file MatlabData.h
 * @brief This file contains a class that outputs Data to a CSV file.
 *
 * @todo Deprecate this class as soon as possible.
 *
 * @author Arthur Viana Lara
 */

#include <fstream>
#include <string>

class MatlabData
{
private:
  std::fstream file;

public:
  MatlabData() = default;
  ~MatlabData() = default;

  // Forbid copies and moves of this class
  MatlabData(const MatlabData& other) = delete;
  MatlabData(MatlabData&& other) = delete;
  MatlabData* operator=(const MatlabData& other) = delete;
  MatlabData* operator=(MatlabData&& other) = delete;

  /**
   * @brief Opens a file with the specified path.
   *
   * @param namefile Path of the file to save.
   * @param varname Unused.
   */
  [[deprecated, nodiscard]] bool startFile(const std::string& namefile, std::string /*varname*/)
  {
    file.open(namefile, std::fstream::out);
    return file.good();
  }

  /**
   * @brief Opens a file in a specified path.
   *
   * @param path Path to create the file in. If a file exists in this path it will be overriden.
   * @return true If the file was created successfully.
   * @return false Otherwise.
   */
  [[nodiscard]] bool startFile(const std::string& path)
  {
    file.open(path, std::fstream::out);
    return file.good();
  }

  /**
   * @brief Checks if the file is opened.
   *
   * @return true If the file is opened and no erros have ocurred.
   * @return false Otherwise.
   */
  bool isOpen() const
  {
    return file.is_open() && file.good();
  }

  /**
   * @brief Close the file and flush its contents to disk.
   */
  void endFile()
  {
    file.close();
  }

  /**
   * @brief Flushes (writes the buffer) the file contents to disk.
   */
  void flush()
  {
    if (file.is_open())
      file.flush();
  }

  /**
   * @brief Add a line of data to the file.
   *
   * @param data Data to save.
   */
  void printFile(const std::vector<double>& data)
  {
    if (file.is_open())
    {
      if (data.size() > 0)
      {
        file << data.at(0);
        int i = 1;
        while (i < data.size())
        {
          file << "," << data.at(i);
          i++;
        }
        file << "\n";
      }
    }
  }

  /**
   * @brief Add a line composed of a single double to the file.
   *
   * @param data Data to save.
   */
  void printFile(double data)
  {
    if (file.is_open())
    {
      file << data << "\n";
    }
  }
};
