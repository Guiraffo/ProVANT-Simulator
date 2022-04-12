#ifndef CONFIGFILE_H
#define CONFIGFILE_H

#include <QStringList>
#include <QtXml/QtXml>

/**
 * @brief The ConfigFile class is responsible for manipulation the config XML
 * file.
 *
 * This class provides an interface for the access and manipulation of the
 * contents of the config file for a simulator model.
 *
 * @todo Question Arthur about the purpose of this class.
 */
class ConfigFile
{
public:
  ConfigFile();
  ConfigFile(const QString& filename);
  void setFilename(const std::string& filename);
  void print() const;

  void clearSensorsAndActuators();

  const QStringList& getSensors() const;
  void setSensors(const QStringList& sensors);
  void addSensor(const QString& sensor);
  void deleteSensor(int pos);

  const QStringList getActuators() const;
  void setActuators(const QStringList& actuators);
  void addActuator(const QString& actuator);
  void deleteActuator(int pos);

  const QString& getControlStrategy() const;
  void setStrategy(const QString& strategy);

  const QString& getTurbulanceModel() const;
  void setTurbulanceModel(const QString& turbulance);

  const QString& getSampleTime() const;
  void setSampleTime(const QString& sampleTime);

  const QString& getCommunication() const;
  void setCommunication(const QString& comm);

  const QString& getErrorLogFilename() const;
  const QString& getRefLogFilename() const;
  const QString& getOutLogFilename() const;
  const QString& getInLog() const;
  void setLog(const QString& erro, const QString& ref, const QString& out,
              const QString& in);

  const QString& getStepTopic() const;
  void setStepTopic(const QString& topic);

  bool createFile();
  void readFile();
  bool writeFile();

  QString readItem(const QString& tag);
  QStringList readAllItems(const QString& tag);

  uint64_t getSimulationDuration() const;
  void setSimulationDuration(uint64_t steps);

  bool getShutdownWhenFinished() const;
  void setShutdownWhenFinished(bool shutdown);

  bool getHilFlagSynchronous() const;
  void setHilFlagSynchronous(bool HilFlagSync);

  bool getHilFlagAsynchronous() const;
  void setHilFlagAsynchronous(bool HilFlagAsync);

  const QString& getUsart1() const;
  void setUsart1(const QString& usart1);

  const QString& getUsart2() const;
  void setUsart2(const QString& usart1);

  int getBaudRate() const;
  void setBaudRate(int baudRate);

  bool getStartPaused() const;
  void setStartPaused(bool paused);

protected:
  /**
   * @brief readItemBool Read the value of a boolean XML element.
   *
   * @param tag Tag name of the element to read.
   * @returns true If the value of the element is equal to 1 or any case
   * combination of the word true.
   * @returns false Otherwise.
   */
  virtual bool readItemBool(const QString& tag);

private:
  QString _filename;
  QDomDocument _document;
  QFile _file;
  QStringList _sensors;
  QStringList _actuators;
  QString _controlStrategy;
  QString _turbulanceModel;
  QString _sampleTime;
  QString _communication;
  QString _errorFilename;
  QString _refFilename;
  QString _outFilename;
  QString _inFilename;
  QString _stepTopic;
  QString _usart1;
  QString _usart2;
  //! Number of steps the simulation must run before the end. Note that a zero
  //! value indicates an infinite (user controlled) simulation.
  uint64_t _duration;
  //! Indicates if the simulation must automatically close when it finishes.
  bool _shutdownWhenFinished;
  //! Indicates if the simulation is in hil synchronous mode.
  bool _HilFlagSynchronous;
  //! Indicates if the simulation is in hil asynchronous mode.
  bool _HilFlagAsynchronous;
  //! Indicates the baudrate used for communication with the serial ports when
  //! in HIL mode.
  int _baudRate;
  //! Indicates if the simulation must start in a paused state or not.
  bool _startPaused;
};

#endif  // CONFIGFILE_H
