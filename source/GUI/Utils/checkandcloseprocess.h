#ifndef CHECKANDCLOSEPROCESS_H
#define CHECKANDCLOSEPROCESS_H

#include <QSet>
#include <QStringList>

class CheckAndCloseProcess
{
public:
  CheckAndCloseProcess();
  CheckAndCloseProcess(const QString& processName);
  CheckAndCloseProcess(const QStringList& processNames);

  const QStringList& getProcessNames() const;
  void addProcess(const QString& processName);
  void addProcesses(const QStringList& processNames);
  void setProcesses(const QStringList& processNames);

  bool isProcessRunning();
  const QSet<int> getPids() const;
  void closeProcesses();

protected:
  /**
   * @brief getPids Check the pids of the process contained in the desired
   * processes list.
   */
  void findPids();

private:
  //! Names of the process to verify and kill
  QStringList _processNames;
  //! List of the PIDs of the process currently running.
  QSet<int> _pids;
};

#endif  // CHECKANDCLOSEPROCESS_H
