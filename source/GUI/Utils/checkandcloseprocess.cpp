#include "checkandcloseprocess.h"

#include <QDebug>
#include <QProcess>

CheckAndCloseProcess::CheckAndCloseProcess()
{
}

CheckAndCloseProcess::CheckAndCloseProcess(const QString& processName)
  : _processNames(processName)
{
}

CheckAndCloseProcess::CheckAndCloseProcess(const QStringList& processNames)
  : _processNames(processNames)
{
}

const QStringList& CheckAndCloseProcess::getProcessNames() const
{
  return _processNames;
}

void CheckAndCloseProcess::addProcess(const QString& processName)
{
  _processNames.append(processName);
}

void CheckAndCloseProcess::addProcesses(const QStringList& processNames)
{
  foreach (const QString newProcess, processNames)
  {
    _processNames.append(newProcess);
  }
}

void CheckAndCloseProcess::setProcesses(const QStringList& processNames)
{
  _processNames = processNames;
}

bool CheckAndCloseProcess::isProcessRunning()
{
  findPids();
  return !_pids.empty();
}

const QSet<int> CheckAndCloseProcess::getPids() const
{
  return _pids;
}

void CheckAndCloseProcess::closeProcesses()
{
  findPids();
  QProcess killProcess;
  killProcess.setProgram("kill");
  QStringList args;
  args << "-15";

  for (auto it = _pids.cbegin(); it != _pids.cend(); ++it)
  {
    args << QString::number(*it);
  }

  killProcess.setArguments(args);
  killProcess.start();
  killProcess.waitForFinished();

  QString res = killProcess.readAllStandardOutput();
  qDebug() << res;
}

void CheckAndCloseProcess::findPids()
{
  foreach (const QString& processName, _processNames)
  {
    QProcess pgrepProcess;
    pgrepProcess.setProgram("pgrep");
    QStringList args;
    args << processName;
    pgrepProcess.setArguments(args);

    pgrepProcess.start();
    pgrepProcess.waitForFinished();

    QString response = pgrepProcess.readAllStandardOutput();
    if (!response.isEmpty())
    {
      QStringList currentPids = response.split("\n");
      foreach (const QString& pidStr, currentPids)
      {
        bool ok = false;
        int pid = pidStr.toDouble(&ok);
        if (ok)
        {
          _pids.insert(pid);
        }
      }
    }
  }
}
