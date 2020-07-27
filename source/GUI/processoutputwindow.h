#ifndef PROCESSOUTPUTWINDOW_H
#define PROCESSOUTPUTWINDOW_H

#include <QMainWindow>

#include <QColor>
#include <QDir>
#include <QProcess>
#include <QProcessEnvironment>
#include <QSet>

namespace Ui {
class ProcessOutputWindow;
}

/**
 * @brief The ProcessOutputWindow class
 *
 * This class is responsible for mananing a process and displaying its output
 * in a user interface.
 *
 * @author Júnio Eduardo de Morais Aquino
 * @date 2019/02/24
 *
 * The most important reason for the design and implementation of this class is
 * to act as a better alternative to opening external terminals as a means to
 * display the output of other programs needed throughout the simulator.
 *
 * This class firstly treats external programs as processes that are managed
 * through the Qt interfaces to provide a portable solution across different
 * operational systems.
 *
 * Beyond that, this class also provides a few niceties, such as the ability to
 * color errors to facilitate detection and future correction, to save the
 * output to a file and copy it to the clipboard.
 *
 * All of the provided capabilities of a Qt managed process are explored to
 * provide the best possible monitoring and error reporting.
 */
class ProcessOutputWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ProcessOutputWindow(QProcess *process,
                                 QWidget *parent = nullptr);
    ~ProcessOutputWindow();

    QProcess *process();
    QProcess *resetProcess();

    bool isProcessRunning() const;

    const QString &processName() const;

    QString output() const;
    QString outputHtml() const;
    QString outputMarkdown() const;

signals:
    void dataAdded(const QString &data);
    void contentChanged(const QString &content);
    void outputCleared();
    void processNameChanged(const QString &name);

    /**
     * @brief processStarted Signal emited when the process object managed
     * by this window starts its exectution.
     */
    void processStarted();
    /**
     * @brief processFinished Singal emited when the process object managed
     * by this window finishes its execution.
     * @param code The exit code returned by process.
     * @param status The exit status of the process (Indicates sucess or
     * failure).
     */
    void processFinished(int code, QProcess::ExitStatus status);
    /**
     * @brief processErrorOcurred Singla emited when an error occurs during
     * the process execution.
     * @param error The error code that ocurred during the process execution.
     */
    void processErrorOcurred(QProcess::ProcessError error);

public slots:
    void addData(const QString &data);
    void addData(const QStringList &data);
    void clearOutput();
    void start();
    void start(const QProcessEnvironment &env);

    void setProcessName(const QString &name);

    void setSaveOutputStartingDir(const QString &dirPath);
    void setSaveOutputStartingDir(const QDir &dir);

    bool savePlaintextOutputToFile(const QString &filePath);
    bool saveHtmlOutputToFile(const QString &filePath);
    bool saveMarkdwonOutputToFile(const QString &filePath);

private slots:
    void readStandardOutFromProcess();
    void readStandardErrorFromProcess();

    void onProcessStart();
    void onProcessFinish(int exitCode, QProcess::ExitStatus exitStatus);
    void onProcessError(QProcess::ProcessError error);

    void saveOutputAction();
    void copyOutpuAction();

protected:
    void addText(const QString &text, const QColor &color = Qt::black);
    virtual void closeEvent(QCloseEvent *event) override;

    bool saveToFile(const QString &content, const QString &filePath);

private:
    //! Pointer to the elements contained in the user interface.
    Ui::ProcessOutputWindow *ui;
    //! Pointer to the process object managed by this window.
    QProcess *_process = nullptr;
    //! The name of the process displayed in the title and in the messages
    //! handled by this window.
    QString _processName = tr("Process");
    //! Stores the directory in which the save output file browser will
    //! be opened in.
    QString _prevSaveOutputDir = QDir::homePath();
    //! Stores the file filter in which the file will be saved.
    QString _prevSelectedFilter = tr("HTML File") + "(*.html);;";
    //! Stores the pid of the parent QProcess
    qint64 _pid = 0;

    /*
     * The colors were added as constants here, but withtout addition of getter
     * and setter methods to allow for easier change of this parameters and
     * possible addition of these values as users options.
     */

    //! Color used to output normal output from the stdout file of the process
    //! to the screen.
    QColor normalOutputColor = Qt::black;
    //! Color used to output errors from the stderr file and from other sources
    //! of erros to the screen.
    QColor errorOutputColor = Qt::red;
    //! Color used to display debugging messages and other content that is not
    //! directly generated to the process to the screen.
    QColor applicationOutputColor = Qt::blue;

    void populateToolbar();
    bool actionOnClosingWhithProcessRunning();
    QSet<QString> getChildProcessPID(int pid,
                                     QSet<QString> ids = QSet<QString>());

    /**
     * @brief _idsToKill Store a list of the child processes PIDs to allow the
     * destructor to emit a kill signal to each child process.
     *
     * This is necessary because killing the parent process does not kill the
     * entire process tree in the Linux OS, so in order to guarantee that no
     * process remain open after a forced closing of the ProcessOutputWindow,
     * it was necessary to create this set of pids and the getChildProcessPID()
     * method.
     */
    QSet<QString> _idsToKill;
};

#endif // PROCESSOUTPUTWINDOW_H
