#include "processoutputwindow.h"
#include "ui_processoutputwindow.h"

#include <QCloseEvent>
#include <QDebug>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QPushButton>
#include <QTextStream>

#include "Utils/appsettings.h"

/**
 * @brief ProcessOutputWindow::ProcessOutputWindow
 * @param process The process object that will be managed by this window.
 * @param parent The QObject that contains this window.
 *
 * Initializes a ProcessOutputWindow.
 *
 * This method is responsible for initializing the user interface and
 * connecting the appropriate signals for the process to work with the output
 * window.
 *
 * Please note that the process needs to be in separate channels mode to work
 * correctly.
 */
ProcessOutputWindow::ProcessOutputWindow(QProcess *process,
                                         QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ProcessOutputWindow),
    _process(process)
{
    ui->setupUi(this);
    /*
     * Setup this window to be deleted after its closing.
     * This needs to be done to ensure proper deletion of the process pointer
     * and also the of the user interface elements.
     */
    setAttribute(Qt::WA_DeleteOnClose, true);

    // Adds the buttons to the toolbar
    populateToolbar();

    // Connect the process signals to the appropriate slots.
    QObject::connect(
                _process,
                &QProcess::readyReadStandardOutput,
                this,
                &ProcessOutputWindow::readStandardOutFromProcess);
    QObject::connect(
                _process,
                &QProcess::readyReadStandardError,
                this,
                &ProcessOutputWindow::readStandardErrorFromProcess);
    QObject::connect(
                _process,
                &QProcess::started,
                this,
                &ProcessOutputWindow::onProcessStart);
    connect(_process,
                  SIGNAL(finished(int, QProcess::ExitStatus)),
                  this,
                  SLOT(onProcessFinish(int, QProcess::ExitStatus)));
    connect(_process,
                  &QProcess::errorOccurred,
                  this,
                  &ProcessOutputWindow::onProcessError);

    // Keeps the error and ouput exits separated.
    _process->setProcessChannelMode(QProcess::SeparateChannels);
}

/**
 * @brief ProcessOutputWindow::~ProcessOutputWindow
 * Delete the user interface objects and the process object.
 *
 * If the deletion of the processo object isn't desired, please be sure to
 * copy a pointer to this object, and reset the value of to pointer managed
 * by this class to nullptr.
 *
 * For more details about this action see the method resetProcess().
 */
ProcessOutputWindow::~ProcessOutputWindow()
{
    qDebug("Deleting ProcessOutputWindow object");

    if(_pid != 0)
        _idsToKill.insert(QString::number(_pid));

    QStringList killIds = _idsToKill.toList();
    if(!killIds.empty()){
        QProcess killChildProcess;
        killChildProcess.start("kill", killIds);
        killChildProcess.waitForFinished();
    }

    if(_process != nullptr)
    {
        if(_process->state() != QProcess::NotRunning)
        {
            _process->kill();
        }
        delete _process;
    }
    delete ui;
}

/**
 * @brief ProcessOutputWindow::process
 * @return The pointer to the process object managed by this window.
 */
QProcess *ProcessOutputWindow::process()
{
    return _process;
}

/**
 * @brief ProcessOutputWindow::resetProcess
 * This function sets the value of the process object managed by this window
 * to nullptr to prevent deletion.
 * @return The pointer to the process object that was handled by this window.
 *
 * @bold Warning: After taking this action the window will no longer take care
 * to properly close and delete the process object after it is closed, this
 * actions must be done manually.
 *
 * Please note, that taking this action before the completion of the process
 * execution can lead to negative side effects in this window exection.
 *
 * This function is intended to be used when there is a need to do some action
 * with the process object after it finishes its execution.
 *
 * To check if the process is still running, see the method isProcessRunning().
 */
QProcess *ProcessOutputWindow::resetProcess()
{
    QProcess *temp = _process;
    _process = nullptr;
    return temp;
}

/**
 * @brief ProcessOutputWindow::isProcessRunning
 * @return True if the process object is not executing any action or if the
 * object was reseted after this window start and false otherwise.
 *
 * See also:  resetProcess().
 */
bool ProcessOutputWindow::isProcessRunning() const
{
    if(_process != nullptr)
    {
        return _process->state() != QProcess::NotRunning;
    }
    return true;
}

/**
 * @brief ProcessOutputWindow::processName
 * @return
 */
const QString &ProcessOutputWindow::processName() const
{
    return _processName;
}

/**
 * @brief ProcessOutputWindow::addData
 * Adds data to the output window.
 * @param data The data to be added to the screen.
 *
 * This function adds the user informed data to the screen and emits the
 * relevant signals.
 */
void ProcessOutputWindow::addData(const QString &data)
{
    if(!data.isEmpty()) {
        ui->textOutput->append(data);
        emit dataAdded(data);
        emit contentChanged(ui->textOutput->toPlainText());
    }
}

/**
 * @brief ProcessOutputWindow::addData
 * Slot to add data to the output panel.
 * @param data A list of new lines of text to add to the output window.
 */
void ProcessOutputWindow::addData(const QStringList &data)
{
    for(QStringList::const_iterator i = data.constBegin();
        i != data.constEnd();
        i++)
    {
        addData(*i);
    }
}

/**
 * @brief ProcessOutputWindow::clear
 * Clears all the text displayed in the window.
 */
void ProcessOutputWindow::clearOutput()
{
    if(!ui->textOutput->toPlainText().isEmpty())
    {
        ui->textOutput->clear();
        emit outputCleared();
        emit contentChanged("");
    }
}

/**
 * @brief ProcessOutputWindow::start
 * Starts the process execution.
 */
void ProcessOutputWindow::start()
{
    _process->start();
}

/**
 * @brief ProcessOutputWindow::start
 * Starts the process execution setting its environment variables.
 * @param env Environment variables for the new process.
 */
void ProcessOutputWindow::start(const QProcessEnvironment &env)
{
    _process->setProcessEnvironment(env);
    _process->start();
}

/**
 * @brief ProcessOutputWindow::setProcessName
 * Defines the name of the process being handled by this window.
 * @param name The name of the process.
 *
 * The process name is used in the window title, the actions performed by the
 * window and in the logging and other messages shown to the user.
 */
void ProcessOutputWindow::setProcessName(const QString &name)
{
    if(name != _processName)
    {
        _processName = name;
        setWindowTitle(name);
        emit processNameChanged(name);
    }
}

/**
 * @brief ProcessOutputWindow::setSaveOutput Dir
 * @param dirPath The path to starting directory of the save output action
 * file browser.
 */
void ProcessOutputWindow::setSaveOutputStartingDir(const QString &dirPath)
{
    _prevSaveOutputDir = dirPath;
}

/**
 * @brief ProcessOutputWindow::setSaveOutputStartingDir
 * @param dir The directory in which the save output action file browser
 * should be opened in.
 */
void ProcessOutputWindow::setSaveOutputStartingDir(const QDir &dir)
{
    setSaveOutputStartingDir(dir.absolutePath());
}

/**
 * @brief ProcessOutputWindow::savePlaintextOutputToFile
 * Saves the plaintext version of the window content to a file.
 * @param filePath The path of the file to save the content to.
 * @return True if the content was saved and false if an error ocurred.
 */
bool ProcessOutputWindow::savePlaintextOutputToFile(const QString &filePath)
{
    return saveToFile(ui->textOutput->toPlainText(), filePath);
}

/**
 * @brief ProcessOutputWindow::saveHtmlOutputToFile
 * Saves the HTML version of the window content to a file.
 * @param filePath The path of the file to save the content to.
 * @return True if the content was saved and false if an error ocurred.
 */
bool ProcessOutputWindow::saveHtmlOutputToFile(const QString &filePath)
{
    return saveToFile(ui->textOutput->toHtml(), filePath);
}

/**
 * @brief ProcessOutputWindow::saveMarkdwonOutputToFile
 * Saves the Markdown version of the content to a file.
 * @param filePath The path of the file to save the content to.
 * @return True if the content was saved and false if an error ocurred.
 * @todo As Qt 5.12 do not support markdown export yet, I changed it to output
 * HTML, this needs to be corrected in the future.
 */
bool ProcessOutputWindow::saveMarkdwonOutputToFile(const QString &filePath)
{
    return saveToFile(ui->textOutput->toHtml(), filePath);
}

/**
 * @brief ProcessOutputWindow::output
 * @return The plaintext version of the output window content.
 */
QString ProcessOutputWindow::output() const
{
    return ui->textOutput->toPlainText();
}

/**
 * @brief ProcessOutputWindow::outputHtml
 * @return The html version of the output window content.
 */
QString ProcessOutputWindow::outputHtml() const
{
    return ui->textOutput->toHtml();
}

/**
 * @brief ProcessOutputWindow::outputMarkdown
 * @return The contents of the output as a markdown typed string.
 * @todo As Qt 5.12 do not support Markdown export, i have changed it to output
 * HTML, this needs to be corrected in the future.
 */
QString ProcessOutputWindow::outputMarkdown() const
{
    return ui->textOutput->toHtml();
}

/**
 * @brief ProcessOutputWindow::readStandardOutFromProcess
 * Slot called to handle the signal readyReadStandardOut.
 *
 * Reads the output from the process, and adds to the ouput screen with the
 * specified color for the standard ouput (default is black).
 */
void ProcessOutputWindow::readStandardOutFromProcess()
{
    addText(QString(_process->readAllStandardOutput()), normalOutputColor);
}

/**
 * @brief ProcessOutputWindow::readStandardErrorFromProcess
 * Slot called to handle the singal readyReadStandardError.
 *
 * Reads the error from the process, and adds to the output screen with the
 * color specified for the error output (default is red).
 */
void ProcessOutputWindow::readStandardErrorFromProcess()
{
    addText(QString(_process->readAllStandardError()), errorOutputColor);
}

/**
 * @brief ProcessOutputWindow::onProcessStart
 * Slot called when the process starts execution.
 *
 * Logs the the process start execution, the called program and the arguments
 * as an info message and add the details of the process execution to the
 * window configured as application output message.
 */
void ProcessOutputWindow::onProcessStart()
{
    // Logs the action
    qInfo() << "Starting the " << _processName << " process with program "
            << _process->program() << " and parameters "
            << _process->arguments() << ".";

    QString args = "";
    for(QStringList::const_iterator i = _process->arguments().constBegin();
        i != _process->arguments().constEnd();
        i++)
    {
        args += ", " + *i;
    }
    args.replace(0, 1, "(");
    args += ")";

    addText(tr("Starting the %1 process execution with the following "
               "parameters: \n"
               "Program: \t%2\n"
               "Arguments: \t%3\n"
               "Working directory: \t%4\n"
               "Process PID: \t%5\n")
            .arg(_processName)
            .arg(_process->program())
            .arg(args)
            .arg(_process->workingDirectory())
            .arg(_process->processId()),
            applicationOutputColor);

    _pid = _process->processId();
    emit processStarted();
}

/**
 * @brief ProcessOutputWindow::onProcessFinish
 * Slot called to handle the siganl emited when a process finishes its
 * execution.
 * @param exitCode The exit code of the process (this parameter is only valid
 * if the status indicates a normal exit).
 * @param status Indicates if an error ocurred or the process exited normally.
 */
void ProcessOutputWindow::onProcessFinish(int exitCode,
                                          QProcess::ExitStatus status)
{
    switch(status)
    {
    case QProcess::NormalExit:
        addText(tr("The %1 process finished execution with return code %2")
                .arg(_processName)
                .arg(exitCode),
                applicationOutputColor);
        break;
    case QProcess::CrashExit:
        addText(tr("The %1 process finished execution with an error.")
                .arg(_processName),
                applicationOutputColor);
        break;
    }

    emit processFinished(exitCode, status);
}

/**
 * @brief ProcessOutputWindow::onProcessError
 * Slot called in the event of a process execution error.
 * @param error The error code indicating what the problem was.
 *
 * This function performs the following actions:
 *
 *  - Logs the error message, program and parameters as a qCritical severity
 * log.
 *  - Creates a message box and shows it to the user informing about the error
 * ocurrence.
 *  - Adds the error text to the ouput screen.
 */
void ProcessOutputWindow::onProcessError(QProcess::ProcessError error)
{
    QString errorMsg;
    qCritical() << "A process error of code " << error
                << " ocurred when trying to execute the process with program "
                << _process->program() << " and parameters ("
                << _process->arguments() << ").";
    switch(error)
    {
    case QProcess::FailedToStart:
        errorMsg = tr("The process failed to start. Either the invoked "
                      "program is missing, or you may have insufficient "
                      "permissions to invoke the program.");
        break;
    case QProcess::Crashed:
        errorMsg = tr("An error ocurred after the process started.");
        break;
    case QProcess::Timedout:
        errorMsg = tr("The process has exausted the time limit for its "
                      "execution without finishing");
        break;
    case QProcess::WriteError:
        errorMsg = tr("An error ocurred when attempting to write data to the "
                      "process.");
        break;
    case QProcess::ReadError:
        errorMsg = tr("An error ocurred when attempting to read data from the "
                      "process.");
        break;
    case QProcess::UnknownError:
        errorMsg = tr("An unknowed error has ocurred during the process "
                      "execution.");
        break;
    }

    // Creates a message box informing the user
    QMessageBox::critical(
                this,
                tr("An error ocurred during process execution"),
                errorMsg);

    // Adds data to the window
    QString details = "\n" + tr("An error ocurred during process execution.") +
            "\n" + errorMsg;
    addText(details, errorOutputColor);

    emit processErrorOcurred(error);
}

/**
 * @brief ProcessOutputWindow::saveOutputAction Action to save the contents of
 * the ouput window to a text file.
 *
 * Presentes the user with an file browser dialog.
 * If the user cancel the action, nothing is done.
 * If the user selects a valid file and the save button, the appriate method
 * is decided based on the file extension selected in the browser window.
 * The user can choose between the following types:
 *  - Text file (plaintext).
 *  - HTML file (HTML content of the window).
 *  - Markdwon file (Markdown contents of the window).
 *  - All files (the same as plaintext)
 *
 * The HTML version preserves the color scheme displayed in the window.
 *
 * If an error occurs when trying to save the file, a dialog box informing
 * the user about this problem will be exhibited.
 */
void ProcessOutputWindow::saveOutputAction()
{
    QString path = QFileDialog::getSaveFileName(
                this,
                tr("Export the %1 process output to").arg(_processName),
                _prevSaveOutputDir,
                tr("Text File") + "(*.txt);;" +
                tr("HTML File") + "*.html);;" +
                tr("Markdown File") + "*.md);;" +
                tr("All Files") + ("*.*"),
                &_prevSelectedFilter);
    if(!path.isEmpty())
    {
        bool res;
        QFileInfo info(path);
        if(info.suffix() == "html")
        {
            _prevSelectedFilter = tr("HTML File") + "*.html);;";
            res = saveHtmlOutputToFile(path);
        }
        else if(info.suffix() == "md")
        {
            _prevSelectedFilter = tr("Markdown File") + "*.md);;";
            res = saveMarkdwonOutputToFile(path);
        }
        else
        {
            _prevSaveOutputDir = tr("Text File") + "(*.txt);;";
            res = savePlaintextOutputToFile(path);
        }
        // Checks if an errour ocurred or the content was saved succesfully.
        if(!res)
        {
            QMessageBox::critical(
                        this,
                        tr("Error when saving file."),
                        tr("An error ocurred while trying the write the "
                           "contents of the ouput window to a file."));
        }
        // Updates the previous directory location
        _prevSaveOutputDir = info.path();
    }
}

/**
 * @brief ProcessOutputWindow::copyOutpuAction
 * Select all the text displayed in the output and copies it to the system
 * clipboard.
 */
void ProcessOutputWindow::copyOutpuAction()
{
    QTextCursor prevCursor = ui->textOutput->textCursor();
    // Select everythin and copy to clipboard
    ui->textOutput->selectAll();
    ui->textOutput->copy();
    // Restore previous cursor
    ui->textOutput->setTextCursor(prevCursor);
}

/**
 * @brief ProcessOutputWindow::addText
 * Adds a block of text ot the ouput window.
 * @param text The text to be added.
 * @param color The color of the text.
 *
 * This is a convenience method created to output a piece of text to the
 * output window with an specified color (default is black).
 *
 * After outputting the text, the color is restored to the previous value.
 */
void ProcessOutputWindow::addText(const QString &text, const QColor &color)
{
    // Stores the previous color and sets the new color
    QColor prevColor = ui->textOutput->textColor();
    ui->textOutput->setTextColor(color);
    // Adds the text
    addData(text);
    // Restore previous value
    ui->textOutput->setTextColor(prevColor);
}

/**
 * @brief ProcessOutputWindow::populateToolbar
 * Creates the actions in the application toolbar.
 *
 * This function is called to create the actions of the toolbar in the
 * application window.
 *
 * Currently, three actions are created:
 *  - Export output (saves the content of the output window to a file).
 *  - Copy output (copys all the content of the output to the system
 * clipboard).
 *  - Clear output window (deletes all of the text contained in the output
 * window).
 */
void ProcessOutputWindow::populateToolbar()
{
    ui->toolBar->setFloatable(false);
    ui->toolBar->setMovable(false);

    // An action to export the text
    QAction *exportOutputAction = new QAction(
                QIcon(":/themify-icons/resources/icons/save.svg"),
                tr("Export output to a text file"),
                this);
    QObject::connect(exportOutputAction,
                     &QAction::triggered,
                     this,
                     &ProcessOutputWindow::saveOutputAction);
    ui->toolBar->addAction(exportOutputAction);

    QAction *copyAction = new QAction(
                QIcon(":/themify-icons/resources/icons/share.svg"),
                tr("Copy output to clipboard"),
                this);
    QObject::connect(copyAction,
                     &QAction::triggered,
                     this,
                     &ProcessOutputWindow::copyOutpuAction);
    ui->toolBar->addAction(copyAction);

    QAction *clearWindowAction = new QAction(
                QIcon(":/themify-icons/resources/icons/eraser.svg"),
                tr("Clear window"),
                this);
    QObject::connect(clearWindowAction,
                     &QAction::triggered,
                     this,
                     &ProcessOutputWindow::clearOutput);
    ui->toolBar->addAction(clearWindowAction);
}

/**
 * @brief ProcessOutputWindow::actionOnClosingWhithProcessRunning
 * Shows a dialog box asking the user about what action to take to close the
 * window with the process still running.
 * @return True if an action was performed and the window should be closed or
 * false if the action was canceled.
 *
 * Currently the following actions are provided:
 *  - Wait: This option will wait for the maximum of 5 minutes for the process
 * to finish normaly and returns true.
 *  - Terminate: This option signals the process to finish without following
 * the normal execution pattern, but it allows the process the time to close
 * any used resources and exit in gracefull way and return true.
 *  - Kill: This option triggers an imediate closing of the process, and does
 * not allow the process to perform any closing tasks and return true.
 *  - Cancel: Does nothing and return false.
 */
bool ProcessOutputWindow::actionOnClosingWhithProcessRunning()
{
    bool proceed = false;
    QMessageBox confirmationBox;
    confirmationBox.setText(tr("The %1 is still running, do you want to "
                               "kill this process?").arg(_processName));

    QPushButton *waitButton = new QPushButton(
                tr("Wait for the process to finish"),
                this);
    waitButton->setToolTip(
                tr("Wait until the process finishes normally."));
    confirmationBox.addButton(waitButton,
                              QMessageBox::ButtonRole::YesRole);

    QPushButton *killButton = new QPushButton(
                tr("Kill the process"),
                this);
    killButton->setToolTip(tr("Finishes the process instantly without giving "
                              "it time to properly close the used resources."));
    confirmationBox.addButton(killButton,
                              QMessageBox::ButtonRole::DestructiveRole);

    QPushButton *terminateButton = new QPushButton(
                tr("Terminate the process"),
                this);
    terminateButton->setToolTip(
                tr("Signal the process the stop now."));
    confirmationBox.addButton(terminateButton,
                              QMessageBox::ButtonRole::ActionRole);


    QPushButton *cancelButton = new QPushButton(
                tr("Cancel"),
                this);
    confirmationBox.addButton(cancelButton,
                              QMessageBox::ButtonRole::RejectRole);

    confirmationBox.addButton(tr("No"), QMessageBox::NoRole);

    // Shows the message box and wait user interaction
    confirmationBox.exec();

    if(confirmationBox.clickedButton() == waitButton)
    {
        const int timeout = 5 * 60 * 1000;
        _process->waitForFinished(timeout);
        proceed = true;
    }
    else if(confirmationBox.clickedButton() == cancelButton)
    {
        proceed = false;
    }
    else
    {
        // Store the set of child process ids
        _idsToKill = getChildProcessPID(_pid);

        if(confirmationBox.clickedButton() == terminateButton)
        {
            _process->terminate();
            proceed = true;
        }
        else if(confirmationBox.clickedButton() == killButton)
        {
            _process->kill();
            proceed = true;
        }
    }

    delete waitButton;
    delete killButton;
    delete terminateButton;
    return proceed;
}

QSet<QString> ProcessOutputWindow::getChildProcessPID(int pid,
                                                      QSet<QString> ids)
{
    // Result
    QProcess getChildProcess;
    QStringList getChildCmds;
    getChildCmds << "--ppid"
                 << QString::number(pid)
                 << "-o"
                 << "pid"
                 << "--no-heading";
    getChildProcess.start("ps", getChildCmds);
    getChildProcess.waitForFinished();
    QString processOutput(getChildProcess.readAllStandardOutput());

    // Check if any pid was returned by the ps process
    if(processOutput.length() != 0)
    {
        QStringList returnedPids = processOutput.split("\n");

        foreach(const QString &id, returnedPids)
        {
            // For each pid that is not in the ids set, run the process again
            if(id.length() != 0 && !ids.contains(id))
            {
                ids.insert(id);
                QSet<QString> resSet = getChildProcessPID(id.toInt(), ids);
                ids.unite(resSet);
            }
        }
    }

    return ids;
}


/**
 * @brief ProcessOutputWindow::closeEvent
 * Method responsible for handling an envent requesting that the window be
 * closed.
 * @param event The event that spawned the call to this method.
 *
 * This method was overritten to make sure the process is correctly closed
 * before closing the window.
 *
 * If the process managed by this window is still running when the action is
 * requested, a dialog box giving a few options is shown to the user. To see
 * more details about these options, see the documention of the
 * actionOnClosingWhithProcessRunning() method.
 *
 * If the user chooses an action in this window, other than cancel, the closing
 * event is accepted and the window is closed.
 * If the user chooses to cancel, the event is rejected.
 *
 * If the process is not running, the base class implementation is called and
 * the window is closed normally.
 */
void ProcessOutputWindow::closeEvent(QCloseEvent *event)
{
    if(_process->state() != QProcess::NotRunning)
    {
        if(actionOnClosingWhithProcessRunning())
        {
            event->accept();
        }
        else
        {
            event->ignore();
        }
    }
    else {
        QMainWindow::closeEvent(event);
    }
}

/**
 * @brief ProcessOutputWindow::saveToFile
 * Saves the content string to a text file.
 * @param content The content to write to the file.
 * @param filePath The path of the file.
 * @return True if the file is successfully saved and false if an error
 * occurred during this process.
 *
 * Writes the content string to a file of text type.
 * If the files exists it will be overwritten.
 *
 * In the case of an error during the process, the file will be closed,
 * the error logged and the function returns false.
 */
bool ProcessOutputWindow::saveToFile(const QString &content,
                                     const QString &filePath)
{
    QFile file(filePath);
    if(file.open(QIODevice::ReadWrite | QIODevice::Text)) {
        QTextStream stream(&file);

        stream << content;

        stream.flush();
        file.close();
        return true;
    }
    else {
        qCritical("An error ocurred when trying to export the output of "
                  "a process to a text file with path %s.",
                  qUtf8Printable(filePath));
        return false;
    }
}
