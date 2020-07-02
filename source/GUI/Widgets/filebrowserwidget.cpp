#include "filebrowserwidget.h"
#include "ui_filebrowserwidget.h"

#include <QFileDialog>
#include <QFileInfo>

/*!
 * @brief FileBrowserWidget::FileBrowserWidget
 * @param parent The widget which contains this object.
 *
 * Initializes an object of the FileBrowserWidget type.
 * Performs the initialization of QWidget and constructs the user interface
 * using the parameteres defined in the form.
 */
FileBrowserWidget::FileBrowserWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FileBrowserWidget)
{
    ui->setupUi(this);
}

/*!
 * @brief FileBrowserWidget::~FileBrowserWidget
 *
 * Force deletion of the ui objects built when this object is instantiated.
 */
FileBrowserWidget::~FileBrowserWidget()
{
    delete ui;
}

/*!
 * @return The type of the file browser shown to the user when the browse
 * toolbutton is clicked.
 */
FileBrowserWidget::BrowserType FileBrowserWidget::browserType() const
{
    return _browserType;
}

/*!
 * \brief FileBrowserWidget::setBrowserType
 * \param browserType The type of the browser window that will be shown to the
 * user.
 */
void FileBrowserWidget::setBrowserType(
        FileBrowserWidget::BrowserType browserType)
{
    _browserType = browserType;
}

/*!
 * \brief FileBrowserWidget::filePath
 * \return The path of the file currently selected in the widget.
 */
const QString FileBrowserWidget::filePath() const
{
    return ui->filePath->text();
}

/*!
 * \brief FileBrowserWidget::browserDialogCaption
 * \return The text displayed in the title of the browser dialog.
 */
const QString &FileBrowserWidget::browserDialogCaption() const
{
    return _browserDialogCaption;
}

/*!
 * \brief FileBrowserWidget::setBrowserDialogCaption
 * \param caption The text displayed in the title of the browser dialog.
 */
void FileBrowserWidget::setBrowserDialogCaption(const QString &caption)
{
    _browserDialogCaption = caption;
}

/*!
 * \brief FileBrowserWidget::browserDir
 * \return The starting directory of the file browser dialog.
 */
const QString &FileBrowserWidget::browserDir() const
{
    return _browserDir;
}

/*!
 * \brief FileBrowserWidget::setBrowserDir
 * \param path The starting directory of the file browser dialog.
 */
void FileBrowserWidget::setBrowserDir(const QString &path)
{
    _browserDir = path;
}

/*!
 * \brief FileBrowserWidget::fileFilter
 * \return The current file filter used in the file browser dialog.
 *
 * This option is only used when the type of the of the browser is not
 * directory.
 */
const QString &FileBrowserWidget::fileFilter() const
{
    return _fileFilter;
}

/*!
 * \brief FileBrowserWidget::setFileFilter
 * \param filter The file filter that will be used in the file browser dialog.
 *
 * See also @link fileFilter @endlink.
 */
void FileBrowserWidget::setFileFilter(const QString &filter)
{
    _fileFilter = filter;
}

/*!
 * \brief FileBrowserWidget::browserButtonTooltip
 * \return The text displayed in the tooltip of the tool button used to show
 * the file browser dialog.
 */
const QString &FileBrowserWidget::browserButtonTooltip() const
{
    return _browserButtonToolTip;
}

/*!
 * \brief FileBrowserWidget::setBrowserButtonTooltip
 * \param tooltip The text that will be displayed in the tool tip of the
 * button used to show the browser dialog.
 */
void FileBrowserWidget::setBrowserButtonTooltip(const QString &tooltip)
{
    _browserButtonToolTip = tooltip;
    ui->browserButton->setToolTip(tooltip);
}

/**
 * @brief FileBrowserWidget::setFilePath
 * @param path The path to the file displayed in the widget.
 *
 * Sets the file currently selected by the widget.
 * This function performs no check if the file exists or the path is valid.
 *
 * When setting a new file path, the starting directory of the file browser is
 * also updated, and if an invalid path is passed, the directory is the home
 * directory of the user.
 */
void FileBrowserWidget::setFilePath(const QString &path)
{
    if(path != ui->filePath->text()) {
        ui->filePath->setText(path);

        QFileInfo finfo(path);
        if(finfo.dir().exists()) {
            _browserDir = finfo.absoluteDir().path();
        }
        else {
            _browserDir = QDir::homePath();
        }

        emit filePathChanged(path);
    }
}

/*!
 * \brief FileBrowserWidget::on_browserButton_clicked
 *
 * Shows the appropriated file browser dialog based on the value of
 * the _browserType member variable.
 *
 * Currently, three options are available for use:
 * * Save File
 * * Open File
 * * Directory
 *
 * No option for selecting multiple files or returning the contents of the
 * files is provided.
 *
 * If a file is selected by the user, the path displayed in the widget is
 * updated and the signal filePathChanged is emited.
 *
 * If the user cancels the operation, there is no change to the state of the
 * widget.
 */
void FileBrowserWidget::on_browserButton_clicked()
{
    QString path;
    const QString dir;

    switch(_browserType) {
        case BrowserType::OpenFile:
            path = QFileDialog::getOpenFileName(
                        this,
                        _browserDialogCaption,
                        _browserDir,
                        _fileFilter);
            break;
        case BrowserType::SaveFile:
            path = QFileDialog::getSaveFileName(
                        this,
                        _browserDialogCaption,
                        _browserDir,
                        _fileFilter);
            break;
        case BrowserType::Directory:
            path = QFileDialog::getExistingDirectory(
                        this,
                        tr("Caption"),
                        _browserDir);
            break;
    }

    if(path != "") {
        setFilePath(path);
    }
}
