#ifndef FILEBROWSERWIDGET_H
#define FILEBROWSERWIDGET_H

#include <QWidget>

#include <QDir>

namespace Ui {
class FileBrowserWidget;
}

/**
 * @brief A Widget to allow for the selection of files or directories and
 * display the selected path.
 *
 * This class allows for setting the type of File Dialog that will be shown to
 * the user, as its caption, initial directory and file filter when appropriate.
 */
class FileBrowserWidget : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(BrowserType browserType
               READ browserType
               WRITE setBrowserType)
    Q_PROPERTY(QString browserDialogCaption
               READ browserDialogCaption
               WRITE setBrowserDialogCaption)
    Q_PROPERTY(QString browserDir
               READ browserDir
               WRITE setBrowserDir)
    Q_PROPERTY(QString fileFilter
               READ fileFilter
               WRITE setFileFilter)
    Q_PROPERTY(QString browserButtonTooltip
               READ browserButtonTooltip
               WRITE setBrowserButtonTooltip)

public:
    explicit FileBrowserWidget(QWidget *parent = nullptr);
    ~FileBrowserWidget();

    //! Available types of browser dialog.
    enum BrowserType {
        OpenFile,
        SaveFile,
        Directory
    };
    Q_ENUM(BrowserType)

    //! Returns the path of the currently selected file.
    const QString filePath() const;

    //! Returns the current type of browser dialog selected.
    BrowserType browserType() const;
    //! Defines the selected type of browser dialog.
    void setBrowserType(BrowserType browserType);

    //! Returns the current text displayed in the caption of the browser dialog.
    const QString &browserDialogCaption() const;
    //! Defines the text that will be displayed in the caption of the browser
    //! dialog.
    void setBrowserDialogCaption(const QString &caption);

    //! Returns the path to the current browser directory.
    const QString &browserDir() const;

    //! Returns the string containg the currently used file filter.
    const QString &fileFilter() const;
    //! Defines the file filter used in the browser dialog.
    void setFileFilter(const QString &filter);

    //! The text displayed in the ToolButton used to open the file browser
    //! dialog.
    const QString &browserButtonTooltip() const;
    //! Defines the text displayed in the tool tip of the button used to open
    //! the file browser dialog.
    void setBrowserButtonTooltip(const QString &tooltip);

signals:
    /**
     * @brief filePathChanged Signal emited when the file selected in the
     * widget has changed.
     * @param path The absolute path of the selected file.
     */
    void filePathChanged(const QString &path);

public slots:
    //! Defines the selected file displayed in the widget.
    void setFilePath(const QString &path);
    //! Defines the current browser directory.
    void setBrowserDir(const QString &path);

private slots:
    //! Shows the file browser dialog.
    void on_browserButton_clicked();

private:
    //! Pointer to the user interface object containing the widgets defined in
    //! this window form.
    Ui::FileBrowserWidget *ui;

    //! Stores the type of the file browser dialog shown to the user.
    BrowserType _browserType = BrowserType::OpenFile;
    //! The text displayed in the file browser dialog.
    QString _browserDialogCaption;
    //! The file filter used in the file browser dialog.
    QString _fileFilter = tr("All Files (*.*)");
    //! The current directory used to start the file browser dialog.
    QString _browserDir = QDir::homePath();
    //! The text displayed in the tool tip of the button used to show the file
    //! browser dialog.
    QString _browserButtonToolTip = tr("Open Browser");
};

#endif // FILEBROWSERWIDGET_H
