#ifndef FILEBROWSERWIDGET_H
#define FILEBROWSERWIDGET_H

#include <QWidget>

#include <QDir>

namespace Ui {
class FileBrowserWidget;
}

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

    enum BrowserType {
        OpenFile,
        SaveFile,
        Directory
    };
    Q_ENUM(BrowserType)

    const QString filePath() const;

    BrowserType browserType() const;
    void setBrowserType(BrowserType browserType);

    const QString &browserDialogCaption() const;
    void setBrowserDialogCaption(const QString &caption);

    const QString &browserDir() const;
    void setBrowserDir(const QString &path);

    const QString &fileFilter() const;
    void setFileFilter(const QString &filter);

    const QString &browserButtonTooltip() const;
    void setBrowserButtonTooltip(const QString &tooltip);

signals:
    void filePathChanged(QString path);

public slots:
    void setFilePath(const QString &path);

private slots:
    void on_browserButton_clicked();

private:
    Ui::FileBrowserWidget *ui;

    BrowserType _browserType = BrowserType::OpenFile;
    QString _browserDialogCaption;
    QString _fileFilter = tr("All Files (*.*)");
    QString _browserDir = QDir::homePath();
    QString _browserButtonToolTip = tr("Open Browser");
};

#endif // FILEBROWSERWIDGET_H
