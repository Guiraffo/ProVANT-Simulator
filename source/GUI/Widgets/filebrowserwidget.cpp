#include "filebrowserwidget.h"
#include "ui_filebrowserwidget.h"

#include <QFileDialog>
#include <QFileInfo>

FileBrowserWidget::FileBrowserWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FileBrowserWidget)
{
    ui->setupUi(this);
}

FileBrowserWidget::~FileBrowserWidget()
{
    delete ui;
}

FileBrowserWidget::BrowserType FileBrowserWidget::browserType() const
{
    return _browserType;
}

void FileBrowserWidget::setBrowserType(FileBrowserWidget::BrowserType browserType)
{
    _browserType = browserType;
}

const QString FileBrowserWidget::filePath() const
{
    return ui->filePath->text();
}

const QString &FileBrowserWidget::browserDialogCaption() const
{
    return _browserDialogCaption;
}

void FileBrowserWidget::setBrowserDialogCaption(const QString &caption)
{
    _browserDialogCaption = caption;
}

const QString &FileBrowserWidget::browserDir() const
{
    return _browserDir;
}

void FileBrowserWidget::setBrowserDir(const QString &path)
{
    _browserDir = path;
}

const QString &FileBrowserWidget::fileFilter() const
{
    return _fileFilter;
}

void FileBrowserWidget::setFileFilter(const QString &filter)
{
    _fileFilter = filter;
}

const QString &FileBrowserWidget::browserButtonTooltip() const
{
    return _browserButtonToolTip;
}

void FileBrowserWidget::setBrowserButtonTooltip(const QString &tooltip)
{
    _browserButtonToolTip = tooltip;
    ui->browserButton->setToolTip(tooltip);
}

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
