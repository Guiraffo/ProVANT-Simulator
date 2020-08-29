#ifndef PROCESSOUTPUTFORMATTER_H
#define PROCESSOUTPUTFORMATTER_H

#include <QObject>

#include <QList>
#include <QTextEdit>

#include "terminalcolortable.h"

enum class BufferMode {
    NormalText,
    FormattingCode,
    EscapedCode
};

class ProcessOutputFormatter : public QObject
{
    Q_OBJECT
public:
    ProcessOutputFormatter(QTextEdit *outputWidget,
                           const TerminalColorTable &colorTable);
    virtual ~ProcessOutputFormatter();
    void update(QChar chr);
    void update(QString &str);

protected:
    void applyOutputFormat();

private:
    QTextEdit *_outputWidget;
    TerminalColorTable _colorTable;
    QString _buffer;
    QList<int> _formatCodeBuffer;
    enum BufferMode _mode;

    QColor calculateColor(QList<int> *buffer);

    /**
     * @brief getRGBFromBuffer Reads 3 numbers from the formatting code buffer,
     * validates their values and returns a QColor with the corresponding RGB
     * code, or an empty QColor if validation fails.
     *
     * This method is a part of the formatting code for setting the color of
     * the foreground and background of the text.
     * For more details @see calculateColor() and applyOutputFormat().
     *
     * @param buffer List of integers read from the formatting code.
     * @return QColor with a RGB value or an empty QColor if validation fails.
     */
    QColor getRGBFromBuffer(QList<int> *buffer);

    /**
     * @brief getCMYFromBuffer Reads a CMY color from the buffer and returns
     * a corresponding QColor object.
     *
     * Reads 3 numbers from the buffer, validate their values and if all
     * validation passses, returns a corresponding QColor object.
     *
     * This method is a part of the formatting code for setting the color of
     * the foreground and background of the text.
     * For more details @see calculateColor() and applyOutputFormat().
     *
     * @param bufffer List of integers read from the formatting code.
     * @return QColor with a CMY value or an empty QColor if validation fails.
     */
    QColor getCMYFromBuffer(QList<int> *bufffer);

    /**
     * @brief getCMYKFromBuffer Reads a CMYK color from the buffer and returns
     * a corresponding QColor object.
     *
     * Reads 4 numbers from the buffer, validate their values and if all
     * validation passes, returns a corresponding QColor object.
     *
     * This method is a part of the formatting code for setting the color of
     * the foreground and background of the text.
     * For more details @see calculateColor() and applyOutputFormat().
     *
     * @param buffer List of integers read from the formatting code.
     * @return QColor with a CMYK value or an empty color if validation fails.
     */
    QColor getCMYKFromBuffer(QList<int> *buffer);

    /**
     * @brief getIndexedColorFromBuffer Returns a QColor from an indexed color
     * table lookup.
     *
     * Reads an integer from the buffer, validates the number, and if all
     * validation passes, returns the corresponding value in the 256 values
     * indexed color table, defined in acoordance with ISO 8613-6.
     *
     * This method is a part of the formatting code for setting the color of
     * the foreground and background of the text.
     * For more details @see calculateColor() and applyOutputFormat().
     *
     * @param buffer List of integers read from the formatting code.
     * @return QColor with an RGB value or an empty QColor if validation fails.
     */
    QColor getIndexedColorFromBuffer(QList<int> *buffer);

    /**
     * @brief getIndexed88ColorFromBuffer Returns a QColor from an indexed
     * color table compatible with the 88 colors implementation.
     *
     * Reads an integer from the buffer, validates the number, and if all
     * validation passess, returns the corresponding value in the 88 color
     * lookup table.
     *
     * This method is a part of the formatting code for setting the color of
     * the foreground and background of the text.
     * For more details @see calculateColor() and applyOutputFormat().
     *
     * This is a compatibility mode and replaces the 256 color table mode in
     * case it is enabled.
     *
     * @param buffer List of integers read from the formatting code.
     * @return QColor with an RGB value corresponding to the 88 colors lookup
     * table or an empty color if validation fails.
     */
    QColor getIndexed88ColorFromBuffer(QList<int> *buffer);
};

#endif // PROCESSOUTPUTFORMATTER_H
