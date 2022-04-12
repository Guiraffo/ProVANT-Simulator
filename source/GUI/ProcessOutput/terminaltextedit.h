#ifndef TERMINALTEXTEDIT_H
#define TERMINALTEXTEDIT_H

#include <QTextEdit>
#include <QScopedPointer>

#include "terminalcolortable.h"

enum class BufferMode
{
  NormalText,
  FormattingCode,
  EscapedCode
};

class TerminalTextEdit : public QTextEdit
{
  Q_OBJECT
public:
  explicit TerminalTextEdit(QWidget* parent = nullptr);

public slots:
  void append(const QString& text);
  void insertPlainText(const QString& text);
  void enableInteraction();
  void disableInteraction();
  void setTextInteractionState(bool enabled = true);

protected:
  void applyOutputFormat();
  void appendInternal(QChar chr);
  void appendInternal(const QString& str);

private:
  TerminalColorTable _colorTable;
  QString _buffer;
  QList<int> _formatCodeBuffer;
  enum BufferMode _mode;

  /**
   * @brief parseFormatBuffer Convert the string in the reading
   * buffer to an integer and adds it to the formatting code buffer.
   *
   * If the conversion fails, a warning message is emitted, the buffer is
   * cleared and execution proceeds as normal.
   */
  void parseFormatBuffer();

  /**
   * @brief calculateColor Helper method to determine a RGB color from the
   * parameters of a SGR formatting code.
   *
   * This method supports both the method of indexed colors (256 and a
   * compatibility mode with a 88 colors table), and the method wich specifies
   * the color based on a color space, with support to RGB, CMY and CMYK color
   * spaces.
   *
   * Optional support to HSL, HSV and RGBA can be added if necessary (but at
   * this time, since no terminal implements this, these modes are not
   * available).
   *
   * @param buffer Buffer with the formatting options to calculate the color.
   * @return
   */
  QColor calculateColor(QList<int>* buffer);

  /**
   * @brief resetOutputFormat Revert the text formatting to the default.
   *
   * The default options are black text color on white background, with no
   * underline, overline, italic, kerning, capitalization, with normal
   * alignment of text (not subscripted nor sobrescripted), with normal font
   * weight (not bold) and with black underline color.
   *
   * The default mode also disables framing, encircling, and hiding of the
   * text.
   *
   * The underline color is reverted to black to allow the correct usage
   * of the SGR parameters 58 and 59 that allow for setting the color of the
   * underline. This is not a standard but was copied from mintty.
   */
  void resetOutputFormat();

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
  QColor getRGBFromBuffer(QList<int>* buffer);

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
  QColor getCMYFromBuffer(QList<int>* bufffer);

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
  QColor getCMYKFromBuffer(QList<int>* buffer);

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
  QColor getIndexedColorFromBuffer(QList<int>* buffer);

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
  QColor getIndexed88ColorFromBuffer(QList<int>* buffer);

  /**
   * @brief _textHidden Indicates if text should be output or not.
   * Part of the implementation for the hidden text mode;
   */
  bool _textHidden = false;

  /**
   * @brief _textFramed Indciates if the text should be put inside a frame
   * or not.
   */
  bool _textFramed = false;

  /**
   * @brief _textEncircled Indicates if the text should be put inside circles
   * or not (this mode could not be implemented without signficative changes
   * to Qt Rich Text Engine, so it is implemented as an framed mode with
   * inverted colors.
   */
  bool _textEncircled = false;

  QFont _defaultFont;
  QString _defaultFontFamily;
};

#endif  // TERMINALTEXTEDIT_H
