#include "terminaltextedit.h"

#include <QDebug>
#include <QTextCharFormat>

TerminalTextEdit::TerminalTextEdit(QWidget *parent) :
    QTextEdit(parent),
    _mode(BufferMode::NormalText)
{
    setReadOnly(true);
    setUndoRedoEnabled(false);
    setTextInteractionFlags(Qt::NoTextInteraction);

    // Stores default font
    _defaultFont = font();
    resetOutputFormat();
}

void TerminalTextEdit::append(const QString &text)
{
//    if(!textCursor().atEnd()) {
//        moveCursor(QTextCursor::End, QTextCursor::MoveAnchor);
//    }
    appendInternal(text);
}

void TerminalTextEdit::insertPlainText(const QString &text)
{
    if(_textHidden)
        QTextEdit::insertPlainText("*");
    else
        QTextEdit::insertPlainText(text);
}

void TerminalTextEdit::disableInteraction()
{
    setTextInteractionState(false);
}

void TerminalTextEdit::setTextInteractionState(bool enabled)
{
    if(enabled) {
        setTextInteractionFlags(Qt::TextBrowserInteraction);
    }
    else {
        setTextInteractionFlags(Qt::NoTextInteraction);
    }
}

void TerminalTextEdit::enableInteraction()
{
    setTextInteractionState(true);
}

void TerminalTextEdit::appendInternal(QChar chr)
{
    switch(_mode) {
    case BufferMode::NormalText:
        if(chr == '\e') {
            _mode = BufferMode::EscapedCode;
            _buffer.clear();
            _formatCodeBuffer.clear();
        }
        else {
            insertPlainText(chr);
        }
        break;
    case BufferMode::EscapedCode:
        // Check if a formatting section will start
        if(chr == '[') {
            _mode = BufferMode::FormattingCode;
        }
        // Otherwise, ignore the new character and return to normal mode
        else {
            _mode = BufferMode::NormalText;
        }
        break;
    case BufferMode::FormattingCode:
        // Verify if the formatting code just finihes
        if(chr == 'm') {
            parseFormatBuffer();
            applyOutputFormat();
            _mode = BufferMode::NormalText;
        }
        // Verify if a formatting code block just finished
        else if(chr == ';') {
            if(_buffer.isEmpty()) {
                // Just ignore, there was an empty block before it
            }
            else {
                parseFormatBuffer();
            }
        }
        // Otherwise, add the text to the buffer
        else {
            _buffer.append(chr);
        }
    }
}

void TerminalTextEdit::appendInternal(const QString &str)
{
    for(int i = 0; i < str.length(); i++) {
        appendInternal(str.at(i));
    }
}

void TerminalTextEdit::parseFormatBuffer()
{
    if(!_buffer.isEmpty())
    {
        bool conversionOk = false;
        int res = _buffer.toInt(&conversionOk);
        if(conversionOk) {
            _formatCodeBuffer.append(res);
        }
        else {
            qDebug() << "Error while trying to convert the format code "
                     << _buffer << " to an integer. This unrecognized "
                     << "formatting code will be igndored.";
        }
        _buffer.clear();
    }
}

void TerminalTextEdit::applyOutputFormat()
{
    if(_formatCodeBuffer.isEmpty()) {
        // Resets format
        resetOutputFormat();
    }
    while(!_formatCodeBuffer.isEmpty()) {
        QList<int> bufferCopy(_formatCodeBuffer);
        int fcode = *_formatCodeBuffer.begin();
        _formatCodeBuffer.pop_front();
        switch(fcode) {
        case 0:
            resetOutputFormat();
            break;
        case 1:
        {
            // Bold text
            QTextCharFormat newFormat;
            newFormat.setFontWeight(QFont::Weight::Bold);
            mergeCurrentCharFormat(newFormat);
        }
            break;
        case 2:
        {
            // Dimmed text
            // Get current color
            QColor color = textColor();
            color.setAlpha(128);
            setTextColor(color);
        }
            break;
        case 3:
        {
            // Italic text
            QTextCharFormat newFormat;
            newFormat.setFontItalic(true);
            mergeCurrentCharFormat(newFormat);
        }
            break;
        case 4:
        {
            // Underlined text
            QTextCharFormat newFormat;
            newFormat.setFontUnderline(true);
            mergeCurrentCharFormat(newFormat);
        }
            break;
        case 6:
            // Rapid blining text (not implemented, does the same thing as
            // normal blinking text
            __attribute__ ((fallthrough)); // Ignore warning
        case 5:
        {
            // Blinking text
            // Implemented as black font weight
            QTextCharFormat newFormat;
            newFormat.setFontWeight(QFont::Weight::Black);
            newFormat.setFontCapitalization(QFont::AllUppercase);
            mergeCurrentCharFormat(newFormat);
        }
            break;
        case 8:
            // Hidden text
            _textHidden = true;
            break;
        case 9:
        {
            // Crossed out text
            QTextCharFormat newFormat;
            newFormat.setFontStrikeOut(true);
            mergeCurrentCharFormat(newFormat);
        }
            break;
        case 10:
        {
            // Restore default font
            QTextCharFormat newFormat;
            newFormat.setFontStyleHint(QFont::AnyStyle);
            newFormat.setFontWeight(QFont::Weight::Normal);
            newFormat.setFontCapitalization(QFont::MixedCase);
            newFormat.setFont(_defaultFont);
            mergeCurrentCharFormat(newFormat);
        }
            break;
        case 11:
        {
            // Alternate font 1
            // Normal Serif Font
            QTextCharFormat fmt;
            fmt.setFontStyleHint(QFont::Serif);
            fmt.setFontFamily(fmt.font().defaultFamily());
            fmt.setFontWeight(QFont::Weight::Normal);
            mergeCurrentCharFormat(fmt);
        }
            break;
        case 12:
        {
            // Alternate font 2
            // Extra Light Serif Font
            QTextCharFormat fmt;
            fmt.setFontStyleHint(QFont::Serif);
            fmt.setFontFamily(fmt.font().defaultFamily());
            fmt.setFontWeight(QFont::Weight::ExtraLight);
            mergeCurrentCharFormat(fmt);
        }
            break;
        case 13:
        {
            // Alternate font 3
            // Normal Helvetica
            QTextCharFormat fmt;
            fmt.setFontStyleHint(QFont::Helvetica);
            fmt.setFontFamily(fmt.font().defaultFamily());
            fmt.setFontWeight(QFont::Weight::Normal);
            mergeCurrentCharFormat(fmt);
        }
            break;
        case 14:
        {
            // Alternate font 4
            // Extra Light Helvetica
            QTextCharFormat fmt;
            fmt.setFontStyleHint(QFont::Helvetica);
            fmt.setFontFamily(fmt.font().defaultFamily());
            fmt.setFontWeight(QFont::Weight::ExtraLight);
            mergeCurrentCharFormat(fmt);
        }
            break;
        case 15:
        {
            // Alternate font 5
            // Normal Decorative
            QTextCharFormat fmt;
            fmt.setFontStyleHint(QFont::Decorative);
            fmt.setFontFamily("Lobster Two");
            fmt.setFontWeight(QFont::Weight::Normal);
            mergeCurrentCharFormat(fmt);
        }
            break;
        case 16:
        {
            // Alternate font 6
            // Extra Light Decorative
            QTextCharFormat fmt;
            fmt.setFontStyleHint(QFont::Decorative);
            fmt.setFontFamily("Lobster Two");
            fmt.setFontWeight(QFont::Weight::ExtraLight);
            mergeCurrentCharFormat(fmt);
        }
            break;
        case 17:
        {
            // Alternate font 7
            // Normal Cursive
            QTextCharFormat fmt;
            fmt.setFontStyleHint(QFont::Fantasy);
            fmt.setFontFamily("UnPilgia");
            fmt.setFontWeight(QFont::Weight::Normal);
            mergeCurrentCharFormat(fmt);
        }
            break;
        case 18:
        {
            // Alternate font 8
            // Extra Light Cursive
            QTextCharFormat fmt;
            fmt.setFontStyleHint(QFont::Fantasy);
            fmt.setFontFamily("UnPilgia");
            fmt.setFontWeight(QFont::Weight::ExtraLight);
            mergeCurrentCharFormat(fmt);
        }
            break;
        case 19:
        {
            // Alternate font 9
            // Small Capps Helvetica
            QTextCharFormat fmt;
            fmt.setFontStyleHint(QFont::Helvetica);
            fmt.setFontFamily(fmt.font().defaultFamily());
            fmt.setFontWeight(QFont::Weight::Normal);
            fmt.setFontCapitalization(QFont::SmallCaps);
            mergeCurrentCharFormat(fmt);
        }
            break;
        case 20:
        {
            // Fraktur text
            // Based on MathHax_Fraktur font
            QTextCharFormat newFormat;
            newFormat.setFontStyleHint(QFont::Fantasy);
            newFormat.setFontFamily("MathJax_Fraktur");
            newFormat.setFontWeight(QFont::Weight::Normal);
            mergeCurrentCharFormat(newFormat);
        }
            break;
        case 21:
        {
            // Reset bold font
            QTextCharFormat newFormat;
            newFormat.setFontWeight(QFont::Weight::Normal);
            mergeCurrentCharFormat(newFormat);
        }
            break;
        case 22:
        {
            // Reset dimmed text
            QColor color = textColor();
            color.setAlpha(255);
            setTextColor(color);
        }
            break;
        case 23:
        {
            // Reverse Italic and Frakftur text
            QTextCharFormat newFormat;
            newFormat.setFontItalic(false);
            newFormat.setFontCapitalization(QFont::MixedCase);
            mergeCurrentCharFormat(newFormat);
        }
            break;
        case 24:
        {
            // Reset Underlined text
            QTextCharFormat newFormat;
            newFormat.setFontUnderline(false);
            mergeCurrentCharFormat(newFormat);
        }
            break;
        case 25:
        {
            // Reset blinking text
            QTextCharFormat newFormat;
            newFormat.setFontWeight(QFont::Weight::Normal);
            newFormat.setFontCapitalization(QFont::Capitalization::MixedCase);
            mergeCurrentCharFormat(newFormat);
        }
            break;
        case 26:
        {
            // Enable font proportional spacing
            QTextCharFormat fmt;
            fmt.setFontStyleHint(QFont::TypeWriter);
            fmt.setFont(QFont("Monospace"));
            mergeCurrentCharFormat(fmt);
        }
            break;
        case 28:
            _textHidden = false;
            break;
        case 29:
        {
            // Reversed crossed out text
            QTextCharFormat newFormat;
            newFormat.setFontStrikeOut(false);
            mergeCurrentCharFormat(newFormat);
        }
            break;
        case 27:
            // Reset color inversion (is implemented inverting the text again)
            __attribute__ ((fallthrough)); // Ignore warning
        case 7:
        {
            QColor foreground = textColor();
            QColor background = textBackgroundColor();
            setTextColor(background);
            setTextBackgroundColor(foreground);
        }
            break;
        case 50:
        {
            // Reset font proportional spacing
            QTextCharFormat fmt;
            fmt.setFontStyleHint(QFont::AnyStyle);
            fmt.setFont(_defaultFont);
            mergeCurrentCharFormat(fmt);
        }
            break;
        case 52:
            // Encircled text (implemented as inverted framed text
        {
            QColor foreground = textColor();
            QColor background = textBackgroundColor();
            setTextColor(background);
            setTextBackgroundColor(foreground);
            _textEncircled = true;
        }
             __attribute__ ((fallthrough)); // Ignore warning
        case 51:
            // Framed text
        {
            _textFramed = true;
            QTextCharFormat newFormat;
            newFormat.setFontOverline(true);
            newFormat.setFontUnderline(true);
            mergeCurrentCharFormat(newFormat);
        }
            break;
        case 53:
        {
            // Implement overlined mode
            QTextCharFormat newFormat;
            newFormat.setFontOverline(true);
            mergeCurrentCharFormat(newFormat);
        }
            break;
        case 54:
            // Reverse framed text
            if(_textFramed)
            {
                _textFramed = false;
                QTextCharFormat newFormat;
                newFormat.setFontOverline(false);
                newFormat.setFontUnderline(false);
                mergeCurrentCharFormat(newFormat);
            }
            if(_textEncircled)
            {
                QColor foreground = textColor();
                QColor background = textBackgroundColor();
                setTextColor(background);
                setTextBackgroundColor(foreground);
                _textEncircled = false;
            }
            break;
        case 55:
        {
            // Reset overlined mode
            QTextCharFormat newFormat;
            newFormat.setFontOverline(false);
            mergeCurrentCharFormat(newFormat);
        }
            break;
        case 73:
        {
            // Implement superscript text mode from mintty
            QTextCharFormat format = currentCharFormat();
            if(format.verticalAlignment() ==
                    QTextCharFormat::AlignNormal ||
               format.verticalAlignment() ==
                    QTextCharFormat::AlignSubScript)
            {
                format.setVerticalAlignment(QTextCharFormat::AlignSuperScript);
            }
            else if(format.verticalAlignment() ==
                    QTextCharFormat::AlignSuperScript)
            {
                format.setVerticalAlignment(QTextCharFormat::AlignNormal);
            }

            mergeCurrentCharFormat(format);
        }
            break;
        case 74:
        {
            // Implement subscript text mode from mintty
            QTextCharFormat format = currentCharFormat();
            if(format.verticalAlignment() ==
                    QTextCharFormat::AlignNormal ||
               format.verticalAlignment() ==
                    QTextCharFormat::AlignSuperScript)
            {
                format.setVerticalAlignment(QTextCharFormat::AlignSubScript);
            }
            else if(format.verticalAlignment() ==
                    QTextCharFormat::AlignSubScript)
            {
                format.setVerticalAlignment(QTextCharFormat::AlignNormal);
            }

            mergeCurrentCharFormat(format);
        }
            break;
        case 38:
        {
            // Select foreground color.
            // This is a compound option dependent on other codes.
            QColor color = calculateColor(&_formatCodeBuffer);
            if(color.isValid()) {
                setTextColor(color);
            }
        }
            break;
        case 48:
        {
            // Select background color.
            // This is a compound option dependent on other codes.
            QColor color = calculateColor(&_formatCodeBuffer);
            if(color.isValid()) {
                setTextBackgroundColor(color);
            }
        }
            break;
        case 58:
        {
            // Set underline color mode (not standard, implemented as in mintty)
            QColor color = calculateColor(&_formatCodeBuffer);
            if(color.isValid()) {
                QTextCharFormat fmt;
                fmt.setUnderlineColor(color);
                mergeCurrentCharFormat(fmt);
            }
        }
            break;
        case 59:
        {
            // Set default underline color mode (not standard,
            // implemented as in mintty).
            QTextCharFormat fmt;
            fmt.setUnderlineColor(_colorTable.black());
            mergeCurrentCharFormat(fmt);
        }
            break;
        case 39:
            // Reset text color to default (black)
            setTextColor(_colorTable.black());
            break;
        case 30:
            // Set text color to black
            setTextColor(_colorTable.black());
            break;
        case 31:
            // Set text color to red
            setTextColor(_colorTable.red());
            break;
        case 32:
            // Set text color to green
            setTextColor(_colorTable.green());
            break;
        case 33:
            // Set text color to yellow
            setTextColor(_colorTable.yellow());
            break;
        case 34:
            // Set text color to blue
            setTextColor(_colorTable.blue());
            break;
        case 35:
            // Set text color to magenta
            setTextColor(_colorTable.magenta());
            break;
        case 36:
            // Set text color to cyan
            setTextColor(_colorTable.cyan());
            break;
        case 37:
            // Set text color to light gray
            setTextColor(_colorTable.white());
            break;
        case 90:
            // Set text color to dark gray (light black)
            setTextColor(_colorTable.gray());
            break;
        case 91:
            // Set text color to bright red
            setTextColor(_colorTable.brightRed());
            break;
        case 92:
            // Set text color to bright green
            setTextColor(_colorTable.brightGreen());
            break;
        case 93:
            // Set text color to bright yellow
            setTextColor(_colorTable.brightYellow());
            break;
        case 94:
            // Set text color to bright blue
            setTextColor(_colorTable.brightBlue());
            break;
        case 95:
            // Set text color to bright magenta
            setTextColor(_colorTable.brightMagenta());
            break;
        case 96:
            // Set text color to bright cyan
            setTextColor(_colorTable.brightCyan());
            break;
        case 97:
            // Set text color to white
            setTextColor(_colorTable.white());
            break;
        case 49:
            // Restore default (white) text background color
            setTextBackgroundColor(QColor(255, 255, 255));
            break;
        case 40:
            // Set text background black
            setTextBackgroundColor(_colorTable.black());
            break;
        case 41:
            // Set text background red
            setTextBackgroundColor(_colorTable.red());
            break;
        case 42:
            // Set text background green
            setTextBackgroundColor(_colorTable.green());
            break;
        case 43:
            // Set text background yellow
            setTextBackgroundColor(_colorTable.yellow());
            break;
        case 44:
            // Set text background blue
            setTextBackgroundColor(_colorTable.blue());
            break;
        case 45:
            // Set text background magenta
            setTextBackgroundColor(_colorTable.magenta());
            break;
        case 46:
            // Set text background cyan
            setTextBackgroundColor(_colorTable.cyan());
            break;
        case 47:
            // Set text background white
            setTextBackgroundColor(_colorTable.white());
            break;
        case 100:
            // Set text background dark gray (bright black)
            setTextBackgroundColor(_colorTable.brightBlack());
            break;
        case 101:
            // Set text background bright red
            setTextBackgroundColor(_colorTable.brightRed());
            break;
        case 102:
            // Set text background bright green
            setTextBackgroundColor(_colorTable.brightGreen());
            break;
        case 103:
            // Set text background bright yellow
            setTextBackgroundColor(_colorTable.brightYellow());
            break;
        case 104:
            // Set text background bright blue
            setTextBackgroundColor(_colorTable.brightBlue());
            break;
        case 105:
            // Set text background bright magenta
            setTextBackgroundColor(_colorTable.brightMagenta());
            break;
        case 106:
            // Set text background bright cyan
            setTextBackgroundColor(_colorTable.brightCyan());
            break;
        case 107:
            // Set text background bright white
            setTextBackgroundColor(QColor(255, 255, 255));
            break;
        }
    }
}

QColor TerminalTextEdit::calculateColor(QList<int> *buffer)
{
    if(!buffer->isEmpty()) {
        // Get next value in the buffer
        int value = *buffer->cbegin();
        buffer->pop_front();
        if(value == 2) {
            // 24 bit color mode
            // Implementing support for ITU T.416 color setup
            if(buffer->length() == 1) {
                /*
                 * If the buffer has only one parameter it is either
                 * 0 -> Implemenation defined (returns black)
                 * 1 -> Transparent (returns black with 0 alpha)
                 */
                int val = *buffer->cbegin();
                buffer->pop_front();
                if(val == 0) {
                    return QColor(_colorTable.black());
                }
                if(val == 1) {
                    return QColor(0, 0, 0, 0);
                }
                // Logs the error and returns an empty color
                qWarning() << QObject::tr("Error while trying to set the text"
                                          " color. Invalid Color-Space-ID (")
                           << val
                           << QObject::tr(") for a buffer with 1 element.");
                return QColor();
            }
            if(buffer->length() == 2) {
                /*
                 * If the buffer has two more parameters, the only valid
                 * implementation would be the Color-Space-ID equal to five.
                 * That is equivalent to using CSI 38;5 or 48;5, that
                 * corresponds to setting the color based on a 256 color indexed
                 * table.
                 */
                int val = *buffer->cbegin();
                buffer->pop_front();
                if(val == 5) {
                    return getIndexedColorFromBuffer(buffer);
                }
                // Log the error and returns an empty color
                qWarning() << tr("Error while trying to set the text color. "
                                 "Invalid Color-Space-ID field value (")
                           << val << ").";
                return QColor();
            }
            if(buffer->length() == 3) {
                /*
                 * If there are 3 values on the buffer, either the
                 * default value was passed (empty Color-Space-ID field), or
                 * the implementation used in KDE console is being used.
                 * Both have the same result of setting the color based on
                 * 3 parameters of the RGB values.
                 */
                return getRGBFromBuffer(buffer);
            }
            if(buffer->length() == 4) {
                /*
                 * If the buffer has size four, that are two possible options:
                 * 2 -> RGB
                 * 3 -> CMY
                 */
                int val = *buffer->cbegin();
                buffer->pop_front();
                if(val == 2) {
                    return getRGBFromBuffer(buffer);
                }
                if(val == 3) {
                    return getCMYFromBuffer(buffer);
                }
                // Logs the error and returns an empty color.
                qWarning() << QObject::tr("Error while trying to set the text "
                                          "color. Invalid Color-Space-ID field "
                                          "value (")
                           << val
                           << QObject::tr(") for a buffer with four elements");
                return QColor();
            }
            if(buffer->length() == 5) {
                /*
                 * If the buffer has size five, the only valid option is
                 * 4 -> CMYK.
                 */
                int val = *buffer->cbegin();
                buffer->pop_front();
                if(val == 4) {
                    return getCMYKFromBuffer(buffer);
                }
                // Logs the error and returns an empty color.
                qWarning() << QObject::tr("Error while trying to set the text "
                                         "color. Invalid Color-Space-ID (")
                           << val
                           << QObject::tr("for a buffer with fivee elements.");
                return QColor();
            }
            // Logs the error and returns an empty color.
            qWarning() << QObject::tr("Error while trying to set the text "
                                      "color. Invalid buffer length (.")
                       << buffer->length()
                       << ").";
            return QColor();
        }
        else if(value == 5) {
            // 256 color lookup table mode
            return getIndexedColorFromBuffer(buffer);
        }
        else {
            qWarning() << QObject::tr("Error while trying to set the text "
                                      "color. Invalid CSI command with code (")
                       << value
                       << ").";
            return QColor();
        }
    }
    qWarning() << QObject::tr("Error while trying to set the text color. "
                              "Empty buffer found.");
    return QColor();
}

void TerminalTextEdit::resetOutputFormat()
{
    // Reset everything
    // Black text color
    setTextColor(_colorTable.black());
    // White text background
    setTextBackgroundColor(QColor(255, 255, 255));
    // Reset font configuration
    QTextCharFormat newFormat;
    newFormat.setFontWeight(QFont::Weight::Normal);
    newFormat.setFontUnderline(false);
    newFormat.setUnderlineColor(_colorTable.black());
    newFormat.setFontStrikeOut(false);
    newFormat.setFontItalic(false);
    newFormat.setFontKerning(false);
    newFormat.setFontOverline(false);
    newFormat.setFontFixedPitch(false);
    newFormat.setFontCapitalization(QFont::MixedCase);
    newFormat.setFontStyleHint(QFont::AnyStyle);
    newFormat.setFont(_defaultFont);
    newFormat.setVerticalAlignment(
                QTextCharFormat::VerticalAlignment::AlignNormal);
    mergeCurrentCharFormat(newFormat);
    // Reset output configuration
    _textHidden = false;
    _textFramed = false;
    _textEncircled = false;
}

QColor TerminalTextEdit::getRGBFromBuffer(QList<int> *buffer)
{
    if(buffer->length() < 3) {
        qWarning() << QObject::tr("Error while trying to set the color of the"
                                  " text. The buffer has las than 3 parameters "
                                  "and therefore a RGB value cannot be read "
                                  "from it.");
        return QColor();
    }

    int r = *buffer->cbegin();
    buffer->pop_front();
    int g = *buffer->cbegin();
    buffer->pop_front();
    int b = *buffer->cbegin();
    buffer->pop_front();

    // Validate the numbers
    if(r < 0 || r > 255) {
        qWarning() << QObject::tr("Error while trying to set the color of the "
                                  "text. Invalid value for the red channel (")
                   << r
                   << ").";
        return QColor();
    }
    if(g < 0 || g > 255) {
        qWarning() << QObject::tr("Error while trying to set the color of the "
                                  "text. Invalid value for the green channel (")
                   << g
                   << ").";
        return QColor();
    }
    if(b < 0 || b > 255) {
        qWarning() << QObject::tr("Error while trying to set the color of the "
                                  "text. Invalid value for the blue channel (")
                   << b
                   << ").";
        return QColor();
    }

    // If the color is valid.
    return QColor(r, g, b);
}

QColor TerminalTextEdit::getCMYFromBuffer(QList<int> *buffer)
{
    if(buffer->length() < 3) {
        qWarning() << QObject::tr("Error while trying to set the color of the "
                                  "text. The buffer has las than 3 parameters "
                                  "and therefore a CMY value cannot be read "
                                  "from it.");
        return QColor();
    }

    int c = *buffer->cbegin();
    buffer->pop_front();
    int m = *buffer->cbegin();
    buffer->pop_front();
    int y = *buffer->cbegin();
    buffer->pop_front();

    // Validate the numbers
    if(c < 0 || c > 255) {
        qWarning() << QObject::tr("Error while trying to set the color of the "
                                  "text. Invalid value for the cyan channel (")
                   << c << ").";
        return QColor();
    }
    if(m < 0 || m > 255) {
        qWarning() << QObject::tr("Error while trying to set the color of the "
                                  "text. Invalid value for the magenta channel "
                                  "(")
                   << m << ").";
        return QColor();
    }
    if(y < 0 || y > 255) {
        qWarning() << QObject::tr("Error while trying to set the color of the "
                                  "text. Invalid value for the yellow channel "
                                  "(")
                   << y << ").";
        return QColor();
    }

    /*
     * If the color is valid, convert the value to CMYK with the method
     * found in http://color.lukas-stratmann.com/color-systems/cmy.html.
     */
    int k = std::min(c, std::min(m, y));
    c -= k;
    m -= k;
    y -= k;

    return QColor::fromCmyk(c, m, y, k);
}

QColor TerminalTextEdit::getCMYKFromBuffer(QList<int> *buffer)
{
    if(buffer->length() < 4) {
        qWarning() << QObject::tr("Error while trying to set the color of the "
                                  "text. The buffer has las than 4 parameters "
                                  "and therefore a CMYK value cannot be read "
                                  "from it.");
        return QColor();
    }

    int c = *buffer->cbegin();
    buffer->pop_front();
    int m = *buffer->cbegin();
    buffer->pop_front();
    int y = *buffer->cbegin();
    buffer->pop_front();
    int k = *buffer->cbegin();
    buffer->pop_front();

    // Validate the numbers
    if(c < 0 || c > 255) {
        qWarning() << QObject::tr("Error while trying to set the color of the "
                                  "text. Invalid value for the cyan channel (")
                   << c << ").";
        return QColor();
    }
    if(m < 0 || m > 255) {
        qWarning() << QObject::tr("Error while trying to set the color of the text. "
                         "Invalid value for the magenta channel (")
                   << m << ").";
        return QColor();
    }
    if(y < 0 || y > 255) {
        qWarning() << QObject::tr("Error while trying to set the color of the "
                                  "text. Invalid value for the yellow channel "
                                  "(")
                   << y << ").";
        return QColor();
    }
    if(k < 0 || k > 255) {
        qWarning() << QObject::tr("Error while trying to set the color of the "
                                  "text. Invalid value for the black channel (")
                   << k << ").";
        return QColor();
    }

    return QColor::fromCmyk(c, m, y, k);
}

QColor TerminalTextEdit::getIndexedColorFromBuffer(QList<int> *buffer)
{
    if(buffer->isEmpty()) {
        qWarning() << QObject::tr("Error while trying to set the color of the "
                                  "text. The indexed color value was not "
                                  "present in the buffer.");
        return QColor();
    }

    int code = *buffer->cbegin();
    buffer->pop_front();

    if(code < 0 || code > 255) {
        qWarning() << QObject::tr("Error while trying to find a RGB equivalent")
                   << QObject::tr(" to color with code ")
                   << code
                   << QObject::tr(" in the 256 colors lookup table");
        return QColor();
    }

    return _colorTable.lookup256ColorCode(code);
}

QColor TerminalTextEdit::getIndexed88ColorFromBuffer(QList<int> *buffer)
{
    if(buffer->isEmpty()) {
        qWarning() << QObject::tr("Error while trying to set the color of the "
                                  "text. The indexed color value was not "
                                  "present in the buffer.");
        return QColor();
    }

    int code = *buffer->cbegin();
    buffer->pop_front();

    if(code < 0 || code > 88) {
        qWarning() << QObject::tr("Error while trying to find a RGB equivalent")
                   << QObject::tr(" to color with code ")
                   << code
                   << QObject::tr(" in the 88 colors lookup table");
        return QColor();
    }

    return _colorTable.lookup256ColorCode(code);
}

