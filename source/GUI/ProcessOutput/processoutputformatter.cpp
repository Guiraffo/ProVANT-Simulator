#include "processoutputformatter.h"

#include <QDebug>

ProcessOutputFormatter::ProcessOutputFormatter(QTextEdit *outputWidget,
                                               const TerminalColorTable &colorTable) :
    _outputWidget(outputWidget),
    _colorTable(colorTable)
{

}

ProcessOutputFormatter::~ProcessOutputFormatter()
{

}

void ProcessOutputFormatter::update(QChar chr)
{
    switch(_mode) {
    case BufferMode::NormalText:
        if(chr == '\e') {
            _mode = BufferMode::FormattingCode;
            _buffer.clear();
            _formatCodeBuffer.clear();
        }
        else
            _outputWidget->append(chr);
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
            applyOutputFormat();
            _mode = BufferMode::NormalText;
        }
        // Verify if a formatting code block just finished
        else if(chr == ';') {
            if(_buffer.isEmpty()) {
                // Just ignore, there was an empty block before it
            }
            else {
                // Try to convert the buffer to an integer
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
        // Otherwise, add the text to the buffer
        else {
            _buffer.append(chr);
        }
    }
}

void ProcessOutputFormatter::update(QString &str)
{
    for(int i = 0; i < str.length(); i++) {
        update(str.at(i));
    }
}

void ProcessOutputFormatter::applyOutputFormat()
{
    while(!_formatCodeBuffer.isEmpty()) {
        QList<int> bufferCopy(_formatCodeBuffer);
        int fcode = *_formatCodeBuffer.begin();
        _formatCodeBuffer.pop_back();
        switch(fcode) {
        case 0:
            // Reset everything
            // Black text color
            _outputWidget->setTextColor(_colorTable.black());
            // White text background
            _outputWidget->setTextBackgroundColor(QColor(255, 255, 255));
            break;
        case 38:
            // Select foreground color.
            // This is a compound option dependent on other codes.
        case 39:
            // Reset text color to default (black)
            _outputWidget->setTextColor(_colorTable.black());
            break;
        case 30:
            // Set text color to black
            _outputWidget->setTextColor(_colorTable.black());
            break;
        case 31:
            // Set text color to red
            _outputWidget->setTextColor(_colorTable.red());
            break;
        case 32:
            // Set text color to green
            _outputWidget->setTextColor(_colorTable.green());
            break;
        case 33:
            // Set text color to yellow
            _outputWidget->setTextColor(_colorTable.yellow());
            break;
        case 34:
            // Set text color to blue
            _outputWidget->setTextColor(_colorTable.blue());
            break;
        case 35:
            // Set text color to magenta
            _outputWidget->setTextColor(_colorTable.magenta());
            break;
        case 36:
            // Set text color to cyan
            _outputWidget->setTextColor(_colorTable.cyan());
            break;
        case 37:
            // Set text color to light gray
            _outputWidget->setTextColor(_colorTable.white());
            break;
        case 90:
            // Set text color to dark gray (light black)
            _outputWidget->setTextColor(_colorTable.gray());
            break;
        case 91:
            // Set text color to bright red
            _outputWidget->setTextColor(_colorTable.brightRed());
            break;
        case 92:
            // Set text color to bright green
            _outputWidget->setTextColor(_colorTable.brightGreen());
            break;
        case 93:
            // Set text color to bright yellow
            _outputWidget->setTextColor(_colorTable.brightYellow());
            break;
        case 94:
            // Set text color to bright blue
            _outputWidget->setTextColor(_colorTable.brightBlue());
            break;
        case 95:
            // Set text color to bright magenta
            _outputWidget->setTextColor(_colorTable.brightMagenta());
            break;
        case 96:
            // Set text color to bright cyan
            _outputWidget->setTextColor(_colorTable.brightCyan());
            break;
        case 97:
            // Set text color to white
            _outputWidget->setTextColor(_colorTable.white());
            break;
        case 49:
            // Restore default (white) text background color
            _outputWidget->setTextBackgroundColor(QColor(255, 255, 255));
            break;
        case 40:
            // Set text background black
            _outputWidget->setTextBackgroundColor(_colorTable.black());
            break;
        case 41:
            // Set text background red
            _outputWidget->setTextBackgroundColor(_colorTable.red());
            break;
        case 42:
            // Set text background green
            _outputWidget->setTextBackgroundColor(_colorTable.green());
            break;
        case 43:
            // Set text background yellow
            _outputWidget->setTextBackgroundColor(_colorTable.yellow());
            break;
        case 44:
            // Set text background blue
            _outputWidget->setTextBackgroundColor(_colorTable.blue());
            break;
        case 45:
            // Set text background magenta
            _outputWidget->setTextBackgroundColor(_colorTable.magenta());
            break;
        case 46:
            // Set text background cyan
            _outputWidget->setTextBackgroundColor(_colorTable.cyan());
            break;
        case 47:
            // Set text background white
            _outputWidget->setTextBackgroundColor(_colorTable.white());
            break;
        case 100:
            // Set text background dark gray (bright black)
            _outputWidget->setTextBackgroundColor(_colorTable.brightBlack());
            break;
        case 101:
            // Set text background bright red
            _outputWidget->setTextBackgroundColor(_colorTable.brightRed());
            break;
        case 102:
            // Set text background bright green
            _outputWidget->setTextBackgroundColor(_colorTable.brightGreen());
            break;
        case 103:
            // Set text background bright yellow
            _outputWidget->setTextBackgroundColor(_colorTable.brightYellow());
            break;
        case 104:
            // Set text background bright blue
            _outputWidget->setTextBackgroundColor(_colorTable.brightBlue());
            break;
        case 105:
            // Set text background bright magenta
            _outputWidget->setTextBackgroundColor(_colorTable.brightMagenta());
            break;
        case 106:
            // Set text background bright cyan
            _outputWidget->setTextBackgroundColor(_colorTable.brightCyan());
            break;
        case 107:
            // Set text background bright white
            _outputWidget->setTextBackgroundColor(QColor(255, 255, 255));
            break;
        }
    }
}

QColor ProcessOutputFormatter::calculateColor(QList<int> *buffer)
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
                qWarning() << tr("Error while trying to set the text color."
                                 "Invalid Color-Space-ID (")
                           << val
                           << tr(") for a buffer with 1 element.");
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
                return getIndexedColorFromBuffer(buffer);
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
                qWarning() << tr("Error while trying to set the text color. "
                                 "Invalid Color-Space-ID field value (")
                           << val
                           << tr(") for a buffer with four elements");
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
                qWarning() << tr("Error while trying to set the text color. "
                                 "Invalid Color-Space-ID (")
                            << val
                            << tr("for a buffer with fivee elements.");
                return QColor();
            }
            // Logs the error and returns an empty color.
            qWarning() << tr("Error while trying to set the text color. "
                             "Invalid buffer length (.")
                       << buffer->length()
                       << tr(").");
            return QColor();
        }
        else if(value == 5) {
            // 256 color lookup table mode
            return getIndexedColorFromBuffer(buffer);
        }
        else {
            qWarning() << tr("Error while trying to set the text color. "
                             "Invalid CSI command with code (")
                       << value << ").";
            return QColor();
        }
    }
    qWarning() << tr("Error while trying to set the text color. "
                     "Empty buffer found.");
    return QColor();
}

QColor ProcessOutputFormatter::getRGBFromBuffer(QList<int> *buffer)
{
    if(buffer->length() < 3) {
        qWarning() << tr("Error while trying to set the color of the text. "
                         "The buffer has las than 3 parameters and therefore "
                         "an RGB value cannot be read from it.");
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
        qWarning() << tr("Error while trying to set the color of the text. "
                         "Invalid value for the red channel (")
                   << r << ").";
        return QColor();
    }
    if(g < 0 || g > 255) {
        qWarning() << tr("Error while trying to set the color of the text. "
                         "Invalid value for the green channel (")
                   << g << ").";
        return QColor();
    }
    if(b < 0 || b > 255) {
        qWarning() << tr("Error while trying to set the color of the text."
                         "Invalid value for the blue channel (")
                   << b << ").";
        return QColor();
    }

    // If the color is valid.
    return QColor(r, g, b);
}

QColor ProcessOutputFormatter::getCMYFromBuffer(QList<int> *buffer)
{
    if(buffer->length() < 3) {
        qWarning() << tr("Error while trying to set the color of the text. "
                         "The buffer has las than 3 parameters and therefore "
                         "an CMY value cannot be read from it.");
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
        qWarning() << tr("Error while trying to set the color of the text. "
                         "Invalid value for the cyan channel (")
                   << c << ").";
        return QColor();
    }
    if(m < 0 || m > 255) {
        qWarning() << tr("Error while trying to set the color of the text. "
                         "Invalid value for the magenta channel (")
                   << m << ").";
        return QColor();
    }
    if(y < 0 || y > 255) {
        qWarning() << tr("Error while trying to set the color of the text."
                         "Invalid value for the yellow channel (")
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

QColor ProcessOutputFormatter::getCMYKFromBuffer(QList<int> *buffer)
{
    if(buffer->length() < 4) {
        qWarning() << tr("Error while trying to set the color of the text. "
                         "The buffer has las than 4 parameters and therefore "
                         "an CMYK value cannot be read from it.");
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
        qWarning() << tr("Error while trying to set the color of the text. "
                         "Invalid value for the cyan channel (")
                   << c << ").";
        return QColor();
    }
    if(m < 0 || m > 255) {
        qWarning() << tr("Error while trying to set the color of the text. "
                         "Invalid value for the magenta channel (")
                   << m << ").";
        return QColor();
    }
    if(y < 0 || y > 255) {
        qWarning() << tr("Error while trying to set the color of the text."
                         "Invalid value for the yellow channel (")
                   << y << ").";
        return QColor();
    }
    if(k < 0 || k > 255) {
        qWarning() << tr("Error while trying to set the color of the text."
                         "Invalid value for the black channel (")
                   << k << ").";
        return QColor();
    }

    return QColor::fromCmyk(c, m, y, k);
}

QColor ProcessOutputFormatter::getIndexedColorFromBuffer(QList<int> *buffer)
{
    if(buffer->isEmpty()) {
        qWarning() << tr("Error while trying to set the color of the text."
                         " The indexed color value was not present in "
                         "the buffer.");
        return QColor();
    }

    int code = *buffer->cbegin();
    buffer->pop_front();

    if(code < 0 || code > 255) {
        qWarning() << "Error while trying to find a RGB equivalent"
                   << " to color with code "
                   << code
                   << " in the 256 colors lookup table";
        return QColor();
    }

    return _colorTable.lookup256ColorCode(code);
}

QColor ProcessOutputFormatter::getIndexed88ColorFromBuffer(QList<int> *buffer)
{
    if(buffer->isEmpty()) {
        qWarning() << tr("Error while trying to set the color of the text."
                         " The indexed color value was not present in "
                         "the buffer.");
        return QColor();
    }

    int code = *buffer->cbegin();
    buffer->pop_front();

    if(code < 0 || code > 88) {
        qWarning() << "Error while trying to find a RGB equivalent"
                   << " to color with code "
                   << code
                   << " in the 88 colors lookup table";
        return QColor();
    }

    return _colorTable.lookup256ColorCode(code);
}
