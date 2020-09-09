/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the implementation of the TerminalColorTable class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "terminalcolortable.h"

TerminalColorTable::TerminalColorTable()
{

}

TerminalColorTable::~TerminalColorTable()
{

}

QColor TerminalColorTable::black() const
{
    return QColor(1,1,1);
}

QColor TerminalColorTable::red() const
{
    return QColor(128, 0, 0);
}

QColor TerminalColorTable::green() const
{
    return QColor(0, 147, 0);
}

QColor TerminalColorTable::yellow() const
{
    return QColor(255, 199, 6);
}

QColor TerminalColorTable::blue() const
{
    return QColor(0, 0, 128);
}

QColor TerminalColorTable::magenta() const
{
    return QColor(180, 0, 158);
}

QColor TerminalColorTable::cyan() const
{
    return QColor(17, 168, 205);
}

QColor TerminalColorTable::white() const
{
    return QColor(204, 204, 204);
}

QColor TerminalColorTable::brightBlack() const
{
    return gray();
}

QColor TerminalColorTable::gray() const
{
    return QColor(85, 85, 85);
}

QColor TerminalColorTable::brightRed() const
{
    return QColor(255, 0, 0);
}

QColor TerminalColorTable::brightGreen() const
{
    return QColor(22, 198, 12);
}

QColor TerminalColorTable::brightYellow() const
{
    return QColor(255, 255, 0);
}

QColor TerminalColorTable::brightBlue() const
{
    return QColor(0, 0, 255);
}

QColor TerminalColorTable::brightMagenta() const
{
    return QColor(255, 0, 255);
}

QColor TerminalColorTable::brightCyan() const
{
    return QColor(0, 255, 255);
}

QColor TerminalColorTable::brightWhite() const
{
    return QColor(242, 242, 242);
}

QColor TerminalColorTable::lookup88ColorCode(int code)
{
    if(code < 16) {
        return lookup16BaseColors(code);
    }
    else if(code < 72) {
        // Separate the code into the RGB components
        int b = (code - 8) % 4;
        int g = ((code - 8 - b) / 4) % 4;
        int r = (code - 8 - b - 4 * g) / 16;

        // Scale the components into the RGB space
        b = (b == 1 ? 136 : (b == 2 ? 204 : (b == 3 ? 255 : 0)));
        g = (g == 1 ? 136 : (g == 2 ? 204 : (g == 3 ? 255 : 0)));
        r = (r == 1 ? 136 : (r == 2 ? 204 : (r == 3 ? 255 : 0)));

        // Create the color
        return QColor(r, g, b);
    }
    // Grayscale pallete
    return QColor::fromHsl(0, 0, 6 * code - 429);
}

QColor TerminalColorTable::lookup256ColorCode(int code)
{
    if(code < 16) {
        return lookup16BaseColors(code);
    }
    if(code < 232) {
        // Separate the code in RGB components
        int b = (code - 16) % 6;
        int g = ((code - 16 - b)/6) % 6;
        int r = (code - 16 - b - 6*g) / 36;

        // Scale the components into the RGB space
        r = r == 0 ? 0 : 40*r + 55;
        g = g == 0 ? 0 : 40*g + 55;
        b = b == 0 ? 0 : 40*b + 55;

        return QColor(r, g, b);
    }
    int grayValue = 10 * code - 2312;
    return QColor(grayValue, grayValue, grayValue);
}

QColor TerminalColorTable::lookup16BaseColors(int code)
{
    switch(code) {
    case 0:
        return QColor(0, 0, 0);
    case 1:
        return QColor(128, 0, 0);
    case 2:
        return QColor(0, 128, 0);
    case 3:
        return QColor(128, 128, 0);
    case 4:
        return QColor(0, 0, 128);
    case 5:
        return QColor(128, 0, 128);
    case 6:
        return QColor(0, 128, 128);
    case 7:
        return QColor(192, 192, 192);
    case 8:
        return QColor(128, 128, 128);
    case 9:
        return QColor(255, 0, 0);
    case 10:
        return QColor(0, 255, 0);
    case 11:
        return QColor(255, 255, 0);
    case 12:
        return QColor(0, 0, 255);
    case 13:
        return QColor(255, 0, 255);
    case 14:
        return QColor(0, 255, 255);
    case 15:
        return QColor(255, 255, 255);
    }
    return QColor();
}
