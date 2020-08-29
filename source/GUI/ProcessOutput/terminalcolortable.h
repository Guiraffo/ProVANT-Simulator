/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the declaration of the TerminalColorTable class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef TERMINALCOLORTABLE_H
#define TERMINALCOLORTABLE_H

#include <QColor>

/**
 * @brief The TerminalColorTable class provides a standard color table to
 * implement the color output of the processes shown in the ProcessOutputWindow.
 * This table defines all of the options contained in the original 4 bit
 * (16 color) table of terminals such as the classic VT100, or graphical modes
 * such as VGA.
 *
 * The colors are defined as a mix and match of the most visually pleasant
 * colors of common terminal emulators, such as the Ubuntu default terminal,
 * Konsolo, xterm, PuTTY, VSCode, Windows 10 and PowerShell.
 */
class TerminalColorTable
{
public:
    /**
     * @brief TerminalColorTable
     */
    TerminalColorTable();

    /**
     * @brief ~TerminalColorTable
     */
    virtual ~TerminalColorTable();

    /**
     * @brief black
     * Defined as in the default Ubuntu color table.
     * @return RGB Color code for black text output.
     */
    QColor black() const;

    /**
     * @brief red
     * Defined as in the Windows Console color table.
     * @return RGB Color code for red text output.
     */
    QColor red() const;

    /**
     * @brief green
     * Defined as in the mIRC color table.
     * @return RGB Color code for green text output.
     */
    QColor green() const;

    /**
     * @brief yellow
     * Defined as in the default Ubuntu color table.
     * @return RGB Color code for yellow text output.
     */
    QColor yellow() const;

    /**
     * @brief blue
     * Defined as in the Windows 10 color table.
     * @return RGB Color code for blue text output.
     */
    QColor blue() const;

    /**
     * @brief magenta
     * Defined as PowerShell6 color table (obs: used code for bright Magenta).
     * @return RGB Color code for magenta text output.
     */
    QColor magenta() const;

    /**
     * @brief cyan
     * Defined as in the VSCode color table.
     * @return RGB Color code for cyan text output.
     */
    QColor cyan() const;

    /**
     * @brief white
     * Defined as in the default Ubuntu color table.
     * @return RGB Color code for white text output.
     */
    QColor white() const;

    /**
     * @brief brightBlack Alias for gray().
     *
     * This methods returns the same color as gray. But as this was defined in
     * the original 16 color table for terminals, it is replicated here.
     *
     * @return RGB Color code for gray text output.
     */
    QColor brightBlack() const;

    /**
     * @brief gray
     * Defined as in the PuTTY color table.
     * @return RGB Color code for gray text output.
     */
    QColor gray() const;

    /**
     * @brief brightRed
     * Defined as full brightness red.
     * @return RGB Color code for bright red text output.
     */
    QColor brightRed() const;

    /**
     * @brief brightGreen
     * Defined as in the PowerShell6 color table, as full brightness green is
     * often hard to read.
     * @return RGB Color code for bright green text output.
     */
    QColor brightGreen() const;

    /**
     * @brief brightYellow
     * Defined as the sum of full brightness red and full brightness green.
     * @return RGB Color code for bright yellow text output.
     */
    QColor brightYellow() const;

    /**
     * @brief brightBlue
     * Defined as full brightness blue.
     * @return RGB Color code for bright blue text output.
     */
    QColor brightBlue() const;

    /**
     * @brief brightMagenta
     * Defined as the sum of full brightness red and full brightness blue.
     * @return RGB Color code for bright magenta text output.
     */
    QColor brightMagenta() const;

    /**
     * @brief brightCyan
     * Defined as the sum of full brightness green and full brightness blue.
     * @return RGB Color code for bright cyan text output.
     */
    QColor brightCyan() const;

    /**
     * @brief brightWhite
     * Defined as in the Windows 10, color table.
     * @return RGB Color code for bright white text output.
     */
    QColor brightWhite() const;

    /**
     * @brief lookup88ColorCode
     *
     * Implements a color lookup table for compatiblity with the 88 colors mode.
     * The table was created with information from
     * http://urwid.org/manual/displayattributes.html#id7.
     *
     * The 88 color table is composed of
     * * 16 base colors, that are identic to the ones implemented in the 256
     * color table;
     * * A 4 * 4 * 4 (64 colors) cube skewed towards the brighter RGB colors;
     * * A grayscale table with 16 steps from (8, 8, 8) to (238, 238, 238).
     *
     * The base colors are implemented as a lookup table in the form of a
     * switch statement. @see lookup16BaseColors().
     * The color cube is separeted in the 3 base components, and then a lookup
     * table is applied to convert the code to an equivalent RGB value in the
     * following form:
     *
     * Code -> RGB Value
     * 0    -> 0 (0x0)
     * 1    -> 136 (0x88)
     * 2    -> 204 (0xCC)
     * 3    -> 255 (0xFF)
     *
     * This was not implemented in an algebraic form because the components
     * are not linearly distributed and are heavily skewed towards brighter
     * (higher) values, and the table is small enough to provide acceptable
     * performance.
     *
     * The grayscale lookup table is calculted from the following mathematical
     * expression: 6 * code - 429.
     *
     * @param code Lookup table index.
     * @return RGB equivalent to the code in a 88 colors lookup table.
     */
    QColor lookup88ColorCode(int code);

    /**
     * @brief lookup256ColorCode
     * Implements a conversion between the 256 color table lookup code and
     * the 24 bit RGB system.
     *
     * The 256 values table is composed of 16 base colors (8 plain and 8 high
     * intensity colors), a color cube of 6 * 6 * 6 (216 colors) and a grayscale
     * pallete with 24 steps from (8, 8, 8) to (238, 238, 238).
     *
     * The 16 base colors are implemented as a lookup table, the color cube
     * is implemented with a combination of a test to ensure that the 0 code
     * returns the correct value and an algebraic equation. Finally the
     * grayscale conversion is calculated from the following mathematical
     * expression: 10 * code - 2312;
     *
     * Table created with information from
     * https://jonasjacek.github.io/colors/.
     *
     * @param code 256 color table index.
     * @return An RGB equivalent of the 256 color indicated by the code.
     */
    QColor lookup256ColorCode(int code);

private:
    /**
     * @brief lookup16BaseColors Implement a lookup table in the form of a
     * switch statement.
     * @param code Index for the lookup table.
     * @return RGB Color representation of the 16 base colors (8 plain and 8
     * high intensity) of a 256 colors lookup table.
     */
    QColor lookup16BaseColors(int code);
};

#endif // TERMINALCOLORTABLE_H
