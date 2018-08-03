#ifndef ENVIRONMENTEXCEPTION_H
#define ENVIRONMENTEXCEPTION_H

#include <exception>
#include "string"

class CustomException : public std::exception {
    const char* file;
    int line;
    const char* func;
    const char* info;

    public:
        CustomException(const char* file_, int line_, const char* func_, const char* info_) : std::exception(),file (file_),line (line_),func (func_),info (info_)
        {
        }

        const char* get_file() const { return file; }
        int get_line() const { return line; }
        const char* get_func() const { return func; }
        const char* get_info() const { return info; }

};



#endif // ENVIRONMENTEXCEPTION_H
