#include "tools.h"
#include "qdebug.h"

Tools::Tools()
{

}

QString Tools::getEnv(char* name)
{
    const char *ret = getenv(name);
    if (ret) return QString(ret);
    else
    {
        std::string info = "Não há variável de ambiente.";
        throw CustomException("tool.cpp",4,"Caminho de diretório","Não há variável de ambiente.");
    }
}
