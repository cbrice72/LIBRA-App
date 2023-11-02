#pragma once
#include <Windows.h>
#include <string>

class Serial  {

public:
    int open(const char *port);
    int write(BYTE data);
    int writestring(std::string str);
    
public:
    HANDLE mhandle;
};
