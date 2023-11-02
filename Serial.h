#pragma once

#include <string>
#include <Windows.h>

class Serial {
  public:
    int open(const char* port);
    int write(BYTE data);
    int writestring(std::string str);

  public:
    HANDLE mhandle;
};
