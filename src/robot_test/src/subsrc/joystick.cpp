#include "linux/joystick.h"
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

char buf[255];
int main ()
{
    int fd = open("/dev/input/js0",0);
    while(1)
    {
    read(fd, buf, sizeof(buf));
    std::cout<<buf<<std::endl;
    }
}