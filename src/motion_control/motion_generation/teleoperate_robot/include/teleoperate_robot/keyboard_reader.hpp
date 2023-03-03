#pragma once

#include <memory.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>



/* ========================================================================== */
/*                               KEYBOARDREADER                               */
/* ========================================================================== */

class KeyboardReader {
public:
    KeyboardReader()
    : kfd(0)
    {
        // Get the console in raw mode
        tcgetattr(kfd, &cooked);
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &=~ (ICANON | ECHO);
        // Setting a new line, then end of file
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);
    }
    
    void read_one(char * c)
    {
        int rc = read(kfd, c, 1);
        if (rc < 0) {
            throw std::runtime_error("read failed");
        }
    }
    
    void shutdown()
    {
        tcsetattr(kfd, TCSANOW, &cooked);
    }

private:
    int kfd;
    struct termios cooked, raw;
};