#include <stdbool.h>

static int file_position = 0;
static const int file_length = 22;

// Helloworld ducky hex
const char script[] = {0x0b, 0x00, 0x08, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x12, 0x00, 0x2c, 0x00, 0x1a, 0x00, 0x12, 0x00, 0x15, 0x00, 0x0f, 0x00, 0x07, 0x00};

void file_open()
{
    file_position = 0;
}

bool file_eof()
{
    return file_position >= file_length;
}

char file_getc()
{
    if (file_position >= file_length) {
        return script[file_length - 1];
    }

    char ret = script[file_position];
    file_position += 1;

    return ret;
}

void file_close()
{
    file_position = 0;
}
