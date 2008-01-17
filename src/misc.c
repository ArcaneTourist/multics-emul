#include "hw6180.h"
#include <stdio.h>
#include <stdarg.h>

static void msg(const char* tag, const char *who, const char* format, va_list ap);

void debug_msg(const char* who, const char* format, ...)
{
    va_list ap;
    va_start(ap, format);
    msg("DEBUG", who, format, ap);
    va_end(ap);
}

void complain_msg(const char* who, const char* format, ...)
{
    va_list ap;
    va_start(ap, format);
    msg("ERR", who, format, ap);
    va_end(ap);
}

static void msg(const char* tag, const char *who, const char* format, va_list ap)
{
    fflush(stdout);
    printf("%s:%*s %s: ", who, 15-strlen(who), "", tag);
    vprintf(format, ap);
    if (*(format + strlen(format) - 1) == '\n') {
        printf("\r");
    }
    fflush(stdout);
}
