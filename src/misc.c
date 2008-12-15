/*
    misc.c -- small routines that didn't have any other home.

    Currently, only some debug messaging functions.
        
*/

#include "hw6180.h"
#include <stdio.h>
#include <stdarg.h>

extern DEVICE cpu_dev;
extern FILE *sim_deb, *sim_log;

static void msg(enum log_level level, const char *who, const char* format, va_list ap);
uint last_IC;
uint last_IC_seg;

void log_msg(enum log_level level, const char* who, const char* format, ...)
{
    if (level == DEBUG_MSG) {
        if (opt_debug == 0)
            return;
        if (cpu_dev.dctrl == 0 && opt_debug < 1)        // todo: should CPU control all debug settings?
            return;
    }

    // Make sure all messages have a prior display of the IC
    if (PPR.IC != last_IC || PPR.PSR != last_IC_seg) {
        last_IC = PPR.IC;
        last_IC_seg = PPR.PSR;
        char *tag = "Debug";
        // char *who = "IC";
            // out_msg("\n%s: %*s %s  %*sIC: %o\n", tag, 7-strlen(tag), "", who, 18-strlen(who), "", PPR.IC);
        char icbuf[80];
        addr_modes_t addr_mode = get_addr_mode();
        if (addr_mode == ABSOLUTE_mode)
            sprintf(icbuf, "%o", PPR.IC);
        else if (addr_mode == BAR_mode)
            sprintf(icbuf, "BAR %o", PPR.IC);
        else
            sprintf(icbuf, "%o|%o", PPR.PSR, PPR.IC);
        // out_msg("\n");
        // out_msg("%s: %*s IC: %s\n", tag, 7-strlen(tag), "", icbuf);
        msg(DEBUG_MSG, NULL, "\n", NULL);
        char buf[80];
        sprintf(buf, "%s: %*s IC: %s\n", tag, 7-strlen(tag), "", icbuf);
        msg(DEBUG_MSG, NULL, buf, NULL);
    }

    va_list ap;
    va_start(ap, format);
#if 0
    char *tag = (level == DEBUG_MSG) ? "Debug" :
        (level == WARN_MSG) ? "WARNING" :
        (level == NOTIFY_MSG) ? "Note" :
        (level == ERR_MSG) ? "ERROR" :
            "???MESSAGE";
    msg(tag, who, format, ap);
#else
    msg(level, who, format, ap);
#endif
    va_end(ap);
}

#if 0
void debug_msg(const char* who, const char* format, ...)
{
    if (opt_debug == 0)
        return;
    if (cpu_dev.dctrl == 0 && opt_debug < 1)        // todo: should CPU control all debug settings?
        return;
    va_list ap;
    va_start(ap, format);
    msg("Debug", who, format, ap);
    va_end(ap);
}

void warn_msg(const char* who, const char* format, ...)
{
    va_list ap;
    va_start(ap, format);
    msg("WARNING", who, format, ap);
    va_end(ap);
}

void complain_msg(const char* who, const char* format, ...)
{
    va_list ap;
    va_start(ap, format);
    msg("ERROR", who, format, ap);
    va_end(ap);
}
#endif


static void crnl_out(FILE *stream, const char *format, va_list ap)
{
    // SIMH does something odd with the terminal, so output CRNL
    int len =strlen(format);
    int nl = *(format + len - 1) == '\n';
    if (nl) {
        char *f = malloc(len + 2);
        if (f) {
            strcpy(f, format);
            *(f + len - 1) = '\r';
            *(f + len) = '\n';
            *(f + len + 1) = 0;
            vfprintf(stream, f, ap);
            free(f);
        } else {
            vprintf(format, ap);
            if (*(format + strlen(format) - 1) == '\n')
                fprintf(stream, "\r");
        }
    } else {
        vfprintf(stream, format, ap);
        if (*(format + strlen(format) - 1) == '\n')
            fprintf(stream, "\r");
    }
}


void out_msg(const char* format, ...)
{
    va_list ap;
    va_start(ap, format);

    FILE *stream = (sim_log != NULL) ? sim_log : stdout;
    crnl_out(stream, format, ap);
    fflush(stream);
}

#if 0
static void sim_hmsg(const char* tag, const char *who, const char* format, va_list ap)
{
    // This version uses SIMH facilities -- not tested
    char buf[2000];
    
    snprintf(buf, sizeof(buf), "%s: %*s %s: %*s", tag, 7-strlen(tag), "", who, 18-strlen(who), "");
    int l = strlen(buf);
    vsnprintf(buf + l, sizeof(buf) - l, format, ap);
    // TODO: setup every device with sim_debtab entries to reflect different debug levels
    sim_debug(~0, &cpu_dev, "%s", buf);
}
#endif

static void msg(enum log_level level, const char *who, const char* format, va_list ap)
{
    // This version does not use SIMH facilities -- except for the sim_deb and sim_log streams

    enum { con, dbg };
    FILE *streams[2];

    streams[con] = (sim_log != NULL) ? sim_log : stdout;
    streams[dbg] = sim_deb;
    if (level == DEBUG_MSG) {
        // Debug messags go to a debug log if one exists, otherwise to
        //  the console
        if (streams[dbg] != NULL)
            streams[con] = NULL;
    } else {
        // Non debug msgs always go to the console.  If a seperate debug
        // log exists, it also gets non-debug msgs.
        streams[dbg] = (sim_log == sim_deb) ? NULL : sim_deb;
    }

    char *tag = (level == DEBUG_MSG) ? "Debug" :
        (level == WARN_MSG) ? "WARNING" :
        (level == NOTIFY_MSG) ? "Note" :
        (level == ERR_MSG) ? "ERROR" :
            "???MESSAGE";

    for (int s = 0; s <= dbg; ++s) {
        FILE *stream = streams[s];
        if (stream == NULL)
            continue;
        if (who != NULL) {
            fprintf(stream, "%s: %*s %s: %*s", tag, 7-strlen(tag), "", who, 18-strlen(who), "");
            fflush(stream);
        }
    
        crnl_out(stream, format, ap);
        fflush(stream);
    }
}
