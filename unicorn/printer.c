/*
 * Unicorn 3D Printer Firmware
 * printer.c
*/
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
#include <poll.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>

#include "led.h"
#include "fan.h"
#include "stepper.h"
#include "heater.h"
#include "unicorn.h"
#include "lmsw.h"
#include "gcode.h"

#include "printer.h"

static bool quit = false;
static int quit_blocking = 1;

static pthread_t control_thread;

static void signal_handler(int signal)
{
    fprintf(stderr, "Terminating on signal %d\n", signal);
    quit = true;
    quit_blocking = 0;
    unicorn_stop();
}

int sys_init(void)
{
    signal(SIGINT,  signal_handler);
    signal(SIGHUP,  signal_handler);
    signal(SIGTERM, signal_handler);

    return 0;
}

void sys_exit(void)
{
    signal(SIGINT,  SIG_DFL);
    signal(SIGHUP,  SIG_DFL);
    signal(SIGTERM, SIG_DFL);
}

static int usage(const char *prog, const char *msg)
{
    if (msg) {
        fprintf(stderr, "%s\n\n", msg);
    }
    
    fprintf(stderr, "Usage: %s [options] [<gcode-filename>]\n", prog);

    return 0;
}

/* The different keys supported on the tty. */
typedef enum {
    KEY_NO    = 0,
    KEY_PAUSE = ' ',
    KEY_QUIT  = 'q',
    KEY_UP    = 'k',
    KEY_DOWN  = 'j',
} tty_key;

/**
 * @brief Handle through which to reference an tty Object.
 */
typedef struct tty_object *tty_handle;

typedef struct tty_attrs_t {
    /* Should the tty_get_key calls block? */
    int    blocking;
} tty_attrs;

#define TTY_DEVICE "/dev/tty"

typedef struct tty_object {
    int             fd;
    struct termios  save_tty;
    int            blocking;     
} tty_object;

const tty_attrs TTY_ATTRS_DEFAULT = {
    0 
};

tty_handle tty_create(tty_attrs *attrs)
{
    tty_handle htty;
    struct termios tty;
    int flags;

    htty = calloc(1, sizeof(tty_object));
    if (htty == NULL) {
        printf("Failed to allocate space for tty Object\n");
        return NULL;
    }

    htty->blocking = attrs->blocking;

    flags = htty->blocking ? O_RDONLY : O_RDONLY | O_NONBLOCK;

    htty->fd = open(TTY_DEVICE, flags);
    if (htty->fd == -1) {
        printf("Failed to open tty file handle\n");
        free(htty);
        return NULL;
    }

    /* Save current terminal */
    if (tcgetattr(htty->fd, &htty->save_tty) == -1) {
        printf("Fail to get curret tty attr\n");
        return NULL;
    }
    tty = htty->save_tty;

    /* Trun of echo and canonical mode */
    tty.c_lflag &= ~(ECHO | ICANON);
    if (!htty->blocking) {
        /* set read to return immediately */
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(htty->fd, TCSAFLUSH, &tty) == -1) {
            printf("Failed to set tty attr\n");
            return NULL;
        }
    } else {
        tty.c_cc[VMIN] = 1;

        if (tcsetattr(htty->fd, TCSANOW, &tty) == -1) {
            printf("Failed to set tty attr\n");
            return NULL;
        }
    }

    return htty;
}

static int validate_key(unsigned char val)
{
    if (val != KEY_PAUSE && val != KEY_QUIT && val != KEY_UP
            && val != KEY_DOWN) {
        return -1;
    }
    return 0;
}

int tty_get_key(tty_handle htty, tty_key *key)
{
    unsigned char  val;
    int  bytes;

    bytes = read(htty->fd, &val, 1);
    if (bytes < 0) { 
        printf("Read key err\n");
        return -1;
    }
        
    if (bytes && validate_key(val) >= 0) {
        *key = val; 
    } else {
        *key = KEY_NO;
    }
    
    return 0;
}

int tty_delete(tty_handle htty)
{
    if (htty) {
        if (tcsetattr(htty->fd, TCSADRAIN, &htty->save_tty) == -1) {
            printf("Restore tty setting err\n"); 
        }

        if (htty->fd >= 0) {
            close(htty->fd);
        }

        free(htty);
    }

    return 0;
}

tty_handle  htty   = NULL;
void *control_thread_worker(void *arg)
{
    int multiply = 100;
    bool pause = false;

    tty_attrs   attrs = TTY_ATTRS_DEFAULT;
    tty_key     key    = KEY_NO;

    attrs.blocking = 1;

    htty = tty_create(&attrs);
    if (htty == NULL) {
        printf("Couldn't create keypad object\n");
        goto cleanup;
    }

    while (!quit)
    {
        if (tty_get_key(htty, &key) < 0) {
            printf("Failed to get key from keypad\n");
            goto cleanup;
        }

        switch (key) 
        {
        case KEY_PAUSE:
            if (!pause) {
                printf("Pause...\n");
                unicorn_pause();
                pause = true;
            } else {
                printf("Resume...\n");
                unicorn_resume();
                pause = false;
            }
            break;

        case KEY_QUIT:
            printf("Quit...\n");
            quit = true;
            quit_blocking = 0;
            unicorn_stop();
            break;
        
        case KEY_UP:
            multiply += 10;
            gcode_set_feed(multiply);
            break;

        case KEY_DOWN:
            multiply -= 10;
            gcode_set_feed(multiply);
            break;

        case KEY_NO:
            break;

        default:
            break;
        }
        
        if (key == KEY_QUIT) {
            break;
        }

        /* Sleep for a while */
        usleep(100000);
    }

cleanup:
    printf("Leaving Control thread\n");
    tty_delete(htty); 

    pthread_exit(NULL);
}

int main(int argc, char *argv[])
{
    int ret;
    char *ptr = NULL;

    fprintf(stderr, "Unicron 3D printer firmware\n");
    
    if (optind >= argc) {
        usage(argv[0], "No <gcode-filename>!!");
        return -1;
    } 

    FILE *fp = fopen(argv[1], "r");
    if (!fp) { 
        return -1;
    }

    sys_init();
    unicorn_init();

    ret = sub_sys_thread_create("control", 
                                &control_thread, 
                                NULL, 
                                &control_thread_worker, 
                                NULL);
    if (ret) {
        return -1;
    } 

    unicorn_start();

    while (!quit) 
    {
        ptr = gcode_get_line_from_file(fp);
        if (!ptr) {
            quit = true;
            pthread_cancel(control_thread);
            quit_blocking = 1;
            unicorn_stop();
            break;
        }

        gcode_process_line();
    }
    
    pthread_join(control_thread, NULL);
    
    unicorn_exit(quit_blocking);
    sys_exit();

    return 0;
}

