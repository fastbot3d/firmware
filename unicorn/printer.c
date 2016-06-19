/*
 * Unicorn 3D Printer Firmware
 * printer.c
*/
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
#include <poll.h>
#include <signal.h>
#include <fcntl.h>
#include <math.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <netinet/in.h>
#include <getopt.h>

#include "gcode.h"
#include "unicorn.h"

static int mode = FW_MODE_REMOTE;
static int debug_log = 0;
static char *file = NULL;
static FILE *fp = NULL;

static bool quit = false;
static bool stop = false;
static bool quit_blocking = false;

static void signal_handler(int signal)
{
    printf("Terminating on signal %d\n", signal);
    syslog(LOG_DEBUG, "Unicorn terminating on signal %d\n", signal);   
    quit = true;
    quit_blocking = false;
    unicorn_exit(quit_blocking);
	exit(0);
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

static pthread_t control_thread;

static void usage(void)
{
    printf("Usage: unicorn [option]\n\n"
            "Options: \n" 
            "-i | --input    gcode input file\n"
            "-t | --test     auto test mode\n"
            "-d | --debug    set debug log level\n"
            "-h | --help     Print this message\n"
           );
}

static void parse_args(int argc, char *argv[])
{
    int c;
    int index; 
    
    const char short_option[] = "i:td:h";
    const struct option long_option[] = {
        {"input",   required_argument, NULL, 'i'},
        {"test",    no_argument,       NULL, 't'},
        {"debug",   required_argument, NULL, 'd'},
        {"help",    no_argument,       NULL, 'h'},
        {0,0,0,0},
    };

    for (;;) {
        c = getopt_long(argc, argv, short_option, long_option, &index); 
        if (c == -1) {
            break; 
        }

        switch (c) {
            case 'i':
                mode = FW_MODE_LOCAL;
                printf("Running mode: local\n");
                file = optarg;
                break;

            case 't':
                printf("Running mode: testing\n");
                mode = FW_MODE_TEST;
                break;

            case 'd':
                debug_log = atoi(optarg);
                break;

            case 'h':
                usage();
                exit(1);

            default:
                usage();
                exit(1);
        }
    }
}

/* The different keys supported on the tty. */
typedef enum {
    KEY_NO     = 0,
    KEY_STOP   = 's',
    KEY_PAUSE  = ' ',
    KEY_QUIT   = 'q',
    KEY_UP     = 'k',
    KEY_DOWN   = 'j',
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
            && val != KEY_DOWN && val != KEY_STOP) {
        printf("key invalid\n");
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
        case KEY_QUIT:
            printf("Quit...\n");
            quit = true;
            quit_blocking = false;
            break;

        case KEY_STOP:
            printf("Stop...\n");
            stop = true;
            quit_blocking = false;
            unicorn_stop(quit_blocking);
            break; 

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

        case KEY_UP:
            multiply += 10;
            if (multiply > 2000) {
                multiply = 2000;
            }
            unicorn_set_feedrate(multiply);
            break;

        case KEY_DOWN:
            multiply -= 10;
            if (multiply < 10) {
                multiply = 10;
            }
            unicorn_set_feedrate(multiply);
            break;

        case KEY_NO:
            break;

        default:
            break;
        }
        
        if (key == KEY_QUIT) {
            goto cleanup;
        }

        /* Sleep for a while */
        usleep(100000);
    }

cleanup:
    tty_delete(htty); 

    printf("Leaving Control thread\n");
    pthread_exit(NULL);
}

int control_init(void)
{
    int ret = 0;
    printf("Create control thread\n");
    ret = sub_sys_thread_create("control", 
                                &control_thread, 
                                NULL, 
                                &control_thread_worker, 
                                NULL);
    if (ret) {
        printf("create control thread err\n");
        ret = -1;
    } 

    return ret;
}

void control_exit(void)
{
    quit = true;

    pthread_join(control_thread, NULL);
}

int read_fd = -1;
int write_fd = -1;
int read_emerg_fd = -1;
#define SERVER_RD_PORT 50002
#define SERVER_WR_PORT 50012
#define SERVER_WR_EMERG_PORT 50013
int server_start(void)
{
    int ret = 0;
    int opt = 1;
    int read_sock = -1; 
    int write_sock = -1;
    int read_emerg_sock = -1;
    struct sockaddr_in read_addr;
    struct sockaddr_in write_addr;
    struct sockaddr_in read_emerg_addr;
    socklen_t addr_len;

    if ((read_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1) {
        printf("create read socket error...\n");
        return false;
    }
    if ((write_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1) {
        printf("create write socket error...\n");
        return false;
    }
    if ((read_emerg_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1) {
        printf("create write socket error...\n");
        return false;
    }

    ret = setsockopt(read_sock, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt));
    if (ret == -1) {
        printf("set read socket error...\n");
        return false;
    }
    ret = setsockopt(write_sock, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt));
    if (ret == -1) {
        printf("set write socket error...\n");
        return false;
    }
    ret = setsockopt(read_emerg_sock, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt));
    if (ret == -1) {
        printf("set write emergency socket error...\n");
        return false;
    }

    bzero(&read_addr, sizeof(struct sockaddr_in));
    read_addr.sin_family = AF_INET;
    read_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    read_addr.sin_port = htons(SERVER_RD_PORT);
    if(bind(read_sock, (struct sockaddr *)&(read_addr), sizeof(struct sockaddr_in)) == -1){
        printf("bind read sock error...\n");
        return false;
    }

    bzero(&write_addr, sizeof(struct sockaddr_in));
    write_addr.sin_family = AF_INET;
    write_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    write_addr.sin_port = htons(SERVER_WR_PORT);
    if(bind(write_sock, (struct sockaddr *)&(write_addr), sizeof(struct sockaddr_in)) == -1) {
        printf("bind write sock error...\n");
        return false;
    }
    bzero(&read_emerg_addr, sizeof(struct sockaddr_in));
	read_emerg_addr.sin_family = AF_INET;
	read_emerg_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	read_emerg_addr.sin_port = htons(SERVER_WR_EMERG_PORT);
	if(bind(read_emerg_sock, (struct sockaddr *)&(read_emerg_addr), sizeof(struct sockaddr_in)) == -1) {
        printf("bind write sock error...\n");
        return false;
    }

    printf("start to listen... \n");
	listen(read_sock, 1);
	listen(write_sock, 1);
	listen(read_emerg_sock, 1);

	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(read_sock,  &fds);
	FD_SET(write_sock, &fds);
	FD_SET(read_emerg_sock, &fds);

	struct timeval timeout;
	timeout.tv_sec = 5;
	timeout.tv_usec = 0;

	int max_fd = write_sock > read_sock ? write_sock : read_sock;
	max_fd = max_fd > read_emerg_sock ? max_fd : read_emerg_sock;

    while (!quit) 
    {
		int rc = select(max_fd + 1, &fds, 0, 0, &timeout);
		if (rc < 0) {
			if(errno == EINTR){
				printf("select errno == EINTR\n");
				continue;
			}
			printf("select err, break ????\n");
			break;
		} else if (rc >0 && FD_ISSET(read_sock, &fds)) {
    		printf("accpet read sock\n");
			read_fd = accept(read_sock, (struct sockaddr *)&read_addr, &addr_len);
		} else if (rc >0 && FD_ISSET(write_sock, &fds)) {
    		printf("accpet write sock\n");
			write_fd = accept(write_sock, (struct sockaddr *)&write_addr, &addr_len);
		} else if (rc >0 && FD_ISSET(read_emerg_sock, &fds)) {
    		printf("accpet write emergency sock\n");
			read_emerg_fd = accept(read_emerg_sock, (struct sockaddr *)&read_emerg_addr, &addr_len);
		} else {
			FD_ZERO(&fds);
			FD_SET(read_sock,  &fds);
			FD_SET(write_sock, &fds);
			FD_SET(read_emerg_sock, &fds);

			timeout.tv_sec = 5;
			timeout.tv_usec = 0;
			continue;
		}
        
        if (read_fd > 0 && write_fd > 0 && read_emerg_fd >0) {
            unicorn_start(read_fd, write_fd, read_emerg_fd);
        }
	}

	if (read_sock > 0) {
    	close(read_sock);
    }

	if (write_sock > 0) {
    	close(write_sock);
    }

	if (read_emerg_sock > 0) {
    	close(read_emerg_sock);
    }

	if (read_fd > 0) {
    	close(read_fd);
    }
	if (write_fd > 0) {
    	close(write_fd);
    }
	if (read_emerg_fd > 0) {
    	close(read_emerg_fd);
    }

    printf("exit \n");
    return 0;
}

int main(int argc, char *argv[])
{
    printf("Unicron: 3D printer firmware\n");
    parse_args(argc, argv);
    
    /* Check mode validity */
    if (mode <= FW_MODE_MIN || mode >= FW_MODE_MAX) {
        printf("Not supported unicorn fw mode %d\n", mode);
        exit(1);
    } else {
        unicorn_set_mode(mode);
    }

    if (mode == FW_MODE_REMOTE) {
        printf("Running mode: testing\n");
    }

    if (mode == FW_MODE_LOCAL) {
        /* Check gcode file validy */
        if (access(file, R_OK) < 0) {
            printf("Gcode file is not able to read, please check %s!\n", 
                    file);
            exit(1);
        }
        fp = fopen(file, "r");
        if (!fp) {
            exit(1);
        }
    }

    sys_init();

	if (mode != FW_MODE_REMOTE) {
    	control_init();
	}

    unicorn_init();
    
    if (mode == FW_MODE_REMOTE) {
        server_start();
    } else if (mode == FW_MODE_TEST) {
        unicorn_test();
    } else if (mode == FW_MODE_LOCAL) {
        unicorn_start(0, 0, 0);

        char *ptr = NULL;
        while (!quit) {
            ptr = gcode_get_line_from_file(fp);
            if (!ptr) {
                quit = true;
                quit_blocking = true;
                break;
            } else {
                gcode_process_line_from_file();
            }
        }
    }
    
    printf("Quit from printer\n");
    quit = true;    
    
    if (!stop) { 
        unicorn_stop(quit_blocking);
    }

    unicorn_exit(quit_blocking);
    
    if (mode == FW_MODE_LOCAL) {
        if (fp) {
            fclose(fp);
        }
    }

    sys_exit();

    return 0;
}

