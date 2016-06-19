#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/wait.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>

#include <sys/time.h>
#include <time.h>

#include "common.h" 
#include "eeprom.h"
#include "servo.h"
#include "stepper.h"

int unicorn_test_usb_storage()
{
	int count = 5;
	struct stat st;
	int fd=0;

	#define U_STORAGE_DEV	"/dev/sda"
	while(count){
		if (stat(U_STORAGE_DEV, &st) == 0) {
			fd = open(U_STORAGE_DEV, O_RDONLY);	
			if(fd < 0 ){
				return -1;
			}

			if(fd > 0 ){
				close(fd);
			}
			break;
		}
    	printf("info: please insert U mass storage\n");

		sleep(2);
		count--;
	}
	if(count == 0){
		return -1;
	} else {
		return 0;
	}
}

int unicorn_test_emmc()
{
	struct stat st;
	int fd;

	#define MMC_DEV	"/dev/mmcblk1"
	if (stat(MMC_DEV, &st) == -1) {
		return -1;
	}

	fd = open(MMC_DEV, O_RDONLY);	
	if(fd < 0 ){
		return -1;
	}

	if(fd > 0 ){
		close(fd);
	}

	return 0;
}

int unicorn_test_eeprom()
{
	unsigned char data_old[512] = {0}, data_new[512] = {0x12};
	unsigned long offset = 0; 

	srand(time(NULL));
	offset = 100 +random()%(1024*8 - 512 -100+1);// n+rand()%(m-n+1); 
	printf("write eeprom 512 byte at offset:%ld \n", offset);
	if(eeprom_read_block(EEPROM_DEV, data_old, sizeof(data_old), offset) < 0){
		printf("EEPROM is BAD 0 !!!!!!!!!!!!!!!!!!!!!!\n");
		return -1;
	} 

	memset(data_new, 0x12, sizeof(data_new));
	if(eeprom_write_block(EEPROM_DEV, data_new, sizeof(data_new), offset) <0) {
		printf("EEPROM is BAD 1 !!!!!!!!!!!!!!!!!!!!!!\n");
		return -1;
	} 

	memset(data_new, 0, sizeof(data_new));
	if(eeprom_read_block(EEPROM_DEV, data_new, sizeof(data_new), offset) <0) {
		printf("EEPROM is BAD 2 !!!!!!!!!!!!!!!!!!!!!!\n");
		return -1;
	}

	int i=0;
	for(i=0; i<sizeof(data_new); i++){
		if(data_new[i] != 0x12){
			break;
		}
	}
	printf("EEPROM is compare i=%d, sizeof(data_new):%d\n", i, sizeof(data_new));
	if(i != sizeof(data_new)){
		printf("EEPROM is BAD 3 !!!!!!!!!!!!!!!!!!!!!!\n");
		return -1;
	}
	if(eeprom_write_block(EEPROM_DEV, data_old, sizeof(data_old), offset) <0) {
		printf("EEPROM is BAD 4 !!!!!!!!!!!!!!!!!!!!!!\n");
		return -1;
	}

	return 0;
}

int unicorn_test_rtc()
{
    struct tm tm; 
    struct timeval tv; 
    time_t timep;
    
    tm.tm_sec = 10; 
    tm.tm_min = 10; 
    tm.tm_hour = 10; 
    tm.tm_mday = 10; 
    tm.tm_mon = 9;
    tm.tm_year = 2019 - 1900;

    timep = mktime(&tm);
    tv.tv_sec = timep;
    tv.tv_usec = 0;
    if(settimeofday(&tv, (struct timezone *) 0) < 0){ 
        printf("Set system datatime error!\n");
        return -1; 
    }   

    system("hwclock  -w");//write sys clock to rtc

    time_t now;
    struct tm *tm_now;
    time(&now);
    tm_now = gmtime(&now);
 
    printf("now datetime: %d-%d-%d %d:%d:%d\n", 
            tm_now->tm_year, 
            tm_now->tm_mon, 
            tm_now->tm_mday, 
            tm_now->tm_hour, 
            tm_now->tm_min, 
            tm_now->tm_sec);

    if(tm.tm_year != tm_now->tm_year){
        printf("rtc is error !!!!");
		return -1;
    }
	return 0;
}

int unicorn_test_network()
{
	system("ifconfig eth0 192.168.1.111");
	system("ping -c 3 192.168.1.1 | grep errors  >/tmp/ping.t");
	system("ping -c 3 192.168.1.1 | grep '0 received'>/tmp/ping2.t");

	struct stat st;
	if ((stat("/tmp/ping.t", &st) == 0 && st.st_size == 0 ) && 
		(stat("/tmp/ping2.t", &st) == 0 && st.st_size == 0 )) {
			return 0;
	} else 
			return -1;
}

int unicorn_test_max6675()
{
#define EXT3_THERMOCOUPLE_PATH 	"/sys/class/hwmon/hwmon0/device/vout" 
	int i = 0;
	float vout_therm = 0; 
	unsigned char buf[200] = {0};

	int fd_therm = open(EXT3_THERMOCOUPLE_PATH, O_RDONLY);
	if (fd_therm>0) {
		for(i=0; i<5 && stepper_check_lmsw(Y_AXIS) == 0; i++){
			memset(buf, 0, sizeof(buf));
			read(fd_therm, buf, sizeof(buf));
			vout_therm = atoi((const char *)buf)/4.0f;
			printf("read temperature:%f\n", vout_therm);
			sleep(1);
		}
		close(fd_therm);
	} else {
		printf("err to open %s\n", EXT3_THERMOCOUPLE_PATH);
		return -1;
	}

	return 0;
}

int unicorn_test_servo(void)
{
    int i;
    int idx = 0;
    channel_tag servo;
    servo = servo_lookup_by_index(idx);
    servo_enable(servo);

    for (i = 0; i < 1000; i++) {

        servo_set_angle(servo, 89);
        sleep(1); 

        servo_set_angle(servo, 1);
        sleep(1); 
    }

    servo_disable(servo);
    return 0;
}
