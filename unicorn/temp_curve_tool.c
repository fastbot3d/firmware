
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <stdio.h>
#include <errno.h>

//#include "parameter.h"
//#include "eeprom.h"

#include "temp_curve_100K.h"


static void make_bin_file(char *fileName, void *data, int len);
static void make_temp_curve_bin();
static char file_dir[256] = {0};

int main(int argc, char **argv)
{
	if(argc < 2){
		printf("usage: %s dir_path \n", argv[0]);
		return 0;
	}

	strcpy(file_dir, argv[1]);
	printf("store dir path:%s \n", file_dir);

	make_temp_curve_bin();
	return 0;
}


static void make_bin_file(char *fileName, void *data, int len)
{
	int ret = -1;
	int fd = -1; 
	char tmp_file_dir[256] = {0};
	
	strcpy(tmp_file_dir, file_dir);
	strcat(tmp_file_dir, "/");
	strcat(tmp_file_dir, fileName);
	//fd = open(fileName, O_WRONLY | O_CREAT);
	fd = open(tmp_file_dir, O_WRONLY | O_CREAT);
	if(fd < 0) {
		printf("error, failed to create file:%s\n", tmp_file_dir);
		perror("failed\n");
		return;
	}

	ret = write(fd, data, len);
	if(ret < 0) {
		printf("error, failed to write file:%s\n", tmp_file_dir);
		return;
	}

	if(fd >0)
		close(fd);
}

static void make_temp_curve_bin()
{
	printf("create file:temp_curve_100K.bin\n");
	make_bin_file("temp_curve_100K.bin", curve_100K_data, sizeof(curve_100K_data));
	//make_bin_file("temp_curve_200K.bin", curve_200K_data, sizeof(curve_200K_data));
}
