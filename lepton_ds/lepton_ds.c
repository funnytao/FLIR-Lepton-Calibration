/*
Copyright (c) 2014, Pure Engineering LLC
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdint.h>
#include <stdbool.h>
#include <dirent.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <limits.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <math.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *device = "/dev/spidev0.1";
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 16000000;
static uint16_t delay;

#define MATRIX (4)
#define OFFSET (0)
#define SAMPLE (100)
#define VOSPI_FRAME_SIZE (164)
uint8_t lepton_frame_packet[VOSPI_FRAME_SIZE];
static unsigned int lepton_image[80][80];
char buf[6];
char reg;

double ambTemp = 0;
DIR *dir;
struct dirent *dirent;
char dev[16];      // Dev ID
char devPath[128]; // Path to device
char buff[256];     // Data from device
char tmpData[6];   // Temp C * 1000 reported by device
char path[] = "/sys/bus/w1/devices";
ssize_t numRead;

int transfer(int fd)
{
	int ret;
	int i;
	int frame_number;
	uint8_t tx[VOSPI_FRAME_SIZE] = {0, };
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)lepton_frame_packet,
		.len = VOSPI_FRAME_SIZE,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");

	if(((lepton_frame_packet[0]&0xf) != 0x0f))
	{
		frame_number = lepton_frame_packet[1];

		if(frame_number < 60 )
		{
			for(i=0;i<80;i++)
			{
				lepton_image[frame_number][i] = (lepton_frame_packet[2*i+4] << 8 | lepton_frame_packet[2*i+5]);
			}
		}
	}
	return frame_number;
}

void getThermalData() {
  int ret = 0;
	int fd;

  fd = open(device, O_RDWR);
	if (fd < 0)
	{
		pabort("can't open device");
	}

	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
	{
		pabort("can't set spi mode");
	}

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
	{
		pabort("can't get spi mode");
	}

	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
	{
		pabort("can't set bits per word");
	}

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
	{
		pabort("can't get bits per word");
	}

	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
	{
		pabort("can't set max speed hz");
	}

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
	{
		pabort("can't get max speed hz");
	}

	// printf("spi mode: %d\n", mode);
	// printf("bits per word: %d\n", bits);
	// printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	while(transfer(fd)!=59){}

	close(fd);
}

void getAmbTemp() {
	int fd = open(devPath, O_RDONLY);
  if(fd == -1) {
  	perror ("Couldn't open the w1 device.");
  	return;
  }
  if ((numRead = read(fd, buff, 256)) > 0) {
	   strncpy(tmpData, strstr(buff, "t=") + 2, 5);
	   float tempC = strtof(tmpData, NULL);
	   tempC /= 1000;
		 ambTemp = (double)tempC;
  }
  //printf("Temp: %f\n", ambTemp);
  close(fd);
}

double rawToTemp(unsigned int rawValue) {
	//calOffset = ambTemp - (calSlope * 8192);
	//return (calSlope * rawValue) + calOffset 0.0217;
	return (0.026 * ((double)rawValue - 8192) + ambTemp);
}

void outputHottest() {
  unsigned int hottest = 0;
  unsigned int centerTemp = 0;
  int row = 0, col = 0;
  for (row = 0; row < 60; row++) {
    for (col = 0; col < 80; col++) {
      if (row >= 30-MATRIX && row < 30+MATRIX && col >= 40-MATRIX && col < 40+MATRIX) {
        centerTemp += lepton_image[row][col];
      }
      hottest = hottest < lepton_image[row][col] ? lepton_image[row][col] : hottest;
    }
  }
  centerTemp /= pow(2*MATRIX, 2);
  //printf("%d	%f\n", centerTemp - 8192, temp);
  // printf("Hottest Point: %f %f %f %f %d\n", rawToTemp(hottest), (temp-ambTemp)/((double)centerTemp-8192), temp, ambTemp, centerTemp - 8192);
	printf("Hottest Point: %f %f %d\n", rawToTemp(hottest), ambTemp, centerTemp - 8192);
}

int main(int argc, char *argv[])
{

  dir = opendir (path);
  if (dir != NULL)
  {
   while ((dirent = readdir (dir)))
    // 1-wire devices are links beginning with 28-
    if (dirent->d_type == DT_LNK &&
      strstr(dirent->d_name, "28-") != NULL) {
     strcpy(dev, dirent->d_name);
     printf("\nDevice: %s\n", dev);
    }
         (void) closedir (dir);
         }
  else
  {
   perror ("Couldn't open the w1 devices directory");
   return 1;
  }
	sprintf(devPath, "%s/%s/w1_slave", path, dev);

	printf("\nOk, your device is working!!\n");

	int cnt = 10;
  while (1) {
		sleep(0.1);
		if (cnt == 10) {
			getAmbTemp();
		}
		cnt--;
		if (cnt == 0) {
			cnt = 10;
		}
    getThermalData();
    outputHottest();
    sleep(1);
  }
	return 0;
}
