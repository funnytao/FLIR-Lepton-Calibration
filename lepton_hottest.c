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
#include <bcm2835.h>
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
double temp = 0;
float calCorrelation;
float calSlope;
float calOffset;
float calComp;
unsigned int maxTemp = 0;
unsigned int minTemp = 65535;

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

void getTemp() {
  reg = 7;
	bcm2835_i2c_begin();
	bcm2835_i2c_write (&reg, 1);
	bcm2835_i2c_read_register_rs(&reg,&buf[0],3);
	temp = (double) (((buf[1]) << 8) + buf[0]);
	temp = (temp * 0.02)-0.01;
	temp = temp - 273.15 + OFFSET;
}

void getAmbTemp() {
  reg = 6;
	bcm2835_i2c_begin();
	bcm2835_i2c_write (&reg, 1);
	bcm2835_i2c_read_register_rs(&reg,&buf[0],3);
	temp = (double) (((buf[1]) << 8) + buf[0]);
	temp = (temp * 0.02)-0.01;
	temp = temp - 273.15 + OFFSET;
}

int linreg(int n, const unsigned int x[], const double y[], float* m, float* b, float* r)
{
	double   sumx = 0.0;
	double   sumx2 = 0.0;
	double   sumxy = 0.0;
	double   sumy = 0.0;
	double   sumy2 = 0.0;
  int i = 0;
	for (i = 0; i < n; i++)
	{
		sumx += x[i];
		sumx2 += pow(x[i], 2);
		sumxy += x[i] * y[i];
		sumy += y[i];
		sumy2 += pow(y[i], 2);
	}
	double denom = (n * sumx2 - pow(sumx, 2));
	if (denom == 0) {
		//singular matrix. can't solve the problem.
		*m = 0;
		*b = 0;
		*r = 0;
		return 1;
	}
	*m = (n * sumxy - sumx * sumy) / denom;
	*b = (sumy * sumx2 - sumx * sumxy) / denom;
	if (r != NULL) {
		*r = (sumxy - sumx * sumy / n) /
			sqrt((sumx2 - pow(sumx, 2) / n) *
			(sumy2 - pow(sumy, 2) / n));
	}
	return 0;
}

unsigned int calAvg() {
  getThermalData();
  unsigned int tmpCnt = 0;
  int row, col;
  for (row = 30-MATRIX; row < 30+MATRIX; row++) {
    for (col = 40-MATRIX; col <= 40+MATRIX; col++) {
      tmpCnt += lepton_image[row][col];
    }
  }
  tmpCnt /= pow(2*MATRIX, 2);
  return tmpCnt;
}

void calibration() {
  unsigned int raw_data[SAMPLE];
  double temp_data[SAMPLE];
  calCorrelation = 0;
  unsigned int avg = 0, old_avg = 0;
  maxTemp = 0;
  minTemp = 65535;

  while (calCorrelation < 0.5) {
    int counter = 0;
    while (counter < SAMPLE) {
      while (abs(avg - old_avg) < 5 || avg == 0) {
        avg = calAvg();
        sleep(0.2);
      }
      printf("%d\n", counter);
      getTemp();
      old_avg = avg;
      maxTemp = maxTemp < avg ? avg : maxTemp;
      minTemp = minTemp > avg ? avg : minTemp;
      raw_data[counter] = avg;
      temp_data[counter] = temp;
      counter++;
      // sleep(0.25);
    }
    linreg(SAMPLE, raw_data, temp_data, &calSlope, &calOffset, &calCorrelation);
    calComp = 0;
  }
}

double rawToTemp(unsigned int rawValue) {
	return (calSlope * rawValue) + calOffset;
}

void outputHottest() {
  unsigned hottest = 0;
  unsigned centerTemp = 0;
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
  float diff = temp - rawToTemp(centerTemp);
  if (abs(diff) > 0.5) {
    calOffset += diff;
  }
  printf("Hottest Point: %f %f %f\n", rawToTemp(hottest), rawToTemp(centerTemp), temp);
  // if (abs(rawToTemp(centerTemp) - temp) > 2) {
  //   calibration();
  // }
}

int main(int argc, char *argv[])
{
	bcm2835_init();
	bcm2835_i2c_begin();
	bcm2835_i2c_set_baudrate(25000);
	// set address
	bcm2835_i2c_setSlaveAddress(0x5a);
	printf("\nOk, your device is working!!\n");
  printf("Calibrating...\n");
  calibration();
  printf("Calibrated!\n");
  int cnt = 0;
  while (1) {
    getTemp();
    getThermalData();
    printf("%d ", cnt);
    outputHottest();
    cnt++;
    if (cnt == 50) {
      cnt = 0;
      calibration();
    }
    sleep(1);
  }
  // while (1) {
  //   getThermalData();
  //   getTemp();
  //   unsigned int tmpCnt = 0;
  //   int row, col;
  //   for (row = 25; row <= 34; row++) {
  //     for (col = 35; col <= 44; col++) {
  //       tmpCnt += lepton_image[row][col];
  //     }
  //   }
  //   tmpCnt /= 100.0;
  //   printf("%f %f\n", rawToTemp(tmpCnt), temp);
  //   sleep(0.5);
  // }

	return 0;
}
