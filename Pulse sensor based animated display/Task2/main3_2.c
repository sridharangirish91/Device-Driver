#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>
#include <time.h>

#define SPI_PATTERN		64
#define SPEED_DISP		136
#define STOP_DISP		247

int fdd, dir;
long spd = 1;
long order[10];
long long distance_prev, distance_pres;
int breakflag;
pthread_mutex_t mut;	

// user thread

void *sensor_read()
{
	int fd, res, retioctl, cmd;
	int k = 0;
	long long timebuf = 0, tmp;
	int dirprev;
	fd = open("/dev/pulse", O_RDWR);
	if (fd < 0 )
	{
 		printf("\nCan not open device file.\n");		
  	}

	while(1)
	{

		pthread_mutex_lock(&mut);
	 	res = write(fd, &res, 1); 
	 	do
	 	{
	 		res = read(fd, &timebuf, sizeof(timebuf));

	 	}while(res == -1); // getting pulse width from driver
	 	
	 	distance_prev = distance_pres;
		tmp = timebuf * 34;
		distance_pres = tmp/200000; // distance in cm
		

		cmd = distance_pres/10;

		spd = ((distance_pres * 10) + 1);

		dirprev = dir;
		

		if(distance_pres - distance_prev > 3)
		{
			dir = 1;
		}
		else if(distance_prev - distance_pres > 3)
		{
			dir = 0;
		}

		switch(cmd) // pattern according to the distance
		{
			case 1:
			if(dir == 0)
			{
				order[0] = 0;
				order[1] = spd;
				order[2] = 1;
				order[3] = spd;
				order[4] = order[5] = 0;
			}

			else if(dir == 1)
			{
				order[0] = 2;
				order[1] = spd;
				order[2] = 3;
				order[3] = spd;
				order[4] = order[5] = 0;
			}
			break;
			case 2:
			if(dir == 0)
			{
				order[0] = 4;
				order[1] = spd;
				order[2] = 5;
				order[3] = spd;
				order[4] = order[5] = 0;
			}

			else if(dir == 1)
			{
				order[0] = 6;
				order[1] = spd;
				order[2] = 7;
				order[3] = spd;
				order[4] = order[5] = 0;
			}
			break;
			case 3:
			if(dir == 0)
			{
				order[0] = 8;
				order[1] = spd;
				order[2] = 9;
				order[3] = spd;
				order[4] = order[5] = 0;
			}

			else if(dir == 1)
			{
				order[0] = 10;
				order[1] = spd;
				order[2] = 11;
				order[3] = spd;
				order[4] = order[5] = 0;
			}
			break;
			case 4:
			if(dir == 0)
			{
				order[0] = 12;
				order[1] = spd;
				order[2] = 13;
				order[3] = spd;
				order[4] = order[5] = 0;
			}

			else if(dir == 1)
			{
				order[0] = 14;
				order[1] = spd;
				order[2] = 15;
				order[3] = spd;
				order[4] = order[5] = 0;
			}
			break;
			default:
			if(dir == 0)
			{
				order[0] = 16;
				order[1] = spd;
				order[2] = 17;
				order[3] = spd;
				order[4] = order[5] = 0;
			}

			else if(dir == 1)
			{
				order[0] = 16;
				order[1] = spd;
				order[2] = 17;
				order[3] = spd;
				order[4] = order[5] = 0;
			}
			break;
		}


		if(breakflag == 0)
			res = write(fdd, (char *) order, 1, NULL); // writing the pattern to display to the led driver
		else 
			break;
		usleep(3000);
		pthread_mutex_unlock(&mut);
		
		
	}
	
	close(fd);
	pthread_exit("sensor read thread");

}

void main()
{
	int j, i = 0, retioctl = 0, res = 0, spd = 4;
	pthread_t sensor_thread;
	void *thread_result;
	int opt;
	uint16_t pattern[20][8] = 
	{   
		{0x0100, 0x0200, 0x0300, 0x0404, 0x051F, 0x0614, 0x0707, 0x0802}, 
		{0x0100, 0x0200, 0x0300, 0x0405, 0x051E, 0x0615, 0x0706, 0x0808}, 

		{0x0800, 0x0700, 0x0600, 0x0504, 0x041F, 0x0314, 0x0207, 0x0102}, 
		{0x0800, 0x0700, 0x0600, 0x0505, 0x041E, 0x0315, 0x0206, 0x0108}, 

		{0x0100, 0x0200, 0x030C, 0x043E, 0x0525, 0x0606, 0x0705, 0x0808}, 
		{0x0100, 0x0200, 0x030C, 0x043F, 0x0524, 0x0607, 0x0704, 0x0802}, 


		{0x0800, 0x0700, 0x060C, 0x053E, 0x0425, 0x0306, 0x0205, 0x0108}, 
		{0x0800, 0x0700, 0x060C, 0x053F, 0x0424, 0x0307, 0x0204, 0x0102}, 

		{0x0100, 0x0219, 0x037E, 0x0468, 0x0509, 0x060E, 0x0708, 0x0804},
		{0x0100, 0x0218, 0x037F, 0x0469, 0x0508, 0x060F, 0x0709, 0x0810},

		{0x0800, 0x0719, 0x067E, 0x0568, 0x0409, 0x030E, 0x0208, 0x0104},
		{0x0800, 0x0718, 0x067F, 0x0569, 0x0408, 0x030F, 0x0209, 0x0110},

		{0x0110, 0x0209, 0x030F, 0x0408, 0x0508, 0x06EC, 0x07FB, 0x0819},
		{0x0104, 0x0208, 0x030E, 0x040B, 0x0508, 0x06E9, 0x07FF, 0x0818},

		{0x0119, 0x02FB, 0x03EC, 0x0408, 0x0508, 0x060F, 0x0709, 0x0810},
		{0x0118, 0x02FF, 0x03E9, 0x0408, 0x050B, 0x060E, 0x0708, 0x0804},

		{0x0118, 0x02FE, 0x03EF, 0x0409, 0x0509, 0x060F, 0x070F, 0x0801},
		{0x0118, 0x02FE, 0x03EF, 0x0409, 0x0509, 0x060F, 0x070F, 0x0801},
	};

	static const char *device = "/dev/spi_led";

	fdd = open(device, O_RDWR);

	retioctl = ioctl(fdd, SPI_PATTERN, (unsigned long)pattern);

	res = pthread_create(&sensor_thread, NULL, sensor_read, NULL);

	printf("\nEnter something to exit: \n");

	scanf("%d", opt);

	breakflag = 1;

	res = pthread_join(sensor_thread, &thread_result);

	order[0] = order[1] = 0;
	res = write(fdd, (char *) order, 1, NULL); // writing the 0,0, to turn off the display, to the led driver
	
	printf("\n\n Terminating.");
	usleep(300000);
	printf(" .");
	usleep(300000);
	printf(" .");
	usleep(300000);
	close(fdd);

	printf("\n\nprogram terminated\n\n");
}