#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

char *arr, *rear;

void main()
{
	int res, ch, ret, pgs, rets = 9, pg =0, dummy;
	long tmp;
	// opening module
	int fd = open("/dev/i2c_flash", O_RDWR);
	arr = malloc(32768);
	rear = malloc(32768);
	if(fd < 0)
		printf("\nError opening module\n");

	else
	{	
		while(1)
		{
			//user menu
			printf("\n1. Read\n2. Write\n3. Check EEPROM Status\n4. Set Page\n5. Get Page\n6. Erase\n7. Exit\nEnter your choice: ");
			scanf("%d", &ch);
			switch(ch)
			{
				case 1:
				//read
						printf("\n Enter number of pages to be read: ");
						scanf("%d", &pg);
						ret = read(fd, rear, pg);
						if(ret < 0)
							printf("\nTRY AGAIN\n");
						else if(ret == -16)
							printf("\nEEPROM BUSY\n");
						else
							printf("\n\n\n String read is: %s\n\n\n ",rear);
						sleep(1);
						break;
				case 2:
				//write
						printf("\n Enter number of pages to be written: ");
						scanf("%d", &pg);
						printf("\n Enter the string to be written: ");
						scanf("%s", arr);
						ret = write(fd, arr, pg);
						if(ret < 0)
							printf("\nEEPROM BUSY\n");
						if(ret == 0)
							printf("\n\n\nWRITE SUCCESSFUL\n\n\n");
						break;
				case 3:
				//ioctl
						ret = ioctl(fd, 0, &res);
						if(res == 1)
							printf("\n EEPROM BUSY\n");
						else
							printf("\n EEPROM NOT BUSY\n");
						break;
				case 4:
				//set page
						printf("\n Enter page number: ");
						scanf("%d", &pgs);
						long tmp = pgs;
						rets = ioctl(fd, 4, &tmp);
						break;
				case 5:
				//get page
						ret = ioctl(fd, 1, &pg);
						printf("\n Current page number: %d", pg);
						break;
				case 6:
				//erase
						ret = ioctl(fd, 3);
						break;
				case 7:
				//exit
						close(fd);
						printf("\n\nTHANK YOU\n\n");
						exit(0);
				default:
				
						printf("INVALID OPTION");
						break;
			}
			sleep(1);
		}
	}
	free(arr);
	free(rear);
}