#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>
#include <time.h>

struct record // structure of data 
{
	int msg_id, source_id, dest_id; // message id, source id and destination id
	char msg[80]; // message body
	long double curtime, prevtime, enqtime; // for time stamp
} sender1, sender2, sender3, receiver1, daemon_rec, receiver2, receiver3; // device objects

int globmsgid = 0, sfw1, sfw2, sfw3, sfr1, sfr2, sfr3, sfd;
pthread_mutex_t sender_mutex, receiver1_mutex, receiver2_mutex, receiver3_mutex;	

// get time for time stamp - gives the number of cycles from inception to the current point
// source reference: http://stackoverflow.com/questions/17401914/why-should-i-use-rdtsc-diferently-on-x86-and-x86-x64
unsigned long get_time()
{
	unsigned long lo, hi;
    asm( "rdtsc" : "=a" (lo), "=d" (hi) ); 
    return( lo | ((uint64_t)hi << 32) );
}

// source for random number function: http://stackoverflow.com/questions/822323/how-to-generate-a-random-number-in-c
int random_number(int min_num, int max_num) // generates random numbers
{
    int result=0,low_num=0,hi_num=0;
    if(min_num<max_num)
    {
        low_num=min_num;
        hi_num=max_num+1; // this is done to include max_num in output.
    }else{
        low_num=max_num+1;// this is done to include max_num in output.
        hi_num=min_num;
    }
    srand(get_time());
    result = (rand()%(hi_num-low_num))+low_num;
    return result;
}

void *thread_write_function1() // sender thread 1
{
	int fd, res;
	
	while(1)
	{
		usleep(100);
		fd = open("/dev/in_q", O_RDWR);
		if (fd < 0 ) 	
		{
	 		printf("\nWrite 1 Can not open device file.\n");		
	 		break;
	 	}

	 	// locking mutex and writing from sender 1

 		pthread_mutex_lock(&sender_mutex);
	 	sender1.msg_id = globmsgid++;
	 	sender1.source_id = 1;
	 	sender1.dest_id = random_number(1,3); // generating a random destination
	 	sender1.prevtime = get_time();
		res = write(fd, &sender1, sizeof(sender1));
		pthread_mutex_unlock(&sender_mutex); // unlock mutex after writing to the queue
		close(fd);
		
		printf("\nSender 1 Writing in destination queue %d. msg_id: %d\n", sender1.dest_id, sender1.msg_id);

		if(sfw1 == 1) // exit the loop after 10s
		{
			break;
		}

	}
	pthread_exit("exit write 1 thread");
}

void *thread_write_function2() // sender thread 2
{
	int fd, res;
	// thread funcitonality
	while(1)
	{
		usleep(100);
		fd = open("/dev/in_q", O_RDWR);
		if (fd < 0 )
		{
	 		printf("\nWrite 2 Can not open device file.\n");		
	 		break;
	 	}
	 	// locking mutex and writing into the queue
 		pthread_mutex_lock(&sender_mutex);
	 	sender2.msg_id = globmsgid++;
	 	sender2.source_id = 2;
	 	sender2.dest_id = random_number(1,3); // generating random destination
	 	sender2.prevtime = get_time();
		res = write(fd, &sender2, sizeof(sender2));
		pthread_mutex_unlock(&sender_mutex); //  mutex unlock
		close(fd);
		printf("\nSender 2 Writing in destination queue %d. msg_id: %d\n", sender2.dest_id, sender2.msg_id);
		if(sfw2 == 1) // exit loop after 10s
		{
			break;
		}

	}
	pthread_exit("exit write 2 thread");
}

void *thread_write_function3() // sender thread 3
{
	int fd, res;
	//thread functionality
	while(1)
	{
		usleep(100);
		fd = open("/dev/in_q", O_RDWR);
		if (fd < 0 )
		{
	 		printf("\nWrite 3 Can not open device file.\n");		
	 		break;
	 	}
	 	// locking mutex and writing into the queue
 		pthread_mutex_lock(&sender_mutex);
	 	sender3.msg_id = globmsgid++;
	 	sender3.source_id = 3;
	 	sender3.dest_id = random_number(1,3); // generating random destination
	 	sender3.prevtime = get_time();
		res = write(fd, &sender3, sizeof(sender3));
		pthread_mutex_unlock(&sender_mutex); // unlocking the mutex
		close(fd);
		printf("\nSender 3 Writing in destination queue %d. msg_id: %d\n", sender3.dest_id, sender3.msg_id);
		if(sfw3 == 1) // exiting the loop after 10s
		{
			break;
		}

	}
	pthread_exit("exit write 3 thread");
}


void *thread_read_function1() // receiver thread 1
{
	int  fd, res;
	// receiver thread 1 functionality
	while(1)
 	{
 		usleep(200);
 		fd = open("/dev/out_q1", O_RDWR);
		if (fd < 0 )
		{
	 		printf("\nRead 1 Can not open device file.\n");		
	 		break;
	 	}
	 	// locking mutex and reading the queue value
 		pthread_mutex_lock(&receiver1_mutex);
		res = read(fd, &receiver1, sizeof(receiver1));
		receiver1.curtime = get_time();
		receiver1.enqtime = (receiver1.curtime - receiver1.prevtime)/400000000.0;
		pthread_mutex_unlock(&receiver1_mutex);
		close(fd);
		if(sfr1 == 1 && res == -1) // exiting loop after 10s and the queue is empty
		{
			break;
		}
		else
		{
			printf("\nQueue 1 Read : msg_id: %d. source_id is %d. time stamp is: %Lf \n", receiver1.msg_id, receiver1.source_id, receiver1.enqtime);
		}
	}
	
	pthread_exit("exit read 1 thread");
}

void *thread_read_function2()
{
	int fd, res;
	while(1)
 	{
 		usleep(200);
 		fd = open("/dev/out_q2", O_RDWR);
		if (fd < 0 )
		{
	 		printf("\nRead 2 Can not open device file.\n");		
	 		break;
	 	}
	 	// locking mutex and reading queue 
 		pthread_mutex_lock(&receiver2_mutex);
		res = read(fd, &receiver2, sizeof(receiver2));
		pthread_mutex_unlock(&receiver2_mutex);
		close(fd);
		receiver2.curtime = get_time();
		receiver2.enqtime = (receiver2.curtime - receiver2.prevtime)/400000000.0;
		if(sfr2 == 1 && res == -1) // exiting loop after 10s and the queue is empty
		{
			break;
		}
		else
		{
			printf("\nQueue 2 Read : msg_id: %d. source_id is %d. time stamp is: %Lf \n", receiver2.msg_id, receiver2.source_id, receiver2.enqtime);
		}
	}
	pthread_exit("exit read 2 thread");
}

void *thread_read_function3()
{
	int fd, res;
 	while(1)
 	{
 		usleep(200);
 		fd = open("/dev/out_q3", O_RDWR);
		if (fd < 0 )
		{
	 		printf("\nREAD 3 Can not open device file.\n");		
	 		break;
	 	}
	 	//locking mutex and reading from queue
 		pthread_mutex_lock(&receiver3_mutex);
		res = read(fd, &receiver3, sizeof(receiver3));
		pthread_mutex_unlock(&receiver3_mutex);
		close(fd);
		receiver3.curtime = get_time();
		receiver3.enqtime = (receiver3.curtime - receiver3.prevtime)/400000000.0;
		if(sfr3 == 1 && res == -1) // exiting loop after 10s and the queue is empty
		{
			break;
		}
		else
		{
			printf("\nQueue 3 Read : msg_id: %d. source_id is %d. time stamp is: %Lf \n", receiver3.msg_id, receiver3.source_id, receiver3.enqtime);
		}
		if(sfr3 == 1)
		{
			break;
		}
		
		
	}
	
	pthread_exit("exit read 3 thread");
}



// daemon thread


void *thread_daemon()
{
	int fd_in, fd_out1, fd_out2, fd_out3;
	int resq, res;

	//daemon funcitonality
	while(1)
	{		
		usleep(50);
		if (fd_in < 0 )
		{
			printf("\nDaemon Can not open input device file.\n");		
			break;
		}

		fd_in = open("/dev/in_q", O_RDWR);
		// locking the in queue and reading data from in queue
		pthread_mutex_lock(&sender_mutex);
		res = read(fd_in, &daemon_rec, sizeof(daemon_rec));
		pthread_mutex_unlock(&sender_mutex);
		close(fd_in);

		if(sfd == 1 && res == -1) // breaking the loop after 10s and all the in queue is empty
		{
			break;
		}

		// routing the read data to the desired output queue as per the destination id

		if(daemon_rec.dest_id == 1)
		{
			fd_out1 = open("/dev/out_q1", O_RDWR);
			pthread_mutex_lock(&receiver1_mutex);
			resq = write(fd_out1, &daemon_rec, sizeof(daemon_rec));
			pthread_mutex_unlock(&receiver1_mutex);
			close(fd_out1);
		}
		else if(daemon_rec.dest_id == 2)
		{
			fd_out2 = open("/dev/out_q2", O_RDWR);
			pthread_mutex_lock(&receiver2_mutex);
			resq = write(fd_out2, &daemon_rec, sizeof(daemon_rec));
			pthread_mutex_unlock(&receiver2_mutex);
			close(fd_out2);
		}
		else if(daemon_rec.dest_id == 3)
		{
			fd_out3 = open("/dev/out_q3", O_RDWR);
			pthread_mutex_lock(&receiver3_mutex);
			resq = write(fd_out3, &daemon_rec, sizeof(daemon_rec));
			pthread_mutex_unlock(&receiver3_mutex);
			close(fd_out3);	
		}
		else
			continue;
	
	}
	pthread_exit("exit daemon thread");
}

// main function

int main(int argc, char **argv)
{

	int res, i;
    pthread_t w1_thread, w2_thread, w3_thread, r1_thread, r2_thread, r3_thread, d_thread;
    void *thread_result;
    // craetint all the threads
    res = pthread_create(&w1_thread, NULL, thread_write_function1, NULL);
    res = pthread_create(&w2_thread, NULL, thread_write_function2, NULL);
    res = pthread_create(&w3_thread, NULL, thread_write_function3, NULL);
    res = pthread_create(&d_thread, NULL, thread_daemon, NULL);
 	res = pthread_create(&r1_thread, NULL, thread_read_function1, NULL);
    res = pthread_create(&r2_thread, NULL, thread_read_function2, NULL);
    res = pthread_create(&r3_thread, NULL, thread_read_function3, NULL);
    
    sleep(10);
    // sleepflags to denote the end of the threads after 10s
	sfd = sfw1 = sfw2 = sfw3 = sfr1 = sfr2 = sfr3 = 1;
    usleep(20);
    // joining the threads back
    res = pthread_join(w1_thread, &thread_result);
    res = pthread_join(w2_thread, &thread_result);
    res = pthread_join(w3_thread, &thread_result);
	res = pthread_join(d_thread, &thread_result);
    res = pthread_join(r1_thread, &thread_result);
    res = pthread_join(r2_thread, &thread_result);
    res = pthread_join(r3_thread, &thread_result);
    printf("\n program terminated\n");
}