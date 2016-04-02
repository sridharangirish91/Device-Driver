1. run the make file - use "make all"
2. give permission to the devices using "sudo chmod 777 /dev/<device_name>"
	where
		the in_queue device name is - in_q
		the out_queue 1 device name is - out_q1
		the out_queue 2 device name is - out_q2
		the out_queue 3 device name is - out_q3
3. compile the program and link pthreads using - gcc main_1.c -lpthread
4. run the output file - ./a.out

the same procedure holds good for both static and dynamic queues. I have placed the main function - main_1.c, the driver - Squeue.c and makefile - Makefile in the
corresponding folders labledded "static" and "dynamic"