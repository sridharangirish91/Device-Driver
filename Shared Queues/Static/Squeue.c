/* ----------------------------------------------- DRIVER gmem --------------------------------------------------
 
 Basic driver example to show skelton methods for several file operations.
 
 ----------------------------------------------------------------------------------------------------------------*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/string.h> 
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/unistd.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>

#include<linux/init.h>
#include<linux/moduleparam.h>
#define IN_Q_NAME               "in_q"  // device name to be created and registered
#define OUT_Q1_NAME             "out_q1"  // device name to be created and registered
#define OUT_Q2_NAME             "out_q2"  // device name to be created and registered
#define OUT_Q3_NAME             "out_q3"  // device name to be created and registered

int headin=-1, tailin=-1, headout1=-1, tailout1=-1, headout2=-1, tailout2=-1, headout3=-1, tailout3=-1; // queue parameters
int eqflag;

/* message structure */
struct record
{
	int msg_id, source_id, dest_id; // message id, source id and destination id
	char msg[80]; // message body
	struct timeval curTime;	// keeps the accummulating time 
};

/* per device structure */
struct Q_NAME 
{
	struct cdev cdev;               /* The cdev structure */
	char name[20];                  	/* Name of device*/
	struct record queue[10];	
	struct mutex mut;				
	int current_write_pointer;
	int tailin, headin, number;
} *bus_in_q, *bus_out_q1, *bus_out_q2, *bus_out_q3;

static dev_t bus_in_q_dev_number,bus_out_q1_dev_number, bus_out_q2_dev_number, bus_out_q3_dev_number;      /* Allotted device number */
struct class *bus_in_q_dev_class, *bus_out_q1_dev_class, *bus_out_q2_dev_class, *bus_out_q3_dev_class;          /* Tie with the device model */
static struct device *bus_in_q_device, *bus_out_q1_device, *bus_out_q2_device, *bus_out_q3_device;

static char *user_name = "Dear John";

module_param(user_name,charp,0000);	//to get parameter from load.sh script to greet the user

int Q_NAME_driver_open(struct inode *inode, struct file *file)
{
	struct Q_NAME *Q_NAME_devp;
	
//	printk("\nopening\n");

	/* Get the per-device structure that contains this cdev */
	Q_NAME_devp = container_of(inode->i_cdev, struct Q_NAME, cdev);


	/* Easy access to cmos_devp from rest of the entry points */
	file->private_data = Q_NAME_devp;
	printk("\n%s is openning \n", Q_NAME_devp->name);
	return 0;
}

/*
 * Release gmem driver
 */
int Q_NAME_driver_release(struct inode *inode, struct file *file)
{
	struct Q_NAME *Q_NAME_devp = file->private_data;
	
	
	printk("\n%s is closing\n", Q_NAME_devp->name);
	
	return 0;
}

// enqueue function 

int enqueue(struct Q_NAME *tempo,const char *buf)
{
	struct record data;
	int a;
	
	a = copy_from_user(&data, buf, sizeof(struct record));// copy from user instead of get user to get structures from user-space

	if(a)
	{
		// ERROR WRITING
		return -2;
	}

	 if(headin == -1) // checks for first node
	 {
	 	tempo->number=1;
	 	headin++;
	 	tailin = 0;
	 	tempo->queue[tailin] = data;
	 	printk(KERN_INFO "\n WRITING IN Q %d TAIL = %d  HEAD = %d  DATA = %d \n", data.dest_id, tailin, headin, tempo->queue[tailin].msg_id);
	}

	else if(tempo->number <= 10)
	{
		tailin ++;
		if(tailin > 9)
			tailin = 0;
		tempo->queue[tailin] = data;
		tempo->number ++;
		printk(KERN_INFO "\n WRITING IN Q %d TAIL = %d  HEAD = %d  DATA = %d \n", data.dest_id, tailin, headin, tempo->queue[tailin].msg_id);
	}
	else
	{
		//FULL QUEUE
		return -1;
	}

	return 0;
}

/*
 * Write to gmem driver
 */
ssize_t Q_NAME_driver_write(struct file *file, const char *buf,
           size_t count, loff_t *ppos)
{

	
	struct Q_NAME *Q_NAME_devp = file->private_data;
	int b;	
	mutex_lock(&(Q_NAME_devp->mut));
	b = enqueue(Q_NAME_devp, buf);
	mutex_unlock(&(Q_NAME_devp->mut));
	
	if(b == -2)
	{
		//Error getting data from user space
		return -2;
	}

	else if(b == -1)
	{
		//QUEUE FULL
		return -1;
	}
		
	return 0;
}

// dequeue function

int dequeue(struct Q_NAME *tempo, char *buf)
{
	
	struct record data;
	int usr_er;

	if(tempo->number < 1)
	{
		//QUEUE EMPTY
	 	return -1;
	}

	else
	{
		data = tempo->queue[headin++];
		printk("\n READING FROM Q %d TAIL = %d  HEAD = %d  DATA = %d \n", data.source_id, tailin, headin, data.msg_id);
		if(headin > 9)
			headin = 0;
		tempo->number --;
	}

	usr_er = copy_to_user(buf, &data, sizeof(struct record));
	if(usr_er)
	{
		//error copying to user
		return -2;
	}
	printk("\nWrite complete\n");
	return 0;
}

/*
 * Read to gmem driver
 */
ssize_t Q_NAME_driver_read(struct file *file, char *buf,
           size_t count, loff_t *ppos)
{
	int er;
	struct Q_NAME *Q_NAME_devp = file->private_data;
	printk("\n\n Inside read  \n\n");
	mutex_lock(&(Q_NAME_devp->mut));
	er = dequeue(Q_NAME_devp, buf);
	mutex_unlock(&(Q_NAME_devp->mut));
	/*
	 * return -1 if queue is empty
	 */
	if (er != 0)
	{
		// EMPTY QUEUE
		
		return -1;
	}
	else if(er == -2)
	{
		// error copying from kernel to user buffer
		return -2;
	}
	else
	{
		/* 
		 * Actually put the data into the buffer 
		 */
		printk("\nRead complete\n");
		/* 
		 * Most read functions return the number of bytes put into the buffer
		 */
	}
	return 0;

}

struct file_operations Q_NAME_fops = 
{
	.owner		= THIS_MODULE,           	 /* Owner */
    .open		= Q_NAME_driver_open,        /* Open method */
    .release	= Q_NAME_driver_release,     /* Release method */
    .write		= Q_NAME_driver_write,       /* Write method */
    .read		= Q_NAME_driver_read,        /* Read method */
};

int __init Q_NAME_driver_init(void)
{
	int ret;
	printk("\nEntered init function\n");
	/* Request dynamic allocation of a device major number */
	if (alloc_chrdev_region(&bus_in_q_dev_number, 0, 4, IN_Q_NAME) < 0) {
			printk(KERN_DEBUG "Can't register device - input queue\n"); return -1;
	}

	/* Populate sysfs entries */
	bus_in_q_dev_class = class_create(THIS_MODULE, IN_Q_NAME);
	bus_out_q1_dev_class = class_create(THIS_MODULE, OUT_Q1_NAME);
	bus_out_q2_dev_class = class_create(THIS_MODULE, OUT_Q2_NAME);
	bus_out_q3_dev_class = class_create(THIS_MODULE, OUT_Q3_NAME);

	/* Allocate memory for the per-device structure */
	bus_in_q = kmalloc(sizeof(struct Q_NAME), GFP_KERNEL);
	bus_out_q1 = kmalloc(sizeof(struct Q_NAME), GFP_KERNEL);
	bus_out_q2 = kmalloc(sizeof(struct Q_NAME), GFP_KERNEL);
	bus_out_q3 = kmalloc(sizeof(struct Q_NAME), GFP_KERNEL);

		
	if (!bus_in_q) {
		printk("Bad Kmalloc - in queue\n"); return -ENOMEM;
	}

	if (!bus_out_q1) {
		printk("Bad Kmalloc - out queue 1\n"); return -ENOMEM;
	}

	if (!bus_out_q2) {
		printk("Bad Kmalloc - out queue 2\n"); return -ENOMEM;
	}

	if (!bus_out_q3) {
		printk("Bad Kmalloc - out queue 3\n"); return -ENOMEM;
	}

	/* Request I/O region */
	sprintf(bus_in_q->name, IN_Q_NAME);
	sprintf(bus_out_q1->name, OUT_Q1_NAME);
	sprintf(bus_out_q2->name, OUT_Q2_NAME);
	sprintf(bus_out_q3->name, OUT_Q3_NAME);

	/* Connect the file operations with the cdev */
	cdev_init(&bus_in_q->cdev, &Q_NAME_fops);
	bus_in_q->cdev.owner = THIS_MODULE;

	cdev_init(&bus_out_q1->cdev, &Q_NAME_fops);
	bus_out_q1->cdev.owner = THIS_MODULE;

	cdev_init(&bus_out_q2->cdev, &Q_NAME_fops);
	bus_out_q2->cdev.owner = THIS_MODULE;

	cdev_init(&bus_out_q3->cdev, &Q_NAME_fops);
	bus_out_q3->cdev.owner = THIS_MODULE;


	/* Connect the major/minor number to the cdev */
	ret = cdev_add(&bus_in_q->cdev, bus_in_q_dev_number, 1);
	if (ret) {
		printk("Bad cdev in queue\n");
		return ret;
	}

	ret = cdev_add(&bus_out_q1->cdev, bus_in_q_dev_number+1, 1);
	if (ret) {
		printk("Bad cdev out queue 1\n");
		return ret;
	}

	ret = cdev_add(&bus_out_q2->cdev, bus_in_q_dev_number+2, 1);
	if (ret) {
		printk("Bad cdev out queue 2\n");
		return ret;
	}

	ret = cdev_add(&bus_out_q3->cdev, (bus_in_q_dev_number+3), 1);
	if (ret) {
		printk("Bad cdev out queue 3\n");
		return ret;
	}

	

	/* Send uevents to udev, so it'll create /dev nodes */
	bus_in_q_device = device_create(bus_in_q_dev_class, NULL, MKDEV(MAJOR(bus_in_q_dev_number), 0), NULL, IN_Q_NAME);		
	bus_out_q1_device = device_create(bus_out_q1_dev_class, NULL, MKDEV(MAJOR(bus_in_q_dev_number), 1), NULL, OUT_Q1_NAME);
	bus_out_q2_device = device_create(bus_out_q2_dev_class, NULL, MKDEV(MAJOR(bus_in_q_dev_number), 2), NULL, OUT_Q2_NAME);
	bus_out_q3_device = device_create(bus_out_q3_dev_class, NULL, MKDEV(MAJOR(bus_in_q_dev_number), 3), NULL, OUT_Q3_NAME);
	// device_create_file(gmem_dev_device, &dev_attr_xxx);

	memset(bus_in_q, 0, sizeof(bus_in_q)); // sizeof the queue is 1120
	memset(bus_out_q1, 0, sizeof(bus_out_q1)); // sizeof the queue is 1120
	memset(bus_out_q2, 0, sizeof(bus_out_q2)); // sizeof the queue is 1120
	memset(bus_out_q3, 0, sizeof(bus_out_q3)); // sizeof the queue is 1120
	
	mutex_init(&bus_in_q->mut);
	mutex_init(&bus_out_q1->mut);
	mutex_init(&bus_out_q2->mut);
	mutex_init(&bus_out_q3->mut);

	bus_in_q->tailin = bus_in_q->headin = -1;
	bus_out_q1->tailin = bus_out_q1->headin = -1;
	bus_out_q2->tailin = bus_out_q2->headin = -1;
	bus_out_q3->tailin = bus_out_q3->headin = -1;

	bus_in_q->number = 0;
	bus_out_q1->number = 0;
	bus_out_q2->number = 0;
	bus_out_q3->number = 0;

	printk("\nqueue drivers initialized.\n");
	return 0;
}
/* Driver Exit */
void __exit Q_NAME_driver_exit(void)
{
	printk("\nEntered exit function\n");
	// device_remove_file(gmem_dev_device, &dev_attr_xxx);
	/* Release the major number */
	unregister_chrdev_region((bus_in_q_dev_number), 1);
	unregister_chrdev_region((bus_out_q1_dev_number), 1);
	unregister_chrdev_region((bus_out_q2_dev_number), 1);
	unregister_chrdev_region((bus_out_q3_dev_number), 1);

	/* Destroy device */
	device_destroy (bus_in_q_dev_class, MKDEV(MAJOR(bus_in_q_dev_number), 0));
	device_destroy (bus_out_q1_dev_class, MKDEV(MAJOR(bus_in_q_dev_number), 1));
	device_destroy (bus_out_q2_dev_class, MKDEV(MAJOR(bus_in_q_dev_number), 2));
	device_destroy (bus_out_q3_dev_class, MKDEV(MAJOR(bus_in_q_dev_number), 3));

	cdev_del(&bus_in_q->cdev);
	cdev_del(&bus_out_q1->cdev);
	cdev_del(&bus_out_q2->cdev);
	cdev_del(&bus_out_q3->cdev);

	kfree(bus_in_q);
	kfree(bus_out_q1);
	kfree(bus_out_q2);
	kfree(bus_out_q3);
	
	/* Destroy driver_class */
	class_destroy(bus_in_q_dev_class);
	class_destroy(bus_out_q1_dev_class);
	class_destroy(bus_out_q2_dev_class);
	class_destroy(bus_out_q3_dev_class);

	printk("queue drivers removed.\n");
}

module_init(Q_NAME_driver_init);
module_exit(Q_NAME_driver_exit);
MODULE_LICENSE("GPL v2");