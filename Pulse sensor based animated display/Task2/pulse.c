/* ----------------------------------------------- PULSE DRIVER - HCSR04 -------------------------------------------------- */
 

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/unistd.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/init.h>
#include <linux/moduleparam.h>

#define DEVICE_NAME 	"pulse"  // device name to be created and registered
#define echo_mux 		30
#define trig_mux 		31
#define trig 			14
#define echo 			15

/* per device structure */
struct hcsr04_dev 
{
	struct cdev cdev;               /* The cdev structure */
	char name[20];                  /* Name of device*/
} *hcsr04_devp;

static dev_t hcsr04_dev_number;      /* Allotted device number */
struct class *hcsr04_dev_class;          /* Tie with the device model */
static struct device *hcsr04_dev_device;
int edgeflag=0;
int echoirq;
unsigned long long risetime, falltime;
long long meantime;

int busyflag = 0;

static char *user_name = "Dear John";

module_param(user_name,charp,0000);	//to get parameter from load.sh script to greet the user

// get time rtdsc
unsigned long long int get_time(void)
{
    unsigned long int lo, hi;
    asm( "rdtsc" : "=a" (lo), "=d" (hi) );
    return( lo | ((uint64_t)hi << 32) );
}

/*
* Open hcsr04 driver
*/

int hcsr04_driver_open(struct inode *inode, struct file *file)
{
	struct hcsr04_dev *hcsr04_devp;

	/* Get the per-device structure that contains this cdev */
	hcsr04_devp = container_of(inode->i_cdev, struct hcsr04_dev, cdev);


	/* Easy access to cmos_devp from rest of the entry points */
	file->private_data = hcsr04_devp;
	return 0;
}

/*
 * Release hcsr04 driver
 */
int hcsr04_driver_release(struct inode *inode, struct file *file)
{
	
	return 0;
}

/*
 * Write to hcsr04 driver
 */
ssize_t hcsr04_driver_write(struct file *file, const char *buf,
           size_t count, loff_t *ppos)
{
	// giving the trigger
	gpio_set_value_cansleep(trig,0);
	gpio_set_value_cansleep(trig,1);

	usleep_range(20,30);

	gpio_set_value_cansleep(trig,0);
	
	return 0;
}
/*
 * Read to hcsr04 driver
 */
ssize_t hcsr04_driver_read(struct file *file, char *buf,
           size_t count, loff_t *ppos)
{
	int bytes_sent = 0;
	/*
	 * If we're at the end of the message, 
	 * return 0 signifying end of file 
	 */

	 // return -1 if measurement is still happening
	if(busyflag)
	{
		return -1;
	}

	/* 
	 * put the data into the user buffer 
	 */
	meantime = meantime / 4;
	bytes_sent = copy_to_user(buf, &meantime, sizeof(meantime));
	/* 
	 * Most read functions return the number of bytes put into the buffer
	 */
	return bytes_sent;

}

static irqreturn_t irq_pulse(int irq, void *id)
{
	
	if(edgeflag == 0) // for rising edge
	{
		busyflag = 1;
		risetime = get_time();
		irq_set_irq_type(irq, IRQF_TRIGGER_FALLING); // changing edge to falling
		edgeflag = 1;
		busyflag = 0;
		return IRQ_HANDLED;
	}

	else if(edgeflag == 1) // for falling edge
	{
		falltime = get_time();
		irq_set_irq_type(irq, IRQF_TRIGGER_RISING); // changing edge to rising
		edgeflag = 0;
		meantime = falltime - risetime;
		busyflag = 0;
		return IRQ_HANDLED;
	}
	return IRQ_HANDLED;
}

/* File operations structure. Defined in linux/fs.h */
static struct file_operations hcsr04_fops = {
    .owner		= THIS_MODULE,           /* Owner */
    .open		= hcsr04_driver_open,        /* Open method */
    .release	= hcsr04_driver_release,     /* Release method */
    .write		= hcsr04_driver_write,       /* Write method */
    .read		= hcsr04_driver_read,        /* Read method */
};

/*
 * Driver Initialization
 */

int __init hcsr04_driver_init(void)
{
	int ret;
	int irq, resr;

	/* Request dynamic allocation of a device major number */
	if (alloc_chrdev_region(&hcsr04_dev_number, 0, 1, DEVICE_NAME) < 0) {
			// printk(KERN_DEBUG "Can't register device\n"); return -1;
	}

	/* Populate sysfs entries */
	hcsr04_dev_class = class_create(THIS_MODULE, DEVICE_NAME);

	/* Allocate memory for the per-device structure */
	hcsr04_devp = kmalloc(sizeof(struct hcsr04_dev), GFP_KERNEL);
		
	if (!hcsr04_devp) {
		// printk("Bad Kmalloc\n"); return -ENOMEM;
	}

	/* Request I/O region */
	sprintf(hcsr04_devp->name, DEVICE_NAME);

	/* Connect the file operations with the cdev */
	cdev_init(&hcsr04_devp->cdev, &hcsr04_fops);
	hcsr04_devp->cdev.owner = THIS_MODULE;

	/* Connect the major/minor number to the cdev */
	ret = cdev_add(&hcsr04_devp->cdev, (hcsr04_dev_number), 1);

	if (ret) {
		// printk("Bad cdev\n");
		return ret;
	}

	/* Send uevents to udev, so it'll create /dev nodes */
	hcsr04_dev_device = device_create(hcsr04_dev_class, NULL, MKDEV(MAJOR(hcsr04_dev_number), 0), NULL, DEVICE_NAME);		

	// initializing gpio

	gpio_request(trig_mux,"14_enable_mux");
    gpio_direction_output(trig_mux,0);
    gpio_set_value_cansleep(trig_mux,0);

    gpio_request(echo_mux,"15_enable_mux");
    gpio_direction_output(echo_mux,0);
    gpio_set_value_cansleep(echo_mux,0);

    gpio_request(trig, "trigger");
    gpio_direction_output(trig,0);
    gpio_set_value_cansleep(trig,0);

    gpio_request(echo, "echo");
    gpio_direction_input(echo);

    // initializing interrupt

    irq = gpio_to_irq(echo);
    echoirq=irq;	

	if(irq < 0)
		 printk("IRQ error\n");

	resr = request_irq(irq, irq_pulse, IRQF_TRIGGER_RISING, DEVICE_NAME, NULL);

	if (resr < 0)
		 printk("request_irq error\n");
	

	printk("pulse driver inserted\n");
	return 0;

}
/* Driver Exit */
void __exit hcsr04_driver_exit(void)
{

	//free irq
	free_irq(echoirq, NULL);

	// device_remove_file(hcsr04_dev_device, &dev_attr_xxx);
	/* Release the major number */
	unregister_chrdev_region((hcsr04_dev_number), 1);

	/* Destroy device */
	device_destroy (hcsr04_dev_class, MKDEV(MAJOR(hcsr04_dev_number), 0));
	cdev_del(&hcsr04_devp->cdev);
	kfree(hcsr04_devp);

	// free gpio
	gpio_free(trig);
	gpio_free(trig_mux);
	gpio_free(echo);
	gpio_free(echo_mux);
	
	/* Destroy driver_class */
	class_destroy(hcsr04_dev_class);

	printk("pulse driver removed\n");
}

module_init(hcsr04_driver_init);
module_exit(hcsr04_driver_exit);

MODULE_AUTHOR("Sridharan Rajagopalan");
MODULE_DESCRIPTION("Pulse driver");
MODULE_LICENSE("GPL v2");
