// reference skeleton of code from i2c-dev.c (freeelectrons.com)

// workqueue reference from threading slides - slide 25
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/unistd.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/cdev.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>

struct i2c_dev {
        struct cdev cdev;
        struct list_head list;
        struct i2c_adapter *adap;
        char name[20];
        struct device *dev;
}*i2c_eep;
int dataready = 0;
char *temp_buffer, *temp_read;

#define I2C_MINORS      	256 
#define I2C_MAJORN   		0  	// for dynamic memory allocation
#define FLASHGETS          	0  	/* Use this slave address */
#define FLASHGETP          	1  	/* Use this slave address, even if it
                                    is already in use by a driver! */
#define FLASHSETP          	4  	/* 0 for 7 bit addrs, != 0 for 10 bit */
 
#define FLASHERASE         	3  	/* Get the adapter functionality mask */
#define I2C_FLASH           "i2c_flash"
#define I2C_PIN				29	// GPIO pin to enable i2c mux
#define LED_PIN				26 	// GPIO pin to control LED pins


static LIST_HEAD(i2c_dev_list);

static dev_t i2c_flash_number;

struct class *i2c_class;
int resmajor, busy_flag = 0; // busy flag
short addrw = 0;
struct i2c_adapter *ad;

static struct workqueue_struct *my_wq;
typedef struct {
struct work_struct my_work;
struct file *file;
size_t count;
} my_work_t;

my_work_t *work, *work2;


// erase function
int eraseall(struct file *file) 
{

    int ret = 0;
    int i = 0;
    short addre = 0;
    char *writedata;      
    
    struct i2c_client *client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);

    gpio_set_value_cansleep(LED_PIN,1);
    addrw = 0;
    busy_flag = 1;
    client = file->private_data;
    client -> adapter = ad;

    
    writedata = kzalloc(2, GFP_KERNEL);
    
    // writing 0xFF into 512 pages
    for(i = 0; i < 512 ; i ++)
    {
        msleep(5);
        writedata[0] = (char)((addre>>8)& 0xff);
        writedata[1] = (char)(addre & 0xff);
        memset(temp_buffer, 1, 64);
        memcpy(writedata+2, &(temp_buffer[i*64]), 64);
        msleep(10);
        ret += i2c_master_send(client, writedata, 66);
        addre = addre >> 6;
        addre ++;
        addre = addre << 6;
    }
    
    kfree(writedata);
    kfree(client);
    busy_flag = 0;
    gpio_set_value_cansleep(LED_PIN,0);
    return 0;
}


// work queue read
static int workread(struct work_struct *work)
{
    
    char *readdata;
    int ret;
    int i;

    struct i2c_client *client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
    //getting parameters from write function
    my_work_t *my_work = (my_work_t *)work;
    busy_flag = 1;   
    client = my_work->file->private_data;
    client->adapter = ad;
    
    readdata = kzalloc(2, GFP_KERNEL);
        msleep(10);
        // setting address
        readdata[0] = (char)((addrw>>8)& 0xff);
        readdata[1] = (char)(addrw & 0xff);
        memset(temp_buffer, 0, my_work->count*64);
        ret = i2c_master_send(client, readdata, 2);
        msleep(10);
        ret = i2c_master_recv(client, temp_buffer, my_work->count * 64);
        msleep(10);
        // for multiple pages
        for(i = 0; i < my_work->count; i++) 
        {
            addrw = addrw >> 6;
            addrw ++;
            if(addrw >= 511)
                addrw = 0;
            addrw = addrw << 6;
        }
            
        kfree(readdata);
    kfree(client);
    busy_flag = 0;
    return 0;
}
// read function
static ssize_t i2cdev_read(struct file *file, char __user *buf, size_t count,
                loff_t *offset)
{
    int ret = 0, retc;
    // setting led on
        gpio_set_value_cansleep(LED_PIN,1);
        //checking if data is ready and eeprom is busy
    if (dataready == 0 && busy_flag == 0) 
    {
        //work2 = (my_work_t *)kmalloc(sizeof(my_work_t), GFP_KERNEL);
        if(work2)
        {
            INIT_WORK((struct work_struct *)work2, workread);    
        }

        work2->file = file;
        work2->count = count;
        ret = queue_work( my_wq, (struct work_struct *)work2);
        memcpy(temp_read, temp_buffer, count * 64);
        dataready = 1;
        // setting led off
        gpio_set_value_cansleep(LED_PIN,0);
        // return -EAGAIN when the data is not ready and eeprom is not busy
        return -EAGAIN;
    }   
    // returning -EBUSY if eeprom is busy
    else if(dataready == 0 && busy_flag ==1)
    {
        // setting led off
        gpio_set_value_cansleep(LED_PIN,0);
        return -EBUSY;
    }
    // returning data when data is ready and eeprom is not busy
    if(dataready == 1)
    {
        
        retc = copy_to_user(buf, temp_read, count*64);
        // setting led off
        gpio_set_value_cansleep(LED_PIN,0);
        dataready = 0;
        return 0;
    }
    return 0;
}

// workqueue function for write
static int workwrite(struct work_struct *work) 
{

        my_work_t *my_work = (my_work_t *)work;
        
        int ret = 0;
        char *writedata;
        int i = 0, writemem;
        struct i2c_client *client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);

        busy_flag = 1;
        client = my_work->file->private_data;
        client -> adapter = ad;
        writemem = 64 + 2;
        writedata = kzalloc(writemem, GFP_KERNEL);
        // chekcing page count         
        if(my_work->count > 512)
        {
            my_work->count = 512;
        }
        
        // multiple write
        for(i = 0; i < my_work->count ; i ++)
        {
            memset(writedata, 0, sizeof(writedata));
            msleep(10);
            msleep(10);
            writedata[0] = (char)((addrw>>8)& 0xff);
            writedata[1] = (char)(addrw & 0xff);
            msleep(10);
            memcpy(writedata+2, &(temp_buffer[i*64]), 64);
            msleep(10);
            ret += i2c_master_send(client, writedata, 66);
            msleep(10);
            addrw = addrw >> 6;
            addrw ++;
            if(addrw >= 511)
                addrw = 0;
            addrw = addrw << 6;
        }
            
        msleep(10);
        kfree(writedata);
        busy_flag = 0;
        kfree(client);
        return 0;
    
    
}

// write
static ssize_t i2cdev_write(struct file *file, const char __user *buf,
                size_t count, loff_t *offset)
{
        int ret = 0, retc;
        // setting led on
        gpio_set_value_cansleep(LED_PIN,1);
        if(busy_flag == 0)
        {
            if (work) 
            {
                work->file = file;
                work->count = count;
                memset(temp_buffer, 0, count*64);
                retc = copy_from_user(temp_buffer, buf, count*64);
                INIT_WORK( (struct work_struct *)work, workwrite );
                ret = queue_work( my_wq, (struct work_struct *)work );
            }   
            // setting led off
            gpio_set_value_cansleep(LED_PIN,0);
            return 0;
        }
        else 
            return -EBUSY;
}
// open function
static int i2cdev_open(struct inode *inode, struct file *file)
{

        struct i2c_client *client;
        ad = kmalloc(sizeof(struct i2c_adapter), GFP_KERNEL);
        // allocating client and setting client adddress
        client = kmalloc(sizeof(*client), GFP_KERNEL);
        client->addr = 0x54;
        ad = i2c_get_adapter(0);
        if (!ad)
        {
                return -ENODEV;
        }
        
        if (!client) {
            ssleep(1);
                i2c_put_adapter(ad);
                return -ENOMEM;
        }
        // linking the adapter
        client->adapter = ad;
        file->private_data = client;
        return 0;
}
// release function
static int i2cdev_release(struct inode *inode, struct file *file)
{
        struct i2c_client *client = file->private_data;
        kfree(client);
        file->private_data = NULL;

        return 0;
}

// ioctl function
static long i2cdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
        int ret;
        long a;
        short p;
        switch (cmd) 
        {
            case FLASHGETS:
            // returning the busy flag to user
                    ret = copy_to_user((char __user*)arg, &busy_flag, sizeof(busy_flag));
                    return 0;

            case FLASHGETP:
            // shifting the page number and returning to user
                    p = addrw >> 6;
                    ret = copy_to_user((char __user*)arg, &p, sizeof(p));

                    return ret;


            case FLASHSETP:
                    ret = copy_from_user(&a, (char *)arg, sizeof(a));
                    // setting page number and assigning it to current page number
                    p = a << 6;
                    if(p < 0 || p > 511 )
                            return -1;
                    addrw = p;
                    return ret;
                    
            case FLASHERASE:
            // calling the eraseall function when user calls erase
                   ret = eraseall(file);
                   return ret;


            default:
                    return -ENOTTY;
        }
        return 0;
}

// linking the functions
static const struct file_operations i2c_dev_fops = {
        .owner          = THIS_MODULE,
       	.unlocked_ioctl = i2cdev_ioctl,
        .read           = i2cdev_read,
        .write          = i2cdev_write,
        .open           = i2cdev_open,
       	.release        = i2cdev_release,
};

// init function
static int __init i2c_dev_init(void)
{
        int ret;
        struct i2c_adapter *ad;
        
    /* Request dynamic allocation of a device major number */
    if (alloc_chrdev_region(&i2c_flash_number, 0, 1, I2C_FLASH) < 0) {
            printk(KERN_DEBUG "Can't register device - input queue\n"); return -1;
    }

    /* Populate sysfs entries */
    i2c_class = class_create(THIS_MODULE, I2C_FLASH);
    temp_buffer = kzalloc(512*64, GFP_KERNEL);
    temp_read = kzalloc(512*64, GFP_KERNEL);
    ad = to_i2c_adapter(0);
    /* Allocate memory for the per-device structure */
    i2c_eep = kmalloc(sizeof(struct i2c_dev), GFP_KERNEL); 

    if(!i2c_eep)
        return -1;
    i2c_eep->adap = ad;
    sprintf(i2c_eep->name, I2C_FLASH);

    /* Connect the file operations with the cdev */
    cdev_init(&i2c_eep->cdev, &i2c_dev_fops);
    i2c_eep->cdev.owner = THIS_MODULE;

    /* Connect the major/minor number to the cdev */
    ret = cdev_add(&i2c_eep->cdev, i2c_flash_number, 1);
    if (ret) {
        printk("Bad cdev\n");   
        return ret;
    }
    // creating device
    i2c_eep->dev = device_create(i2c_class, NULL,  MKDEV(MAJOR(i2c_flash_number),0), NULL, I2C_FLASH);

    //creating work queues
    my_wq = create_workqueue("my_queue");
    if(!my_wq)
    {
        return -1;       
        
    }
    work = (my_work_t *)kmalloc(sizeof(my_work_t), GFP_KERNEL);
    work2 = (my_work_t *)kmalloc(sizeof(my_work_t), GFP_KERNEL);
    // initializing I2C pins
    gpio_request(I2C_PIN,"I2C_enable_mux");
    gpio_direction_output(I2C_PIN,0);
    gpio_set_value_cansleep(I2C_PIN,0);
    //initializing LED
    gpio_request(LED_PIN, "LED");
    gpio_direction_output(LED_PIN,0);
    
    printk("\nInit complete\n");
    
    return 0;
}



static void __exit i2c_dev_exit(void)
{
	// unregistering device
    unregister_chrdev_region((i2c_flash_number), 1);
    device_destroy (i2c_class, MKDEV(i2c_flash_number, 0));
    cdev_del(&i2c_eep->cdev);
    // freeing memeory
    kfree(i2c_eep);
    kfree(work);
    kfree(temp_buffer);
    kfree(temp_read);
    kfree(work2);
    class_destroy(i2c_class);
    printk("i2c drivers removed.\n");
}


MODULE_AUTHOR("Sridharan Rajagopalan");
MODULE_DESCRIPTION("i2c_flash");
MODULE_LICENSE("GPL");
module_init(i2c_dev_init);
module_exit(i2c_dev_exit);