// reference: spidev.c

/* ----------------------------------------------- LED DRIVER - SPI -------------------------------------------------- */

#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/uaccess.h>

#define SPIDEV_MAJOR                    153     /* assigned */
#define N_SPI_MINORS                    32      /* ... up to 256 */

#define SPI_PATTERN     64

#define SPI_MOSI_MUX 43
#define SPI_CLK_MUX 42
#define SPI_CS_MUX 55

#define STOP_DISP 247

static DECLARE_BITMAP(minors, N_SPI_MINORS);

#define SPI_MODE_MASK           (SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
                                | SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
                                | SPI_NO_CS | SPI_READY | SPI_TX_DUAL \
                                | SPI_TX_QUAD | SPI_RX_DUAL | SPI_RX_QUAD)

struct spidev_data {
        dev_t                   devt;
        spinlock_t              spi_lock;
        struct spi_device       *spi;
        struct list_head        device_entry;
        struct mutex            buf_lock;
        unsigned                users;
        u8                      *buffer;
};

struct spidev_data      *spidevg;
struct                  spi_device *spiput;
struct file *filpg;
spinlock_t mut;
int breakdisp, exitthread;

int tf;

uint16_t pattern[20][8];
long order[10];
long spd = 1;

static struct task_struct *tsk; 


static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 4096;

module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

/*-------------------------------------------------------------------------*/

static int thread_function(void *data);

/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void spidev_complete(void *arg)
{
        complete(arg);
}

static ssize_t
spidev_sync(struct spidev_data *spidev, struct spi_message *message)
{
        DECLARE_COMPLETION_ONSTACK(done);
        int status;

        message->complete = spidev_complete;
        message->context = &done;

        spin_lock_irq(&spidev->spi_lock);
        if (spidev->spi == NULL)
                status = -ESHUTDOWN;
        else
                status = spi_async(spidev->spi, message);
        spin_unlock_irq(&spidev->spi_lock);

        if (status == 0) {
                wait_for_completion(&done);
                status = message->status;
                if (status == 0)
                        status = message->actual_length;
        }
        return status;
}

static inline ssize_t
spidev_sync_write(struct spidev_data *spidev, size_t len)
{
        struct spi_transfer     t = {
                        .tx_buf         = spidev->buffer,
                        .len            = len,
                };
        struct spi_message      m;

        spi_message_init(&m);
        spi_message_add_tail(&t, &m);
        return spidev_sync(spidev, &m);
}

static inline ssize_t
spidev_sync_read(struct spidev_data *spidev, size_t len)
{
        struct spi_transfer     t = {
                        .rx_buf         = spidev->buffer,
                        .len            = len,
                };
        struct spi_message      m;

        spi_message_init(&m);
        spi_message_add_tail(&t, &m);
        return spidev_sync(spidev, &m);
}

/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup */
static ssize_t
spidev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
        struct spidev_data      *spidev;
        ssize_t                 status = 0;

        /* chipselect only toggles at start or end of operation */
        if (count > bufsiz)
                return -EMSGSIZE;

        spidev = filp->private_data;

        mutex_lock(&spidev->buf_lock);
        status = spidev_sync_read(spidev, count);
        if (status > 0) {
                unsigned long   missing;

                missing = copy_to_user(buf, spidev->buffer, status);
                if (missing == status)
                        status = -EFAULT;
                else
                        status = status - missing;
        }
        mutex_unlock(&spidev->buf_lock);

        return status;
}

// function to write a data on to the display
void writefunc(uint16_t tx)
{
        int retval = 0;

        mutex_lock(&spidevg->buf_lock);
        
        memcpy(spidevg->buffer, &tx, 2);
        
        retval = spidev_sync_write(spidevg, 2);
        
        if(retval < 0)
            printk("SPI MSG SEND FAIL");

        mutex_unlock(&spidevg->buf_lock);
}


/* Write-only message with current device setup */
static ssize_t
spidev_write(struct file *filp, const char __user *buf,
                size_t count, loff_t *f_pos)
{
        int missing;
        int o1, o2;
        spin_lock_irq(&mut);
        missing = copy_from_user(order, buf, sizeof(order));
        o1 = order[0];
        o2 = order[1];
        spin_unlock_irq(&mut);
        if(missing != 0)
        {
            printk("ERROR copying oder from use\n");
            return -1;
        }
        if(o1 == 0 && o2 == 0)
        {
            writefunc(0x0c00);
            breakdisp = 0;
        }
        else
            breakdisp = 1;
        if(tf == 0)
        {
            tsk = kthread_run(thread_function, NULL, "spi thread %d", 1); 
            tf = 1;
        }
        

        return 0;
}

static long
spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{

  
        int ret;
        switch (cmd) 
        {
            // ioctl command for receiving the pattern
            case SPI_PATTERN:
                ret = copy_from_user(pattern, (char *)arg, sizeof(pattern));
                
                if(ret != 0)
                {
                    printk(" pattern copy from user error");
                    return -1;
                }
                break;
            // command to terminate the display
            default:
                break;
        }
        return 0;
}

#ifdef CONFIG_COMPAT
static long
spidev_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
        return spidev_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define spidev_compat_ioctl NULL
#endif /* CONFIG_COMPAT */


// kernel thread function
static int thread_function(void *data)
{
    int i = 0, j = 0; 
    long prev = -1, pres = -1, next = -1;
    while(1)
    {
        if(breakdisp == 1)
        {   
            
            for( i = 0; ; i+=2)
            {
                spin_lock_irq(&mut);

                prev = pres;
                pres = order[i];
                next = order[i+1];

                if(pres == 0 && next == 0)
                    break;

                j = 0;
                while(j < 8)
                {
                    writefunc(pattern[ order[i] ][j]);
                    j++;
                }
                if(breakdisp == 0)
                {
                    msleep(10);
                    spin_unlock_irq(&mut);
                    break;
                }
                msleep((order[i+1]));

                spin_unlock_irq(&mut);
            }
            
            if(breakdisp == 0)
            {
                msleep(10);
                break;
            }
        }
        else
            break;
    }
    tf = 0;
    return 0;
}

// Initialize the display

void icinit(struct spidev_data *spidev,  struct file *filp)
{
    int                     retval = 0;
    struct                  spi_device *spi;

    // spi initialization setup
    spin_lock_irq(&spidev->spi_lock);
    spi = spi_dev_get(spidev->spi);
    spiput = spi;
    spin_unlock_irq(&spidev->spi_lock);
    
    spi->mode = 0;
    spi->bits_per_word = 16;
    spi->max_speed_hz = 10000000;
    retval = spi_setup(spi);
    if(retval < 0)
    {
        printk("ERROR SPI INITIALIZATION\n\n");
    }
    
    //ic initialization

    writefunc(0x0900);
    writefunc(0x0A0A);
    writefunc(0x0B07);
    writefunc(0x0F00);
    writefunc(0x0C01);

}

// open function

static int spidev_open(struct inode *inode, struct file *filp)
{
    
    int                     status = -ENXIO;

    struct spidev_data      *spidev;        
        mutex_lock(&device_list_lock);

        list_for_each_entry(spidev, &device_list, device_entry) {
                if (spidev->devt == inode->i_rdev) {
                        status = 0;
                        break;
                }
        }
        if (status == 0) 
        {
                if (!spidev->buffer) {
                        spidev->buffer = kmalloc(bufsiz, GFP_KERNEL);
                        if (!spidev->buffer) {
                                dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
                                status = -ENOMEM;
                        }

                }
                
                spidev->users++;
                filp->private_data = spidev;
                nonseekable_open(inode, filp);
                mutex_unlock(&device_list_lock);
                
                // init ic init

                spidevg = spidev;
                filpg = filp;
                icinit(spidev, filp);
                
        } 
        else
        {
                printk("spidev: nothing for minor %d\n", iminor(inode));
                mutex_unlock(&device_list_lock);
        }
        return status;             
}

static int spidev_release(struct inode *inode, struct file *filp)
{
        struct spidev_data      *spidev;
        int                     status = 0;

        mutex_lock(&device_list_lock);
        spidev = filp->private_data;
        filp->private_data = NULL;

        /* last close? */
        spidev->users--;
        if (!spidev->users) {
                int             dofree;

                kfree(spidev->buffer);
                spidev->buffer = NULL;

                /* ... after we unbound from the underlying device? */
                spin_lock_irq(&spidev->spi_lock);
                dofree = (spidev->spi == NULL);
                spin_unlock_irq(&spidev->spi_lock);

                if (dofree)
                        kfree(spidev);
        }
        mutex_unlock(&device_list_lock);



        return status;
}

static const struct file_operations spidev_fops = {
        .owner =        THIS_MODULE,
        /* REVISIT switch to aio primitives, so that userspace
         * gets more complete API coverage.  It'll simplify things
         * too, except for the locking.
         */
        .write =        spidev_write,
        .read =         spidev_read,
        .unlocked_ioctl = spidev_ioctl,
        .compat_ioctl = spidev_compat_ioctl,
        .open =         spidev_open,
        .release =      spidev_release,
        .llseek =       no_llseek,
};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *spidev_class;

/*-------------------------------------------------------------------------*/

static int spidev_probe(struct spi_device *spi)
{
        struct spidev_data      *spidev;
        int                     status;
        unsigned long           minor;

        /* Allocate driver data */
        spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
        if (!spidev)
                return -ENOMEM;

        /* Initialize the driver data */
        spidev->spi = spi;
        spin_lock_init(&spidev->spi_lock);
        mutex_init(&spidev->buf_lock);

        INIT_LIST_HEAD(&spidev->device_entry);

        /* If we can allocate a minor number, hook up this device.
         * Reusing minors is fine so long as udev or mdev is working.
         */
        mutex_lock(&device_list_lock);
        minor = find_first_zero_bit(minors, N_SPI_MINORS);
        if (minor < N_SPI_MINORS) {
                struct device *dev;

                spidev->devt = MKDEV(SPIDEV_MAJOR, minor);
                dev = device_create(spidev_class, &spi->dev, spidev->devt,
                                    spidev, "spi_led");
                status = PTR_RET(dev);
        } else {
                dev_dbg(&spi->dev, "no minor number available!\n");
                status = -ENODEV;
        }
        if (status == 0) {
                set_bit(minor, minors);
                list_add(&spidev->device_entry, &device_list);
        }
        mutex_unlock(&device_list_lock);

        if (status == 0)
                spi_set_drvdata(spi, spidev);
        else
                kfree(spidev);

        return status;
}

static int spidev_remove(struct spi_device *spi)
{
        struct spidev_data      *spidev = spi_get_drvdata(spi);

        /* make sure ops on existing fds can abort cleanly */
        spin_lock_irq(&spidev->spi_lock);
        spidev->spi = NULL;
        spin_unlock_irq(&spidev->spi_lock);

        /* prevent new opens */
        mutex_lock(&device_list_lock);
        list_del(&spidev->device_entry);
        device_destroy(spidev_class, spidev->devt);
        clear_bit(MINOR(spidev->devt), minors);
        if (spidev->users == 0)
                kfree(spidev);
        mutex_unlock(&device_list_lock);

        return 0;
}

static const struct of_device_id spidev_dt_ids[] = {
        { .compatible = "rohm,dh2228fv" },
        {},
};

MODULE_DEVICE_TABLE(of, spidev_dt_ids);

static struct spi_driver spidev_spi_driver = {
        .driver = {
                .name =         "spidev",
                .owner =        THIS_MODULE,
                .of_match_table = of_match_ptr(spidev_dt_ids),
        },
        .probe =        spidev_probe,
        .remove =       spidev_remove,

        /* NOTE:  suspend/resume methods are not necessary here.
         * We don't do anything except pass the requests to/from
         * the underlying controller.  The refrigerator handles
         * most issues; the controller driver handles the rest.
         */
};

/*-------------------------------------------------------------------------*/

static int __init spidev_init(void)
{
        int status;



        /* Claim our 256 reserved device numbers.  Then register a class
         * that will key udev/mdev to add/remove /dev nodes.  Last, register
         * the driver which manages those device numbers.
         */
        BUILD_BUG_ON(N_SPI_MINORS > 256);
        status = register_chrdev(SPIDEV_MAJOR, "spi_led", &spidev_fops);
        if (status < 0)
                return status;

        spidev_class = class_create(THIS_MODULE, "spi_led");
        if (IS_ERR(spidev_class)) {
                unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
                return PTR_ERR(spidev_class);
        }

        status = spi_register_driver(&spidev_spi_driver);
        if (status < 0) {
                class_destroy(spidev_class);
                unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
        }

        // initializing gpio
        
        gpio_request(SPI_MOSI_MUX,"SPI_MOSI_MUX");
        gpio_direction_output(SPI_MOSI_MUX,0);
        gpio_set_value_cansleep(SPI_MOSI_MUX,0);

        gpio_request(SPI_CS_MUX,"SPI_CS_MUX");
        gpio_direction_output(SPI_CS_MUX,0);
        gpio_set_value_cansleep(SPI_CS_MUX,0);

        gpio_request(SPI_CLK_MUX,"SPI_CLK_MUX");
        gpio_direction_output(SPI_CLK_MUX,0);
        gpio_set_value_cansleep(SPI_CLK_MUX,0);

        

        printk("LED driver inserted\n");
        

        return status;
}
module_init(spidev_init);

static void __exit spidev_exit(void)
{
        gpio_free(SPI_MOSI_MUX);
        gpio_free(SPI_CS_MUX);
        gpio_free(SPI_CLK_MUX);
        spi_dev_put(spiput);
        spi_unregister_driver(&spidev_spi_driver);
        class_destroy(spidev_class);
        unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
        printk("LED driver removed\n");
}
module_exit(spidev_exit);

MODULE_AUTHOR("Sridharan Rajagopalan");
MODULE_DESCRIPTION("LED driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spidev");