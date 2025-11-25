/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * < Your name here >
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>
#include <linux/string.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
/*
 * Declare a prototype so we can define the "unused" attribute and keep
 * the compiler happy. This function is not yet used, because this helpcode
 * is a stub.
 */
//static int __attribute__((unused)) lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *);
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct * state) {
    struct lunix_sensor_struct *sensor;
    struct lunix_msr_data_struct *msr_data;

    WARN_ON(!(sensor = state->sensor));
    msr_data = sensor->msr_data[state->type];
    WARN_ON(!msr_data);

    if (msr_data->last_update > state->buf_timestamp) return 1;
    else return 0;
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	
	debug("entering\n");
	pr_err("Entering update function\n");

	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	/* ? */
	/* Why use spinlocks? See LDD3, p. 119 */
    struct lunix_msr_data_struct *msr_data;
    sensor = state->sensor;
    WARN_ON(!sensor);
    
    spin_lock(&sensor->lock);
    msr_data = sensor->msr_data[state->type];
    uint16_t raw_values = msr_data->values[0];  //check for uint32_t
    spin_unlock(&sensor->lock);

	pr_err("I got the raw data\n");

	/*
	 * Any new data available?
	 */
	/* ? */
    if (!lunix_chrdev_state_needs_refresh(state)) {
        pr_err("No update needed\n");
        return -EAGAIN;
    }

	/*
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */
    /* ? */
    long int_info;
    switch (state->type) {
        case 0 : 
			int_info = lookup_voltage[raw_values];
			break;
        case 1 : 
			int_info = lookup_temperature[raw_values];
			break;
        case 2 : 
			int_info = lookup_light[raw_values];
			break; 
        default: 
			pr_err("invalid measurement type\n");
			break;
    }

    long integer_part = int_info / 1000;
    long decimal_part = int_info % 1000;
    char text_info[20];
    state->buf_lim = snprintf(text_info, sizeof(text_info), "%ld.%03ld", integer_part, decimal_part);  //check int long types

    int ret = down_interruptible(&state->lock);
	if (ret) return -ERESTARTSYS;

	strcpy(state->buf_data, text_info);

    /*for (int i=0; i<sizeof(text_info); i++) {	
		state->buf_data[i] = text_info[i];
		if (state->buf_data[i] == '\0') break;
	}*/

	pr_err("Data copied succesfully to buf_data\n");

    state->buf_timestamp = ktime_get_real_seconds();
    up(&state->lock);
	ret = 0;

	debug("leaving\n");
	pr_err("Update leaving\n");
	return ret;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	/* ? */
	int ret;

	debug("entering\n");
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
	unsigned int minor_num = iminor(inode);
	unsigned int mes_type = minor_num & 0x07;
	unsigned int sensor_num = (minor_num & ~(0x07) >> 3);
	
	/* Allocate a new Lunix character device private state structure */
	/* ? */
	struct lunix_chrdev_state_struct *private_state;
    private_state = kzalloc(sizeof(*private_state), GFP_KERNEL);
    private_state->type = mes_type;
    private_state->sensor = &lunix_sensors[sensor_num];  //check the index of the table
    filp->private_data = (void*)private_state;
out:
	debug("leaving, with ret = %d\n", ret);
	pr_err("Opened the file succesfully\n");
	return ret;
}


static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	pr_info("%s: Entering\n", __func__);
    filp->private_data = NULL;
	pr_info("%s: Leaving\n", __func__);
	return 0;
}


static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	//ssize_t ret;
	pr_err("read function start");

	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

	pr_err("After warns\n");

	/* Lock? */
	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement) do so
	 */
    //wait_queue_head_t *wq;
    struct lunix_msr_data_struct *msr_data;

    //wq = &sensor->wq;
    msr_data = sensor->msr_data[state->type];
	pr_err("Copied msr_data of sensor\n");
    
	if (*f_pos == 0) {
		while (lunix_chrdev_state_update(state) == -EAGAIN) {
			/* ? */
			/* The process needs to sleep */
			/* See LDD3, page 153 for a hint */
             wait_event_interruptible(sensor->wq, state->buf_timestamp < msr_data->last_update);
		}
	}

    char text[20]; 
	strcpy(text, state->buf_data);
	pr_err("buff_data copied to text\n");

    /*for (int i=0; i<20; i++){
        text[i] = state->buf_data[i];
		if (state->buf_data[i] == '\0') break;
    }*/
	/* End of file */
	/* ? */
    if (*f_pos >= state->buf_lim) {
        *f_pos = 0;
        return 0;
    }
	/* Determine the number of cached bytes to copy to userspace */
	/* ? */
    int not_copied, delta, to_copy = (*f_pos + cnt) < strlen(text) ? cnt : (strlen(text) - *f_pos);
    not_copied = copy_to_user(usrbuf, &text[*f_pos], to_copy);
    delta = to_copy - not_copied;
    if (not_copied) {
        pr_warn("We cannot copy %d number of chars\n", delta);
        return -EFAULT;
    }
	pr_err("read function end");

	*f_pos += delta;
	return delta; //check for ssize_t for ret

	/* Auto-rewind on EOF mode? */
	/* ? */
    //already made from us

	/*
	 * The next two lines  are just meant to suppress a compiler warning
	 * for the "unused" out: label, and for the uninitialized "ret" value.
	 * It's true, this helpcode is a stub, and doesn't use them properly.
	 * Remove them when you've started working on this code.
	 */
	//ret = -ENODEV;
	//goto out;
//out:
	/* Unlock? */
	//return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops = 
{
	.owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;
	
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	/* ? */
	/* register_chrdev_region? */
    ret = register_chrdev_region(dev_no, lunix_minor_cnt, "lunix");
   

	/* Since this code is a stub, exit early */
	//return 0;

	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}
	/* ? */
	/* cdev_add? */
    ret = cdev_add(&lunix_chrdev_cdev, dev_no, lunix_minor_cnt);
	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
