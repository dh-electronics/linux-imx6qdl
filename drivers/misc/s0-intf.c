/*
 * s0-intf.c - S0 Interface defined in EN 62053-31. For receiving of consumption
 *             measured values in building automation. The interface should not
 *             be confused with the S0 bus of ISDN.
 *
 * Copyright (C) 2020 Ludwig Zenz <lzenz@dh-electronics.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/circ_buf.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>

#include <linux/of.h>
#include <linux/of_irq.h>


static dev_t s0intf_devt;        /* the first major/minor numbers of char dev */

#define S0INTF_DRV_NAME "s0intf"
#define S0INTF_DRV_VERSION "0v01"
#define S0INTF_DRV_MAX_DEV 8
#define S0INTF_DRV_FIRST_MINOR 0

/* function codes */
#define FC_CNT_START   0x10   /* start counting pulses, init at 0 */
#define FC_CNT_RESET   0x11   /* reset counter to 0 */
#define FC_CNT_READ    0x20   /* read current pulses count */

#define S0_IOCTL_MAGIC 0xE0   /* magic number for IOCTL */

struct s0intf_ioctl_context {
	int counter;             /* current impulse counter value*/
};

#define S0_IOCTL_CNT_START  _IO(S0_IOCTL_MAGIC, FC_CNT_START)
#define S0_IOCTL_CNT_RESET  _IOR(S0_IOCTL_MAGIC, FC_CNT_RESET, sizeof(struct s0intf_ioctl_context))
#define S0_IOCTL_CNT_READ   _IOR(S0_IOCTL_MAGIC, FC_CNT_READ, sizeof(struct s0intf_ioctl_context))

#define S0_CIRCBUFF_LEN 1024  /* x timestamps */

/* Bit definition of flags */
#define FLAG_S0INTF_ENABLED   0
#define FLAG_S0INTF_OVERFLOW  1

static struct class *s0intf_class;

struct s0_circ_buf {
	struct timespec64 * data;
	int head;
	int tail;
};

struct s0_device {
	struct platform_device *pdev;
	struct cdev cdev;
	int minor;
	char *name;

	int irq;
	unsigned long flags;

	/* use workqueue */
	struct work_struct work;

	/* waitqueue to wake up blocking read when data is available */
	wait_queue_head_t wait;
	bool data_avail; /* if circ_buf has data */

	atomic_t counter; /* count incoming pulses */
	struct s0_circ_buf circ_buf; /* circular buffer for timestamps */

	struct spinlock producer_lock;
	struct spinlock consumer_lock;
};

static struct workqueue_struct *s0intf_wq;

/* device attributes **********************************************************/

static ssize_t
attr_driver_version_show(struct device *device,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "s0intf driver version %s\n", S0INTF_DRV_VERSION);
}
static DEVICE_ATTR(driver_version, S_IRUGO, attr_driver_version_show, NULL);


static ssize_t
attr_counter_show(struct device *device,
				struct device_attribute *attr, char *buf)
{
	struct s0_device *sdev = dev_get_drvdata(device);

	return sprintf(buf, "counter: %u\n", atomic_read(&sdev->counter));
}
static DEVICE_ATTR(counter, S_IRUGO, attr_counter_show, NULL);


static ssize_t
attr_overflow_show(struct device *device,
				struct device_attribute *attr, char *buf)
{
	struct s0_device *sdev = dev_get_drvdata(device);

	return sprintf(buf, "overflow: %u\n", atomic_read(&sdev->counter));
}
static DEVICE_ATTR(overflow, S_IRUGO, attr_overflow_show, NULL);


static struct attribute *dev_attrs[] = {
	&dev_attr_overflow.attr,
	&dev_attr_counter.attr,
	&dev_attr_driver_version.attr,
	NULL,
};

static struct attribute_group dev_attr_group = {
	.attrs = dev_attrs,
};

static const struct attribute_group *dev_attr_groups[] = {
	&dev_attr_group,
	NULL,
};

/****************************************************************++************/

/*
 * dump oldest timestamps
 * - delete the 10 oldest timestamps
 * - do nothing if at least one timestamp can be stored
 */
static inline void
dump_oldest_timestamp(struct s0_device *sdev, unsigned long head, unsigned long tail)
{
	unsigned long flags;
	unsigned long rhead, rtail;
	struct  s0_circ_buf *cbuf = &sdev->circ_buf;

	/* check for empty space and return if any */
	if (likely(CIRC_SPACE(head, tail, S0_CIRCBUFF_LEN) >= 1))
		return;

	spin_lock_irqsave(&sdev->consumer_lock, flags);
	{
		rhead = smp_load_acquire(&cbuf->head);
		rtail = cbuf->tail;

		smp_store_release(&cbuf->tail,
				(rtail + 10) & (S0_CIRCBUFF_LEN - 1));
	}
	spin_unlock_irqrestore(&sdev->consumer_lock, flags);
}


/*
 * workqueue handler to wake up reader
 * - the userspace wakeup sometimes took too long for the irq routine.
 * - that is the reason to use a workqueue at all.
 */
static void s0intf_worker(struct work_struct *work)
{
	struct s0_device *sdev = container_of(work, struct s0_device, work);

	sdev->data_avail = true;
	wake_up(&sdev->wait);
}

/*
 * interrupt handler
 * - a minimum pulse length of 10 ms can be detected.
 */
static irqreturn_t s0intf_irq_handler(int irq, void *dev_id)
{
	struct s0_device *sdev = (struct s0_device *)dev_id;
	struct s0_circ_buf *cbuf = &sdev->circ_buf;
	struct timespec64 ts_temp;
	unsigned long flags;
	unsigned long head, tail;

	if (!test_bit(FLAG_S0INTF_ENABLED, &sdev->flags))
		return IRQ_HANDLED;

	/* get timestamp before messing with spinlocks or atomic operations */
	ktime_get_ts64(&ts_temp);

	atomic_inc(&sdev->counter);
	if (atomic_read(&sdev->counter) < 0) {
		// handle overflow - extremely rare 
		set_bit(FLAG_S0INTF_OVERFLOW, &sdev->flags);
		atomic_set(&sdev->counter, 0);
	}

	spin_lock_irqsave(&sdev->producer_lock, flags);
	{
		head = cbuf->head;
		tail = READ_ONCE(cbuf->tail);
		dump_oldest_timestamp(sdev, head, tail);
		cbuf->data[head] = ts_temp; /* store new timestamp */
		smp_store_release(&cbuf->head, (head+1) & (S0_CIRCBUFF_LEN-1));
	}
	spin_unlock_irqrestore(&sdev->producer_lock, flags);

	queue_work(s0intf_wq, &sdev->work);

	return IRQ_HANDLED;
}

/* file operations - app interface ********************************************/

/**
 * s0intf_write - the write function.
 *
 * @file: pointer to file structure
 * @ubuf: pointer to user buffer
 * @length: buffer length
 * @offset: data offset in buffer
 *
 * Return: >=0 data length on success , <0 on error
 */
ssize_t s0intf_write(struct file *filep, const char *buffer, size_t len,
		loff_t *offset)
{
	struct s0_device *sdev = filep->private_data;
	if(sdev == NULL)
		return -ENODATA;

	dev_info(&sdev->pdev->dev, "write() with no features!\n");
	return 0;
}

/**
 * s0intf_read - the read function.
 *
 * @file: pointer to file structure
 * @ubuf: pointer to user buffer
 * @length: buffer length
 * @offset: data offset in buffer
 *
 * Return: >=0 data length on success , <0 on error
 */
ssize_t s0intf_read(struct file *filep, char __user *buffer,
		size_t len, loff_t *offset)
{
	unsigned long flags;
	unsigned long head, tail;
	struct  s0_circ_buf *cbuf;
	int count_ts = 0;
	int avail_ts = 0;
	size_t len1 = 0, len2 = 0;
	size_t len_ts;

	struct s0_device *sdev = filep->private_data;
	if (sdev == NULL)
		return -ENODATA;

	cbuf = &sdev->circ_buf;
	if (cbuf->data == NULL)
		return -ENODATA;

	/* calculate count of timestamps which fit into user buffer */
	len_ts = sizeof(cbuf->data[0]);
	count_ts = len / len_ts;
	head = smp_load_acquire(&cbuf->head);
	tail = cbuf->tail;

	/* check for available data */
	if (CIRC_CNT(head, tail, S0_CIRCBUFF_LEN) < 1) {
		sdev->data_avail = false; /* wait for data */
		wait_event_interruptible(sdev->wait, sdev->data_avail);
	}

	spin_lock_irqsave(&sdev->consumer_lock, flags);
	{
		head = smp_load_acquire(&cbuf->head);
		tail = cbuf->tail;

		avail_ts = CIRC_CNT(head, tail, S0_CIRCBUFF_LEN);
		if (avail_ts < count_ts)
			count_ts = avail_ts;

		if (count_ts >= 1) {
			/* check for circ buffer flip around */
			if (tail + count_ts >= S0_CIRCBUFF_LEN) {
				len1 = (S0_CIRCBUFF_LEN - tail - 1) * len_ts;
				len2 = (count_ts - (S0_CIRCBUFF_LEN - tail - 1)) * len_ts;
			} else {
				len1 = count_ts * len_ts;
				len2 = 0;
			}

			if (copy_to_user(buffer, &cbuf->data[tail], len1)) {
				dev_err(&sdev->pdev->dev, "copy_to_user() failed\n");
				return -EFAULT;
			}
			if (len2 != 0) {
				if (copy_to_user(buffer + len1, &cbuf->data[0], len2)) {
					dev_err(&sdev->pdev->dev, "copy_to_user() failed\n");
					return -EFAULT;
				}
			}
			/* finish read descriptor before incrementing tail. */
			smp_store_release(&cbuf->tail,
				(tail + count_ts) & (S0_CIRCBUFF_LEN - 1));
		}
	}
	spin_unlock_irqrestore(&sdev->consumer_lock, flags);
	return len1 + len2;
}

/**
 * s0intf_ioctl - the IOCTL function
 *
 * @file: pointer to file structure
 * @cmd: ioctl command
 * @data: pointer to mei message structure
 *
 * Return: 0 on success , <0 on error
 */
static long s0intf_ioctl(struct file *filep, unsigned int cmd,
							unsigned long arg)
{
	int status = 0;
	int temp_counter;
	int err;

	struct s0_device *sdev = filep->private_data;
	if (sdev == NULL)
		return -ENODATA;

	if (_IOC_TYPE(cmd) != S0_IOCTL_MAGIC) {
		dev_err(&sdev->pdev->dev, "S0_IOCTL_MAGIC not recognized!\n");
		return -EINVAL;
	}

	switch (_IOC_NR(cmd)) {
	case FC_CNT_START:
	{
		if (test_and_set_bit(FLAG_S0INTF_ENABLED, &sdev->flags))
			dev_info(&sdev->pdev->dev, "already enabled\n");
		break;
	}
	case FC_CNT_RESET:
	{
		/* read current pulses count and reset counter afterwards */
		struct s0intf_ioctl_context data_;

		// Check if the received value points into user-space
		if (!access_ok(VERIFY_WRITE, (struct s0intf_ioctl_context*)arg,
							sizeof(data_))) {
			dev_err(&sdev->pdev->dev, "ioctl context invalid!\n");
			return -EFAULT;
		}

		temp_counter = atomic_read(&sdev->counter);
		if (!atomic_sub_and_test(temp_counter, &sdev->counter)) {
			/* if atomic_sub_and_test result != 1,
			 * than counter is not 0 (isr occoured) */
			dev_warn(&sdev->pdev->dev,"irq during coutner reset\n");
		}

		data_.counter = temp_counter;

		err = copy_to_user((struct s0intf_ioctl_context*)arg,
							&data_, sizeof(data_));
		if (err < 0) {
			dev_err(&sdev->pdev->dev, "copy_to_user failed!\n");
			return -EFAULT;
		}
		break;
	}
	case FC_CNT_READ:
	{	/* read current pulses count */
		struct s0intf_ioctl_context data_;

		// Check if the received value points into user-space
		if (!access_ok(VERIFY_WRITE,
		(struct s0intf_ioctl_context*)arg, sizeof(data_))) {
			dev_err(&sdev->pdev->dev, "ioctl context invalid!\n");
			return -EFAULT;
		}

		data_.counter = atomic_read(&sdev->counter);

		err = copy_to_user((struct s0intf_ioctl_context*)arg,
							&data_, sizeof(data_));
		if (err < 0) {
			dev_err(&sdev->pdev->dev, "copy_to_user failed!\n");
			return -EFAULT;
		}

		break;
	}
	default:
		dev_err(&sdev->pdev->dev, "unknown ioctl cmd!!\n");
		break;
	}

	return status;
}

/**
 * s0intf_open - the open function
 *
 * @inode: pointer to inode structure
 * @file: pointer to file structure
 *
 * Return: 0 on success, <0 on error
 */
int s0intf_open(struct inode *inode, struct file *filep)
{
	struct s0_device *sdev;

	sdev = container_of(inode->i_cdev, struct s0_device, cdev);
	if(sdev == NULL)
		return -ENODATA;

	filep->private_data = (struct device*)sdev;
	return 0;
}

/**
 * s0intf_release - the release function
 *
 * @inode: pointer to inode structure
 * @file: pointer to file structure
 *
 * Return: 0 on success, <0 on error
 */
int s0intf_release(struct inode *inode, struct file *filep)
{	
	struct s0_device *sdev;

	sdev = container_of(inode->i_cdev, struct s0_device, cdev);
	if(sdev == NULL)
		return -ENODATA;

	filep->private_data = NULL;
	return 0;
}

static struct file_operations s0intf_fops = {
	.owner = THIS_MODULE,
	.open = s0intf_open,
	.release = s0intf_release,
	.read = s0intf_read,
	.write = s0intf_write,
	.unlocked_ioctl = s0intf_ioctl,
};

/****************************************************************************/

/*
 * init the circular buffer to store timestamps until the
 * userspace does read them
 */
static int init_circ_buf(struct s0_device *sdev, struct device_node *node)
{
	struct s0_circ_buf *circ = &sdev->circ_buf;

	/* init circular buffer */
	circ->head = 0;
	circ->tail = 0;

	circ->data = devm_kzalloc(&sdev->pdev->dev,
			S0_CIRCBUFF_LEN * sizeof(circ->data[0]), GFP_KERNEL);
	if(!circ->data) {
		dev_err(&sdev->pdev->dev, "failed to alloc mem circ buffer\n");
		return -ENOMEM;
	}

	memset(circ->data, 0, S0_CIRCBUFF_LEN * sizeof(circ->data[0]));
	return 0;
}

static int s0intf_probe(struct platform_device *pdev)
{
	dev_t devno;
	struct device *clsdev; /* class device */
	struct device_node *node;
	struct s0_device *sdev;
	const u32 *id;
	int size;
	int ret = 0;
	
	/* resource managed kzalloc */
	sdev = devm_kzalloc(&pdev->dev, sizeof(*sdev), GFP_KERNEL);
	if(!sdev)
		return -ENOMEM;

	sdev->pdev = pdev;

	node = pdev->dev.of_node;
	if (node == NULL)
		return -ENODEV;

	/* read id from devicetree - use it for minor id */
	id = of_get_property(node, "id", &size);
	if (id == NULL) {
		dev_err(&pdev->dev, "failed to get s0intf device id\n");
		return -ENODEV;
	}
	sdev->minor = of_read_number(id, 1);

	sdev->name = (char*)kmalloc((3+strlen(node->name)), GFP_KERNEL);
	if(!sdev->name){
		dev_err(&pdev->dev, "failed to allocate memory for dev name\n");
		return -ENOMEM;
	}
	sprintf(sdev->name, "%s", node->name);

	spin_lock_init(&sdev->producer_lock);
	spin_lock_init(&sdev->consumer_lock);

	ret = init_circ_buf(sdev, node);
	if (ret < 0) {
		goto free_dev_name;
	}

	/* init waitqueue for read() file-op */
	init_waitqueue_head(&sdev->wait);
	sdev->data_avail = false;

	atomic_set(&sdev->counter, 0);
	sdev->flags = 0;

	INIT_WORK(&sdev->work, s0intf_worker);

	sdev->irq = irq_of_parse_and_map(node, 0);
	ret = devm_request_irq(&pdev->dev, sdev->irq, s0intf_irq_handler,
				0, sdev->name, (void*)sdev);
	if(ret < 0){
		dev_err(&pdev->dev, "request_irq failed for %s\n", sdev->name);
		ret = -ENOMEM;
		goto free_dev_name;
	}

	devno = MKDEV(MAJOR(s0intf_devt), sdev->minor);
	cdev_init(&sdev->cdev, &s0intf_fops);
	sdev->cdev.owner = THIS_MODULE;

	/* Add the device */
	ret = cdev_add(&sdev->cdev, devno, 1);
	if (ret) {
		dev_err(&pdev->dev, "unable to add device %d:%d\n",
			MAJOR(s0intf_devt), sdev->minor);
		goto free_dev_name;
	}

	clsdev = device_create_with_groups(s0intf_class, pdev->dev.parent,
					   devno, sdev, dev_attr_groups,
					   "s0intf%d", sdev->minor);
	if (IS_ERR(clsdev)) {
		dev_err(&pdev->dev, "unable to create device %d:%d\n",
					MAJOR(s0intf_devt), sdev->minor);
		ret = IS_ERR(clsdev);
		goto err_dev_create;
	}

	platform_set_drvdata(pdev, sdev);
	dev_info(&pdev->dev, "probed\n");
	return 0;

err_dev_create:
	cdev_del(&sdev->cdev);
free_dev_name:
	kfree(sdev->name);
	return ret;
}

static int s0intf_remove(struct platform_device *pdev)
{
	int devno;
	struct s0_device *sdev;

	sdev = platform_get_drvdata(pdev);
	devno = sdev->cdev.dev;

	dev_info(&sdev->pdev->dev, "removing %s\n", sdev->name);

	cdev_del(&sdev->cdev);
	device_destroy(s0intf_class, devno);
	kfree(sdev->name);
	return 0;
}

static struct of_device_id s0intf_of_match[] = {
	{ .compatible = "dh,s0-intf", },
	{ },
};
MODULE_DEVICE_TABLE(of, s0intf_of_match);

static struct platform_driver s0intf_driver = {
	.probe = s0intf_probe,
	.remove = s0intf_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = S0INTF_DRV_NAME,
		.of_match_table = s0intf_of_match,
	},
};

static int __init s0intf_module_init(void)
{
	int ret;

	pr_info("s0intf: init entered\n");

	s0intf_class = class_create(THIS_MODULE, "s0intf");
	if (IS_ERR(s0intf_class)) {
		pr_err("s0intf: couldn't create class\n");
		ret = PTR_ERR(s0intf_class);
		goto err;
	}

	ret = alloc_chrdev_region(&s0intf_devt,
					S0INTF_DRV_FIRST_MINOR,
					S0INTF_DRV_MAX_DEV,
					S0INTF_DRV_NAME);
	if (ret) {
		pr_err("s0intf: unable to allocate char dev region\n");
		goto err_class;
	}

	ret = platform_driver_register(&s0intf_driver);
	if (ret < 0) {
		pr_err("Failed to register platform driver s0intf\n");
		goto err_chrdev;
	}

	/* allocate high priority workqueue */
	s0intf_wq = alloc_workqueue(S0INTF_DRV_NAME, WQ_SYSFS, 0);
	if (!s0intf_wq) {
		ret = -ENOMEM;
		goto err_workqueue;
	}

	pr_info("s0intf: init done\n");
	return ret;

err_workqueue:
	platform_driver_unregister(&s0intf_driver);
err_chrdev:
	unregister_chrdev_region(s0intf_devt, S0INTF_DRV_MAX_DEV);
err_class:
	class_destroy(s0intf_class);
err:
	return ret;
}

module_init(s0intf_module_init);
/* Remark: No module exit -> support only static linked kernel module */

MODULE_AUTHOR("Ludwig Zenz <lzenz@dh-electronics.com>");
MODULE_DESCRIPTION("S0 Interface - counting pulses for metering purpose");
MODULE_VERSION(S0INTF_DRV_VERSION);
MODULE_LICENSE("GPL");
