/* uio_cfi2_main:
	UIO driver to access the cfi2 fpga with multiple uart implementations
	-> this driver is to have access to main functionality
	-> to access the uarts you have to user further instances.

   Copyright (C) 2013 DH electronics GmbH
     Author: Ludwig Zenz <lzenz@dh-electronics.de>

   This is a straight-forward UIO driver, where interrupts are disabled
   by the interrupt handler and re-enabled via a write to the UIO device
   by the userspace-part.

   The only part that may seem odd is the use of a logical OR when
   storing and restoring enabled interrupts. This is done because the
   userspace-part could directly modify the Interrupt Enable Register
   at any time. To reduce possible conflicts, the kernel driver uses
   a logical OR to make more controlled changes (rather than blindly
   overwriting previous values).

   Race conditions exist if the userspace-part directly modifies the
   Interrupt Enable Register while in operation. The consequences are
   that certain interrupts would fail to be enabled or disabled. For
   this reason, the userspace-part should only directly modify the
   Interrupt Enable Register at the beginning (to get things going).
   The userspace-part can safely disable interrupts at any time using
   a write to the UIO device.
*/

#include <linux/platform_device.h>
#include <linux/uio_driver.h>
#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/stringify.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#include <linux/device.h>
#include <linux/io.h>
#include <linux/gpio.h>

#include <linux/workqueue.h>


#include <asm/io.h>
#include <linux/ioport.h>

#define DRIVER_NAME 	"uio_cfi2_main"

#define MYFPGA_BASE1    0x08000000
#define MYFPGA_SIZE     0x00020000 /* 16 bit address space */

#define ADDROFF_GENERAL 0x00000000
#define SIZE_GENERAL	0x00000200
#define ADDROFF_MISC	0x00000200
#define SIZE_MISC	0x00000600
#define ADDROFF_MUXFPGA	0x00000F00
#define SIZE_MUXFPGA	0x00000100


struct uio_platdata {
	struct uio_info *uioinfo;
	struct platform_device *pdev;
};

#ifndef CONFIG_OF
static struct resource cfi2_fpga_resources[] = {
    {
        .start = MYFPGA_BASE1 + ADDROFF_GENERAL,
        .end   = MYFPGA_BASE1 + ADDROFF_GENERAL + SIZE_GENERAL - 1,
        .name  = "cfi2_general",
        .flags = IORESOURCE_MEM
    },
    {
        .start = MYFPGA_BASE1 + ADDROFF_MISC,
        .end   = MYFPGA_BASE1 + ADDROFF_MISC + SIZE_MISC - 1,
        .name  = "cfi2_misc",
        .flags = IORESOURCE_MEM
    },
    {
        .start = MYFPGA_BASE1 + ADDROFF_MUXFPGA,
        .end   = MYFPGA_BASE1 + ADDROFF_MUXFPGA + SIZE_MUXFPGA - 1,
        .name  = "cfi2_mux",
        .flags = IORESOURCE_MEM
    },
};

static struct uio_info cfi2_fpga_uio_info = {
   .name = DRIVER_NAME,
   .version = "0.1",
   .irq = 0,
   .irq_flags = IRQF_TRIGGER_NONE,
};
#endif

static int uio_cfi2_fpga_probe(struct platform_device *pdev)
{
	struct uio_info *uioinfo = dev_get_platdata(&pdev->dev);
	struct uio_platdata *pdata;
	struct uio_mem *uiomem;
	int ret = -ENODEV;
	int i;

	if (pdev->dev.of_node) {
	
		/* alloc uioinfo for one device */
		uioinfo = devm_kzalloc(&pdev->dev, sizeof(*uioinfo),
				       GFP_KERNEL);
		if (!uioinfo) {
			dev_err(&pdev->dev, "unable to kmalloc\n");
			return -ENOMEM;
		}
		uioinfo->name = pdev->dev.of_node->name;
		dev_dbg(&pdev->dev, "name: %s\n", uioinfo->name);
		uioinfo->version = "devicetree";
		
		/* Multiple IRQs are not supported */
	}
	
	if (!uioinfo || !uioinfo->name || !uioinfo->version) {
		dev_dbg(&pdev->dev, "%s: err_uioinfo\n", __func__);
		return ret;
	}
        
	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev, "unable to kmalloc\n");
		return -ENOMEM;
	}

	pdata->uioinfo = uioinfo;
	pdata->pdev = pdev;

	uiomem = &uioinfo->mem[0];

	for (i = 0; i < pdev->num_resources; ++i) {
		struct resource *r = &pdev->resource[i];

		if (r->flags != IORESOURCE_MEM)
			continue;

		if (uiomem >= &uioinfo->mem[MAX_UIO_MAPS]) {
			dev_warn(&pdev->dev, "device has more than "
					__stringify(MAX_UIO_MAPS)
					" I/O memory resources.\n");
			break;
		}

		uiomem->memtype = UIO_MEM_PHYS;
		uiomem->addr = r->start;
		uiomem->size = resource_size(r);
		uiomem->name = r->name;
		
		dev_dbg(&pdev->dev, "uiomem setup: %s\n", uiomem->name);
		
		++uiomem;
	}

	while (uiomem < &uioinfo->mem[MAX_UIO_MAPS]) {
		uiomem->size = 0;
		++uiomem;
	}

	pdata->uioinfo->priv = pdata;

        pm_runtime_enable(&pdev->dev);

	ret = uio_register_device(&pdev->dev, pdata->uioinfo);
	if (ret) {
		kfree(pdata);
                dev_dbg(&pdev->dev, "uio_cfi2_fpga_probe() ERROR!\n");
                pm_runtime_disable(&pdev->dev);
		return ret;
	}

	platform_set_drvdata(pdev, pdata);
	
	dev_dbg(&pdev->dev, "uio_cfi2_fpga_probe() SUCCESS!\n");

	return 0;
}

static int uio_cfi2_fpga_remove(struct platform_device *pdev)
{
	struct uio_platdata *pdata = platform_get_drvdata(pdev);
	uio_unregister_device(pdata->uioinfo);
	kfree(pdata);
	return 0;
}

static int uio_cfi2_fpga_runtime_nop(struct device *dev)
{
	/* Runtime PM callback shared between ->runtime_suspend()
	 * and ->runtime_resume(). Simply returns success.
	 *
	 * In this driver pm_runtime_get_sync() and pm_runtime_put_sync()
	 * are used at open() and release() time. This allows the
	 * Runtime PM code to turn off power to the device while the
	 * device is unused, ie before open() and after release().
	 *
	 * This Runtime PM callback does not need to save or restore
	 * any registers since user space is responsbile for hardware
	 * register reinitialization after open().
	 */
	return 0;
}

static const struct dev_pm_ops uio_cfi2_fpga_dev_pm_ops = {
	.runtime_suspend = uio_cfi2_fpga_runtime_nop,
	.runtime_resume = uio_cfi2_fpga_runtime_nop,
};

#ifdef CONFIG_OF
static struct of_device_id uio_of_cfi2_fpga_match[] = {
	{ .compatible = "dh,cfi2-main" },
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, uio_of_cfi2_fpga_match);
module_param_string(of_id, uio_of_cfi2_fpga_match[0].compatible, 128, 0);
MODULE_PARM_DESC(of_id, "Openfirmware id of the device to be handled by uio");
#endif

static struct platform_driver uio_cfi2_fpga_driver = {
	 .probe = uio_cfi2_fpga_probe,
	 .remove = uio_cfi2_fpga_remove,
 	 .driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &uio_cfi2_fpga_dev_pm_ops,
		.of_match_table = of_match_ptr(uio_of_cfi2_fpga_match),
	},

 };
 
#ifndef CONFIG_OF
static struct platform_device *cfi2_fpga_uio_pdev;
#endif

static int __init cfi2_init_module(void)
{
#ifndef CONFIG_OF
    /* register platform device */
    cfi2_fpga_uio_pdev = platform_device_register_resndata (NULL,
                                                         DRIVER_NAME,
                                                         -1,
                                                         cfi2_fpga_resources,
                                                         3,
                                                         &cfi2_fpga_uio_info,
                                                         sizeof(struct uio_info)
                                                        );
    if (IS_ERR(cfi2_fpga_uio_pdev)) {
        return PTR_ERR(cfi2_fpga_uio_pdev);
    }
#endif

    return platform_driver_register(&uio_cfi2_fpga_driver);
}

static void __exit cfi2_exit_module(void)
{
#ifndef CONFIG_OF
    platform_device_unregister(cfi2_fpga_uio_pdev);
#endif
    platform_driver_unregister(&uio_cfi2_fpga_driver);
}

module_init(cfi2_init_module);
module_exit(cfi2_exit_module);

MODULE_DESCRIPTION("UIO driver for the CFI2 FPGA MAIN");
MODULE_AUTHOR("Ludwig Zenz <lzenz@dh-electronics.de>");
MODULE_LICENSE("GPL v2");

