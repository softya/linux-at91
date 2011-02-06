/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2008 - 2011 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * The full GNU General Public License is included in this distribution
 * in the file called LICENSE.GPL.
 *
 * BSD LICENSE
 *
 * Copyright(c) 2008 - 2011 Intel Corporation. All rights reserved.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * This file contains the isci_module initialization routines.
 *
 * isci_module_init.c
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <asm/string.h>
#include "isci_module.h"
#include "isci_task.h"
#include "sci_controller_constants.h"
#include "intel_pci.h"

MODULE_LICENSE("Dual BSD/GPL");


static int lldd_max_execute_num = 64;

static struct isci_module isci_module_struct = {

	.controller_count	= 0,
	.pci_devices		= LIST_HEAD_INIT(isci_module_struct
						 .pci_devices),
	.timer_list_struct	= {
		.timers		= LIST_HEAD_INIT(
			isci_module_struct.timer_list_struct
			.timers),
	},
	.stt			= NULL,

};

#if defined(PBG_HBA_A0_BUILD) || defined(PBG_HBA_A2_BUILD)
/*
 * For the FPGA and all of the A-Step Silicon, the chip package could
 * contain up to two SCU controllers (SCU-0 & SCU-1). Each SCU
 * controller is presented as its own PCI function.  Therefore, the
 * the last data field (driver_data) in each structure below
 * indicates the controller index 0=SCU-0, 1=SCU-1 of the package.
 */
static DEFINE_PCI_DEVICE_TABLE(isci_id_table) = {
	/* Arlington FPGA device ID. */
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x3318), 0, 0, 0 },

	/* Pleasant Ridge/Patsburg SCU-0 Device IDs. */
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x1D60), 0, 0, 0 },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x1D62), 0, 0, 0 },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x1D64), 0, 0, 0 },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x1D66), 0, 0, 0 },

	/* Pleasant Ridge/Patsburg SCU-1 Device IDs. */
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x1D61), 0, 0, 1 },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x1D63), 0, 0, 1 },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x1D65), 0, 0, 1 },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x1D67), 0, 0, 1 },
	{}
};
#elif defined(PBG_HBA_BETA_BUILD)
/*
 *  For B-step the silicon, the definition of the driver_data field
 *  changed. B-step silicon now presents only one PCI function, but
 *  depending on the Device ID of the part, the PCI function could
 *  contain up to 2 SCU controllers.  So for B-step silicon, the
 *  driver_data field indicates the number of controllers in the
 *  package vs. controller index.
 */
static DEFINE_PCI_DEVICE_TABLE(isci_id_table) = {
	/* Patsburg B-step Silicon single controller Device IDs. */
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x1D61), 0, 0, 1 },   /* SKU B3 */
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x1D65), 0, 0, 1 },   /* SKU B1 */
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x1D69), 0, 0, 1 },   /* SKU B0 */
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x1D6B), 0, 0, 1 },   /* SKU A0 */

	/* Patsburg B-step Silicon dual controller Device IDs. */
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x1D60), 0, 0, 2 },   /* SKU D3, T3 */
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x1D62), 0, 0, 2 },   /* SKU D2, T2 */
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x1D64), 0, 0, 2 },   /* SKU D1, T1 */
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x1D68), 0, 0, 2 },   /* SKU D0, T0 */
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x1D6A), 0, 0, 2 },   /* SKU D0-    */
	{}
};
#else
#error COMPILER_ERR_NO_STEP_TYPE
#endif /* end PCI_DEVICE_TABLE definition */

static int __devinit isci_module_pci_probe(
	struct pci_dev *dev_p,
	const struct pci_device_id *device_id_p);

static void __devexit isci_module_pci_remove(struct pci_dev *dev_p);

MODULE_DEVICE_TABLE(pci, isci_id_table);

static struct pci_driver isci_pci_driver = {
	.name		= SCI_DRIVER_NAME,
	.id_table	= isci_id_table,
	.probe		= isci_module_pci_probe,
	.remove		= __devexit_p(isci_module_pci_remove),
};

/*
 * Maintain an 'uninitialized' value for module parameters
 */
#define UNINIT_PARAM 255

#ifdef OEM_PARAM_WORKAROUND
/*
 * OEM settable parameters. Note: These are temporary, eventually
 * we will obtain the settings for these parameters via NVRAM on
 * the SCU controller, but requires OROM support which does not
 * exist yet...
 *
 * Initialize to invalid values, will determine defaults later
 *
 * For all defined arrays:
 * elements 0-3 are for SCU0, ports 0-3
 * elements 4-7 are for SCU1, ports 0-3
 *
 * valid configurations for one SCU are:
 *  P0  P1  P2  P3
 * ----------------
 * 0xF,0x0,0x0,0x0 # 1 x4 port
 * 0x3,0x0,0x4,0x8 # Phys 0 and 1 are a x2 port, phy 2 and phy 3 are each x1
 *                 # ports
 * 0x1,0x2,0xC,0x0 # Phys 0 and 1 are each x1 ports, phy 2 and phy 3 are a x2
 *                 # port
 * 0x3,0x0,0xC,0x0 # Phys 0 and 1 are a x2 port, phy 2 and phy 3 are a x2 port
 * 0x1,0x2,0x4,0x8 # Each phy is a x1 port (this is the default configuration)
 *
 * if there is a port/phy on which you do not wish to override the default
 * values, use the value assigned to UNINIT_PARAM.
 */
static unsigned int phy_mask[SCI_MAX_PHYS * SCI_MAX_CONTROLLERS] = {
	UNINIT_PARAM,
	UNINIT_PARAM,
	UNINIT_PARAM,
	UNINIT_PARAM,
	UNINIT_PARAM,
	UNINIT_PARAM,
	UNINIT_PARAM,
	UNINIT_PARAM
};

static unsigned int phy_mask_num;
module_param_array(phy_mask, int, &phy_mask_num, 0);
MODULE_PARM_DESC(phy_mask, "phy mask for all 8 ports");

/*
 * sas_addresses[] will contain NULL pointers when not set during
 * module load.
 *
 * if there is a port/phy on which you do not wish to override the default
 * values, use the value "0000000000000000". SAS address of zero's is
 * considered invalid and will not be used.
 */
static char *sas_addresses[SCI_MAX_PHYS * SCI_MAX_CONTROLLERS];

static unsigned int addr_num;
module_param_array(sas_addresses, charp, &addr_num, 0);
MODULE_PARM_DESC(sas_addresses, "sas addresses for all 8 ports");

#endif

/*
 * User settable parameters. Initialize to invalid values, will
 * determine defaults later
 *
 * For all defined arrays:
 * elements 0-3 are for SCU0, ports 0-3
 * elements 4-7 are for SCU1, ports 0-3
 */
static unsigned int phy_gen[SCI_MAX_PHYS * SCI_MAX_CONTROLLERS] = {
	UNINIT_PARAM,
	UNINIT_PARAM,
	UNINIT_PARAM,
	UNINIT_PARAM,
	UNINIT_PARAM,
	UNINIT_PARAM,
	UNINIT_PARAM,
	UNINIT_PARAM
};

static unsigned int phy_gen_num;
module_param_array(phy_gen, int, &phy_gen_num, 0);
MODULE_PARM_DESC(phy_gen, "phy generation assignment for all 8 ports");

/* linux isci specific settings */
int loglevel = 3;
module_param(loglevel, int, S_IRUGO | S_IWUSR);

static int disable_msix;
module_param(disable_msix, int, S_IRUGO | S_IWUSR);

static inline int dev_is_sata(struct domain_device *dev)
{
	return dev->rphy->identify.target_port_protocols & SAS_PROTOCOL_SATA;
}

static int isci_target_alloc(struct scsi_target *starget)
{
	int ret = sas_target_alloc(starget);

	if (ret == 0) {
		struct domain_device *dev = starget->hostdata;
		isci_logger(trace, "found device, domain_device = %p\n", dev);
		if (dev_is_sata(dev))
			isci_logger(trace, "device %p is sata\n", dev);
	}
	return ret;
}

static int isci_slave_alloc(struct scsi_device *scsi_dev)
{
	sas_slave_alloc(scsi_dev);
	isci_logger(trace, "id = %d, lun = %d, channel = %d, "
		    "domain_device = %p\n ",
		    scsi_dev->id, scsi_dev->lun, scsi_dev->channel,
		    sdev_to_domain_dev(scsi_dev));
	return 0;
}

#ifndef ISCI_SLAVE_ALLOC
#define ISCI_SLAVE_ALLOC isci_slave_alloc
#endif /* not ISCI_SLAVE_ALLOC */

#ifndef ISCI_SLAVE_DESTROY
#define ISCI_SLAVE_DESTROY sas_slave_destroy
#endif /* not ISCI_SLAVE_DESTROY */

static struct scsi_host_template isci_sht = {

	.module				= THIS_MODULE,
	.name				= "isci",

	.queuecommand			= isci_queuecommand,

	.target_alloc			= isci_target_alloc,
	.slave_configure		= sas_slave_configure,
	.slave_destroy			= ISCI_SLAVE_DESTROY,
	.scan_finished			= isci_host_scan_finished,
	.scan_start			= isci_host_scan_start,
	.change_queue_depth		= sas_change_queue_depth,
	.change_queue_type		= sas_change_queue_type,
	.bios_param			= sas_bios_param,
	.can_queue			= 64,
	.cmd_per_lun			= 64,
	.this_id			= -1,
	.sg_tablesize			= SG_ALL,
	.max_sectors			= SCSI_DEFAULT_MAX_SECTORS,
	.use_clustering			= ENABLE_CLUSTERING,
	.eh_device_reset_handler	= sas_eh_device_reset_handler,
	.eh_bus_reset_handler		= isci_bus_reset_handler,
	.slave_alloc			= ISCI_SLAVE_ALLOC,
	.target_destroy			= sas_target_destroy,
	.ioctl				= sas_ioctl,
};

static struct sas_domain_function_template isci_domain_functions = {

	/* The class calls these to notify the LLDD of an event. */
	.lldd_port_formed	= isci_port_formed,
	.lldd_port_deformed	= isci_port_deformed,

	/* The class calls these when a device is found or gone. */
	.lldd_dev_found		= isci_remote_device_found,
	.lldd_dev_gone		= isci_remote_device_gone,

	.lldd_execute_task	= isci_task_execute_task,
	/* Task Management Functions. Must be called from process context. */
	.lldd_abort_task	= isci_task_abort_task,
	.lldd_abort_task_set	= isci_task_abort_task_set,
	.lldd_clear_aca		= isci_task_clear_aca,
	.lldd_clear_task_set	= isci_task_clear_task_set,
	.lldd_I_T_nexus_reset	= isci_task_I_T_nexus_reset,
	.lldd_lu_reset		= isci_task_lu_reset,
	.lldd_query_task	= isci_task_query_task,

	/* Port and Adapter management */
	.lldd_clear_nexus_port	= isci_task_clear_nexus_port,
	.lldd_clear_nexus_ha	= isci_task_clear_nexus_ha,

	/* Phy management */
	.lldd_control_phy	= isci_phy_control,
};


/******************************************************************************
* P R O T E C T E D  M E T H O D S
******************************************************************************/



/**
 * isci_module_register_sas_ha() - This method initializes various lldd
 *    specific members of the sas_ha struct and calls the libsas
 *    sas_register_ha() function.
 * @isci_host: This parameter specifies the lldd specific wrapper for the
 *    libsas sas_ha struct.
 *
 * This method returns an error code indicating sucess or failure. The user
 * should check for possible memory allocation error return otherwise, a zero
 * indicates success.
 */
static int isci_module_register_sas_ha(
	struct isci_host *isci_host)
{
	int i;
	struct sas_ha_struct *sas_ha = &(isci_host->sas_ha);
	struct asd_sas_phy **sas_phys;
	struct asd_sas_port **sas_ports;

	sas_phys = kmalloc(SCI_MAX_PHYS * sizeof(void *), GFP_KERNEL);
	if (!sas_phys)
		return -ENOMEM;

	sas_ports = kmalloc(SCI_MAX_PORTS * sizeof(void *), GFP_KERNEL);
	if (!sas_ports) {
		kfree(sas_phys);
		return -ENOMEM;
	}


	/*----------------- Libsas Initialization Stuff----------------------
	 * Set various fields in the sas_ha struct:
	 */

	sas_ha->sas_ha_name = (char *)&(isci_host->ha_name);
	sas_ha->lldd_module = THIS_MODULE;
	sas_ha->sas_addr    = &(isci_host->sas_addr[0]);

	/* set the array of phy and port structs.  */
	for (i = 0; i < SCI_MAX_PHYS; i++) {
		sas_phys[i] = &(isci_host->phys[i].sas_phy);
		sas_ports[i] = &(isci_host->sas_ports[i]);
	}

	sas_ha->sas_phy  = sas_phys;
	sas_ha->sas_port = sas_ports;
	sas_ha->num_phys = SCI_MAX_PHYS;

	sas_ha->lldd_queue_size = 64; /* asd_ha->seq.can_queue; */
	sas_ha->lldd_max_execute_num = lldd_max_execute_num;
	sas_ha->strict_wide_ports = 1;


	sas_register_ha(sas_ha);

	return 0;
}

static void isci_module_unregister_sas_ha(
	struct isci_host *isci_host)
{
	sas_unregister_ha(&(isci_host->sas_ha));

	sas_remove_host(isci_host->shost);
	scsi_remove_host(isci_host->shost);
	scsi_host_put(isci_host->shost);

	kfree(isci_host->sas_ha.sas_phy);
	kfree(isci_host->sas_ha.sas_port);

}


/**
 * isci_pci_deinit() - This method disables the pci device and unmaps/releases
 *    memory regions used by the specified BARs
 * @dev_p: This parameter specifies the pci device being disabled.
 * @isci_host: This parameter specifies the host adapter representing this pci
 *    device.
 * @bar_num: This parameter speifies last BAR number to be released starting
 *    with zero.
 *
 */
static void isci_pci_deinit(
	struct pci_dev *dev_p,
	struct isci_pci_func *isci_pci,
	int bar_num)
{
	isci_logger(trace, "\n", 0);

	for (; bar_num >= 0; bar_num--) {
		if (isci_pci->pci_bar[bar_num].virt_addr)
			iounmap(isci_pci->pci_bar[bar_num].virt_addr);

		isci_logger(trace,
			    "releasing bar #%d\n",
			    bar_num
			    );
		pci_release_region(dev_p, bar_num * 2);
	}

	pci_disable_device(dev_p);
}


/**
 * isci_module_pci_init() - This method performs various PCI initialization
 *    steps with the OS.
 * @dev_p: This parameter specifies the pci device being initialzed.
 * @isci_host: This parameter specifies the host adapter representing this pci
 *    device.
 *
 * This method returns an error code indicating sucess or failure, a zero
 * indicates success.
 */
static int __devinit isci_module_pci_init(
	struct pci_dev *dev_p,
	struct isci_pci_func *isci_pci)
{
	int err = 0;
	int bar_num = 0;

	err = pci_enable_device(dev_p);
	if (err) {
		isci_logger(error,
			    "failed enable PCI device %s!\n",
			    pci_name(dev_p)
			    );
		return err;
	}

	pci_set_master(dev_p);

	pci_set_drvdata(dev_p, isci_pci);

	for (bar_num = 0; bar_num < SCI_PCI_BAR_COUNT; bar_num++) {

		if (pci_request_region(dev_p, bar_num * 2, isci_pci->pci_func_name)) {
			isci_logger(error,
				    "pci_request_region failed for barNum %d\n",
				    bar_num
				    );
			err = -ENODEV;
			goto err_out;
		}
		isci_pci->pci_bar[bar_num].phys_addr =
			pci_resource_start(
				dev_p,
				bar_num * 2
				);
		isci_pci->pci_bar[bar_num].len =
			pci_resource_len(
				dev_p,
				bar_num * 2
				);
		isci_pci->pci_bar[bar_num].virt_addr =
			ioremap(
				isci_pci->pci_bar[bar_num].phys_addr,
				isci_pci->pci_bar[bar_num].len
				);

		if (NULL == isci_pci->pci_bar[bar_num].virt_addr) {
			err = -ENOMEM;
			goto err_out;
		}

#ifdef RCJ
		isci_logger(trace,
			    "HA Name: %s, isci_host->controller_id = %x\n"
			    "pci_bar[%d].phys_addr = %lx\n"
			    "pci_bar[%d].len = 0x%x\n"
			    "pci_bar[%d].virt_addr = %p\n",
			    isci_host->ha_name, isci_host->controller_id,
			    bar_num, isci_host->pci_bar[bar_num].phys_addr,
			    bar_num, isci_host->pci_bar[bar_num].len,
			    bar_num, isci_host->pci_bar[bar_num].virt_addr
			    );
#else
		isci_logger(trace,
			    "PCI Name: %s, isci_pci_func->controller_count = %d\n"
			    "pci_bar[%d].phys_addr = %lx\n"
			    "pci_bar[%d].len = 0x%x\n"
			    "pci_bar[%d].virt_addr = %p\n",
			    isci_pci->pci_func_name, isci_pci->controller_count,
			    bar_num, isci_pci->pci_bar[bar_num].phys_addr,
			    bar_num, isci_pci->pci_bar[bar_num].len,
			    bar_num, isci_pci->pci_bar[bar_num].virt_addr
			    );
#endif
	} /* for (bar_num = 0; bar_num < SCI_PCI_BAR_COUNT; bar_num++) */


	goto out;

 err_out:
	isci_pci_deinit(
		dev_p,
		isci_pci,
		bar_num
		);
 out:
	return err;
}

static void isci_module_free_interrupts(
	struct isci_pci_func *isci_pci)
{
	int i, ctl_idx, sci_num_msix_entries;
	struct isci_host *isci_host;

	if (isci_pci->msix_int_enabled) {
		/*
		 *  Determine the number of vectors associated with this
		 *  PCI function.
		 */
		sci_num_msix_entries = (isci_pci->controller_count == 2) ?
				       (SCI_NUM_MSI_X_INT + 2) : SCI_NUM_MSI_X_INT;
		/*
		 *  For MSIX, free IRQs on per entry & controller basis.
		 */
		for (i = 0; i < sci_num_msix_entries; i++) {
			ctl_idx = (isci_pci->msix_entries[i].entry < 2) ? 0 : 1;
			isci_host = &(isci_pci->ctrl[ctl_idx]);
			free_irq(isci_pci->msix_entries[i].vector, (void *)isci_host);
		}
		pci_disable_msix(isci_pci->k_pci_dev);
	} else {
		/*
		 *  For legacy, free IRQs on IRQ line and PCI Function basis.
		 */
		free_irq(isci_pci->k_pci_dev->irq, isci_pci);
	}
}

static int isci_module_enable_interrupts(
	struct isci_pci_func *isci_pci)
{
	int i;
	int err = 0;
	struct pci_dev *dev_p = isci_pci->k_pci_dev;
	int sci_num_msix_entries;

	/*
	 *  Determine the number of vectors associated with this
	 *  PCI function.
	 */
	sci_num_msix_entries = (isci_pci->controller_count == 2) ?
			       (SCI_NUM_MSI_X_INT + 2) : SCI_NUM_MSI_X_INT;

	memset(isci_pci->msix_entries, 0, sizeof(isci_pci->msix_entries));

	for (i = 0; i < sci_num_msix_entries; i++)
		isci_pci->msix_entries[i].entry = i;

	err = -1;

	if (!disable_msix)
		err = pci_enable_msix(
			dev_p,
			isci_pci->msix_entries,
			sci_num_msix_entries
			);

	if (!err) { /* Successfully enabled MSIX */

		isci_pci->msix_int_enabled = 1;
		for (i = 0; i < sci_num_msix_entries; i++) {
			u32 ctl_idx = (isci_pci->msix_entries[i].entry < 2) ? 0 : 1;

			isci_logger(trace,
				    "msix entry = %d, vector = %d\n",
				    isci_pci->msix_entries[i].entry,
				    isci_pci->msix_entries[i].vector
				    );

			/* @todo: need to handle error case. */
			err = request_irq(
				isci_pci->msix_entries[i].vector,
				isci_isr,
				IRQF_SHARED,
				"isci-msix",
				&(isci_pci->ctrl[ctl_idx])
				);
			if (err)
				isci_logger(error,
					    "request_irq failed - err = 0x%x\n",
					    err
					    );

		}

	} else {
		isci_logger(trace, "using legacy interrupts\n", 0);
		isci_pci->msix_int_enabled = 0;
		err = request_irq(
			isci_pci->k_pci_dev->irq,
			isci_legacy_isr,
			IRQF_SHARED,
			"isci",
			isci_pci
			);
		if (err)
			isci_logger(error, "request_irq failed - err = 0x%x\n",
				    err);

	}

	return err;
}

#ifdef OEM_PARAM_WORKAROUND

/**
 * isci_module_parse_oem_parameters() - This method will take OEM parameters
 *    from the module init parameters and copy them to oem_params. This will
 *    only copy values that are not set to the module parameter default values
 * @oem_parameters: This parameter specifies the controller default OEM
 *    parameters. It is expected that this has been initialized to the default
 *    parameters for the controller
 *
 *
 */
enum sci_status isci_module_parse_oem_parameters(
	union scic_oem_parameters *oem_params,
	int scu_index)
{
	int i;

	/* check for valid inputs */
	if (!(scu_index >= 0
	      && scu_index < SCI_MAX_CONTROLLERS
	      && oem_params != NULL)) {
		return SCI_FAILURE;
	}

	for (i = 0; i < SCI_MAX_PHYS; i++) {
		unsigned int array_idx = i + (SCI_MAX_PHYS * scu_index);

		char *p = sas_addresses[array_idx];

		u32 saddr_high;
		u32 saddr_low;
		int addr_idx;
		u32 addr_len = 0;

		saddr_high = 0;
		saddr_low = 0;

		if (p != NULL)
			addr_len = strlen(p);

		if ((p != NULL) && (addr_len < 16)) {

			for (addr_idx = 0; addr_idx < addr_len; addr_idx++) {
				unsigned char d = isdigit(p[addr_idx])
						  ? p[addr_idx] - '0'
						  : p[addr_idx] - 'A' + 10;

				if (addr_idx < 8)
					saddr_high |= (unsigned int)d
						      << ((7 - addr_idx) * 4);
				else
					saddr_low |= (unsigned int)d
						     << ((15 - addr_idx) * 4);
			}

			if (!(saddr_high == 0 && saddr_low == 0)) {
				oem_params->sds1.phys[i].sas_address.high
					= saddr_high;
				oem_params->sds1.phys[i].sas_address.low
					= saddr_low;
			}

		}
	}

	for (i = 0; i < SCI_MAX_PORTS; i++) {
		unsigned int array_idx = i + (SCI_MAX_PORTS * scu_index);
		unsigned int pmask = phy_mask[array_idx];

		if (pmask != UNINIT_PARAM)
			oem_params->sds1.ports[i].phy_mask = pmask;
	}

	return SCI_SUCCESS;
}

#endif

/**
 * isci_module_parse_user_parameters() - This method will take user parameters
 *    from the module init parameters and copy them to user_params. This will
 *    only copy values that are not set to the module parameter default values
 * @user_parameters: This parameter specifies the controller default user
 *    parameters. It is expected that this has been initialized to the default
 *    parameters for the controller
 *
 *
 */
enum sci_status isci_module_parse_user_parameters(
	union scic_user_parameters *user_params,
	int scu_index)
{
	int i;

	if (!(scu_index >= 0
	      && scu_index < SCI_MAX_CONTROLLERS
	      && user_params != NULL)) {
		return SCI_FAILURE;
	}

	for (i = 0; i < SCI_MAX_PORTS; i++) {
		unsigned int array_idx = i + (SCI_MAX_PORTS * scu_index);
		unsigned int gen = phy_gen[array_idx];

		/* only grab values that are not set as uninitialized */
		if (gen != UNINIT_PARAM)
			user_params->sds1.phys[i].max_speed_generation = gen;

	}

	return SCI_SUCCESS;
}

/******************************************************************************
* P U B L I C    M E T H O D S
******************************************************************************/

static ssize_t isci_show_can_queue(
	struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	static int min_queue = 256;
	struct isci_host *isci_host =
		pci_get_drvdata(to_pci_dev(dev));

	if (isci_host->can_queue < min_queue)
		min_queue = isci_host->can_queue;
	return snprintf(buf, PAGE_SIZE, "can_queue = %d, min_queue = %d\n",
			isci_host->can_queue, min_queue);
}

static
DEVICE_ATTR(can_queue, S_IRUGO,
	    isci_show_can_queue, NULL);

/**
 * isci_module_init_pci_function_name() - This method will initialize the name
 *    used in PCI Function initialization.
 * @This: parameter specifies the host adapter structure.
 *
 *
 */
static void isci_module_init_pci_function_name(
	struct isci_pci_func *isci_pci,
	unsigned int controller_count)
{
	/* Create the PCI function's unique name. */
	sprintf(isci_pci->pci_func_name, ISCI_PCI_FUNC_NAME_FMT, controller_count);

	isci_logger(
		trace,
		"isci_pci_func %p\n" "   pci_func_name = %s\n",
		isci_pci,
		isci_pci->pci_func_name,
		0);
}

/**
 * isci_module_pci_probe() - This method is the PCI probe function called by
 *    the Linux kernel when a device match is found.
 * @dev_p: This parameter specifies the pci device being probed.
 * @device_id_p: This parameter points to the device_struct, we supplied when
 *    we registered as a pci driver, representing the matching pci device.
 *
 * This method returns an error code indicating sucess or failure, a zero
 * indicates success.
 */
static int __devinit isci_module_pci_probe(
	struct pci_dev *dev_p,
	const struct pci_device_id *device_id_p)
{
	struct Scsi_Host *shost[2] = { NULL, NULL };
	struct isci_pci_func *isci_pci;
	int err = 0, ctlr_count = 0, count, core_lib_idx;

#if defined(PBG_HBA_A0_BUILD) || defined(PBG_HBA_A2_BUILD)
	unsigned int scu_idx = (unsigned int)device_id_p->driver_data;
#endif
	bool found = false;
	struct sci_pci_common_header pci_header;
	void *scil_memory;

	isci_logger(trace, "\n", 0);

	/*
	 *  First make sure there is room in the module for an SCI Core Library.
	 *  If so, get the core library size, allocate memory for core library,
	 *  store pointer in scu_module struct.
	 *  Concept:
	 *      1 isci_module structure per driver load image
	 *      1 Sci Libarary and isci_pci_func structure per pci function
	 *      upto 2 isci_host (SCIC controllers) per pci function depending on
	 *          SKU.
	 */
	for (core_lib_idx = 0; core_lib_idx < SCI_MAX_PCI_DEVICES; core_lib_idx++) {
		if ((isci_module_struct.core_lib[core_lib_idx].core_lib_handle == NULL) &&
		    (isci_module_struct.core_lib[core_lib_idx].core_lib_memory == NULL)) {
			found = true;
			break;
		}
	}

	if (found == false) {
		isci_logger(
			error,
			" exceeded max core lib count of %d\n",
			SCI_MAX_PCI_DEVICES);
		err = -ENOMEM;
		goto out;
	}
	scil_memory = isci_module_struct.core_lib[core_lib_idx].core_lib_memory =
			      kzalloc(
				      scic_library_get_object_size(SCI_MAX_CONTROLLERS),
				      GFP_KERNEL);

	if (NULL == scil_memory) {
		isci_logger(error, " kmalloc failed for library memory\n", 0);
		err = -ENOMEM;
		goto out;
	}
	/*
	 *  Construct core library using memory allocated for core library above.
	 */
	isci_module_struct.core_lib[core_lib_idx].core_lib_handle
		= scic_library_construct(scil_memory, SCI_MAX_CONTROLLERS);

	sci_logger_enable(
		sci_object_get_logger(
			isci_module_struct.core_lib[core_lib_idx].core_lib_handle),
		0xFFFFFFFF,
		0xFF);

	/*
	 *  Set association to the scu_module struct in the core library obj.
	 */
	sci_object_set_association(
		isci_module_struct.core_lib[core_lib_idx].core_lib_handle,
		(void *)&isci_module_struct);
	/*
	 * Set the PCI info for this PCI function in the SCIC library.
	 */
	pci_header.vendor_id    = dev_p->vendor;
	pci_header.device_id    = dev_p->device;
	pci_header.revision     = dev_p->revision;

	scic_library_set_pci_info(
		isci_module_struct.core_lib[core_lib_idx].core_lib_handle,
		&pci_header);

	/*
	 * Find out how many controllers are in this PCI function (device).
	 * SCIC determines this based on SKU.
	 */
	ctlr_count = scic_library_get_pci_device_controller_count(
		isci_module_struct.core_lib[core_lib_idx].core_lib_handle);

	/*
	 * Now allocate the isci_pci_func struct which in turn contains
	 * the isci_host structures that correspond to the SCU controllers.
	 */
	isci_pci = kzalloc(sizeof(struct isci_pci_func), GFP_KERNEL);
	if (!isci_pci) {
		err = -ENOMEM;
		goto lib_out;
	}
	/*
	 *  Set key fields in the isci_pci_func structure.
	 */
	isci_pci->controller_count = ctlr_count;
	isci_module_init_pci_function_name(isci_pci, ctlr_count);
	isci_pci->k_pci_driver = &(isci_pci_driver);
	isci_pci->k_pci_dev = dev_p;
	isci_pci->core_lib_handle = isci_module_struct.core_lib[core_lib_idx].core_lib_handle;
	isci_pci->core_lib_array_index = core_lib_idx;
	INIT_LIST_HEAD(&(isci_pci->node));

	/*------------- Linux Kernel Module Init stuff ----------------------*/
#if defined(PBG_HBA_A0_BUILD) || defined(PBG_HBA_A2_BUILD)
	/* Setup names based on the controller index */
	isci_host_init_controller_names(&(isci_pci->ctrl[0]), scu_idx);
#elif defined(PBG_HBA_BETA_BUILD)
	/*
	 *  Setup controller names for each controller on this PCI function.
	 *  Also link the isci_module to each controller's parent field and
	 *  link each controller back to it's PCI function.
	 */
	for (count = 0; count < ctlr_count; count++) {
		isci_host_init_controller_names(&(isci_pci->ctrl[count]), count);
		isci_pci->ctrl[count].parent = &isci_module_struct;
		isci_pci->ctrl[count].parent_pci_function = isci_pci;
	}
#else
#error COMPILER_ERR_NO_STEP_TYPE
#endif

	/* pci_set_dma_mask, dma_pool_create, map pci mem and io regions,
	 * create procfs and sysfs entries.
	 * get oem parameters and poulate sas addresses.
	 */
	err = isci_module_pci_init(dev_p, isci_pci);
	if (err) {
		kfree(isci_pci);
		err = -ENOMEM;
		goto lib_out;
	}

	err = device_create_file(&dev_p->dev, &dev_attr_can_queue);

	/*----------- SCSI Midlayer Initialization Stuff -------------------*/
	/*
	 *  Allocate the SCSI Host structures and link them to their
	 *  respective SCU controllers (isci_hosts).
	 */
	for (count = 0; count < ctlr_count; count++) {
		shost[count] = scsi_host_alloc(&isci_sht, sizeof(void *));
		if (!shost[count]) {
			err = -ENODEV;
			goto err_do_pci_deinit;
		}
		/*
		 *  SCSI host adapter instance was allocated. Link to corresponding
		 *  SCI Controller.
		 */
		isci_pci->ctrl[count].shost = shost[count];
		isci_pci->ctrl[count].pdev = dev_p;
	}

	err = isci_module_enable_interrupts(isci_pci);
	if (err) {
		err = -ENODEV;
		goto err_do_pci_deinit;
	}

	if (pci_set_dma_mask(dev_p, DMA_BIT_MASK(64))
	    || pci_set_consistent_dma_mask(dev_p, DMA_BIT_MASK(64))) {

		if (pci_set_dma_mask(dev_p, DMA_BIT_MASK(32))
		    || pci_set_consistent_dma_mask(dev_p, DMA_BIT_MASK(32))) {
			isci_logger(error, "set dma mask failed!\n", 0);
			err = -ENODEV;
			goto err_do_pci_deinit;
		}

	}

	/*------------- SCIC controller Initialization Stuff ---------------*/
	/*
	 *  Initialize each SCU controller on the PCI device.  Also register
	 *  a scsi host adapter instance for each SCU controller to the
	 *  kernel.
	 */
	for (count = 0; count < ctlr_count; count++) {
		struct isci_host *isci_host = &(isci_pci->ctrl[count]);
		err = isci_host_init(dev_p, isci_host);
		if (err) {
			isci_logger(error, "isci_host_init failed - err = %d\n", err);
			scsi_host_put(shost[count]);
			goto err_do_free_irq;
		}

		SHOST_TO_SAS_HA(shost[count]) = &isci_host->sas_ha;
		isci_host->sas_ha.core.shost = shost[count];
		(shost[count])->transportt = isci_module_struct.stt;

		/* SPB Debug: where do these max vaules come from ? */
		(shost[count])->max_id = ~0;
		(shost[count])->max_lun = ~0;
		(shost[count])->max_cmd_len = 16;

		/*----------- SCSI Midlayer Initialization Stuff -------------------
		 * struct scsi_transport_template* is stored in the scsi_host
		 * struct transport member. the scsi_host struct pointer is
		 * passed in the call to scsi_add_host().
		 */
		err = scsi_add_host(shost[count], &dev_p->dev);
		if (err) {
			scsi_host_put(shost[count]);
			goto err_do_free_irq;
		}
	}

	for (count = 0; count < SCI_MAX_CONTROLLERS; count++) {
		struct isci_host *isci_host = &(isci_pci->ctrl[count]);
		err = isci_module_register_sas_ha(isci_host);
		if (err) {
			isci_host_mdl_deallocate_coherent(isci_host);
		}
	}
	if (err) goto err_do_scsi_host_clean;
	/*
	 *  Now scan each controller for attached devices
	 *  (i.e. device discovery).  The api attempts to
	 *  schedule an async scan by default.  However,
	 *  if resources to schedule the scan aren't available,
	 *  it will call our scan entry points and perform device
	 *  discovery under the current thread.
	 */
	for (count = 0; count < ctlr_count; count++) {
		scsi_scan_host(shost[count]);
	}

	goto out;

 err_do_scsi_host_clean:
	for (count = 0; count < ctlr_count; count++) {
		if (shost[count] != NULL) {
			scsi_host_put(shost[count]);
			scsi_remove_host(shost[count]);
		}
	}

 err_do_free_irq:
	isci_module_free_interrupts(isci_pci);

 err_do_pci_deinit:
	isci_pci_deinit(
		dev_p,
		isci_pci,
		SCI_PCI_BAR_COUNT - 1
		);
	kfree(isci_pci);

 lib_out:
	kfree(scil_memory);
	isci_module_struct.core_lib[core_lib_idx].core_lib_handle = NULL;
	isci_module_struct.core_lib[core_lib_idx].core_lib_memory = NULL;
 out:
	return err;
}

/**
 * isci_module_pci_remove() - This method is the PCI probe function called by
 *    the Linux kernel when a device is removed.
 * @dev_p: This parameter specifies the pci device being removed.
 *
 */
static void __devexit isci_module_pci_remove(
	struct pci_dev *dev_p)
{
	int i;
	struct isci_pci_func *isci_pci =
		(struct isci_pci_func *)pci_get_drvdata(dev_p);
	struct isci_host *isci_host;

	isci_logger(trace, "\n", 0);

	for (i = 0; i < isci_pci->controller_count; i++) {
		isci_host = &(isci_pci->ctrl[i]);
		isci_module_unregister_sas_ha(isci_host);
		isci_host_deinit(isci_host);
		scic_controller_disable_interrupts(isci_host->core_controller);
	}

	device_remove_file(&dev_p->dev, &dev_attr_can_queue);

	isci_module_free_interrupts(isci_pci);

	/*
	 *  Now that SAS HA is unregistered per controller,
	 *  controller unregistered with SCIL and
	 *  IRQs freed, clean up memory per controller.
	 */
	for (i = 0; i < isci_pci->controller_count; i++) {
		isci_host = &(isci_pci->ctrl[i]);
		isci_host_mdl_deallocate_coherent(isci_host);
	}

	isci_pci_deinit(dev_p, isci_pci, SCI_PCI_BAR_COUNT - 1);

	list_del(&(isci_pci->node));
	if (list_empty(&(isci_module_struct.pci_devices)))
		isci_timer_list_destroy(
			&(isci_module_struct.timer_list_struct)
			);

	kfree(isci_pci);
}

#define SCI_MAX_TIMER_COUNT 25

/**
 * isci_module_init() - This method is the module init function called by when
 *    this module is loaded. It performs global initialization with libsas and
 *    the SCI Core Library, then registers as a PCI driver.
 *
 * This method returns an error code indicating sucess or failure, a zero
 * indicates success.
 */
static __init int isci_module_init(
	void)
{
	int count;

	/*
	 *  Initialize core_lib fields of isci_module_struct.
	 */
	for (count = 0; count < SCI_MAX_PCI_DEVICES; count++) {
		isci_module_struct.core_lib[count].core_lib_handle = NULL;
		isci_module_struct.core_lib[count].core_lib_memory = NULL;
	}
	/*
	 *  Set the loglevel for the module.
	 */
	isci_module_struct.loglevel = loglevel;

	/*------- Libsas Initialization Stuff -------------------------------
	 * static struct &isci_domain_functions contains lldd function
	 * pointers to handle sas transport class actions, is passed into
	 * sas_domain_attach_transport(), which adds libsas functions to and
	 *  returns a scsi_transport_template.
	 */
	isci_module_struct.stt =
		sas_domain_attach_transport(&isci_domain_functions);

	if (NULL == isci_module_struct.stt) {
		isci_logger(error, "sas_domain_attach_transport failed\n", 0);
		goto err;
	}

	/* alloc and intialize timers, add to timer_list.  */
	isci_timer_list_construct(
		&isci_module_struct.timer_list_struct,
		SCI_MAX_TIMER_COUNT
		);

	if ((pci_register_driver(&isci_pci_driver)) < 0) {
		isci_logger(error, "Register PCI failed\n", 0);
		goto err;
	}

	return 0;

 err:

	return 1;
}


/**
 * isci_module_exit() - This method is the module exit function called by when
 *    this module is unloaded.
 *
 *
 */
static __exit void isci_module_exit(
	void)
{
	int count;

	pci_unregister_driver(&isci_pci_driver);
	sas_release_transport(isci_module_struct.stt);
	for (count = 0; count < SCI_MAX_PCI_DEVICES; count++) {
		if (isci_module_struct.core_lib[count].core_lib_memory != NULL)
			kfree(isci_module_struct.core_lib[count].core_lib_memory);
	}
}

module_init(isci_module_init);
module_exit(isci_module_exit);
