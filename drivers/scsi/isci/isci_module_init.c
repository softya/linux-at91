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
#include "scic_remote_device.h"

MODULE_LICENSE("Dual BSD/GPL");


static int lldd_max_execute_num = 64;

struct kmem_cache *isci_kmem_cache;
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

#if defined(CONFIG_PBG_HBA_A0) || defined(CONFIG_PBG_HBA_A2)
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
#elif defined(CONFIG_PBG_HBA_BETA)
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
	.name		= DRV_NAME,
	.id_table	= isci_id_table,
	.probe		= isci_module_pci_probe,
	.remove		= __devexit_p(isci_module_pci_remove),
};

/* linux isci specific settings */
int loglevel = 3;
module_param(loglevel, int, S_IRUGO | S_IWUSR);

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
static int isci_module_register_sas_ha(struct isci_host *isci_host)
{
	int i;
	struct sas_ha_struct *sas_ha = &(isci_host->sas_ha);
	struct asd_sas_phy **sas_phys;
	struct asd_sas_port **sas_ports;

	sas_phys = devm_kzalloc(&isci_host->pdev->dev,
				SCI_MAX_PHYS * sizeof(void *),
				GFP_KERNEL);
	if (!sas_phys)
		return -ENOMEM;

	sas_ports = devm_kzalloc(&isci_host->pdev->dev,
				 SCI_MAX_PORTS * sizeof(void *),
				 GFP_KERNEL);
	if (!sas_ports)
		return -ENOMEM;

	/*----------------- Libsas Initialization Stuff----------------------
	 * Set various fields in the sas_ha struct:
	 */

	sas_ha->sas_ha_name = DRV_NAME;
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

static void isci_module_unregister_sas_ha(struct isci_host *isci_host)
{
	if (!isci_host)
		return;

	sas_unregister_ha(&(isci_host->sas_ha));

	sas_remove_host(isci_host->shost);
	scsi_remove_host(isci_host->shost);
	scsi_host_put(isci_host->shost);
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
	int bar_mask;
	void __iomem * const *iomap;

	err = pcim_enable_device(dev_p);
	if (err) {
		isci_logger(error,
			    "failed enable PCI device %s!\n",
			    pci_name(dev_p)
			    );
		return err;
	}

	for (bar_num = 0; bar_num < SCI_PCI_BAR_COUNT; bar_num++)
		bar_mask |= 1 << (bar_num * 2);

	err = pcim_iomap_regions(dev_p, bar_mask, DRV_NAME);
	if (err)
		return err;

	iomap = pcim_iomap_table(dev_p);
	if (!iomap)
		return -ENOMEM;

	for (bar_num = 0; bar_num < SCI_PCI_BAR_COUNT; bar_num++)
		isci_pci->pci_bar[bar_num].virt_addr = iomap[bar_num * 2];

	pci_set_master(dev_p);

	pci_set_drvdata(dev_p, isci_pci);

	return 0;
}

static int isci_module_enable_interrupts(struct isci_pci_func *isci_pci)
{
	int i, j;
	int err = -EINVAL;
	struct pci_dev *dev_p = isci_pci->k_pci_dev;
	int sci_num_msix_entries;
	struct msix_entry *msix;

	/*
	 *  Determine the number of vectors associated with this
	 *  PCI function.
	 */
	sci_num_msix_entries = (isci_pci->controller_count == 2) ?
			       (SCI_NUM_MSI_X_INT + 2) : SCI_NUM_MSI_X_INT;

	memset(isci_pci->msix_entries, 0, sizeof(isci_pci->msix_entries));

	for (i = 0; i < sci_num_msix_entries; i++)
		isci_pci->msix_entries[i].entry = i;

	err = pci_enable_msix(dev_p, isci_pci->msix_entries,
			      sci_num_msix_entries);

	if (!err) { /* Successfully enabled MSIX */
		for (i = 0; i < sci_num_msix_entries; i++) {
			u32 ctl_idx =
				(isci_pci->msix_entries[i].entry < 2) ? 0 : 1;

			isci_logger(trace,
				    "msix entry = %d, vector = %d\n",
				    isci_pci->msix_entries[i].entry,
				    isci_pci->msix_entries[i].vector
				    );

			/* @todo: need to handle error case. */
			err = devm_request_irq(&dev_p->dev,
					       isci_pci->msix_entries[i].vector,
					       isci_isr,
					       0,
					       "isci-msix",
					       &(isci_pci->ctrl[ctl_idx]));
			if (err) {
				isci_logger(error,
					    "request_irq failed - err = 0x%x\n",
					    err
					    );
				for (j = 0; j < i; j++) {
					msix = &isci_pci->msix_entries[j];
					devm_free_irq(&dev_p->dev,
						      msix->vector,
						      &(isci_pci->ctrl[ctl_idx]));
				}
				pci_disable_msix(dev_p);
				goto intx;
			}

		}

	} else {
intx:
		err = devm_request_irq(&dev_p->dev,
				       isci_pci->k_pci_dev->irq,
				       isci_legacy_isr,
				       IRQF_SHARED,
				       "isci",
				       isci_pci);
		if (err)
			isci_logger(error, "request_irq failed - err = 0x%x\n",
				    err);

	}

	return err;
}

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
	int scu_index,
	struct isci_firmware *fw)
{
	int i;

	/* check for valid inputs */
	if (!(scu_index >= 0
	      && scu_index < SCI_MAX_CONTROLLERS
	      && oem_params != NULL)) {
		return SCI_FAILURE;
	}

	for (i = 0; i < SCI_MAX_PHYS; i++) {
		int array_idx = i + (SCI_MAX_PHYS * scu_index);
		u64 sas_addr = fw->sas_addrs[array_idx];

		if (sas_addr != 0) {
			oem_params->sds1.phys[i].sas_address.low =
				(u32)(sas_addr & 0xffffffff);
			oem_params->sds1.phys[i].sas_address.high =
				(u32)((sas_addr >> 32) & 0xffffffff);
		}
	}

	for (i = 0; i < SCI_MAX_PORTS; i++) {
		int array_idx = i + (SCI_MAX_PORTS * scu_index);
		u32 pmask = fw->phy_masks[array_idx];

		oem_params->sds1.ports[i].phy_mask = pmask;
	}

	return SCI_SUCCESS;
}

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
	int scu_index,
	struct isci_firmware *fw)
{
	int i;

	if (!(scu_index >= 0
	      && scu_index < SCI_MAX_CONTROLLERS
	      && user_params != NULL)) {
		return SCI_FAILURE;
	}

	for (i = 0; i < SCI_MAX_PORTS; i++) {
		int array_idx = i + (SCI_MAX_PORTS * scu_index);
		u32 gen = fw->phy_gens[array_idx];

		user_params->sds1.phys[i].max_speed_generation = gen;

	}

	return SCI_SUCCESS;
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
	for (core_lib_idx = 0;
	     core_lib_idx < SCI_MAX_PCI_DEVICES;
	     core_lib_idx++) {
		if ((isci_module_struct.core_lib[core_lib_idx].core_lib_handle
					== NULL) &&
		    (isci_module_struct.core_lib[core_lib_idx].core_lib_memory
					== NULL)) {
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

	scil_memory =
		isci_module_struct.core_lib[core_lib_idx].core_lib_memory =
			devm_kzalloc(&dev_p->dev,
				     scic_library_get_object_size(
					     SCI_MAX_CONTROLLERS),
				     GFP_KERNEL);
	if (!scil_memory) {
		isci_logger(error, " kmalloc failed for library memory\n", 0);
		return -ENOMEM;
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
	isci_pci = devm_kzalloc(&dev_p->dev,
				sizeof(struct isci_pci_func),
				GFP_KERNEL);
	if (!isci_pci) {
		err = -ENOMEM;
		goto lib_out;
	}
	/*
	 *  Set key fields in the isci_pci_func structure.
	 */
	isci_pci->controller_count = ctlr_count;
	isci_pci->k_pci_driver = &(isci_pci_driver);
	isci_pci->k_pci_dev = dev_p;
	isci_pci->core_lib_handle = isci_module_struct.core_lib[core_lib_idx].core_lib_handle;
	isci_pci->core_lib_array_index = core_lib_idx;
	INIT_LIST_HEAD(&(isci_pci->node));

#if defined(CONFIG_PBG_HBA_BETA)
	/*  Link the isci_module to each controller's parent field and
	 *  link each controller back to it's PCI function.
	 */
	for (count = 0; count < ctlr_count; count++) {
		isci_pci->ctrl[count].parent = &isci_module_struct;
		isci_pci->ctrl[count].parent_pci_function = isci_pci;
	}
#endif

	/* pci_set_dma_mask, dma_pool_create, map pci mem and io regions,
	 * create procfs and sysfs entries.
	 * get oem parameters and poulate sas addresses.
	 */
	err = isci_module_pci_init(dev_p, isci_pci);
	if (err) {
		err = -ENOMEM;
		goto lib_out;
	}

	for (count = 0; count < ctlr_count; count++) {
		shost[count] = scsi_host_alloc(&isci_sht, sizeof(void *));
		if (!shost[count]) {
			err = -ENODEV;
			goto lib_out;
		}
		isci_pci->ctrl[count].shost = shost[count];
		isci_pci->ctrl[count].pdev = dev_p;
	}

	err = isci_module_enable_interrupts(isci_pci);
	if (err) {
		err = -ENODEV;
		goto lib_out;
	}

	err = pci_set_dma_mask(dev_p, DMA_BIT_MASK(64));
	if (err) {
		err = pci_set_dma_mask(dev_p, DMA_BIT_MASK(32));
		if (err) {
			isci_logger(error, "set DMA mask failed!\n", 0);
			goto lib_out;
		}
	}

	err = pci_set_consistent_dma_mask(dev_p, DMA_BIT_MASK(64));
	if (err) {
		err = pci_set_consistent_dma_mask(dev_p, DMA_BIT_MASK(32));
		if (err) {
			isci_logger(error,
				    "set consistent DMA mask failed!\n", 0);
			goto lib_out;

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
			goto lib_out;
		}

		SHOST_TO_SAS_HA(shost[count]) = &isci_host->sas_ha;
		isci_host->sas_ha.core.shost = shost[count];
		(shost[count])->transportt = isci_module_struct.stt;

		/* SPB Debug: where do these max vaules come from ? */
		(shost[count])->max_id = ~0;
		(shost[count])->max_lun = ~0;
		(shost[count])->max_cmd_len = MAX_COMMAND_SIZE;

		/*----------- SCSI Midlayer Initialization Stuff --------------
		 * struct scsi_transport_template* is stored in the scsi_host
		 * struct transport member. the scsi_host struct pointer is
		 * passed in the call to scsi_add_host().
		 */
		err = scsi_add_host(shost[count], &dev_p->dev);
		if (err) {
			scsi_host_put(shost[count]);
			goto lib_out;
		}
	}

	for (count = 0; count < SCI_MAX_CONTROLLERS; count++) {
		struct isci_host *isci_host = &(isci_pci->ctrl[count]);
		err = isci_module_register_sas_ha(isci_host);
	}
	if (err)
		goto err_do_scsi_host_clean;

	for (count = 0; count < ctlr_count; count++)
		scsi_scan_host(shost[count]);

	return 0;

 err_do_scsi_host_clean:
	for (count = 0; count < ctlr_count; count++) {
		if (shost[count] != NULL) {
			scsi_host_put(shost[count]);
			scsi_remove_host(shost[count]);
		}
	}

 lib_out:
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
static void __devexit isci_module_pci_remove(struct pci_dev *dev_p)
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

	list_del(&(isci_pci->node));
	if (list_empty(&(isci_module_struct.pci_devices)))
		isci_timer_list_destroy(
			&(isci_module_struct.timer_list_struct)
			);
}

#define SCI_MAX_TIMER_COUNT 25

static __init int isci_module_init(void)
{
	int count;
	int err = -ENOMEM;

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

	isci_kmem_cache = kmem_cache_create(DRV_NAME,
					    sizeof(struct isci_remote_device) +
					    scic_remote_device_get_object_size(),
					    0, 0, NULL);
	if (!isci_kmem_cache)
		goto err;


	/*------- Libsas Initialization Stuff -------------------------------
	 * static struct &isci_domain_functions contains lldd function
	 * pointers to handle sas transport class actions, is passed into
	 * sas_domain_attach_transport(), which adds libsas functions to and
	 *  returns a scsi_transport_template.
	 */
	isci_module_struct.stt =
		sas_domain_attach_transport(&isci_domain_functions);

	if (!isci_module_struct.stt) {
		pr_err("%s: sas_domain_attach_transport failed\n", __func__);
		goto err_kmem;
	}

	/* alloc and intialize timers, add to timer_list.  */
	isci_timer_list_construct(
		&isci_module_struct.timer_list_struct,
		SCI_MAX_TIMER_COUNT
		);

	err = pci_register_driver(&isci_pci_driver);
	if (err) {
		pr_err("%s: Register PCI failed\n", __func__);
		goto err_sas;
	}

	return 0;
 err_sas:
	sas_release_transport(isci_module_struct.stt);
 err_kmem:
	kmem_cache_destroy(isci_kmem_cache);
 err:
	return err;
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
	pci_unregister_driver(&isci_pci_driver);
	sas_release_transport(isci_module_struct.stt);
	kmem_cache_destroy(isci_kmem_cache);
}

MODULE_FIRMWARE(ISCI_FW_NAME);
module_init(isci_module_init);
module_exit(isci_module_exit);
