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
 * isci_host.h
 */



#if !defined(_SCI_HOST_H_)
#define _SCI_HOST_H_

#include "isci_phy.h"
/*#include "isci_task.h"*/
#include "isci_timers.h"
#include "isci_remote_device.h"
#include "scic_user_callback.h"

#define DRV_NAME "isci"
#define SCI_PCI_BAR_COUNT 2
#define SCI_NUM_MSI_X_INT 2
#define SCI_MSIX_NORMAL_VECTOR 0
#define SCI_MSIX_ERROR_VECTOR 1
#define SCI_MSIX_SINGLE_VECTOR 1
#define SCI_MSIX_DOUBLE_VECTOR 2
#define ISCI_CAN_QUEUE_VAL 250
#define SCIC_CONTROLLER_STOP_TIMEOUT 5000

struct coherent_memory_info {
	struct list_head node;
	dma_addr_t dma_handle;
	void *vaddr;
	size_t size;
	struct sci_physical_memory_descriptor *mde;
};

/**
 * struct isci_host - This class contains the SCI Library controller specific
 *    info.
 *
 *
 */
struct isci_host {

	SCI_CONTROLLER_HANDLE_T core_controller;
	struct scic_controller_handler_methods scic_irq_handlers[SCI_NUM_MSI_X_INT];
	union scic_oem_parameters oem_parameters;

	int controller_id;
	struct isci_timer_list timer_list_struct;
	void *core_ctrl_memory;
	struct dma_pool *dma_pool;
	unsigned int dma_pool_alloc_size;
	struct isci_phy phys[SCI_MAX_PHYS];

	/* isci_ports and sas_ports are implicitly parallel to the
	 * ports maintained by the core
	 */
	struct isci_port isci_ports[SCI_MAX_PORTS];
	struct asd_sas_port sas_ports[SCI_MAX_PORTS];
	struct sas_ha_struct sas_ha;

	int can_queue;
	spinlock_t queue_lock;
	spinlock_t state_lock;

	struct pci_dev *pdev;
	u8 sas_addr[SAS_ADDR_SIZE];

	enum isci_status status;
	struct Scsi_Host *shost;
	struct isci_module *parent;
	struct isci_pci_func *parent_pci_function;
	struct tasklet_struct completion_tasklet;
	struct list_head mdl_struct_list;
	struct list_head requests_to_complete;
	struct list_head requests_to_abort;
	struct completion stop_complete;
	struct completion start_complete;
	spinlock_t scic_lock;
};


/**
 * struct isci_pci_func - This class represents the pci function containing the
 *    controllers. Depending on PCI SKU, there could be up to 2 controllers in
 *    the PCI function.  SCI_MAX_CONTROLLERS is defined at compile time.
 *
 *
 */

struct isci_pci_func {
	u8 controller_count;
	u8 reserved[3];
	struct list_head node;

	struct pci_driver *k_pci_driver;
	struct pci_dev *pdev;

	struct pci_bar_info {
		unsigned long phys_addr;
		void *virt_addr;
		int len;
	} pci_bar[SCI_PCI_BAR_COUNT];

#if defined(CONFIG_PBG_HBA_BETA)
	struct msix_entry msix_entries[4 + 1];
#else
	struct msix_entry msix_entries[SCI_NUM_MSI_X_INT + 1];
#endif
	int core_lib_array_index;
	SCI_LIBRARY_HANDLE_T core_lib_handle;

	struct isci_host ctrl[SCI_MAX_CONTROLLERS];
};


static inline
enum isci_status isci_host_get_state(
	struct isci_host *isci_host)
{
	return isci_host->status;
}


static inline void isci_host_change_state(
	struct isci_host *isci_host,
	enum isci_status status)
{
	unsigned long flags;

	scic_cb_logger_log_states(
		0, 0, "%s: isci_host = %p, state = 0x%x",
		__func__, isci_host, status
		);
	spin_lock_irqsave(&isci_host->state_lock, flags);
	isci_host->status = status;
	spin_unlock_irqrestore(&isci_host->state_lock, flags);

}

static inline int isci_host_can_queue(
	struct isci_host *isci_host,
	int num)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&isci_host->queue_lock, flags);
	if ((isci_host->can_queue - num) < 0) {
		scic_cb_logger_log_trace(
			0, 0, "%s: isci_host->can_queue = %d\n",
			__func__, isci_host->can_queue
			);
		ret = -SAS_QUEUE_FULL;

	} else
		isci_host->can_queue -= num;

	spin_unlock_irqrestore(&isci_host->queue_lock, flags);

	return ret;
}

static inline void isci_host_can_dequeue(
	struct isci_host *isci_host,
	int num)
{
	unsigned long flags;

	spin_lock_irqsave(&isci_host->queue_lock, flags);
	isci_host->can_queue += num;
	spin_unlock_irqrestore(&isci_host->queue_lock, flags);
}

/**
 * isci_host_from_sas_ha() - This accessor retrieves the isci_host object
 *    reference from the Linux sas_ha_struct reference.
 * @ha_struct,: This parameter points to the Linux sas_ha_struct object
 *
 * A reference to the associated isci_host structure.
 */
#define isci_host_from_sas_ha(ha_struct) \
	((struct isci_host *)(ha_struct)->lldd_ha)

/**
 * isci_host_scan_finished() -
 *
 * This function is one of the SCSI Host Template functions. The SCSI midlayer
 * calls this function during a target scan, approx. once every 10 millisecs.
 */
int isci_host_scan_finished(
	struct Scsi_Host *,
	unsigned long);


/**
 * isci_host_scan_start() -
 *
 * This function is one of the SCSI Host Template function, called by the SCSI
 * mid layer berfore a target scan begins. The core library controller start
 * routine is called from here.
 */
void isci_host_scan_start(
	struct Scsi_Host *);

/**
 * isci_host_start_complete() -
 *
 * This function is called by the core library, through the ISCI Module, to
 * indicate controller start status.
 */
void isci_host_start_complete(
	struct isci_host *,
	enum sci_status);

void isci_host_stop_complete(
	struct isci_host *isci_host,
	enum sci_status completion_status);

/**
 * isci_host_pci_get_bar() -
 *
 * This function is called by the core library, through the ISCI Module, to get
 * contents of the specified BAR.
 */
void *isci_host_pci_get_bar(
	struct isci_host *,
	unsigned int);


/**
 * isci_host_init() -
 *
 * This function intializes a newly created host adapter object.
 */
int isci_host_init(
	struct pci_dev *,
	struct isci_host *);

void isci_host_init_controller_names(
	struct isci_host *isci_host,
	unsigned int controller_idx);

void isci_host_deinit(
	struct isci_host *);

void isci_host_port_link_up(
	struct isci_host *,
	SCI_PORT_HANDLE_T,
	SCI_PHY_HANDLE_T);
int isci_host_dev_found(struct domain_device *);

void isci_host_remote_device_start_complete(
	struct isci_host *,
	struct isci_remote_device *,
	enum sci_status);

#endif /* !defined(_SCI_HOST_H_) */
