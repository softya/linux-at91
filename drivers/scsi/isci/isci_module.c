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
 * This file contains isci module object implementation.
 *
 *
 */

#include "isci_module.h"
#include "isci_request.h"
#include "isci_sata.h"
#include "isci_task.h"

/**
 * scic_cb_timer_create() - This callback method asks the user to create a
 *    timer and provide a handle for this timer for use in further timer
 *    interactions. The appropriate isci timer object function is called to
 *    create a timer object.
 * @timer_callback: This parameter specifies the callback method to be invoked
 *    whenever the timer expires.
 * @controller: This parameter specifies the controller with which this timer
 *    is to be associated.
 * @cookie: This parameter specifies a piece of information that the user must
 *    retain.  This cookie is to be supplied by the user anytime a timeout
 *    occurs for the created timer.
 *
 * This method returns a handle to a timer object created by the user.  The
 * handle will be utilized for all further interactions relating to this timer.
 */
void *scic_cb_timer_create(
	SCI_CONTROLLER_HANDLE_T controller,
	SCI_TIMER_CALLBACK_T timer_callback,
	void *cookie)
{
	struct isci_host *isci_host;
	struct isci_timer *timer = NULL;

	isci_host = (struct isci_host *)sci_object_get_association(controller);

	dev_dbg(&isci_host->pdev->dev,
		"%s: isci_host = %p",
		__func__, isci_host);

	timer = isci_timer_create(&isci_host->timer_list_struct,
				  isci_host,
				  cookie,
				  timer_callback);

	dev_dbg(&isci_host->pdev->dev, "%s: timer = %p\n", __func__, timer);

	return (void *)timer;
}


/**
 * scic_cb_timer_start() - This callback method asks the user to start the
 *    supplied timer. The appropriate isci timer object function is called to
 *    start the timer.
 * @controller: This parameter specifies the controller with which this timer
 *    is to associated.
 * @timer: This parameter specifies the timer to be started.
 * @milliseconds: This parameter specifies the number of milliseconds for which
 *    to stall.  The operating system driver is allowed to round this value up
 *    where necessary.
 *
 */
void scic_cb_timer_start(
	SCI_CONTROLLER_HANDLE_T controller,
	void *timer,
	u32 milliseconds)
{
	struct isci_host *isci_host;

	isci_host =
		(struct isci_host *)sci_object_get_association(controller);

	dev_dbg(&isci_host->pdev->dev,
		"%s: isci_host = %p, timer = %p, milliseconds = %d\n",
		__func__, isci_host, timer, milliseconds);

	isci_timer_start((struct isci_timer *)timer, milliseconds);

}

/**
 * scic_cb_timer_stop() - This callback method asks the user to stop the
 *    supplied timer. The appropriate isci timer object function is called to
 *    stop the timer.
 * @controller: This parameter specifies the controller with which this timer
 *    is to associated.
 * @timer: This parameter specifies the timer to be stopped.
 *
 */
void scic_cb_timer_stop(
	SCI_CONTROLLER_HANDLE_T controller,
	void *timer)
{
	struct isci_host *isci_host;

	isci_host =
		(struct isci_host *)sci_object_get_association(controller);

	dev_dbg(&isci_host->pdev->dev,
		"%s: isci_host = %p, timer = %p\n",
		__func__, isci_host, timer);

	isci_timer_stop((struct isci_timer *)timer);
}

/**
 * scic_cb_stall_execution() - This method is called when the core requires the
 *    OS driver to stall execution.  This method is utilized during
 *    initialization or non-performance paths only.
 * @microseconds: This parameter specifies the number of microseconds for which
 *    to stall.  The operating system driver is allowed to round this value up
 *    where necessary.
 *
 * none.
 */
void scic_cb_stall_execution(
	u32 microseconds)
{
	udelay(microseconds);
}

/**
 * scic_cb_controller_start_complete() - This user callback will inform the
 *    user that the controller has finished the start process. The associated
 *    isci host adapter's start_complete function is called.
 * @controller: This parameter specifies the controller that was started.
 * @completion_status: This parameter specifies the results of the start
 *    operation.  SCI_SUCCESS indicates successful completion.
 *
 */
void scic_cb_controller_start_complete(
	SCI_CONTROLLER_HANDLE_T controller,
	enum sci_status completion_status)
{
	struct isci_host *isci_host =
		(struct isci_host *)sci_object_get_association(controller);

	dev_dbg(&isci_host->pdev->dev,
		"%s: isci_host = %p\n", __func__, isci_host);

	isci_host_start_complete(isci_host, completion_status);
}

/**
 * scic_cb_controller_stop_complete() - This user callback will inform the user
 *    that the controller has finished the stop process. The associated isci
 *    host adapter's start_complete function is called.
 * @controller: This parameter specifies the controller that was stopped.
 * @completion_status: This parameter specifies the results of the stop
 *    operation.  SCI_SUCCESS indicates successful completion.
 *
 */
void scic_cb_controller_stop_complete(
	SCI_CONTROLLER_HANDLE_T controller,
	enum sci_status completion_status)
{
	struct isci_host *isci_host =
		(struct isci_host *)sci_object_get_association(controller);

	dev_dbg(&isci_host->pdev->dev,
		"%s: status = 0x%x\n", __func__, completion_status);
	isci_host_stop_complete(isci_host, completion_status);
}

/**
 * scic_cb_io_request_complete() - This user callback will inform the user that
 *    an IO request has completed.
 * @controller: This parameter specifies the controller on which the IO is
 *    completing.
 * @remote_device: This parameter specifies the remote device on which this IO
 *    request is completing.
 * @io_request: This parameter specifies the IO request that has completed.
 * @completion_status: This parameter specifies the results of the IO request
 *    operation.  SCI_SUCCESS indicates successful completion.
 *
 */
void scic_cb_io_request_complete(
	SCI_CONTROLLER_HANDLE_T controller,
	SCI_REMOTE_DEVICE_HANDLE_T remote_device,
	SCI_IO_REQUEST_HANDLE_T scic_io_request,
	enum sci_io_status completion_status)
{
	struct isci_request *request;
	struct isci_host *isci_host;

	isci_host =
		(struct isci_host *)sci_object_get_association(controller);

	request =
		(struct isci_request *)sci_object_get_association(
			scic_io_request
			);

	isci_request_io_request_complete(isci_host,
					 request,
					 completion_status);
}

/**
 * scic_cb_task_request_complete() - This user callback will inform the user
 *    that a task management request completed.
 * @controller: This parameter specifies the controller on which the task
 *    management request is completing.
 * @remote_device: This parameter specifies the remote device on which this
 *    task management request is completing.
 * @task_request: This parameter specifies the task management request that has
 *    completed.
 * @completion_status: This parameter specifies the results of the IO request
 *    operation.  SCI_SUCCESS indicates successful completion.
 *
 */
void scic_cb_task_request_complete(
	SCI_CONTROLLER_HANDLE_T controller,
	SCI_REMOTE_DEVICE_HANDLE_T remote_device,
	SCI_TASK_REQUEST_HANDLE_T scic_task_request,
	enum sci_task_status completion_status)
{
	struct isci_request *request;
	struct isci_host *isci_host;

	isci_host =
		(struct isci_host *)sci_object_get_association(controller);

	request =
		(struct isci_request *)sci_object_get_association(
			scic_task_request);

	isci_task_request_complete(isci_host, request, completion_status);
}

#ifndef SCI_GET_PHYSICAL_ADDRESS_OPTIMIZATION_ENABLED
/**
 * scic_cb_io_request_get_physical_address() - This callback method asks the
 *    user to provide the physical address for the supplied virtual address
 *    when building an io request object.
 * @controller: This parameter is the core controller object handle.
 * @io_request: This parameter is the io request object handle for which the
 *    physical address is being requested.
 *
 *
 */
void scic_cb_io_request_get_physical_address(
	SCI_CONTROLLER_HANDLE_T controller,
	SCI_IO_REQUEST_HANDLE_T io_request,
	void *virtual_address,
	dma_addr_t *physical_address)
{
	struct isci_host *isci_host =
		(struct isci_host *)sci_object_get_association(controller);

	struct isci_request *request =
		(struct isci_request *)sci_object_get_association(io_request);

	char *requested_address = (char *)virtual_address;
	char *base_address = (char *)request;

	dev_dbg(&isci_host->pdev->dev,
		"%s: isci_host = %p\n",
		__func__,
		isci_host);

	/*
	 * First check to see if the requested address was allocated as part
	 * of this specific isci_request. This is the most common usage.
	 */

	if (requested_address >= base_address
	    && (requested_address - base_address) < request->request_alloc_size) {

		*physical_address = request->request_daddr
				    + (requested_address - base_address);

	} else {

		/*
		 * Since there are only a handful of blocks of DMA'able
		 * memory we've allocated, we'll need to find which block
		 * this specific virtual_address belongs. I hate this
		 * because its a slow lookup in an io path. May need to
		 * create a hash or something in the controller object
		 * for this.
		 */

		struct coherent_memory_info *mdl_struct;
		struct list_head *curr_element;

		list_for_each(curr_element, &isci_host->mdl_struct_list) {
			mdl_struct = list_entry(curr_element,
						struct coherent_memory_info,
						node);

			base_address = (char *)mdl_struct->vaddr;

			if (requested_address >= base_address
			    && (requested_address - base_address)
			    < mdl_struct->size) {

				*physical_address = mdl_struct->dma_handle
						    + (requested_address
						       - base_address);

				return;
			}
		}

		/*
		 * if we drop down here, we're in trouble as the caller is
		 * probably trying to find the address of an SGL element. If
		 * that is the case, this is not the proper function to call,
		 * as the physical address is stored within the SGL element
		 */
		BUG();
	}
}
#endif

/**
 * scic_cb_io_request_get_transfer_length() - This callback method asks the
 *    user to provide the number of bytes to be transfered as part of this
 *    request.
 * @scic_user_io_request: This parameter points to the user's IO request
 *    object.  It is a cookie that allows the user to provide the necessary
 *    information for this callback.
 *
 * This method returns the number of payload data bytes to be transfered for
 * this IO request.
 */
u32 scic_cb_io_request_get_transfer_length(
	void *scic_user_io_request)
{
	return isci_request_io_request_get_transfer_length(
		       scic_user_io_request
		       );
}


/**
 * scic_cb_io_request_get_data_direction() - This callback method asks the user
 *    to provide the data direction for this request.
 * @scic_user_io_request: This parameter points to the user's IO request
 *    object.  It is a cookie that allows the user to provide the necessary
 *    information for this callback.
 *
 * This method returns the value of SCI_IO_REQUEST_DATA_OUT or
 * SCI_IO_REQUEST_DATA_IN, or SCI_IO_REQUEST_NO_DATA.
 */
SCI_IO_REQUEST_DATA_DIRECTION scic_cb_io_request_get_data_direction(
	void *scic_user_io_request)
{
	return isci_request_io_request_get_data_direction(
		       scic_user_io_request
		       );
}


#ifndef SCI_SGL_OPTIMIZATION_ENABLED
/**
 * scic_cb_io_request_get_next_sge() - This callback method asks the user to
 *    provide the address to where the next Scatter-Gather Element is located.
 * @scic_user_io_request: This parameter points to the user's IO request
 *    object.  It is a cookie that allows the user to provide the necessary
 *    information for this callback.
 * @current_sge_address: This parameter specifies the address for the current
 *    SGE (i.e. the one that has just processed).
 *
 * An address specifying the location for the next scatter gather element to be
 * processed.
 */
void scic_cb_io_request_get_next_sge(
	void *scic_user_io_request,
	void *current_sge_address,
	void **next_sge)
{
	*next_sge = isci_request_io_request_get_next_sge(
		scic_user_io_request,
		current_sge_address
		);
}
#endif

/**
 * scic_cb_sge_get_address_field() - This callback method asks the user to
 *    provide the contents of the "address" field in the Scatter-Gather Element.
 * @scic_user_io_request: This parameter points to the user's IO request
 *    object.  It is a cookie that allows the user to provide the necessary
 *    information for this callback.
 * @sge_address: This parameter specifies the address for the SGE from which to
 *    retrieve the address field.
 *
 * A physical address specifying the contents of the SGE's address field.
 */
dma_addr_t scic_cb_sge_get_address_field(
	void *scic_user_io_request,
	void *sge_address)
{
	return isci_request_sge_get_address_field(
		       scic_user_io_request,
		       sge_address
		       );
}

/**
 * scic_cb_sge_get_length_field() - This callback method asks the user to
 *    provide the contents of the "length" field in the Scatter-Gather Element.
 * @scic_user_io_request: This parameter points to the user's IO request
 *    object.  It is a cookie that allows the user to provide the necessary
 *    information for this callback.
 * @sge_address: This parameter specifies the address for the SGE from which to
 *    retrieve the address field.
 *
 * This method returns the length field specified inside the SGE referenced by
 * the sge_address parameter.
 */
u32 scic_cb_sge_get_length_field(
	void *scic_user_io_request,
	void *sge_address)
{
	return isci_request_sge_get_length_field(
		       scic_user_io_request,
		       sge_address
		       );
}

/**
 * scic_cb_ssp_io_request_get_cdb_address() - This callback method asks the
 *    user to provide the address for the command descriptor block (CDB)
 *    associated with this IO request.
 * @scic_user_io_request: This parameter points to the user's IO request
 *    object.  It is a cookie that allows the user to provide the necessary
 *    information for this callback.
 *
 * This method returns the virtual address of the CDB.
 */
void *scic_cb_ssp_io_request_get_cdb_address(
	void *scic_user_io_request)
{
	return isci_request_ssp_io_request_get_cdb_address(
		       scic_user_io_request
		       );
}

/**
 * scic_cb_ssp_io_request_get_cdb_length() - This callback method asks the user
 *    to provide the length of the command descriptor block (CDB) associated
 *    with this IO request.
 * @scic_user_io_request: This parameter points to the user's IO request
 *    object.  It is a cookie that allows the user to provide the necessary
 *    information for this callback.
 *
 * This method returns the length of the CDB.
 */
u32 scic_cb_ssp_io_request_get_cdb_length(
	void *scic_user_io_request)
{
	return isci_request_ssp_io_request_get_cdb_length(
		       scic_user_io_request
		       );
}

/**
 * scic_cb_ssp_io_request_get_lun() - This callback method asks the user to
 *    provide the Logical Unit (LUN) associated with this IO request.
 * @scic_user_io_request: This parameter points to the user's IO request
 *    object.  It is a cookie that allows the user to provide the necessary
 *    information for this callback.
 *
 * This method returns the LUN associated with this request. This should be u64?
 */
u32 scic_cb_ssp_io_request_get_lun(
	void *scic_user_io_request)
{
	return isci_request_ssp_io_request_get_lun(scic_user_io_request);
}

/**
 * scic_cb_ssp_io_request_get_task_attribute() - This callback method asks the
 *    user to provide the task attribute associated with this IO request.
 * @scic_user_io_request: This parameter points to the user's IO request
 *    object.  It is a cookie that allows the user to provide the necessary
 *    information for this callback.
 *
 * This method returns the task attribute associated with this IO request.
 */
u32 scic_cb_ssp_io_request_get_task_attribute(
	void *scic_user_io_request)
{
	return isci_request_ssp_io_request_get_task_attribute(
		       scic_user_io_request
		       );
}

/**
 * scic_cb_ssp_io_request_get_command_priority() - This callback method asks
 *    the user to provide the command priority associated with this IO request.
 * @scic_user_io_request: This parameter points to the user's IO request
 *    object.  It is a cookie that allows the user to provide the necessary
 *    information for this callback.
 *
 * This method returns the command priority associated with this IO request.
 */
u32 scic_cb_ssp_io_request_get_command_priority(
	void *scic_user_io_request)
{
	return isci_request_ssp_io_request_get_command_priority(
		       scic_user_io_request
		       );
}

/**
 * scic_cb_ssp_task_request_get_lun() - This method returns the Logical Unit to
 *    be utilized for this task management request.
 * @scic_user_task_request: This parameter points to the user's task request
 *    object.  It is a cookie that allows the user to provide the necessary
 *    information for this callback.
 *
 * This method returns the LUN associated with this request. This should be u64?
 */
u32 scic_cb_ssp_task_request_get_lun(
	void *scic_user_task_request)
{
	return isci_task_ssp_request_get_lun(
		       (struct isci_request *)scic_user_task_request
		       );
}

/**
 * scic_cb_ssp_task_request_get_function() - This method returns the task
 *    management function to be utilized for this task request.
 * @scic_user_task_request: This parameter points to the user's task request
 *    object.  It is a cookie that allows the user to provide the necessary
 *    information for this callback.
 *
 * This method returns an unsigned byte representing the task management
 * function to be performed.
 */
u8 scic_cb_ssp_task_request_get_function(
	void *scic_user_task_request)
{
	return isci_task_ssp_request_get_function(
		       (struct isci_request *)scic_user_task_request
		       );
}

/**
 * scic_cb_ssp_task_request_get_io_tag_to_manage() - This method returns the
 *    task management IO tag to be managed. Depending upon the task management
 *    function the value returned from this method may be ignored.
 * @scic_user_task_request: This parameter points to the user's task request
 *    object.  It is a cookie that allows the user to provide the necessary
 *    information for this callback.
 *
 * This method returns an unsigned 16-bit word depicting the IO tag to be
 * managed.
 */
u16 scic_cb_ssp_task_request_get_io_tag_to_manage(
	void *scic_user_task_request)
{
	return isci_task_ssp_request_get_io_tag_to_manage(
		       (struct isci_request *)scic_user_task_request
		       );
}

/**
 * scic_cb_ssp_task_request_get_response_data_address() - This callback method
 *    asks the user to provide the virtual address of the response data buffer
 *    for the supplied IO request.
 * @scic_user_task_request: This parameter points to the user's task request
 *    object.  It is a cookie that allows the user to provide the necessary
 *    information for this callback.
 *
 * This method returns the virtual address for the response data buffer
 * associated with this IO request.
 */
void *scic_cb_ssp_task_request_get_response_data_address(
	void *scic_user_task_request)
{
	return isci_task_ssp_request_get_response_data_address(
		       (struct isci_request *)scic_user_task_request
		       );
}

/**
 * scic_cb_ssp_task_request_get_response_data_length() - This callback method
 *    asks the user to provide the length of the response data buffer for the
 *    supplied IO request.
 * @scic_user_task_request: This parameter points to the user's task request
 *    object.  It is a cookie that allows the user to provide the necessary
 *    information for this callback.
 *
 * This method returns the length of the response buffer data associated with
 * this IO request.
 */
u32 scic_cb_ssp_task_request_get_response_data_length(
	void *scic_user_task_request)
{
	return isci_task_ssp_request_get_response_data_length(
		       (struct isci_request *)scic_user_task_request
		       );
}

#if !defined(DISABLE_ATAPI)
/**
 * scic_cb_stp_packet_io_request_get_cdb_address() - This user callback asks
 *    the user to provide stp packet io's the CDB address.
 * @scic_user_io_request:
 *
 * The packet IO's cdb adress.
 */
void *scic_cb_stp_packet_io_request_get_cdb_address(
	void *scic_user_io_request)
{
	return isci_request_stp_packet_io_request_get_cdb_address(
		       scic_user_io_request
		       );
}


/**
 * scic_cb_stp_packet_io_request_get_cdb_length() - This user callback asks the
 *    user to provide stp packet io's the CDB length.
 * @scic_user_io_request:
 *
 * The packet IO's cdb length.
 */
u32 scic_cb_stp_packet_io_request_get_cdb_length(
	void *scic_user_io_request)
{
	return isci_request_stp_packet_io_request_get_cdb_length(
		       scic_user_io_request
		       );
}
#endif /* #if !defined(DISABLE_ATAPI) */

/**
 * scic_cb_pci_get_bar() - In this method the user must return the base address
 *    register (BAR) value for the supplied base address register number.
 * @controller: The controller for which to retrieve the bar number.
 * @bar_number: This parameter depicts the BAR index/number to be read.
 *
 * Return a pointer value indicating the contents of the BAR. NULL indicates an
 * invalid BAR index/number was specified. All other values indicate a valid
 * VIRTUAL address from the BAR.
 */
void *scic_cb_pci_get_bar(
	SCI_CONTROLLER_HANDLE_T controller,
	u16 bar_num)
{
	struct isci_host *isci_host;

	isci_host =
		(struct isci_host *)sci_object_get_association(controller);

	return isci_host_pci_get_bar(isci_host, bar_num);

}

/**
 * scic_cb_pci_read_dword() - In this method the user must read from PCI memory.
 * @controller: The controller for which to read a DWORD.
 * @address: This parameter depicts the address from which to read.
 *
 * The value being returned from the PCI memory location. This PCI memory
 * access calls likely need to be optimized into macro?
 */
u32 scic_cb_pci_read_dword(
	SCI_CONTROLLER_HANDLE_T controller,
	void *address)
{
	u32 ret;

	ret = readl(address);

	return ret;
}

/**
 * scic_cb_pci_write_dword() - In this method the user must write to PCI memory.
 * @controller: The controller for which to read a DWORD.
 * @address: This parameter depicts the address into which to write.
 * @write_value: This parameter depicts the value being written into the PCI
 *    memory location.
 *
 * This PCI memory access calls likely need to be optimized into macro?
 */
void scic_cb_pci_write_dword(
	SCI_CONTROLLER_HANDLE_T controller,
	void *address,
	u32 write_value)
{
	writel(write_value, address);
}

/**
 * scic_cb_port_stop_complete() - This method informs the user when a stop
 *    operation on the port has completed.
 * @controller: This parameter represents the controller which contains the
 *    port.
 * @port: This parameter specifies the SCI port object for which the callback
 *    is being invoked.
 * @completion_status: This parameter specifies the status for the operation
 *    being completed.
 *
 */
void scic_cb_port_stop_complete(
	SCI_CONTROLLER_HANDLE_T controller,
	SCI_PORT_HANDLE_T port,
	enum sci_status completion_status)
{
	pr_warn("%s:************************************************\n",
		__func__);
}

/**
 * scic_cb_port_hard_reset_complete() - This method informs the user when a
 *    hard reset on the port has completed.  This hard reset could have been
 *    initiated by the user or by the remote port.
 * @controller: This parameter represents the controller which contains the
 *    port.
 * @port: This parameter specifies the SCI port object for which the callback
 *    is being invoked.
 * @completion_status: This parameter specifies the status for the operation
 *    being completed.
 *
 */
void scic_cb_port_hard_reset_complete(
	SCI_CONTROLLER_HANDLE_T controller,
	SCI_PORT_HANDLE_T port,
	enum sci_status completion_status)
{
	struct isci_port *isci_port
		= (struct isci_port *)sci_object_get_association(port);

	isci_port_hard_reset_complete(isci_port, completion_status);
}

/**
 * scic_cb_port_ready() - This method informs the user that the port is now in
 *    a ready state and can be utilized to issue IOs.
 * @controller: This parameter represents the controller which contains the
 *    port.
 * @port: This parameter specifies the SCI port object for which the callback
 *    is being invoked.
 *
 */
void scic_cb_port_ready(
	SCI_CONTROLLER_HANDLE_T controller,
	SCI_PORT_HANDLE_T port)
{
	struct isci_port *isci_port;
	struct isci_host *isci_host;

	isci_host =
		(struct isci_host *)sci_object_get_association(controller);

	isci_port =
		(struct isci_port *)sci_object_get_association(port);

	dev_dbg(&isci_host->pdev->dev,
		"%s: isci_port = %p\n", __func__, isci_port);

	isci_port_ready(isci_host, isci_port);
}

/**
 * scic_cb_port_not_ready() - This method informs the user that the port is now
 *    not in a ready (i.e. busy) state and can't be utilized to issue IOs.
 * @controller: This parameter represents the controller which contains the
 *    port.
 * @port: This parameter specifies the SCI port object for which the callback
 *    is being invoked.
 *
 */
void scic_cb_port_not_ready(
	SCI_CONTROLLER_HANDLE_T controller,
	SCI_PORT_HANDLE_T port,
	u32 reason_code)
{
	struct isci_port *isci_port;
	struct isci_host *isci_host;

	isci_host =
		(struct isci_host *)sci_object_get_association(controller);

	isci_port =
		(struct isci_port *)sci_object_get_association(port);

	dev_dbg(&isci_host->pdev->dev,
		"%s: isci_port = %p\n", __func__, isci_port);

	isci_port_not_ready(isci_host, isci_port);
}

/**
 * scic_cb_port_invalid_link_up() - This method informs the SCI Core user that
 *    a phy/link became ready, but the phy is not allowed in the port.  In some
 *    situations the underlying hardware only allows for certain phy to port
 *    mappings.  If these mappings are violated, then this API is invoked.
 * @controller: This parameter represents the controller which contains the
 *    port.
 * @port: This parameter specifies the SCI port object for which the callback
 *    is being invoked.
 * @phy: This parameter specifies the phy that came ready, but the phy can't be
 *    a valid member of the port.
 *
 */
void scic_cb_port_invalid_link_up(
	SCI_CONTROLLER_HANDLE_T controller,
	SCI_PORT_HANDLE_T port,
	SCI_PHY_HANDLE_T phy)
{
	pr_warn("%s:************************************************\n",
		__func__);
}

/**
 * scic_cb_port_bc_change_primitive_recieved() - This callback method informs
 *    the user that a broadcast change primitive was received.
 * @controller: This parameter represents the controller which contains the
 *    port.
 * @port: This parameter specifies the SCI port object for which the callback
 *    is being invoked.  For instances where the phy on which the primitive was
 *    received is not part of a port, this parameter will be
 *    SCI_INVALID_HANDLE_T.
 * @phy: This parameter specifies the phy on which the primitive was received.
 *
 */
void scic_cb_port_bc_change_primitive_recieved(
	SCI_CONTROLLER_HANDLE_T controller,
	SCI_PORT_HANDLE_T port,
	SCI_PHY_HANDLE_T phy)
{
	struct isci_host *isci_host;

	isci_host =
		(struct isci_host *)sci_object_get_association(controller);

	dev_dbg(&isci_host->pdev->dev,
		"%s: port = %p, phy = %p\n", __func__, port, phy);
	isci_port_bc_change_recieved(isci_host, port, phy);
}




/**
 * scic_cb_port_link_up() - This callback method informs the user that a phy
 *    has become operational and is capable of communicating with the remote
 *    end point.
 * @controller: This parameter represents the controller associated with the
 *    phy.
 * @port: This parameter specifies the port object for which the user callback
 *    is being invoked.  There may be conditions where this parameter can be
 *    SCI_INVALID_HANDLE
 * @phy: This parameter specifies the phy object for which the user callback is
 *    being invoked.
 *
 * none.
 */
void scic_cb_port_link_up(
	SCI_CONTROLLER_HANDLE_T controller,
	SCI_PORT_HANDLE_T port,
	SCI_PHY_HANDLE_T phy)
{
	struct isci_host *isci_host;

	isci_host =
		(struct isci_host *)sci_object_get_association(controller);

	dev_dbg(&isci_host->pdev->dev,
		"%s: phy = %p\n", __func__, phy);

	isci_port_link_up(isci_host, port, phy);
}

/**
 * scic_cb_port_link_down() - This callback method informs the user that a phy
 *    is no longer operational and is not capable of communicating with the
 *    remote end point.
 * @controller: This parameter represents the controller associated with the
 *    phy.
 * @port: This parameter specifies the port object for which the user callback
 *    is being invoked.  There may be conditions where this parameter can be
 *    SCI_INVALID_HANDLE
 * @phy: This parameter specifies the phy object for which the user callback is
 *    being invoked.
 *
 * none.
 */
void scic_cb_port_link_down(
	SCI_CONTROLLER_HANDLE_T controller,
	SCI_PORT_HANDLE_T port,
	SCI_PHY_HANDLE_T phy)
{
	struct isci_host *isci_host;
	struct isci_phy *isci_phy;
	struct isci_port *isci_port;

	isci_host =
		(struct isci_host *)sci_object_get_association(controller);

	isci_phy =
		(struct isci_phy *)sci_object_get_association(phy);

	isci_port =
		(struct isci_port *)sci_object_get_association(port);

	dev_dbg(&isci_host->pdev->dev,
		"%s: isci_port = %p\n", __func__, isci_port);

	isci_port_link_down(isci_host, isci_phy, isci_port);
}

/**
 * scic_cb_remote_device_start_complete() - This user callback method will
 *    inform the user that a start operation has completed.
 * @controller: This parameter specifies the core controller associated with
 *    the completion callback.
 * @remote_device: This parameter specifies the remote device associated with
 *    the completion callback.
 * @completion_status: This parameter specifies the completion status for the
 *    operation.
 *
 */
void scic_cb_remote_device_start_complete(
	SCI_CONTROLLER_HANDLE_T controller,
	SCI_REMOTE_DEVICE_HANDLE_T remote_device,
	enum sci_status completion_status)
{
	struct isci_host *isci_host;
	struct isci_remote_device *isci_device;

	isci_host =
		(struct isci_host *)sci_object_get_association(controller);

	isci_device =
		(struct isci_remote_device *)sci_object_get_association(
			remote_device
			);

	dev_dbg(&isci_host->pdev->dev,
		"%s: isci_device = %p\n", __func__, isci_device);

	isci_remote_device_start_complete(
		isci_host, isci_device, completion_status);

}

/**
 * scic_cb_remote_device_stop_complete() - This user callback method will
 *    inform the user that a stop operation has completed.
 * @controller: This parameter specifies the core controller associated with
 *    the completion callback.
 * @remote_device: This parameter specifies the remote device associated with
 *    the completion callback.
 * @completion_status: This parameter specifies the completion status for the
 *    operation.
 *
 */
void scic_cb_remote_device_stop_complete(
	SCI_CONTROLLER_HANDLE_T controller,
	SCI_REMOTE_DEVICE_HANDLE_T remote_device,
	enum sci_status completion_status)
{
	struct isci_host *isci_host;
	struct isci_remote_device *isci_device;

	isci_host =
		(struct isci_host *)sci_object_get_association(controller);

	isci_device =
		(struct isci_remote_device *)sci_object_get_association(
			remote_device
			);

	dev_dbg(&isci_host->pdev->dev,
		"%s: isci_device = %p\n", __func__, isci_device);

	isci_remote_device_stop_complete(
		isci_host, isci_device, completion_status);

}

/**
 * scic_cb_remote_device_ready() - This user callback method will inform the
 *    user that a remote device is now capable of handling IO requests.
 * @controller: This parameter specifies the core controller associated with
 *    the completion callback.
 * @remote_device: This parameter specifies the remote device associated with
 *    the callback.
 *
 */
void scic_cb_remote_device_ready(
	SCI_CONTROLLER_HANDLE_T controller,
	SCI_REMOTE_DEVICE_HANDLE_T remote_device)
{
	struct isci_remote_device *isci_device =
		(struct isci_remote_device *)
		sci_object_get_association(remote_device);

	dev_dbg(&isci_device->isci_port->isci_host->pdev->dev,
		"%s: isci_device = %p\n", __func__, isci_device);

	isci_remote_device_ready(isci_device);
}

/**
 * scic_cb_remote_device_not_ready() - This user callback method will inform
 *    the user that a remote device is no longer capable of handling IO
 *    requests (until a ready callback is invoked).
 * @controller: This parameter specifies the core controller associated with
 *    the completion callback.
 * @remote_device: This parameter specifies the remote device associated with
 *    the callback.
 * @reason_code: This parameter specifies the reason for the remote device
 *    going to a not ready state.
 *
 */
void scic_cb_remote_device_not_ready(
	SCI_CONTROLLER_HANDLE_T controller,
	SCI_REMOTE_DEVICE_HANDLE_T remote_device,
	u32 reason_code)
{
	struct isci_remote_device *isci_device =
		(struct isci_remote_device *)
		sci_object_get_association(remote_device);

	struct isci_host *isci_host;

	isci_host =
		(struct isci_host *)sci_object_get_association(controller);

	dev_dbg(&isci_host->pdev->dev,
		"%s: isci_device = %p, reason_code = %x\n",
		__func__, isci_device, reason_code);

	isci_remote_device_not_ready(isci_device, reason_code);
}

/**
 * scic_cb_io_request_do_copy_rx_frames() - This callback method asks the user
 *    if the received RX frame data is to be copied to the SGL or should be
 *    stored by the SCI core to be retrieved later with the
 *    scic_io_request_get_rx_frame().
 * @scic_user_io_request: This parameter points to the user's IO request
 *    object.  It is a cookie that allows the user to provide the necessary
 *    information for this callback.
 *
 * This method returns true if the SCI core should copy the received frame data
 * to the SGL location or false if the SCI user wants to retrieve the frame
 * data at a later time.
 */
bool scic_cb_io_request_do_copy_rx_frames(
	void *scic_user_io_request)
{
	struct sas_task *task
		= isci_request_access_task(
		(struct isci_request *)scic_user_io_request
		);

	return (task->data_dir == DMA_NONE) ? false : true;
}

/**
 * scic_cb_get_virtual_address() - This callback method asks the user to
 *    provide the virtual address for the supplied physical address.
 * @controller: This parameter is the core controller object handle.
 * @physical_address: This parameter is the physical address which is to be
 *    returned as a virtual address.
 *
 * The method returns the virtual address for the supplied physical address.
 */
void *scic_cb_get_virtual_address(
	SCI_CONTROLLER_HANDLE_T controller,
	dma_addr_t physical_address)
{
	void *virt_addr = (void *)phys_to_virt(physical_address);

	return virt_addr;
}

/**
 * scic_cb_request_get_sat_protocol() - This callback method asks the user to
 *    return the SAT protocol definition for this IO request.  This method is
 *    only called by the SCI core if the request type constructed is SATA.
 * @scic_user_io_request: This parameter points to the user's IO request
 *    object.  It is a cookie that allows the user to provide the necessary
 *    information for this callback.
 *
 * This method returns one of the sat.h defined protocols for the given io
 * request.
 */
u8 scic_cb_request_get_sat_protocol(
	void *scic_user_io_request)
{
	return isci_sata_get_sat_protocol(
		       (struct isci_request *)scic_user_io_request
		       );
}
