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
 * This file contains the struct scic_sds_controller state handler implementation for
 *    the base state machine.
 *
 *
 */

#include "scic_phy.h"
#include "scic_port.h"
#include "scic_controller.h"
#include "scic_sds_pci.h"
#include "scic_sds_remote_device.h"
#include "scic_sds_request.h"
#include "scic_sds_controller.h"
#include "scic_sds_controller_registers.h"
#include "sci_util.h"
#include "sci_environment.h"

#define SCU_CONTEXT_RAM_INIT_STALL_TIME      200

/**
 * smu_dcc_get_max_ports() -
 *
 * This macro returns the maximum number of logical ports supported by the
 * hardware. The caller passes in the value read from the device context
 * capacity register and this macro will mash and shift the value appropriately.
 */
#define smu_dcc_get_max_ports(dcc_value) \
	(\
		(((dcc_value) & SMU_DEVICE_CONTEXT_CAPACITY_MAX_LP_MASK) \
		 >> SMU_DEVICE_CONTEXT_CAPACITY_MAX_LP_SHIFT) + 1 \
	)

/**
 * smu_dcc_get_max_task_context() -
 *
 * This macro returns the maximum number of task contexts supported by the
 * hardware. The caller passes in the value read from the device context
 * capacity register and this macro will mash and shift the value appropriately.
 */
#define smu_dcc_get_max_task_context(dcc_value)	\
	(\
		(((dcc_value) & SMU_DEVICE_CONTEXT_CAPACITY_MAX_TC_MASK) \
		 >> SMU_DEVICE_CONTEXT_CAPACITY_MAX_TC_SHIFT) + 1 \
	)

/**
 * smu_dcc_get_max_remote_node_context() -
 *
 * This macro returns the maximum number of remote node contexts supported by
 * the hardware. The caller passes in the value read from the device context
 * capacity register and this macro will mash and shift the value appropriately.
 */
#define smu_dcc_get_max_remote_node_context(dcc_value) \
	(\
		(((dcc_value) & SMU_DEVICE_CONTEXT_CAPACITY_MAX_RNC_MASK) \
		 >> SMU_DEVICE_CONTEXT_CAPACITY_MAX_RNC_SHIFT) + 1 \
	)

/*
 * *****************************************************************************
 * * DEFAULT STATE HANDLERS
 * ***************************************************************************** */

/**
 *
 * @controller: This is struct sci_base_controller object which is cast into a
 *    struct scic_sds_controller object.
 * @remote_device: This is struct sci_base_remote_device which, if it was used, would
 *    be cast to a struct scic_sds_remote_device.
 * @io_request: This is the struct sci_base_request which, if it was used, would be
 *    cast to a SCIC_SDS_IO_REQUEST.
 * @io_tag: This is the IO tag to be assigned to the IO request or
 *    SCI_CONTROLLER_INVALID_IO_TAG.
 *
 * This method is called when the struct scic_sds_controller default start io/task
 * handler is in place. - Issue a warning message enum sci_status
 * SCI_FAILURE_INVALID_STATE
 */
static enum sci_status scic_sds_controller_default_start_operation_handler(
	struct sci_base_controller *controller,
	struct sci_base_remote_device *remote_device,
	struct sci_base_request *io_request,
	u16 io_tag)
{
	struct scic_sds_controller *this_controller;

	this_controller = (struct scic_sds_controller *)controller;

	dev_warn(scic_to_dev(this_controller),
		 "%s: SCIC Controller requested to start an io/task from "
		 "invalid state %d\n",
		 __func__,
		 sci_base_state_machine_get_state(
			 scic_sds_controller_get_base_state_machine(
				 this_controller)));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @controller: This is struct sci_base_controller object which is cast into a
 *    struct scic_sds_controller object.
 * @remote_device: This is struct sci_base_remote_device which, if it was used, would
 *    be cast to a struct scic_sds_remote_device.
 * @io_request: This is the struct sci_base_request which, if it was used, would be
 *    cast to a SCIC_SDS_IO_REQUEST.
 *
 * This method is called when the struct scic_sds_controller default request handler
 * is in place. - Issue a warning message enum sci_status SCI_FAILURE_INVALID_STATE
 */
static enum sci_status scic_sds_controller_default_request_handler(
	struct sci_base_controller *controller,
	struct sci_base_remote_device *remote_device,
	struct sci_base_request *io_request)
{
	struct scic_sds_controller *this_controller;

	this_controller = (struct scic_sds_controller *)controller;

	dev_warn(scic_to_dev(this_controller),
		"%s: SCIC Controller request operation from invalid state %d\n",
		__func__,
		sci_base_state_machine_get_state(
			scic_sds_controller_get_base_state_machine(
				this_controller)));

	return SCI_FAILURE_INVALID_STATE;
}

/*
 * *****************************************************************************
 * * GENERAL (COMMON) STATE HANDLERS
 * ***************************************************************************** */

/**
 *
 * @controller: The struct sci_base_controller object which is cast into a
 *    struct scic_sds_controller object.
 *
 * This method is called when the struct scic_sds_controller is in the ready state
 * reset handler is in place. - Transition to
 * SCI_BASE_CONTROLLER_STATE_RESETTING enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_controller_general_reset_handler(
	struct sci_base_controller *controller)
{
	struct scic_sds_controller *this_controller;

	this_controller = (struct scic_sds_controller *)controller;

	/*
	 * The reset operation is not a graceful cleanup just perform the state
	 * transition. */
	sci_base_state_machine_change_state(
		scic_sds_controller_get_base_state_machine(this_controller),
		SCI_BASE_CONTROLLER_STATE_RESETTING
		);

	return SCI_SUCCESS;
}

/*
 * *****************************************************************************
 * * RESET STATE HANDLERS
 * ***************************************************************************** */

/**
 *
 * @controller: This is the struct sci_base_controller object which is cast into a
 *    struct scic_sds_controller object.
 *
 * This method is the struct scic_sds_controller initialize handler for the reset
 * state. - Currently this function does nothing enum sci_status SCI_FAILURE This
 * function is not yet implemented and is a valid request from the reset state.
 */
static enum sci_status scic_sds_controller_reset_state_initialize_handler(
	struct sci_base_controller *controller)
{
	u32 index;
	enum sci_status result = SCI_SUCCESS;
	struct scic_sds_controller *this_controller;

	this_controller = (struct scic_sds_controller *)controller;

	sci_base_state_machine_change_state(
		scic_sds_controller_get_base_state_machine(this_controller),
		SCI_BASE_CONTROLLER_STATE_INITIALIZING
		);

	this_controller->timeout_timer = scic_cb_timer_create(
		this_controller,
		(void (*)(void *))scic_sds_controller_timeout_handler,
		(void (*)(void *))controller);

	scic_sds_controller_initialize_phy_startup(this_controller);

	scic_sds_controller_initialize_power_control(this_controller);

	/*
	 * There is nothing to do here for B0 since we do not have to
	 * program the AFE registers.
	 * / @todo The AFE settings are supposed to be correct for the B0 but
	 * /       presently they seem to be wrong. */
	scic_sds_controller_afe_initialization(this_controller);

	if (SCI_SUCCESS == result) {
		u32 status;
		u32 terminate_loop;

		/* Take the hardware out of reset */
		SMU_SMUSRCR_WRITE(this_controller, 0x00000000);

		/*
		 * / @todo Provide meaningfull error code for hardware failure
		 * result = SCI_FAILURE_CONTROLLER_HARDWARE; */
		result = SCI_FAILURE;
		terminate_loop = 100;

		while (terminate_loop-- && (result != SCI_SUCCESS)) {
			/* Loop until the hardware reports success */
			scic_cb_stall_execution(SCU_CONTEXT_RAM_INIT_STALL_TIME);
			status = SMU_SMUCSR_READ(this_controller);

			if ((status & SCU_RAM_INIT_COMPLETED) == SCU_RAM_INIT_COMPLETED) {
				result = SCI_SUCCESS;
			}
		}
	}

	if (result == SCI_SUCCESS) {
		u32 max_supported_ports;
		u32 max_supported_devices;
		u32 max_supported_io_requests;
		u32 device_context_capacity;

		/*
		 * Determine what are the actaul device capacities that the
		 * hardware will support */
		device_context_capacity = SMU_DCC_READ(this_controller);

		max_supported_ports =
			smu_dcc_get_max_ports(device_context_capacity);
		max_supported_devices =
			smu_dcc_get_max_remote_node_context(device_context_capacity);
		max_supported_io_requests =
			smu_dcc_get_max_task_context(device_context_capacity);

		/* Make all PEs that are unassigned match up with the logical ports */
		for (index = 0; index < max_supported_ports; index++) {
			scu_register_write(
				this_controller,
				this_controller->scu_registers->peg0.ptsg.protocol_engine[index],
				index
				);
		}

		/* Record the smaller of the two capacity values */
		this_controller->logical_port_entries =
			min(max_supported_ports, this_controller->logical_port_entries);

		this_controller->task_context_entries =
			min(max_supported_io_requests, this_controller->task_context_entries);

		this_controller->remote_node_entries =
			min(max_supported_devices, this_controller->remote_node_entries);

		/*
		 * Now that we have the correct hardware reported minimum values
		 * build the MDL for the controller.  Default to a performance
		 * configuration. */
		scic_controller_set_mode(this_controller, SCI_MODE_SPEED);
	}

	/* Initialize hardware PCI Relaxed ordering in DMA engines */
	if (result == SCI_SUCCESS) {
		u32 dma_configuration;

		/* Configure the payload DMA */
		dma_configuration = SCU_PDMACR_READ(this_controller);
		dma_configuration |= SCU_PDMACR_GEN_BIT(PCI_RELAXED_ORDERING_ENABLE);
		SCU_PDMACR_WRITE(this_controller, dma_configuration);

		/* Configure the control DMA */
		dma_configuration = SCU_CDMACR_READ(this_controller);
		dma_configuration |= SCU_CDMACR_GEN_BIT(PCI_RELAXED_ORDERING_ENABLE);
		SCU_CDMACR_WRITE(this_controller, dma_configuration);
	}

	/*
	 * Initialize the PHYs before the PORTs because the PHY registers
	 * are accessed during the port initialization. */
	if (result == SCI_SUCCESS) {
		/* Initialize the phys */
		for (index = 0;
		     (result == SCI_SUCCESS) && (index < SCI_MAX_PHYS);
		     index++) {
			result = scic_sds_phy_initialize(
				&this_controller->phy_table[index],
				&this_controller->scu_registers->peg0.pe[index].ll
				);
		}
	}

	if (result == SCI_SUCCESS) {
		/* Initialize the logical ports */
		for (index = 0;
		     (index < this_controller->logical_port_entries)
		     && (result == SCI_SUCCESS);
		     index++) {
			result = scic_sds_port_initialize(
				&this_controller->port_table[index],
				&this_controller->scu_registers->peg0.pe[index].tl,
				&this_controller->scu_registers->peg0.ptsg.port[index],
				&this_controller->scu_registers->peg0.ptsg.protocol_engine,
				&this_controller->scu_registers->peg0.viit[index]
				);
		}
	}

	if (SCI_SUCCESS == result) {
		result = scic_sds_port_configuration_agent_initialize(
			this_controller,
			&this_controller->port_agent
			);
	}

	/* Advance the controller state machine */
	if (result == SCI_SUCCESS) {
		sci_base_state_machine_change_state(
			scic_sds_controller_get_base_state_machine(this_controller),
			SCI_BASE_CONTROLLER_STATE_INITIALIZED
			);
	} else {
		sci_base_state_machine_change_state(
			scic_sds_controller_get_base_state_machine(this_controller),
			SCI_BASE_CONTROLLER_STATE_FAILED
			);
	}

	return result;
}

/*
 * *****************************************************************************
 * * INITIALIZED STATE HANDLERS
 * ***************************************************************************** */

/**
 *
 * @controller: This is the struct sci_base_controller object which is cast into a
 *    struct scic_sds_controller object.
 * @timeout: This is the allowed time for the controller object to reach the
 *    started state.
 *
 * This method is the struct scic_sds_controller start handler for the initialized
 * state. - Validate we have a good memory descriptor table - Initialze the
 * physical memory before programming the hardware - Program the SCU hardware
 * with the physical memory addresses passed in the memory descriptor table. -
 * Initialzie the TCi pool - Initialize the RNi pool - Initialize the
 * completion queue - Initialize the unsolicited frame data - Take the SCU port
 * task scheduler out of reset - Start the first phy object. - Transition to
 * SCI_BASE_CONTROLLER_STATE_STARTING. enum sci_status SCI_SUCCESS if all of the
 * controller start operations complete
 * SCI_FAILURE_UNSUPPORTED_INFORMATION_FIELD if one or more of the memory
 * descriptor fields is invalid.
 */
static enum sci_status scic_sds_controller_initialized_state_start_handler(
	struct sci_base_controller *controller,
	u32 timeout)
{
	u16 index;
	enum sci_status result;
	struct scic_sds_controller *this_controller;

	this_controller = (struct scic_sds_controller *)controller;

	/* Make sure that the SCI User filled in the memory descriptor table correctly */
	result = scic_sds_controller_validate_memory_descriptor_table(this_controller);

	if (result == SCI_SUCCESS) {
		/* The memory descriptor list looks good so program the hardware */
		scic_sds_controller_ram_initialization(this_controller);
	}

	if (SCI_SUCCESS == result) {
		/* Build the TCi free pool */
		sci_pool_initialize(this_controller->tci_pool);
		for (index = 0; index < this_controller->task_context_entries; index++) {
			sci_pool_put(this_controller->tci_pool, index);
		}

		/* Build the RNi free pool */
		scic_sds_remote_node_table_initialize(
			&this_controller->available_remote_nodes,
			this_controller->remote_node_entries
			);
	}

	if (SCI_SUCCESS == result) {
		/*
		 * Before anything else lets make sure we will not be interrupted
		 * by the hardware. */
		scic_controller_disable_interrupts(this_controller);

		/* Enable the port task scheduler */
		scic_sds_controller_enable_port_task_scheduler(this_controller);

		/* Assign all the task entries to this controller physical function */
		scic_sds_controller_assign_task_entries(this_controller);

		/* Now initialze the completion queue */
		scic_sds_controller_initialize_completion_queue(this_controller);

		/* Initialize the unsolicited frame queue for use */
		scic_sds_controller_initialize_unsolicited_frame_queue(this_controller);
	}

	if (SCI_SUCCESS == result) {
		scic_sds_controller_start_next_phy(this_controller);

		scic_cb_timer_start(this_controller,
				    this_controller->timeout_timer,
				    timeout);

		sci_base_state_machine_change_state(
			scic_sds_controller_get_base_state_machine(this_controller),
			SCI_BASE_CONTROLLER_STATE_STARTING
			);
	}

	return result;
}

/*
 * *****************************************************************************
 * * INITIALIZED STATE HANDLERS
 * ***************************************************************************** */

/**
 *
 * @controller: This is struct scic_sds_controller which receives the link up
 *    notification.
 * @port: This is struct scic_sds_port with which the phy is associated.
 * @phy: This is the struct scic_sds_phy which has gone link up.
 *
 * This method is called when the struct scic_sds_controller is in the starting state
 * link up handler is called.  This method will perform the following: - Stop
 * the phy timer - Start the next phy - Report the link up condition to the
 * port object none
 */
static void scic_sds_controller_starting_state_link_up_handler(
	struct scic_sds_controller *this_controller,
	struct scic_sds_port *port,
	struct scic_sds_phy *phy)
{
	scic_sds_controller_phy_timer_stop(this_controller);

	this_controller->port_agent.link_up_handler(
		this_controller, &this_controller->port_agent, port, phy
		);
	/* scic_sds_port_link_up(port, phy); */

	scic_sds_controller_start_next_phy(this_controller);
}

/**
 *
 * @controller: This is struct scic_sds_controller which receives the link down
 *    notification.
 * @port: This is struct scic_sds_port with which the phy is associated.
 * @phy: This is the struct scic_sds_phy which has gone link down.
 *
 * This method is called when the struct scic_sds_controller is in the starting state
 * link down handler is called. - Report the link down condition to the port
 * object none
 */
static void scic_sds_controller_starting_state_link_down_handler(
	struct scic_sds_controller *this_controller,
	struct scic_sds_port *port,
	struct scic_sds_phy *phy)
{
	this_controller->port_agent.link_down_handler(
		this_controller, &this_controller->port_agent, port, phy
		);
	/* scic_sds_port_link_down(port, phy); */
}

/*
 * *****************************************************************************
 * * READY STATE HANDLERS
 * ***************************************************************************** */

/**
 *
 * @controller: The struct sci_base_controller object which is cast into a
 *    struct scic_sds_controller object.
 * @timeout: The timeout for when the stop operation should report a failure.
 *
 * This method is called when the struct scic_sds_controller is in the ready state
 * stop handler is called. - Start the timeout timer - Transition to
 * SCI_BASE_CONTROLLER_STATE_STOPPING. enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_controller_ready_state_stop_handler(
	struct sci_base_controller *controller,
	u32 timeout)
{
	struct scic_sds_controller *this_controller;

	this_controller = (struct scic_sds_controller *)controller;

	scic_cb_timer_start(this_controller,
			    this_controller->timeout_timer,
			    timeout);

	sci_base_state_machine_change_state(
		scic_sds_controller_get_base_state_machine(this_controller),
		SCI_BASE_CONTROLLER_STATE_STOPPING
		);

	return SCI_SUCCESS;
}

/**
 *
 * @controller: This is struct sci_base_controller object which is cast into a
 *    struct scic_sds_controller object.
 * @remote_device: This is struct sci_base_remote_device which is cast to a
 *    struct scic_sds_remote_device object.
 * @io_request: This is the struct sci_base_request which is cast to a
 *    SCIC_SDS_IO_REQUEST object.
 * @io_tag: This is the IO tag to be assigned to the IO request or
 *    SCI_CONTROLLER_INVALID_IO_TAG.
 *
 * This method is called when the struct scic_sds_controller is in the ready state and
 * the start io handler is called. - Start the io request on the remote device
 * - if successful - assign the io_request to the io_request_table - post the
 * request to the hardware enum sci_status SCI_SUCCESS if the start io operation
 * succeeds SCI_FAILURE_INSUFFICIENT_RESOURCES if the IO tag could not be
 * allocated for the io request. SCI_FAILURE_INVALID_STATE if one or more
 * objects are not in a valid state to accept io requests. How does the io_tag
 * parameter get assigned to the io request?
 */
static enum sci_status scic_sds_controller_ready_state_start_io_handler(
	struct sci_base_controller *controller,
	struct sci_base_remote_device *remote_device,
	struct sci_base_request *io_request,
	u16 io_tag)
{
	enum sci_status status;

	struct scic_sds_controller *this_controller;
	struct scic_sds_request *the_request;
	struct scic_sds_remote_device *the_device;

	this_controller = (struct scic_sds_controller *)controller;
	the_request = (struct scic_sds_request *)io_request;
	the_device = (struct scic_sds_remote_device *)remote_device;

	status = scic_sds_remote_device_start_io(this_controller, the_device, the_request);

	if (status == SCI_SUCCESS) {
		this_controller->io_request_table[
			scic_sds_io_tag_get_index(the_request->io_tag)] = the_request;

		scic_sds_controller_post_request(
			this_controller,
			scic_sds_request_get_post_context(the_request)
			);
	}

	return status;
}

/**
 *
 * @controller: This is struct sci_base_controller object which is cast into a
 *    struct scic_sds_controller object.
 * @remote_device: This is struct sci_base_remote_device which is cast to a
 *    struct scic_sds_remote_device object.
 * @io_request: This is the struct sci_base_request which is cast to a
 *    SCIC_SDS_IO_REQUEST object.
 *
 * This method is called when the struct scic_sds_controller is in the ready state and
 * the complete io handler is called. - Complete the io request on the remote
 * device - if successful - remove the io_request to the io_request_table
 * enum sci_status SCI_SUCCESS if the start io operation succeeds
 * SCI_FAILURE_INVALID_STATE if one or more objects are not in a valid state to
 * accept io requests.
 */
static enum sci_status scic_sds_controller_ready_state_complete_io_handler(
	struct sci_base_controller *controller,
	struct sci_base_remote_device *remote_device,
	struct sci_base_request *io_request)
{
	u16 index;
	enum sci_status status;
	struct scic_sds_controller *this_controller;
	struct scic_sds_request *the_request;
	struct scic_sds_remote_device *the_device;

	this_controller = (struct scic_sds_controller *)controller;
	the_request = (struct scic_sds_request *)io_request;
	the_device = (struct scic_sds_remote_device *)remote_device;

	status = scic_sds_remote_device_complete_io(
		this_controller, the_device, the_request);

	if (status == SCI_SUCCESS) {
		index = scic_sds_io_tag_get_index(the_request->io_tag);
		this_controller->io_request_table[index] = SCI_INVALID_HANDLE;
	}

	return status;
}

/**
 *
 * @controller: This is struct sci_base_controller object which is cast into a
 *    struct scic_sds_controller object.
 * @remote_device: This is struct sci_base_remote_device which is cast to a
 *    struct scic_sds_remote_device object.
 * @io_request: This is the struct sci_base_request which is cast to a
 *    SCIC_SDS_IO_REQUEST object.
 *
 * This method is called when the struct scic_sds_controller is in the ready state and
 * the continue io handler is called. enum sci_status
 */
static enum sci_status scic_sds_controller_ready_state_continue_io_handler(
	struct sci_base_controller *controller,
	struct sci_base_remote_device *remote_device,
	struct sci_base_request *io_request)
{
	struct scic_sds_controller *this_controller;
	struct scic_sds_request *the_request;

	the_request     = (struct scic_sds_request *)io_request;
	this_controller = (struct scic_sds_controller *)controller;

	this_controller->io_request_table[
		scic_sds_io_tag_get_index(the_request->io_tag)] = the_request;

	scic_sds_controller_post_request(
		this_controller,
		scic_sds_request_get_post_context(the_request)
		);

	return SCI_SUCCESS;
}

/**
 *
 * @controller: This is struct sci_base_controller object which is cast into a
 *    struct scic_sds_controller object.
 * @remote_device: This is struct sci_base_remote_device which is cast to a
 *    struct scic_sds_remote_device object.
 * @io_request: This is the struct sci_base_request which is cast to a
 *    SCIC_SDS_IO_REQUEST object.
 * @task_tag: This is the task tag to be assigned to the task request or
 *    SCI_CONTROLLER_INVALID_IO_TAG.
 *
 * This method is called when the struct scic_sds_controller is in the ready state and
 * the start task handler is called. - The remote device is requested to start
 * the task request - if successful - assign the task to the io_request_table -
 * post the request to the SCU hardware enum sci_status SCI_SUCCESS if the start io
 * operation succeeds SCI_FAILURE_INSUFFICIENT_RESOURCES if the IO tag could
 * not be allocated for the io request. SCI_FAILURE_INVALID_STATE if one or
 * more objects are not in a valid state to accept io requests. How does the io
 * tag get assigned in this code path?
 */
static enum sci_status scic_sds_controller_ready_state_start_task_handler(
	struct sci_base_controller *controller,
	struct sci_base_remote_device *remote_device,
	struct sci_base_request *io_request,
	u16 task_tag)
{
	struct scic_sds_controller *this_controller = (struct scic_sds_controller *)
						 controller;
	struct scic_sds_request *the_request     = (struct scic_sds_request *)
					      io_request;
	struct scic_sds_remote_device *the_device      = (struct scic_sds_remote_device *)
						    remote_device;
	enum sci_status status;

	status = scic_sds_remote_device_start_task(
		this_controller, the_device, the_request
		);

	if (status == SCI_SUCCESS) {
		this_controller->io_request_table[
			scic_sds_io_tag_get_index(the_request->io_tag)] = the_request;

		scic_sds_controller_post_request(
			this_controller,
			scic_sds_request_get_post_context(the_request)
			);
	} else if (status == SCI_FAILURE_RESET_DEVICE_PARTIAL_SUCCESS) {
		this_controller->io_request_table[
			scic_sds_io_tag_get_index(the_request->io_tag)] = the_request;

		/*
		 * We will let framework know this task request started successfully,
		 * although core is still woring on starting the request (to post tc when
		 * RNC is resumed.) */
		status = SCI_SUCCESS;
	}
	return status;
}

/**
 *
 * @controller: This is struct sci_base_controller object which is cast into a
 *    struct scic_sds_controller object.
 * @remote_device: This is struct sci_base_remote_device which is cast to a
 *    struct scic_sds_remote_device object.
 * @io_request: This is the struct sci_base_request which is cast to a
 *    SCIC_SDS_IO_REQUEST object.
 *
 * This method is called when the struct scic_sds_controller is in the ready state and
 * the terminate request handler is called. - call the io request terminate
 * function - if successful - post the terminate request to the SCU hardware
 * enum sci_status SCI_SUCCESS if the start io operation succeeds
 * SCI_FAILURE_INVALID_STATE if one or more objects are not in a valid state to
 * accept io requests.
 */
static enum sci_status scic_sds_controller_ready_state_terminate_request_handler(
	struct sci_base_controller *controller,
	struct sci_base_remote_device *remote_device,
	struct sci_base_request *io_request)
{
	struct scic_sds_controller *this_controller = (struct scic_sds_controller *)
						 controller;
	struct scic_sds_request *the_request     = (struct scic_sds_request *)
					      io_request;
	enum sci_status status;

	status = scic_sds_io_request_terminate(the_request);
	if (status == SCI_SUCCESS) {
		/*
		 * Utilize the original post context command and or in the POST_TC_ABORT
		 * request sub-type. */
		scic_sds_controller_post_request(
			this_controller,
			scic_sds_request_get_post_context(the_request)
			| SCU_CONTEXT_COMMAND_REQUEST_POST_TC_ABORT
			);
	}

	return status;
}

/**
 *
 * @controller: This is struct scic_sds_controller which receives the link up
 *    notification.
 * @port: This is struct scic_sds_port with which the phy is associated.
 * @phy: This is the struct scic_sds_phy which has gone link up.
 *
 * This method is called when the struct scic_sds_controller is in the starting state
 * link up handler is called.  This method will perform the following: - Stop
 * the phy timer - Start the next phy - Report the link up condition to the
 * port object none
 */
static void scic_sds_controller_ready_state_link_up_handler(
	struct scic_sds_controller *this_controller,
	struct scic_sds_port *port,
	struct scic_sds_phy *phy)
{
	this_controller->port_agent.link_up_handler(
		this_controller, &this_controller->port_agent, port, phy
		);
}

/**
 *
 * @controller: This is struct scic_sds_controller which receives the link down
 *    notification.
 * @port: This is struct scic_sds_port with which the phy is associated.
 * @phy: This is the struct scic_sds_phy which has gone link down.
 *
 * This method is called when the struct scic_sds_controller is in the starting state
 * link down handler is called. - Report the link down condition to the port
 * object none
 */
static void scic_sds_controller_ready_state_link_down_handler(
	struct scic_sds_controller *this_controller,
	struct scic_sds_port *port,
	struct scic_sds_phy *phy)
{
	this_controller->port_agent.link_down_handler(
		this_controller, &this_controller->port_agent, port, phy
		);
}

/*
 * *****************************************************************************
 * * STOPPING STATE HANDLERS
 * ***************************************************************************** */

/**
 *
 * @controller: This is struct sci_base_controller object which is cast into a
 *    struct scic_sds_controller object.
 * @remote_device: This is struct sci_base_remote_device which is cast to a
 *    struct scic_sds_remote_device object.
 * @io_request: This is the struct sci_base_request which is cast to a
 *    SCIC_SDS_IO_REQUEST object.
 *
 * This method is called when the struct scic_sds_controller is in a stopping state
 * and the complete io handler is called. - This function is not yet
 * implemented enum sci_status SCI_FAILURE
 */
static enum sci_status scic_sds_controller_stopping_state_complete_io_handler(
	struct sci_base_controller *controller,
	struct sci_base_remote_device *remote_device,
	struct sci_base_request *io_request)
{
	struct scic_sds_controller *this_controller;

	this_controller = (struct scic_sds_controller *)controller;

	/* / @todo Implement this function */
	return SCI_FAILURE;
}

/**
 *
 * @controller: This is struct sci_base_controller object which is cast into a
 *    struct scic_sds_controller object.
 * @remote_device: This is struct sci_base_remote_device which is cast to a
 *    struct scic_sds_remote_device object.
 * @io_request: This is the struct sci_base_request which is cast to a
 *    SCIC_SDS_IO_REQUEST object.
 *
 * This method is called when the struct scic_sds_controller is in a stopping state
 * and the complete task handler is called. - This function is not yet
 * implemented enum sci_status SCI_FAILURE
 */

/*
 * *****************************************************************************
 * * STOPPED STATE HANDLERS
 * ***************************************************************************** */

/*
 * *****************************************************************************
 * * FAILED STATE HANDLERS
 * ***************************************************************************** */

const struct scic_sds_controller_state_handler scic_sds_controller_state_handler_table[] = {
	[SCI_BASE_CONTROLLER_STATE_INITIAL] = {
		.base.start_io     = scic_sds_controller_default_start_operation_handler,
		.base.complete_io  = scic_sds_controller_default_request_handler,
		.base.continue_io  = scic_sds_controller_default_request_handler,
		.terminate_request = scic_sds_controller_default_request_handler,
	},
	[SCI_BASE_CONTROLLER_STATE_RESET] = {
		.base.initialize   = scic_sds_controller_reset_state_initialize_handler,
		.base.start_io     = scic_sds_controller_default_start_operation_handler,
		.base.complete_io  = scic_sds_controller_default_request_handler,
		.base.continue_io  = scic_sds_controller_default_request_handler,
		.terminate_request = scic_sds_controller_default_request_handler,
	},
	[SCI_BASE_CONTROLLER_STATE_INITIALIZING] = {
		.base.start_io     = scic_sds_controller_default_start_operation_handler,
		.base.complete_io  = scic_sds_controller_default_request_handler,
		.base.continue_io  = scic_sds_controller_default_request_handler,
		.terminate_request = scic_sds_controller_default_request_handler,
	},
	[SCI_BASE_CONTROLLER_STATE_INITIALIZED] = {
		.base.start        = scic_sds_controller_initialized_state_start_handler,
		.base.start_io     = scic_sds_controller_default_start_operation_handler,
		.base.complete_io  = scic_sds_controller_default_request_handler,
		.base.continue_io  = scic_sds_controller_default_request_handler,
		.terminate_request = scic_sds_controller_default_request_handler,
	},
	[SCI_BASE_CONTROLLER_STATE_STARTING] = {
		.base.start_io     = scic_sds_controller_default_start_operation_handler,
		.base.complete_io  = scic_sds_controller_default_request_handler,
		.base.continue_io  = scic_sds_controller_default_request_handler,
		.terminate_request = scic_sds_controller_default_request_handler,
		.link_up           = scic_sds_controller_starting_state_link_up_handler,
		.link_down	   = scic_sds_controller_starting_state_link_down_handler
	},
	[SCI_BASE_CONTROLLER_STATE_READY] = {
		.base.stop         = scic_sds_controller_ready_state_stop_handler,
		.base.reset        = scic_sds_controller_general_reset_handler,
		.base.start_io     = scic_sds_controller_ready_state_start_io_handler,
		.base.complete_io  = scic_sds_controller_ready_state_complete_io_handler,
		.base.continue_io  = scic_sds_controller_ready_state_continue_io_handler,
		.base.start_task   = scic_sds_controller_ready_state_start_task_handler,
		.base.complete_task = scic_sds_controller_ready_state_complete_io_handler,
		.terminate_request = scic_sds_controller_ready_state_terminate_request_handler,
		.link_up           = scic_sds_controller_ready_state_link_up_handler,
		.link_down	   = scic_sds_controller_ready_state_link_down_handler
	},
	[SCI_BASE_CONTROLLER_STATE_RESETTING] = {
		.base.start_io     = scic_sds_controller_default_start_operation_handler,
		.base.complete_io  = scic_sds_controller_default_request_handler,
		.base.continue_io  = scic_sds_controller_default_request_handler,
		.terminate_request = scic_sds_controller_default_request_handler,
	},
	[SCI_BASE_CONTROLLER_STATE_STOPPING] = {
		.base.start_io     = scic_sds_controller_default_start_operation_handler,
		.base.complete_io  = scic_sds_controller_stopping_state_complete_io_handler,
		.base.continue_io  = scic_sds_controller_default_request_handler,
		.terminate_request = scic_sds_controller_default_request_handler,
	},
	[SCI_BASE_CONTROLLER_STATE_STOPPED] = {
		.base.reset        = scic_sds_controller_general_reset_handler,
		.base.start_io     = scic_sds_controller_default_start_operation_handler,
		.base.complete_io  = scic_sds_controller_default_request_handler,
		.base.continue_io  = scic_sds_controller_default_request_handler,
		.terminate_request = scic_sds_controller_default_request_handler,
	},
	[SCI_BASE_CONTROLLER_STATE_FAILED] = {
		.base.reset        = scic_sds_controller_general_reset_handler,
		.base.start_io     = scic_sds_controller_default_start_operation_handler,
		.base.complete_io  = scic_sds_controller_default_request_handler,
		.base.continue_io  = scic_sds_controller_default_request_handler,
		.terminate_request = scic_sds_controller_default_request_handler,
	},
};
