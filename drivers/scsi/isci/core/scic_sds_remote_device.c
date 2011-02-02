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
 * This file contains the implementation of remote device methods.
 *
 *
 */

#include "intel_sas.h"
#include "sci_util.h"
#include "sci_environment.h"
#include "scic_port.h"
#include "scic_phy.h"
#include "scic_remote_device.h"
#include "scic_sds_port.h"
#include "scic_sds_phy.h"
#include "scic_sds_remote_device.h"
#include "scic_sds_request.h"
#include "scic_sds_controller.h"

#define SCIC_SDS_REMOTE_DEVICE_RESET_TIMEOUT  (1000)

/*
 * *****************************************************************************
 * *  CORE REMOTE DEVICE PRIVATE METHODS
 * ***************************************************************************** */

/*
 * *****************************************************************************
 * *  CORE REMOTE DEVICE PUBLIC METHODS
 * ***************************************************************************** */

u32 scic_remote_device_get_object_size(void)
{
	return sizeof(struct scic_sds_remote_device)
	       + sizeof(struct scic_sds_remote_node_context);
}

/* --------------------------------------------------------------------------- */

void scic_remote_device_construct(struct scic_sds_port *sci_port,
				  struct scic_sds_remote_device *sci_dev)
{
	sci_dev->owning_port = sci_port;
	sci_dev->started_request_count = 0;
	sci_dev->rnc = (struct scic_sds_remote_node_context *) &sci_dev[1];

	sci_base_remote_device_construct(
		&sci_dev->parent,
		scic_sds_remote_device_state_table
		);

	scic_sds_remote_node_context_construct(
		sci_dev,
		sci_dev->rnc,
		SCIC_SDS_REMOTE_NODE_CONTEXT_INVALID_INDEX
		);

	sci_object_set_association(sci_dev->rnc, sci_dev);
}

/* --------------------------------------------------------------------------- */

enum sci_status scic_remote_device_da_construct(
	SCI_REMOTE_DEVICE_HANDLE_T remote_device)
{
	enum sci_status status;
	u16 remote_node_index;
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)
						remote_device;
	struct sci_sas_identify_address_frame_protocols protocols;

	/*
	 * This information is request to determine how many remote node context
	 * entries will be needed to store the remote node. */
	scic_sds_port_get_attached_protocols(this_device->owning_port, &protocols);
	this_device->target_protocols.u.all = protocols.u.all;
	this_device->is_direct_attached = true;
#if !defined(DISABLE_ATAPI)
	this_device->is_atapi = scic_sds_remote_device_is_atapi(this_device);
#endif

	status = scic_sds_controller_allocate_remote_node_context(
		this_device->owning_port->owning_controller,
		this_device,
		&remote_node_index
		);

	if (status == SCI_SUCCESS) {
		scic_sds_remote_node_context_set_remote_node_index(
			this_device->rnc, remote_node_index
			);

		scic_sds_port_get_attached_sas_address(
			this_device->owning_port, &this_device->device_address
			);

		if (this_device->target_protocols.u.bits.attached_ssp_target) {
			this_device->has_ready_substate_machine = false;
		} else if (this_device->target_protocols.u.bits.attached_stp_target) {
			this_device->has_ready_substate_machine = true;

			sci_base_state_machine_construct(
				&this_device->ready_substate_machine,
				&this_device->parent.parent,
				scic_sds_stp_remote_device_ready_substate_table,
				SCIC_SDS_STP_REMOTE_DEVICE_READY_SUBSTATE_IDLE
				);
		} else if (this_device->target_protocols.u.bits.attached_smp_target) {
			this_device->has_ready_substate_machine = true;

			/* add the SMP ready substate machine construction here */
			sci_base_state_machine_construct(
				&this_device->ready_substate_machine,
				&this_device->parent.parent,
				scic_sds_smp_remote_device_ready_substate_table,
				SCIC_SDS_SMP_REMOTE_DEVICE_READY_SUBSTATE_IDLE
				);
		}

		this_device->connection_rate = scic_sds_port_get_max_allowed_speed(
			this_device->owning_port
			);

		/* / @todo Should I assign the port width by reading all of the phys on the port? */
		this_device->device_port_width = 1;
	}

	return status;
}


/* --------------------------------------------------------------------------- */

static void scic_sds_remote_device_get_info_from_smp_discover_response(
	struct scic_sds_remote_device *this_device,
	struct smp_response_discover *discover_response)
{
	/* decode discover_response to set sas_address to this_device. */
	this_device->device_address.high =
		discover_response->attached_sas_address.high;

	this_device->device_address.low =
		discover_response->attached_sas_address.low;

	this_device->target_protocols.u.all = discover_response->protocols.u.all;
}


/* --------------------------------------------------------------------------- */

enum sci_status scic_remote_device_ea_construct(
	SCI_REMOTE_DEVICE_HANDLE_T remote_device,
	struct smp_response_discover *discover_response)
{
	enum sci_status status;

	struct scic_sds_remote_device *this_device;
	struct scic_sds_controller *the_controller;

	this_device = (struct scic_sds_remote_device *)remote_device;

	the_controller = scic_sds_port_get_controller(this_device->owning_port);

	scic_sds_remote_device_get_info_from_smp_discover_response(
		this_device, discover_response
		);

	status = scic_sds_controller_allocate_remote_node_context(
		the_controller,
		this_device,
		&this_device->rnc->remote_node_index
		);

	if (status == SCI_SUCCESS) {
		if (this_device->target_protocols.u.bits.attached_ssp_target) {
			this_device->has_ready_substate_machine = false;
		} else if (this_device->target_protocols.u.bits.attached_smp_target) {
			this_device->has_ready_substate_machine = true;

			/* add the SMP ready substate machine construction here */
			sci_base_state_machine_construct(
				&this_device->ready_substate_machine,
				&this_device->parent.parent,
				scic_sds_smp_remote_device_ready_substate_table,
				SCIC_SDS_SMP_REMOTE_DEVICE_READY_SUBSTATE_IDLE
				);
		} else if (this_device->target_protocols.u.bits.attached_stp_target) {
			this_device->has_ready_substate_machine = true;

			sci_base_state_machine_construct(
				&this_device->ready_substate_machine,
				&this_device->parent.parent,
				scic_sds_stp_remote_device_ready_substate_table,
				SCIC_SDS_STP_REMOTE_DEVICE_READY_SUBSTATE_IDLE
				);
		}

		/*
		 * For SAS-2 the physical link rate is actually a logical link
		 * rate that incorporates multiplexing.  The SCU doesn't
		 * incorporate multiplexing and for the purposes of the
		 * connection the logical link rate is that same as the
		 * physical.  Furthermore, the SAS-2 and SAS-1.1 fields overlay
		 * one another, so this code works for both situations. */
		this_device->connection_rate = min_t(u16,
			scic_sds_port_get_max_allowed_speed(this_device->owning_port),
			discover_response->u2.sas1_1.negotiated_physical_link_rate
			);

		/* / @todo Should I assign the port width by reading all of the phys on the port? */
		this_device->device_port_width = 1;
	}

	return status;
}

/* --------------------------------------------------------------------------- */

enum sci_status scic_remote_device_destruct(
	SCI_REMOTE_DEVICE_HANDLE_T remote_device)
{
	struct scic_sds_remote_device *this_device;

	this_device = (struct scic_sds_remote_device *)remote_device;

	return this_device->state_handlers->parent.destruct_handler(&this_device->parent);
}

/* --------------------------------------------------------------------------- */

enum sci_status scic_remote_device_start(
	SCI_REMOTE_DEVICE_HANDLE_T remote_device,
	u32 timeout)
{
	struct scic_sds_remote_device *this_device;

	this_device = (struct scic_sds_remote_device *)remote_device;

	return this_device->state_handlers->parent.start_handler(&this_device->parent);
}

/* --------------------------------------------------------------------------- */

enum sci_status scic_remote_device_stop(
	SCI_REMOTE_DEVICE_HANDLE_T remote_device,
	u32 timeout)
{
	struct scic_sds_remote_device *this_device;

	this_device = (struct scic_sds_remote_device *)remote_device;

	return this_device->state_handlers->parent.stop_handler(&this_device->parent);
}

/**
 *
 * @this_device: The remote device for which the reset is being requested.
 *
 * This method invokes the remote device reset handler. enum sci_status
 */
enum sci_status scic_remote_device_reset(
	SCI_REMOTE_DEVICE_HANDLE_T remote_device)
{
	struct scic_sds_remote_device *this_device;

	this_device = (struct scic_sds_remote_device *)remote_device;

	return this_device->state_handlers->parent.reset_handler(&this_device->parent);
}

/**
 *
 * @this_device: The remote device for which the reset is being requested.
 *
 * This method invokes the remote device reset handler. enum sci_status
 */
enum sci_status scic_remote_device_reset_complete(
	SCI_REMOTE_DEVICE_HANDLE_T remote_device)
{
	struct scic_sds_remote_device *this_device;

	this_device = (struct scic_sds_remote_device *)remote_device;

	return this_device->state_handlers->parent.reset_complete_handler(&this_device->parent);
}

/**
 *
 * @this_device: The remote device for which the reset is being requested.
 *
 * This method invokes the remote device reset handler. enum sci_status
 */

/* --------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------- */

enum sci_sas_link_rate scic_remote_device_get_connection_rate(
	SCI_REMOTE_DEVICE_HANDLE_T remote_device)
{
	struct scic_sds_remote_device *this_device;

	this_device = (struct scic_sds_remote_device *)remote_device;

	return this_device->connection_rate;
}

/* --------------------------------------------------------------------------- */

void scic_remote_device_get_protocols(
	SCI_REMOTE_DEVICE_HANDLE_T remote_device,
	struct smp_discover_response_protocols *protocols)
{
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)
						remote_device;

	protocols->u.all = this_device->target_protocols.u.all;
}

/* --------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------- */
#if !defined(DISABLE_ATAPI)
bool scic_remote_device_is_atapi(
	SCI_REMOTE_DEVICE_HANDLE_T device_handle)
{
	return ((struct scic_sds_remote_device *)device_handle)->is_atapi;
}
#endif


/*
 * *****************************************************************************
 * *  SCU DRIVER STANDARD (SDS) REMOTE DEVICE IMPLEMENTATIONS
 * ***************************************************************************** */

/**
 *
 *
 * Remote device timer requirements
 */
#define SCIC_SDS_REMOTE_DEVICE_MINIMUM_TIMER_COUNT (0)
#define SCIC_SDS_REMOTE_DEVICE_MAXIMUM_TIMER_COUNT (SCI_MAX_REMOTE_DEVICES)

/**
 * This method returns the minimum number of timers required for all remote
 *    devices.
 *
 * u32
 */

/**
 * This method returns the maximum number of timers requried for all remote
 *    devices.
 *
 * u32
 */

/* --------------------------------------------------------------------------- */

/**
 *
 * @this_device: The remote device for which the suspend is being requested.
 *
 * This method invokes the remote device suspend state handler. enum sci_status
 */
enum sci_status scic_sds_remote_device_suspend(
	struct scic_sds_remote_device *this_device,
	u32 suspend_type)
{
	return this_device->state_handlers->suspend_handler(this_device, suspend_type);
}

/**
 *
 * @this_device: The remote device for which the resume is being requested.
 *
 * This method invokes the remote device resume state handler. enum sci_status
 */
enum sci_status scic_sds_remote_device_resume(
	struct scic_sds_remote_device *this_device)
{
	return this_device->state_handlers->resume_handler(this_device);
}

/**
 *
 * @this_device: The remote device for which the event handling is being
 *    requested.
 * @frame_index: This is the frame index that is being processed.
 *
 * This method invokes the frame handler for the remote device state machine
 * enum sci_status
 */
enum sci_status scic_sds_remote_device_frame_handler(
	struct scic_sds_remote_device *this_device,
	u32 frame_index)
{
	return this_device->state_handlers->frame_handler(this_device, frame_index);
}

/**
 *
 * @this_device: The remote device for which the event handling is being
 *    requested.
 * @event_code: This is the event code that is to be processed.
 *
 * This method invokes the remote device event handler. enum sci_status
 */
enum sci_status scic_sds_remote_device_event_handler(
	struct scic_sds_remote_device *this_device,
	u32 event_code)
{
	return this_device->state_handlers->event_handler(this_device, event_code);
}

/**
 *
 * @controller: The controller that is starting the io request.
 * @this_device: The remote device for which the start io handling is being
 *    requested.
 * @io_request: The io request that is being started.
 *
 * This method invokes the remote device start io handler. enum sci_status
 */
enum sci_status scic_sds_remote_device_start_io(
	struct scic_sds_controller *controller,
	struct scic_sds_remote_device *this_device,
	struct scic_sds_request *io_request)
{
	return this_device->state_handlers->parent.start_io_handler(
		       &this_device->parent, &io_request->parent);
}

/**
 *
 * @controller: The controller that is completing the io request.
 * @this_device: The remote device for which the complete io handling is being
 *    requested.
 * @io_request: The io request that is being completed.
 *
 * This method invokes the remote device complete io handler. enum sci_status
 */
enum sci_status scic_sds_remote_device_complete_io(
	struct scic_sds_controller *controller,
	struct scic_sds_remote_device *this_device,
	struct scic_sds_request *io_request)
{
	return this_device->state_handlers->parent.complete_io_handler(
		       &this_device->parent, &io_request->parent);
}

/**
 *
 * @controller: The controller that is starting the task request.
 * @this_device: The remote device for which the start task handling is being
 *    requested.
 * @io_request: The task request that is being started.
 *
 * This method invokes the remote device start task handler. enum sci_status
 */
enum sci_status scic_sds_remote_device_start_task(
	struct scic_sds_controller *controller,
	struct scic_sds_remote_device *this_device,
	struct scic_sds_request *io_request)
{
	return this_device->state_handlers->parent.start_task_handler(
		       &this_device->parent, &io_request->parent);
}

/**
 *
 * @controller: The controller that is completing the task request.
 * @this_device: The remote device for which the complete task handling is
 *    being requested.
 * @io_request: The task request that is being completed.
 *
 * This method invokes the remote device complete task handler. enum sci_status
 */

/**
 *
 * @this_device:
 * @request:
 *
 * This method takes the request and bulids an appropriate SCU context for the
 * request and then requests the controller to post the request. none
 */
void scic_sds_remote_device_post_request(
	struct scic_sds_remote_device *this_device,
	u32 request)
{
	u32 context;

	context = scic_sds_remote_device_build_command_context(this_device, request);

	scic_sds_controller_post_request(
		scic_sds_remote_device_get_controller(this_device),
		context
		);
}

#if !defined(DISABLE_ATAPI)
/**
 *
 * @this_device: The device to be checked.
 *
 * This method check the signature fis of a stp device to decide whether a
 * device is atapi or not. true if a device is atapi device. False if a device
 * is not atapi.
 */
bool scic_sds_remote_device_is_atapi(
	struct scic_sds_remote_device *this_device)
{
	if (!this_device->target_protocols.u.bits.attached_stp_target)
		return false;
	else if (this_device->is_direct_attached) {
		struct scic_sds_phy *phy;
		struct scic_sata_phy_properties properties;
		struct sata_fis_reg_d2h *signature_fis;
		phy = scic_sds_port_get_a_connected_phy(this_device->owning_port);
		scic_sata_phy_get_properties(phy, &properties);

		/* decode the signature fis. */
		signature_fis = &(properties.signature_fis);

		if ((signature_fis->sector_count  == 0x01)
		    && (signature_fis->lba_low       == 0x01)
		    && (signature_fis->lba_mid       == 0x14)
		    && (signature_fis->lba_high      == 0xEB)
		    && ((signature_fis->device & 0x5F) == 0x00)
		    ) {
			/* An ATA device supporting the PACKET command set. */
			return true;
		} else
			return false;
	} else {
		/* Expander supported ATAPI device is not currently supported. */
		return false;
	}
}
#endif
