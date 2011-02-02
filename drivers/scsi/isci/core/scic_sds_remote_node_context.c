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
 * This file contains the structures, constants, and prototypes associated with
 *    the remote node context in the silicon.  It exists to model and manage
 *    the remote node context in the silicon.
 *
 *
 */

#include "sci_util.h"
#include "scic_sds_controller.h"
#include "scic_sds_remote_device.h"
#include "scic_sds_remote_node_context.h"

/**
 *
 *
 *
 */
void scic_sds_remote_node_context_construct(
	struct scic_sds_remote_device *device,
	struct scic_sds_remote_node_context *rnc,
	u16 remote_node_index)
{
	memset(rnc, 0, sizeof(struct scic_sds_remote_node_context));

	rnc->remote_node_index = remote_node_index;
	rnc->device            = device;
	rnc->destination_state = SCIC_SDS_REMOTE_NODE_DESTINATION_STATE_UNSPECIFIED;

	sci_base_state_machine_construct(
		&rnc->state_machine,
		&rnc->parent,
		scic_sds_remote_node_context_state_table,
		SCIC_SDS_REMOTE_NODE_CONTEXT_INITIAL_STATE
		);

	sci_base_state_machine_start(&rnc->state_machine);
}

/**
 *
 * @this_rnc: The RNC for which the is posted request is being made.
 *
 * This method will return true if the RNC is not in the initial state.  In all
 * other states the RNC is considered active and this will return true. The
 * destroy request of the state machine drives the RNC back to the initial
 * state.  If the state machine changes then this routine will also have to be
 * changed. bool true if the state machine is not in the initial state false if
 * the state machine is in the initial state
 */

/**
 *
 * @this_rnc: The state of the remote node context object to check.
 *
 * This method will return true if the remote node context is in a READY state
 * otherwise it will return false bool true if the remote node context is in
 * the ready state. false if the remote node context is not in the ready state.
 */
bool scic_sds_remote_node_context_is_ready(
	struct scic_sds_remote_node_context *this_rnc)
{
	u32 current_state = sci_base_state_machine_get_state(&this_rnc->state_machine);

	if (current_state == SCIC_SDS_REMOTE_NODE_CONTEXT_READY_STATE) {
		return true;
	}

	return false;
}

/**
 *
 * @this_device: The remote device to use to construct the RNC buffer.
 * @rnc: The buffer into which the remote device data will be copied.
 *
 * This method will construct the RNC buffer for this remote device object. none
 */
void scic_sds_remote_node_context_construct_buffer(
	struct scic_sds_remote_node_context *this_rnc)
{
	union scu_remote_node_context *rnc;
	struct scic_sds_controller *the_controller;

	the_controller = scic_sds_remote_device_get_controller(this_rnc->device);

	rnc = scic_sds_controller_get_remote_node_context_buffer(
		the_controller, this_rnc->remote_node_index);

	memset(
		rnc,
		0x00,
		sizeof(union scu_remote_node_context)
		* scic_sds_remote_device_node_count(this_rnc->device)
		);

	rnc->ssp.remote_node_index = this_rnc->remote_node_index;
	rnc->ssp.remote_node_port_width = this_rnc->device->device_port_width;
	rnc->ssp.logical_port_index =
		scic_sds_remote_device_get_port_index(this_rnc->device);

	rnc->ssp.remote_sas_address_hi = SCIC_SWAP_DWORD(this_rnc->device->device_address.high);
	rnc->ssp.remote_sas_address_lo = SCIC_SWAP_DWORD(this_rnc->device->device_address.low);

	rnc->ssp.nexus_loss_timer_enable = true;
	rnc->ssp.check_bit               = false;
	rnc->ssp.is_valid                = false;
	rnc->ssp.is_remote_node_context  = true;
	rnc->ssp.function_number         = 0;

	rnc->ssp.arbitration_wait_time = 0;


	if (
		this_rnc->device->target_protocols.u.bits.attached_sata_device
		|| this_rnc->device->target_protocols.u.bits.attached_stp_target
		) {
		rnc->ssp.connection_occupancy_timeout =
			the_controller->user_parameters.sds1.stp_max_occupancy_timeout;
		rnc->ssp.connection_inactivity_timeout =
			the_controller->user_parameters.sds1.stp_inactivity_timeout;
	} else {
		rnc->ssp.connection_occupancy_timeout  =
			the_controller->user_parameters.sds1.ssp_max_occupancy_timeout;
		rnc->ssp.connection_inactivity_timeout =
			the_controller->user_parameters.sds1.ssp_inactivity_timeout;
	}

	rnc->ssp.initial_arbitration_wait_time = 0;

	/* Open Address Frame Parameters */
	rnc->ssp.oaf_connection_rate = this_rnc->device->connection_rate;
	rnc->ssp.oaf_features = 0;
	rnc->ssp.oaf_source_zone_group = 0;
	rnc->ssp.oaf_more_compatibility_features = 0;
}

/* --------------------------------------------------------------------------- */
