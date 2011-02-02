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
 * This file contains the state machine that models the remote node context in
 *    the silicon.
 *
 *
 */

#include "scic_sds_controller.h"
#include "scic_sds_port.h"
#include "scic_sds_remote_device.h"
#include "scic_sds_remote_node_context.h"
#include "scu_task_context.h"

/*
 * *****************************************************************************
 * * REMOTE NODE CONTEXT PRIVATE METHODS
 * ***************************************************************************** */

/**
 *
 *
 * This method just calls the user callback function and then resets the
 * callback.
 */
static void scic_sds_remote_node_context_notify_user(
	struct scic_sds_remote_node_context *rnc)
{
	if (rnc->user_callback != NULL) {
		(*rnc->user_callback)(rnc->user_cookie);

		rnc->user_callback = NULL;
		rnc->user_cookie = NULL;
	}
}

/**
 *
 *
 * This method will continue the remote node context state machine by
 * requesting to resume the remote node context state machine from its current
 * state.
 */
static void scic_sds_remote_node_context_continue_state_transitions(
	struct scic_sds_remote_node_context *rnc)
{
	if (rnc->destination_state == SCIC_SDS_REMOTE_NODE_DESTINATION_STATE_READY) {
		rnc->state_handlers->resume_handler(
			rnc, rnc->user_callback, rnc->user_cookie
			);
	}
}

/**
 *
 * @this_rnc: The remote node context object that is to be validated.
 *
 * This method will mark the rnc buffer as being valid and post the request to
 * the hardware. none
 */
static void scic_sds_remote_node_context_validate_context_buffer(
	struct scic_sds_remote_node_context *this_rnc)
{
	union scu_remote_node_context *rnc_buffer;

	rnc_buffer = scic_sds_controller_get_remote_node_context_buffer(
		scic_sds_remote_device_get_controller(this_rnc->device),
		this_rnc->remote_node_index
		);

	rnc_buffer->ssp.is_valid = true;

	if (
		!this_rnc->device->is_direct_attached
		&& this_rnc->device->target_protocols.u.bits.attached_stp_target
		) {
		scic_sds_remote_device_post_request(
			this_rnc->device,
			SCU_CONTEXT_COMMAND_POST_RNC_96
			);
	} else {
		scic_sds_remote_device_post_request(
			this_rnc->device,
			SCU_CONTEXT_COMMAND_POST_RNC_32
			);

		if (this_rnc->device->is_direct_attached) {
			scic_sds_port_set_direct_attached_device_id(
				this_rnc->device->owning_port,
				this_rnc->remote_node_index
				);
		}
	}
}

/**
 *
 * @this_rnc: The remote node context object that is to be invalidated.
 *
 * This method will update the RNC buffer and post the invalidate request. none
 */
static void scic_sds_remote_node_context_invalidate_context_buffer(
	struct scic_sds_remote_node_context *this_rnc)
{
	union scu_remote_node_context *rnc_buffer;

	rnc_buffer = scic_sds_controller_get_remote_node_context_buffer(
		scic_sds_remote_device_get_controller(this_rnc->device),
		this_rnc->remote_node_index
		);

	rnc_buffer->ssp.is_valid = false;

	scic_sds_remote_device_post_request(
		this_rnc->device,
		SCU_CONTEXT_COMMAND_POST_RNC_INVALIDATE
		);

	if (this_rnc->device->is_direct_attached) {
		scic_sds_port_set_direct_attached_device_id(
			this_rnc->device->owning_port,
			SCIC_SDS_REMOTE_NODE_CONTEXT_INVALID_INDEX
			);
	}
}

/*
 * *****************************************************************************
 * * REMOTE NODE CONTEXT STATE ENTER AND EXIT METHODS
 * ***************************************************************************** */

/**
 *
 *
 *
 */
static void scic_sds_remote_node_context_initial_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_remote_node_context *rnc;

	rnc = (struct scic_sds_remote_node_context *)object;

	SET_STATE_HANDLER(
		rnc,
		scic_sds_remote_node_context_state_handler_table,
		SCIC_SDS_REMOTE_NODE_CONTEXT_INITIAL_STATE
		);

	/*
	 * Check to see if we have gotten back to the initial state because someone
	 * requested to destroy the remote node context object. */
	if (
		rnc->state_machine.previous_state_id
		== SCIC_SDS_REMOTE_NODE_CONTEXT_INVALIDATING_STATE
		) {
		rnc->destination_state = SCIC_SDS_REMOTE_NODE_DESTINATION_STATE_UNSPECIFIED;

		scic_sds_remote_node_context_notify_user(rnc);
	}
}

/**
 *
 *
 *
 */
static void scic_sds_remote_node_context_posting_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_remote_node_context *this_rnc;

	this_rnc = (struct scic_sds_remote_node_context *)object;

	SET_STATE_HANDLER(
		this_rnc,
		scic_sds_remote_node_context_state_handler_table,
		SCIC_SDS_REMOTE_NODE_CONTEXT_POSTING_STATE
		);

	scic_sds_remote_node_context_validate_context_buffer(this_rnc);
}

/**
 *
 *
 *
 */
static void scic_sds_remote_node_context_invalidating_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_remote_node_context *rnc;

	rnc = (struct scic_sds_remote_node_context *)object;

	SET_STATE_HANDLER(
		rnc,
		scic_sds_remote_node_context_state_handler_table,
		SCIC_SDS_REMOTE_NODE_CONTEXT_INVALIDATING_STATE
		);

	scic_sds_remote_node_context_invalidate_context_buffer(rnc);
}

/**
 *
 *
 *
 */
static void scic_sds_remote_node_context_resuming_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_remote_node_context *rnc;

	rnc = (struct scic_sds_remote_node_context *)object;

	SET_STATE_HANDLER(
		rnc,
		scic_sds_remote_node_context_state_handler_table,
		SCIC_SDS_REMOTE_NODE_CONTEXT_RESUMING_STATE
		);

	scic_sds_remote_device_post_request(
		rnc->device,
		SCU_CONTEXT_COMMAND_POST_RNC_RESUME
		);
}

/**
 *
 *
 *
 */
static void scic_sds_remote_node_context_ready_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_remote_node_context *rnc;

	rnc = (struct scic_sds_remote_node_context *)object;

	SET_STATE_HANDLER(
		rnc,
		scic_sds_remote_node_context_state_handler_table,
		SCIC_SDS_REMOTE_NODE_CONTEXT_READY_STATE
		);

	rnc->destination_state = SCIC_SDS_REMOTE_NODE_DESTINATION_STATE_UNSPECIFIED;

	if (rnc->user_callback != NULL) {
		scic_sds_remote_node_context_notify_user(rnc);
	}
}

/**
 *
 *
 *
 */
static void scic_sds_remote_node_context_tx_suspended_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_remote_node_context *rnc;

	rnc = (struct scic_sds_remote_node_context *)object;

	SET_STATE_HANDLER(
		rnc,
		scic_sds_remote_node_context_state_handler_table,
		SCIC_SDS_REMOTE_NODE_CONTEXT_TX_SUSPENDED_STATE
		);

	scic_sds_remote_node_context_continue_state_transitions(rnc);
}

/**
 *
 *
 *
 */
static void scic_sds_remote_node_context_tx_rx_suspended_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_remote_node_context *rnc;

	rnc = (struct scic_sds_remote_node_context *)object;

	SET_STATE_HANDLER(
		rnc,
		scic_sds_remote_node_context_state_handler_table,
		SCIC_SDS_REMOTE_NODE_CONTEXT_TX_RX_SUSPENDED_STATE
		);

	scic_sds_remote_node_context_continue_state_transitions(rnc);
}

/**
 *
 *
 *
 */
static void scic_sds_remote_node_context_await_suspension_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_remote_node_context *rnc;

	rnc = (struct scic_sds_remote_node_context *)object;

	SET_STATE_HANDLER(
		rnc,
		scic_sds_remote_node_context_state_handler_table,
		SCIC_SDS_REMOTE_NODE_CONTEXT_AWAIT_SUSPENSION_STATE
		);
}

/* --------------------------------------------------------------------------- */

const struct sci_base_state scic_sds_remote_node_context_state_table[] = {
	[SCIC_SDS_REMOTE_NODE_CONTEXT_INITIAL_STATE] = {
		.enter_state = scic_sds_remote_node_context_initial_state_enter,
	},
	[SCIC_SDS_REMOTE_NODE_CONTEXT_POSTING_STATE] = {
		.enter_state = scic_sds_remote_node_context_posting_state_enter,
	},
	[SCIC_SDS_REMOTE_NODE_CONTEXT_INVALIDATING_STATE] = {
		.enter_state = scic_sds_remote_node_context_invalidating_state_enter,
	},
	[SCIC_SDS_REMOTE_NODE_CONTEXT_RESUMING_STATE] = {
		.enter_state = scic_sds_remote_node_context_resuming_state_enter,
	},
	[SCIC_SDS_REMOTE_NODE_CONTEXT_READY_STATE] = {
		.enter_state = scic_sds_remote_node_context_ready_state_enter,
	},
	[SCIC_SDS_REMOTE_NODE_CONTEXT_TX_SUSPENDED_STATE] = {
		.enter_state = scic_sds_remote_node_context_tx_suspended_state_enter,
	},
	[SCIC_SDS_REMOTE_NODE_CONTEXT_TX_RX_SUSPENDED_STATE] = {
		.enter_state = scic_sds_remote_node_context_tx_rx_suspended_state_enter,
	},
	[SCIC_SDS_REMOTE_NODE_CONTEXT_AWAIT_SUSPENSION_STATE] = {
		.enter_state = scic_sds_remote_node_context_await_suspension_state_enter,
	},
};

