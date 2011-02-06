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

#include "sci_base_state_machine.h"
#include "scic_remote_device.h"
#include "scic_sds_logger.h"
#include "scic_sds_port.h"
#include "scic_sds_remote_device.h"
#include "scic_sds_remote_node_context.h"
#include "scu_event_codes.h"
#include "scu_task_context.h"

/**
 *
 * @this_rnc:
 * @the_callback:
 * @callback_parameter:
 *
 * This method will setup the remote node context object so it will transition
 * to its ready state.  If the remote node context is already setup to
 * transition to its final state then this function does nothing. none
 */
static void scic_sds_remote_node_context_setup_to_resume(
	struct scic_sds_remote_node_context *this_rnc,
	SCICS_SDS_REMOTE_NODE_CONTEXT_CALLBACK the_callback,
	void *callback_parameter)
{
	if (this_rnc->destination_state != SCIC_SDS_REMOTE_NODE_DESTINATION_STATE_FINAL) {
		this_rnc->destination_state = SCIC_SDS_REMOTE_NODE_DESTINATION_STATE_READY;
		this_rnc->user_callback     = the_callback;
		this_rnc->user_cookie       = callback_parameter;
	}
}

/**
 *
 * @this_rnc:
 * @the_callback:
 * @callback_parameter:
 *
 * This method will setup the remote node context object so it will transistion
 * to its final state. none
 */
static void scic_sds_remote_node_context_setup_to_destory(
	struct scic_sds_remote_node_context *this_rnc,
	SCICS_SDS_REMOTE_NODE_CONTEXT_CALLBACK the_callback,
	void *callback_parameter)
{
	this_rnc->destination_state = SCIC_SDS_REMOTE_NODE_DESTINATION_STATE_FINAL;
	this_rnc->user_callback     = the_callback;
	this_rnc->user_cookie       = callback_parameter;
}

/**
 *
 * @this_rnc:
 * @the_callback:
 *
 * This method will continue to resume a remote node context.  This is used in
 * the states where a resume is requested while a resume is in progress.
 */
static enum sci_status scic_sds_remote_node_context_continue_to_resume_handler(
	struct scic_sds_remote_node_context *this_rnc,
	SCICS_SDS_REMOTE_NODE_CONTEXT_CALLBACK the_callback,
	void *callback_parameter)
{
	if (this_rnc->destination_state == SCIC_SDS_REMOTE_NODE_DESTINATION_STATE_READY) {
		this_rnc->user_callback = the_callback;
		this_rnc->user_cookie   = callback_parameter;

		return SCI_SUCCESS;
	}

	return SCI_FAILURE_INVALID_STATE;
}

/* --------------------------------------------------------------------------- */

static enum sci_status scic_sds_remote_node_context_default_destruct_handler(
	struct scic_sds_remote_node_context *this_rnc,
	SCICS_SDS_REMOTE_NODE_CONTEXT_CALLBACK the_callback,
	void *callback_parameter)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger(this_rnc->device),
				 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
				 "SCIC Remote Node Context 0x%x requested to stop while in unexpected state %d\n",
				 this_rnc, sci_base_state_machine_get_state(&this_rnc->state_machine)
				 ));

	/*
	 * We have decided that the destruct request on the remote node context can not fail
	 * since it is either in the initial/destroyed state or is can be destroyed. */
	return SCI_SUCCESS;
}

static enum sci_status scic_sds_remote_node_context_default_suspend_handler(
	struct scic_sds_remote_node_context *this_rnc,
	u32 suspend_type,
	SCICS_SDS_REMOTE_NODE_CONTEXT_CALLBACK the_callback,
	void *callback_parameter)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger(this_rnc->device),
				 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
				 "SCIC Remote Node Context 0x%x requested to suspend while in wrong state %d\n",
				 this_rnc, sci_base_state_machine_get_state(&this_rnc->state_machine)
				 ));

	return SCI_FAILURE_INVALID_STATE;
}

static enum sci_status scic_sds_remote_node_context_default_resume_handler(
	struct scic_sds_remote_node_context *this_rnc,
	SCICS_SDS_REMOTE_NODE_CONTEXT_CALLBACK the_callback,
	void *callback_parameter)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger(this_rnc->device),
				 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
				 "SCIC Remote Node Context 0x%x requested to resume while in wrong state %d\n",
				 this_rnc, sci_base_state_machine_get_state(&this_rnc->state_machine)
				 ));

	return SCI_FAILURE_INVALID_STATE;
}

static enum sci_status scic_sds_remote_node_context_default_start_io_handler(
	struct scic_sds_remote_node_context *this_rnc,
	struct scic_sds_request *the_request)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger(this_rnc->device),
				 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
				 "SCIC Remote Node Context 0x%x requested to start io 0x%x while in wrong state %d\n",
				 this_rnc, the_request, sci_base_state_machine_get_state(&this_rnc->state_machine)
				 ));

	return SCI_FAILURE_REMOTE_DEVICE_RESET_REQUIRED;
}

static enum sci_status scic_sds_remote_node_context_default_start_task_handler(
	struct scic_sds_remote_node_context *this_rnc,
	struct scic_sds_request *the_request)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger(this_rnc->device),
				 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
				 "SCIC Remote Node Context 0x%x requested to start task 0x%x while in wrong state %d\n",
				 this_rnc, the_request, sci_base_state_machine_get_state(&this_rnc->state_machine)
				 ));

	return SCI_FAILURE;
}

static enum sci_status scic_sds_remote_node_context_default_event_handler(
	struct scic_sds_remote_node_context *this_rnc,
	u32 event_code)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger(this_rnc->device),
				 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
				 "SCIC Remote Node Context 0x%x requested to process event 0x%x while in wrong state %d\n",
				 this_rnc, event_code, sci_base_state_machine_get_state(&this_rnc->state_machine)
				 ));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @this_rnc: The rnc for which the task request is targeted.
 * @the_request: The request which is going to be started.
 *
 * This method determines if the task request can be started by the SCU
 * hardware. When the RNC is in the ready state any task can be started.
 * enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_remote_node_context_success_start_task_handler(
	struct scic_sds_remote_node_context *this_rnc,
	struct scic_sds_request *the_request)
{
	return SCI_SUCCESS;
}

/**
 *
 * @this_rnc:
 * @the_callback:
 * @callback_parameter:
 *
 * This method handles destruct calls from the various state handlers.  The
 * remote node context can be requested to destroy from any state. If there was
 * a user callback it is always replaced with the request to destroy user
 * callback. enum sci_status
 */
static enum sci_status scic_sds_remote_node_context_general_destruct_handler(
	struct scic_sds_remote_node_context *this_rnc,
	SCICS_SDS_REMOTE_NODE_CONTEXT_CALLBACK the_callback,
	void *callback_parameter)
{
	scic_sds_remote_node_context_setup_to_destory(
		this_rnc, the_callback, callback_parameter
		);

	sci_base_state_machine_change_state(
		&this_rnc->state_machine,
		SCIC_SDS_REMOTE_NODE_CONTEXT_INVALIDATING_STATE
		);

	return SCI_SUCCESS;
}

/* --------------------------------------------------------------------------- */

static enum sci_status scic_sds_remote_node_context_initial_state_resume_handler(
	struct scic_sds_remote_node_context *this_rnc,
	SCICS_SDS_REMOTE_NODE_CONTEXT_CALLBACK the_callback,
	void *callback_parameter)
{
	if (this_rnc->remote_node_index != SCIC_SDS_REMOTE_NODE_CONTEXT_INVALID_INDEX) {
		scic_sds_remote_node_context_setup_to_resume(
			this_rnc, the_callback, callback_parameter
			);

		scic_sds_remote_node_context_construct_buffer(this_rnc);

		/*
		 * In the unlikely condition that we resume a remote node context
		 * after requesting to destroy the remote node context we must initialize
		 * the state logging on the resume request. */
		scic_sds_remote_node_context_initialize_state_logging(this_rnc);

		sci_base_state_machine_change_state(
			&this_rnc->state_machine,
			SCIC_SDS_REMOTE_NODE_CONTEXT_POSTING_STATE
			);

		return SCI_SUCCESS;
	}

	return SCI_FAILURE_INVALID_STATE;
}

/* --------------------------------------------------------------------------- */

static enum sci_status scic_sds_remote_node_context_posting_state_event_handler(
	struct scic_sds_remote_node_context *this_rnc,
	u32 event_code)
{
	enum sci_status status;

	switch (scu_get_event_code(event_code)) {
	case SCU_EVENT_POST_RNC_COMPLETE:
		status = SCI_SUCCESS;

		sci_base_state_machine_change_state(
			&this_rnc->state_machine,
			SCIC_SDS_REMOTE_NODE_CONTEXT_READY_STATE
			);
		break;

	default:
		status = SCI_FAILURE;
		SCIC_LOG_WARNING((
					 sci_base_object_get_logger(this_rnc->device),
					 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
					 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
					 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
					 "SCIC Remote Node Context 0x%x requested to process unexpected event 0x%x while in posting state\n",
					 this_rnc, event_code
					 ));
		break;
	}

	return status;
}

/* --------------------------------------------------------------------------- */

static enum sci_status scic_sds_remote_node_context_invalidating_state_destruct_handler(
	struct scic_sds_remote_node_context *this_rnc,
	SCICS_SDS_REMOTE_NODE_CONTEXT_CALLBACK the_callback,
	void *callback_parameter)
{
	scic_sds_remote_node_context_setup_to_destory(
		this_rnc, the_callback, callback_parameter
		);

	return SCI_SUCCESS;
}

static enum sci_status scic_sds_remote_node_context_invalidating_state_event_handler(
	struct scic_sds_remote_node_context *this_rnc,
	u32 event_code)
{
	enum sci_status status;

	if (scu_get_event_code(event_code) == SCU_EVENT_POST_RNC_INVALIDATE_COMPLETE) {
		status = SCI_SUCCESS;

		if (this_rnc->destination_state == SCIC_SDS_REMOTE_NODE_DESTINATION_STATE_FINAL) {
			sci_base_state_machine_change_state(
				&this_rnc->state_machine,
				SCIC_SDS_REMOTE_NODE_CONTEXT_INITIAL_STATE
				);
		} else {
			sci_base_state_machine_change_state(
				&this_rnc->state_machine,
				SCIC_SDS_REMOTE_NODE_CONTEXT_POSTING_STATE
				);
		}
	} else {
		switch (scu_get_event_type(event_code)) {
		case SCU_EVENT_TYPE_RNC_SUSPEND_TX:
		case SCU_EVENT_TYPE_RNC_SUSPEND_TX_RX:
			/*
			 * We really dont care if the hardware is going to suspend
			 * the device since it's being invalidated anyway */
			SCIC_LOG_INFO((
					      sci_base_object_get_logger(this_rnc->device),
					      SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
					      SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
					      SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
					      "SCIC Remote Node Context 0x%x was suspeneded by hardware while being invalidated.\n",
					      this_rnc
					      ));
			status = SCI_SUCCESS;
			break;

		default:
			SCIC_LOG_WARNING((
						 sci_base_object_get_logger(this_rnc->device),
						 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
						 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
						 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
						 "SCIC Remote Node Context 0x%x requested to process event 0x%x while in state %d.\n",
						 this_rnc, event_code, sci_base_state_machine_get_state(&this_rnc->state_machine)
						 ));
			status = SCI_FAILURE;
			break;
		}
	}

	return status;
}

/* --------------------------------------------------------------------------- */


static enum sci_status scic_sds_remote_node_context_resuming_state_event_handler(
	struct scic_sds_remote_node_context *this_rnc,
	u32 event_code)
{
	enum sci_status status;

	if (scu_get_event_code(event_code) == SCU_EVENT_POST_RCN_RELEASE) {
		status = SCI_SUCCESS;

		sci_base_state_machine_change_state(
			&this_rnc->state_machine,
			SCIC_SDS_REMOTE_NODE_CONTEXT_READY_STATE
			);
	} else {
		switch (scu_get_event_type(event_code)) {
		case SCU_EVENT_TYPE_RNC_SUSPEND_TX:
		case SCU_EVENT_TYPE_RNC_SUSPEND_TX_RX:
			/*
			 * We really dont care if the hardware is going to suspend
			 * the device since it's being resumed anyway */
			SCIC_LOG_INFO((
					      sci_base_object_get_logger(this_rnc->device),
					      SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
					      SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
					      SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
					      "SCIC Remote Node Context 0x%x was suspeneded by hardware while being resumed.\n",
					      this_rnc
					      ));
			status = SCI_SUCCESS;
			break;

		default:
			SCIC_LOG_WARNING((
						 sci_base_object_get_logger(this_rnc->device),
						 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
						 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
						 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
						 "SCIC Remote Node Context 0x%x requested to process event 0x%x while in state %d.\n",
						 this_rnc, event_code, sci_base_state_machine_get_state(&this_rnc->state_machine)
						 ));
			status = SCI_FAILURE;
			break;
		}
	}

	return status;
}

/* --------------------------------------------------------------------------- */

/**
 *
 * @this_rnc: The remote node context object being suspended.
 * @the_callback: The callback when the suspension is complete.
 * @callback_parameter: The parameter that is to be passed into the callback.
 *
 * This method will handle the suspend requests from the ready state.
 * SCI_SUCCESS
 */
static enum sci_status scic_sds_remote_node_context_ready_state_suspend_handler(
	struct scic_sds_remote_node_context *this_rnc,
	u32 suspend_type,
	SCICS_SDS_REMOTE_NODE_CONTEXT_CALLBACK the_callback,
	void *callback_parameter)
{
	this_rnc->user_callback   = the_callback;
	this_rnc->user_cookie     = callback_parameter;
	this_rnc->suspension_code = suspend_type;

	if (suspend_type == SCI_SOFTWARE_SUSPENSION) {
		scic_sds_remote_device_post_request(
			this_rnc->device,
			SCU_CONTEXT_COMMAND_POST_RNC_SUSPEND_TX
			);
	}

	sci_base_state_machine_change_state(
		&this_rnc->state_machine,
		SCIC_SDS_REMOTE_NODE_CONTEXT_AWAIT_SUSPENSION_STATE
		);

	return SCI_SUCCESS;
}

/**
 *
 * @this_rnc: The rnc for which the io request is targeted.
 * @the_request: The request which is going to be started.
 *
 * This method determines if the io request can be started by the SCU hardware.
 * When the RNC is in the ready state any io request can be started. enum sci_status
 * SCI_SUCCESS
 */
static enum sci_status scic_sds_remote_node_context_ready_state_start_io_handler(
	struct scic_sds_remote_node_context *this_rnc,
	struct scic_sds_request *the_request)
{
	return SCI_SUCCESS;
}


static enum sci_status scic_sds_remote_node_context_ready_state_event_handler(
	struct scic_sds_remote_node_context *this_rnc,
	u32 event_code)
{
	enum sci_status status;

	switch (scu_get_event_type(event_code)) {
	case SCU_EVENT_TL_RNC_SUSPEND_TX:
		sci_base_state_machine_change_state(
			&this_rnc->state_machine,
			SCIC_SDS_REMOTE_NODE_CONTEXT_TX_SUSPENDED_STATE
			);

		this_rnc->suspension_code = scu_get_event_specifier(event_code);
		status = SCI_SUCCESS;
		break;

	case SCU_EVENT_TL_RNC_SUSPEND_TX_RX:
		sci_base_state_machine_change_state(
			&this_rnc->state_machine,
			SCIC_SDS_REMOTE_NODE_CONTEXT_TX_RX_SUSPENDED_STATE
			);

		this_rnc->suspension_code = scu_get_event_specifier(event_code);
		status = SCI_SUCCESS;
		break;

	default:
		SCIC_LOG_WARNING((
					 sci_base_object_get_logger(this_rnc->device),
					 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
					 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
					 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
					 "SCIC Remote Node Context 0x%x requested to process event 0x%x while in state %d.\n",
					 this_rnc, event_code, sci_base_state_machine_get_state(&this_rnc->state_machine)
					 ));

		status = SCI_FAILURE;
		break;
	}

	return status;
}

/* --------------------------------------------------------------------------- */

static enum sci_status scic_sds_remote_node_context_tx_suspended_state_resume_handler(
	struct scic_sds_remote_node_context *this_rnc,
	SCICS_SDS_REMOTE_NODE_CONTEXT_CALLBACK the_callback,
	void *callback_parameter)
{
	enum sci_status status;
	struct smp_discover_response_protocols protocols;

	scic_sds_remote_node_context_setup_to_resume(
		this_rnc, the_callback, callback_parameter
		);

	/* TODO: consider adding a resume action of NONE, INVALIDATE, WRITE_TLCR */

	scic_remote_device_get_protocols(this_rnc->device, &protocols);

	if (
		(protocols.u.bits.attached_ssp_target == 1)
		|| (protocols.u.bits.attached_smp_target == 1)
		) {
		sci_base_state_machine_change_state(
			&this_rnc->state_machine,
			SCIC_SDS_REMOTE_NODE_CONTEXT_RESUMING_STATE
			);

		status = SCI_SUCCESS;
	} else if (protocols.u.bits.attached_stp_target == 1) {
		if (this_rnc->device->is_direct_attached) {
			/* @todo Fix this since I am being silly in writing to the STPTLDARNI register. */
			scic_sds_port_set_direct_attached_device_id(
				this_rnc->device->owning_port,
				this_rnc->remote_node_index
				);

			sci_base_state_machine_change_state(
				&this_rnc->state_machine,
				SCIC_SDS_REMOTE_NODE_CONTEXT_RESUMING_STATE
				);
		} else {
			sci_base_state_machine_change_state(
				&this_rnc->state_machine,
				SCIC_SDS_REMOTE_NODE_CONTEXT_INVALIDATING_STATE
				);
		}

		status = SCI_SUCCESS;
	} else {
		status = SCI_FAILURE;
	}

	return status;
}

/**
 *
 * @this_rnc: The remote node context which is to receive the task request.
 * @the_request: The task request to be transmitted to to the remote target
 *    device.
 *
 * This method will report a success or failure attempt to start a new task
 * request to the hardware.  Since all task requests are sent on the high
 * priority queue they can be sent when the RCN is in a TX suspend state.
 * enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_remote_node_context_suspended_start_task_handler(
	struct scic_sds_remote_node_context *this_rnc,
	struct scic_sds_request *the_request)
{
	scic_sds_remote_node_context_resume(this_rnc, NULL, NULL);

	return SCI_SUCCESS;
}

/* --------------------------------------------------------------------------- */

static enum sci_status scic_sds_remote_node_context_tx_rx_suspended_state_resume_handler(
	struct scic_sds_remote_node_context *this_rnc,
	SCICS_SDS_REMOTE_NODE_CONTEXT_CALLBACK the_callback,
	void *callback_parameter)
{
	scic_sds_remote_node_context_setup_to_resume(
		this_rnc, the_callback, callback_parameter
		);

	sci_base_state_machine_change_state(
		&this_rnc->state_machine,
		SCIC_SDS_REMOTE_NODE_CONTEXT_RESUMING_STATE
		);

	return SCI_FAILURE_INVALID_STATE;
}

/* --------------------------------------------------------------------------- */

/**
 *
 *
 *
 */
static enum sci_status scic_sds_remote_node_context_await_suspension_state_resume_handler(
	struct scic_sds_remote_node_context *this_rnc,
	SCICS_SDS_REMOTE_NODE_CONTEXT_CALLBACK the_callback,
	void *callback_parameter)
{
	scic_sds_remote_node_context_setup_to_resume(
		this_rnc, the_callback, callback_parameter
		);

	return SCI_SUCCESS;
}

/**
 *
 * @this_rnc: The remote node context which is to receive the task request.
 * @the_request: The task request to be transmitted to to the remote target
 *    device.
 *
 * This method will report a success or failure attempt to start a new task
 * request to the hardware.  Since all task requests are sent on the high
 * priority queue they can be sent when the RCN is in a TX suspend state.
 * enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_remote_node_context_await_suspension_state_start_task_handler(
	struct scic_sds_remote_node_context *this_rnc,
	struct scic_sds_request *the_request)
{
	return SCI_SUCCESS;
}

static enum sci_status scic_sds_remote_node_context_await_suspension_state_event_handler(
	struct scic_sds_remote_node_context *this_rnc,
	u32 event_code)
{
	enum sci_status status;

	switch (scu_get_event_type(event_code)) {
	case SCU_EVENT_TL_RNC_SUSPEND_TX:
		sci_base_state_machine_change_state(
			&this_rnc->state_machine,
			SCIC_SDS_REMOTE_NODE_CONTEXT_TX_SUSPENDED_STATE
			);

		this_rnc->suspension_code = scu_get_event_specifier(event_code);
		status = SCI_SUCCESS;
		break;

	case SCU_EVENT_TL_RNC_SUSPEND_TX_RX:
		sci_base_state_machine_change_state(
			&this_rnc->state_machine,
			SCIC_SDS_REMOTE_NODE_CONTEXT_TX_RX_SUSPENDED_STATE
			);

		this_rnc->suspension_code = scu_get_event_specifier(event_code);
		status = SCI_SUCCESS;
		break;

	default:
		SCIC_LOG_WARNING((
					 sci_base_object_get_logger(this_rnc->device),
					 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
					 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
					 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
					 "SCIC Remote Node Context 0x%x requested to process event 0x%x while in state %d.\n",
					 this_rnc, event_code, sci_base_state_machine_get_state(&this_rnc->state_machine)
					 ));

		status = SCI_FAILURE;
		break;
	}

	return status;
}

/* --------------------------------------------------------------------------- */

struct scic_sds_remote_node_context_handlers
scic_sds_remote_node_context_state_handler_table[
	SCIC_SDS_REMOTE_NODE_CONTEXT_MAX_STATES] =
{
	/* SCIC_SDS_REMOTE_NODE_CONTEXT_INITIAL_STATE */
	{
		scic_sds_remote_node_context_default_destruct_handler,
		scic_sds_remote_node_context_default_suspend_handler,
		scic_sds_remote_node_context_initial_state_resume_handler,
		scic_sds_remote_node_context_default_start_io_handler,
		scic_sds_remote_node_context_default_start_task_handler,
		scic_sds_remote_node_context_default_event_handler
	},
	/* SCIC_SDS_REMOTE_NODE_CONTEXT_POSTING_STATE */
	{
		scic_sds_remote_node_context_general_destruct_handler,
		scic_sds_remote_node_context_default_suspend_handler,
		scic_sds_remote_node_context_continue_to_resume_handler,
		scic_sds_remote_node_context_default_start_io_handler,
		scic_sds_remote_node_context_default_start_task_handler,
		scic_sds_remote_node_context_posting_state_event_handler
	},
	/* SCIC_SDS_REMOTE_NODE_CONTEXT_INVALIDATING_STATE */
	{
		scic_sds_remote_node_context_invalidating_state_destruct_handler,
		scic_sds_remote_node_context_default_suspend_handler,
		scic_sds_remote_node_context_continue_to_resume_handler,
		scic_sds_remote_node_context_default_start_io_handler,
		scic_sds_remote_node_context_default_start_task_handler,
		scic_sds_remote_node_context_invalidating_state_event_handler
	},
	/* SCIC_SDS_REMOTE_NODE_CONTEXT_RESUMING_STATE */
	{
		scic_sds_remote_node_context_general_destruct_handler,
		scic_sds_remote_node_context_default_suspend_handler,
		scic_sds_remote_node_context_continue_to_resume_handler,
		scic_sds_remote_node_context_default_start_io_handler,
		scic_sds_remote_node_context_success_start_task_handler,
		scic_sds_remote_node_context_resuming_state_event_handler
	},
	/* SCIC_SDS_REMOTE_NODE_CONTEXT_READY_STATE */
	{
		scic_sds_remote_node_context_general_destruct_handler,
		scic_sds_remote_node_context_ready_state_suspend_handler,
		scic_sds_remote_node_context_default_resume_handler,
		scic_sds_remote_node_context_ready_state_start_io_handler,
		scic_sds_remote_node_context_success_start_task_handler,
		scic_sds_remote_node_context_ready_state_event_handler
	},
	/* SCIC_SDS_REMOTE_NODE_CONTEXT_TX_SUSPENDED_STATE */
	{
		scic_sds_remote_node_context_general_destruct_handler,
		scic_sds_remote_node_context_default_suspend_handler,
		scic_sds_remote_node_context_tx_suspended_state_resume_handler,
		scic_sds_remote_node_context_default_start_io_handler,
		scic_sds_remote_node_context_suspended_start_task_handler,
		scic_sds_remote_node_context_default_event_handler
	},
	/* SCIC_SDS_REMOTE_NODE_CONTEXT_TX_RX_SUSPENDED_STATE */
	{
		scic_sds_remote_node_context_general_destruct_handler,
		scic_sds_remote_node_context_default_suspend_handler,
		scic_sds_remote_node_context_tx_rx_suspended_state_resume_handler,
		scic_sds_remote_node_context_default_start_io_handler,
		scic_sds_remote_node_context_suspended_start_task_handler,
		scic_sds_remote_node_context_default_event_handler
	},
	/* SCIC_SDS_REMOTE_NODE_CONTEXT_AWAIT_SUSPENSION_STATE */
	{
		scic_sds_remote_node_context_general_destruct_handler,
		scic_sds_remote_node_context_default_suspend_handler,
		scic_sds_remote_node_context_await_suspension_state_resume_handler,
		scic_sds_remote_node_context_default_start_io_handler,
		scic_sds_remote_node_context_await_suspension_state_start_task_handler,
		scic_sds_remote_node_context_await_suspension_state_event_handler
	}
};

