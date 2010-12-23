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
#if !defined(DISABLE_ATAPI)

#include "intel_ata.h"
#include "intel_sat.h"
#include "sci_base_state.h"
#include "scic_sds_request.h"
#include "scic_sds_controller.h"
#include "scic_sds_remote_device.h"
#include "scic_sds_stp_packet_request.h"
#include "scic_user_callback.h"

/**
 * This file contains the Packet IO started substate machine for the
 *    SCIC_SDS_IO_REQUEST object.
 *
 *
 */
void scic_sds_stp_packet_request_started_packet_phase_await_tc_completion_enter(
	struct sci_base_object *object)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)object;

	SET_STATE_HANDLER(
		this_request,
		scic_sds_stp_packet_request_started_substate_handler_table,
		SCIC_SDS_STP_PACKET_REQUEST_STARTED_PACKET_PHASE_AWAIT_TC_COMPLETION_SUBSTATE
		);

	scic_sds_remote_device_set_working_request(
		this_request->target_device, this_request
		);
}

void scic_sds_stp_packet_request_started_packet_phase_await_pio_setup_enter(
	struct sci_base_object *object)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)object;

	SET_STATE_HANDLER(
		this_request,
		scic_sds_stp_packet_request_started_substate_handler_table,
		SCIC_SDS_STP_PACKET_REQUEST_STARTED_PACKET_PHASE_AWAIT_PIO_SETUP_SUBSTATE
		);
}

void scic_sds_stp_packet_request_started_command_phase_await_tc_completion_enter(
	struct sci_base_object *object)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)object;
	u8 sat_packet_protocol =
		scic_cb_request_get_sat_protocol(this_request->user_request);

	struct scu_task_context *task_context;
	enum sci_status status;

	/*
	 * Recycle the TC and reconstruct it for sending out data fis containing
	 * CDB. */
	task_context = scic_sds_controller_get_task_context_buffer(
		this_request->owning_controller, this_request->io_tag);

	if (sat_packet_protocol == SAT_PROTOCOL_PACKET_NON_DATA)
		scu_stp_packet_request_command_phase_reconstruct_raw_frame_task_context(
			this_request, task_context);
	else
		scu_stp_packet_request_command_phase_construct_task_context(
			this_request, task_context);

	/* send the new TC out. */
	status = this_request->owning_controller->state_handlers->parent.continue_io_handler(
		&this_request->owning_controller->parent,
		&this_request->target_device->parent,
		&this_request->parent
		);

	if (status == SCI_SUCCESS)
		SET_STATE_HANDLER(
			this_request,
			scic_sds_stp_packet_request_started_substate_handler_table,
			SCIC_SDS_STP_PACKET_REQUEST_STARTED_COMMAND_PHASE_AWAIT_TC_COMPLETION_SUBSTATE
			);
}

void scic_sds_stp_packet_request_started_command_phase_await_d2h_fis_enter(
	struct sci_base_object *object)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)object;

	SET_STATE_HANDLER(
		this_request,
		scic_sds_stp_packet_request_started_substate_handler_table,
		SCIC_SDS_STP_PACKET_REQUEST_STARTED_COMMAND_PHASE_AWAIT_D2H_FIS_SUBSTATE
		);
}

void scic_sds_stp_packet_request_started_completion_delay_enter(
	struct sci_base_object *object)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)object;

	SET_STATE_HANDLER(
		this_request,
		scic_sds_stp_packet_request_started_substate_handler_table,
		SCIC_SDS_STP_PACKET_REQUEST_STARTED_COMPLETION_DELAY_SUBSTATE
		);
}


/* --------------------------------------------------------------------------- */
const struct sci_base_state scic_sds_stp_packet_request_started_substate_table[] = {
	[SCIC_SDS_STP_PACKET_REQUEST_STARTED_PACKET_PHASE_AWAIT_TC_COMPLETION_SUBSTATE] = {
		.enter_state = scic_sds_stp_packet_request_started_packet_phase_await_tc_completion_enter,
	},
	[SCIC_SDS_STP_PACKET_REQUEST_STARTED_PACKET_PHASE_AWAIT_PIO_SETUP_SUBSTATE] = {
		.enter_state = scic_sds_stp_packet_request_started_packet_phase_await_pio_setup_enter,
	},
	[SCIC_SDS_STP_PACKET_REQUEST_STARTED_COMMAND_PHASE_AWAIT_TC_COMPLETION_SUBSTATE] = {
		.enter_state = scic_sds_stp_packet_request_started_command_phase_await_tc_completion_enter,
	},
	[SCIC_SDS_STP_PACKET_REQUEST_STARTED_COMMAND_PHASE_AWAIT_D2H_FIS_SUBSTATE] = {
		.enter_state = scic_sds_stp_packet_request_started_command_phase_await_d2h_fis_enter,
	},
	[SCIC_SDS_STP_PACKET_REQUEST_STARTED_COMPLETION_DELAY_SUBSTATE] = {
		.enter_state scic_sds_stp_packet_request_started_completion_delay_enter,
	}
};

#endif /* !defined(DISABLE_ATAPI) */
