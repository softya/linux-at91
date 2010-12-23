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

#include "intel_ata.h"
#include "sci_base_state.h"
#include "scic_io_request.h"
#include "scic_sds_controller.h"
#include "scic_sds_remote_device.h"
#include "scic_sds_stp_request.h"

/**
 * This file contains the SATA/STP soft reset started substate machine for the
 *    SCIC_SDS_IO_REQUEST object.
 *
 *
 */

static void scic_sds_stp_request_started_soft_reset_await_h2d_asserted_completion_enter(
	struct sci_base_object *object)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)object;

	SET_STATE_HANDLER(
		this_request,
		scic_sds_stp_request_started_soft_reset_substate_handler_table,
		SCIC_SDS_STP_REQUEST_STARTED_SOFT_RESET_AWAIT_H2D_ASSERTED_COMPLETION_SUBSTATE
		);

	scic_sds_remote_device_set_working_request(
		this_request->target_device, this_request
		);
}

static void scic_sds_stp_request_started_soft_reset_await_h2d_diagnostic_completion_enter(
	struct sci_base_object *object)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)object;
	sci_base_controller_request_handler_t continue_io;
	struct scu_task_context *task_context;
	struct sata_fis_reg_h2d *h2d_fis;
	struct scic_sds_controller *scic;
	enum sci_status status;
	u32 state;

	/* Clear the SRST bit */
	h2d_fis = scic_stp_io_request_get_h2d_reg_address(this_request);
	h2d_fis->control = 0;

	/* Clear the TC control bit */
	task_context = scic_sds_controller_get_task_context_buffer(
		this_request->owning_controller, this_request->io_tag);
	task_context->control_frame = 0;

	scic = this_request->owning_controller;
	state = scic->parent.state_machine.current_state_id;
	continue_io = scic_sds_controller_state_handler_table[state].base.continue_io;

	status = continue_io(&scic->parent, &this_request->target_device->parent,
			     &this_request->parent);

	if (status == SCI_SUCCESS) {
		SET_STATE_HANDLER(
			this_request,
			scic_sds_stp_request_started_soft_reset_substate_handler_table,
			SCIC_SDS_STP_REQUEST_STARTED_SOFT_RESET_AWAIT_H2D_DIAGNOSTIC_COMPLETION_SUBSTATE
			);
	}
}

static void scic_sds_stp_request_started_soft_reset_await_d2h_response_enter(
	struct sci_base_object *object)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)object;

	SET_STATE_HANDLER(
		this_request,
		scic_sds_stp_request_started_soft_reset_substate_handler_table,
		SCIC_SDS_STP_REQUEST_STARTED_SOFT_RESET_AWAIT_D2H_RESPONSE_FRAME_SUBSTATE
		);
}

/* --------------------------------------------------------------------------- */

const struct sci_base_state scic_sds_stp_request_started_soft_reset_substate_table[] = {
	[SCIC_SDS_STP_REQUEST_STARTED_SOFT_RESET_AWAIT_H2D_ASSERTED_COMPLETION_SUBSTATE] = {
		.enter_state = scic_sds_stp_request_started_soft_reset_await_h2d_asserted_completion_enter,
	},
	[SCIC_SDS_STP_REQUEST_STARTED_SOFT_RESET_AWAIT_H2D_DIAGNOSTIC_COMPLETION_SUBSTATE] = {
		.enter_state = scic_sds_stp_request_started_soft_reset_await_h2d_diagnostic_completion_enter,
	},
	[SCIC_SDS_STP_REQUEST_STARTED_SOFT_RESET_AWAIT_D2H_RESPONSE_FRAME_SUBSTATE] = {
		.enter_state = scic_sds_stp_request_started_soft_reset_await_d2h_response_enter,
	},
};

