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
 * This file contains the implementation of the state handlers for the UDMA STP
 *    IO Request state machine
 *
 *
 */

#include "intel_sata.h"
#include "scic_sds_controller.h"
#include "scic_sds_remote_device.h"
#include "scic_sds_stp_request.h"
#include "scu_completion_codes.h"
#include "scu_event_codes.h"

static void scic_sds_stp_request_udma_complete_request(
	struct scic_sds_request *this_request,
	u32 scu_status,
	enum sci_status sci_status)
{
	scic_sds_request_set_status(
		this_request, scu_status, sci_status
		);

	sci_base_state_machine_change_state(
		&this_request->parent.state_machine,
		SCI_BASE_REQUEST_STATE_COMPLETED
		);
}

/**
 *
 * @this_request:
 * @frame_index:
 *
 * enum sci_status
 */
static enum sci_status scic_sds_stp_request_udma_general_frame_handler(
	struct scic_sds_request *this_request,
	u32 frame_index)
{
	enum sci_status status;
	struct sata_fis_header *frame_header;
	u32 *frame_buffer;

	status = scic_sds_unsolicited_frame_control_get_header(
		&this_request->owning_controller->uf_control,
		frame_index,
		(void **)&frame_header
		);

	if (
		(status == SCI_SUCCESS)
		&& (frame_header->fis_type == SATA_FIS_TYPE_REGD2H)
		) {
		scic_sds_unsolicited_frame_control_get_buffer(
			&this_request->owning_controller->uf_control,
			frame_index,
			(void **)&frame_buffer
			);

		scic_sds_controller_copy_sata_response(
			&((struct scic_sds_stp_request *)this_request)->d2h_reg_fis,
			(u32 *)frame_header,
			frame_buffer
			);
	}

	scic_sds_controller_release_frame(
		this_request->owning_controller, frame_index);

	return status;
}

/**
 * This method process TC completions while in the state where we are waiting
 *    for TC completions.
 * @this_request:
 * @completion_code:
 *
 * enum sci_status
 */
static enum sci_status scic_sds_stp_request_udma_await_tc_completion_tc_completion_handler(
	struct scic_sds_request *request,
	u32 completion_code)
{
	enum sci_status status = SCI_SUCCESS;
	struct scic_sds_stp_request *this_request = (struct scic_sds_stp_request *)request;

	switch (SCU_GET_COMPLETION_TL_STATUS(completion_code)) {
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_GOOD):
		scic_sds_stp_request_udma_complete_request(
			&this_request->parent, SCU_TASK_DONE_GOOD, SCI_SUCCESS
			);
		break;

	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_UNEXP_FIS):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_REG_ERR):
		/*
		 * We must check ther response buffer to see if the D2H Register FIS was
		 * received before we got the TC completion. */
		if (this_request->d2h_reg_fis.fis_type == SATA_FIS_TYPE_REGD2H) {
			scic_sds_remote_device_suspend(
				this_request->parent.target_device,
				SCU_EVENT_SPECIFIC(SCU_NORMALIZE_COMPLETION_STATUS(completion_code))
				);

			scic_sds_stp_request_udma_complete_request(
				&this_request->parent,
				SCU_TASK_DONE_CHECK_RESPONSE,
				SCI_FAILURE_IO_RESPONSE_VALID
				);
		} else {
			/*
			 * If we have an error completion status for the TC then we can expect a
			 * D2H register FIS from the device so we must change state to wait for it */
			sci_base_state_machine_change_state(
				&this_request->parent.started_substate_machine,
				SCIC_SDS_STP_REQUEST_STARTED_UDMA_AWAIT_D2H_REG_FIS_SUBSTATE
				);
		}
		break;

	/*
	 * / @todo Check to see if any of these completion status need to wait for
	 * /       the device to host register fis. */
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_INV_FIS_LEN):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_MAX_PLD_ERR):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_LL_R_ERR):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_CRC_ERR):
		scic_sds_remote_device_suspend(
			this_request->parent.target_device,
			SCU_EVENT_SPECIFIC(SCU_NORMALIZE_COMPLETION_STATUS(completion_code))
			);
	/* Fall through to the default case */
	default:
		/* All other completion status cause the IO to be complete. */
		scic_sds_stp_request_udma_complete_request(
			&this_request->parent,
			SCU_NORMALIZE_COMPLETION_STATUS(completion_code),
			SCI_FAILURE_CONTROLLER_SPECIFIC_IO_ERR
			);
		break;
	}

	return status;
}

static enum sci_status scic_sds_stp_request_udma_await_d2h_reg_fis_frame_handler(
	struct scic_sds_request *this_request,
	u32 frame_index)
{
	enum sci_status status;

	/* Use the general frame handler to copy the resposne data */
	status = scic_sds_stp_request_udma_general_frame_handler(this_request, frame_index);

	if (status == SCI_SUCCESS) {
		scic_sds_stp_request_udma_complete_request(
			this_request,
			SCU_TASK_DONE_CHECK_RESPONSE,
			SCI_FAILURE_IO_RESPONSE_VALID
			);
	}

	return status;
}

/* --------------------------------------------------------------------------- */

const struct scic_sds_io_request_state_handler scic_sds_stp_request_started_udma_substate_handler_table[] = {
	[SCIC_SDS_STP_REQUEST_STARTED_UDMA_AWAIT_TC_COMPLETION_SUBSTATE] = {
		.parent.start_handler    = scic_sds_request_default_start_handler,
		.parent.abort_handler    = scic_sds_request_started_state_abort_handler,
		.parent.complete_handler = scic_sds_request_default_complete_handler,
		.parent.destruct_handler = scic_sds_request_default_destruct_handler,
		.tc_completion_handler   = scic_sds_stp_request_udma_await_tc_completion_tc_completion_handler,
		.event_handler           = scic_sds_request_default_event_handler,
		.frame_handler           = scic_sds_stp_request_udma_general_frame_handler,
	},
	[SCIC_SDS_STP_REQUEST_STARTED_UDMA_AWAIT_D2H_REG_FIS_SUBSTATE] = {
		.parent.start_handler    = scic_sds_request_default_start_handler,
		.parent.abort_handler    = scic_sds_request_started_state_abort_handler,
		.parent.complete_handler = scic_sds_request_default_complete_handler,
		.parent.destruct_handler = scic_sds_request_default_destruct_handler,
		.tc_completion_handler   = scic_sds_request_default_tc_completion_handler,
		.event_handler           = scic_sds_request_default_event_handler,
		.frame_handler           = scic_sds_stp_request_udma_await_d2h_reg_fis_frame_handler,
	},
};
