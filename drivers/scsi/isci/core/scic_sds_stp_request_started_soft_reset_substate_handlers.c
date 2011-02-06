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
#include "intel_sata.h"
#include "sci_types.h"
#include "sci_util.h"
#include "scic_user_callback.h"
#include "scic_sds_logger.h"
#include "scic_sds_controller.h"
#include "scic_sds_request.h"
#include "scic_sds_stp_request.h"
#include "scic_sds_stp_pio_request.h"
#include "scu_completion_codes.h"
#include "scu_event_codes.h"

/**
 * This file contains all of the handler method implementations for the
 *    SATA/STP Non-Data IO request protocol state machine.
 *
 *
 */

/**
 *
 * @this_request:
 * @completion_code:
 *
 * This method processes a TC completion.  The expected TC completion is for
 * the transmission of the H2D register FIS containing the SATA/STP non-data
 * request. This method always successfully processes the TC completion.
 * SCI_SUCCESS This value is always returned.
 */
static enum sci_status scic_sds_stp_request_soft_reset_await_h2d_asserted_tc_completion_handler(
	struct scic_sds_request *this_request,
	u32 completion_code)
{
	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_request),
			       SCIC_LOG_OBJECT_STP_IO_REQUEST,
			       "scic_sds_stp_request_soft_reset_await_h2d_tc_completion_handler(0x%x, 0x%x) enter\n",
			       this_request, completion_code
			       ));

	switch (SCU_GET_COMPLETION_TL_STATUS(completion_code)) {
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_GOOD):
		scic_sds_request_set_status(
			this_request, SCU_TASK_DONE_GOOD, SCI_SUCCESS
			);

		sci_base_state_machine_change_state(
			&this_request->started_substate_machine,
			SCIC_SDS_STP_REQUEST_STARTED_SOFT_RESET_AWAIT_H2D_DIAGNOSTIC_COMPLETION_SUBSTATE
			);
		break;

	default:
		/*
		 * All other completion status cause the IO to be complete.  If a NAK
		 * was received, then it is up to the user to retry the request. */
		scic_sds_request_set_status(
			this_request,
			SCU_NORMALIZE_COMPLETION_STATUS(completion_code),
			SCI_FAILURE_CONTROLLER_SPECIFIC_IO_ERR
			);

		sci_base_state_machine_change_state(
			&this_request->parent.state_machine, SCI_BASE_REQUEST_STATE_COMPLETED
			);
		break;
	}

	return SCI_SUCCESS;
}

/**
 *
 * @this_request:
 * @completion_code:
 *
 * This method processes a TC completion.  The expected TC completion is for
 * the transmission of the H2D register FIS containing the SATA/STP non-data
 * request. This method always successfully processes the TC completion.
 * SCI_SUCCESS This value is always returned.
 */
static enum sci_status scic_sds_stp_request_soft_reset_await_h2d_diagnostic_tc_completion_handler(
	struct scic_sds_request *this_request,
	u32 completion_code)
{
	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_request),
			       SCIC_LOG_OBJECT_STP_IO_REQUEST,
			       "scic_sds_stp_request_soft_reset_await_h2d_tc_completion_handler(0x%x, 0x%x) enter\n",
			       this_request, completion_code
			       ));

	switch (SCU_GET_COMPLETION_TL_STATUS(completion_code)) {
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_GOOD):
		scic_sds_request_set_status(
			this_request, SCU_TASK_DONE_GOOD, SCI_SUCCESS
			);

		sci_base_state_machine_change_state(
			&this_request->started_substate_machine,
			SCIC_SDS_STP_REQUEST_STARTED_SOFT_RESET_AWAIT_D2H_RESPONSE_FRAME_SUBSTATE
			);
		break;

	default:
		/*
		 * All other completion status cause the IO to be complete.  If a NAK
		 * was received, then it is up to the user to retry the request. */
		scic_sds_request_set_status(
			this_request,
			SCU_NORMALIZE_COMPLETION_STATUS(completion_code),
			SCI_FAILURE_CONTROLLER_SPECIFIC_IO_ERR
			);

		sci_base_state_machine_change_state(
			&this_request->parent.state_machine, SCI_BASE_REQUEST_STATE_COMPLETED
			);
		break;
	}

	return SCI_SUCCESS;
}

/**
 *
 * @request: This parameter specifies the request for which a frame has been
 *    received.
 * @frame_index: This parameter specifies the index of the frame that has been
 *    received.
 *
 * This method processes frames received from the target while waiting for a
 * device to host register FIS.  If a non-register FIS is received during this
 * time, it is treated as a protocol violation from an IO perspective. Indicate
 * if the received frame was processed successfully.
 */
static enum sci_status scic_sds_stp_request_soft_reset_await_d2h_frame_handler(
	struct scic_sds_request *request,
	u32 frame_index)
{
	enum sci_status status;
	struct sata_fis_header *frame_header;
	u32 *frame_buffer;
	struct scic_sds_stp_request *this_request = (struct scic_sds_stp_request *)request;

	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_request),
			       SCIC_LOG_OBJECT_STP_IO_REQUEST,
			       "scic_sds_stp_request_soft_reset_await_d2h_frame_handler(0x%x, 0x%x) enter\n",
			       this_request, frame_index
			       ));

	status = scic_sds_unsolicited_frame_control_get_header(
		&(this_request->parent.owning_controller->uf_control),
		frame_index,
		(void **)&frame_header
		);

	if (status == SCI_SUCCESS) {
		switch (frame_header->fis_type) {
		case SATA_FIS_TYPE_REGD2H:
			scic_sds_unsolicited_frame_control_get_buffer(
				&(this_request->parent.owning_controller->uf_control),
				frame_index,
				(void **)&frame_buffer
				);

			scic_sds_controller_copy_sata_response(
				&this_request->d2h_reg_fis, (u32 *)frame_header, frame_buffer
				);

			/* The command has completed with error */
			scic_sds_request_set_status(
				&this_request->parent,
				SCU_TASK_DONE_CHECK_RESPONSE,
				SCI_FAILURE_IO_RESPONSE_VALID
				);
			break;

		default:
			SCIC_LOG_WARNING((
						 sci_base_object_get_logger(this_request),
						 SCIC_LOG_OBJECT_STP_IO_REQUEST,
						 "IO Request:0x%x Frame Id:%d protocol violation occurred\n",
						 this_request, frame_index
						 ));

			scic_sds_request_set_status(
				&this_request->parent,
				SCU_TASK_DONE_UNEXP_FIS,
				SCI_FAILURE_PROTOCOL_VIOLATION
				);
			break;
		}

		sci_base_state_machine_change_state(
			&this_request->parent.parent.state_machine,
			SCI_BASE_REQUEST_STATE_COMPLETED
			);

		/* Frame has been decoded return it to the controller */
		scic_sds_controller_release_frame(
			this_request->parent.owning_controller, frame_index
			);
	} else {
		SCIC_LOG_ERROR((
				       sci_base_object_get_logger(this_request),
				       SCIC_LOG_OBJECT_STP_IO_REQUEST,
				       "SCIC IO Request 0x%x could not get frame header for frame index %d, status %x\n",
				       this_request, frame_index, status
				       ));
	}

	return status;
}

/* --------------------------------------------------------------------------- */

struct scic_sds_io_request_state_handler
scic_sds_stp_request_started_soft_reset_substate_handler_table
[SCIC_SDS_STP_REQUEST_STARTED_SOFT_RESET_MAX_SUBSTATES] =
{
	/* SCIC_SDS_STP_REQUEST_STARTED_SOFT_RESET_AWAIT_H2D_ASSERTED_COMPLETION_SUBSTATE */
	{
		{
			scic_sds_request_default_start_handler,
			scic_sds_request_started_state_abort_handler,
			scic_sds_request_default_complete_handler,
			scic_sds_request_default_destruct_handler
		},
		scic_sds_stp_request_soft_reset_await_h2d_asserted_tc_completion_handler,
		scic_sds_request_default_event_handler,
		scic_sds_request_default_frame_handler
	},
	/* SCIC_SDS_STP_REQUEST_STARTED_SOFT_RESET_AWAIT_H2D_DIAGNOSTIC_COMPLETION_SUBSTATE */
	{
		{
			scic_sds_request_default_start_handler,
			scic_sds_request_started_state_abort_handler,
			scic_sds_request_default_complete_handler,
			scic_sds_request_default_destruct_handler
		},
		scic_sds_stp_request_soft_reset_await_h2d_diagnostic_tc_completion_handler,
		scic_sds_request_default_event_handler,
		scic_sds_request_default_frame_handler
	},
	/* SCIC_SDS_STP_REQUEST_STARTED_SOFT_RESET_AWAIT_D2H_RESPONSE_FRAME_SUBSTATE */
	{
		{
			scic_sds_request_default_start_handler,
			scic_sds_request_started_state_abort_handler,
			scic_sds_request_default_complete_handler,
			scic_sds_request_default_destruct_handler
		},
		scic_sds_request_default_tc_completion_handler,
		scic_sds_request_default_event_handler,
		scic_sds_stp_request_soft_reset_await_d2h_frame_handler
	}
};

