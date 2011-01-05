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

/**
 * This file contains the SMP request started substate handlers.
 *
 *
 */

#include "intel_sas.h"
#include "intel_sat.h"
#include "sci_util.h"
#include "scic_sds_request.h"
#include "scic_controller.h"
#include "scic_sds_logger.h"
#include "scic_sds_controller.h"
#include "scu_completion_codes.h"
#include "scu_task_context.h"
#include "scic_sds_stp_packet_request.h"
#include "scic_sds_remote_device.h"


/**
 * This method processes the completions transport layer (TL) status to
 *    determine if the Packet FIS was sent successfully. If the Packet FIS was
 *    sent successfully, then the state for the Packet request transits to
 *    waiting for a PIO SETUP frame.
 * @this_request: This parameter specifies the request for which the TC
 *    completion was received.
 * @completion_code: This parameter indicates the completion status information
 *    for the TC.
 *
 * Indicate if the tc completion handler was successful. SCI_SUCCESS currently
 * this method always returns success.
 */
enum sci_status scic_sds_stp_packet_request_packet_phase_await_tc_completion_tc_completion_handler(
	struct scic_sds_request *this_request,
	u32 completion_code)
{
	enum sci_status status = SCI_SUCCESS;

	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_request),
			       SCIC_LOG_OBJECT_STP_IO_REQUEST,
			       "scic_sds_stp_packet_request_packet_phase_await_tc_completion_tc_completion_handler(0x%x, 0x%x) enter\n",
			       this_request, completion_code
			       ));

	switch (SCU_GET_COMPLETION_TL_STATUS(completion_code)) {
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_GOOD):
		scic_sds_request_set_status(
			this_request, SCU_TASK_DONE_GOOD, SCI_SUCCESS
			);

		sci_base_state_machine_change_state(
			&this_request->started_substate_machine,
			SCIC_SDS_STP_PACKET_REQUEST_STARTED_PACKET_PHASE_AWAIT_PIO_SETUP_SUBSTATE
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
			&this_request->parent.state_machine,
			SCI_BASE_REQUEST_STATE_COMPLETED
			);
		break;
	}

	return status;
}


/**
 * This method processes an unsolicited frame while the Packet request is
 *    waiting for a PIO SETUP FIS.  It will release the unsolicited frame, and
 *    transition the request to the COMMAND_PHASE_AWAIT_TC_COMPLETION_SUBSTATE
 *    state.
 * @this_request: This parameter specifies the request for which the
 *    unsolicited frame was received.
 * @frame_index: This parameter indicates the unsolicited frame index that
 *    should contain the response.
 *
 * This method returns an indication of whether the pio setup frame was handled
 * successfully or not. SCI_SUCCESS Currently this value is always returned and
 * indicates successful processing of the TC response.
 */
enum sci_status scic_sds_stp_packet_request_packet_phase_await_pio_setup_frame_handler(
	struct scic_sds_request *request,
	u32 frame_index)
{
	enum sci_status status;
	struct sata_fis_header *frame_header;
	u32 *frame_buffer;
	struct scic_sds_stp_request *this_request;

	this_request = (struct scic_sds_stp_request *)request;

	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_request),
			       SCIC_LOG_OBJECT_STP_IO_REQUEST,
			       "scic_sds_stp_packet_request_packet_phase_await_pio_setup_frame_handler(0x%x, 0x%x) enter\n",
			       this_request, frame_index
			       ));

	status = scic_sds_unsolicited_frame_control_get_header(
		&(this_request->parent.owning_controller->uf_control),
		frame_index,
		(void **)&frame_header
		);

	if (status == SCI_SUCCESS) {
		ASSERT(frame_header->fis_type == SATA_FIS_TYPE_PIO_SETUP);

		/*
		 * Get from the frame buffer the PIO Setup Data, although we don't need
		 * any info from this pio setup fis. */
		scic_sds_unsolicited_frame_control_get_buffer(
			&(this_request->parent.owning_controller->uf_control),
			frame_index,
			(void **)&frame_buffer
			);

		/*
		 * Get the data from the PIO Setup
		 * The SCU Hardware returns first word in the frame_header and the rest
		 * of the data is in the frame buffer so we need to back up one dword */
		this_request->type.packet.device_preferred_cdb_length =
			(u16)((struct sata_fis_pio_setup *)(&frame_buffer[-1]))->transfter_count;

		/* Frame has been decoded return it to the controller */
		scic_sds_controller_release_frame(
			this_request->parent.owning_controller, frame_index
			);

		sci_base_state_machine_change_state(
			&this_request->parent.started_substate_machine,
			SCIC_SDS_STP_PACKET_REQUEST_STARTED_COMMAND_PHASE_AWAIT_TC_COMPLETION_SUBSTATE
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


/**
 * This method processes the completions transport layer (TL) status to
 *    determine if the PACKET command data FIS was sent successfully. If
 *    successfully, then the state for the packet request transits to COMPLETE
 *    state. If not successfuly, the request transits to
 *    COMMAND_PHASE_AWAIT_D2H_FIS_SUBSTATE.
 * @this_request: This parameter specifies the request for which the TC
 *    completion was received.
 * @completion_code: This parameter indicates the completion status information
 *    for the TC.
 *
 * Indicate if the tc completion handler was successful. SCI_SUCCESS currently
 * this method always returns success.
 */
enum sci_status scic_sds_stp_packet_request_command_phase_await_tc_completion_tc_completion_handler(
	struct scic_sds_request *this_request,
	u32 completion_code)
{
	enum sci_status status = SCI_SUCCESS;
	u8 sat_packet_protocol =
		scic_cb_request_get_sat_protocol(this_request->user_request);

	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_request),
			       SCIC_LOG_OBJECT_STP_IO_REQUEST,
			       "scic_sds_stp_packet_request_command_phase_await_tc_completion_tc_completion_handler(0x%x, 0x%x) enter\n",
			       this_request, completion_code
			       ));

	switch (SCU_GET_COMPLETION_TL_STATUS(completion_code)) {
	case (SCU_TASK_DONE_GOOD << SCU_COMPLETION_TL_STATUS_SHIFT):
		scic_sds_request_set_status(
			this_request, SCU_TASK_DONE_GOOD, SCI_SUCCESS
			);

		if (sat_packet_protocol == SAT_PROTOCOL_PACKET_DMA_DATA_IN
		     || sat_packet_protocol == SAT_PROTOCOL_PACKET_DMA_DATA_OUT
		     )
			sci_base_state_machine_change_state(
				&this_request->parent.state_machine,
				SCI_BASE_REQUEST_STATE_COMPLETED
				);
		else
			sci_base_state_machine_change_state(
				&this_request->started_substate_machine,
				SCIC_SDS_STP_PACKET_REQUEST_STARTED_COMMAND_PHASE_AWAIT_D2H_FIS_SUBSTATE
				);
		break;

	case (SCU_TASK_DONE_UNEXP_FIS << SCU_COMPLETION_TL_STATUS_SHIFT):
		if (scic_io_request_get_number_of_bytes_transferred(this_request) <
		    scic_cb_io_request_get_transfer_length(this_request->user_request)) {
			scic_sds_request_set_status(
				this_request, SCU_TASK_DONE_GOOD, SCI_SUCCESS_IO_DONE_EARLY
				);

			sci_base_state_machine_change_state(
				&this_request->parent.state_machine,
				SCI_BASE_REQUEST_STATE_COMPLETED
				);

			status = this_request->sci_status;
		}
		break;

	case (SCU_TASK_DONE_EXCESS_DATA << SCU_COMPLETION_TL_STATUS_SHIFT):
		/* In this case, there is no UF coming after. compelte the IO now. */
		scic_sds_request_set_status(
			this_request, SCU_TASK_DONE_GOOD, SCI_SUCCESS
			);

		sci_base_state_machine_change_state(
			&this_request->parent.state_machine,
			SCI_BASE_REQUEST_STATE_COMPLETED
			);

		break;

	default:
		if (this_request->sci_status != SCI_SUCCESS) {  /* The io status was set already. This means an UF for the status
								 * fis was received already.
								 */

			/*
			 * A device suspension event is expected, we need to have the device
			 * coming out of suspension, then complete the IO. */
			sci_base_state_machine_change_state(
				&this_request->started_substate_machine,
				SCIC_SDS_STP_PACKET_REQUEST_STARTED_COMPLETION_DELAY_SUBSTATE
				);

			/* change the device state to ATAPI_ERROR. */
			sci_base_state_machine_change_state(
				&this_request->target_device->ready_substate_machine,
				SCIC_SDS_STP_REMOTE_DEVICE_READY_SUBSTATE_ATAPI_ERROR
				);

			status = this_request->sci_status;
		} else {  /* If receiving any non-sucess TC status, no UF received yet, then an UF for
			   * the status fis is coming after.
			   */
			scic_sds_request_set_status(
				this_request,
				SCU_TASK_DONE_CHECK_RESPONSE,
				SCI_FAILURE_IO_RESPONSE_VALID
				);

			sci_base_state_machine_change_state(
				&this_request->started_substate_machine,
				SCIC_SDS_STP_PACKET_REQUEST_STARTED_COMMAND_PHASE_AWAIT_D2H_FIS_SUBSTATE
				);
		}
		break;
	}

	return status;
}


/**
 * This method processes an unsolicited frame.
 * @this_request: This parameter specifies the request for which the
 *    unsolicited frame was received.
 * @frame_index: This parameter indicates the unsolicited frame index that
 *    should contain the response.
 *
 * This method returns an indication of whether the UF frame was handled
 * successfully or not. SCI_SUCCESS Currently this value is always returned and
 * indicates successful processing of the TC response.
 */
enum sci_status scic_sds_stp_packet_request_command_phase_common_frame_handler(
	struct scic_sds_request *request,
	u32 frame_index)
{
	enum sci_status status;
	struct sata_fis_header *frame_header;
	u32 *frame_buffer;
	struct scic_sds_stp_request *this_request;

	this_request = (struct scic_sds_stp_request *)request;

	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_request),
			       SCIC_LOG_OBJECT_STP_IO_REQUEST,
			       "scic_sds_stp_packet_request_command_phase_await_d2h_frame_handler(0x%x, 0x%x) enter\n",
			       this_request, frame_index
			       ));

	status = scic_sds_unsolicited_frame_control_get_header(
		&(this_request->parent.owning_controller->uf_control),
		frame_index,
		(void **)&frame_header
		);

	if (status == SCI_SUCCESS) {
		ASSERT(frame_header->fis_type == SATA_FIS_TYPE_REGD2H);

		/*
		 * Get from the frame buffer the PIO Setup Data, although we don't need
		 * any info from this pio setup fis. */
		scic_sds_unsolicited_frame_control_get_buffer(
			&(this_request->parent.owning_controller->uf_control),
			frame_index,
			(void **)&frame_buffer
			);

		scic_sds_controller_copy_sata_response(
			&this_request->d2h_reg_fis, (u32 *)frame_header, frame_buffer
			);

		/* Frame has been decoded return it to the controller */
		scic_sds_controller_release_frame(
			this_request->parent.owning_controller, frame_index
			);
	}

	return status;
}

/**
 * This method processes an unsolicited frame while the packet request is
 *    expecting TC completion. It will process the FIS and construct sense data.
 * @this_request: This parameter specifies the request for which the
 *    unsolicited frame was received.
 * @frame_index: This parameter indicates the unsolicited frame index that
 *    should contain the response.
 *
 * This method returns an indication of whether the UF frame was handled
 * successfully or not. SCI_SUCCESS Currently this value is always returned and
 * indicates successful processing of the TC response.
 */
enum sci_status scic_sds_stp_packet_request_command_phase_await_tc_completion_frame_handler(
	struct scic_sds_request *request,
	u32 frame_index)
{
	struct scic_sds_stp_request *this_request = (struct scic_sds_stp_request *)request;

	enum sci_status status =
		scic_sds_stp_packet_request_command_phase_common_frame_handler(
			request, frame_index);

	if (status == SCI_SUCCESS) {
		/* The command has completed with error status from target device. */
		status = scic_sds_stp_packet_request_process_status_fis(
			request, &this_request->d2h_reg_fis);

		if (status != SCI_SUCCESS) {
			scic_sds_request_set_status(
				&this_request->parent,
				SCU_TASK_DONE_CHECK_RESPONSE,
				status
				);
		} else
			scic_sds_request_set_status(
				&this_request->parent, SCU_TASK_DONE_GOOD, SCI_SUCCESS
				);
	}

	return status;
}


/**
 * This method processes an unsolicited frame while the packet request is
 *    expecting TC completion. It will process the FIS and construct sense data.
 * @this_request: This parameter specifies the request for which the
 *    unsolicited frame was received.
 * @frame_index: This parameter indicates the unsolicited frame index that
 *    should contain the response.
 *
 * This method returns an indication of whether the UF frame was handled
 * successfully or not. SCI_SUCCESS Currently this value is always returned and
 * indicates successful processing of the TC response.
 */
enum sci_status scic_sds_stp_packet_request_command_phase_await_d2h_fis_frame_handler(
	struct scic_sds_request *request,
	u32 frame_index)
{
	enum sci_status status =
		scic_sds_stp_packet_request_command_phase_common_frame_handler(
			request, frame_index);

	struct scic_sds_stp_request *this_request = (struct scic_sds_stp_request *)request;

	if (status == SCI_SUCCESS) {
		/* The command has completed with error status from target device. */
		status = scic_sds_stp_packet_request_process_status_fis(
			request, &this_request->d2h_reg_fis);

		if (status != SCI_SUCCESS) {
			scic_sds_request_set_status(
				request,
				SCU_TASK_DONE_CHECK_RESPONSE,
				status
				);
		} else
			scic_sds_request_set_status(
				request, SCU_TASK_DONE_GOOD, SCI_SUCCESS
				);

		/*
		 * Always complete the NON_DATA command right away, no need to delay completion
		 * even an error status fis came from target device. */
		sci_base_state_machine_change_state(
			&request->parent.state_machine,
			SCI_BASE_REQUEST_STATE_COMPLETED
			);
	}

	return status;
}

enum sci_status scic_sds_stp_packet_request_started_completion_delay_complete_handler(
	struct sci_base_request *request)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)request;

	sci_base_state_machine_change_state(
		&this_request->parent.state_machine,
		SCI_BASE_REQUEST_STATE_COMPLETED
		);

	return this_request->sci_status;
}

/* --------------------------------------------------------------------------- */

const struct scic_sds_io_request_state_handler scic_sds_stp_packet_request_started_substate_handler_table[] = {
	[SCIC_SDS_STP_PACKET_REQUEST_STARTED_PACKET_PHASE_AWAIT_TC_COMPLETION_SUBSTATE] = {
		.parent.start_handler    = scic_sds_request_default_start_handler,
		.parent.abort_handler    = scic_sds_request_started_state_abort_handler,
		.parent.complete_handler = scic_sds_request_default_complete_handler,
		.parent.destruct_handler = scic_sds_request_default_destruct_handler
		.tc_completion_handler   = scic_sds_stp_packet_request_packet_phase_await_tc_completion_tc_completion_handler,
		.event_handler           = scic_sds_request_default_event_handler,
		.frame_handler           = scic_sds_request_default_frame_handler
	},
	[SCIC_SDS_STP_PACKET_REQUEST_STARTED_PACKET_PHASE_AWAIT_PIO_SETUP_SUBSTATE] = {
		.parent.start_handler    = scic_sds_request_default_start_handler,
		.parent.abort_handler    = scic_sds_request_started_state_abort_handler,
		.parent.complete_handler = scic_sds_request_default_complete_handler,
		.parent.destruct_handler = scic_sds_request_default_destruct_handler
		.tc_completion_handler   = scic_sds_request_default_tc_completion_handler,
		.event_handler           = scic_sds_request_default_event_handler,
		.frame_handler           = scic_sds_stp_packet_request_packet_phase_await_pio_setup_frame_handler
	},
	[SCIC_SDS_STP_PACKET_REQUEST_STARTED_COMMAND_PHASE_AWAIT_TC_COMPLETION_SUBSTATE] = {
		.parent.start_handler    = scic_sds_request_default_start_handler,
		.parent.abort_handler    = scic_sds_request_started_state_abort_handler,
		.parent.complete_handler = scic_sds_request_default_complete_handler,
		.parent.destruct_handler = scic_sds_request_default_destruct_handler
		.tc_completion_handler   = scic_sds_stp_packet_request_command_phase_await_tc_completion_tc_completion_handler,
		.event_handler           = scic_sds_request_default_event_handler,
		.frame_handler           = scic_sds_stp_packet_request_command_phase_await_tc_completion_frame_handler
	},
	[SCIC_SDS_STP_PACKET_REQUEST_STARTED_COMMAND_PHASE_AWAIT_D2H_FIS_SUBSTATE] = {
		.parent.start_handler    = scic_sds_request_default_start_handler,
		.parent.abort_handler    = scic_sds_request_started_state_abort_handler,
		.parent.complete_handler = scic_sds_request_default_complete_handler,
		.parent.destruct_handler = scic_sds_request_default_destruct_handler
		.tc_completion_handler   = scic_sds_request_default_tc_completion_handler,
		.event_handler           = scic_sds_request_default_event_handler,
		.frame_handler           = scic_sds_stp_packet_request_command_phase_await_d2h_fis_frame_handler
	},
	[SCIC_SDS_STP_PACKET_REQUEST_STARTED_COMPLETION_DELAY_SUBSTATE] = {
		.parent.start_handler    = scic_sds_request_default_start_handler,
		.parent.abort_handler    = scic_sds_request_started_state_abort_handler,
		.parent.complete_handler = scic_sds_stp_packet_request_started_completion_delay_complete_handler,
		.parent.destruct_handler = scic_sds_request_default_destruct_handler
		.tc_completion_handler   = scic_sds_request_default_tc_completion_handler,
		.event_handler           = scic_sds_request_default_event_handler,
		.frame_handler           = scic_sds_request_default_frame_handler
	},
};

#endif /* !defined(DISABLE_ATAPI) */
