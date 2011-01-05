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
 * This file contains the SMP request started substate handlers.
 *
 *
 */

#include "intel_sas.h"
#include "sci_util.h"
#include "scic_sds_request.h"
#include "scic_controller.h"
#include "scic_sds_logger.h"
#include "scic_sds_controller.h"
#include "scu_completion_codes.h"
#include "scu_task_context.h"
#include "scic_sds_smp_request.h"


/**
 * This method processes an unsolicited frame while the SMP request is waiting
 *    for a response frame.  It will copy the response data, release the
 *    unsolicited frame, and transition the request to the
 *    SCI_BASE_REQUEST_STATE_COMPLETED state.
 * @this_request: This parameter specifies the request for which the
 *    unsolicited frame was received.
 * @frame_index: This parameter indicates the unsolicited frame index that
 *    should contain the response.
 *
 * This method returns an indication of whether the response frame was handled
 * successfully or not. SCI_SUCCESS Currently this value is always returned and
 * indicates successful processing of the TC response.
 */
static enum sci_status scic_sds_smp_request_await_response_frame_handler(
	struct scic_sds_request *this_request,
	u32 frame_index)
{
	enum sci_status status;
	void *frame_header;
	struct smp_response_header *this_frame_header;
	u8 *user_smp_buffer = this_request->response_buffer;

	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_request),
			       SCIC_LOG_OBJECT_SMP_IO_REQUEST,
			       "scic_sds_smp_request_await_response_frame_handler(0x%x, 0x%x) enter\n",
			       this_request, frame_index
			       ));

	status = scic_sds_unsolicited_frame_control_get_header(
		&(scic_sds_request_get_controller(this_request)->uf_control),
		frame_index,
		&frame_header
		);

	/* byte swap the header. */
	scic_word_copy_with_swap(
		(u32 *)user_smp_buffer,
		frame_header,
		sizeof(struct smp_response_header) / sizeof(u32)
		);
	this_frame_header = (struct smp_response_header *)user_smp_buffer;

	if (this_frame_header->smp_frame_type == SMP_FRAME_TYPE_RESPONSE) {
		void *smp_response_buffer;

		status = scic_sds_unsolicited_frame_control_get_buffer(
			&(scic_sds_request_get_controller(this_request)->uf_control),
			frame_index,
			&smp_response_buffer
			);

		scic_word_copy_with_swap(
			(u32 *)(user_smp_buffer + sizeof(struct smp_response_header)),
			smp_response_buffer,
			sizeof(union smp_response_body) / sizeof(u32)
			);
		if (this_frame_header->function == SMP_FUNCTION_DISCOVER) {
			struct smp_response *this_smp_response;

			this_smp_response = (struct smp_response *)user_smp_buffer;

			/*
			 * Some expanders only report an attached SATA device, and
			 * not an STP target.  Since the core depends on the STP
			 * target attribute to correctly build I/O, set the bit now
			 * if necessary. */
			if (this_smp_response->response.discover.protocols.u.bits.attached_sata_device
			    && !this_smp_response->response.discover.protocols.u.bits.attached_stp_target) {
				this_smp_response->response.discover.protocols.u.bits.attached_stp_target = 1;

				SCIC_LOG_TRACE((
						       sci_base_object_get_logger(this_request),
						       SCIC_LOG_OBJECT_SMP_IO_REQUEST,
						       "scic_sds_smp_request_await_response_frame_handler(0x%x) Found SATA dev, setting STP bit.\n",
						       this_request
						       ));
			}
		}

		/*
		 * Don't need to copy to user space. User instead will refer to
		 * core request's response buffer. */

		/*
		 * copy the smp response to framework smp request's response buffer.
		 * scic_sds_smp_request_copy_response(this_request); */

		scic_sds_request_set_status(
			this_request, SCU_TASK_DONE_GOOD, SCI_SUCCESS
			);

		sci_base_state_machine_change_state(
			&this_request->started_substate_machine,
			SCIC_SDS_SMP_REQUEST_STARTED_SUBSTATE_AWAIT_TC_COMPLETION
			);
	} else {
		/* This was not a response frame why did it get forwarded? */
		SCIC_LOG_ERROR((
				       sci_base_object_get_logger(this_request),
				       SCIC_LOG_OBJECT_SMP_IO_REQUEST,
				       "SCIC SMP Request 0x%08x received unexpected frame %d type 0x%02x\n",
				       this_request, frame_index, this_frame_header->smp_frame_type
				       ));

		scic_sds_request_set_status(
			this_request,
			SCU_TASK_DONE_SMP_FRM_TYPE_ERR,
			SCI_FAILURE_CONTROLLER_SPECIFIC_IO_ERR
			);

		sci_base_state_machine_change_state(
			&this_request->parent.state_machine,
			SCI_BASE_REQUEST_STATE_COMPLETED
			);
	}

	scic_sds_controller_release_frame(
		this_request->owning_controller, frame_index
		);

	return SCI_SUCCESS;
}


/**
 * This method processes an abnormal TC completion while the SMP request is
 *    waiting for a response frame.  It decides what happened to the IO based
 *    on TC completion status.
 * @this_request: This parameter specifies the request for which the TC
 *    completion was received.
 * @completion_code: This parameter indicates the completion status information
 *    for the TC.
 *
 * Indicate if the tc completion handler was successful. SCI_SUCCESS currently
 * this method always returns success.
 */
static enum sci_status scic_sds_smp_request_await_response_tc_completion_handler(
	struct scic_sds_request *this_request,
	u32 completion_code)
{
	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_request),
			       SCIC_LOG_OBJECT_SMP_IO_REQUEST,
			       "scic_sds_smp_request_await_response_tc_completion_handler(0x%x, 0x%x) enter\n",
			       this_request, completion_code
			       ));

	switch (SCU_GET_COMPLETION_TL_STATUS(completion_code)) {
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_GOOD):
		/*
		 * In the AWAIT RESPONSE state, any TC completion is unexpected.
		 * but if the TC has success status, we complete the IO anyway. */
		scic_sds_request_set_status(
			this_request, SCU_TASK_DONE_GOOD, SCI_SUCCESS
			);

		sci_base_state_machine_change_state(
			&this_request->parent.state_machine,
			SCI_BASE_REQUEST_STATE_COMPLETED
			);
		break;

	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_SMP_RESP_TO_ERR):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_SMP_UFI_ERR):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_SMP_FRM_TYPE_ERR):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_SMP_LL_RX_ERR):
		/*
		 * These status has been seen in a specific LSI expander, which sometimes
		 * is not able to send smp response within 2 ms. This causes our hardware
		 * break the connection and set TC completion with one of these SMP_XXX_XX_ERR
		 * status. For these type of error, we ask scic user to retry the request. */
		scic_sds_request_set_status(
			this_request, SCU_TASK_DONE_SMP_RESP_TO_ERR, SCI_FAILURE_RETRY_REQUIRED
			);

		sci_base_state_machine_change_state(
			&this_request->parent.state_machine,
			SCI_BASE_REQUEST_STATE_COMPLETED
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

	return SCI_SUCCESS;
}


/**
 * This method processes the completions transport layer (TL) status to
 *    determine if the SMP request was sent successfully. If the SMP request
 *    was sent successfully, then the state for the SMP request transits to
 *    waiting for a response frame.
 * @this_request: This parameter specifies the request for which the TC
 *    completion was received.
 * @completion_code: This parameter indicates the completion status information
 *    for the TC.
 *
 * Indicate if the tc completion handler was successful. SCI_SUCCESS currently
 * this method always returns success.
 */
static enum sci_status scic_sds_smp_request_await_tc_completion_tc_completion_handler(
	struct scic_sds_request *this_request,
	u32 completion_code)
{
	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_request),
			       SCIC_LOG_OBJECT_SMP_IO_REQUEST,
			       "scic_sds_smp_request_await_tc_completion_tc_completion_handler(0x%x, 0x%x) enter\n",
			       this_request, completion_code
			       ));

	switch (SCU_GET_COMPLETION_TL_STATUS(completion_code)) {
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_GOOD):
		scic_sds_request_set_status(
			this_request, SCU_TASK_DONE_GOOD, SCI_SUCCESS
			);

		sci_base_state_machine_change_state(
			&this_request->parent.state_machine,
			SCI_BASE_REQUEST_STATE_COMPLETED
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

	return SCI_SUCCESS;
}


const struct scic_sds_io_request_state_handler scic_sds_smp_request_started_substate_handler_table[] = {
	[SCIC_SDS_SMP_REQUEST_STARTED_SUBSTATE_AWAIT_RESPONSE] = {
		.parent.start_handler    = scic_sds_request_default_start_handler,
		.parent.abort_handler    = scic_sds_request_started_state_abort_handler,
		.parent.complete_handler = scic_sds_request_default_complete_handler,
		.parent.destruct_handler = scic_sds_request_default_destruct_handler,
		.tc_completion_handler   = scic_sds_smp_request_await_response_tc_completion_handler,
		.event_handler           = scic_sds_request_default_event_handler,
		.frame_handler           = scic_sds_smp_request_await_response_frame_handler,
	},
	[SCIC_SDS_SMP_REQUEST_STARTED_SUBSTATE_AWAIT_TC_COMPLETION] = {
		.parent.start_handler    = scic_sds_request_default_start_handler,
		.parent.abort_handler    = scic_sds_request_started_state_abort_handler,
		.parent.complete_handler = scic_sds_request_default_complete_handler,
		.parent.destruct_handler = scic_sds_request_default_destruct_handler,
		.tc_completion_handler   =  scic_sds_smp_request_await_tc_completion_tc_completion_handler,
		.event_handler           =  scic_sds_request_default_event_handler,
		.frame_handler           =  scic_sds_request_default_frame_handler,
	}
};

