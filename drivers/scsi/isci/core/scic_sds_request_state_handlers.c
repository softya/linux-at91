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
 * This file contains the implementation for the SCIC_SDS_IO_REQUEST state
 *    handlers.  The set of state handlers is put in place on entry to any base
 *    state.
 *
 *
 */

#include "intel_sas.h"
#include "sci_util.h"
#include "scic_controller.h"
#include "scic_sds_logger.h"
#include "scic_sds_controller.h"
#include "scic_sds_request.h"
#include "scu_completion_codes.h"
#include "scu_task_context.h"
#include "scic_sds_unsolicited_frame_control.h"

/*
 * *****************************************************************************
 * *  DEFAULT STATE HANDLERS
 * ***************************************************************************** */

/**
 * scic_sds_request_default_start_handler() -
 * @request: This is the struct sci_base_request object that is cast to the
 *    SCIC_SDS_IO_REQUEST_T object for which the start operation is requested.
 *
 * This method is the default action to take when an SCIC_SDS_IO_REQUEST_T
 * object receives a scic_sds_request_start() request.  The default action is
 * to log a warning and return a failure status. enum sci_status
 * SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_request_default_start_handler(
	struct sci_base_request *request)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger((struct scic_sds_request *)request),
				 (
					 SCIC_LOG_OBJECT_SSP_IO_REQUEST
					 | SCIC_LOG_OBJECT_STP_IO_REQUEST
					 | SCIC_LOG_OBJECT_SMP_IO_REQUEST
				 ),
				 "SCIC IO Request requested to start while in wrong state %d\n",
				 sci_base_state_machine_get_state(
					 &((struct scic_sds_request *)request)->parent.state_machine)
				 ));

	return SCI_FAILURE_INVALID_STATE;
}

static enum sci_status scic_sds_request_default_abort_handler(
	struct sci_base_request *request)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger((struct scic_sds_request *)request),
				 (
					 SCIC_LOG_OBJECT_SSP_IO_REQUEST
					 | SCIC_LOG_OBJECT_STP_IO_REQUEST
					 | SCIC_LOG_OBJECT_SMP_IO_REQUEST
				 ),
				 "SCIC IO Request requested to abort while in wrong state %d\n",
				 sci_base_state_machine_get_state(
					 &((struct scic_sds_request *)request)->parent.state_machine)
				 ));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 * scic_sds_request_default_complete_handler() -
 * @request: This is the struct sci_base_request object that is cast to the
 *    SCIC_SDS_IO_REQUEST_T object for which the start operation is requested.
 *
 * This method is the default action to take when an SCIC_SDS_IO_REQUEST_T
 * object receives a scic_sds_request_complete() request.  The default action
 * is to log a warning and return a failure status. enum sci_status
 * SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_request_default_complete_handler(
	struct sci_base_request *request)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger((struct scic_sds_request *)request),
				 (
					 SCIC_LOG_OBJECT_SSP_IO_REQUEST
					 | SCIC_LOG_OBJECT_STP_IO_REQUEST
					 | SCIC_LOG_OBJECT_SMP_IO_REQUEST
				 ),
				 "SCIC IO Request requested to complete while in wrong state %d\n",
				 sci_base_state_machine_get_state(
					 &((struct scic_sds_request *)request)->parent.state_machine)
				 ));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 * scic_sds_request_default_destruct_handler() -
 * @request: This is the struct sci_base_request object that is cast to the
 *    SCIC_SDS_IO_REQUEST_T object for which the start operation is requested.
 *
 * This method is the default action to take when an SCIC_SDS_IO_REQUEST_T
 * object receives a scic_sds_request_complete() request.  The default action
 * is to log a warning and return a failure status. enum sci_status
 * SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_request_default_destruct_handler(
	struct sci_base_request *request)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger((struct scic_sds_request *)request),
				 (
					 SCIC_LOG_OBJECT_SSP_IO_REQUEST
					 | SCIC_LOG_OBJECT_STP_IO_REQUEST
					 | SCIC_LOG_OBJECT_SMP_IO_REQUEST
				 ),
				 "SCIC IO Request requested to destroy while in wrong state %d\n",
				 sci_base_state_machine_get_state(
					 &((struct scic_sds_request *)request)->parent.state_machine)
				 ));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 * scic_sds_request_default_tc_completion_handler() -
 * @request: This is the struct sci_base_request object that is cast to the
 *    SCIC_SDS_IO_REQUEST_T object for which the start operation is requested.
 *
 * This method is the default action to take when an SCIC_SDS_IO_REQUEST_T
 * object receives a scic_sds_task_request_complete() request.  The default
 * action is to log a warning and return a failure status. enum sci_status
 * SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_request_default_tc_completion_handler(
	struct scic_sds_request *this_request,
	u32 completion_code)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger(this_request),
				 (
					 SCIC_LOG_OBJECT_SSP_IO_REQUEST
					 | SCIC_LOG_OBJECT_STP_IO_REQUEST
					 | SCIC_LOG_OBJECT_SMP_IO_REQUEST
				 ),
				 "SCIC IO Request given task completion notification %x while in wrong state %d\n",
				 completion_code,
				 sci_base_state_machine_get_state(&this_request->parent.state_machine)
				 ));

	return SCI_FAILURE_INVALID_STATE;

}

/**
 * scic_sds_request_default_event_handler() -
 * @request: This is the struct sci_base_request object that is cast to the
 *    SCIC_SDS_IO_REQUEST_T object for which the start operation is requested.
 *
 * This method is the default action to take when an SCIC_SDS_IO_REQUEST_T
 * object receives a scic_sds_request_event_handler() request.  The default
 * action is to log a warning and return a failure status. enum sci_status
 * SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_request_default_event_handler(
	struct scic_sds_request *this_request,
	u32 event_code)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger(this_request),
				 (
					 SCIC_LOG_OBJECT_SSP_IO_REQUEST
					 | SCIC_LOG_OBJECT_STP_IO_REQUEST
					 | SCIC_LOG_OBJECT_SMP_IO_REQUEST
				 ),
				 "SCIC IO Request given event code notification %x while in wrong state %d\n",
				 event_code,
				 sci_base_state_machine_get_state(&this_request->parent.state_machine)
				 ));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 * scic_sds_request_default_frame_handler() -
 * @request: This is the struct sci_base_request object that is cast to the
 *    SCIC_SDS_IO_REQUEST_T object for which the start operation is requested.
 *
 * This method is the default action to take when an SCIC_SDS_IO_REQUEST_T
 * object receives a scic_sds_request_event_handler() request.  The default
 * action is to log a warning and return a failure status. enum sci_status
 * SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_request_default_frame_handler(
	struct scic_sds_request *this_request,
	u32 frame_index)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger(this_request),
				 (
					 SCIC_LOG_OBJECT_SSP_IO_REQUEST
					 | SCIC_LOG_OBJECT_STP_IO_REQUEST
					 | SCIC_LOG_OBJECT_SMP_IO_REQUEST
				 ),
				 "SCIC IO Request given unexpected frame %x while in state %d\n",
				 frame_index,
				 sci_base_state_machine_get_state(&this_request->parent.state_machine)
				 ));

	scic_sds_controller_release_frame(
		this_request->owning_controller, frame_index);

	return SCI_FAILURE_INVALID_STATE;
}

/*
 * *****************************************************************************
 * *  CONSTRUCTED STATE HANDLERS
 * ***************************************************************************** */

/**
 * scic_sds_request_constructed_state_start_handler() -
 * @request: This is the struct sci_base_request object that is cast to the
 *    SCIC_SDS_IO_REQUEST_T object for which the start operation is requested.
 *
 * This method implements the action taken when a constructed
 * SCIC_SDS_IO_REQUEST_T object receives a scic_sds_request_start() request.
 * This method will, if necessary, allocate a TCi for the io request object and
 * then will, if necessary, copy the constructed TC data into the actual TC
 * buffer.  If everything is successful the post context field is updated with
 * the TCi so the controller can post the request to the hardware. enum sci_status
 * SCI_SUCCESS SCI_FAILURE_INSUFFICIENT_RESOURCES
 */
static enum sci_status scic_sds_request_constructed_state_start_handler(
	struct sci_base_request *request)
{
	struct scu_task_context *task_context;
	struct scic_sds_request *this_request = (struct scic_sds_request *)request;

	if (this_request->io_tag == SCI_CONTROLLER_INVALID_IO_TAG) {
		this_request->io_tag =
			scic_controller_allocate_io_tag(this_request->owning_controller);
	}

	/* Record the IO Tag in the request */
	if (this_request->io_tag != SCI_CONTROLLER_INVALID_IO_TAG) {
		task_context = this_request->task_context_buffer;

		task_context->task_index = scic_sds_io_tag_get_index(this_request->io_tag);

		switch (task_context->protocol_type) {
		case SCU_TASK_CONTEXT_PROTOCOL_SMP:
		case SCU_TASK_CONTEXT_PROTOCOL_SSP:
			/* SSP/SMP Frame */
			task_context->type.ssp.tag = this_request->io_tag;
			task_context->type.ssp.target_port_transfer_tag = 0xFFFF;
			break;

		case SCU_TASK_CONTEXT_PROTOCOL_STP:
			/*
			 * STP/SATA Frame
			 * task_context->type.stp.ncq_tag = this_request->ncq_tag; */
			break;

		case SCU_TASK_CONTEXT_PROTOCOL_NONE:
			/* / @todo When do we set no protocol type? */
			break;

		default:
			/* This should never happen since we build the IO requests */
			break;
		}

		/*
		 * Check to see if we need to copy the task context buffer
		 * or have been building into the task context buffer */
		if (this_request->was_tag_assigned_by_user == false) {
			scic_sds_controller_copy_task_context(
				this_request->owning_controller, this_request
				);
		}

		/* Add to the post_context the io tag value */
		this_request->post_context |= scic_sds_io_tag_get_index(this_request->io_tag);

		/* Everything is good go ahead and change state */
		sci_base_state_machine_change_state(
			&this_request->parent.state_machine,
			SCI_BASE_REQUEST_STATE_STARTED
			);

		return SCI_SUCCESS;
	}

	return SCI_FAILURE_INSUFFICIENT_RESOURCES;
}

/**
 * scic_sds_request_constructed_state_abort_handler() -
 * @request: This is the struct sci_base_request object that is cast to the
 *    SCIC_SDS_IO_REQUEST_T object for which the start operation is requested.
 *
 * This method implements the action to be taken when an SCIC_SDS_IO_REQUEST_T
 * object receives a scic_sds_request_terminate() request. Since the request
 * has not yet been posted to the hardware the request transitions to the
 * completed state. enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_request_constructed_state_abort_handler(
	struct sci_base_request *request)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)request;

	/*
	 * This request has been terminated by the user make sure that the correct
	 * status code is returned */
	scic_sds_request_set_status(
		this_request,
		SCU_TASK_DONE_TASK_ABORT,
		SCI_FAILURE_IO_TERMINATED
		);

	sci_base_state_machine_change_state(
		&this_request->parent.state_machine,
		SCI_BASE_REQUEST_STATE_COMPLETED
		);

	return SCI_SUCCESS;
}

/*
 * *****************************************************************************
 * *  STARTED STATE HANDLERS
 * ***************************************************************************** */

/**
 * scic_sds_request_started_state_abort_handler() -
 * @request: This is the struct sci_base_request object that is cast to the
 *    SCIC_SDS_IO_REQUEST_T object for which the start operation is requested.
 *
 * This method implements the action to be taken when an SCIC_SDS_IO_REQUEST_T
 * object receives a scic_sds_request_terminate() request. Since the request
 * has been posted to the hardware the io request state is changed to the
 * aborting state. enum sci_status SCI_SUCCESS
 */
enum sci_status scic_sds_request_started_state_abort_handler(
	struct sci_base_request *request)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)request;

	if (this_request->has_started_substate_machine) {
		sci_base_state_machine_stop(&this_request->started_substate_machine);
	}

	sci_base_state_machine_change_state(
		&this_request->parent.state_machine,
		SCI_BASE_REQUEST_STATE_ABORTING
		);

	return SCI_SUCCESS;
}

/**
 * scic_sds_request_started_state_tc_completion_handler() - This method process
 *    TC (task context) completions for normal IO request (i.e. Task/Abort
 *    Completions of type 0).  This method will update the
 *    SCIC_SDS_IO_REQUEST_T::status field.
 * @this_request: This parameter specifies the request for which a completion
 *    occurred.
 * @completion_code: This parameter specifies the completion code recieved from
 *    the SCU.
 *
 */
enum sci_status scic_sds_request_started_state_tc_completion_handler(
	struct scic_sds_request *this_request,
	u32 completion_code)
{
	u8 data_present;
	struct sci_ssp_response_iu *response_buffer;

	/**
	 * @todo Any SDMA return code of other than 0 is bad
	 *       decode 0x003C0000 to determine SDMA status
	 */
	switch (SCU_GET_COMPLETION_TL_STATUS(completion_code)) {
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_GOOD):
		scic_sds_request_set_status(
			this_request, SCU_TASK_DONE_GOOD, SCI_SUCCESS
			);
		break;

	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_EARLY_RESP):
	{
		/*
		 * There are times when the SCU hardware will return an early response
		 * because the io request specified more data than is returned by the
		 * target device (mode pages, inquiry data, etc.).  We must check the
		 * response stats to see if this is truly a failed request or a good
		 * request that just got completed early. */
		struct sci_ssp_response_iu *response = (struct sci_ssp_response_iu *)
						  this_request->response_buffer;
		scic_word_copy_with_swap(
			this_request->response_buffer,
			this_request->response_buffer,
			sizeof(struct sci_ssp_response_iu) / sizeof(u32)
			);

		if (response->status == 0) {
			scic_sds_request_set_status(
				this_request, SCU_TASK_DONE_GOOD, SCI_SUCCESS_IO_DONE_EARLY
				);
		} else {
			scic_sds_request_set_status(
				this_request,
				SCU_TASK_DONE_CHECK_RESPONSE,
				SCI_FAILURE_IO_RESPONSE_VALID
				);
		}
	}
	break;

	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_CHECK_RESPONSE):
		scic_word_copy_with_swap(
			this_request->response_buffer,
			this_request->response_buffer,
			sizeof(struct sci_ssp_response_iu) / sizeof(u32)
			);

		scic_sds_request_set_status(
			this_request,
			SCU_TASK_DONE_CHECK_RESPONSE,
			SCI_FAILURE_IO_RESPONSE_VALID
			);
		break;

	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_RESP_LEN_ERR):
		/*
		 * / @todo With TASK_DONE_RESP_LEN_ERR is the response frame guaranteed
		 * /       to be received before this completion status is posted? */
		response_buffer =
			(struct sci_ssp_response_iu *)this_request->response_buffer;
		data_present =
			response_buffer->data_present & SCI_SSP_RESPONSE_IU_DATA_PRESENT_MASK;

		if ((data_present == 0x01) || (data_present == 0x02)) {
			scic_sds_request_set_status(
				this_request,
				SCU_TASK_DONE_CHECK_RESPONSE,
				SCI_FAILURE_IO_RESPONSE_VALID
				);
		} else {
			scic_sds_request_set_status(
				this_request, SCU_TASK_DONE_GOOD, SCI_SUCCESS
				);
		}
		break;

	/* only stp device gets suspended. */
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_ACK_NAK_TO):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_LL_PERR):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_NAK_ERR):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_DATA_LEN_ERR):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_LL_ABORT_ERR):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_XR_WD_LEN):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_MAX_PLD_ERR):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_UNEXP_RESP):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_UNEXP_SDBFIS):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_REG_ERR):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_SDB_ERR):
		if (this_request->protocol == SCIC_STP_PROTOCOL) {
			scic_sds_request_set_status(
				this_request,
				SCU_GET_COMPLETION_TL_STATUS(completion_code) >> SCU_COMPLETION_TL_STATUS_SHIFT,
				SCI_FAILURE_REMOTE_DEVICE_RESET_REQUIRED
				);
		} else {
			scic_sds_request_set_status(
				this_request,
				SCU_GET_COMPLETION_TL_STATUS(completion_code) >> SCU_COMPLETION_TL_STATUS_SHIFT,
				SCI_FAILURE_CONTROLLER_SPECIFIC_IO_ERR
				);
		}
		break;

	/* both stp/ssp device gets suspended */
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_LF_ERR):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_OPEN_REJECT_WRONG_DESTINATION):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_OPEN_REJECT_RESERVED_ABANDON_1):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_OPEN_REJECT_RESERVED_ABANDON_2):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_OPEN_REJECT_RESERVED_ABANDON_3):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_OPEN_REJECT_BAD_DESTINATION):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_OPEN_REJECT_ZONE_VIOLATION):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_OPEN_REJECT_STP_RESOURCES_BUSY):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_OPEN_REJECT_PROTOCOL_NOT_SUPPORTED):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_OPEN_REJECT_CONNECTION_RATE_NOT_SUPPORTED):
		scic_sds_request_set_status(
			this_request,
			SCU_GET_COMPLETION_TL_STATUS(completion_code) >> SCU_COMPLETION_TL_STATUS_SHIFT,
			SCI_FAILURE_REMOTE_DEVICE_RESET_REQUIRED
			);
		break;

	/* neither ssp nor stp gets suspended. */
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_NAK_CMD_ERR):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_UNEXP_XR):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_XR_IU_LEN_ERR):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_SDMA_ERR):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_OFFSET_ERR):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_EXCESS_DATA):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_SMP_RESP_TO_ERR):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_SMP_UFI_ERR):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_SMP_FRM_TYPE_ERR):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_SMP_LL_RX_ERR):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_UNEXP_DATA):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_OPEN_FAIL):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_VIIT_ENTRY_NV):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_IIT_ENTRY_NV):
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_RNCNV_OUTBOUND):
	default:
		scic_sds_request_set_status(
			this_request,
			SCU_GET_COMPLETION_TL_STATUS(completion_code) >> SCU_COMPLETION_TL_STATUS_SHIFT,
			SCI_FAILURE_CONTROLLER_SPECIFIC_IO_ERR
			);
		break;
	}

	/**
	 * @todo This is probably wrong for ACK/NAK timeout conditions
	 */

	/* In all cases we will treat this as the completion of the IO request. */
	sci_base_state_machine_change_state(
		&this_request->parent.state_machine,
		SCI_BASE_REQUEST_STATE_COMPLETED
		);

	return SCI_SUCCESS;
}

/**
 * scic_sds_request_started_state_frame_handler() -
 * @request: This is the struct sci_base_request object that is cast to the
 *    SCIC_SDS_IO_REQUEST_T object for which the start operation is requested.
 * @frame_index: This is the index of the unsolicited frame to be processed.
 *
 * This method implements the action to be taken when an SCIC_SDS_IO_REQUEST_T
 * object receives a scic_sds_request_frame_handler() request. This method
 * first determines the frame type received.  If this is a response frame then
 * the response data is copied to the io request response buffer for processing
 * at completion time. If the frame type is not a response buffer an error is
 * logged. enum sci_status SCI_SUCCESS SCI_FAILURE_INVALID_PARAMETER_VALUE
 */
static enum sci_status scic_sds_request_started_state_frame_handler(
	struct scic_sds_request *this_request,
	u32 frame_index)
{
	enum sci_status status;
	struct sci_ssp_frame_header *frame_header;

	/* / @todo If this is a response frame we must record that we received it */
	status = scic_sds_unsolicited_frame_control_get_header(
		&(scic_sds_request_get_controller(this_request)->uf_control),
		frame_index,
		(void **)&frame_header
		);

	if (frame_header->frame_type == SCI_SAS_RESPONSE_FRAME) {
		struct sci_ssp_response_iu *response_buffer;

		status = scic_sds_unsolicited_frame_control_get_buffer(
			&(scic_sds_request_get_controller(this_request)->uf_control),
			frame_index,
			(void **)&response_buffer
			);

		scic_word_copy_with_swap(
			this_request->response_buffer,
			(u32 *)response_buffer,
			sizeof(struct sci_ssp_response_iu)
			);

		response_buffer = (struct sci_ssp_response_iu *)this_request->response_buffer;

		if (
			(response_buffer->data_present == 0x01)
			|| (response_buffer->data_present == 0x02)
			) {
			scic_sds_request_set_status(
				this_request,
				SCU_TASK_DONE_CHECK_RESPONSE,
				SCI_FAILURE_CONTROLLER_SPECIFIC_IO_ERR
				);
		} else {
			scic_sds_request_set_status(
				this_request, SCU_TASK_DONE_GOOD, SCI_SUCCESS
				);
		}

	} else {
		/* This was not a response frame why did it get forwarded? */
		SCIC_LOG_ERROR((
				       sci_base_object_get_logger(this_request),
				       SCIC_LOG_OBJECT_SSP_IO_REQUEST,
				       "SCIC IO Request 0x%x received unexpected frame %d type 0x%02x\n",
				       this_request, frame_index, frame_header->frame_type
				       ));
	}

	/*
	 * In any case we are done with this frame buffer return it to the
	 * controller */
	scic_sds_controller_release_frame(
		this_request->owning_controller, frame_index
		);

	return SCI_SUCCESS;
}

/*
 * *****************************************************************************
 * *  COMPLETED STATE HANDLERS
 * ***************************************************************************** */


/**
 * scic_sds_request_completed_state_complete_handler() -
 * @request: This is the struct sci_base_request object that is cast to the
 *    SCIC_SDS_IO_REQUEST_T object for which the start operation is requested.
 *
 * This method implements the action to be taken when an SCIC_SDS_IO_REQUEST_T
 * object receives a scic_sds_request_complete() request. This method frees up
 * any io request resources that have been allocated and transitions the
 * request to its final state. Consider stopping the state machine instead of
 * transitioning to the final state? enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_request_completed_state_complete_handler(
	struct sci_base_request *request)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)request;

	if (this_request->was_tag_assigned_by_user != true) {
		scic_controller_free_io_tag(
			this_request->owning_controller, this_request->io_tag
			);
	}

	if (this_request->saved_rx_frame_index != SCU_INVALID_FRAME_INDEX) {
		scic_sds_controller_release_frame(
			this_request->owning_controller, this_request->saved_rx_frame_index);
	}

	sci_base_state_machine_change_state(
		&this_request->parent.state_machine,
		SCI_BASE_REQUEST_STATE_FINAL
		);

	scic_sds_request_deinitialize_state_logging(this_request);

	return SCI_SUCCESS;
}

/*
 * *****************************************************************************
 * *  ABORTING STATE HANDLERS
 * ***************************************************************************** */

/**
 * scic_sds_request_aborting_state_abort_handler() -
 * @request: This is the struct sci_base_request object that is cast to the
 *    SCIC_SDS_IO_REQUEST_T object for which the start operation is requested.
 *
 * This method implements the action to be taken when an SCIC_SDS_IO_REQUEST_T
 * object receives a scic_sds_request_terminate() request. This method is the
 * io request aborting state abort handlers.  On receipt of a multiple
 * terminate requests the io request will transition to the completed state.
 * This should not happen in normal operation. enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_request_aborting_state_abort_handler(
	struct sci_base_request *request)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)request;

	sci_base_state_machine_change_state(
		&this_request->parent.state_machine,
		SCI_BASE_REQUEST_STATE_COMPLETED
		);

	return SCI_SUCCESS;
}

/**
 * scic_sds_request_aborting_state_tc_completion_handler() -
 * @request: This is the struct sci_base_request object that is cast to the
 *    SCIC_SDS_IO_REQUEST_T object for which the start operation is requested.
 *
 * This method implements the action to be taken when an SCIC_SDS_IO_REQUEST_T
 * object receives a scic_sds_request_task_completion() request. This method
 * decodes the completion type waiting for the abort task complete
 * notification. When the abort task complete is received the io request
 * transitions to the completed state. enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_request_aborting_state_tc_completion_handler(
	struct scic_sds_request *this_request,
	u32 completion_code)
{
	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_request),
			       SCIC_LOG_OBJECT_TASK_MANAGEMENT,
			       "scic_sds_request_aborting_state_tc_completion_handler(0x%x,0x%x) enter\n",
			       this_request, completion_code
			       ));

	switch (SCU_GET_COMPLETION_TL_STATUS(completion_code)) {
	case (SCU_TASK_DONE_GOOD << SCU_COMPLETION_TL_STATUS_SHIFT):
	case (SCU_TASK_DONE_TASK_ABORT << SCU_COMPLETION_TL_STATUS_SHIFT):
		scic_sds_request_set_status(
			this_request, SCU_TASK_DONE_TASK_ABORT, SCI_FAILURE_IO_TERMINATED
			);

		sci_base_state_machine_change_state(
			&this_request->parent.state_machine,
			SCI_BASE_REQUEST_STATE_COMPLETED
			);
		break;

	default:
		/*
		 * Unless we get some strange error wait for the task abort to complete
		 * TODO: Should there be a state change for this completion? */
		break;
	}

	return SCI_SUCCESS;
}

/**
 * scic_sds_request_aborting_state_frame_handler() -
 * @request: This is the struct sci_base_request object that is cast to the
 *    SCIC_SDS_IO_REQUEST_T object for which the start operation is requested.
 *
 * This method implements the action to be taken when an SCIC_SDS_IO_REQUEST_T
 * object receives a scic_sds_request_frame_handler() request. This method
 * discards the unsolicited frame since we are waiting for the abort task
 * completion. enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_request_aborting_state_frame_handler(
	struct scic_sds_request *this_request,
	u32 frame_index)
{
	/* TODO: Is it even possible to get an unsolicited frame in the aborting state? */

	scic_sds_controller_release_frame(
		this_request->owning_controller, frame_index);

	return SCI_SUCCESS;
}

/* --------------------------------------------------------------------------- */

struct scic_sds_io_request_state_handler
	scic_sds_request_state_handler_table[SCI_BASE_REQUEST_MAX_STATES] =
{
	/* SCI_BASE_REQUEST_STATE_INITIAL */
	{
		{
			scic_sds_request_default_start_handler,
			scic_sds_request_default_abort_handler,
			scic_sds_request_default_complete_handler,
			scic_sds_request_default_destruct_handler
		},
		scic_sds_request_default_tc_completion_handler,
		scic_sds_request_default_event_handler,
		scic_sds_request_default_frame_handler
	},
	/* SCI_BASE_REQUEST_STATE_CONSTRUCTED */
	{
		{
			scic_sds_request_constructed_state_start_handler,
			scic_sds_request_constructed_state_abort_handler,
			scic_sds_request_default_complete_handler,
			scic_sds_request_default_destruct_handler
		},
		scic_sds_request_default_tc_completion_handler,
		scic_sds_request_default_event_handler,
		scic_sds_request_default_frame_handler
	},
	/* SCI_BASE_REQUEST_STATE_STARTED */
	{
		{
			scic_sds_request_default_start_handler,
			scic_sds_request_started_state_abort_handler,
			scic_sds_request_default_complete_handler,
			scic_sds_request_default_destruct_handler
		},
		scic_sds_request_started_state_tc_completion_handler,
		scic_sds_request_default_event_handler,
		scic_sds_request_started_state_frame_handler
	},
	/* SCI_BASE_REQUEST_STATE_COMPLETED */
	{
		{
			scic_sds_request_default_start_handler,
			scic_sds_request_default_abort_handler,
			scic_sds_request_completed_state_complete_handler,
			scic_sds_request_default_destruct_handler
		},
		scic_sds_request_default_tc_completion_handler,
		scic_sds_request_default_event_handler,
		scic_sds_request_default_frame_handler
	},
	/* SCI_BASE_REQUEST_STATE_ABORTING */
	{
		{
			scic_sds_request_default_start_handler,
			scic_sds_request_aborting_state_abort_handler,
			scic_sds_request_default_complete_handler,
			scic_sds_request_default_destruct_handler
		},
		scic_sds_request_aborting_state_tc_completion_handler,
		scic_sds_request_default_event_handler,
		scic_sds_request_aborting_state_frame_handler,
	},
	/* SCI_BASE_REQUEST_STATE_FINAL */
	{
		{
			scic_sds_request_default_start_handler,
			scic_sds_request_default_abort_handler,
			scic_sds_request_default_complete_handler,
			scic_sds_request_default_destruct_handler
		},
		scic_sds_request_default_tc_completion_handler,
		scic_sds_request_default_event_handler,
		scic_sds_request_default_frame_handler
	}
};

