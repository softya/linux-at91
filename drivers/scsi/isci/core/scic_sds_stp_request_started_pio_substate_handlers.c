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
#include "intel_sat.h"
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
#include "scic_sds_unsolicited_frame_control.h"
#include "scic_sds_remote_device.h"
#include "scu_task_context.h"

/**
 *
 *
 *
 */

#define SCU_MAX_FRAME_BUFFER_SIZE  0x400  /* 1K is the maximum SCU frame data payload */

/**
 *
 * @this_request:
 * @length:
 *
 * This function will transmit DATA_FIS from (current sgl + offset) for input
 * parameter length. current sgl and offset is alreay stored in the IO request
 * enum sci_status
 */

static enum sci_status scic_sds_stp_request_pio_data_out_trasmit_data_frame(
	struct scic_sds_request *this_request,
	u32 length)
{
	struct scic_sds_stp_request *this_sds_stp_request = (struct scic_sds_stp_request *)this_request;
	sci_base_controller_request_handler_t continue_io;
	struct scu_sgl_element *current_sgl;
	struct scic_sds_controller *scic;
	u32 state;

	/*
	 * Recycle the TC and reconstruct it for sending out DATA FIS containing
	 * for the data from current_sgl+offset for the input length */
	struct scu_task_context *task_context = scic_sds_controller_get_task_context_buffer(
		this_request->owning_controller,
		this_request->io_tag
		);

	if (this_sds_stp_request->type.pio.request_current.sgl_set == SCU_SGL_ELEMENT_PAIR_A)
		current_sgl = &(this_sds_stp_request->type.pio.request_current.sgl_pair->A);
	else
		current_sgl = &(this_sds_stp_request->type.pio.request_current.sgl_pair->B);

	/* update the TC */
	task_context->command_iu_upper = current_sgl->address_upper;
	task_context->command_iu_lower = current_sgl->address_lower;
	task_context->transfer_length_bytes = length;
	task_context->type.stp.fis_type = SATA_FIS_TYPE_DATA;

	/* send the new TC out. */
	scic = this_request->owning_controller;
	state = scic->parent.state_machine.current_state_id;
	continue_io = scic_sds_controller_state_handler_table[state].base.continue_io;
	return continue_io(&scic->parent, &this_request->target_device->parent,
			   &this_request->parent);
}

/**
 *
 * @this_request:
 *
 * enum sci_status
 */
static enum sci_status scic_sds_stp_request_pio_data_out_transmit_data(
	struct scic_sds_request *this_sds_request)
{

	struct scu_sgl_element *current_sgl;
	u32 sgl_offset;
	u32 remaining_bytes_in_current_sgl = 0;
	enum sci_status status = SCI_SUCCESS;

	struct scic_sds_stp_request *this_sds_stp_request = (struct scic_sds_stp_request *)this_sds_request;

	sgl_offset = this_sds_stp_request->type.pio.request_current.sgl_offset;

	if (this_sds_stp_request->type.pio.request_current.sgl_set == SCU_SGL_ELEMENT_PAIR_A) {
		current_sgl = &(this_sds_stp_request->type.pio.request_current.sgl_pair->A);
		remaining_bytes_in_current_sgl = this_sds_stp_request->type.pio.request_current.sgl_pair->A.length - sgl_offset;
	} else {
		current_sgl = &(this_sds_stp_request->type.pio.request_current.sgl_pair->B);
		remaining_bytes_in_current_sgl = this_sds_stp_request->type.pio.request_current.sgl_pair->B.length - sgl_offset;
	}


	if (this_sds_stp_request->type.pio.pio_transfer_bytes > 0) {
		if (this_sds_stp_request->type.pio.pio_transfer_bytes >= remaining_bytes_in_current_sgl) {
			/* recycle the TC and send the H2D Data FIS from (current sgl + sgl_offset) and length = remaining_bytes_in_current_sgl */
			status = scic_sds_stp_request_pio_data_out_trasmit_data_frame(this_sds_request, remaining_bytes_in_current_sgl);
			if (status == SCI_SUCCESS) {
				this_sds_stp_request->type.pio.pio_transfer_bytes -= remaining_bytes_in_current_sgl;

				/* update the current sgl, sgl_offset and save for future */
				current_sgl = scic_sds_stp_request_pio_get_next_sgl(this_sds_stp_request);
				sgl_offset = 0;
			}
		} else if (this_sds_stp_request->type.pio.pio_transfer_bytes < remaining_bytes_in_current_sgl) {
			/* recycle the TC and send the H2D Data FIS from (current sgl + sgl_offset) and length = type.pio.pio_transfer_bytes */
			scic_sds_stp_request_pio_data_out_trasmit_data_frame(this_sds_request, this_sds_stp_request->type.pio.pio_transfer_bytes);

			if (status == SCI_SUCCESS) {
				/* Sgl offset will be adjusted and saved for future */
				sgl_offset += this_sds_stp_request->type.pio.pio_transfer_bytes;
				current_sgl->address_lower += this_sds_stp_request->type.pio.pio_transfer_bytes;
				this_sds_stp_request->type.pio.pio_transfer_bytes = 0;
			}
		}
	}

	if (status == SCI_SUCCESS) {
		this_sds_stp_request->type.pio.request_current.sgl_offset = sgl_offset;
	}

	return status;
}

/**
 *
 * @this_request: The request that is used for the SGL processing.
 * @data_buffer: The buffer of data to be copied.
 * @length: The length of the data transfer.
 *
 * Copy the data from the buffer for the length specified to the IO reqeust SGL
 * specified data region. enum sci_status
 */
static enum sci_status scic_sds_stp_request_pio_data_in_copy_data_buffer(
	struct scic_sds_stp_request *this_request,
	u8 *data_buffer,
	u32 length)
{
	enum sci_status status;
	struct scu_sgl_element *current_sgl;
	u32 sgl_offset;
	u32 data_offset;
	u8 *source_address;
	u8 *destination_address;
	u32 copy_length;

	/* Initial setup to get the current working SGL and the offset within the buffer */
	current_sgl =
		(this_request->type.pio.request_current.sgl_set == SCU_SGL_ELEMENT_PAIR_A) ?
		&(this_request->type.pio.request_current.sgl_pair->A) :
		&(this_request->type.pio.request_current.sgl_pair->B);

	sgl_offset = this_request->type.pio.request_current.sgl_offset;

	source_address = data_buffer;
	data_offset = 0;

	status = SCI_SUCCESS;

	/* While we are still doing Ok and there is more data to transfer */
	while (
		(length > 0)
		&& (status == SCI_SUCCESS)
		) {
		if (current_sgl->length == sgl_offset) {
			/* This SGL has been exauhasted so we need to get the next SGL */
			current_sgl = scic_sds_stp_request_pio_get_next_sgl(this_request);

			if (current_sgl == NULL)
				status = SCI_FAILURE;
			else
				sgl_offset = 0;
		} else {
			dma_addr_t physical_address;

			sci_cb_make_physical_address(
				physical_address,
				current_sgl->address_upper,
				current_sgl->address_lower
				);

			destination_address = (u8 *)scic_cb_get_virtual_address(
				this_request->parent.owning_controller,
				physical_address
				);

			source_address += data_offset;
			destination_address += sgl_offset;

			copy_length = MIN(length, current_sgl->length - sgl_offset);

			memcpy(destination_address, source_address, copy_length);

			length -= copy_length;
			sgl_offset += copy_length;
			data_offset += copy_length;
		}
	}

	this_request->type.pio.request_current.sgl_offset = sgl_offset;

	return status;
}

/**
 *
 * @this_request: The PIO DATA IN request that is to receive the data.
 * @data_buffer: The buffer to copy from.
 *
 * Copy the data buffer to the io request data region. enum sci_status
 */
static enum sci_status scic_sds_stp_request_pio_data_in_copy_data(
	struct scic_sds_stp_request *this_request,
	u8 *data_buffer)
{
	enum sci_status status;

	/*
	 * If there is less than 1K remaining in the transfer request
	 * copy just the data for the transfer */
	if (this_request->type.pio.pio_transfer_bytes < SCU_MAX_FRAME_BUFFER_SIZE) {
		status = scic_sds_stp_request_pio_data_in_copy_data_buffer(
			this_request, data_buffer, this_request->type.pio.pio_transfer_bytes);

		if (status == SCI_SUCCESS)
			this_request->type.pio.pio_transfer_bytes = 0;
	} else {
		/* We are transfering the whole frame so copy */
		status = scic_sds_stp_request_pio_data_in_copy_data_buffer(
			this_request, data_buffer, SCU_MAX_FRAME_BUFFER_SIZE);

		if (status == SCI_SUCCESS)
			this_request->type.pio.pio_transfer_bytes -= SCU_MAX_FRAME_BUFFER_SIZE;
	}

	return status;
}

/**
 *
 * @this_request:
 * @completion_code:
 *
 * enum sci_status
 */
static enum sci_status scic_sds_stp_request_pio_await_h2d_completion_tc_completion_handler(
	struct scic_sds_request *this_request,
	u32 completion_code)
{
	enum sci_status status = SCI_SUCCESS;

	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_request),
			       SCIC_LOG_OBJECT_STP_IO_REQUEST,
			       "scic_sds_stp_request_pio_data_in_await_h2d_completion_tc_completion_handler(0x%x, 0x%x) enter\n",
			       this_request, completion_code
			       ));

	switch (SCU_GET_COMPLETION_TL_STATUS(completion_code)) {
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_GOOD):
		scic_sds_request_set_status(
			this_request, SCU_TASK_DONE_GOOD, SCI_SUCCESS
			);

		sci_base_state_machine_change_state(
			&this_request->started_substate_machine,
			SCIC_SDS_STP_REQUEST_STARTED_PIO_AWAIT_FRAME_SUBSTATE
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
 *
 * @this_request:
 * @frame_index:
 *
 * enum sci_status
 */
static enum sci_status scic_sds_stp_request_pio_await_frame_frame_handler(
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
			       "scic_sds_stp_request_pio_data_in_await_frame_frame_handler(0x%x, 0x%x) enter\n",
			       this_request, frame_index
			       ));

	status = scic_sds_unsolicited_frame_control_get_header(
		&(this_request->parent.owning_controller->uf_control),
		frame_index,
		(void **)&frame_header
		);

	if (status == SCI_SUCCESS) {
		switch (frame_header->fis_type) {
		case SATA_FIS_TYPE_PIO_SETUP:
			/* Get from the frame buffer the PIO Setup Data */
			scic_sds_unsolicited_frame_control_get_buffer(
				&(this_request->parent.owning_controller->uf_control),
				frame_index,
				(void **)&frame_buffer
				);

			/*
			 * Get the data from the PIO Setup
			 * The SCU Hardware returns first word in the frame_header and the rest
			 * of the data is in the frame buffer so we need to back up one dword */
			this_request->type.pio.pio_transfer_bytes =
				(u16)((struct sata_fis_pio_setup *)(&frame_buffer[-1]))->transfter_count;
			this_request->type.pio.ending_status =
				(u8)((struct sata_fis_pio_setup *)(&frame_buffer[-1]))->ending_status;

			scic_sds_controller_copy_sata_response(
				&this_request->d2h_reg_fis, (u32 *)frame_header, frame_buffer
				);

			this_request->d2h_reg_fis.status =
				this_request->type.pio.ending_status;

			/* The next state is dependent on whether the request was PIO Data-in or Data out */
			if (this_request->type.pio.sat_protocol == SAT_PROTOCOL_PIO_DATA_IN) {
				sci_base_state_machine_change_state(
					&this_request->parent.started_substate_machine,
					SCIC_SDS_STP_REQUEST_STARTED_PIO_DATA_IN_AWAIT_DATA_SUBSTATE
					);
			} else if (this_request->type.pio.sat_protocol == SAT_PROTOCOL_PIO_DATA_OUT) {
				/* Transmit data */
				status = scic_sds_stp_request_pio_data_out_transmit_data(request);
				if (status == SCI_SUCCESS) {
					sci_base_state_machine_change_state(
						&this_request->parent.started_substate_machine,
						SCIC_SDS_STP_REQUEST_STARTED_PIO_DATA_OUT_TRANSMIT_DATA_SUBSTATE
						);
				}
			}
			break;

		case SATA_FIS_TYPE_SETDEVBITS:
			sci_base_state_machine_change_state(
				&this_request->parent.started_substate_machine,
				SCIC_SDS_STP_REQUEST_STARTED_PIO_AWAIT_FRAME_SUBSTATE
				);
			break;

		case SATA_FIS_TYPE_REGD2H:
			if ((frame_header->status & ATA_STATUS_REG_BSY_BIT) == 0) {
				scic_sds_unsolicited_frame_control_get_buffer(
					&(this_request->parent.owning_controller->uf_control),
					frame_index,
					(void **)&frame_buffer
					);

				scic_sds_controller_copy_sata_response(
					&this_request->d2h_reg_fis, (u32 *)frame_header, frame_buffer);

				scic_sds_request_set_status(
					&this_request->parent,
					SCU_TASK_DONE_CHECK_RESPONSE,
					SCI_FAILURE_IO_RESPONSE_VALID
					);

				sci_base_state_machine_change_state(
					&this_request->parent.parent.state_machine,
					SCI_BASE_REQUEST_STATE_COMPLETED
					);
			} else {
				/*
				 * Now why is the drive sending a D2H Register FIS when it is still busy?
				 * Do nothing since we are still in the right state. */
				SCIC_LOG_INFO((
						      sci_base_object_get_logger(this_request),
						      SCIC_LOG_OBJECT_STP_IO_REQUEST,
						      "SCIC PIO Request 0x%x received D2H Register FIS with BSY status 0x%x\n",
						      this_request, frame_header->status
						      ));
			}
			break;

		default:
			break;
		}

		/* Frame is decoded return it to the controller */
		scic_sds_controller_release_frame(
			this_request->parent.owning_controller,
			frame_index
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
 *
 * @this_request:
 * @frame_index:
 *
 * enum sci_status
 */
static enum sci_status scic_sds_stp_request_pio_data_in_await_data_frame_handler(
	struct scic_sds_request *request,
	u32 frame_index)
{
	enum sci_status status;
	struct sata_fis_header *frame_header;
	struct sata_fis_data *frame_buffer;
	struct scic_sds_stp_request *this_request;

	this_request = (struct scic_sds_stp_request *)request;

	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_request),
			       SCIC_LOG_OBJECT_STP_IO_REQUEST,
			       "scic_sds_stp_request_pio_data_in_await_data_frame_handler(0x%x, 0x%x) enter\n",
			       this_request, frame_index
			       ));

	status = scic_sds_unsolicited_frame_control_get_header(
		&(this_request->parent.owning_controller->uf_control),
		frame_index,
		(void **)&frame_header
		);

	if (status == SCI_SUCCESS) {
		if (frame_header->fis_type == SATA_FIS_TYPE_DATA) {
			if (this_request->type.pio.request_current.sgl_pair == NULL) {
				this_request->parent.saved_rx_frame_index = frame_index;
				this_request->type.pio.pio_transfer_bytes = 0;
			} else {
				status = scic_sds_unsolicited_frame_control_get_buffer(
					&(this_request->parent.owning_controller->uf_control),
					frame_index,
					(void **)&frame_buffer
					);

				status = scic_sds_stp_request_pio_data_in_copy_data(this_request, (u8 *)frame_buffer);

				/* Frame is decoded return it to the controller */
				scic_sds_controller_release_frame(
					this_request->parent.owning_controller,
					frame_index
					);
			}

			/*
			 * Check for the end of the transfer, are there more bytes remaining
			 * for this data transfer */
			if (
				(status == SCI_SUCCESS)
				&& (this_request->type.pio.pio_transfer_bytes == 0)
				) {
				if ((this_request->type.pio.ending_status & ATA_STATUS_REG_BSY_BIT) == 0) {
					scic_sds_request_set_status(
						&this_request->parent,
						SCU_TASK_DONE_CHECK_RESPONSE,
						SCI_FAILURE_IO_RESPONSE_VALID
						);

					sci_base_state_machine_change_state(
						&this_request->parent.parent.state_machine,
						SCI_BASE_REQUEST_STATE_COMPLETED
						);
				} else {
					sci_base_state_machine_change_state(
						&this_request->parent.started_substate_machine,
						SCIC_SDS_STP_REQUEST_STARTED_PIO_AWAIT_FRAME_SUBSTATE
						);
				}
			}
		} else {
			SCIC_LOG_ERROR((
					       sci_base_object_get_logger(this_request),
					       SCIC_LOG_OBJECT_STP_IO_REQUEST,
					       "SCIC PIO Request 0x%x received frame %d with fis type 0x%02x when expecting a data fis.\n",
					       this_request, frame_index, frame_header->fis_type
					       ));

			scic_sds_request_set_status(
				&this_request->parent,
				SCU_TASK_DONE_GOOD,
				SCI_FAILURE_IO_REQUIRES_SCSI_ABORT
				);

			sci_base_state_machine_change_state(
				&this_request->parent.parent.state_machine,
				SCI_BASE_REQUEST_STATE_COMPLETED
				);

			/* Frame is decoded return it to the controller */
			scic_sds_controller_release_frame(
				this_request->parent.owning_controller,
				frame_index
				);
		}
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
 *
 * @this_request:
 * @completion_code:
 *
 * enum sci_status
 */
static enum sci_status scic_sds_stp_request_pio_data_out_await_data_transmit_completion_tc_completion_handler(

	struct scic_sds_request *this_request,
	u32 completion_code)
{
	enum sci_status status                     = SCI_SUCCESS;
	bool all_frames_transferred     = false;

	struct scic_sds_stp_request *this_scic_sds_stp_request = (struct scic_sds_stp_request *)this_request;

	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_request),
			       SCIC_LOG_OBJECT_STP_IO_REQUEST,
			       "scic_sds_stp_request_pio_data_in_await_h2d_completion_tc_completion_handler(0x%x, 0x%x) enter\n",
			       this_request, completion_code
			       ));

	switch (SCU_GET_COMPLETION_TL_STATUS(completion_code)) {
	case SCU_MAKE_COMPLETION_STATUS(SCU_TASK_DONE_GOOD):
		/* Transmit data */
		if (this_scic_sds_stp_request->type.pio.pio_transfer_bytes != 0) {
			status = scic_sds_stp_request_pio_data_out_transmit_data(this_request);
			if (status == SCI_SUCCESS) {
				if (this_scic_sds_stp_request->type.pio.pio_transfer_bytes == 0)
					all_frames_transferred = true;
			}
		} else if (this_scic_sds_stp_request->type.pio.pio_transfer_bytes == 0) {
			/* this will happen if the all data is written at the first time after the pio setup fis is recieved */
			all_frames_transferred  = true;
		}

		/* all data transferred. */
		if (all_frames_transferred) {
			/*
			 * Change the state to SCIC_SDS_STP_REQUEST_STARTED_PIO_DATA_IN_AWAIT_FRAME_SUBSTATE
			 * and wait for PIO_SETUP fis / or D2H REg fis. */
			sci_base_state_machine_change_state(
				&this_request->started_substate_machine,
				SCIC_SDS_STP_REQUEST_STARTED_PIO_AWAIT_FRAME_SUBSTATE
				);
		}
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
 *
 * @request: This is the request which is receiving the event.
 * @event_code: This is the event code that the request on which the request is
 *    expected to take action.
 *
 * This method will handle any link layer events while waiting for the data
 * frame. enum sci_status SCI_SUCCESS SCI_FAILURE
 */
static enum sci_status scic_sds_stp_request_pio_data_in_await_data_event_handler(
	struct scic_sds_request *request,
	u32 event_code)
{
	enum sci_status status;

	switch (scu_get_event_specifier(event_code)) {
	case SCU_TASK_DONE_CRC_ERR << SCU_EVENT_SPECIFIC_CODE_SHIFT:
		/*
		 * We are waiting for data and the SCU has R_ERR the data frame.
		 * Go back to waiting for the D2H Register FIS */
		sci_base_state_machine_change_state(
			&request->started_substate_machine,
			SCIC_SDS_STP_REQUEST_STARTED_PIO_AWAIT_FRAME_SUBSTATE
			);

		status = SCI_SUCCESS;
		break;

	default:
		SCIC_LOG_ERROR((
				       sci_base_object_get_logger(request),
				       SCIC_LOG_OBJECT_STP_IO_REQUEST,
				       "SCIC PIO Request 0x%x received unexpected event 0x%08x\n",
				       request, event_code
				       ));

		/* / @todo Should we fail the PIO request when we get an unexpected event? */
		status = SCI_FAILURE;
		break;
	}

	return status;
}

/* --------------------------------------------------------------------------- */

const struct scic_sds_io_request_state_handler scic_sds_stp_request_started_pio_substate_handler_table[] = {
	[SCIC_SDS_STP_REQUEST_STARTED_PIO_AWAIT_H2D_COMPLETION_SUBSTATE] = {
		.parent.start_handler    = scic_sds_request_default_start_handler,
		.parent.abort_handler    = scic_sds_request_started_state_abort_handler,
		.parent.complete_handler = scic_sds_request_default_complete_handler,
		.parent.destruct_handler = scic_sds_request_default_destruct_handler,
		.tc_completion_handler   = scic_sds_stp_request_pio_await_h2d_completion_tc_completion_handler,
		.event_handler           = scic_sds_request_default_event_handler,
		.frame_handler           = scic_sds_request_default_frame_handler
	},
	[SCIC_SDS_STP_REQUEST_STARTED_PIO_AWAIT_FRAME_SUBSTATE] = {
		.parent.start_handler    = scic_sds_request_default_start_handler,
		.parent.abort_handler    = scic_sds_request_started_state_abort_handler,
		.parent.complete_handler = scic_sds_request_default_complete_handler,
		.parent.destruct_handler = scic_sds_request_default_destruct_handler,
		.tc_completion_handler   = scic_sds_request_default_tc_completion_handler,
		.event_handler           = scic_sds_request_default_event_handler,
		.frame_handler           = scic_sds_stp_request_pio_await_frame_frame_handler
	},
	[SCIC_SDS_STP_REQUEST_STARTED_PIO_DATA_IN_AWAIT_DATA_SUBSTATE] = {
		.parent.start_handler    = scic_sds_request_default_start_handler,
		.parent.abort_handler    = scic_sds_request_started_state_abort_handler,
		.parent.complete_handler = scic_sds_request_default_complete_handler,
		.parent.destruct_handler = scic_sds_request_default_destruct_handler,
		.tc_completion_handler   = scic_sds_request_default_tc_completion_handler,
		.event_handler           = scic_sds_stp_request_pio_data_in_await_data_event_handler,
		.frame_handler           = scic_sds_stp_request_pio_data_in_await_data_frame_handler
	},
	[SCIC_SDS_STP_REQUEST_STARTED_PIO_DATA_OUT_TRANSMIT_DATA_SUBSTATE] = {
		.parent.start_handler    = scic_sds_request_default_start_handler,
		.parent.abort_handler    = scic_sds_request_started_state_abort_handler,
		.parent.complete_handler = scic_sds_request_default_complete_handler,
		.parent.destruct_handler = scic_sds_request_default_destruct_handler,
		.tc_completion_handler   = scic_sds_stp_request_pio_data_out_await_data_transmit_completion_tc_completion_handler,
		.event_handler           = scic_sds_request_default_event_handler,
		.frame_handler           = scic_sds_request_default_frame_handler,
	}
};
