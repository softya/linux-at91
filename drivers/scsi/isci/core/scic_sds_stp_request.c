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

#include "intel_sat.h"
#include "intel_sata.h"
#include "sci_types.h"
#include "scic_remote_device.h"
#include "scic_user_callback.h"
#include "scic_sds_controller.h"
#include "scic_sds_remote_device.h"
#include "scic_sds_stp_request.h"
#include "scic_sds_stp_pio_request.h"

/**
 * scic_sds_stp_request_get_h2d_reg_buffer() -
 *
 * This macro returns the address of the stp h2d reg fis buffer in the io
 * request memory
 */
#define scic_sds_stp_request_get_h2d_reg_buffer(memory)	\
	((struct sata_fis_reg_h2d *)(\
		 ((char *)(memory)) + sizeof(struct scic_sds_stp_request) \
		 ))

/**
 * scic_sds_stp_request_get_response_buffer() -
 *
 * This macro returns the address of the ssp response iu buffer in the io
 * request memory
 */
#define scic_sds_stp_request_get_response_buffer(memory) \
	((struct sata_fis_reg_d2h *)(\
		 ((char *)(scic_sds_stp_request_get_h2d_reg_buffer(memory))) \
		 + sizeof(struct sata_fis_reg_h2d) \
		 ))

/**
 * scic_sds_stp_request_get_task_context_buffer() -
 *
 * This macro returns the address of the task context buffer in the io request
 * memory
 */
#define scic_sds_stp_request_get_task_context_buffer(memory) \
	((struct scu_task_context *)(\
		 ((char *)(scic_sds_stp_request_get_response_buffer(memory))) \
		 + sizeof(struct sci_ssp_response_iu) \
		 ))

/**
 * scic_sds_stp_request_get_sgl_element_buffer() -
 *
 * This macro returns the address of the sgl elment pairs in the io request
 * memory buffer
 */
#define scic_sds_stp_request_get_sgl_element_buffer(memory) \
	((struct scu_sgl_element_pair *)(\
		 ((char *)(scic_sds_stp_request_get_task_context_buffer(memory))) \
		 + sizeof(struct scu_task_context) \
		 ))

/**
 *
 *
 * This method return the memory space required for STP PIO requests. u32
 */
u32 scic_sds_stp_request_get_object_size(void)
{
	return sizeof(struct scic_sds_stp_request)
	       + sizeof(struct sata_fis_reg_h2d)
	       + sizeof(struct sata_fis_reg_d2h)
	       + sizeof(struct scu_task_context)
	       + sizeof(struct scu_sgl_element_pair) * SCU_MAX_SGL_ELEMENT_PAIRS;
}

/**
 *
 *
 *
 */
void scic_sds_stp_request_assign_buffers(
	struct scic_sds_request *request)
{
	struct scic_sds_stp_request *this_request = (struct scic_sds_stp_request *)request;

	this_request->parent.command_buffer =
		scic_sds_stp_request_get_h2d_reg_buffer(this_request);
	this_request->parent.response_buffer =
		scic_sds_stp_request_get_response_buffer(this_request);
	this_request->parent.sgl_element_pair_buffer =
		scic_sds_stp_request_get_sgl_element_buffer(this_request);
	this_request->parent.sgl_element_pair_buffer =
		scic_sds_request_align_sgl_element_buffer(this_request->parent.sgl_element_pair_buffer);

	if (this_request->parent.was_tag_assigned_by_user == false) {
		this_request->parent.task_context_buffer =
			scic_sds_stp_request_get_task_context_buffer(this_request);
		this_request->parent.task_context_buffer =
			scic_sds_request_align_task_context_buffer(this_request->parent.task_context_buffer);
	}
}

/**
 * This method is will fill in the SCU Task Context for any type of SATA
 *    request.  This is called from the various SATA constructors.
 * @this_request: The general IO request object which is to be used in
 *    constructing the SCU task context.
 * @task_context: The buffer pointer for the SCU task context which is being
 *    constructed.
 *
 * The general io request construction is complete. The buffer assignment for
 * the command buffer is complete. none Revisit task context construction to
 * determine what is common for SSP/SMP/STP task context structures.
 */
static void scu_sata_reqeust_construct_task_context(
	struct scic_sds_request *this_request,
	struct scu_task_context *task_context)
{
	dma_addr_t physical_address;
	struct scic_sds_controller *owning_controller;
	struct scic_sds_remote_device *target_device;
	struct scic_sds_port *target_port;

	owning_controller = scic_sds_request_get_controller(this_request);
	target_device = scic_sds_request_get_device(this_request);
	target_port = scic_sds_request_get_port(this_request);

	/* Fill in the TC with the its required data */
	task_context->abort = 0;
	task_context->priority = SCU_TASK_PRIORITY_NORMAL;
	task_context->initiator_request = 1;
	task_context->connection_rate =
		scic_remote_device_get_connection_rate(target_device);
	task_context->protocol_engine_index =
		scic_sds_controller_get_protocol_engine_group(owning_controller);
	task_context->logical_port_index =
		scic_sds_port_get_index(target_port);
	task_context->protocol_type = SCU_TASK_CONTEXT_PROTOCOL_STP;
	task_context->valid = SCU_TASK_CONTEXT_VALID;
	task_context->context_type = SCU_TASK_CONTEXT_TYPE;

	task_context->remote_node_index =
		scic_sds_remote_device_get_index(this_request->target_device);
	task_context->command_code = 0;

	task_context->link_layer_control = 0;
	task_context->do_not_dma_ssp_good_response = 1;
	task_context->strict_ordering = 0;
	task_context->control_frame = 0;
	task_context->timeout_enable = 0;
	task_context->block_guard_enable = 0;

	task_context->address_modifier = 0;
	task_context->task_phase = 0x01;

	task_context->ssp_command_iu_length =
		(sizeof(struct sata_fis_reg_h2d) - sizeof(u32)) / sizeof(u32);

	/* Set the first word of the H2D REG FIS */
	task_context->type.words[0] = *(u32 *)this_request->command_buffer;

	if (this_request->was_tag_assigned_by_user) {
		/* Build the task context now since we have already read the data */
		this_request->post_context = (
			SCU_CONTEXT_COMMAND_REQUEST_TYPE_POST_TC
			| (
				scic_sds_controller_get_protocol_engine_group(owning_controller)
				<< SCU_CONTEXT_COMMAND_PROTOCOL_ENGINE_GROUP_SHIFT
				)
			| (
				scic_sds_port_get_index(target_port)
				<< SCU_CONTEXT_COMMAND_LOGICAL_PORT_SHIFT
				)
			| scic_sds_io_tag_get_index(this_request->io_tag)
			);
	} else {
		/* Build the task context now since we have already read the data */
		this_request->post_context = (
			SCU_CONTEXT_COMMAND_REQUEST_TYPE_POST_TC
			| (
				scic_sds_controller_get_protocol_engine_group(owning_controller)
				<< SCU_CONTEXT_COMMAND_PROTOCOL_ENGINE_GROUP_SHIFT
				)
			| (
				scic_sds_port_get_index(target_port)
				<< SCU_CONTEXT_COMMAND_LOGICAL_PORT_SHIFT
				)
			/* This is not assigned because we have to wait until we get a TCi */
			);
	}

	/*
	 * Copy the physical address for the command buffer to the SCU Task Context
	 * We must offset the command buffer by 4 bytes because the first 4 bytes are
	 * transfered in the body of the TC */
	scic_cb_io_request_get_physical_address(
		scic_sds_request_get_controller(this_request),
		this_request,
		((char *)this_request->command_buffer) + sizeof(u32),
		&physical_address
		);

	task_context->command_iu_upper =
		sci_cb_physical_address_upper(physical_address);
	task_context->command_iu_lower =
		sci_cb_physical_address_lower(physical_address);

	/* SATA Requests do not have a response buffer */
	task_context->response_iu_upper = 0;
	task_context->response_iu_lower = 0;
}

/**
 *
 * @this_request:
 *
 * This method will perform any general sata request construction. What part of
 * SATA IO request construction is general? none
 */
void scic_sds_stp_non_ncq_request_construct(
	struct scic_sds_request *this_request)
{
	this_request->has_started_substate_machine = true;
}

/**
 *
 * @this_request: This parameter specifies the request to be constructed as an
 *    optimized request.
 * @optimized_task_type: This parameter specifies whether the request is to be
 *    an UDMA request or a NCQ request. - A value of 0 indicates UDMA. - A
 *    value of 1 indicates NCQ.
 *
 * This method will perform request construction common to all types of STP
 * requests that are optimized by the silicon (i.e. UDMA, NCQ). This method
 * returns an indication as to whether the construction was successful.
 */
static void scic_sds_stp_optimized_request_construct(
	struct scic_sds_request *this_request,
	u8 optimized_task_type,
	u32 transfer_length,
	SCI_IO_REQUEST_DATA_DIRECTION data_direction)
{
	struct scu_task_context *task_context = this_request->task_context_buffer;

	/* Build the STP task context structure */
	scu_sata_reqeust_construct_task_context(this_request, task_context);

	/* Copy over the SGL elements */
	scic_sds_request_build_sgl(this_request);

	/* Copy over the number of bytes to be transfered */
	task_context->transfer_length_bytes = transfer_length;

	if (data_direction == SCI_IO_REQUEST_DATA_OUT) {
		/*
		 * The difference between the DMA IN and DMA OUT request task type
		 * values are consistent with the difference between FPDMA READ
		 * and FPDMA WRITE values.  Add the supplied task type parameter
		 * to this difference to set the task type properly for this
		 * DATA OUT (WRITE) case. */
		task_context->task_type = optimized_task_type + (SCU_TASK_TYPE_DMA_OUT
								 - SCU_TASK_TYPE_DMA_IN);
	} else {
		/*
		 * For the DATA IN (READ) case, simply save the supplied
		 * optimized task type. */
		task_context->task_type = optimized_task_type;
	}
}

/**
 *
 * @this_request: This parameter specifies the request to be constructed.
 *
 * This method will construct the STP UDMA request and its associated TC data.
 * This method returns an indication as to whether the construction was
 * successful. SCI_SUCCESS Currently this method always returns this value.
 */
enum sci_status scic_sds_stp_udma_request_construct(
	struct scic_sds_request *this_request,
	u32 transfer_length,
	SCI_IO_REQUEST_DATA_DIRECTION data_direction)
{
	scic_sds_stp_non_ncq_request_construct(this_request);

	scic_sds_stp_optimized_request_construct(
		this_request,
		SCU_TASK_TYPE_DMA_IN,
		transfer_length,
		data_direction
		);

	sci_base_state_machine_construct(
		&this_request->started_substate_machine,
		&this_request->parent.parent,
		scic_sds_stp_request_started_udma_substate_table,
		SCIC_SDS_STP_REQUEST_STARTED_UDMA_AWAIT_TC_COMPLETION_SUBSTATE
		);

	return SCI_SUCCESS;
}

/**
 *
 * @this_request: This parameter specifies the request to be constructed.
 *
 * This method will construct the STP UDMA request and its associated TC data.
 * This method returns an indication as to whether the construction was
 * successful. SCI_SUCCESS Currently this method always returns this value.
 */
enum sci_status scic_sds_stp_ncq_request_construct(
	struct scic_sds_request *this_request,
	u32 transfer_length,
	SCI_IO_REQUEST_DATA_DIRECTION data_direction)
{
	scic_sds_stp_optimized_request_construct(
		this_request,
		SCU_TASK_TYPE_FPDMAQ_READ,
		transfer_length,
		data_direction
		);
	return SCI_SUCCESS;
}

/**
 *
 * @this_request: This parameter specifies the STP request object for which to
 *    construct a RAW command frame task context.
 * @task_context: This parameter specifies the SCU specific task context buffer
 *    to construct.
 *
 * This method performs the operations common to all SATA/STP requests
 * utilizing the raw frame method. none
 */
void scu_stp_raw_request_construct_task_context(
	struct scic_sds_stp_request *this_request,
	struct scu_task_context *task_context)
{
	scu_sata_reqeust_construct_task_context(&this_request->parent, task_context);

	task_context->control_frame         = 0;
	task_context->priority              = SCU_TASK_PRIORITY_NORMAL;
	task_context->task_type             = SCU_TASK_TYPE_SATA_RAW_FRAME;
	task_context->type.stp.fis_type     = SATA_FIS_TYPE_REGH2D;
	task_context->transfer_length_bytes = sizeof(struct sata_fis_reg_h2d) - sizeof(u32);
}

/**
 *
 * @this_request: This parameter specifies the core request object to
 *    construction into an STP/SATA non-data request.
 *
 * This method will construct the STP Non-data request and its associated TC
 * data.  A non-data request essentially behaves like a 0 length read request
 * in the SCU. This method currently always returns SCI_SUCCESS
 */
enum sci_status scic_sds_stp_non_data_request_construct(
	struct scic_sds_request *this_request)
{
	scic_sds_stp_non_ncq_request_construct(this_request);

	/* Build the STP task context structure */
	scu_stp_raw_request_construct_task_context(
		(struct scic_sds_stp_request *)this_request,
		this_request->task_context_buffer
		);

	sci_base_state_machine_construct(
		&this_request->started_substate_machine,
		&this_request->parent.parent,
		scic_sds_stp_request_started_non_data_substate_table,
		SCIC_SDS_STP_REQUEST_STARTED_NON_DATA_AWAIT_H2D_COMPLETION_SUBSTATE
		);

	return SCI_SUCCESS;
}


enum sci_status scic_sds_stp_soft_reset_request_construct(
	struct scic_sds_request *this_request)
{
	scic_sds_stp_non_ncq_request_construct(this_request);

	/* Build the STP task context structure */
	scu_stp_raw_request_construct_task_context(
		(struct scic_sds_stp_request *)this_request,
		this_request->task_context_buffer
		);

	sci_base_state_machine_construct(
		&this_request->started_substate_machine,
		&this_request->parent.parent,
		scic_sds_stp_request_started_soft_reset_substate_table,
		SCIC_SDS_STP_REQUEST_STARTED_SOFT_RESET_AWAIT_H2D_ASSERTED_COMPLETION_SUBSTATE
		);

	return SCI_SUCCESS;
}

/*
 * ****************************************************************************
 * * SCIC Interface Implementation
 * **************************************************************************** */

void scic_stp_io_request_set_ncq_tag(
	SCI_IO_REQUEST_HANDLE_T scic_io_request,
	u16 ncq_tag)
{
	/**
	 * @note This could be made to return an error to the user if the user
	 *       attempts to set the NCQ tag in the wrong state.
	 */
	struct scic_sds_request *this_request = (struct scic_sds_request *)scic_io_request;

	this_request->task_context_buffer->type.stp.ncq_tag = ncq_tag;
}

/* --------------------------------------------------------------------------- */

void *scic_stp_io_request_get_h2d_reg_address(
	SCI_IO_REQUEST_HANDLE_T scic_io_request
	) {
	struct scic_sds_request *this_request = (struct scic_sds_request *)scic_io_request;

	return this_request->command_buffer;
}

/* --------------------------------------------------------------------------- */

void *scic_stp_io_request_get_d2h_reg_address(
	SCI_IO_REQUEST_HANDLE_T scic_io_request
	) {
	struct scic_sds_stp_request *this_request = (struct scic_sds_stp_request *)scic_io_request;

	return &this_request->d2h_reg_fis;
}

