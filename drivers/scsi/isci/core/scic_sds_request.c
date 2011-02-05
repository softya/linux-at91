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
 * This file contains the implementation for the operations on an
 *    SCIC_SDS_IO_REQUEST object.
 *
 *
 */

#include "intel_sat.h"
#include "intel_sata.h"
#include "intel_sas.h"
#include "sci_util.h"
#include "sci_environment.h"
#include "sci_base_request.h"
#include "scic_controller.h"
#include "scic_io_request.h"
#include "scic_remote_device.h"
#include "scic_user_callback.h"
#include "scic_sds_request.h"
#include "scic_sds_pci.h"
#include "scic_sds_stp_request.h"
#include "scic_sds_controller.h"
#include "scic_sds_controller_registers.h"
#include "scic_sds_remote_device.h"
#include "scic_sds_port.h"
#include "scu_constants.h"
#include "scu_task_context.h"
#include "scic_sds_smp_request.h"
#include "scic_sds_unsolicited_frame_control.h"
#include "sci_types.h"

#if !defined(DISABLE_ATAPI)
#include "scic_sds_stp_packet_request.h"
#endif

/*
 * ****************************************************************************
 * * SCIC SDS IO REQUEST CONSTANTS
 * **************************************************************************** */

/**
 *
 *
 * We have no timer requirements for IO requests right now
 */
#define SCIC_SDS_IO_REQUEST_MINIMUM_TIMER_COUNT (0)
#define SCIC_SDS_IO_REQUEST_MAXIMUM_TIMER_COUNT (0)

/*
 * ****************************************************************************
 * * SCIC SDS IO REQUEST MACROS
 * **************************************************************************** */

/**
 * scic_sds_request_get_user_request() -
 *
 * This is a helper macro to return the os handle for this request object.
 */
#define scic_sds_request_get_user_request(request) \
	((request)->user_request)


/**
 * scic_ssp_io_request_get_object_size() -
 *
 * This macro returns the sizeof memory required to store the an SSP IO
 * request.  This does not include the size of the SGL or SCU Task Context
 * memory.
 */
#define scic_ssp_io_request_get_object_size() \
	(\
		sizeof(struct sci_ssp_command_iu) \
		+ sizeof(struct sci_ssp_response_iu)	\
	)

/**
 * scic_sds_ssp_request_get_command_buffer() -
 *
 * This macro returns the address of the ssp command buffer in the io request
 * memory
 */
#define scic_sds_ssp_request_get_command_buffer(memory)	\
	((struct sci_ssp_command_iu *)(\
		 ((char *)(memory)) + sizeof(struct scic_sds_request) \
		 ))

/**
 * scic_sds_ssp_request_get_response_buffer() -
 *
 * This macro returns the address of the ssp response buffer in the io request
 * memory
 */
#define scic_sds_ssp_request_get_response_buffer(memory) \
	((struct sci_ssp_response_iu *)(\
		 ((char *)(scic_sds_ssp_request_get_command_buffer(memory))) \
		 + sizeof(struct sci_ssp_command_iu)	\
		 ))

/**
 * scic_sds_ssp_request_get_task_context_buffer() -
 *
 * This macro returns the address of the task context buffer in the io request
 * memory
 */
#define scic_sds_ssp_request_get_task_context_buffer(memory) \
	((struct scu_task_context *)(\
		 ((char *)(scic_sds_ssp_request_get_response_buffer(memory))) \
		 + sizeof(struct sci_ssp_response_iu) \
		 ))

/**
 * scic_sds_ssp_request_get_sgl_element_buffer() -
 *
 * This macro returns the address of the sgl elment pairs in the io request
 * memory buffer
 */
#define scic_sds_ssp_request_get_sgl_element_buffer(memory) \
	((struct scu_sgl_element_pair *)(\
		 ((char *)(scic_sds_ssp_request_get_task_context_buffer(memory))) \
		 + sizeof(struct scu_task_context) \
		 ))


/**
 * scic_ssp_task_request_get_object_size() -
 *
 * This macro returns the sizeof of memory required to store an SSP Task
 * request.  This does not include the size of the SCU Task Context memory.
 */
#define scic_ssp_task_request_get_object_size()	\
	(\
		sizeof(struct sci_ssp_task_iu) \
		+ sizeof(struct sci_ssp_response_iu)	\
	)

/**
 * scic_sds_ssp_task_request_get_command_buffer() -
 *
 * This macro returns the address of the ssp command buffer in the task request
 * memory.  Yes its the same as the above macro except for the name.
 */
#define scic_sds_ssp_task_request_get_command_buffer(memory) \
	((struct sci_ssp_task_iu *)(\
		 ((char *)(memory)) + sizeof(struct scic_sds_request) \
		 ))

/**
 * scic_sds_ssp_task_request_get_response_buffer() -
 *
 * This macro returns the address of the ssp response buffer in the task
 * request memory.
 */
#define scic_sds_ssp_task_request_get_response_buffer(memory) \
	((struct sci_ssp_response_iu *)(\
		 ((char *)(scic_sds_ssp_task_request_get_command_buffer(memory))) \
		 + sizeof(struct sci_ssp_task_iu) \
		 ))

/**
 * scic_sds_ssp_task_request_get_task_context_buffer() -
 *
 * This macro returs the task context buffer for the SSP task request.
 */
#define scic_sds_ssp_task_request_get_task_context_buffer(memory) \
	((struct scu_task_context *)(\
		 ((char *)(scic_sds_ssp_task_request_get_response_buffer(memory))) \
		 + sizeof(struct sci_ssp_response_iu) \
		 ))



/*
 * ****************************************************************************
 * * SCIC SDS IO REQUEST PRIVATE METHODS
 * **************************************************************************** */

/**
 *
 *
 * This method returns the size required to store an SSP IO request object. u32
 */
static u32 scic_sds_ssp_request_get_object_size(void)
{
	return sizeof(struct scic_sds_request)
	       + scic_ssp_io_request_get_object_size()
	       + sizeof(struct scu_task_context)
	       + CACHE_LINE_SIZE
	       + sizeof(struct scu_sgl_element_pair) * SCU_MAX_SGL_ELEMENT_PAIRS;
}

/**
 * This method returns the sgl element pair for the specificed sgl_pair index.
 * @this_request: This parameter specifies the IO request for which to retrieve
 *    the Scatter-Gather List element pair.
 * @sgl_pair_index: This parameter specifies the index into the SGL element
 *    pair to be retrieved.
 *
 * This method returns a pointer to an struct scu_sgl_element_pair.
 */
static struct scu_sgl_element_pair *scic_sds_request_get_sgl_element_pair(
	struct scic_sds_request *this_request,
	u32 sgl_pair_index
	) {
	struct scu_task_context *task_context;

	task_context = (struct scu_task_context *)this_request->task_context_buffer;

	if (sgl_pair_index == 0) {
		return &task_context->sgl_pair_ab;
	} else if (sgl_pair_index == 1) {
		return &task_context->sgl_pair_cd;
	}

	return &this_request->sgl_element_pair_buffer[sgl_pair_index - 2];
}

/**
 * This function will build the SGL list for an IO request.
 * @this_request: This parameter specifies the IO request for which to build
 *    the Scatter-Gather List.
 *
 */
void scic_sds_request_build_sgl(
	struct scic_sds_request *this_request)
{
	void *os_sge;
	void *os_handle;
	dma_addr_t physical_address;
	u32 sgl_pair_index = 0;
	struct scu_sgl_element_pair *scu_sgl_list   = NULL;
	struct scu_sgl_element_pair *previous_pair  = NULL;

	os_handle = scic_sds_request_get_user_request(this_request);
	scic_cb_io_request_get_next_sge(os_handle, NULL, &os_sge);

	while (os_sge != NULL) {
		scu_sgl_list =
			scic_sds_request_get_sgl_element_pair(this_request, sgl_pair_index);

		SCU_SGL_COPY(os_handle, scu_sgl_list->A, os_sge);

		scic_cb_io_request_get_next_sge(os_handle, os_sge, &os_sge);

		if (os_sge != NULL) {
			SCU_SGL_COPY(os_handle, scu_sgl_list->B, os_sge);

			scic_cb_io_request_get_next_sge(os_handle, os_sge, &os_sge);
		} else {
			SCU_SGL_ZERO(scu_sgl_list->B);
		}

		if (previous_pair != NULL) {
			scic_cb_io_request_get_physical_address(
				scic_sds_request_get_controller(this_request),
				this_request,
				scu_sgl_list,
				&physical_address
				);

			previous_pair->next_pair_upper =
				upper_32_bits(physical_address);
			previous_pair->next_pair_lower =
				lower_32_bits(physical_address);
		}

		previous_pair = scu_sgl_list;
		sgl_pair_index++;
	}

	if (scu_sgl_list != NULL) {
		scu_sgl_list->next_pair_upper = 0;
		scu_sgl_list->next_pair_lower = 0;
	}
}

/**
 * This method initializes common portions of the io request object. This
 *    includes construction of the struct sci_base_request parent.
 * @the_controller: This parameter specifies the controller for which the
 *    request is being constructed.
 * @the_target: This parameter specifies the remote device for which the
 *    request is being constructed.
 * @io_tag: This parameter specifies the IO tag to be utilized for this
 *    request.  This parameter can be set to SCI_CONTROLLER_INVALID_IO_TAG.
 * @user_io_request_object: This parameter specifies the user request object
 *    for which the request is being constructed.
 * @this_request: This parameter specifies the request being constructed.
 *
 */
static void scic_sds_general_request_construct(
	struct scic_sds_controller *the_controller,
	struct scic_sds_remote_device *the_target,
	u16 io_tag,
	void *user_io_request_object,
	struct scic_sds_request *this_request)
{
	sci_base_request_construct(
		&this_request->parent,
		scic_sds_request_state_table
		);

	this_request->io_tag = io_tag;
	this_request->user_request = user_io_request_object;
	this_request->owning_controller = the_controller;
	this_request->target_device = the_target;
	this_request->has_started_substate_machine = false;
	this_request->protocol = SCIC_NO_PROTOCOL;
	this_request->saved_rx_frame_index = SCU_INVALID_FRAME_INDEX;
	this_request->device_sequence = scic_sds_remote_device_get_sequence(the_target);

	this_request->sci_status   = SCI_SUCCESS;
	this_request->scu_status   = 0;
	this_request->post_context = 0xFFFFFFFF;

	this_request->is_task_management_request = false;

	if (io_tag == SCI_CONTROLLER_INVALID_IO_TAG) {
		this_request->was_tag_assigned_by_user = false;
		this_request->task_context_buffer = NULL;
	} else {
		this_request->was_tag_assigned_by_user = true;

		this_request->task_context_buffer =
			scic_sds_controller_get_task_context_buffer(
				this_request->owning_controller, io_tag);
	}
}

/**
 * This method build the remainder of the IO request object.
 * @this_request: This parameter specifies the request object being constructed.
 *
 * The scic_sds_general_request_construct() must be called before this call is
 * valid. none
 */
static void scic_sds_ssp_io_request_assign_buffers(
	struct scic_sds_request *this_request)
{
	this_request->command_buffer =
		scic_sds_ssp_request_get_command_buffer(this_request);
	this_request->response_buffer =
		scic_sds_ssp_request_get_response_buffer(this_request);
	this_request->sgl_element_pair_buffer =
		scic_sds_ssp_request_get_sgl_element_buffer(this_request);
	this_request->sgl_element_pair_buffer =
		scic_sds_request_align_sgl_element_buffer(this_request->sgl_element_pair_buffer);

	if (this_request->was_tag_assigned_by_user == false) {
		this_request->task_context_buffer =
			scic_sds_ssp_request_get_task_context_buffer(this_request);
		this_request->task_context_buffer =
			scic_sds_request_align_task_context_buffer(this_request->task_context_buffer);
	}
}

/**
 * This method constructs the SSP Command IU data for this io request object.
 * @this_request: This parameter specifies the request object for which the SSP
 *    command information unit is being built.
 *
 */
static void scic_sds_io_request_build_ssp_command_iu(
	struct scic_sds_request *this_request)
{
	struct sci_ssp_command_iu *command_frame;
	void *os_handle;
	u32 cdb_length;
	u32 *cdb_buffer;

	command_frame =
		(struct sci_ssp_command_iu *)this_request->command_buffer;

	os_handle = scic_sds_request_get_user_request(this_request);

	command_frame->lun_upper = 0;
	command_frame->lun_lower = scic_cb_ssp_io_request_get_lun(os_handle);

	((u32 *)command_frame)[2] = 0;

	cdb_length = scic_cb_ssp_io_request_get_cdb_length(os_handle);
	cdb_buffer = (u32 *)scic_cb_ssp_io_request_get_cdb_address(os_handle);

	if (cdb_length > 16) {
		command_frame->additional_cdb_length = cdb_length - 16;
	}

	/* / @todo Is it ok to leave junk at the end of the cdb buffer? */
	scic_word_copy_with_swap(
		(u32 *)(&command_frame->cdb),
		(u32 *)(cdb_buffer),
		(cdb_length + 3) / sizeof(u32)
		);

	command_frame->enable_first_burst = 0;
	command_frame->task_priority =
		scic_cb_ssp_io_request_get_command_priority(os_handle);
	command_frame->task_attribute =
		scic_cb_ssp_io_request_get_task_attribute(os_handle);
}


/**
 * This method constructs the SSP Task IU data for this io request object.
 * @this_request:
 *
 */
static void scic_sds_task_request_build_ssp_task_iu(
	struct scic_sds_request *this_request)
{
	struct sci_ssp_task_iu *command_frame;
	void *os_handle;

	command_frame =
		(struct sci_ssp_task_iu *)this_request->command_buffer;

	os_handle = scic_sds_request_get_user_request(this_request);

	command_frame->lun_upper = 0;
	command_frame->lun_lower = scic_cb_ssp_task_request_get_lun(os_handle);

	((u32 *)command_frame)[2] = 0;

	command_frame->task_function =
		scic_cb_ssp_task_request_get_function(os_handle);
	command_frame->task_tag =
		scic_cb_ssp_task_request_get_io_tag_to_manage(os_handle);
}


/**
 * This method is will fill in the SCU Task Context for any type of SSP request.
 * @this_request:
 * @task_context:
 *
 */
static void scu_ssp_reqeust_construct_task_context(
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
	task_context->priority = 0;
	task_context->initiator_request = 1;
	task_context->connection_rate =
		scic_remote_device_get_connection_rate(target_device);
	task_context->protocol_engine_index =
		scic_sds_controller_get_protocol_engine_group(owning_controller);
	task_context->logical_port_index =
		scic_sds_port_get_index(target_port);
	task_context->protocol_type = SCU_TASK_CONTEXT_PROTOCOL_SSP;
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

	/* task_context->type.ssp.tag = this_request->io_tag; */
	task_context->task_phase = 0x01;

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

	/* Copy the physical address for the command buffer to the SCU Task Context */
	scic_cb_io_request_get_physical_address(
		scic_sds_request_get_controller(this_request),
		this_request,
		this_request->command_buffer,
		&physical_address
		);

	task_context->command_iu_upper =
		upper_32_bits(physical_address);
	task_context->command_iu_lower =
		lower_32_bits(physical_address);

	/* Copy the physical address for the response buffer to the SCU Task Context */
	scic_cb_io_request_get_physical_address(
		scic_sds_request_get_controller(this_request),
		this_request,
		this_request->response_buffer,
		&physical_address
		);

	task_context->response_iu_upper =
		upper_32_bits(physical_address);
	task_context->response_iu_lower =
		lower_32_bits(physical_address);
}

/**
 * This method is will fill in the SCU Task Context for a SSP IO request.
 * @this_request:
 *
 */
static void scu_ssp_io_request_construct_task_context(
	struct scic_sds_request *this_request,
	SCI_IO_REQUEST_DATA_DIRECTION data_direction,
	u32 transfer_length_bytes)
{
	struct scu_task_context *task_context;

	task_context = scic_sds_request_get_task_context(this_request);

	scu_ssp_reqeust_construct_task_context(this_request, task_context);

	task_context->ssp_command_iu_length = sizeof(struct sci_ssp_command_iu) / sizeof(u32);
	task_context->type.ssp.frame_type = SCI_SAS_COMMAND_FRAME;

	switch (data_direction) {
	case SCI_IO_REQUEST_DATA_IN:
	case SCI_IO_REQUEST_NO_DATA:
		task_context->task_type = SCU_TASK_TYPE_IOREAD;
		break;
	case SCI_IO_REQUEST_DATA_OUT:
		task_context->task_type = SCU_TASK_TYPE_IOWRITE;
		break;
	}

	task_context->transfer_length_bytes = transfer_length_bytes;

	if (task_context->transfer_length_bytes > 0) {
		scic_sds_request_build_sgl(this_request);
	}
}


/**
 * This method will fill in the remainder of the io request object for SSP Task
 *    requests.
 * @this_request:
 *
 */
static void scic_sds_ssp_task_request_assign_buffers(
	struct scic_sds_request *this_request)
{
	/* Assign all of the buffer pointers */
	this_request->command_buffer =
		scic_sds_ssp_task_request_get_command_buffer(this_request);
	this_request->response_buffer =
		scic_sds_ssp_task_request_get_response_buffer(this_request);
	this_request->sgl_element_pair_buffer = NULL;

	if (this_request->was_tag_assigned_by_user == false) {
		this_request->task_context_buffer =
			scic_sds_ssp_task_request_get_task_context_buffer(this_request);
		this_request->task_context_buffer =
			scic_sds_request_align_task_context_buffer(this_request->task_context_buffer);
	}
}

/**
 * This method will fill in the SCU Task Context for a SSP Task request.  The
 *    following important settings are utilized: -# priority ==
 *    SCU_TASK_PRIORITY_HIGH.  This ensures that the task request is issued
 *    ahead of other task destined for the same Remote Node. -# task_type ==
 *    SCU_TASK_TYPE_IOREAD.  This simply indicates that a normal request type
 *    (i.e. non-raw frame) is being utilized to perform task management. -#
 *    control_frame == 1.  This ensures that the proper endianess is set so
 *    that the bytes are transmitted in the right order for a task frame.
 * @this_request: This parameter specifies the task request object being
 *    constructed.
 *
 */
static void scu_ssp_task_request_construct_task_context(
	struct scic_sds_request *this_request)
{
	struct scu_task_context *task_context;

	task_context = scic_sds_request_get_task_context(this_request);

	scu_ssp_reqeust_construct_task_context(this_request, task_context);

	task_context->control_frame                = 1;
	task_context->priority                     = SCU_TASK_PRIORITY_HIGH;
	task_context->task_type                    = SCU_TASK_TYPE_RAW_FRAME;
	task_context->transfer_length_bytes        = 0;
	task_context->type.ssp.frame_type          = SCI_SAS_TASK_FRAME;
	task_context->ssp_command_iu_length = sizeof(struct sci_ssp_task_iu) / sizeof(u32);
}


/**
 * This method constructs the SSP Command IU data for this ssp passthrough
 *    comand request object.
 * @this_request: This parameter specifies the request object for which the SSP
 *    command information unit is being built.
 *
 * enum sci_status, returns invalid parameter is cdb > 16
 */


/**
 * This method constructs the SATA request object.
 * @this_request:
 * @sat_protocol:
 * @transfer_length:
 * @data_direction:
 * @copy_rx_frame:
 *
 * enum sci_status
 */
static enum sci_status scic_io_request_construct_sata(
	struct scic_sds_request *this_request,
	u8 sat_protocol,
	u32 transfer_length,
	SCI_IO_REQUEST_DATA_DIRECTION data_direction,
	bool copy_rx_frame)
{
	enum sci_status status = SCI_SUCCESS;

	switch (sat_protocol) {
	case SAT_PROTOCOL_PIO_DATA_IN:
	case SAT_PROTOCOL_PIO_DATA_OUT:
		status = scic_sds_stp_pio_request_construct(this_request, sat_protocol, copy_rx_frame);
		break;

	case SAT_PROTOCOL_UDMA_DATA_IN:
	case SAT_PROTOCOL_UDMA_DATA_OUT:
		status = scic_sds_stp_udma_request_construct(this_request, transfer_length, data_direction);
		break;

	case SAT_PROTOCOL_ATA_HARD_RESET:
	case SAT_PROTOCOL_SOFT_RESET:
		status = scic_sds_stp_soft_reset_request_construct(this_request);
		break;

	case SAT_PROTOCOL_NON_DATA:
		status = scic_sds_stp_non_data_request_construct(this_request);
		break;

	case SAT_PROTOCOL_FPDMA:
		status = scic_sds_stp_ncq_request_construct(this_request, transfer_length, data_direction);
		break;

#if !defined(DISABLE_ATAPI)
	case SAT_PROTOCOL_PACKET_NON_DATA:
	case SAT_PROTOCOL_PACKET_DMA_DATA_IN:
	case SAT_PROTOCOL_PACKET_DMA_DATA_OUT:
	case SAT_PROTOCOL_PACKET_PIO_DATA_IN:
	case SAT_PROTOCOL_PACKET_PIO_DATA_OUT:
		status = scic_sds_stp_packet_request_construct(this_request);
		break;
#endif

	case SAT_PROTOCOL_DMA_QUEUED:
	case SAT_PROTOCOL_DMA:
	case SAT_PROTOCOL_DEVICE_DIAGNOSTIC:
	case SAT_PROTOCOL_DEVICE_RESET:
	case SAT_PROTOCOL_RETURN_RESPONSE_INFO:
	default:
		dev_err(scic_to_dev(this_request->owning_controller),
			"%s: SCIC IO Request 0x%p received un-handled "
			"SAT Protocl %d.\n",
			__func__, this_request, sat_protocol);

		status = SCI_FAILURE;
		break;
	}

	return status;
}

/*
 * ****************************************************************************
 * * SCIC Interface Implementation
 * **************************************************************************** */




/* --------------------------------------------------------------------------- */

u32 scic_io_request_get_object_size(void)
{
	u32 ssp_request_size;
	u32 stp_request_size;
	u32 smp_request_size;

	ssp_request_size = scic_sds_ssp_request_get_object_size();
	stp_request_size = scic_sds_stp_request_get_object_size();
	smp_request_size = scic_sds_smp_request_get_object_size();

	return max(ssp_request_size, max(stp_request_size, smp_request_size));
}

/* --------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------- */

enum sci_status scic_io_request_construct(
	SCI_CONTROLLER_HANDLE_T scic_controller,
	SCI_REMOTE_DEVICE_HANDLE_T scic_remote_device,
	u16 io_tag,
	void *user_io_request_object,
	void *scic_io_request_memory,
	struct scic_sds_request **new_scic_io_request_handle)
{
	enum sci_status status = SCI_SUCCESS;
	struct scic_sds_request *this_request;
	struct smp_discover_response_protocols device_protocol;

	this_request = (struct scic_sds_request *)scic_io_request_memory;

	/* Build the common part of the request */
	scic_sds_general_request_construct(
		(struct scic_sds_controller *)scic_controller,
		(struct scic_sds_remote_device *)scic_remote_device,
		io_tag,
		user_io_request_object,
		this_request
		);

	if (
		scic_sds_remote_device_get_index((struct scic_sds_remote_device *)scic_remote_device)
		== SCIC_SDS_REMOTE_NODE_CONTEXT_INVALID_INDEX
		) {
		return SCI_FAILURE_INVALID_REMOTE_DEVICE;
	}

	scic_remote_device_get_protocols(scic_remote_device, &device_protocol);

	if (device_protocol.u.bits.attached_ssp_target) {
		scic_sds_ssp_io_request_assign_buffers(this_request);
	} else if (device_protocol.u.bits.attached_stp_target) {
		scic_sds_stp_request_assign_buffers(this_request);
		memset(this_request->command_buffer, 0, sizeof(struct sata_fis_reg_h2d));
	} else if (device_protocol.u.bits.attached_smp_target) {
		scic_sds_smp_request_assign_buffers(this_request);
		memset(this_request->command_buffer, 0, sizeof(struct smp_request));
	} else {
		status = SCI_FAILURE_UNSUPPORTED_PROTOCOL;
	}

	if (status == SCI_SUCCESS) {
		memset(
			this_request->task_context_buffer,
			0,
			SCI_FIELD_OFFSET(struct scu_task_context, sgl_pair_ab)
			);
		*new_scic_io_request_handle = scic_io_request_memory;
	}

	return status;
}

/* --------------------------------------------------------------------------- */


enum sci_status scic_task_request_construct(
	SCI_CONTROLLER_HANDLE_T controller,
	SCI_REMOTE_DEVICE_HANDLE_T remote_device,
	u16 io_tag,
	void *user_io_request_object,
	void *scic_task_request_memory,
	struct scic_sds_request **new_scic_task_request_handle)
{
	enum sci_status status = SCI_SUCCESS;
	struct scic_sds_request *this_request = (struct scic_sds_request *)
					   scic_task_request_memory;
	struct smp_discover_response_protocols device_protocol;

	/* Build the common part of the request */
	scic_sds_general_request_construct(
		(struct scic_sds_controller *)controller,
		(struct scic_sds_remote_device *)remote_device,
		io_tag,
		user_io_request_object,
		this_request
		);

	scic_remote_device_get_protocols(remote_device, &device_protocol);

	if (device_protocol.u.bits.attached_ssp_target) {
		scic_sds_ssp_task_request_assign_buffers(this_request);

		this_request->has_started_substate_machine = true;

		/* Construct the started sub-state machine. */
		sci_base_state_machine_construct(
			&this_request->started_substate_machine,
			&this_request->parent.parent,
			scic_sds_io_request_started_task_mgmt_substate_table,
			SCIC_SDS_IO_REQUEST_STARTED_TASK_MGMT_SUBSTATE_AWAIT_TC_COMPLETION
			);
	} else if (device_protocol.u.bits.attached_stp_target) {
		scic_sds_stp_request_assign_buffers(this_request);
	} else {
		status = SCI_FAILURE_UNSUPPORTED_PROTOCOL;
	}

	if (status == SCI_SUCCESS) {
		this_request->is_task_management_request = true;
		memset(this_request->task_context_buffer, 0x00, sizeof(struct scu_task_context));
		*new_scic_task_request_handle            = scic_task_request_memory;
	}

	return status;
}


enum sci_status scic_io_request_construct_basic_ssp(
	struct scic_sds_request *sci_req)
{
	void *os_handle;

	sci_req->protocol = SCIC_SSP_PROTOCOL;

	os_handle = scic_sds_request_get_user_request(sci_req);

	scu_ssp_io_request_construct_task_context(
		sci_req,
		scic_cb_io_request_get_data_direction(os_handle),
		scic_cb_io_request_get_transfer_length(os_handle)
		);


	scic_sds_io_request_build_ssp_command_iu(sci_req);

	sci_base_state_machine_change_state(
		&sci_req->parent.state_machine,
		SCI_BASE_REQUEST_STATE_CONSTRUCTED
		);

	return SCI_SUCCESS;
}


enum sci_status scic_task_request_construct_ssp(
	struct scic_sds_request *sci_req)
{
	/* Construct the SSP Task SCU Task Context */
	scu_ssp_task_request_construct_task_context(sci_req);

	/* Fill in the SSP Task IU */
	scic_sds_task_request_build_ssp_task_iu(sci_req);

	sci_base_state_machine_change_state(
		&sci_req->parent.state_machine,
		SCI_BASE_REQUEST_STATE_CONSTRUCTED
		);

	return SCI_SUCCESS;
}


enum sci_status scic_io_request_construct_basic_sata(
	struct scic_sds_request *sci_req)
{
	enum sci_status status;
	struct scic_sds_stp_request *this_stp_request;
	u8 sat_protocol;
	u32 transfer_length;
	SCI_IO_REQUEST_DATA_DIRECTION data_direction;
	bool copy_rx_frame = false;

	this_stp_request = (struct scic_sds_stp_request *)sci_req;

	sci_req->protocol = SCIC_STP_PROTOCOL;

	transfer_length =
		scic_cb_io_request_get_transfer_length(sci_req->user_request);
	data_direction =
		scic_cb_io_request_get_data_direction(sci_req->user_request);

	sat_protocol = scic_cb_request_get_sat_protocol(sci_req->user_request);
	copy_rx_frame = scic_cb_io_request_do_copy_rx_frames(this_stp_request->parent.user_request);

	status = scic_io_request_construct_sata(
		sci_req,
		sat_protocol,
		transfer_length,
		data_direction,
		copy_rx_frame
		);

	if (status == SCI_SUCCESS)
		sci_base_state_machine_change_state(
			&sci_req->parent.state_machine,
			SCI_BASE_REQUEST_STATE_CONSTRUCTED
			);

	return status;
}


enum sci_status scic_task_request_construct_sata(
	struct scic_sds_request *sci_req)
{
	enum sci_status status;
	u8 sat_protocol = scic_cb_request_get_sat_protocol(sci_req->user_request);

	switch (sat_protocol) {
	case SAT_PROTOCOL_ATA_HARD_RESET:
	case SAT_PROTOCOL_SOFT_RESET:
		status = scic_sds_stp_soft_reset_request_construct(sci_req);
		break;

	default:
		dev_err(scic_to_dev(sci_req->owning_controller),
			"%s: SCIC IO Request 0x%p received un-handled SAT "
			"Protocl %d.\n",
			__func__,
			sci_req,
			sat_protocol);

		status = SCI_FAILURE;
		break;
	}

	if (status == SCI_SUCCESS)
		sci_base_state_machine_change_state(
			&sci_req->parent.state_machine,
			SCI_BASE_REQUEST_STATE_CONSTRUCTED
			);

	return status;
}


u16 scic_io_request_get_io_tag(
	struct scic_sds_request *sci_req)
{
	return sci_req->io_tag;
}


u32 scic_request_get_controller_status(
	struct scic_sds_request *sci_req)
{
	return sci_req->scu_status;
}


void *scic_io_request_get_command_iu_address(
	struct scic_sds_request *sci_req)
{
	return sci_req->command_buffer;
}


void *scic_io_request_get_response_iu_address(
	struct scic_sds_request *sci_req)
{
	return sci_req->response_buffer;
}


#define SCU_TASK_CONTEXT_SRAM 0x200000
u32 scic_io_request_get_number_of_bytes_transferred(
	struct scic_sds_request *scic_sds_request)
{
	u32 ret_val = 0;

	if (SMU_AMR_READ(scic_sds_request->owning_controller) == 0) {
		/*
		 * get the bytes of data from the Address == BAR1 + 20002Ch + (256*TCi) where
		 *   BAR1 is the scu_registers
		 *   0x20002C = 0x200000 + 0x2c
		 *            = start of task context SRAM + offset of (type.ssp.data_offset)
		 *   TCi is the io_tag of struct scic_sds_request */
		ret_val =  scic_sds_pci_read_scu_dword(
			scic_sds_request->owning_controller,
			(
				(u8 *)scic_sds_request->owning_controller->scu_registers +
				(SCU_TASK_CONTEXT_SRAM + SCI_FIELD_OFFSET(struct scu_task_context, type.ssp.data_offset)) +
				((sizeof(struct scu_task_context)) * scic_sds_io_tag_get_index(scic_sds_request->io_tag))
			)
			);
	}

	return ret_val;
}


/*
 * ****************************************************************************
 * * SCIC SDS Interface Implementation
 * **************************************************************************** */

/**
 *
 * @this_request: The SCIC_SDS_IO_REQUEST_T object for which the start
 *    operation is to be executed.
 *
 * This method invokes the base state start request handler for the
 * SCIC_SDS_IO_REQUEST_T object. enum sci_status
 */
enum sci_status scic_sds_request_start(
	struct scic_sds_request *this_request)
{
	if (
		this_request->device_sequence
		== scic_sds_remote_device_get_sequence(this_request->target_device)
		) {
		return this_request->state_handlers->parent.start_handler(
			       &this_request->parent
			       );
	}

	return SCI_FAILURE;
}

/**
 *
 * @this_request: The SCIC_SDS_IO_REQUEST_T object for which the start
 *    operation is to be executed.
 *
 * This method invokes the base state terminate request handber for the
 * SCIC_SDS_IO_REQUEST_T object. enum sci_status
 */
enum sci_status scic_sds_io_request_terminate(
	struct scic_sds_request *this_request)
{
	return this_request->state_handlers->parent.abort_handler(
		       &this_request->parent);
}

/**
 *
 * @this_request: The SCIC_SDS_IO_REQUEST_T object for which the start
 *    operation is to be executed.
 *
 * This method invokes the base state request completion handler for the
 * SCIC_SDS_IO_REQUEST_T object. enum sci_status
 */
enum sci_status scic_sds_io_request_complete(
	struct scic_sds_request *this_request)
{
	return this_request->state_handlers->parent.complete_handler(
		       &this_request->parent);
}

/**
 *
 * @this_request: The SCIC_SDS_IO_REQUEST_T object for which the start
 *    operation is to be executed.
 * @event_code: The event code returned by the hardware for the task reqeust.
 *
 * This method invokes the core state handler for the SCIC_SDS_IO_REQUEST_T
 * object. enum sci_status
 */
enum sci_status scic_sds_io_request_event_handler(
	struct scic_sds_request *this_request,
	u32 event_code)
{
	return this_request->state_handlers->event_handler(this_request, event_code);
}

/**
 *
 * @this_request: The SCIC_SDS_IO_REQUEST_T object for which the start
 *    operation is to be executed.
 * @frame_index: The frame index returned by the hardware for the reqeust
 *    object.
 *
 * This method invokes the core state frame handler for the
 * SCIC_SDS_IO_REQUEST_T object. enum sci_status
 */
enum sci_status scic_sds_io_request_frame_handler(
	struct scic_sds_request *this_request,
	u32 frame_index)
{
	return this_request->state_handlers->frame_handler(this_request, frame_index);
}

/**
 *
 * @this_request: The SCIC_SDS_IO_REQUEST_T object for which the task start
 *    operation is to be executed.
 *
 * This method invokes the core state task complete handler for the
 * SCIC_SDS_IO_REQUEST_T object. enum sci_status
 */

/*
 * ****************************************************************************
 * * SCIC SDS PROTECTED METHODS
 * **************************************************************************** */

/**
 * This method copies response data for requests returning response data
 *    instead of sense data.
 * @this_request: This parameter specifies the request object for which to copy
 *    the response data.
 *
 */
void scic_sds_io_request_copy_response(
	struct scic_sds_request *this_request)
{
	void *response_buffer;
	u32 user_response_length;
	u32 core_response_length;
	struct sci_ssp_response_iu *ssp_response;

	ssp_response = (struct sci_ssp_response_iu *)this_request->response_buffer;

	response_buffer = scic_cb_ssp_task_request_get_response_data_address(
		this_request->user_request
		);

	user_response_length = scic_cb_ssp_task_request_get_response_data_length(
		this_request->user_request
		);

	core_response_length = sci_ssp_get_response_data_length(
		ssp_response->response_data_length
		);

	user_response_length = min(user_response_length, core_response_length);

	memcpy(response_buffer, ssp_response->data, user_response_length);
}

