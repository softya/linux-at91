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

#include "scic_sds_stp_packet_request.h"
#include "scic_sds_logger.h"
#include "scic_sds_controller.h"
#include "scic_sds_remote_device.h"
#include "scic_remote_device.h"
#include "sci_util.h"
#include "intel_sas.h"
#include "intel_ata.h"
#include "intel_sata.h"
#include "scic_user_callback.h"
#include "sati_translator_sequence.h"


/**
 * This method will fill in the SCU Task Context for a PACKET fis. And
 *    construct the request STARTED sub-state machine for Packet Protocol IO.
 * @this_request: This parameter specifies the stp packet request object being
 *    constructed.
 *
 */
enum sci_status scic_sds_stp_packet_request_construct(
	struct scic_sds_request *this_request)
{
	struct sata_fis_reg_h2d *h2d_fis =
		scic_stp_io_request_get_h2d_reg_address(
			this_request
			);

	/*
	 * Work around, we currently only support PACKET DMA protocol, so we
	 * need to make change to Packet Fis features field. */
	h2d_fis->features = h2d_fis->features | ATA_PACKET_FEATURE_DMA;

	scic_sds_stp_non_ncq_request_construct(this_request);

	/* Build the Packet Fis task context structure */
	scu_stp_raw_request_construct_task_context(
		(struct scic_sds_stp_request *)this_request,
		this_request->task_context_buffer
		);

	sci_base_state_machine_construct(
		&this_request->started_substate_machine,
		&this_request->parent.parent,
		scic_sds_stp_packet_request_started_substate_table,
		SCIC_SDS_STP_PACKET_REQUEST_STARTED_PACKET_PHASE_AWAIT_TC_COMPLETION_SUBSTATE
		);

	return SCI_SUCCESS;
}


/**
 * This method will fill in the SCU Task Context for a Packet request command
 *    phase in PACKET DMA DATA (IN/OUT) type. The following important settings
 *    are utilized: -# task_type == SCU_TASK_TYPE_PACKET_DMA.  This simply
 *    indicates that a normal request type (i.e. non-raw frame) is being
 *    utilized to perform task management. -# control_frame == 1.  This ensures
 *    that the proper endianess is set so that the bytes are transmitted in the
 *    right order for a smp request frame.
 * @this_request: This parameter specifies the smp request object being
 *    constructed.
 * @task_context: The task_context to be reconstruct for packet request command
 *    phase.
 *
 */
void scu_stp_packet_request_command_phase_construct_task_context(
	struct scic_sds_request *this_request,
	struct scu_task_context *task_context)
{
	void *atapi_cdb;
	u32 atapi_cdb_length;
	struct scic_sds_stp_request *stp_request = (struct scic_sds_stp_request *)this_request;

	/*
	 * reference: SSTL 1.13.4.2
	 * task_type, sata_direction */
	if (scic_cb_io_request_get_data_direction(this_request->user_request)
	     == SCI_IO_REQUEST_DATA_OUT) {
		task_context->task_type = SCU_TASK_TYPE_PACKET_DMA_OUT;
		task_context->sata_direction = 0;
	} else {  /* todo: for NO_DATA command, we need to send out raw frame. */
		task_context->task_type = SCU_TASK_TYPE_PACKET_DMA_IN;
		task_context->sata_direction = 1;
	}

	/* sata header */
	memset(&(task_context->type.stp), 0, sizeof(struct STP_TASK_CONTEXT));
	task_context->type.stp.fis_type = SATA_FIS_TYPE_DATA;

	/*
	 * Copy in the command IU with CDB so that the commandIU address doesn't
	 * change. */
	memset(this_request->command_buffer, 0, sizeof(struct sata_fis_reg_h2d));

	atapi_cdb =
		scic_cb_stp_packet_io_request_get_cdb_address(this_request->user_request);

	atapi_cdb_length =
		scic_cb_stp_packet_io_request_get_cdb_length(this_request->user_request);

	memcpy(((u8 *)this_request->command_buffer + sizeof(u32)), atapi_cdb, atapi_cdb_length);

	atapi_cdb_length =
		MAX(atapi_cdb_length, stp_request->type.packet.device_preferred_cdb_length);

	task_context->ssp_command_iu_length =
		((atapi_cdb_length % 4) == 0) ?
		(atapi_cdb_length / 4) : ((atapi_cdb_length / 4) + 1);

	/* task phase is set to TX_CMD */
	task_context->task_phase = 0x1;

	/* retry counter */
	task_context->stp_retry_count = 0;

	if (scic_cb_request_is_initial_construction(this_request->user_request)) {
		/* data transfer size. */
		task_context->transfer_length_bytes =
			scic_cb_io_request_get_transfer_length(this_request->user_request);

		/* setup sgl */
		scic_sds_request_build_sgl(this_request);
	} else {
		/* data transfer size, need to be 4 bytes aligned. */
		task_context->transfer_length_bytes = (SCSI_FIXED_SENSE_DATA_BASE_LENGTH + 2);

		scic_sds_stp_packet_internal_request_sense_build_sgl(this_request);
	}
}

/**
 * This method will fill in the SCU Task Context for a DATA fis containing CDB
 *    in Raw Frame type. The TC for previous Packet fis was already there, we
 *    only need to change the H2D fis content.
 * @this_request: This parameter specifies the smp request object being
 *    constructed.
 * @task_context: The task_context to be reconstruct for packet request command
 *    phase.
 *
 */
void scu_stp_packet_request_command_phase_reconstruct_raw_frame_task_context(
	struct scic_sds_request *this_request,
	struct scu_task_context *task_context)
{
	void *atapi_cdb =
		scic_cb_stp_packet_io_request_get_cdb_address(this_request->user_request);

	u32 atapi_cdb_length =
		scic_cb_stp_packet_io_request_get_cdb_length(this_request->user_request);

	memset(this_request->command_buffer, 0, sizeof(struct sata_fis_reg_h2d));
	memcpy(((u8 *)this_request->command_buffer + sizeof(u32)), atapi_cdb, atapi_cdb_length);

	memset(&(task_context->type.stp), 0, sizeof(struct STP_TASK_CONTEXT));
	task_context->type.stp.fis_type = SATA_FIS_TYPE_DATA;

	/*
	 * Note the data send out has to be 4 bytes aligned. Or else out hardware will
	 * patch non-zero bytes and cause the target device unhappy. */
	task_context->transfer_length_bytes = 12;
}


/*
 * *@brief This methods decode the D2H status FIS and retrieve the sense data,
 *          then pass the sense data to user request.
 *
 ***@param[in] this_request The request receive D2H status FIS.
 ***@param[in] status_fis The D2H status fis to be processed.
 *
 */
enum sci_status scic_sds_stp_packet_request_process_status_fis(
	struct scic_sds_request *this_request,
	struct sata_fis_reg_d2h *status_fis)
{
	enum sci_status status = SCI_SUCCESS;

	/* TODO: Process the error status fis, retrieve sense data. */
	if (status_fis->status & ATA_STATUS_REG_ERROR_BIT)
		status = SCI_FAILURE_IO_RESPONSE_VALID;

	return status;
}

/*
 * *@brief This methods builds sgl for internal REQUEST SENSE stp packet
 *          command using this request response buffer, only one sge is
 *          needed.
 *
 ***@param[in] this_request The request receive request sense data.
 *
 */
void scic_sds_stp_packet_internal_request_sense_build_sgl(
	struct scic_sds_request *this_request)
{
	void *sge;
	struct scu_sgl_element_pair *scu_sgl_list   = NULL;
	struct scu_task_context *task_context;
	dma_addr_t physical_address;

	struct sci_ssp_response_iu *rsp_iu =
		(struct sci_ssp_response_iu *)this_request->response_buffer;

	sge =  (void *)&rsp_iu->data[0];

	task_context = (struct scu_task_context *)this_request->task_context_buffer;
	scu_sgl_list = &task_context->sgl_pair_ab;

	scic_cb_io_request_get_physical_address(
		scic_sds_request_get_controller(this_request),
		this_request,
		((char *)sge),
		&physical_address
		);

	scu_sgl_list->A.address_upper = sci_cb_physical_address_upper(physical_address);
	scu_sgl_list->A.address_lower = sci_cb_physical_address_lower(physical_address);
	scu_sgl_list->A.length = task_context->transfer_length_bytes;
	scu_sgl_list->A.address_modifier = 0;

	SCU_SGL_ZERO(scu_sgl_list->B);
}

#endif /* #if !defined(DISABLE_ATAPI) */
