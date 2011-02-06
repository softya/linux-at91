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
#include "sci_base_state_machine.h"
#include "scic_user_callback.h"
#include "scic_remote_device.h"
#include "scic_sds_stp_request.h"
#include "scic_sds_stp_pio_request.h"
#include "scic_sds_controller.h"
#include "scic_sds_remote_device.h"
#include "scu_task_context.h"

/**
 *
 * @this_request:
 *
 * Get the next SGL element from the request. - Check on which SGL element pair
 * we are working - if working on SLG pair element A - advance to element B -
 * else - check to see if there are more SGL element pairs for this IO request
 * - if there are more SGL element pairs - advance to the next pair and return
 * element A struct scu_sgl_element*
 */
struct scu_sgl_element *scic_sds_stp_request_pio_get_next_sgl(
	struct scic_sds_stp_request *this_request
	) {
	struct scu_sgl_element *current_sgl;

	if (this_request->type.pio.request_current.sgl_set == SCU_SGL_ELEMENT_PAIR_A) {
		if (
			(this_request->type.pio.request_current.sgl_pair->B.address_lower == 0)
			&& (this_request->type.pio.request_current.sgl_pair->B.address_upper == 0)
			) {
			current_sgl = NULL;
		} else {
			this_request->type.pio.request_current.sgl_set = SCU_SGL_ELEMENT_PAIR_B;
			current_sgl = &(this_request->type.pio.request_current.sgl_pair->B);
		}
	} else {
		if (
			(this_request->type.pio.request_current.sgl_pair->next_pair_lower == 0)
			&& (this_request->type.pio.request_current.sgl_pair->next_pair_upper == 0)
			) {
			current_sgl = NULL;
		} else {
			dma_addr_t physical_address;

			sci_cb_make_physical_address(
				physical_address,
				this_request->type.pio.request_current.sgl_pair->next_pair_upper,
				this_request->type.pio.request_current.sgl_pair->next_pair_lower
				);

			this_request->type.pio.request_current.sgl_pair =
				(struct scu_sgl_element_pair *)scic_cb_get_virtual_address(
					this_request->parent.owning_controller,
					physical_address
					);

			this_request->type.pio.request_current.sgl_set = SCU_SGL_ELEMENT_PAIR_A;

			current_sgl = &(this_request->type.pio.request_current.sgl_pair->A);
		}
	}

	return current_sgl;
}

/**
 *
 * @scic_io_request: The core request object which is cast to a SATA PIO
 *    request object.
 *
 * This method will construct the SATA PIO request. This method returns an
 * indication as to whether the construction was successful. SCI_SUCCESS
 * Currently this method always returns this value.
 */
enum sci_status scic_sds_stp_pio_request_construct(
	struct scic_sds_request *scic_io_request,
	u8 sat_protocol,
	bool copy_rx_frame)
{
	struct scic_sds_stp_request *this_request;

	this_request = (struct scic_sds_stp_request *)scic_io_request;

	scic_sds_stp_non_ncq_request_construct(&this_request->parent);

	scu_stp_raw_request_construct_task_context(
		this_request, this_request->parent.task_context_buffer
		);

	this_request->type.pio.current_transfer_bytes = 0;
	this_request->type.pio.ending_error = 0;
	this_request->type.pio.ending_status = 0;

	this_request->type.pio.request_current.sgl_offset = 0;
	this_request->type.pio.request_current.sgl_set = SCU_SGL_ELEMENT_PAIR_A;
	this_request->type.pio.sat_protocol = sat_protocol;

	if (copy_rx_frame) {
		scic_sds_request_build_sgl(&this_request->parent);
		/*
		 * Since the IO request copy of the TC contains the same data as
		 * the actual TC this pointer is vaild for either. */
		this_request->type.pio.request_current.sgl_pair =
			&this_request->parent.task_context_buffer->sgl_pair_ab;
	} else {
		/* The user does not want the data copied to the SGL buffer location */
		this_request->type.pio.request_current.sgl_pair = NULL;
	}

	sci_base_state_machine_construct(
		&this_request->parent.started_substate_machine,
		&this_request->parent.parent.parent,
		scic_sds_stp_request_started_pio_substate_table,
		SCIC_SDS_STP_REQUEST_STARTED_PIO_AWAIT_H2D_COMPLETION_SUBSTATE
		);

	return SCI_SUCCESS;
}

