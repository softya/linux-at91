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

#include "sci_base_state.h"
#include "scic_sds_remote_device.h"
#include "scic_sds_stp_request.h"
#include "scic_sds_stp_pio_request.h"

/**
 * This file contains the SATA PIO Data In/Out and started substate machine for
 *    the SCIC_SDS_IO_REQUEST object.
 *
 *
 */

static void scic_sds_stp_request_started_pio_await_h2d_completion_enter(
	struct sci_base_object *object)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)object;

	SET_STATE_HANDLER(
		this_request,
		scic_sds_stp_request_started_pio_substate_handler_table,
		SCIC_SDS_STP_REQUEST_STARTED_PIO_AWAIT_H2D_COMPLETION_SUBSTATE
		);

	scic_sds_remote_device_set_working_request(
		this_request->target_device, this_request);
}

static void scic_sds_stp_request_started_pio_await_frame_enter(
	struct sci_base_object *object)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)object;

	SET_STATE_HANDLER(
		this_request,
		scic_sds_stp_request_started_pio_substate_handler_table,
		SCIC_SDS_STP_REQUEST_STARTED_PIO_AWAIT_FRAME_SUBSTATE
		);
}

static void scic_sds_stp_request_started_pio_data_in_await_data_enter(
	struct sci_base_object *object)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)object;

	SET_STATE_HANDLER(
		this_request,
		scic_sds_stp_request_started_pio_substate_handler_table,
		SCIC_SDS_STP_REQUEST_STARTED_PIO_DATA_IN_AWAIT_DATA_SUBSTATE
		);
}

static void scic_sds_stp_request_started_pio_data_out_transmit_data_enter(
	struct sci_base_object *object)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)object;

	SET_STATE_HANDLER(
		this_request,
		scic_sds_stp_request_started_pio_substate_handler_table,
		SCIC_SDS_STP_REQUEST_STARTED_PIO_DATA_OUT_TRANSMIT_DATA_SUBSTATE
		);
}

/* --------------------------------------------------------------------------- */

const struct sci_base_state scic_sds_stp_request_started_pio_substate_table[] = {
	[SCIC_SDS_STP_REQUEST_STARTED_PIO_AWAIT_H2D_COMPLETION_SUBSTATE] = {
		.enter_state = scic_sds_stp_request_started_pio_await_h2d_completion_enter,
	},
	[SCIC_SDS_STP_REQUEST_STARTED_PIO_AWAIT_FRAME_SUBSTATE] = {
		.enter_state = scic_sds_stp_request_started_pio_await_frame_enter,
	},
	[SCIC_SDS_STP_REQUEST_STARTED_PIO_DATA_IN_AWAIT_DATA_SUBSTATE] = {
		.enter_state = scic_sds_stp_request_started_pio_data_in_await_data_enter,
	},
	[SCIC_SDS_STP_REQUEST_STARTED_PIO_DATA_OUT_TRANSMIT_DATA_SUBSTATE] = {
		.enter_state = scic_sds_stp_request_started_pio_data_out_transmit_data_enter,
	}
};
