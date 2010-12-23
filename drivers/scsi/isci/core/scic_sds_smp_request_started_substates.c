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
 * This file contains the enter/exit methods associated with each of the SMP
 *    request states.  For more information on the SMP request state machine
 *    please refer to scic_sds_io_request.h
 *
 *
 */

#include "scic_sds_request.h"
#include "sci_base_state_machine.h"
#include "scic_sds_remote_device.h"

/**
 * This method performs the actions required when entering the
 *    SCIC_SDS_SMP_REQUEST_STARTED_SUBSTATE_AWAIT_TC_RESPONSE sub-state. This
 *    includes setting the IO request state handlers for this sub-state.
 * @object: This parameter specifies the request object for which the sub-state
 *    change is occuring.
 *
 * none.
 */
static void scic_sds_smp_request_started_await_response_substate_enter(
	struct sci_base_object *object)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)object;

	SET_STATE_HANDLER(
		this_request,
		scic_sds_smp_request_started_substate_handler_table,
		SCIC_SDS_SMP_REQUEST_STARTED_SUBSTATE_AWAIT_RESPONSE
		);
}

/**
 * This method performs the actions required when entering the
 *    SCIC_SDS_SMP_REQUEST_STARTED_SUBSTATE_AWAIT_TC_COMPLETION sub-state.
 *    This includes setting the SMP request state handlers for this sub-state.
 * @object: This parameter specifies the request object for which the sub-state
 *    change is occuring.
 *
 * none.
 */
static void scic_sds_smp_request_started_await_tc_completion_substate_enter(
	struct sci_base_object *object)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)object;

	SET_STATE_HANDLER(
		this_request,
		scic_sds_smp_request_started_substate_handler_table,
		SCIC_SDS_SMP_REQUEST_STARTED_SUBSTATE_AWAIT_TC_COMPLETION
		);
}

const struct sci_base_state scic_sds_smp_request_started_substate_table[] = {
	[SCIC_SDS_SMP_REQUEST_STARTED_SUBSTATE_AWAIT_RESPONSE] = {
		.enter_state = scic_sds_smp_request_started_await_response_substate_enter,
	},
	[SCIC_SDS_SMP_REQUEST_STARTED_SUBSTATE_AWAIT_TC_COMPLETION] = {
		.enter_state = scic_sds_smp_request_started_await_tc_completion_substate_enter,
	},
};

