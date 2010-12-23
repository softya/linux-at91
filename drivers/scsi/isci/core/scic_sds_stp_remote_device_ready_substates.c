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
#include "scic_remote_device.h"
#include "scic_user_callback.h"
#include "scic_sds_controller.h"
#include "scic_sds_port.h"
#include "scic_sds_remote_device.h"

/*
 * *****************************************************************************
 * *  STP REMOTE DEVICE READY SUBSTATE PRIVATE METHODS
 * ***************************************************************************** */

static void scic_sds_stp_remote_device_ready_idle_substate_resume_complete_handler(
	void *user_cookie)
{
	struct scic_sds_remote_device *this_device;

	this_device = (struct scic_sds_remote_device *)user_cookie;

	/*
	 * For NCQ operation we do not issue a
	 * scic_cb_remote_device_not_ready().  As a result, avoid sending
	 * the ready notification. */
	if (this_device->ready_substate_machine.previous_state_id
	    != SCIC_SDS_STP_REMOTE_DEVICE_READY_SUBSTATE_NCQ) {
		scic_cb_remote_device_ready(
			scic_sds_remote_device_get_controller(this_device), this_device
			);
	}
}

/*
 * *****************************************************************************
 * *  STP REMOTE DEVICE READY IDLE SUBSTATE
 * ***************************************************************************** */

/**
 *
 * @device: This is the SCI base object which is cast into a
 *    struct scic_sds_remote_device object.
 *
 */
static void scic_sds_stp_remote_device_ready_idle_substate_enter(
	struct sci_base_object *device)
{
	struct scic_sds_remote_device *this_device;

	this_device = (struct scic_sds_remote_device *)device;

	SET_STATE_HANDLER(
		this_device,
		scic_sds_stp_remote_device_ready_substate_handler_table,
		SCIC_SDS_STP_REMOTE_DEVICE_READY_SUBSTATE_IDLE
		);

	this_device->working_request = NULL;

	if (scic_sds_remote_node_context_is_ready(this_device->rnc)) {
		/*
		 * Since the RNC is ready, it's alright to finish completion
		 * processing (e.g. signal the remote device is ready). */
		scic_sds_stp_remote_device_ready_idle_substate_resume_complete_handler(
			this_device
			);
	} else {
		scic_sds_remote_node_context_resume(
			this_device->rnc,
			scic_sds_stp_remote_device_ready_idle_substate_resume_complete_handler,
			this_device
			);
	}
}

/*
 * *****************************************************************************
 * *  STP REMOTE DEVICE READY CMD SUBSTATE
 * ***************************************************************************** */

/**
 *
 * @device: This is the SCI base object which is cast into a
 *    struct scic_sds_remote_device object.
 *
 */
static void scic_sds_stp_remote_device_ready_cmd_substate_enter(
	struct sci_base_object *device)
{
	struct scic_sds_remote_device *this_device;

	this_device = (struct scic_sds_remote_device *)device;

	ASSERT(this_device->working_request != NULL);

	SET_STATE_HANDLER(
		this_device,
		scic_sds_stp_remote_device_ready_substate_handler_table,
		SCIC_SDS_STP_REMOTE_DEVICE_READY_SUBSTATE_CMD
		);

	scic_cb_remote_device_not_ready(
		scic_sds_remote_device_get_controller(this_device),
		this_device,
		SCIC_REMOTE_DEVICE_NOT_READY_SATA_REQUEST_STARTED
		);
}

/*
 * *****************************************************************************
 * *  STP REMOTE DEVICE READY NCQ SUBSTATE
 * ***************************************************************************** */

/**
 *
 * @device: This is the SCI base object which is cast into a
 *    struct scic_sds_remote_device object.
 *
 */
static void scic_sds_stp_remote_device_ready_ncq_substate_enter(
	struct sci_base_object *device)
{
	struct scic_sds_remote_device *this_device;

	this_device = (struct scic_sds_remote_device *)device;

	SET_STATE_HANDLER(
		this_device,
		scic_sds_stp_remote_device_ready_substate_handler_table,
		SCIC_SDS_STP_REMOTE_DEVICE_READY_SUBSTATE_NCQ
		);
}

/*
 * *****************************************************************************
 * *  STP REMOTE DEVICE READY NCQ ERROR SUBSTATE
 * ***************************************************************************** */

/**
 *
 * @device: This is the SCI base object which is cast into a
 *    struct scic_sds_remote_device object.
 *
 */
static void scic_sds_stp_remote_device_ready_ncq_error_substate_enter(
	struct sci_base_object *device)
{
	struct scic_sds_remote_device *this_device;

	this_device = (struct scic_sds_remote_device *)device;

	SET_STATE_HANDLER(
		this_device,
		scic_sds_stp_remote_device_ready_substate_handler_table,
		SCIC_SDS_STP_REMOTE_DEVICE_READY_SUBSTATE_NCQ_ERROR
		);

	if (this_device->not_ready_reason ==
	    SCIC_REMOTE_DEVICE_NOT_READY_SATA_SDB_ERROR_FIS_RECEIVED) {
		scic_cb_remote_device_not_ready(
			scic_sds_remote_device_get_controller(this_device),
			this_device,
			this_device->not_ready_reason
			);
	}
}

/*
 * *****************************************************************************
 * *  STP REMOTE DEVICE READY AWAIT RESET SUBSTATE
 * ***************************************************************************** */

/**
 * The enter routine to READY AWAIT RESET substate.
 * @device: This is the SCI base object which is cast into a
 *    struct scic_sds_remote_device object.
 *
 */
static void scic_sds_stp_remote_device_ready_await_reset_substate_enter(
	struct sci_base_object *device)
{
	struct scic_sds_remote_device *this_device;

	this_device = (struct scic_sds_remote_device *)device;

	SET_STATE_HANDLER(
		this_device,
		scic_sds_stp_remote_device_ready_substate_handler_table,
		SCIC_SDS_STP_REMOTE_DEVICE_READY_SUBSTATE_AWAIT_RESET
		);
}

#if !defined(DISABLE_ATAPI)
/*
 * *****************************************************************************
 * *  STP REMOTE DEVICE READY ATAPI ERROR SUBSTATE
 * ***************************************************************************** */

/**
 * The enter routine to READY ATAPI ERROR substate.
 * @device: This is the SCI base object which is cast into a
 *    struct scic_sds_remote_device object.
 *
 */
void scic_sds_stp_remote_device_ready_atapi_error_substate_enter(
	struct sci_base_object *device)
{
	struct scic_sds_remote_device *this_device;

	this_device = (struct scic_sds_remote_device *)device;

	SET_STATE_HANDLER(
		this_device,
		scic_sds_stp_remote_device_ready_substate_handler_table,
		SCIC_SDS_STP_REMOTE_DEVICE_READY_SUBSTATE_ATAPI_ERROR
		);
}
#endif /* !defined(DISABLE_ATAPI) */

/* --------------------------------------------------------------------------- */

const struct sci_base_state scic_sds_stp_remote_device_ready_substate_table[] = {
	[SCIC_SDS_STP_REMOTE_DEVICE_READY_SUBSTATE_IDLE] = {
		.enter_state = scic_sds_stp_remote_device_ready_idle_substate_enter,
	},
	[SCIC_SDS_STP_REMOTE_DEVICE_READY_SUBSTATE_CMD] = {
		.enter_state = scic_sds_stp_remote_device_ready_cmd_substate_enter,
	},
	[SCIC_SDS_STP_REMOTE_DEVICE_READY_SUBSTATE_NCQ] = {
		.enter_state = scic_sds_stp_remote_device_ready_ncq_substate_enter,
	},
	[SCIC_SDS_STP_REMOTE_DEVICE_READY_SUBSTATE_NCQ_ERROR] = {
		.enter_state = scic_sds_stp_remote_device_ready_ncq_error_substate_enter,
	},
#if !defined(DISABLE_ATAPI)
	[SCIC_SDS_STP_REMOTE_DEVICE_READY_SUBSTATE_ATAPI_ERROR] = {
		.enter_state = scic_sds_stp_remote_device_ready_atapi_error_substate_enter,
	},
#endif
	[SCIC_SDS_STP_REMOTE_DEVICE_READY_SUBSTATE_AWAIT_RESET] = {
		.enter_state = scic_sds_stp_remote_device_ready_await_reset_substate_enter,
	},
};
