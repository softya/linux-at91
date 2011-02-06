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
 * This file contains the implementation of the core io request state machine.
 *
 *
 */

#include "scic_user_callback.h"
#include "scic_sds_request.h"
#include "scu_completion_codes.h"


/**
 * scic_sds_request_initial_state_enter() -
 * @object: This parameter specifies the base object for which the state
 *    transition is occurring.
 *
 * This method implements the actions taken when entering the
 * SCI_BASE_REQUEST_STATE_INITIAL state. This state is entered when the initial
 * base request is constructed. Entry into the initial state sets all handlers
 * for the io request object to their default handlers. none
 */
static void scic_sds_request_initial_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)object;

	SET_STATE_HANDLER(
		this_request,
		scic_sds_request_state_handler_table,
		SCI_BASE_REQUEST_STATE_INITIAL
		);
}

/**
 * scic_sds_request_constructed_state_enter() -
 * @object: The io request object that is to enter the constructed state.
 *
 * This method implements the actions taken when entering the
 * SCI_BASE_REQUEST_STATE_CONSTRUCTED state. The method sets the state handlers
 * for the the constructed state. none
 */
static void scic_sds_request_constructed_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)object;

	SET_STATE_HANDLER(
		this_request,
		scic_sds_request_state_handler_table,
		SCI_BASE_REQUEST_STATE_CONSTRUCTED
		);
}

/**
 * scic_sds_request_started_state_enter() -
 * @object: This parameter specifies the base object for which the state
 *    transition is occuring.  This is cast into a SCIC_SDS_IO_REQUEST object.
 *
 * This method implements the actions taken when entering the
 * SCI_BASE_REQUEST_STATE_STARTED state. If the io request object type is a
 * SCSI Task request we must enter the started substate machine. none
 */
static void scic_sds_request_started_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)object;

	SET_STATE_HANDLER(
		this_request,
		scic_sds_request_state_handler_table,
		SCI_BASE_REQUEST_STATE_STARTED
		);

	/*
	 * Most of the request state machines have a started substate machine so
	 * start its execution on the entry to the started state. */
	if (this_request->has_started_substate_machine == true)
		sci_base_state_machine_start(&this_request->started_substate_machine);
}

/**
 * scic_sds_request_started_state_exit() -
 * @object: This parameter specifies the base object for which the state
 *    transition is occuring.  This object is cast into a SCIC_SDS_IO_REQUEST
 *    object.
 *
 * This method implements the actions taken when exiting the
 * SCI_BASE_REQUEST_STATE_STARTED state. For task requests the action will be
 * to stop the started substate machine. none
 */
static void scic_sds_request_started_state_exit(
	struct sci_base_object *object)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)object;

	if (this_request->has_started_substate_machine == true)
		sci_base_state_machine_stop(&this_request->started_substate_machine);
}

/**
 * scic_sds_request_completed_state_enter() -
 * @object: This parameter specifies the base object for which the state
 *    transition is occuring.  This object is cast into a SCIC_SDS_IO_REQUEST
 *    object.
 *
 * This method implements the actions taken when entering the
 * SCI_BASE_REQUEST_STATE_COMPLETED state.  This state is entered when the
 * SCIC_SDS_IO_REQUEST has completed.  The method will decode the request
 * completion status and convert it to an enum sci_status to return in the
 * completion callback function. none
 */
static void scic_sds_request_completed_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)object;

	SET_STATE_HANDLER(
		this_request,
		scic_sds_request_state_handler_table,
		SCI_BASE_REQUEST_STATE_COMPLETED
		);

	/* Tell the SCI_USER that the IO request is complete */
	if (this_request->is_task_management_request == false) {
		scic_cb_io_request_complete(
			scic_sds_request_get_controller(this_request),
			scic_sds_request_get_device(this_request),
			this_request,
			this_request->sci_status
			);
	} else {
		scic_cb_task_request_complete(
			scic_sds_request_get_controller(this_request),
			scic_sds_request_get_device(this_request),
			this_request,
			this_request->sci_status
			);
	}
}

/**
 * scic_sds_request_aborting_state_enter() -
 * @object: This parameter specifies the base object for which the state
 *    transition is occuring.  This object is cast into a SCIC_SDS_IO_REQUEST
 *    object.
 *
 * This method implements the actions taken when entering the
 * SCI_BASE_REQUEST_STATE_ABORTING state. none
 */
static void scic_sds_request_aborting_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)object;

	/* Setting the abort bit in the Task Context is required by the silicon. */
	this_request->task_context_buffer->abort = 1;

	SET_STATE_HANDLER(
		this_request,
		scic_sds_request_state_handler_table,
		SCI_BASE_REQUEST_STATE_ABORTING
		);
}

/**
 * scic_sds_request_final_state_enter() -
 * @object: This parameter specifies the base object for which the state
 *    transition is occuring.  This is cast into a SCIC_SDS_IO_REQUEST object.
 *
 * This method implements the actions taken when entering the
 * SCI_BASE_REQUEST_STATE_FINAL state. The only action required is to put the
 * state handlers in place. none
 */
static void scic_sds_request_final_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_request *this_request = (struct scic_sds_request *)object;

	SET_STATE_HANDLER(
		this_request,
		scic_sds_request_state_handler_table,
		SCI_BASE_REQUEST_STATE_FINAL
		);
}

/* --------------------------------------------------------------------------- */

struct sci_base_state
	scic_sds_request_state_table[SCI_BASE_REQUEST_MAX_STATES] =
{
	{
		SCI_BASE_REQUEST_STATE_INITIAL,
		scic_sds_request_initial_state_enter,
		NULL
	},
	{
		SCI_BASE_REQUEST_STATE_CONSTRUCTED,
		scic_sds_request_constructed_state_enter,
		NULL
	},
	{
		SCI_BASE_REQUEST_STATE_STARTED,
		scic_sds_request_started_state_enter,
		scic_sds_request_started_state_exit
	},
	{
		SCI_BASE_REQUEST_STATE_COMPLETED,
		scic_sds_request_completed_state_enter,
		NULL
	},
	{
		SCI_BASE_REQUEST_STATE_ABORTING,
		scic_sds_request_aborting_state_enter,
		NULL
	},
	{
		SCI_BASE_REQUEST_STATE_FINAL,
		scic_sds_request_final_state_enter,
		NULL
	}
};

