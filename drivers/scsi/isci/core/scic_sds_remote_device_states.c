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
 * This file contains the operations that are taken on the enter and exit state
 *    transitions for the struct sci_base_remote_device state machine.
 *
 *
 */

#include "scic_remote_device.h"
#include "scic_user_callback.h"
#include "scic_sds_controller.h"
#include "scic_sds_remote_device.h"
#include "scic_sds_request.h"
#include "scic_sds_controller.h"
#include "scic_sds_port.h"

/**
 *
 * @object: This is the struct sci_base_object that is cast into a
 *    struct scic_sds_remote_device.
 *
 * This is the enter method for the SCI_BASE_REMOTE_DEVICE_STATE_INITIAL it
 * immediatly transitions the remote device object to the stopped state. none
 */
static void scic_sds_remote_device_initial_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)object;

	SET_STATE_HANDLER(
		this_device,
		scic_sds_remote_device_state_handler_table,
		SCI_BASE_REMOTE_DEVICE_STATE_INITIAL
		);

	/* Initial state is a transitional state to the stopped state */
	sci_base_state_machine_change_state(
		scic_sds_remote_device_get_base_state_machine(this_device),
		SCI_BASE_REMOTE_DEVICE_STATE_STOPPED
		);
}

/**
 *
 * @object: This is the struct sci_base_object that is cast into a
 *    struct scic_sds_remote_device.
 *
 * This is the enter method for the SCI_BASE_REMOTE_DEVICE_STATE_INITIAL it
 * sets the stopped state handlers and if this state is entered from the
 * SCI_BASE_REMOTE_DEVICE_STATE_STOPPING then the SCI User is informed that the
 * device stop is complete. none
 */
static void scic_sds_remote_device_stopped_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)object;

	SET_STATE_HANDLER(
		this_device,
		scic_sds_remote_device_state_handler_table,
		SCI_BASE_REMOTE_DEVICE_STATE_STOPPED
		);

	/*
	 * If we are entering from the stopping state let the SCI User know that
	 * the stop operation has completed. */
	if (this_device->parent.state_machine.previous_state_id
	    == SCI_BASE_REMOTE_DEVICE_STATE_STOPPING) {
		scic_cb_remote_device_stop_complete(
			scic_sds_remote_device_get_controller(this_device),
			this_device,
			SCI_SUCCESS
			);
	}
}

/**
 *
 * @object: This is the struct sci_base_object that is cast into a
 *    struct scic_sds_remote_device.
 *
 * This is the enter method for the SCI_BASE_REMOTE_DEVICE_STATE_STARTING it
 * sets the starting state handlers, sets the device not ready, and posts the
 * remote node context to the hardware. none
 */
static void scic_sds_remote_device_starting_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_controller *the_controller;
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)object;

	the_controller = scic_sds_remote_device_get_controller(this_device);

	SET_STATE_HANDLER(
		this_device,
		scic_sds_remote_device_state_handler_table,
		SCI_BASE_REMOTE_DEVICE_STATE_STARTING
		);

	scic_cb_remote_device_not_ready(
		the_controller,
		this_device,
		SCIC_REMOTE_DEVICE_NOT_READY_START_REQUESTED
		);
}

/**
 *
 * @object: This is the struct sci_base_object that is cast into a
 *    struct scic_sds_remote_device.
 *
 * This is the exit method for the SCI_BASE_REMOTE_DEVICE_STATE_STARTING it
 * reports that the device start is complete. none
 */
static void scic_sds_remote_device_starting_state_exit(
	struct sci_base_object *object)
{
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)object;

	/*
	 * / @todo Check the device object for the proper return code for this
	 * /       callback */
	scic_cb_remote_device_start_complete(
		scic_sds_remote_device_get_controller(this_device),
		this_device,
		SCI_SUCCESS
		);
}

/**
 *
 * @object: This is the struct sci_base_object that is cast into a
 *    struct scic_sds_remote_device.
 *
 * This is the enter method for the SCI_BASE_REMOTE_DEVICE_STATE_READY it sets
 * the ready state handlers, and starts the ready substate machine. none
 */
static void scic_sds_remote_device_ready_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_controller *the_controller;
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)object;

	the_controller = scic_sds_remote_device_get_controller(this_device);

	SET_STATE_HANDLER(
		this_device,
		scic_sds_remote_device_state_handler_table,
		SCI_BASE_REMOTE_DEVICE_STATE_READY
		);

	the_controller->remote_device_sequence[this_device->rnc->remote_node_index]++;

	if (this_device->has_ready_substate_machine) {
		sci_base_state_machine_start(&this_device->ready_substate_machine);
	} else {
		scic_cb_remote_device_ready(the_controller, this_device);
	}
}

/**
 *
 * @object: This is the struct sci_base_object that is cast into a
 *    struct scic_sds_remote_device.
 *
 * This is the exit method for the SCI_BASE_REMOTE_DEVICE_STATE_READY it does
 * nothing. none
 */
static void scic_sds_remote_device_ready_state_exit(
	struct sci_base_object *object)
{
	struct scic_sds_controller *the_controller;
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)object;

	the_controller = scic_sds_remote_device_get_controller(this_device);

	if (this_device->has_ready_substate_machine) {
		sci_base_state_machine_stop(&this_device->ready_substate_machine);
	} else {
		scic_cb_remote_device_not_ready(
			the_controller,
			this_device,
			SCIC_REMOTE_DEVICE_NOT_READY_STOP_REQUESTED
			);
	}
}

/**
 *
 * @object: This is the struct sci_base_object that is cast into a
 *    struct scic_sds_remote_device.
 *
 * This is the enter method for the SCI_BASE_REMOTE_DEVICE_STATE_STOPPING it
 * sets the stopping state handlers and posts an RNC invalidate request to the
 * SCU hardware. none
 */
static void scic_sds_remote_device_stopping_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)object;

	SET_STATE_HANDLER(
		this_device,
		scic_sds_remote_device_state_handler_table,
		SCI_BASE_REMOTE_DEVICE_STATE_STOPPING
		);
}

/**
 *
 * @object: This is the struct sci_base_object that is cast into a
 *    struct scic_sds_remote_device.
 *
 * This is the enter method for the SCI_BASE_REMOTE_DEVICE_STATE_FAILED it sets
 * the stopping state handlers. none
 */
static void scic_sds_remote_device_failed_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)object;

	SET_STATE_HANDLER(
		this_device,
		scic_sds_remote_device_state_handler_table,
		SCI_BASE_REMOTE_DEVICE_STATE_FAILED
		);
}

/**
 *
 * @object: This is the struct sci_base_object that is cast into a
 *    struct scic_sds_remote_device.
 *
 * This is the enter method for the SCI_BASE_REMOTE_DEVICE_STATE_RESETTING it
 * sets the resetting state handlers. none
 */
static void scic_sds_remote_device_resetting_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)object;

	SET_STATE_HANDLER(
		this_device,
		scic_sds_remote_device_state_handler_table,
		SCI_BASE_REMOTE_DEVICE_STATE_RESETTING
		);

	scic_sds_remote_node_context_suspend(
		this_device->rnc, SCI_SOFTWARE_SUSPENSION, NULL, NULL);
}

/**
 *
 * @object: This is the struct sci_base_object that is cast into a
 *    struct scic_sds_remote_device.
 *
 * This is the exit method for the SCI_BASE_REMOTE_DEVICE_STATE_RESETTING it
 * does nothing. none
 */
static void scic_sds_remote_device_resetting_state_exit(
	struct sci_base_object *object)
{
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)object;

	scic_sds_port_set_direct_attached_device_id(
		scic_sds_remote_device_get_port(this_device),
		this_device->rnc->remote_node_index
		);

	scic_sds_remote_node_context_resume(this_device->rnc, NULL, NULL);
}

/**
 *
 * @object: This is the struct sci_base_object that is cast into a
 *    struct scic_sds_remote_device.
 *
 * This is the enter method for the SCI_BASE_REMOTE_DEVICE_STATE_FINAL it sets
 * the final state handlers. none
 */
static void scic_sds_remote_device_final_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)object;

	SET_STATE_HANDLER(
		this_device,
		scic_sds_remote_device_state_handler_table,
		SCI_BASE_REMOTE_DEVICE_STATE_FINAL
		);
}

/* --------------------------------------------------------------------------- */

const struct sci_base_state scic_sds_remote_device_state_table[] = {
	[SCI_BASE_REMOTE_DEVICE_STATE_INITIAL] = {
		.enter_state = scic_sds_remote_device_initial_state_enter,
	},
	[SCI_BASE_REMOTE_DEVICE_STATE_STOPPED] = {
		.enter_state = scic_sds_remote_device_stopped_state_enter,
	},
	[SCI_BASE_REMOTE_DEVICE_STATE_STARTING] = {
		.enter_state = scic_sds_remote_device_starting_state_enter,
		.exit_state  = scic_sds_remote_device_starting_state_exit
	},
	[SCI_BASE_REMOTE_DEVICE_STATE_READY] = {
		.enter_state = scic_sds_remote_device_ready_state_enter,
		.exit_state  = scic_sds_remote_device_ready_state_exit
	},
	[SCI_BASE_REMOTE_DEVICE_STATE_STOPPING] = {
		.enter_state = scic_sds_remote_device_stopping_state_enter,
	},
	[SCI_BASE_REMOTE_DEVICE_STATE_FAILED] = {
		.enter_state = scic_sds_remote_device_failed_state_enter,
	},
	[SCI_BASE_REMOTE_DEVICE_STATE_RESETTING] = {
		.enter_state = scic_sds_remote_device_resetting_state_enter,
		.exit_state  = scic_sds_remote_device_resetting_state_exit
	},
	[SCI_BASE_REMOTE_DEVICE_STATE_FINAL] = {
		.enter_state = scic_sds_remote_device_final_state_enter,
	},
};

