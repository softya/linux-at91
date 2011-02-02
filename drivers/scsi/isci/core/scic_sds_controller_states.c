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
 * This file contains the enter/exit state function implementation for the
 *    struct scic_sds_controller base state machine.
 *
 *
 */

#include "scic_user_callback.h"
#include "scic_controller.h"
#include "scic_sds_controller.h"
#include "scic_sds_controller.h"
#include "scic_sds_controller_registers.h"

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_controller
 *    object.
 *
 * This method implements the actions taken by the struct scic_sds_controller on entry
 * to the SCI_BASE_CONTROLLER_STATE_INITIAL. - Set the state handlers to the
 * controllers initial state. none This function should initialze the
 * controller object.
 */
static void scic_sds_controller_initial_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_controller *this_controller;

	this_controller = (struct scic_sds_controller *)object;

	sci_base_state_machine_change_state(
		&this_controller->parent.state_machine, SCI_BASE_CONTROLLER_STATE_RESET);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_controller
 *    object.
 *
 * This method implements the actions taken by the struct scic_sds_controller on exit
 * from the SCI_BASE_CONTROLLER_STATE_STARTING. - This function stops the
 * controller starting timeout timer. none
 */
static void scic_sds_controller_starting_state_exit(
	struct sci_base_object *object)
{
	struct scic_sds_controller *this_controller;

	this_controller = (struct scic_sds_controller *)object;

	scic_cb_timer_stop(object, this_controller->timeout_timer);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_controller
 *    object.
 *
 * This method implements the actions taken by the struct scic_sds_controller on entry
 * to the SCI_BASE_CONTROLLER_STATE_READY. - Set the state handlers to the
 * controllers ready state. none
 */
static void scic_sds_controller_ready_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_controller *this_controller;

	this_controller = (struct scic_sds_controller *)object;

	/* set the default interrupt coalescence number and timeout value. */
	scic_controller_set_interrupt_coalescence(
		this_controller, 0x10, 250);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_controller
 *    object.
 *
 * This method implements the actions taken by the struct scic_sds_controller on exit
 * from the SCI_BASE_CONTROLLER_STATE_READY. - This function does nothing. none
 */
static void scic_sds_controller_ready_state_exit(
	struct sci_base_object *object)
{
	struct scic_sds_controller *this_controller;

	this_controller = (struct scic_sds_controller *)object;

	/* disable interrupt coalescence. */
	scic_controller_set_interrupt_coalescence(this_controller, 0, 0);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_controller
 *    object.
 *
 * This method implements the actions taken by the struct scic_sds_controller on entry
 * to the SCI_BASE_CONTROLLER_STATE_READY. - Set the state handlers to the
 * controllers ready state. - Stop the phys on this controller - Stop the ports
 * on this controller - Stop all of the remote devices on this controller none
 */
static void scic_sds_controller_stopping_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_controller *this_controller;

	this_controller = (struct scic_sds_controller *)object;

	/* Stop all of the components for this controller */
	scic_sds_controller_stop_phys(this_controller);
	scic_sds_controller_stop_ports(this_controller);
	scic_sds_controller_stop_devices(this_controller);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_controller
 *    object.
 *
 * This method implements the actions taken by the struct scic_sds_controller on exit
 * from the SCI_BASE_CONTROLLER_STATE_STOPPING. - This function stops the
 * controller stopping timeout timer. none
 */
static void scic_sds_controller_stopping_state_exit(
	struct sci_base_object *object)
{
	struct scic_sds_controller *this_controller;

	this_controller = (struct scic_sds_controller *)object;

	scic_cb_timer_stop(this_controller, this_controller->timeout_timer);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_controller
 *    object.
 *
 * This method implements the actions taken by the struct scic_sds_controller on entry
 * to the SCI_BASE_CONTROLLER_STATE_RESETTING. - Set the state handlers to the
 * controllers resetting state. - Write to the SCU hardware reset register to
 * force a reset - Transition to the SCI_BASE_CONTROLLER_STATE_RESET none
 */
static void scic_sds_controller_resetting_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_controller *this_controller;

	this_controller = (struct scic_sds_controller *)object;

	scic_sds_controller_reset_hardware(this_controller);

	sci_base_state_machine_change_state(
		scic_sds_controller_get_base_state_machine(this_controller),
		SCI_BASE_CONTROLLER_STATE_RESET
		);
}

/* --------------------------------------------------------------------------- */

const struct sci_base_state scic_sds_controller_state_table[] = {
	[SCI_BASE_CONTROLLER_STATE_INITIAL] = {
		.enter_state = scic_sds_controller_initial_state_enter,
	},
	[SCI_BASE_CONTROLLER_STATE_RESET] = {},
	[SCI_BASE_CONTROLLER_STATE_INITIALIZING] = {},
	[SCI_BASE_CONTROLLER_STATE_INITIALIZED] = {},
	[SCI_BASE_CONTROLLER_STATE_STARTING] = {
		.exit_state  = scic_sds_controller_starting_state_exit,
	},
	[SCI_BASE_CONTROLLER_STATE_READY] = {
		.enter_state = scic_sds_controller_ready_state_enter,
		.exit_state  = scic_sds_controller_ready_state_exit,
	},
	[SCI_BASE_CONTROLLER_STATE_RESETTING] = {
		.enter_state = scic_sds_controller_resetting_state_enter,
	},
	[SCI_BASE_CONTROLLER_STATE_STOPPING] = {
		.enter_state = scic_sds_controller_stopping_state_enter,
		.exit_state = scic_sds_controller_stopping_state_exit,
	},
	[SCI_BASE_CONTROLLER_STATE_STOPPED] = {},
	[SCI_BASE_CONTROLLER_STATE_FAILED] = {}
};

