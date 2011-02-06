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
 * This file contains the implementation of the struct scic_sds_port base state
 *    machine.
 *
 *
 */

#include "scic_port.h"
#include "scic_user_callback.h"
#include "scic_sds_controller.h"
#include "scic_sds_port.h"
#include "scic_sds_port_registers.h"
#include "scic_sds_remote_node_context.h"

/*
 * ******************************************************************************
 * *  PORT STATE PRIVATE METHODS
 * ****************************************************************************** */

/**
 *
 * @this_port: This is the port object which to suspend.
 *
 * This method will enable the SCU Port Task Scheduler for this port object but
 * will leave the port task scheduler in a suspended state. none
 */
static void scic_sds_port_enable_port_task_scheduler(
	struct scic_sds_port *this_port)
{
	u32 pts_control_value;

	pts_control_value = scu_port_task_scheduler_read(this_port, control);

	pts_control_value |= SCU_PTSxCR_GEN_BIT(ENABLE) | SCU_PTSxCR_GEN_BIT(SUSPEND);

	scu_port_task_scheduler_write(this_port, control, pts_control_value);
}

/**
 *
 * @this_port: This is the port object which to resume.
 *
 * This method will disable the SCU port task scheduler for this port object.
 * none
 */
static void scic_sds_port_disable_port_task_scheduler(
	struct scic_sds_port *this_port)
{
	u32 pts_control_value;

	pts_control_value = scu_port_task_scheduler_read(this_port, control);

	pts_control_value &= ~(SCU_PTSxCR_GEN_BIT(ENABLE)
			       | SCU_PTSxCR_GEN_BIT(SUSPEND));

	scu_port_task_scheduler_write(this_port, control, pts_control_value);
}

/*
 * ******************************************************************************
 * *  PORT STATE METHODS
 * ****************************************************************************** */

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_port object.
 *
 * This method will perform the actions required by the struct scic_sds_port on
 * entering the SCI_BASE_PORT_STATE_STOPPED. This function sets the stopped
 * state handlers for the struct scic_sds_port object and disables the port task
 * scheduler in the hardware. none
 */
static void scic_sds_port_stopped_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_port *this_port;

	this_port = (struct scic_sds_port *)object;

	scic_sds_port_set_base_state_handlers(
		this_port, SCI_BASE_PORT_STATE_STOPPED
		);

	if (
		SCI_BASE_PORT_STATE_STOPPING
		== this_port->parent.state_machine.previous_state_id
		) {
		/*
		 * If we enter this state becasuse of a request to stop
		 * the port then we want to disable the hardwares port
		 * task scheduler. */
		scic_sds_port_disable_port_task_scheduler(this_port);
	}
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_port object.
 *
 * This method will perform the actions required by the struct scic_sds_port on
 * exiting the SCI_BASE_STATE_STOPPED. This function enables the SCU hardware
 * port task scheduler. none
 */
static void scic_sds_port_stopped_state_exit(
	struct sci_base_object *object)
{
	struct scic_sds_port *this_port;

	this_port = (struct scic_sds_port *)object;

	/* Enable and suspend the port task scheduler */
	scic_sds_port_enable_port_task_scheduler(this_port);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_port object.
 *
 * This method will perform the actions required by the struct scic_sds_port on
 * entering the SCI_BASE_PORT_STATE_READY. This function sets the ready state
 * handlers for the struct scic_sds_port object, reports the port object as not ready
 * and starts the ready substate machine. none
 */
static void scic_sds_port_ready_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_port *this_port;

	this_port = (struct scic_sds_port *)object;

	/* Put the ready state handlers in place though they will not be there long */
	scic_sds_port_set_base_state_handlers(
		this_port, SCI_BASE_PORT_STATE_READY
		);

	if (
		SCI_BASE_PORT_STATE_RESETTING
		== this_port->parent.state_machine.previous_state_id
		) {
		scic_cb_port_hard_reset_complete(
			scic_sds_port_get_controller(this_port),
			this_port,
			SCI_SUCCESS
			);
	} else {
		/* Notify the caller that the port is not yet ready */
		scic_cb_port_not_ready(
			scic_sds_port_get_controller(this_port),
			this_port,
			SCIC_PORT_NOT_READY_NO_ACTIVE_PHYS
			);
	}

	/* Start the ready substate machine */
	sci_base_state_machine_start(
		scic_sds_port_get_ready_substate_machine(this_port)
		);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_port object.
 *
 * This method will perform the actions required by the struct scic_sds_port on
 * exiting the SCI_BASE_STATE_READY. This function does nothing. none
 */
static void scic_sds_port_ready_state_exit(
	struct sci_base_object *object)
{
	struct scic_sds_port *this_port;

	this_port = (struct scic_sds_port *)object;

	sci_base_state_machine_stop(&this_port->ready_substate_machine);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_port object.
 *
 * This method will perform the actions required by the struct scic_sds_port on
 * entering the SCI_BASE_PORT_STATE_RESETTING. This function sets the resetting
 * state handlers for the struct scic_sds_port object. none
 */
static void scic_sds_port_resetting_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_port *this_port;

	this_port = (struct scic_sds_port *)object;

	scic_sds_port_set_base_state_handlers(
		this_port, SCI_BASE_PORT_STATE_RESETTING
		);

	scic_sds_port_set_direct_attached_device_id(
		this_port,
		SCIC_SDS_REMOTE_NODE_CONTEXT_INVALID_INDEX
		);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_port object.
 *
 * This method will perform the actions required by the struct scic_sds_port on
 * exiting the SCI_BASE_STATE_RESETTING. This function does nothing. none
 */
static void scic_sds_port_resetting_state_exit(
	struct sci_base_object *object)
{
	struct scic_sds_port *this_port;

	this_port = (struct scic_sds_port *)object;

	scic_cb_timer_stop(
		scic_sds_port_get_controller(this_port),
		this_port->timer_handle
		);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_port object.
 *
 * This method will perform the actions required by the struct scic_sds_port on
 * entering the SCI_BASE_PORT_STATE_STOPPING. This function sets the stopping
 * state handlers for the struct scic_sds_port object. none
 */
static void scic_sds_port_stopping_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_port *this_port;

	this_port = (struct scic_sds_port *)object;

	scic_sds_port_set_base_state_handlers(
		this_port, SCI_BASE_PORT_STATE_STOPPING
		);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_port object.
 *
 * This method will perform the actions required by the struct scic_sds_port on
 * exiting the SCI_BASE_STATE_STOPPING. This function does nothing. none
 */
static void scic_sds_port_stopping_state_exit(
	struct sci_base_object *object)
{
	struct scic_sds_port *this_port;

	this_port = (struct scic_sds_port *)object;

	scic_cb_timer_stop(
		scic_sds_port_get_controller(this_port),
		this_port->timer_handle
		);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_port object.
 *
 * This method will perform the actions required by the struct scic_sds_port on
 * entering the SCI_BASE_PORT_STATE_STOPPING. This function sets the stopping
 * state handlers for the struct scic_sds_port object. none
 */
static void scic_sds_port_failed_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_port *this_port;

	this_port = (struct scic_sds_port *)object;

	scic_sds_port_set_base_state_handlers(
		this_port,
		SCI_BASE_PORT_STATE_FAILED
		);

	scic_cb_port_hard_reset_complete(
		scic_sds_port_get_controller(this_port),
		this_port,
		SCI_FAILURE_TIMEOUT
		);
}

/* --------------------------------------------------------------------------- */

struct sci_base_state scic_sds_port_state_table[SCI_BASE_PORT_MAX_STATES] =
{
	{
		SCI_BASE_PORT_STATE_STOPPED,
		scic_sds_port_stopped_state_enter,
		scic_sds_port_stopped_state_exit
	},
	{
		SCI_BASE_PORT_STATE_STOPPING,
		scic_sds_port_stopping_state_enter,
		scic_sds_port_stopping_state_exit
	},
	{
		SCI_BASE_PORT_STATE_READY,
		scic_sds_port_ready_state_enter,
		scic_sds_port_ready_state_exit
	},
	{
		SCI_BASE_PORT_STATE_RESETTING,
		scic_sds_port_resetting_state_enter,
		scic_sds_port_resetting_state_exit
	},
	{
		SCI_BASE_PORT_STATE_FAILED,
		scic_sds_port_failed_state_enter,
		NULL
	}
};

