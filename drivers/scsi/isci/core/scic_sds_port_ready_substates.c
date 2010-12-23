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
 * This file has the implementation for the struct scic_sds_port ready substate
 *    machine.
 *
 *
 */

#include "scic_phy.h"
#include "scic_port.h"
#include "scic_user_callback.h"
#include "scic_sds_controller.h"
#include "scic_sds_port.h"
#include "scic_sds_phy.h"
#include "scic_sds_port_registers.h"

/**
 * scic_sds_port_set_ready_state_handlers() -
 *
 * This macro sets the port ready substate handlers.
 */
#define scic_sds_port_set_ready_state_handlers(port, state_id) \
	scic_sds_port_set_state_handlers(\
		port, &scic_sds_port_ready_substate_handler_table[(state_id)] \
		)

/*
 * ******************************************************************************
 * *  PORT STATE PRIVATE METHODS
 * ****************************************************************************** */

/**
 *
 * @this_port: This is the struct scic_sds_port object to suspend.
 *
 * This method will susped the port task scheduler for this port object. none
 */
static void scic_sds_port_suspend_port_task_scheduler(
	struct scic_sds_port *this_port)
{
	u32 pts_control_value;
	u32 tl_control_value;

	pts_control_value = scu_port_task_scheduler_read(this_port, control);
	tl_control_value = scu_transport_layer_read(this_port, control);

	pts_control_value |= SCU_PTSxCR_GEN_BIT(SUSPEND);
	tl_control_value  |= SCU_TLCR_GEN_BIT(CLEAR_TCI_NCQ_MAPPING_TABLE);

	scu_port_task_scheduler_write(this_port, control, pts_control_value);
	scu_transport_layer_write(this_port, control, tl_control_value);
}

/**
 *
 * @this_port: This is the struct scic_sds_port object to resume.
 *
 * This method will resume the port task scheduler for this port object. none
 */
static void scic_sds_port_resume_port_task_scheduler(
	struct scic_sds_port *this_port)
{
	u32 pts_control_value;

	pts_control_value = scu_port_task_scheduler_read(this_port, control);

	pts_control_value &= ~SCU_PTSxCR_GEN_BIT(SUSPEND);

	scu_port_task_scheduler_write(this_port, control, pts_control_value);
}

/*
 * ******************************************************************************
 * *  PORT READY SUBSTATE METHODS
 * ****************************************************************************** */

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_port object.
 *
 * This method will perform the actions required by the struct scic_sds_port on
 * entering the SCIC_SDS_PORT_READY_SUBSTATE_WAITING. This function checks the
 * port for any ready phys.  If there is at least one phy in a ready state then
 * the port transitions to the ready operational substate. none
 */
static void scic_sds_port_ready_substate_waiting_enter(
	struct sci_base_object *object)
{
	struct scic_sds_port *this_port = (struct scic_sds_port *)object;

	scic_sds_port_set_ready_state_handlers(
		this_port, SCIC_SDS_PORT_READY_SUBSTATE_WAITING
		);

	scic_sds_port_suspend_port_task_scheduler(this_port);

	this_port->not_ready_reason = SCIC_PORT_NOT_READY_NO_ACTIVE_PHYS;

	if (this_port->active_phy_mask != 0) {
		/* At least one of the phys on the port is ready */
		sci_base_state_machine_change_state(
			&this_port->ready_substate_machine,
			SCIC_SDS_PORT_READY_SUBSTATE_OPERATIONAL
			);
	}
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_port object.
 *
 * This method will perform the actions required by the struct scic_sds_port on
 * entering the SCIC_SDS_PORT_READY_SUBSTATE_OPERATIONAL. This function sets
 * the state handlers for the port object, notifies the SCI User that the port
 * is ready, and resumes port operations. none
 */
static void scic_sds_port_ready_substate_operational_enter(
	struct sci_base_object *object)
{
	u32 index;
	struct scic_sds_port *this_port = (struct scic_sds_port *)object;

	scic_sds_port_set_ready_state_handlers(
		this_port, SCIC_SDS_PORT_READY_SUBSTATE_OPERATIONAL
		);

	scic_cb_port_ready(
		scic_sds_port_get_controller(this_port), this_port
		);

	for (index = 0; index < SCI_MAX_PHYS; index++) {
		if (this_port->phy_table[index] != NULL) {
			scic_sds_port_write_phy_assignment(
				this_port, this_port->phy_table[index]
				);
		}
	}

	scic_sds_port_update_viit_entry(this_port);

	scic_sds_port_resume_port_task_scheduler(this_port);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_port object.
 *
 * This method will perform the actions required by the struct scic_sds_port on
 * exiting the SCIC_SDS_PORT_READY_SUBSTATE_OPERATIONAL. This function reports
 * the port not ready and suspends the port task scheduler. none
 */
static void scic_sds_port_ready_substate_operational_exit(
	struct sci_base_object *object)
{
	struct scic_sds_port *this_port = (struct scic_sds_port *)object;

	scic_cb_port_not_ready(
		scic_sds_port_get_controller(this_port),
		this_port,
		this_port->not_ready_reason
		);
}

/*
 * ******************************************************************************
 * *  PORT READY CONFIGURING METHODS
 * ****************************************************************************** */

/**
 * scic_sds_port_ready_substate_configuring_enter() -
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_port object.
 *
 * This method will perform the actions required by the struct scic_sds_port on
 * exiting the SCIC_SDS_PORT_READY_SUBSTATE_OPERATIONAL. This function reports
 * the port not ready and suspends the port task scheduler. none
 */
static void scic_sds_port_ready_substate_configuring_enter(
	struct sci_base_object *object)
{
	struct scic_sds_port *this_port = (struct scic_sds_port *)object;

	scic_sds_port_set_ready_state_handlers(
		this_port, SCIC_SDS_PORT_READY_SUBSTATE_CONFIGURING
		);

	if (this_port->active_phy_mask == 0) {
		scic_cb_port_not_ready(
			scic_sds_port_get_controller(this_port),
			this_port,
			SCIC_PORT_NOT_READY_NO_ACTIVE_PHYS
			);

		sci_base_state_machine_change_state(
			&this_port->ready_substate_machine,
			SCIC_SDS_PORT_READY_SUBSTATE_WAITING
			);
	} else if (this_port->started_request_count == 0) {
		sci_base_state_machine_change_state(
			&this_port->ready_substate_machine,
			SCIC_SDS_PORT_READY_SUBSTATE_OPERATIONAL
			);
	}
}

static void scic_sds_port_ready_substate_configuring_exit(
	struct sci_base_object *object)
{
	struct scic_sds_port *this_port = (struct scic_sds_port *)object;

	scic_sds_port_suspend_port_task_scheduler(this_port);
}

/* --------------------------------------------------------------------------- */

const struct sci_base_state scic_sds_port_ready_substate_table[] = {
	[SCIC_SDS_PORT_READY_SUBSTATE_WAITING] = {
		.enter_state = scic_sds_port_ready_substate_waiting_enter,
	},
	[SCIC_SDS_PORT_READY_SUBSTATE_OPERATIONAL] = {
		.enter_state = scic_sds_port_ready_substate_operational_enter,
		.exit_state  = scic_sds_port_ready_substate_operational_exit
	},
	[SCIC_SDS_PORT_READY_SUBSTATE_CONFIGURING] = {
		.enter_state = scic_sds_port_ready_substate_configuring_enter,
		.exit_state  = scic_sds_port_ready_substate_configuring_exit
	},
};

