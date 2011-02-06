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
 * This file implements the ready substate handlers for the struct scic_sds_port
 *    object.
 *
 *
 */

#include "scic_phy.h"
#include "scic_port.h"
#include "scic_user_callback.h"
#include "scic_controller.h"
#include "scic_sds_logger.h"
#include "scic_sds_controller.h"
#include "scic_sds_port.h"
#include "scic_sds_phy.h"
#include "scic_sds_remote_device.h"
#include "scic_sds_request.h"

/*
 * ****************************************************************************
 * *  READY SUBSTATE HANDLERS
 * **************************************************************************** */

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 *
 * This method is the general ready state stop handler for the struct scic_sds_port
 * object.  This function will transition the ready substate machine to its
 * final state. enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_port_ready_substate_stop_handler(
	struct sci_base_port *port)
{
	struct scic_sds_port *this_port = (struct scic_sds_port *)port;

	sci_base_state_machine_change_state(
		&this_port->parent.state_machine,
		SCI_BASE_PORT_STATE_STOPPING
		);

	return SCI_SUCCESS;
}

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 * @device: This is the struct sci_base_remote_device object which is not used in this
 *    function.
 * @io_request: This is the struct sci_base_request object which is not used in this
 *    function.
 *
 * This method is the general ready substate complete io handler for the
 * struct scic_sds_port object.  This function decrments the outstanding request count
 * for this port object. enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_port_ready_substate_complete_io_handler(
	struct scic_sds_port *port,
	struct scic_sds_remote_device *device,
	struct scic_sds_request *io_request)
{
	struct scic_sds_port *this_port = (struct scic_sds_port *)port;

	scic_sds_port_decrement_request_count(this_port);

	return SCI_SUCCESS;
}

static enum sci_status scic_sds_port_ready_substate_add_phy_handler(
	struct sci_base_port *port,
	struct sci_base_phy *phy)
{
	struct scic_sds_port *this_port = (struct scic_sds_port *)port;
	struct scic_sds_phy *this_phy  = (struct scic_sds_phy *)phy;
	enum sci_status status;

	status = scic_sds_port_set_phy(this_port, this_phy);

	if (status == SCI_SUCCESS) {
		scic_sds_port_general_link_up_handler(this_port, this_phy, true);

		this_port->not_ready_reason = SCIC_PORT_NOT_READY_RECONFIGURING;

		sci_base_state_machine_change_state(
			&this_port->ready_substate_machine,
			SCIC_SDS_PORT_READY_SUBSTATE_CONFIGURING
			);
	}

	return status;
}


static enum sci_status scic_sds_port_ready_substate_remove_phy_handler(
	struct sci_base_port *port,
	struct sci_base_phy *phy)
{
	struct scic_sds_port *this_port = (struct scic_sds_port *)port;
	struct scic_sds_phy *this_phy  = (struct scic_sds_phy *)phy;
	enum sci_status status;

	status = scic_sds_port_clear_phy(this_port, this_phy);

	if (status == SCI_SUCCESS) {
		scic_sds_port_deactivate_phy(this_port, this_phy, true);

		this_port->not_ready_reason = SCIC_PORT_NOT_READY_RECONFIGURING;

		sci_base_state_machine_change_state(
			&this_port->ready_substate_machine,
			SCIC_SDS_PORT_READY_SUBSTATE_CONFIGURING
			);
	}

	return status;
}

/*
 * ****************************************************************************
 * *  READY SUBSTATE WAITING HANDLERS
 * **************************************************************************** */

/**
 *
 * @this_port: This is the struct scic_sds_port object that which has a phy that has
 *    gone link up.
 * @the_phy: This is the struct scic_sds_phy object that has gone link up.
 *
 * This method is the ready waiting substate link up handler for the
 * struct scic_sds_port object.  This methos will report the link up condition for
 * this port and will transition to the ready operational substate. none
 */
static void scic_sds_port_ready_waiting_substate_link_up_handler(
	struct scic_sds_port *this_port,
	struct scic_sds_phy *the_phy)
{
	/*
	 * Since this is the first phy going link up for the port we can just enable
	 * it and continue. */
	scic_sds_port_activate_phy(this_port, the_phy, true);

	sci_base_state_machine_change_state(
		&this_port->ready_substate_machine,
		SCIC_SDS_PORT_READY_SUBSTATE_OPERATIONAL
		);
}

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 * @device: This is the struct sci_base_remote_device object which is not used in this
 *    request.
 * @io_request: This is the struct sci_base_request object which is not used in this
 *    function.
 *
 * This method is the ready waiting substate start io handler for the
 * struct scic_sds_port object. The port object can not accept new requests so the
 * request is failed. enum sci_status SCI_FAILURE_INVALID_STATE
 */
static enum sci_status scic_sds_port_ready_waiting_substate_start_io_handler(
	struct scic_sds_port *port,
	struct scic_sds_remote_device *device,
	struct scic_sds_request *io_request)
{
	return SCI_FAILURE_INVALID_STATE;
}

/*
 * ****************************************************************************
 * *  READY SUBSTATE OPERATIONAL HANDLERS
 * **************************************************************************** */

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 * @timeout: This is the timeout for the reset request to complete.
 *
 * This method will casue the port to reset. enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_port_ready_operational_substate_reset_handler(
	struct sci_base_port *port,
	u32 timeout)
{
	enum sci_status status = SCI_FAILURE_INVALID_PHY;
	u32 phy_index;
	struct scic_sds_port *this_port = (struct scic_sds_port *)port;
	struct scic_sds_phy *selected_phy = SCI_INVALID_HANDLE;


	/* Select a phy on which we can send the hard reset request. */
	for (
		phy_index = 0;
		(phy_index < SCI_MAX_PHYS)
		&& (selected_phy == SCI_INVALID_HANDLE);
		phy_index++
		) {
		selected_phy = this_port->phy_table[phy_index];

		if (
			(selected_phy != SCI_INVALID_HANDLE)
			&& !scic_sds_port_active_phy(this_port, selected_phy)
			) {
			/* We found a phy but it is not ready select different phy */
			selected_phy = SCI_INVALID_HANDLE;
		}
	}

	/* If we have a phy then go ahead and start the reset procedure */
	if (selected_phy != SCI_INVALID_HANDLE) {
		status = scic_sds_phy_reset(selected_phy);

		if (status == SCI_SUCCESS) {
			scic_cb_timer_start(
				scic_sds_port_get_controller(this_port),
				this_port->timer_handle,
				timeout
				);

			this_port->not_ready_reason = SCIC_PORT_NOT_READY_HARD_RESET_REQUESTED;

			sci_base_state_machine_change_state(
				&this_port->parent.state_machine,
				SCI_BASE_PORT_STATE_RESETTING
				);
		}
	}

	return status;
}

/**
 * scic_sds_port_ready_operational_substate_link_up_handler() -
 * @this_port: This is the struct scic_sds_port object that which has a phy that has
 *    gone link up.
 * @the_phy: This is the struct scic_sds_phy object that has gone link up.
 *
 * This method is the ready operational substate link up handler for the
 * struct scic_sds_port object. This function notifies the SCI User that the phy has
 * gone link up. none
 */
static void scic_sds_port_ready_operational_substate_link_up_handler(
	struct scic_sds_port *this_port,
	struct scic_sds_phy *the_phy)
{
	scic_sds_port_general_link_up_handler(this_port, the_phy, true);
}

/**
 * scic_sds_port_ready_operational_substate_link_down_handler() -
 * @this_port: This is the struct scic_sds_port object that which has a phy that has
 *    gone link down.
 * @the_phy: This is the struct scic_sds_phy object that has gone link down.
 *
 * This method is the ready operational substate link down handler for the
 * struct scic_sds_port object. This function notifies the SCI User that the phy has
 * gone link down and if this is the last phy in the port the port will change
 * state to the ready waiting substate. none
 */
static void scic_sds_port_ready_operational_substate_link_down_handler(
	struct scic_sds_port *this_port,
	struct scic_sds_phy *the_phy)
{
	scic_sds_port_deactivate_phy(this_port, the_phy, true);

	/*
	 * If there are no active phys left in the port, then transition
	 * the port to the WAITING state until such time as a phy goes
	 * link up. */
	if (this_port->active_phy_mask == 0) {
		sci_base_state_machine_change_state(
			scic_sds_port_get_ready_substate_machine(this_port),
			SCIC_SDS_PORT_READY_SUBSTATE_WAITING
			);
	}
}

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 * @device: This is the struct sci_base_remote_device object which is not used in this
 *    function.
 * @io_request: This is the struct sci_base_request object which is not used in this
 *    function.
 *
 * This method is the ready operational substate start io handler for the
 * struct scic_sds_port object.  This function incremetns the outstanding request
 * count for this port object. enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_port_ready_operational_substate_start_io_handler(
	struct scic_sds_port *port,
	struct scic_sds_remote_device *device,
	struct scic_sds_request *io_request)
{
	struct scic_sds_port *this_port = (struct scic_sds_port *)port;

	scic_sds_port_increment_request_count(this_port);

	return SCI_SUCCESS;
}

/*
 * ****************************************************************************
 * *  READY SUBSTATE OPERATIONAL HANDLERS
 * **************************************************************************** */

/**
 * scic_sds_port_ready_configuring_substate_add_phy_handler() -
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 *
 * This is the default method for a port add phy request.  It will report a
 * warning and exit. enum sci_status SCI_FAILURE_INVALID_STATE
 */
static enum sci_status scic_sds_port_ready_configuring_substate_add_phy_handler(
	struct sci_base_port *port,
	struct sci_base_phy *phy)
{
	struct scic_sds_port *this_port = (struct scic_sds_port *)port;
	struct scic_sds_phy *this_phy  = (struct scic_sds_phy *)phy;
	enum sci_status status;

	status = scic_sds_port_set_phy(this_port, this_phy);

	if (status == SCI_SUCCESS) {
		scic_sds_port_general_link_up_handler(this_port, this_phy, true);

		/*
		 * Re-enter the configuring state since this may be the last phy in
		 * the port. */
		sci_base_state_machine_change_state(
			&this_port->ready_substate_machine,
			SCIC_SDS_PORT_READY_SUBSTATE_CONFIGURING
			);
	}

	return status;
}

/**
 * scic_sds_port_ready_configuring_substate_remove_phy_handler() -
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 *
 * This is the default method for a port remove phy request.  It will report a
 * warning and exit. enum sci_status SCI_FAILURE_INVALID_STATE
 */
static enum sci_status scic_sds_port_ready_configuring_substate_remove_phy_handler(
	struct sci_base_port *port,
	struct sci_base_phy *phy)
{
	struct scic_sds_port *this_port = (struct scic_sds_port *)port;
	struct scic_sds_phy *this_phy  = (struct scic_sds_phy *)phy;
	enum sci_status status;

	status = scic_sds_port_clear_phy(this_port, this_phy);

	if (status == SCI_SUCCESS) {
		scic_sds_port_deactivate_phy(this_port, this_phy, true);

		/*
		 * Re-enter the configuring state since this may be the last phy in
		 * the port. */
		sci_base_state_machine_change_state(
			&this_port->ready_substate_machine,
			SCIC_SDS_PORT_READY_SUBSTATE_CONFIGURING
			);
	}

	return status;
}

/**
 * scic_sds_port_ready_configuring_substate_complete_io_handler() -
 * @port: This is the port that is being requested to complete the io request.
 * @device: This is the device on which the io is completing.
 *
 * This method will decrement the outstanding request count for this port. If
 * the request count goes to 0 then the port can be reprogrammed with its new
 * phy data.
 */
static enum sci_status scic_sds_port_ready_configuring_substate_complete_io_handler(
	struct scic_sds_port *port,
	struct scic_sds_remote_device *device,
	struct scic_sds_request *io_request)
{
	scic_sds_port_decrement_request_count(port);

	if (port->started_request_count == 0) {
		sci_base_state_machine_change_state(
			&port->ready_substate_machine,
			SCIC_SDS_PORT_READY_SUBSTATE_OPERATIONAL
			);
	}

	return SCI_SUCCESS;
}

/* --------------------------------------------------------------------------- */

struct scic_sds_port_state_handler
scic_sds_port_ready_substate_handler_table[SCIC_SDS_PORT_READY_MAX_SUBSTATES] =
{
	/* SCIC_SDS_PORT_READY_SUBSTATE_WAITING */
	{
		{
			scic_sds_port_default_start_handler,
			scic_sds_port_ready_substate_stop_handler,
			scic_sds_port_default_destruct_handler,
			scic_sds_port_default_reset_handler,
			scic_sds_port_ready_substate_add_phy_handler,
			scic_sds_port_default_remove_phy_handler
		},
		scic_sds_port_default_frame_handler,
		scic_sds_port_default_event_handler,
		scic_sds_port_ready_waiting_substate_link_up_handler,
		scic_sds_port_default_link_down_handler,
		scic_sds_port_ready_waiting_substate_start_io_handler,
		scic_sds_port_ready_substate_complete_io_handler,
	},
	/* SCIC_SDS_PORT_READY_SUBSTATE_OPERATIONAL */
	{
		{
			scic_sds_port_default_start_handler,
			scic_sds_port_ready_substate_stop_handler,
			scic_sds_port_default_destruct_handler,
			scic_sds_port_ready_operational_substate_reset_handler,
			scic_sds_port_ready_substate_add_phy_handler,
			scic_sds_port_ready_substate_remove_phy_handler
		},
		scic_sds_port_default_frame_handler,
		scic_sds_port_default_event_handler,
		scic_sds_port_ready_operational_substate_link_up_handler,
		scic_sds_port_ready_operational_substate_link_down_handler,
		scic_sds_port_ready_operational_substate_start_io_handler,
		scic_sds_port_ready_substate_complete_io_handler
	},
	/* SCIC_SDS_PORT_READY_SUBSTATE_CONFIGURING */
	{
		{
			scic_sds_port_default_start_handler,
			scic_sds_port_ready_substate_stop_handler,
			scic_sds_port_default_destruct_handler,
			scic_sds_port_default_reset_handler,
			scic_sds_port_ready_configuring_substate_add_phy_handler,
			scic_sds_port_ready_configuring_substate_remove_phy_handler
		},
		scic_sds_port_default_frame_handler,
		scic_sds_port_default_event_handler,
		scic_sds_port_default_link_up_handler,
		scic_sds_port_default_link_down_handler,
		scic_sds_port_default_start_io_handler,
		scic_sds_port_ready_configuring_substate_complete_io_handler
	}
};

