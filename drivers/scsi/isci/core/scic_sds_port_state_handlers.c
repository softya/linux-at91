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
 * This file contains the base state handlers for the struct scic_sds_port object.
 *
 *
 */

#include "intel_sas.h"
#include "sci_environment.h"
#include "scic_controller.h"
#include "scic_phy.h"
#include "scic_port.h"
#include "scic_user_callback.h"
#include "scic_sds_controller.h"
#include "scic_sds_port.h"
#include "scic_sds_phy.h"
#include "scic_sds_remote_device.h"
#include "scic_sds_request.h"
#include "scic_sds_port_registers.h"

/*
 * ***************************************************************************
 * *  DEFAULT HANDLERS
 * *************************************************************************** */

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 *
 * This is the default method for port a start request.  It will report a
 * warning and exit. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_port_default_start_handler(
	struct sci_base_port *port)
{
	struct scic_sds_port *sci_port = (struct scic_sds_port *)port;

	dev_warn(sciport_to_dev(sci_port),
		 "%s: SCIC Port 0x%p requested to start while in invalid "
		 "state %d\n",
		 __func__,
		 port,
		 sci_base_state_machine_get_state(
			 scic_sds_port_get_base_state_machine(
				 (struct scic_sds_port *)port)));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 *
 * This is the default method for a port stop request.  It will report a
 * warning and exit. enum sci_status SCI_FAILURE_INVALID_STATE
 */
static enum sci_status scic_sds_port_default_stop_handler(
	struct sci_base_port *port)
{
	struct scic_sds_port *sci_port = (struct scic_sds_port *)port;

	dev_warn(sciport_to_dev(sci_port),
		 "%s: SCIC Port 0x%p requested to stop while in invalid "
		 "state %d\n",
		 __func__,
		 port,
		 sci_base_state_machine_get_state(
			 scic_sds_port_get_base_state_machine(
				 (struct scic_sds_port *)port)));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 *
 * This is the default method for a port destruct request.  It will report a
 * warning and exit. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_port_default_destruct_handler(
	struct sci_base_port *port)
{
	struct scic_sds_port *sci_port = (struct scic_sds_port *)port;

	dev_warn(sciport_to_dev(sci_port),
		 "%s: SCIC Port 0x%p requested to destruct while in invalid "
		 "state %d\n",
		 __func__,
		 port,
		 sci_base_state_machine_get_state(
			 scic_sds_port_get_base_state_machine(
				 (struct scic_sds_port *)port)));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 * @timeout: This is the timeout for the reset request to complete.
 *
 * This is the default method for a port reset request.  It will report a
 * warning and exit. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_port_default_reset_handler(
	struct sci_base_port *port,
	u32 timeout)
{
	struct scic_sds_port *sci_port = (struct scic_sds_port *)port;

	dev_warn(sciport_to_dev(sci_port),
		 "%s: SCIC Port 0x%p requested to reset while in invalid "
		 "state %d\n",
		 __func__,
		 port,
		 sci_base_state_machine_get_state(
			 scic_sds_port_get_base_state_machine(
				 (struct scic_sds_port *)port)));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 *
 * This is the default method for a port add phy request.  It will report a
 * warning and exit. enum sci_status SCI_FAILURE_INVALID_STATE
 */
static enum sci_status scic_sds_port_default_add_phy_handler(
	struct sci_base_port *port,
	struct sci_base_phy *phy)
{
	struct scic_sds_port *sci_port = (struct scic_sds_port *)port;

	dev_warn(sciport_to_dev(sci_port),
		 "%s: SCIC Port 0x%p requested to add phy 0x%p while in "
		 "invalid state %d\n",
		 __func__,
		 port,
		 phy,
		 sci_base_state_machine_get_state(
			 scic_sds_port_get_base_state_machine(
				 (struct scic_sds_port *)port)));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 *
 * This is the default method for a port remove phy request.  It will report a
 * warning and exit. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_port_default_remove_phy_handler(
	struct sci_base_port *port,
	struct sci_base_phy *phy)
{
	struct scic_sds_port *sci_port = (struct scic_sds_port *)port;

	dev_warn(sciport_to_dev(sci_port),
		 "%s: SCIC Port 0x%p requested to remove phy 0x%p while in "
		 "invalid state %d\n",
		 __func__,
		 port,
		 phy,
		 sci_base_state_machine_get_state(
			 scic_sds_port_get_base_state_machine(
				 (struct scic_sds_port *)port)));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 *
 * This is the default method for a port unsolicited frame request.  It will
 * report a warning and exit. enum sci_status SCI_FAILURE_INVALID_STATE Is it even
 * possible to receive an unsolicited frame directed to a port object?  It
 * seems possible if we implementing virtual functions but until then?
 */
enum sci_status scic_sds_port_default_frame_handler(
	struct scic_sds_port *port,
	u32 frame_index)
{
	dev_warn(sciport_to_dev(port),
		 "%s: SCIC Port 0x%p requested to process frame %d while in "
		 "invalid state %d\n",
		 __func__,
		 port,
		 frame_index,
		 sci_base_state_machine_get_state(
			 scic_sds_port_get_base_state_machine(port)));

	scic_sds_controller_release_frame(
		scic_sds_port_get_controller(port), frame_index
		);

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 *
 * This is the default method for a port event request.  It will report a
 * warning and exit. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_port_default_event_handler(
	struct scic_sds_port *port,
	u32 event_code)
{
	dev_warn(sciport_to_dev(port),
		 "%s: SCIC Port 0x%p requested to process event 0x%x while "
		 "in invalid state %d\n",
		 __func__,
		 port,
		 event_code,
		 sci_base_state_machine_get_state(
			 scic_sds_port_get_base_state_machine(
				 (struct scic_sds_port *)port)));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 *
 * This is the default method for a port link up notification.  It will report
 * a warning and exit. enum sci_status SCI_FAILURE_INVALID_STATE
 */
void scic_sds_port_default_link_up_handler(
	struct scic_sds_port *this_port,
	struct scic_sds_phy *phy)
{
	dev_warn(sciport_to_dev(this_port),
		 "%s: SCIC Port 0x%p received link_up notification from phy "
		 "0x%p while in invalid state %d\n",
		 __func__,
		 this_port,
		 phy,
		 sci_base_state_machine_get_state(
			 scic_sds_port_get_base_state_machine(this_port)));
}

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 *
 * This is the default method for a port link down notification.  It will
 * report a warning and exit. enum sci_status SCI_FAILURE_INVALID_STATE
 */
void scic_sds_port_default_link_down_handler(
	struct scic_sds_port *this_port,
	struct scic_sds_phy *phy)
{
	dev_warn(sciport_to_dev(this_port),
		 "%s: SCIC Port 0x%p received link down notification from "
		 "phy 0x%p while in invalid state %d\n",
		 __func__,
		 this_port,
		 phy,
		 sci_base_state_machine_get_state(
			 scic_sds_port_get_base_state_machine(this_port)));
}

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 *
 * This is the default method for a port start io request.  It will report a
 * warning and exit. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_port_default_start_io_handler(
	struct scic_sds_port *this_port,
	struct scic_sds_remote_device *device,
	struct scic_sds_request *io_request)
{
	dev_warn(sciport_to_dev(this_port),
		 "%s: SCIC Port 0x%p requested to start io request 0x%p "
		 "while in invalid state %d\n",
		 __func__,
		 this_port,
		 io_request,
		 sci_base_state_machine_get_state(
			 scic_sds_port_get_base_state_machine(this_port)));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 *
 * This is the default method for a port complete io request.  It will report a
 * warning and exit. enum sci_status SCI_FAILURE_INVALID_STATE
 */
static enum sci_status scic_sds_port_default_complete_io_handler(
	struct scic_sds_port *this_port,
	struct scic_sds_remote_device *device,
	struct scic_sds_request *io_request)
{
	dev_warn(sciport_to_dev(this_port),
		 "%s: SCIC Port 0x%p requested to complete io request 0x%p "
		 "while in invalid state %d\n",
		 __func__,
		 this_port,
		 io_request,
		 sci_base_state_machine_get_state(
			 scic_sds_port_get_base_state_machine(this_port)));

	return SCI_FAILURE_INVALID_STATE;
}

/*
 * ****************************************************************************
 * * GENERAL STATE HANDLERS
 * **************************************************************************** */

/**
 *
 * @port: This is the struct scic_sds_port object on which the io request count will
 *    be decremented.
 * @device: This is the struct scic_sds_remote_device object to which the io request
 *    is being directed.  This parameter is not required to complete this
 *    operation.
 * @io_request: This is the request that is being completed on this port
 *    object.  This parameter is not required to complete this operation.
 *
 * This is a general complete io request handler for the struct scic_sds_port object.
 * enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_port_general_complete_io_handler(
	struct scic_sds_port *port,
	struct scic_sds_remote_device *device,
	struct scic_sds_request *io_request)
{
	struct scic_sds_port *this_port = (struct scic_sds_port *)port;

	scic_sds_port_decrement_request_count(this_port);

	return SCI_SUCCESS;
}

/*
 * ****************************************************************************
 * * STOPPED STATE HANDLERS
 * **************************************************************************** */

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 *
 * This method takes the struct scic_sds_port from a stopped state and attempts to
 * start it.  To start a port it must have no assiged devices and it must have
 * at least one phy assigned to it.  If those conditions are met then the port
 * can transition to the ready state. enum sci_status
 * SCI_FAILURE_UNSUPPORTED_PORT_CONFIGURATION This struct scic_sds_port object could
 * not be started because the port configuration is not valid. SCI_SUCCESS the
 * start request is successful and the struct scic_sds_port object has transitioned to
 * the SCI_BASE_PORT_STATE_READY.
 */
static enum sci_status scic_sds_port_stopped_state_start_handler(
	struct sci_base_port *port)
{
	u32 phy_mask;
	struct scic_sds_port *this_port = (struct scic_sds_port *)port;

	if (this_port->assigned_device_count > 0) {
		/*
		 * / @todo This is a start failure operation because there are still
		 * /       devices assigned to this port.  There must be no devices
		 * /       assigned to a port on a start operation. */
		return SCI_FAILURE_UNSUPPORTED_PORT_CONFIGURATION;
	}

	phy_mask = scic_sds_port_get_phys(this_port);

	/*
	 * There are one or more phys assigned to this port.  Make sure
	 * the port's phy mask is in fact legal and supported by the
	 * silicon. */
	if (scic_sds_port_is_phy_mask_valid(this_port, phy_mask) == true) {
		sci_base_state_machine_change_state(
			scic_sds_port_get_base_state_machine(this_port),
			SCI_BASE_PORT_STATE_READY
			);

		return SCI_SUCCESS;
	}

	return SCI_FAILURE_UNSUPPORTED_PORT_CONFIGURATION;
}

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 *
 * This method takes the struct scic_sds_port that is in a stopped state and handles a
 * stop request.  This function takes no action. enum sci_status SCI_SUCCESS the
 * stop request is successful as the struct scic_sds_port object is already stopped.
 */
static enum sci_status scic_sds_port_stopped_state_stop_handler(
	struct sci_base_port *port)
{
	/* We are already stopped so there is nothing to do here */
	return SCI_SUCCESS;
}

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 *
 * This method takes the struct scic_sds_port that is in a stopped state and handles
 * the destruct request.  The stopped state is the only state in which the
 * struct scic_sds_port can be destroyed.  This function causes the port object to
 * transition to the SCI_BASE_PORT_STATE_FINAL. enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_port_stopped_state_destruct_handler(
	struct sci_base_port *port)
{
	struct scic_sds_port *this_port = (struct scic_sds_port *)port;

	sci_base_state_machine_stop(&this_port->parent.state_machine);

	return SCI_SUCCESS;
}

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 * @phy: This is the struct sci_base_phy object which is cast into a struct scic_sds_phy
 *    object.
 *
 * This method takes the struct scic_sds_port that is in a stopped state and handles
 * the add phy request.  In MPC mode the only time a phy can be added to a port
 * is in the SCI_BASE_PORT_STATE_STOPPED. enum sci_status
 * SCI_FAILURE_UNSUPPORTED_PORT_CONFIGURATION is returned when the phy can not
 * be added to the port. SCI_SUCCESS if the phy is added to the port.
 */
static enum sci_status scic_sds_port_stopped_state_add_phy_handler(
	struct sci_base_port *port,
	struct sci_base_phy *phy)
{
	struct scic_sds_port *this_port = (struct scic_sds_port *)port;
	struct scic_sds_phy *this_phy  = (struct scic_sds_phy *)phy;
	struct sci_sas_address port_sas_address;

	/* Read the port assigned SAS Address if there is one */
	scic_sds_port_get_sas_address(this_port, &port_sas_address);

	if (port_sas_address.high != 0 && port_sas_address.low != 0) {
		struct sci_sas_address phy_sas_address;

		/*
		 * Make sure that the PHY SAS Address matches the SAS Address
		 * for this port. */
		scic_sds_phy_get_sas_address(this_phy, &phy_sas_address);

		if (
			(port_sas_address.high != phy_sas_address.high)
			|| (port_sas_address.low  != phy_sas_address.low)
			) {
			return SCI_FAILURE_UNSUPPORTED_PORT_CONFIGURATION;
		}
	}

	return scic_sds_port_set_phy(this_port, this_phy);
}


/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 * @phy: This is the struct sci_base_phy object which is cast into a struct scic_sds_phy
 *    object.
 *
 * This method takes the struct scic_sds_port that is in a stopped state and handles
 * the remove phy request.  In MPC mode the only time a phy can be removed from
 * a port is in the SCI_BASE_PORT_STATE_STOPPED. enum sci_status
 * SCI_FAILURE_UNSUPPORTED_PORT_CONFIGURATION is returned when the phy can not
 * be added to the port. SCI_SUCCESS if the phy is added to the port.
 */
static enum sci_status scic_sds_port_stopped_state_remove_phy_handler(
	struct sci_base_port *port,
	struct sci_base_phy *phy)
{
	struct scic_sds_port *this_port = (struct scic_sds_port *)port;
	struct scic_sds_phy *this_phy  = (struct scic_sds_phy *)phy;

	return scic_sds_port_clear_phy(this_port, this_phy);
}

/*
 * ****************************************************************************
 * *  READY STATE HANDLERS
 * **************************************************************************** */

/*
 * ****************************************************************************
 * *  RESETTING STATE HANDLERS
 * **************************************************************************** */

/*
 * ****************************************************************************
 * *  STOPPING STATE HANDLERS
 * **************************************************************************** */

/**
 *
 * @port: This is the struct scic_sds_port object on which the io request count will
 *    be decremented.
 * @device: This is the struct scic_sds_remote_device object to which the io request
 *    is being directed.  This parameter is not required to complete this
 *    operation.
 * @io_request: This is the request that is being completed on this port
 *    object.  This parameter is not required to complete this operation.
 *
 * This method takes the struct scic_sds_port that is in a stopping state and handles
 * the complete io request. Should the request count reach 0 then the port
 * object will transition to the stopped state. enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_port_stopping_state_complete_io_handler(
	struct scic_sds_port *port,
	struct scic_sds_remote_device *device,
	struct scic_sds_request *io_request)
{
	struct scic_sds_port *this_port = (struct scic_sds_port *)port;

	scic_sds_port_decrement_request_count(this_port);

	if (this_port->started_request_count == 0) {
		sci_base_state_machine_change_state(
			scic_sds_port_get_base_state_machine(this_port),
			SCI_BASE_PORT_STATE_STOPPED
			);
	}

	return SCI_SUCCESS;
}

/*
 * ****************************************************************************
 * *  RESETTING STATE HANDLERS
 * **************************************************************************** */

/**
 *
 * @port: This is the port object which is being requested to stop.
 *
 * This method will stop a failed port.  This causes a transition to the
 * stopping state. enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_port_reset_state_stop_handler(
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
 *
 * This method will transition a failed port to its ready state.  The port
 * failed because a hard reset request timed out but at some time later one or
 * more phys in the port became ready. enum sci_status SCI_SUCCESS
 */
static void scic_sds_port_reset_state_link_up_handler(
	struct scic_sds_port *this_port,
	struct scic_sds_phy *phy)
{
	/*
	 * / @todo We should make sure that the phy that has gone link up is the same
	 * /       one on which we sent the reset.  It is possible that the phy on
	 * /       which we sent the reset is not the one that has gone link up and we
	 * /       want to make sure that phy being reset comes back.  Consider the
	 * /       case where a reset is sent but before the hardware processes the
	 * /       reset it get a link up on the port because of a hot plug event.
	 * /       because of the reset request this phy will go link down almost
	 * /       immediately. */

	/*
	 * In the resetting state we don't notify the user regarding
	 * link up and link down notifications. */
	scic_sds_port_general_link_up_handler(this_port, phy, false);
}

/**
 *
 * @port: This is the struct sci_base_port object which is cast into a struct scic_sds_port
 *    object.
 *
 * This method process link down notifications that occur during a port reset
 * operation. Link downs can occur during the reset operation. enum sci_status
 * SCI_SUCCESS
 */
static void scic_sds_port_reset_state_link_down_handler(
	struct scic_sds_port *this_port,
	struct scic_sds_phy *phy)
{
	/*
	 * In the resetting state we don't notify the user regarding
	 * link up and link down notifications. */
	scic_sds_port_deactivate_phy(this_port, phy, false);
}

/* --------------------------------------------------------------------------- */

struct scic_sds_port_state_handler
scic_sds_port_state_handler_table[SCI_BASE_PORT_MAX_STATES] =
{
	/* SCI_BASE_PORT_STATE_STOPPED */
	{
		{
			scic_sds_port_stopped_state_start_handler,
			scic_sds_port_stopped_state_stop_handler,
			scic_sds_port_stopped_state_destruct_handler,
			scic_sds_port_default_reset_handler,
			scic_sds_port_stopped_state_add_phy_handler,
			scic_sds_port_stopped_state_remove_phy_handler
		},
		scic_sds_port_default_frame_handler,
		scic_sds_port_default_event_handler,
		scic_sds_port_default_link_up_handler,
		scic_sds_port_default_link_down_handler,
		scic_sds_port_default_start_io_handler,
		scic_sds_port_default_complete_io_handler
	},
	/* SCI_BASE_PORT_STATE_STOPPING */
	{
		{
			scic_sds_port_default_start_handler,
			scic_sds_port_default_stop_handler,
			scic_sds_port_default_destruct_handler,
			scic_sds_port_default_reset_handler,
			scic_sds_port_default_add_phy_handler,
			scic_sds_port_default_remove_phy_handler
		},
		scic_sds_port_default_frame_handler,
		scic_sds_port_default_event_handler,
		scic_sds_port_default_link_up_handler,
		scic_sds_port_default_link_down_handler,
		scic_sds_port_default_start_io_handler,
		scic_sds_port_stopping_state_complete_io_handler
	},
	/* SCI_BASE_PORT_STATE_READY */
	{
		{
			scic_sds_port_default_start_handler,
			scic_sds_port_default_stop_handler,
			scic_sds_port_default_destruct_handler,
			scic_sds_port_default_reset_handler,
			scic_sds_port_default_add_phy_handler,
			scic_sds_port_default_remove_phy_handler
		},
		scic_sds_port_default_frame_handler,
		scic_sds_port_default_event_handler,
		scic_sds_port_default_link_up_handler,
		scic_sds_port_default_link_down_handler,
		scic_sds_port_default_start_io_handler,
		scic_sds_port_general_complete_io_handler
	},
	/* SCI_BASE_PORT_STATE_RESETTING */
	{
		{
			scic_sds_port_default_start_handler,
			scic_sds_port_reset_state_stop_handler,
			scic_sds_port_default_destruct_handler,
			scic_sds_port_default_reset_handler,
			scic_sds_port_default_add_phy_handler,
			scic_sds_port_default_remove_phy_handler
		},
		scic_sds_port_default_frame_handler,
		scic_sds_port_default_event_handler,
		scic_sds_port_reset_state_link_up_handler,
		scic_sds_port_reset_state_link_down_handler,
		scic_sds_port_default_start_io_handler,
		scic_sds_port_general_complete_io_handler
	},
	/* SCI_BASE_PORT_STATE_FAILED */
	{
		{
			scic_sds_port_default_start_handler,
			scic_sds_port_default_stop_handler,
			scic_sds_port_default_destruct_handler,
			scic_sds_port_default_reset_handler,
			scic_sds_port_default_add_phy_handler,
			scic_sds_port_default_remove_phy_handler
		},
		scic_sds_port_default_frame_handler,
		scic_sds_port_default_event_handler,
		scic_sds_port_default_link_up_handler,
		scic_sds_port_default_link_down_handler,
		scic_sds_port_default_start_io_handler,
		scic_sds_port_general_complete_io_handler
	}
};

