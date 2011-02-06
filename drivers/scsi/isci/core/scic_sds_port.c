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
 * This file contains the implementation for the public and protected methods
 *    for the struct scic_sds_port object.
 *
 *
 */

#include "scic_phy.h"
#include "scic_port.h"
#include "scic_controller.h"
#include "scic_user_callback.h"

#include "scic_sds_controller.h"
#include "scic_sds_port.h"
#include "scic_sds_phy.h"
#include "scic_sds_remote_device.h"
#include "scic_sds_request.h"
#include "scic_sds_port_registers.h"
#include "scic_sds_logger.h"
#include "scic_sds_phy_registers.h"

static void scic_sds_port_invalid_link_up(
	struct scic_sds_port *this_port,
	struct scic_sds_phy *phy);
static void scic_sds_port_timeout_handler(
	void *port);
#define SCIC_SDS_PORT_MIN_TIMER_COUNT  (SCI_MAX_PORTS)
#define SCIC_SDS_PORT_MAX_TIMER_COUNT  (SCI_MAX_PORTS)

#define SCIC_SDS_PORT_HARD_RESET_TIMEOUT  (1000)

/**
 *
 * @this_port: This is the port object to which the phy is being assigned.
 * @phy_index: This is the phy index that is being assigned to the port.
 *
 * This method will return a true value if the specified phy can be assigned to
 * this port The following is a list of phys for each port that are allowed: -
 * Port 0 - 3 2 1 0 - Port 1 -     1 - Port 2 - 3 2 - Port 3 - 3 This method
 * doesn't preclude all configurations.  It merely ensures that a phy is part
 * of the allowable set of phy identifiers for that port.  For example, one
 * could assign phy 3 to port 0 and no other phys.  Please refer to
 * scic_sds_port_is_phy_mask_valid() for information regarding whether the
 * phy_mask for a port can be supported. bool true if this is a valid phy
 * assignment for the port false if this is not a valid phy assignment for the
 * port
 */
bool scic_sds_port_is_valid_phy_assignment(
	struct scic_sds_port *this_port,
	u32 phy_index)
{
	/* Initialize to invalid value. */
	u32 existing_phy_index = SCI_MAX_PHYS;
	u32 index;

	if ((this_port->physical_port_index == 1) && (phy_index != 1)) {
		return false;
	}

	if (this_port->physical_port_index == 3 && phy_index != 3) {
		return false;
	}

	if (
		(this_port->physical_port_index == 2)
		&& ((phy_index == 0) || (phy_index == 1))
		) {
		return false;
	}

	for (index = 0; index < SCI_MAX_PHYS; index++) {
		if ((this_port->phy_table[index] != NULL)
		    && (index != phy_index)) {
			existing_phy_index = index;
		}
	}

	/*
	 * Ensure that all of the phys in the port are capable of
	 * operating at the same maximum link rate. */
	if (
		(existing_phy_index < SCI_MAX_PHYS)
		&& (this_port->owning_controller->user_parameters.sds1.phys[
			    phy_index].max_speed_generation !=
		    this_port->owning_controller->user_parameters.sds1.phys[
			    existing_phy_index].max_speed_generation)
		)
		return false;

	return true;
}

/**
 * This method requests a list (mask) of the phys contained in the supplied SAS
 *    port.
 * @this_port: a handle corresponding to the SAS port for which to return the
 *    phy mask.
 *
 * Return a bit mask indicating which phys are a part of this port. Each bit
 * corresponds to a phy identifier (e.g. bit 0 = phy id 0).
 */
u32 scic_sds_port_get_phys(
	struct scic_sds_port *this_port)
{
	u32 index;
	u32 mask;

	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_port),
			       SCIC_LOG_OBJECT_PORT,
			       "scic_sds_port_get_phys(0x%x) enter\n",
			       this_port
			       ));

	mask = 0;

	for (index = 0; index < SCI_MAX_PHYS; index++) {
		if (this_port->phy_table[index] != NULL) {
			mask |= (1 << index);
		}
	}

	return mask;
}

/**
 *
 * @this_port: This is the port object for which to determine if the phy mask
 *    can be supported.
 *
 * This method will return a true value if the port's phy mask can be supported
 * by the SCU. The following is a list of valid PHY mask configurations for
 * each port: - Port 0 - [[3  2] 1] 0 - Port 1 -        [1] - Port 2 - [[3] 2]
 * - Port 3 -  [3] This method returns a boolean indication specifying if the
 * phy mask can be supported. true if this is a valid phy assignment for the
 * port false if this is not a valid phy assignment for the port
 */
bool scic_sds_port_is_phy_mask_valid(
	struct scic_sds_port *this_port,
	u32 phy_mask)
{
	if (this_port->physical_port_index == 0) {
		if (((phy_mask & 0x0F) == 0x0F)
		    || ((phy_mask & 0x03) == 0x03)
		    || ((phy_mask & 0x01) == 0x01)
		    || (phy_mask == 0))
			return true;
	} else if (this_port->physical_port_index == 1) {
		if (((phy_mask & 0x02) == 0x02)
		    || (phy_mask == 0))
			return true;
	} else if (this_port->physical_port_index == 2) {
		if (((phy_mask & 0x0C) == 0x0C)
		    || ((phy_mask & 0x04) == 0x04)
		    || (phy_mask == 0))
			return true;
	} else if (this_port->physical_port_index == 3) {
		if (((phy_mask & 0x08) == 0x08)
		    || (phy_mask == 0))
			return true;
	}

	return false;
}

/**
 *
 * @this_port: This parameter specifies the port from which to return a
 *    connected phy.
 *
 * This method retrieves a currently active (i.e. connected) phy contained in
 * the port.  Currently, the lowest order phy that is connected is returned.
 * This method returns a pointer to a SCIS_SDS_PHY object. NULL This value is
 * returned if there are no currently active (i.e. connected to a remote end
 * point) phys contained in the port. All other values specify a struct scic_sds_phy
 * object that is active in the port.
 */
static struct scic_sds_phy *scic_sds_port_get_a_connected_phy(
	struct scic_sds_port *this_port
	) {
	u32 index;
	struct scic_sds_phy *phy;

	for (index = 0; index < SCI_MAX_PHYS; index++) {
		/*
		 * Ensure that the phy is both part of the port and currently
		 * connected to the remote end-point. */
		phy = this_port->phy_table[index];
		if (
			(phy != NULL)
			&& scic_sds_port_active_phy(this_port, phy)
			) {
			return phy;
		}
	}

	return NULL;
}

/**
 * scic_sds_port_set_phy() -
 * @out]: port The port object to which the phy assignement is being made.
 * @out]: phy The phy which is being assigned to the port.
 *
 * This method attempts to make the assignment of the phy to the port. If
 * successful the phy is assigned to the ports phy table. bool true if the phy
 * assignment can be made. false if the phy assignement can not be made. This
 * is a functional test that only fails if the phy is currently assigned to a
 * different port.
 */
enum sci_status scic_sds_port_set_phy(
	struct scic_sds_port *port,
	struct scic_sds_phy *phy)
{
	/*
	 * Check to see if we can add this phy to a port
	 * that means that the phy is not part of a port and that the port does
	 * not already have a phy assinged to the phy index. */
	if (
		(port->phy_table[phy->phy_index] == SCI_INVALID_HANDLE)
		&& (scic_sds_phy_get_port(phy) == SCI_INVALID_HANDLE)
		&& scic_sds_port_is_valid_phy_assignment(port, phy->phy_index)
		) {
		/*
		 * Phy is being added in the stopped state so we are in MPC mode
		 * make logical port index = physical port index */
		port->logical_port_index = port->physical_port_index;
		port->phy_table[phy->phy_index] = phy;
		scic_sds_phy_set_port(phy, port);

		return SCI_SUCCESS;
	}

	return SCI_FAILURE;
}

/**
 * scic_sds_port_clear_phy() -
 * @out]: port The port from which the phy is being cleared.
 * @out]: phy The phy being cleared from the port.
 *
 * This method will clear the phy assigned to this port.  This method fails if
 * this phy is not currently assinged to this port. bool true if the phy is
 * removed from the port. false if this phy is not assined to this port.
 */
enum sci_status scic_sds_port_clear_phy(
	struct scic_sds_port *port,
	struct scic_sds_phy *phy)
{
	/* Make sure that this phy is part of this port */
	if (
		(port->phy_table[phy->phy_index] == phy)
		&& (scic_sds_phy_get_port(phy) == port)
		) {
		/* Yep it is assigned to this port so remove it */
		scic_sds_phy_set_port(
			phy,
			&scic_sds_port_get_controller(port)->port_table[SCI_MAX_PORTS]
			);

		port->phy_table[phy->phy_index] = SCI_INVALID_HANDLE;

		return SCI_SUCCESS;
	}

	return SCI_FAILURE;
}

/**
 * scic_sds_port_add_phy() -
 * @this_port: This parameter specifies the port in which the phy will be added.
 * @the_phy: This parameter is the phy which is to be added to the port.
 *
 * This method will add a PHY to the selected port. This method returns an
 * enum sci_status. SCI_SUCCESS the phy has been added to the port. Any other status
 * is failre to add the phy to the port.
 */
enum sci_status scic_sds_port_add_phy(
	struct scic_sds_port *this_port,
	struct scic_sds_phy *the_phy)
{
	return this_port->state_handlers->parent.add_phy_handler(
		       &this_port->parent, &the_phy->parent);
}


/**
 * scic_sds_port_remove_phy() -
 * @this_port: This parameter specifies the port in which the phy will be added.
 * @the_phy: This parameter is the phy which is to be added to the port.
 *
 * This method will remove the PHY from the selected PORT. This method returns
 * an enum sci_status. SCI_SUCCESS the phy has been removed from the port. Any other
 * status is failre to add the phy to the port.
 */
enum sci_status scic_sds_port_remove_phy(
	struct scic_sds_port *this_port,
	struct scic_sds_phy *the_phy)
{
	return this_port->state_handlers->parent.remove_phy_handler(
		       &this_port->parent, &the_phy->parent);
}

/**
 * This method requests the SAS address for the supplied SAS port from the SCI
 *    implementation.
 * @this_port: a handle corresponding to the SAS port for which to return the
 *    SAS address.
 * @sas_address: This parameter specifies a pointer to a SAS address structure
 *    into which the core will copy the SAS address for the port.
 *
 */
void scic_sds_port_get_sas_address(
	struct scic_sds_port *this_port,
	struct sci_sas_address *sas_address)
{
	u32 index;

	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_port),
			       SCIC_LOG_OBJECT_PORT,
			       "scic_sds_port_get_sas_address(0x%x, 0x%x) enter\n",
			       this_port, sas_address
			       ));

	sas_address->high = 0;
	sas_address->low  = 0;

	for (index = 0; index < SCI_MAX_PHYS; index++) {
		if (this_port->phy_table[index] != NULL) {
			scic_sds_phy_get_sas_address(this_port->phy_table[index], sas_address);
		}
	}
}

/**
 * This method will indicate which protocols are supported by this port.
 * @this_port: a handle corresponding to the SAS port for which to return the
 *    supported protocols.
 * @protocols: This parameter specifies a pointer to an IAF protocol field
 *    structure into which the core will copy the protocol values for the port.
 *     The values are returned as part of a bit mask in order to allow for
 *    multi-protocol support.
 *
 */
static void scic_sds_port_get_protocols(
	struct scic_sds_port *this_port,
	struct sci_sas_identify_address_frame_protocols *protocols)
{
	u8 index;

	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_port),
			       SCIC_LOG_OBJECT_PORT,
			       "scic_sds_port_get_protocols(0x%x, 0x%x) enter\n",
			       this_port, protocols
			       ));

	protocols->u.all = 0;

	for (index = 0; index < SCI_MAX_PHYS; index++) {
		if (this_port->phy_table[index] != NULL) {
			scic_sds_phy_get_protocols(this_port->phy_table[index], protocols);
		}
	}
}

/**
 * This method requests the SAS address for the device directly attached to
 *    this SAS port.
 * @this_port: a handle corresponding to the SAS port for which to return the
 *    SAS address.
 * @sas_address: This parameter specifies a pointer to a SAS address structure
 *    into which the core will copy the SAS address for the device directly
 *    attached to the port.
 *
 */
void scic_sds_port_get_attached_sas_address(
	struct scic_sds_port *this_port,
	struct sci_sas_address *sas_address)
{
	struct sci_sas_identify_address_frame_protocols protocols;
	struct scic_sds_phy *phy;

	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_port),
			       SCIC_LOG_OBJECT_PORT,
			       "scic_sds_port_get_attached_sas_address(0x%x, 0x%x) enter\n",
			       this_port, sas_address
			       ));

	/*
	 * Ensure that the phy is both part of the port and currently
	 * connected to the remote end-point. */
	phy = scic_sds_port_get_a_connected_phy(this_port);
	if (phy != NULL) {
		scic_sds_phy_get_attached_phy_protocols(phy, &protocols);

		if (!protocols.u.bits.stp_target) {
			scic_sds_phy_get_attached_sas_address(phy, sas_address);
		} else {
			scic_sds_phy_get_sas_address(phy, sas_address);
			sas_address->low += phy->phy_index;
		}
	} else {
		sas_address->high = 0;
		sas_address->low  = 0;
	}
}

/**
 * This method will indicate which protocols are supported by this remote
 *    device.
 * @this_port: a handle corresponding to the SAS port for which to return the
 *    supported protocols.
 * @protocols: This parameter specifies a pointer to an IAF protocol field
 *    structure into which the core will copy the protocol values for the port.
 *     The values are returned as part of a bit mask in order to allow for
 *    multi-protocol support.
 *
 */
void scic_sds_port_get_attached_protocols(
	struct scic_sds_port *this_port,
	struct sci_sas_identify_address_frame_protocols *protocols)
{
	struct scic_sds_phy *phy;

	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_port),
			       SCIC_LOG_OBJECT_PORT,
			       "scic_sds_port_get_attached_protocols(0x%x, 0x%x) enter\n",
			       this_port, protocols
			       ));

	/*
	 * Ensure that the phy is both part of the port and currently
	 * connected to the remote end-point. */
	phy = scic_sds_port_get_a_connected_phy(this_port);
	if (phy != NULL)
		scic_sds_phy_get_attached_phy_protocols(phy, protocols);
	else
		protocols->u.all = 0;
}

/**
 * This method returns the amount of memory requred for a port object.
 *
 * u32
 */

/**
 * This method returns the minimum number of timers required for all port
 *    objects.
 *
 * u32
 */

/**
 * This method returns the maximum number of timers required for all port
 *    objects.
 *
 * u32
 */

#ifdef SCI_LOGGING
static void scic_sds_port_initialize_state_logging(
	struct scic_sds_port *this_port)
{
	sci_base_state_machine_logger_initialize(
		&this_port->parent.state_machine_logger,
		&this_port->parent.state_machine,
		&this_port->parent.parent,
		scic_cb_logger_log_states,
		"struct scic_sds_port", "base state machine",
		SCIC_LOG_OBJECT_PORT
		);

	sci_base_state_machine_logger_initialize(
		&this_port->ready_substate_machine_logger,
		&this_port->ready_substate_machine,
		&this_port->parent.parent,
		scic_cb_logger_log_states,
		"struct scic_sds_port", "ready substate machine",
		SCIC_LOG_OBJECT_PORT
		);
}
#endif

/**
 *
 * @this_port:
 * @port_index:
 *
 *
 */
void scic_sds_port_construct(
	struct scic_sds_port *this_port,
	u8 port_index,
	struct scic_sds_controller *owning_controller)
{
	u32 index;

	sci_base_port_construct(
		&this_port->parent,
		sci_base_object_get_logger(owning_controller),
		scic_sds_port_state_table
		);

	sci_base_state_machine_construct(
		scic_sds_port_get_ready_substate_machine(this_port),
		&this_port->parent.parent,
		scic_sds_port_ready_substate_table,
		SCIC_SDS_PORT_READY_SUBSTATE_WAITING
		);

	scic_sds_port_initialize_state_logging(this_port);

	this_port->logical_port_index  = SCIC_SDS_DUMMY_PORT;
	this_port->physical_port_index = port_index;
	this_port->active_phy_mask     = 0;

	this_port->owning_controller = owning_controller;

	this_port->started_request_count = 0;
	this_port->assigned_device_count = 0;

	this_port->timer_handle = SCI_INVALID_HANDLE;

	this_port->transport_layer_registers = NULL;
	this_port->port_task_scheduler_registers = NULL;

	for (index = 0; index < SCI_MAX_PHYS; index++) {
		this_port->phy_table[index] = NULL;
	}
}

/**
 * This method performs initialization of the supplied port. Initialization
 *    includes: - state machine initialization - member variable initialization
 *    - configuring the phy_mask
 * @this_port:
 * @transport_layer_registers:
 * @port_task_scheduler_registers:
 * @port_configuration_regsiter:
 *
 * enum sci_status SCI_FAILURE_UNSUPPORTED_PORT_CONFIGURATION This value is returned
 * if the phy being added to the port
 */
enum sci_status scic_sds_port_initialize(
	struct scic_sds_port *this_port,
	void *transport_layer_registers,
	void *port_task_scheduler_registers,
	void *port_configuration_regsiter,
	void *viit_registers)
{
	u32 tl_control;

	this_port->transport_layer_registers      = transport_layer_registers;
	this_port->port_task_scheduler_registers  = port_task_scheduler_registers;
	this_port->port_pe_configuration_register = port_configuration_regsiter;
	this_port->viit_registers                 = viit_registers;

	scic_sds_port_set_direct_attached_device_id(
		this_port,
		SCIC_SDS_REMOTE_NODE_CONTEXT_INVALID_INDEX
		);

	/*
	 * Hardware team recommends that we enable the STP prefetch
	 * for all ports */
	tl_control = SCU_TLCR_READ(this_port);
	tl_control |= SCU_TLCR_GEN_BIT(STP_WRITE_DATA_PREFETCH);
	SCU_TLCR_WRITE(this_port, tl_control);

	/*
	 * If this is not the dummy port make the assignment of
	 * the timer and start the state machine */
	if (this_port->physical_port_index != SCI_MAX_PORTS) {
		/* / @todo should we create the timer at create time? */
		this_port->timer_handle = scic_cb_timer_create(
			scic_sds_port_get_controller(this_port),
			scic_sds_port_timeout_handler,
			this_port
			);

	} else {
		/*
		 * Force the dummy port into a condition where it rejects all requests
		 * as its in an invalid state for any operation.
		 * / @todo should we set a set of specical handlers for the dummy port? */
		scic_sds_port_set_base_state_handlers(
			this_port, SCI_BASE_PORT_STATE_STOPPED
			);
	}

	return SCI_SUCCESS;
}

/**
 *
 * @this_port: This is the struct scic_sds_port object for which has a phy that has
 *    gone link up.
 * @the_phy: This is the struct scic_sds_phy object that has gone link up.
 * @do_notify_user: This parameter specifies whether to inform the user (via
 *    scic_cb_port_link_up()) as to the fact that a new phy as become ready.
 *
 * This method is the a general link up handler for the struct scic_sds_port object.
 * This function will determine if this struct scic_sds_phy can be assigned to this
 * struct scic_sds_port object. If the struct scic_sds_phy object can is not a valid PHY for
 * this port then the function will notify the SCIC_USER. A PHY can only be
 * part of a port if it's attached SAS ADDRESS is the same as all other PHYs in
 * the same port. none
 */
void scic_sds_port_general_link_up_handler(
	struct scic_sds_port *this_port,
	struct scic_sds_phy *the_phy,
	bool do_notify_user)
{
	struct sci_sas_address port_sas_address;
	struct sci_sas_address phy_sas_address;

	scic_sds_port_get_attached_sas_address(this_port, &port_sas_address);
	scic_sds_phy_get_attached_sas_address(the_phy, &phy_sas_address);

	/*
	 * If the SAS address of the new phy matches the SAS address of
	 * other phys in the port OR this is the first phy in the port,
	 * then activate the phy and allow it to be used for operations
	 * in this port. */
	if (
		(
			(phy_sas_address.high == port_sas_address.high)
			&& (phy_sas_address.low  == port_sas_address.low)
		)
		|| (this_port->active_phy_mask == 0)
		) {
		scic_sds_port_activate_phy(this_port, the_phy, do_notify_user);

		if (this_port->parent.state_machine.current_state_id
		    == SCI_BASE_PORT_STATE_RESETTING) {
			sci_base_state_machine_change_state(
				&this_port->parent.state_machine, SCI_BASE_PORT_STATE_READY
				);
		}
	} else {
		scic_sds_port_invalid_link_up(this_port, the_phy);
	}
}

/* --------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------- */

enum sci_status scic_port_start(
	SCI_PORT_HANDLE_T handle)
{
	struct scic_sds_port *this_port = (struct scic_sds_port *)handle;

	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_port),
			       SCIC_LOG_OBJECT_PORT,
			       "scic_port_start(0x%x) enter\n",
			       handle
			       ));

	return this_port->state_handlers->parent.start_handler(&this_port->parent);
}

/* --------------------------------------------------------------------------- */

enum sci_status scic_port_stop(
	SCI_PORT_HANDLE_T handle)
{
	struct scic_sds_port *this_port = (struct scic_sds_port *)handle;

	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_port),
			       SCIC_LOG_OBJECT_PORT,
			       "scic_port_stop(0x%x) enter\n",
			       handle
			       ));

	return this_port->state_handlers->parent.stop_handler(&this_port->parent);
}

/* --------------------------------------------------------------------------- */

enum sci_status scic_port_get_properties(
	SCI_PORT_HANDLE_T port,
	struct scic_port_properties *properties)
{
	struct scic_sds_port *this_port = (struct scic_sds_port *)port;

	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_port),
			       SCIC_LOG_OBJECT_PORT,
			       "scic_port_get_properties(0x%x, 0x%x) enter\n",
			       port, properties
			       ));

	if (
		(port == SCI_INVALID_HANDLE)
		|| (this_port->logical_port_index == SCIC_SDS_DUMMY_PORT)
		) {
		return SCI_FAILURE_INVALID_PORT;
	}

	properties->index    = this_port->logical_port_index;
	properties->phy_mask = scic_sds_port_get_phys(this_port);
	scic_sds_port_get_sas_address(this_port, &properties->local.sas_address);
	scic_sds_port_get_protocols(this_port, &properties->local.protocols);
	scic_sds_port_get_attached_sas_address(this_port, &properties->remote.sas_address);
	scic_sds_port_get_attached_protocols(this_port, &properties->remote.protocols);

	return SCI_SUCCESS;
}

/* --------------------------------------------------------------------------- */

enum sci_status scic_port_hard_reset(
	SCI_PORT_HANDLE_T handle,
	u32 reset_timeout)
{
	struct scic_sds_port *this_port = (struct scic_sds_port *)handle;

	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_port),
			       SCIC_LOG_OBJECT_PORT,
			       "scic_port_hard_reset(0x%x, 0x%x) enter\n",
			       handle, reset_timeout
			       ));

	return this_port->state_handlers->parent.reset_handler(
		       &this_port->parent,
		       reset_timeout
		       );
}

/**
 *
 * @this_port: The port for which the direct attached device id is to be
 *    assigned.
 *
 * This method assigns the direct attached device ID for this port.
 */
void scic_sds_port_set_direct_attached_device_id(
	struct scic_sds_port *this_port,
	u32 device_id)
{
	u32 tl_control;

	SCU_STPTLDARNI_WRITE(this_port, device_id);

	/*
	 * The read should guarntee that the first write gets posted
	 * before the next write */
	tl_control = SCU_TLCR_READ(this_port);
	tl_control |= SCU_TLCR_GEN_BIT(CLEAR_TCI_NCQ_MAPPING_TABLE);
	SCU_TLCR_WRITE(this_port, tl_control);
}


/**
 *
 * @this_port: This is the port on which the phy should be enabled.
 * @the_phy: This is the specific phy which to enable.
 * @do_notify_user: This parameter specifies whether to inform the user (via
 *    scic_cb_port_link_up()) as to the fact that a new phy as become ready.
 *
 * This method will activate the phy in the port. Activation includes: - adding
 * the phy to the port - enabling the Protocol Engine in the silicon. -
 * notifying the user that the link is up. none
 */
void scic_sds_port_activate_phy(
	struct scic_sds_port *this_port,
	struct scic_sds_phy *the_phy,
	bool do_notify_user)
{
	struct scic_sds_controller *controller;
	struct sci_sas_identify_address_frame_protocols protocols;

	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_port),
			       SCIC_LOG_OBJECT_PORT,
			       "scic_sds_port_activate_phy(0x%x,0x%x,0x%x) enter\n",
			       this_port, the_phy, do_notify_user
			       ));

	controller = scic_sds_port_get_controller(this_port);
	scic_sds_phy_get_attached_phy_protocols(the_phy, &protocols);

	/* If this is sata port then the phy has already been resumed */
	if (!protocols.u.bits.stp_target) {
		scic_sds_phy_resume(the_phy);
	}

	this_port->active_phy_mask |= 1 << the_phy->phy_index;

	scic_sds_controller_clear_invalid_phy(controller, the_phy);

	if (do_notify_user == true)
		scic_cb_port_link_up(this_port->owning_controller, this_port, the_phy);
}

/**
 *
 * @this_port: This is the port on which the phy should be deactivated.
 * @the_phy: This is the specific phy that is no longer active in the port.
 * @do_notify_user: This parameter specifies whether to inform the user (via
 *    scic_cb_port_link_down()) as to the fact that a new phy as become ready.
 *
 * This method will deactivate the supplied phy in the port. none
 */
void scic_sds_port_deactivate_phy(
	struct scic_sds_port *this_port,
	struct scic_sds_phy *the_phy,
	bool do_notify_user)
{
	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_port),
			       SCIC_LOG_OBJECT_PORT,
			       "scic_sds_port_deactivate_phy(0x%x,0x%x,0x%x) enter\n",
			       this_port, the_phy, do_notify_user
			       ));

	this_port->active_phy_mask &= ~(1 << the_phy->phy_index);

	the_phy->max_negotiated_speed = SCI_SAS_NO_LINK_RATE;

	/* Re-assign the phy back to the LP as if it were a narrow port */
	SCU_PCSPExCR_WRITE(this_port, the_phy->phy_index, the_phy->phy_index);

	if (do_notify_user == true)
		scic_cb_port_link_down(this_port->owning_controller, this_port, the_phy);
}

/**
 *
 * @this_port: This is the port on which the phy should be disabled.
 * @the_phy: This is the specific phy which to disabled.
 *
 * This method will disable the phy and report that the phy is not valid for
 * this port object. None
 */
static void scic_sds_port_invalid_link_up(
	struct scic_sds_port *this_port,
	struct scic_sds_phy *the_phy)
{
	struct scic_sds_controller *controller = scic_sds_port_get_controller(this_port);

	/*
	 * Check to see if we have alreay reported this link as bad and if not go
	 * ahead and tell the SCI_USER that we have discovered an invalid link. */
	if ((controller->invalid_phy_mask & (1 << the_phy->phy_index)) == 0) {
		scic_sds_controller_set_invalid_phy(controller, the_phy);

		scic_cb_port_invalid_link_up(controller, this_port, the_phy);
	}
}

/**
 * This method returns false if the port only has a single phy object assigned.
 *     If there are no phys or more than one phy then the method will return
 *    true.
 * @this_port: The port for which the wide port condition is to be checked.
 *
 * bool true Is returned if this is a wide ported port. false Is returned if
 * this is a narrow port.
 */
static bool scic_sds_port_is_wide(
	struct scic_sds_port *this_port)
{
	u32 index;
	u32 phy_count = 0;

	for (index = 0; index < SCI_MAX_PHYS; index++) {
		if (this_port->phy_table[index] != NULL) {
			phy_count++;
		}
	}

	return phy_count != 1;
}

/**
 * This method is called by the PHY object when the link is detected. if the
 *    port wants the PHY to continue on to the link up state then the port
 *    layer must return true.  If the port object returns false the phy object
 *    must halt its attempt to go link up.
 * @this_port: The port associated with the phy object.
 * @the_phy: The phy object that is trying to go link up.
 *
 * true if the phy object can continue to the link up condition. true Is
 * returned if this phy can continue to the ready state. false Is returned if
 * can not continue on to the ready state. This notification is in place for
 * wide ports and direct attached phys.  Since there are no wide ported SATA
 * devices this could become an invalid port configuration.
 */
bool scic_sds_port_link_detected(
	struct scic_sds_port *this_port,
	struct scic_sds_phy *the_phy)
{
	struct sci_sas_identify_address_frame_protocols protocols;

	scic_sds_phy_get_attached_phy_protocols(the_phy, &protocols);

	if (
		(this_port->logical_port_index != SCIC_SDS_DUMMY_PORT)
		&& (protocols.u.bits.stp_target)
		&& scic_sds_port_is_wide(this_port)
		) {
		scic_sds_port_invalid_link_up(this_port, the_phy);

		return false;
	}

	return true;
}

/**
 * This method is the entry point for the phy to inform the port that it is now
 *    in a ready state
 * @this_port:
 *
 *
 */
void scic_sds_port_link_up(
	struct scic_sds_port *this_port,
	struct scic_sds_phy *the_phy)
{
	the_phy->is_in_link_training = false;

	this_port->state_handlers->link_up_handler(this_port, the_phy);
}

/**
 * This method is the entry point for the phy to inform the port that it is no
 *    longer in a ready state
 * @this_port:
 *
 *
 */
void scic_sds_port_link_down(
	struct scic_sds_port *this_port,
	struct scic_sds_phy *the_phy)
{
	this_port->state_handlers->link_down_handler(this_port, the_phy);
}

/**
 * This method is called to start an IO request on this port.
 * @this_port:
 * @the_device:
 * @the_io_request:
 *
 * enum sci_status
 */
enum sci_status scic_sds_port_start_io(
	struct scic_sds_port *this_port,
	struct scic_sds_remote_device *the_device,
	struct scic_sds_request *the_io_request)
{
	return this_port->state_handlers->start_io_handler(
		       this_port, the_device, the_io_request);
}

/**
 * This method is called to complete an IO request to the port.
 * @this_port:
 * @the_device:
 * @the_io_request:
 *
 * enum sci_status
 */
enum sci_status scic_sds_port_complete_io(
	struct scic_sds_port *this_port,
	struct scic_sds_remote_device *the_device,
	struct scic_sds_request *the_io_request)
{
	return this_port->state_handlers->complete_io_handler(
		       this_port, the_device, the_io_request);
}

/**
 * This method is provided to timeout requests for port operations. Mostly its
 *    for the port reset operation.
 *
 *
 */
static void scic_sds_port_timeout_handler(
	void *port)
{
	u32 current_state;
	struct scic_sds_port *this_port;

	this_port = (struct scic_sds_port *)port;
	current_state = sci_base_state_machine_get_state(
		&this_port->parent.state_machine);

	if (current_state == SCI_BASE_PORT_STATE_RESETTING) {
		/*
		 * if the port is still in the resetting state then the timeout fired
		 * before the reset completed. */
		sci_base_state_machine_change_state(
			&this_port->parent.state_machine,
			SCI_BASE_PORT_STATE_FAILED
			);
	} else if (current_state == SCI_BASE_PORT_STATE_STOPPED) {
		/*
		 * if the port is stopped then the start request failed
		 * In this case stay in the stopped state. */
		SCIC_LOG_ERROR((
				       sci_base_object_get_logger(this_port),
				       SCIC_LOG_OBJECT_PORT,
				       "SCIC Port 0x%x failed to stop before tiemout.\n",
				       this_port
				       ));
	} else if (current_state == SCI_BASE_PORT_STATE_STOPPING) {
		/* if the port is still stopping then the stop has not completed */
		scic_cb_port_stop_complete(
			scic_sds_port_get_controller(this_port),
			port,
			SCI_FAILURE_TIMEOUT
			);
	} else {
		/*
		 * The port is in the ready state and we have a timer reporting a timeout
		 * this should not happen. */
		SCIC_LOG_ERROR((
				       sci_base_object_get_logger(this_port),
				       SCIC_LOG_OBJECT_PORT,
				       "SCIC Port 0x%x is processing a timeout operation in state %d.\n",
				       this_port, current_state
				       ));
	}
}

/* --------------------------------------------------------------------------- */

#ifdef SCIC_DEBUG_ENABLED
void scic_sds_port_decrement_request_count(
	struct scic_sds_port *this_port)
{
	if (this_port->started_request_count == 0) {
		SCIC_LOG_WARNING((
					 sci_base_object_get_logger(this_port),
					 SCIC_LOG_OBJECT_PORT,
					 "SCIC Port object requested to decrement started io count past zero.\n"
					 ));
	} else {
		this_port->started_request_count--;
	}
}
#endif

/**
 * This function updates the hardwares VIIT entry for this port.
 *
 *
 */
void scic_sds_port_update_viit_entry(
	struct scic_sds_port *this_port)
{
	struct sci_sas_address sas_address;

	scic_sds_port_get_sas_address(this_port, &sas_address);

	scu_port_viit_register_write(
		this_port, initiator_sas_address_hi, sas_address.high);

	scu_port_viit_register_write(
		this_port, initiator_sas_address_lo, sas_address.low);

	/* This value get cleared just in case its not already cleared */
	scu_port_viit_register_write(
		this_port, reserved, 0);

	/* We are required to update the status register last */
	scu_port_viit_register_write(
		this_port, status, (
			SCU_VIIT_ENTRY_ID_VIIT
			| SCU_VIIT_IPPT_INITIATOR
			| ((1 << this_port->physical_port_index) << SCU_VIIT_ENTRY_LPVIE_SHIFT)
			| SCU_VIIT_STATUS_ALL_VALID
			)
		);
}

/**
 * This method returns the maximum allowed speed for data transfers on this
 *    port.  This maximum allowed speed evaluates to the maximum speed of the
 *    slowest phy in the port.
 * @this_port: This parameter specifies the port for which to retrieve the
 *    maximum allowed speed.
 *
 * This method returns the maximum negotiated speed of the slowest phy in the
 * port.
 */
enum sci_sas_link_rate scic_sds_port_get_max_allowed_speed(
	struct scic_sds_port *this_port)
{
	u16 index             = 0;
	enum sci_sas_link_rate max_allowed_speed = SCI_SAS_600_GB;
	struct scic_sds_phy *phy               = NULL;

	/*
	 * Loop through all of the phys in this port and find the phy with the
	 * lowest maximum link rate. */
	for (index = 0; index < SCI_MAX_PHYS; index++) {
		phy = this_port->phy_table[index];
		if (
			(phy != NULL)
			&& (scic_sds_port_active_phy(this_port, phy) == true)
			&& (phy->max_negotiated_speed < max_allowed_speed)
			)
			max_allowed_speed = phy->max_negotiated_speed;
	}

	return max_allowed_speed;
}


/**
 * This method passes the event to core user.
 * @this_port: The port that a BCN happens.
 * @this_phy: The phy that receives BCN.
 *
 */
void scic_sds_port_broadcast_change_received(
	struct scic_sds_port *this_port,
	struct scic_sds_phy *this_phy)
{
	/* notify the user. */
	scic_cb_port_bc_change_primitive_recieved(
		this_port->owning_controller, this_port, this_phy
		);
}


/**
 * This API methhod enables the broadcast change notification from underneath
 *    hardware.
 * @this_port: The port that a BCN had been disabled from.
 *
 */
void scic_port_enable_broadcast_change_notification(
	SCI_PORT_HANDLE_T port)
{
	struct scic_sds_port *this_port = (struct scic_sds_port *)port;
	struct scic_sds_phy *phy;
	u32 register_value;
	u8 index;

	/* Loop through all of the phys to enable BCN. */
	for (index = 0; index < SCI_MAX_PHYS; index++) {
		phy = this_port->phy_table[index];
		if (phy != NULL) {
			register_value = SCU_SAS_LLCTL_READ(phy);

			/* clear the bit by writing 1. */
			SCU_SAS_LLCTL_WRITE(phy, register_value);
		}
	}
}

