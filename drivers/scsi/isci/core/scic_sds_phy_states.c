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
 * This file contains the implementation of the struct scic_sds_phy base state machine.
 *
 *
 */

#include "sci_base_state.h"
#include "scic_user_callback.h"
#include "scic_sds_port.h"
#include "scic_sds_phy.h"
#include "scic_sds_phy_registers.h"

/*
 * ****************************************************************************
 * *  PHY STATE PRIVATE METHODS
 * **************************************************************************** */

/**
 *
 * @this_phy: This is the struct scic_sds_phy object to stop.
 *
 * This method will stop the struct scic_sds_phy object. This does not reset the
 * protocol engine it just suspends it and places it in a state where it will
 * not cause the end device to power up. none
 */
static void scu_link_layer_stop_protocol_engine(
	struct scic_sds_phy *this_phy)
{
	u32 scu_sas_pcfg_value;
	u32 enable_spinup_value;

	/* Suspend the protocol engine and place it in a sata spinup hold state */
	scu_sas_pcfg_value  = SCU_SAS_PCFG_READ(this_phy);
	scu_sas_pcfg_value |= (
		SCU_SAS_PCFG_GEN_BIT(OOB_RESET)
		| SCU_SAS_PCFG_GEN_BIT(SUSPEND_PROTOCOL_ENGINE)
		| SCU_SAS_PCFG_GEN_BIT(SATA_SPINUP_HOLD)
		);
	SCU_SAS_PCFG_WRITE(this_phy, scu_sas_pcfg_value);

	/* Disable the notify enable spinup primitives */
	enable_spinup_value = SCU_SAS_ENSPINUP_READ(this_phy);
	enable_spinup_value &= ~SCU_ENSPINUP_GEN_BIT(ENABLE);
	SCU_SAS_ENSPINUP_WRITE(this_phy, enable_spinup_value);
}

/**
 *
 *
 * This method will start the OOB/SN state machine for this struct scic_sds_phy object.
 */
static void scu_link_layer_start_oob(
	struct scic_sds_phy *this_phy)
{
	u32 scu_sas_pcfg_value;

	scu_sas_pcfg_value = SCU_SAS_PCFG_READ(this_phy);
	scu_sas_pcfg_value |= SCU_SAS_PCFG_GEN_BIT(OOB_ENABLE);
	scu_sas_pcfg_value &=
		~(SCU_SAS_PCFG_GEN_BIT(OOB_RESET) | SCU_SAS_PCFG_GEN_BIT(HARD_RESET));

	SCU_SAS_PCFG_WRITE(this_phy, scu_sas_pcfg_value);
}

/**
 *
 *
 * This method will transmit a hard reset request on the specified phy. The SCU
 * hardware requires that we reset the OOB state machine and set the hard reset
 * bit in the phy configuration register. We then must start OOB over with the
 * hard reset bit set.
 */
static void scu_link_layer_tx_hard_reset(
	struct scic_sds_phy *this_phy)
{
	u32 phy_configuration_value;

	/*
	 * SAS Phys must wait for the HARD_RESET_TX event notification to transition
	 * to the starting state. */
	phy_configuration_value = SCU_SAS_PCFG_READ(this_phy);
	phy_configuration_value |=
		(SCU_SAS_PCFG_GEN_BIT(HARD_RESET) | SCU_SAS_PCFG_GEN_BIT(OOB_RESET));
	SCU_SAS_PCFG_WRITE(this_phy, phy_configuration_value);

	/* Now take the OOB state machine out of reset */
	phy_configuration_value |= SCU_SAS_PCFG_GEN_BIT(OOB_ENABLE);
	phy_configuration_value &= ~SCU_SAS_PCFG_GEN_BIT(OOB_RESET);
	SCU_SAS_PCFG_WRITE(this_phy, phy_configuration_value);
}

/*
 * ****************************************************************************
 * *  PHY BASE STATE METHODS
 * **************************************************************************** */

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on
 * entering the SCI_BASE_PHY_STATE_INITIAL. - This function sets the state
 * handlers for the phy object base state machine initial state. none
 */
static void scic_sds_phy_initial_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	scic_sds_phy_set_base_state_handlers(this_phy, SCI_BASE_PHY_STATE_STOPPED);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on
 * entering the SCI_BASE_PHY_STATE_INITIAL. - This function sets the state
 * handlers for the phy object base state machine initial state. - The SCU
 * hardware is requested to stop the protocol engine. none
 */
static void scic_sds_phy_stopped_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	/* / @todo We need to get to the controller to place this PE in a reset state */

	scic_sds_phy_set_base_state_handlers(this_phy, SCI_BASE_PHY_STATE_STOPPED);

	scu_link_layer_stop_protocol_engine(this_phy);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on
 * entering the SCI_BASE_PHY_STATE_STARTING. - This function sets the state
 * handlers for the phy object base state machine starting state. - The SCU
 * hardware is requested to start OOB/SN on this protocl engine. - The phy
 * starting substate machine is started. - If the previous state was the ready
 * state then the struct scic_sds_controller is informed that the phy has gone link
 * down. none
 */
static void scic_sds_phy_starting_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	scic_sds_phy_set_base_state_handlers(this_phy, SCI_BASE_PHY_STATE_STARTING);

	scu_link_layer_stop_protocol_engine(this_phy);
	scu_link_layer_start_oob(this_phy);

	/* We don't know what kind of phy we are going to be just yet */
	this_phy->protocol = SCIC_SDS_PHY_PROTOCOL_UNKNOWN;
	this_phy->bcn_received_while_port_unassigned = false;

	/* Change over to the starting substate machine to continue */
	sci_base_state_machine_start(&this_phy->starting_substate_machine);

	if (this_phy->parent.state_machine.previous_state_id
	    == SCI_BASE_PHY_STATE_READY) {
		scic_sds_controller_link_down(
			scic_sds_phy_get_controller(this_phy),
			scic_sds_phy_get_port(this_phy),
			this_phy
			);
	}
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on
 * entering the SCI_BASE_PHY_STATE_READY. - This function sets the state
 * handlers for the phy object base state machine ready state. - The SCU
 * hardware protocol engine is resumed. - The struct scic_sds_controller is informed
 * that the phy object has gone link up. none
 */
static void scic_sds_phy_ready_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	scic_sds_phy_set_base_state_handlers(this_phy, SCI_BASE_PHY_STATE_READY);

	scic_sds_controller_link_up(
		scic_sds_phy_get_controller(this_phy),
		scic_sds_phy_get_port(this_phy),
		this_phy
		);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on exiting
 * the SCI_BASE_PHY_STATE_INITIAL. This function suspends the SCU hardware
 * protocol engine represented by this struct scic_sds_phy object. none
 */
static void scic_sds_phy_ready_state_exit(
	struct sci_base_object *object)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	scic_sds_phy_suspend(this_phy);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on
 * entering the SCI_BASE_PHY_STATE_RESETTING. - This function sets the state
 * handlers for the phy object base state machine resetting state. none
 */
static void scic_sds_phy_resetting_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	scic_sds_phy_set_base_state_handlers(this_phy, SCI_BASE_PHY_STATE_RESETTING);

	/*
	 * The phy is being reset, therefore deactivate it from the port.
	 * In the resetting state we don't notify the user regarding
	 * link up and link down notifications. */
	scic_sds_port_deactivate_phy(this_phy->owning_port, this_phy, false);

	if (this_phy->protocol == SCIC_SDS_PHY_PROTOCOL_SAS) {
		scu_link_layer_tx_hard_reset(this_phy);
	} else {
		/*
		 * The SCU does not need to have a descrete reset state so just go back to
		 * the starting state. */
		sci_base_state_machine_change_state(
			&this_phy->parent.state_machine,
			SCI_BASE_PHY_STATE_STARTING
			);
	}
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on
 * entering the SCI_BASE_PHY_STATE_FINAL. - This function sets the state
 * handlers for the phy object base state machine final state. none
 */
static void scic_sds_phy_final_state_enter(
	struct sci_base_object *object)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	scic_sds_phy_set_base_state_handlers(this_phy, SCI_BASE_PHY_STATE_FINAL);

	/* Nothing to do here */
}

/* --------------------------------------------------------------------------- */

struct sci_base_state scic_sds_phy_state_table[SCI_BASE_PHY_MAX_STATES] =
{
	{
		SCI_BASE_PHY_STATE_INITIAL,
		scic_sds_phy_initial_state_enter,
		NULL,
	},
	{
		SCI_BASE_PHY_STATE_STOPPED,
		scic_sds_phy_stopped_state_enter,
		NULL,
	},
	{
		SCI_BASE_PHY_STATE_STARTING,
		scic_sds_phy_starting_state_enter,
		NULL,
	},
	{
		SCI_BASE_PHY_STATE_READY,
		scic_sds_phy_ready_state_enter,
		scic_sds_phy_ready_state_exit,
	},
	{
		SCI_BASE_PHY_STATE_RESETTING,
		scic_sds_phy_resetting_state_enter,
		NULL,
	},
	{
		SCI_BASE_PHY_STATE_FINAL,
		scic_sds_phy_final_state_enter,
		NULL,
	}
};


