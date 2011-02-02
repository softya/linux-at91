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
 * This file contains the implementation of the struct scic_sds_phy public and
 *    protected methods.
 *
 *
 */

#include "scic_user_callback.h"
#include "scic_phy.h"
#include "scic_sds_phy.h"
#include "scic_sds_port.h"
#include "scic_sds_phy_registers.h"
#include "sci_util.h"
#include "sci_environment.h"

#define SCIC_SDS_PHY_MIN_TIMER_COUNT  (SCI_MAX_PHYS)
#define SCIC_SDS_PHY_MAX_TIMER_COUNT  (SCI_MAX_PHYS)

/* Maximum arbitration wait time in micro-seconds */
#define SCIC_SDS_PHY_MAX_ARBITRATION_WAIT_TIME  (700)

/*
 * *****************************************************************************
 * * SCIC SDS PHY Internal Methods
 * ***************************************************************************** */

/**
 * This method will initialize the phy link layer registers
 * @this_phy:
 * @link_layer_registers:
 *
 * enum sci_status
 */
static enum sci_status scic_sds_phy_link_layer_initialization(
	struct scic_sds_phy *this_phy,
	struct scu_link_layer_registers *link_layer_registers)
{
	u32 phy_configuration;
	struct sas_capabilities phy_capabilities;
	u32 parity_check = 0;
	u32 parity_count = 0;
	u32 link_layer_control;

	this_phy->link_layer_registers = link_layer_registers;

	/* Set our IDENTIFY frame data */
   #define SCI_END_DEVICE 0x01

	SCU_SAS_TIID_WRITE(
		this_phy,
		(SCU_SAS_TIID_GEN_BIT(SMP_INITIATOR)
		 | SCU_SAS_TIID_GEN_BIT(SSP_INITIATOR)
		 | SCU_SAS_TIID_GEN_BIT(STP_INITIATOR)
		 | SCU_SAS_TIID_GEN_BIT(DA_SATA_HOST)
		 | SCU_SAS_TIID_GEN_VAL(DEVICE_TYPE, SCI_END_DEVICE))
		);

	/* Write the device SAS Address */
	SCU_SAS_TIDNH_WRITE(this_phy, 0xFEDCBA98);
	SCU_SAS_TIDNL_WRITE(this_phy, this_phy->phy_index);

	/* Write the source SAS Address */
	SCU_SAS_TISSAH_WRITE(
		this_phy,
		this_phy->owning_port->owning_controller->oem_parameters.sds1.phys[
			this_phy->phy_index].sas_address.high
		);
	SCU_SAS_TISSAL_WRITE(
		this_phy,
		this_phy->owning_port->owning_controller->oem_parameters.sds1.phys[
			this_phy->phy_index].sas_address.low
		);

	/* Clear and Set the PHY Identifier */
	SCU_SAS_TIPID_WRITE(this_phy, 0x00000000);
	SCU_SAS_TIPID_WRITE(this_phy, SCU_SAS_TIPID_GEN_VALUE(ID, this_phy->phy_index));

	/* Change the initial state of the phy configuration register */
	phy_configuration = SCU_SAS_PCFG_READ(this_phy);

	/* Hold OOB state machine in reset */
	phy_configuration |=  SCU_SAS_PCFG_GEN_BIT(OOB_RESET);
	SCU_SAS_PCFG_WRITE(this_phy, phy_configuration);

	/* Configure the SNW capabilities */
	phy_capabilities.u.all = 0;
	phy_capabilities.u.bits.start                      = 1;
	phy_capabilities.u.bits.gen3_without_ssc_supported = 1;
	phy_capabilities.u.bits.gen2_without_ssc_supported = 1;
	phy_capabilities.u.bits.gen1_without_ssc_supported = 1;
	if (this_phy->owning_port->owning_controller->oem_parameters.sds1.
	    controller.do_enable_ssc == true) {
		phy_capabilities.u.bits.gen3_with_ssc_supported = 1;
		phy_capabilities.u.bits.gen2_with_ssc_supported = 1;
		phy_capabilities.u.bits.gen1_with_ssc_supported = 1;
	}

	/*
	 * The SAS specification indicates that the phy_capabilities that
	 * are transmitted shall have an even parity.  Calculate the parity. */
	parity_check = phy_capabilities.u.all;
	while (parity_check != 0) {
		if (parity_check & 0x1)
			parity_count++;
		parity_check >>= 1;
	}

	/*
	 * If parity indicates there are an odd number of bits set, then
	 * set the parity bit to 1 in the phy capabilities. */
	if ((parity_count % 2) != 0)
		phy_capabilities.u.bits.parity = 1;

	SCU_SAS_PHYCAP_WRITE(this_phy, phy_capabilities.u.all);

	/* Set the enable spinup period but disable the ability to send notify enable spinup */
	SCU_SAS_ENSPINUP_WRITE(this_phy, SCU_ENSPINUP_GEN_VAL(COUNT, 0x33));

#if defined(CONFIG_PBG_HBA_A0) || defined(CONFIG_PBG_HBA_A2) || defined(CONFIG_PBG_HBA_BETA)
	/* / @todo Provide a way to write this register correctly */
	scu_link_layer_register_write(this_phy, afe_lookup_table_control, 0x02108421);
#else
	/* / @todo Provide a way to write this register correctly */
	scu_link_layer_register_write(this_phy, afe_lookup_table_control, 0x0e739ce7);
#endif

	link_layer_control = SCU_SAS_LLCTL_GEN_VAL(
		NO_OUTBOUND_TASK_TIMEOUT,
		(u8)this_phy->owning_port->owning_controller->
		user_parameters.sds1.no_outbound_task_timeout
		);

/* #define COMPILED_MAX_LINK_RATE SCU_SAS_LINK_LAYER_CONTROL_MAX_LINK_RATE_GEN1 */
/* #define COMPILED_MAX_LINK_RATE SCU_SAS_LINK_LAYER_CONTROL_MAX_LINK_RATE_GEN2 */
#define COMPILED_MAX_LINK_RATE SCU_SAS_LINK_LAYER_CONTROL_MAX_LINK_RATE_GEN3

	if (this_phy->owning_port->owning_controller->user_parameters.sds1.
	    phys[this_phy->phy_index].max_speed_generation == SCIC_SDS_PARM_GEN3_SPEED) {
		link_layer_control |= SCU_SAS_LLCTL_GEN_VAL(
			MAX_LINK_RATE, COMPILED_MAX_LINK_RATE
			);
	} else if (this_phy->owning_port->owning_controller->user_parameters.sds1.
		   phys[this_phy->phy_index].max_speed_generation == SCIC_SDS_PARM_GEN2_SPEED) {
		link_layer_control |= SCU_SAS_LLCTL_GEN_VAL(
			MAX_LINK_RATE,
			MIN(
				SCU_SAS_LINK_LAYER_CONTROL_MAX_LINK_RATE_GEN2,
				COMPILED_MAX_LINK_RATE)
			);
	} else {
		link_layer_control |= SCU_SAS_LLCTL_GEN_VAL(
			MAX_LINK_RATE,
			MIN(
				SCU_SAS_LINK_LAYER_CONTROL_MAX_LINK_RATE_GEN1,
				COMPILED_MAX_LINK_RATE)
			);
	}

	scu_link_layer_register_write(
		this_phy, link_layer_control, link_layer_control
		);

	/*
	 * Program the max ARB time for the PHY to 700us so we inter-operate with
	 * the PMC expander which shuts down PHYs if the expander PHY generates too
	 * many breaks.  This time value will guarantee that the initiator PHY will
	 * generate the break. */
#if defined(CONFIG_PBG_HBA_A0) || defined(CONFIG_PBG_HBA_A2)
	scu_link_layer_register_write(
		this_phy,
		maximum_arbitration_wait_timer_timeout,
		SCIC_SDS_PHY_MAX_ARBITRATION_WAIT_TIME
		);
#endif  /* defined(CONFIG_PBG_HBA_A0) || defined(CONFIG_PBG_HBA_A2) */

	/*
	 * Set the link layer hang detection to 500ms (0x1F4) from its default
	 * value of 128ms.  Max value is 511 ms. */
	scu_link_layer_register_write(
		this_phy, link_layer_hang_detection_timeout, 0x1F4
		);

	/* We can exit the initial state to the stopped state */
	sci_base_state_machine_change_state(
		scic_sds_phy_get_base_state_machine(this_phy),
		SCI_BASE_PHY_STATE_STOPPED
		);

	return SCI_SUCCESS;
}

/**
 *
 * @cookie: This object is cast to the struct scic_sds_phy object.
 *
 * This function will handle the sata SIGNATURE FIS timeout condition.  It will
 * restart the starting substate machine since we dont know what has actually
 * happening. none
 */
static void scic_sds_phy_sata_timeout(SCI_OBJECT_HANDLE_T cookie)
{
	struct scic_sds_phy *this_phy = (struct scic_sds_phy *)cookie;

	dev_dbg(sciphy_to_dev(this_phy),
		 "%s: SCIC SDS Phy 0x%p did not receive signature fis before "
		 "timeout.\n",
		 __func__,
		 this_phy);

	sci_base_state_machine_stop(
		scic_sds_phy_get_starting_substate_machine(this_phy));

	sci_base_state_machine_change_state(
		scic_sds_phy_get_base_state_machine(this_phy),
		SCI_BASE_PHY_STATE_STARTING
		);
}

/*
 * *****************************************************************************
 * * SCIC SDS PHY External Methods
 * ***************************************************************************** */

/**
 * This method returns the object size for a phy object.
 *
 * u32
 */

/**
 * This method returns the minimum number of timers required for a phy object.
 *
 * u32
 */

/**
 * This method returns the maximum number of timers required for a phy object.
 *
 * u32
 */

#ifdef SCIC_DEBUG_ENABLED
/**
 * scic_sds_phy_observe_state_change() -
 * @our_observer:
 *
 * Debug code to record the state transitions in the phy
 */
void scic_sds_phy_observe_state_change(
	struct sci_base_observer *our_observer,
	struct sci_base_subject *the_subject)
{
	struct scic_sds_phy *this_phy;
	struct sci_base_state_machine *the_state_machine;

	u8 transition_requestor;
	u32 base_state_id;
	u32 starting_substate_id;

	the_state_machine = (struct sci_base_state_machine *)the_subject;
	this_phy = (struct scic_sds_phy *)the_state_machine->state_machine_owner;

	if (the_state_machine == &this_phy->parent.state_machine) {
		transition_requestor = 0x01;
	} else if (the_state_machine == &this_phy->starting_substate_machine) {
		transition_requestor = 0x02;
	} else {
		transition_requestor = 0xFF;
	}

	base_state_id =
		sci_base_state_machine_get_state(&this_phy->parent.state_machine);
	starting_substate_id =
		sci_base_state_machine_get_state(&this_phy->starting_substate_machine);

	this_phy->state_record.state_transition_table[
		this_phy->state_record.index++] = ((transition_requestor << 24)
						   | ((u8)base_state_id << 8)
						   | ((u8)starting_substate_id));

	this_phy->state_record.index =
		this_phy->state_record.index & (MAX_STATE_TRANSITION_RECORD - 1);

}
#endif /* SCIC_DEBUG_ENABLED */

#ifdef SCIC_DEBUG_ENABLED
/**
 * scic_sds_phy_initialize_state_recording() -
 *
 * This method initializes the state record debug information for the phy
 * object. The state machines for the phy object must be constructed before
 * this function is called.
 */
void scic_sds_phy_initialize_state_recording(
	struct scic_sds_phy *this_phy)
{
	this_phy->state_record.index = 0;

	sci_base_observer_initialize(
		&this_phy->state_record.base_state_observer,
		scic_sds_phy_observe_state_change,
		&this_phy->parent.state_machine.parent
		);

	sci_base_observer_initialize(
		&this_phy->state_record.starting_state_observer,
		scic_sds_phy_observe_state_change,
		&this_phy->starting_substate_machine.parent
		);
}
#endif /* SCIC_DEBUG_ENABLED */

/**
 * This method will construct the struct scic_sds_phy object
 * @this_phy:
 * @owning_port:
 * @phy_index:
 *
 */
void scic_sds_phy_construct(
	struct scic_sds_phy *this_phy,
	struct scic_sds_port *owning_port,
	u8 phy_index)
{
	/*
	 * Call the base constructor first
	 */
	sci_base_phy_construct(
		&this_phy->parent,
		scic_sds_phy_state_table
		);

	/* Copy the rest of the input data to our locals */
	this_phy->owning_port = owning_port;
	this_phy->phy_index = phy_index;
	this_phy->bcn_received_while_port_unassigned = false;
	this_phy->protocol = SCIC_SDS_PHY_PROTOCOL_UNKNOWN;
	this_phy->link_layer_registers = NULL;
	this_phy->max_negotiated_speed = SCI_SAS_NO_LINK_RATE;

	/* Clear out the identification buffer data */
	memset(&this_phy->phy_type, 0, sizeof(this_phy->phy_type));

	/* Initialize the the substate machines */
	sci_base_state_machine_construct(
		&this_phy->starting_substate_machine,
		&this_phy->parent.parent,
		scic_sds_phy_starting_substates,
		SCIC_SDS_PHY_STARTING_SUBSTATE_INITIAL
		);

   #ifdef SCIC_DEBUG_ENABLED
	scic_sds_phy_initialize_state_recording(this_phy);
   #endif /* SCIC_DEBUG_ENABLED */
}

/**
 * This method returns the port currently containing this phy. If the phy is
 *    currently contained by the dummy port, then the phy is considered to not
 *    be part of a port.
 * @this_phy: This parameter specifies the phy for which to retrieve the
 *    containing port.
 *
 * This method returns a handle to a port that contains the supplied phy.
 * SCI_INVALID_HANDLE This value is returned if the phy is not part of a real
 * port (i.e. it's contained in the dummy port). !SCI_INVALID_HANDLE All other
 * values indicate a handle/pointer to the port containing the phy.
 */
SCI_PORT_HANDLE_T scic_sds_phy_get_port(
	struct scic_sds_phy *this_phy)
{
	if (scic_sds_port_get_index(this_phy->owning_port) == SCIC_SDS_DUMMY_PORT)
		return SCI_INVALID_HANDLE;

	return this_phy->owning_port;
}

/**
 * This method will assign a port to the phy object.
 * @out]: this_phy This parameter specifies the phy for which to assign a port
 *    object.
 *
 *
 */
void scic_sds_phy_set_port(
	struct scic_sds_phy *this_phy,
	struct scic_sds_port *the_port)
{
	this_phy->owning_port = the_port;

	if (this_phy->bcn_received_while_port_unassigned) {
		this_phy->bcn_received_while_port_unassigned = false;
		scic_sds_port_broadcast_change_received(this_phy->owning_port, this_phy);
	}
}

/**
 * This method will initialize the constructed phy
 * @this_phy:
 * @link_layer_registers:
 *
 * enum sci_status
 */
enum sci_status scic_sds_phy_initialize(
	struct scic_sds_phy *this_phy,
	struct scu_link_layer_registers *link_layer_registers)
{
	/* Create the SIGNATURE FIS Timeout timer for this phy */
	this_phy->sata_timeout_timer = scic_cb_timer_create(
		scic_sds_phy_get_controller(this_phy),
		scic_sds_phy_sata_timeout,
		this_phy
		);

	/* Perofrm the initialization of the PE hardware */
	scic_sds_phy_link_layer_initialization(this_phy, link_layer_registers);

	/*
	 * There is nothing that needs to be done in this state just
	 * transition to the stopped state. */
	sci_base_state_machine_change_state(
		scic_sds_phy_get_base_state_machine(this_phy),
		SCI_BASE_PHY_STATE_STOPPED
		);

	return SCI_SUCCESS;
}


/**
 *
 * @this_phy: The phy object to be suspended.
 *
 * This function will perform the register reads/writes to suspend the SCU
 * hardware protocol engine. none
 */
void scic_sds_phy_suspend(
	struct scic_sds_phy *this_phy)
{
	u32 scu_sas_pcfg_value;

	scu_sas_pcfg_value = SCU_SAS_PCFG_READ(this_phy);

	scu_sas_pcfg_value |= SCU_SAS_PCFG_GEN_BIT(SUSPEND_PROTOCOL_ENGINE);

	SCU_SAS_PCFG_WRITE(this_phy, scu_sas_pcfg_value);
}

/**
 *
 * @this_phy: The phy object to resume.
 *
 * This function will perform the register reads/writes required to resume the
 * SCU hardware protocol engine. none
 */
void scic_sds_phy_resume(
	struct scic_sds_phy *this_phy)
{
	u32 scu_sas_pcfg_value;

	scu_sas_pcfg_value = SCU_SAS_PCFG_READ(this_phy);

	scu_sas_pcfg_value &= ~SCU_SAS_PCFG_GEN_BIT(SUSPEND_PROTOCOL_ENGINE);

	SCU_SAS_PCFG_WRITE(this_phy, scu_sas_pcfg_value);
}

/**
 * This method returns the local sas address assigned to this phy.
 * @this_phy: This parameter specifies the phy for which to retrieve the local
 *    SAS address.
 * @sas_address: This parameter specifies the location into which to copy the
 *    local SAS address.
 *
 */
void scic_sds_phy_get_sas_address(
	struct scic_sds_phy *this_phy,
	struct sci_sas_address *sas_address)
{
	sas_address->high = SCU_SAS_TISSAH_READ(this_phy);
	sas_address->low  = SCU_SAS_TISSAL_READ(this_phy);
}

/**
 * This method returns the remote end-point (i.e. attached) sas address
 *    assigned to this phy.
 * @this_phy: This parameter specifies the phy for which to retrieve the remote
 *    end-point SAS address.
 * @sas_address: This parameter specifies the location into which to copy the
 *    remote end-point SAS address.
 *
 */
void scic_sds_phy_get_attached_sas_address(
	struct scic_sds_phy *this_phy,
	struct sci_sas_address *sas_address)
{
	sas_address->high
		= this_phy->phy_type.sas.identify_address_frame_buffer.sas_address.high;
	sas_address->low
		= this_phy->phy_type.sas.identify_address_frame_buffer.sas_address.low;
}

/**
 * This method returns the supported protocols assigned to this phy
 * @this_phy:
 *
 *
 */
void scic_sds_phy_get_protocols(
	struct scic_sds_phy *this_phy,
	struct sci_sas_identify_address_frame_protocols *protocols)
{
	protocols->u.all = (u16)(SCU_SAS_TIID_READ(this_phy) & 0x0000FFFF);
}

/**
 *
 * @this_phy: The parameter is the phy object for which the attached phy
 *    protcols are to be returned.
 *
 * This method returns the supported protocols for the attached phy.  If this
 * is a SAS phy the protocols are returned from the identify address frame. If
 * this is a SATA phy then protocols are made up and the target phy is an STP
 * target phy. The caller will get the entire set of bits for the protocol
 * value.
 */
void scic_sds_phy_get_attached_phy_protocols(
	struct scic_sds_phy *this_phy,
	struct sci_sas_identify_address_frame_protocols *protocols)
{
	protocols->u.all = 0;

	if (this_phy->protocol == SCIC_SDS_PHY_PROTOCOL_SAS) {
		protocols->u.all =
			this_phy->phy_type.sas.identify_address_frame_buffer.protocols.u.all;
	} else if (this_phy->protocol == SCIC_SDS_PHY_PROTOCOL_SATA) {
		protocols->u.bits.stp_target = 1;
	}
}

/*
 * *****************************************************************************
 * * SCIC SDS PHY Handler Redirects
 * ***************************************************************************** */

/**
 * This method will attempt to start the phy object. This request is only valid
 *    when the phy is in the stopped state
 * @this_phy:
 *
 * enum sci_status
 */
enum sci_status scic_sds_phy_start(
	struct scic_sds_phy *this_phy)
{
	return this_phy->state_handlers->parent.start_handler(&this_phy->parent);
}

/**
 * This method will attempt to stop the phy object.
 * @this_phy:
 *
 * enum sci_status SCI_SUCCESS if the phy is going to stop SCI_INVALID_STATE if the
 * phy is not in a valid state to stop
 */
enum sci_status scic_sds_phy_stop(
	struct scic_sds_phy *this_phy)
{
	return this_phy->state_handlers->parent.stop_handler(&this_phy->parent);
}

/**
 * This method will attempt to reset the phy.  This request is only valid when
 *    the phy is in an ready state
 * @this_phy:
 *
 * enum sci_status
 */
enum sci_status scic_sds_phy_reset(
	struct scic_sds_phy *this_phy)
{
	return this_phy->state_handlers->parent.reset_handler(
		       &this_phy->parent
		       );
}

/**
 * This method will process the event code recieved.
 * @this_phy:
 * @event_code:
 *
 * enum sci_status
 */
enum sci_status scic_sds_phy_event_handler(
	struct scic_sds_phy *this_phy,
	u32 event_code)
{
	return this_phy->state_handlers->event_handler(this_phy, event_code);
}

/**
 * This method will process the frame index recieved.
 * @this_phy:
 * @frame_index:
 *
 * enum sci_status
 */
enum sci_status scic_sds_phy_frame_handler(
	struct scic_sds_phy *this_phy,
	u32 frame_index)
{
	return this_phy->state_handlers->frame_handler(this_phy, frame_index);
}

/**
 * This method will give the phy permission to consume power
 * @this_phy:
 *
 * enum sci_status
 */
enum sci_status scic_sds_phy_consume_power_handler(
	struct scic_sds_phy *this_phy)
{
	return this_phy->state_handlers->consume_power_handler(this_phy);
}

/*
 * *****************************************************************************
 * * SCIC PHY Public Methods
 * ***************************************************************************** */


/* --------------------------------------------------------------------------- */

enum sci_status scic_sas_phy_get_properties(
	SCI_PHY_HANDLE_T phy,
	struct scic_sas_phy_properties *properties)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)phy;

	if (this_phy->protocol == SCIC_SDS_PHY_PROTOCOL_SAS) {
		memcpy(
			&properties->received_iaf,
			&this_phy->phy_type.sas.identify_address_frame_buffer,
			sizeof(struct sci_sas_identify_address_frame)
			);

		properties->received_capabilities.u.all
			= SCU_SAS_RECPHYCAP_READ(this_phy);

		return SCI_SUCCESS;
	}

	return SCI_FAILURE;
}

/* --------------------------------------------------------------------------- */

enum sci_status scic_sata_phy_get_properties(
	SCI_PHY_HANDLE_T phy,
	struct scic_sata_phy_properties *properties)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)phy;

	if (this_phy->protocol == SCIC_SDS_PHY_PROTOCOL_SATA) {
		memcpy(
			&properties->signature_fis,
			&this_phy->phy_type.sata.signature_fis_buffer,
			sizeof(struct sata_fis_reg_d2h)
			);

		/* / @todo add support for port selectors. */
		properties->is_port_selector_present = false;

		return SCI_SUCCESS;
	}

	return SCI_FAILURE;
}

/* --------------------------------------------------------------------------- */




/* --------------------------------------------------------------------------- */



/* --------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------- */



