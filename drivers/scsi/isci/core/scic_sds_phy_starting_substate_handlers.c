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
 * This file implements the state handler methods for the struct scic_sds_phy object
 *    starting substate machine.
 *
 *
 */

#include "sci_util.h"
#include "sci_environment.h"
#include "intel_ata.h"
#include "intel_sata.h"
#include "sci_base_state_machine.h"
#include "scic_sds_controller.h"
#include "scic_sds_port.h"
#include "scic_sds_phy.h"
#include "scic_sds_phy_registers.h"
#include "scu_event_codes.h"

/*
 * *****************************************************************************
 * * SCIC SDS PHY HELPER FUNCTIONS
 * ***************************************************************************** */


/**
 *
 * @this_phy: The phy object that received SAS PHY DETECTED.
 *
 * This method continues the link training for the phy as if it were a SAS PHY
 * instead of a SATA PHY. This is done because the completion queue had a SAS
 * PHY DETECTED event when the state machine was expecting a SATA PHY event.
 * none
 */
static void scic_sds_phy_start_sas_link_training(
	struct scic_sds_phy *this_phy)
{
	u32 phy_control;

	phy_control = SCU_SAS_PCFG_READ(this_phy);
	phy_control |= SCU_SAS_PCFG_GEN_BIT(SATA_SPINUP_HOLD);
	SCU_SAS_PCFG_WRITE(this_phy, phy_control);

	sci_base_state_machine_change_state(
		&this_phy->starting_substate_machine,
		SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SAS_SPEED_EN
		);

	this_phy->protocol = SCIC_SDS_PHY_PROTOCOL_SAS;
}

/**
 *
 * @this_phy: The phy object that received a SATA SPINUP HOLD event
 *
 * This method continues the link training for the phy as if it were a SATA PHY
 * instead of a SAS PHY.  This is done because the completion queue had a SATA
 * SPINUP HOLD event when the state machine was expecting a SAS PHY event. none
 */
static void scic_sds_phy_start_sata_link_training(
	struct scic_sds_phy *this_phy)
{
	sci_base_state_machine_change_state(
		&this_phy->starting_substate_machine,
		SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_POWER
		);

	this_phy->protocol = SCIC_SDS_PHY_PROTOCOL_SATA;
}

/**
 * This method performs processing common to all protocols upon completion of
 *    link training.
 * @this_phy: This parameter specifies the phy object for which link training
 *    has completed.
 * @max_link_rate: This parameter specifies the maximum link rate to be
 *    associated with this phy.
 * @next_state: This parameter specifies the next state for the phy's starting
 *    sub-state machine.
 *
 */
static void scic_sds_phy_complete_link_training(
	struct scic_sds_phy *this_phy,
	enum sci_sas_link_rate max_link_rate,
	u32 next_state)
{
	this_phy->max_negotiated_speed = max_link_rate;

	sci_base_state_machine_change_state(
		scic_sds_phy_get_starting_substate_machine(this_phy), next_state
		);
}

/**
 *
 * @this_phy: The struct scic_sds_phy object to restart.
 *
 * This method restarts the struct scic_sds_phy objects base state machine in the
 * starting state from any starting substate. none
 */
static void scic_sds_phy_restart_starting_state(
	struct scic_sds_phy *this_phy)
{
	/* Stop the current substate machine */
	sci_base_state_machine_stop(
		scic_sds_phy_get_starting_substate_machine(this_phy)
		);

	/* Re-enter the base state machine starting state */
	sci_base_state_machine_change_state(
		scic_sds_phy_get_base_state_machine(this_phy),
		SCI_BASE_PHY_STATE_STARTING
		);
}

/*
 * *****************************************************************************
 * * SCIC SDS PHY EVENT_HANDLERS
 * ***************************************************************************** */

/**
 *
 * @phy: This struct scic_sds_phy object which has received an event.
 * @event_code: This is the event code which the phy object is to decode.
 *
 * This method is called when an event notification is received for the phy
 * object when in the state SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SPEED_EN. -
 * decode the event - sas phy detected causes a state transition to the wait
 * for speed event notification. - any other events log a warning message and
 * set a failure status enum sci_status SCI_SUCCESS on any valid event notification
 * SCI_FAILURE on any unexpected event notifation
 */
static enum sci_status scic_sds_phy_starting_substate_await_ossp_event_handler(
	struct scic_sds_phy *this_phy,
	u32 event_code)
{
	u32 result = SCI_SUCCESS;

	switch (scu_get_event_code(event_code)) {
	case SCU_EVENT_SAS_PHY_DETECTED:
		scic_sds_phy_start_sas_link_training(this_phy);
		this_phy->is_in_link_training = true;
		break;

	case SCU_EVENT_SATA_SPINUP_HOLD:
		scic_sds_phy_start_sata_link_training(this_phy);
		this_phy->is_in_link_training = true;
		break;

	default:
		dev_warn(sciphy_to_dev(this_phy),
			 "%s: PHY starting substate machine recieved "
			 "unexpected event_code %x\n",
			 __func__,
			 event_code);

		result = SCI_FAILURE;
		break;
	}

	return result;
}

/**
 *
 * @phy: This struct scic_sds_phy object which has received an event.
 * @event_code: This is the event code which the phy object is to decode.
 *
 * This method is called when an event notification is received for the phy
 * object when in the state SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SPEED_EN. -
 * decode the event - sas phy detected returns us back to this state. - speed
 * event detected causes a state transition to the wait for iaf. - identify
 * timeout is an un-expected event and the state machine is restarted. - link
 * failure events restart the starting state machine - any other events log a
 * warning message and set a failure status enum sci_status SCI_SUCCESS on any valid
 * event notification SCI_FAILURE on any unexpected event notifation
 */
static enum sci_status scic_sds_phy_starting_substate_await_sas_phy_speed_event_handler(
	struct scic_sds_phy *this_phy,
	u32 event_code)
{
	u32 result = SCI_SUCCESS;

	switch (scu_get_event_code(event_code)) {
	case SCU_EVENT_SAS_PHY_DETECTED:
		/*
		 * Why is this being reported again by the controller?
		 * We would re-enter this state so just stay here */
		break;

	case SCU_EVENT_SAS_15:
	case SCU_EVENT_SAS_15_SSC:
		scic_sds_phy_complete_link_training(
			this_phy, SCI_SAS_150_GB, SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_IAF_UF
			);
		break;

	case SCU_EVENT_SAS_30:
	case SCU_EVENT_SAS_30_SSC:
		scic_sds_phy_complete_link_training(
			this_phy, SCI_SAS_300_GB, SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_IAF_UF
			);
		break;

	case SCU_EVENT_SAS_60:
	case SCU_EVENT_SAS_60_SSC:
		scic_sds_phy_complete_link_training(
			this_phy, SCI_SAS_600_GB, SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_IAF_UF
			);
		break;

	case SCU_EVENT_SATA_SPINUP_HOLD:
		/*
		 * We were doing SAS PHY link training and received a SATA PHY event
		 * continue OOB/SN as if this were a SATA PHY */
		scic_sds_phy_start_sata_link_training(this_phy);
		break;

	case SCU_EVENT_LINK_FAILURE:
		/* Link failure change state back to the starting state */
		scic_sds_phy_restart_starting_state(this_phy);
		break;

	default:
		dev_warn(sciphy_to_dev(this_phy),
			 "%s: PHY starting substate machine recieved "
			 "unexpected event_code %x\n",
			 __func__,
			 event_code);

		result = SCI_FAILURE;
		break;
	}

	return result;
}

/**
 *
 * @phy: This struct scic_sds_phy object which has received an event.
 * @event_code: This is the event code which the phy object is to decode.
 *
 * This method is called when an event notification is received for the phy
 * object when in the state SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_IAF_UF. -
 * decode the event - sas phy detected event backs up the state machine to the
 * await speed notification. - identify timeout is an un-expected event and the
 * state machine is restarted. - link failure events restart the starting state
 * machine - any other events log a warning message and set a failure status
 * enum sci_status SCI_SUCCESS on any valid event notification SCI_FAILURE on any
 * unexpected event notifation
 */
static enum sci_status scic_sds_phy_starting_substate_await_iaf_uf_event_handler(
	struct scic_sds_phy *this_phy,
	u32 event_code)
{
	u32 result = SCI_SUCCESS;

	switch (scu_get_event_code(event_code)) {
	case SCU_EVENT_SAS_PHY_DETECTED:
		/* Backup the state machine */
		scic_sds_phy_start_sas_link_training(this_phy);
		break;

	case SCU_EVENT_SATA_SPINUP_HOLD:
		/*
		 * We were doing SAS PHY link training and received a SATA PHY event
		 * continue OOB/SN as if this were a SATA PHY */
		scic_sds_phy_start_sata_link_training(this_phy);
		break;

	case SCU_EVENT_RECEIVED_IDENTIFY_TIMEOUT:
	case SCU_EVENT_LINK_FAILURE:
	case SCU_EVENT_HARD_RESET_RECEIVED:
		/* Start the oob/sn state machine over again */
		scic_sds_phy_restart_starting_state(this_phy);
		break;

	default:
		dev_warn(sciphy_to_dev(this_phy),
			 "%s: PHY starting substate machine recieved "
			 "unexpected event_code %x\n",
			 __func__,
			 event_code);

		result = SCI_FAILURE;
		break;
	}

	return result;
}

/**
 *
 * @phy: This struct scic_sds_phy object which has received an event.
 * @event_code: This is the event code which the phy object is to decode.
 *
 * This method is called when an event notification is received for the phy
 * object when in the state SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_POWER. -
 * decode the event - link failure events restart the starting state machine -
 * any other events log a warning message and set a failure status enum sci_status
 * SCI_SUCCESS on a link failure event SCI_FAILURE on any unexpected event
 * notifation
 */
static enum sci_status scic_sds_phy_starting_substate_await_sas_power_event_handler(
	struct scic_sds_phy *this_phy,
	u32 event_code)
{
	u32 result = SCI_SUCCESS;

	switch (scu_get_event_code(event_code)) {
	case SCU_EVENT_LINK_FAILURE:
		/* Link failure change state back to the starting state */
		scic_sds_phy_restart_starting_state(this_phy);
		break;

	default:
		dev_warn(sciphy_to_dev(this_phy),
			"%s: PHY starting substate machine recieved unexpected "
			"event_code %x\n",
			__func__,
			event_code);

		result = SCI_FAILURE;
		break;
	}

	return result;
}

/**
 *
 * @phy: This struct scic_sds_phy object which has received an event.
 * @event_code: This is the event code which the phy object is to decode.
 *
 * This method is called when an event notification is received for the phy
 * object when in the state SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_POWER. -
 * decode the event - link failure events restart the starting state machine -
 * sata spinup hold events are ignored since they are expected - any other
 * events log a warning message and set a failure status enum sci_status SCI_SUCCESS
 * on a link failure event SCI_FAILURE on any unexpected event notifation
 */
static enum sci_status scic_sds_phy_starting_substate_await_sata_power_event_handler(
	struct scic_sds_phy *this_phy,
	u32 event_code)
{
	u32 result = SCI_SUCCESS;

	switch (scu_get_event_code(event_code)) {
	case SCU_EVENT_LINK_FAILURE:
		/* Link failure change state back to the starting state */
		scic_sds_phy_restart_starting_state(this_phy);
		break;

	case SCU_EVENT_SATA_SPINUP_HOLD:
		/* These events are received every 10ms and are expected while in this state */
		break;

	case SCU_EVENT_SAS_PHY_DETECTED:
		/*
		 * There has been a change in the phy type before OOB/SN for the
		 * SATA finished start down the SAS link traning path. */
		scic_sds_phy_start_sas_link_training(this_phy);
		break;

	default:
		dev_warn(sciphy_to_dev(this_phy),
			 "%s: PHY starting substate machine recieved "
			 "unexpected event_code %x\n",
			 __func__,
			 event_code);

		result = SCI_FAILURE;
		break;
	}

	return result;
}

/**
 *
 * @phy: This struct scic_sds_phy object which has received an event.
 * @event_code: This is the event code which the phy object is to decode.
 *
 * This method is called when an event notification is received for the phy
 * object when in the state SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_PHY_EN. -
 * decode the event - link failure events restart the starting state machine -
 * sata spinup hold events are ignored since they are expected - sata phy
 * detected event change to the wait speed event - any other events log a
 * warning message and set a failure status enum sci_status SCI_SUCCESS on a link
 * failure event SCI_FAILURE on any unexpected event notifation
 */
static enum sci_status scic_sds_phy_starting_substate_await_sata_phy_event_handler(
	struct scic_sds_phy *this_phy,
	u32 event_code)
{
	u32 result = SCI_SUCCESS;

	switch (scu_get_event_code(event_code)) {
	case SCU_EVENT_LINK_FAILURE:
		/* Link failure change state back to the starting state */
		scic_sds_phy_restart_starting_state(this_phy);
		break;

	case SCU_EVENT_SATA_SPINUP_HOLD:
		/*
		 * These events might be received since we dont know how many may be in
		 * the completion queue while waiting for power */
		break;

	case SCU_EVENT_SATA_PHY_DETECTED:
		this_phy->protocol = SCIC_SDS_PHY_PROTOCOL_SATA;

		/* We have received the SATA PHY notification change state */
		sci_base_state_machine_change_state(
			scic_sds_phy_get_starting_substate_machine(this_phy),
			SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_SPEED_EN
			);
		break;

	case SCU_EVENT_SAS_PHY_DETECTED:
		/*
		 * There has been a change in the phy type before OOB/SN for the
		 * SATA finished start down the SAS link traning path. */
		scic_sds_phy_start_sas_link_training(this_phy);
		break;

	default:
		dev_warn(sciphy_to_dev(this_phy),
			 "%s: PHY starting substate machine recieved "
			 "unexpected event_code %x\n",
			 __func__,
			 event_code);

		result = SCI_FAILURE;
		break;
	}

	return result;
}

/**
 *
 * @phy: This struct scic_sds_phy object which has received an event.
 * @event_code: This is the event code which the phy object is to decode.
 *
 * This method is called when an event notification is received for the phy
 * object when in the state SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_SPEED_EN.
 * - decode the event - sata phy detected returns us back to this state. -
 * speed event detected causes a state transition to the wait for signature. -
 * link failure events restart the starting state machine - any other events
 * log a warning message and set a failure status enum sci_status SCI_SUCCESS on any
 * valid event notification SCI_FAILURE on any unexpected event notifation
 */
static enum sci_status scic_sds_phy_starting_substate_await_sata_speed_event_handler(
	struct scic_sds_phy *this_phy,
	u32 event_code)
{
	u32 result = SCI_SUCCESS;

	switch (scu_get_event_code(event_code)) {
	case SCU_EVENT_SATA_PHY_DETECTED:
		/*
		 * The hardware reports multiple SATA PHY detected events
		 * ignore the extras */
		break;

	case SCU_EVENT_SATA_15:
	case SCU_EVENT_SATA_15_SSC:
		scic_sds_phy_complete_link_training(
			this_phy,
			SCI_SAS_150_GB,
			SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SIG_FIS_UF
			);
		break;

	case SCU_EVENT_SATA_30:
	case SCU_EVENT_SATA_30_SSC:
		scic_sds_phy_complete_link_training(
			this_phy,
			SCI_SAS_300_GB,
			SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SIG_FIS_UF
			);
		break;

	case SCU_EVENT_SATA_60:
	case SCU_EVENT_SATA_60_SSC:
		scic_sds_phy_complete_link_training(
			this_phy,
			SCI_SAS_600_GB,
			SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SIG_FIS_UF
			);
		break;

	case SCU_EVENT_LINK_FAILURE:
		/* Link failure change state back to the starting state */
		scic_sds_phy_restart_starting_state(this_phy);
		break;

	case SCU_EVENT_SAS_PHY_DETECTED:
		/*
		 * There has been a change in the phy type before OOB/SN for the
		 * SATA finished start down the SAS link traning path. */
		scic_sds_phy_start_sas_link_training(this_phy);
		break;

	default:
		dev_warn(sciphy_to_dev(this_phy),
			 "%s: PHY starting substate machine recieved "
			 "unexpected event_code %x\n",
			 __func__,
			 event_code);

		result = SCI_FAILURE;
		break;
	}

	return result;
}

/**
 *
 * @phy: This struct scic_sds_phy object which has received an event.
 * @event_code: This is the event code which the phy object is to decode.
 *
 * This method is called when an event notification is received for the phy
 * object when in the state SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SIG_FIS_UF. -
 * decode the event - sas phy detected event backs up the state machine to the
 * await speed notification. - identify timeout is an un-expected event and the
 * state machine is restarted. - link failure events restart the starting state
 * machine - any other events log a warning message and set a failure status
 * enum sci_status SCI_SUCCESS on any valid event notification SCI_FAILURE on any
 * unexpected event notifation
 */
static enum sci_status scic_sds_phy_starting_substate_await_sig_fis_event_handler(
	struct scic_sds_phy *this_phy,
	u32 event_code)
{
	u32 result = SCI_SUCCESS;

	switch (scu_get_event_code(event_code)) {
	case SCU_EVENT_SATA_PHY_DETECTED:
		/* Backup the state machine */
		sci_base_state_machine_change_state(
			scic_sds_phy_get_starting_substate_machine(this_phy),
			SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_SPEED_EN
			);
		break;

	case SCU_EVENT_LINK_FAILURE:
		/* Link failure change state back to the starting state */
		scic_sds_phy_restart_starting_state(this_phy);
		break;

	default:
		dev_warn(sciphy_to_dev(this_phy),
			 "%s: PHY starting substate machine recieved "
			 "unexpected event_code %x\n",
			 __func__,
			 event_code);

		result = SCI_FAILURE;
		break;
	}

	return result;
}


/*
 * *****************************************************************************
 * *  SCIC SDS PHY FRAME_HANDLERS
 * ***************************************************************************** */

/**
 *
 * @phy: This is struct scic_sds_phy object which is being requested to decode the
 *    frame data.
 * @frame_index: This is the index of the unsolicited frame which was received
 *    for this phy.
 *
 * This method decodes the unsolicited frame when the struct scic_sds_phy is in the
 * SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_IAF_UF. - Get the UF Header - If the UF
 * is an IAF - Copy IAF data to local phy object IAF data buffer. - Change
 * starting substate to wait power. - else - log warning message of unexpected
 * unsolicted frame - release frame buffer enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_phy_starting_substate_await_iaf_uf_frame_handler(
	struct scic_sds_phy *this_phy,
	u32 frame_index)
{
	enum sci_status result;
	u32 *frame_words;
	struct sci_sas_identify_address_frame *identify_frame;

	result = scic_sds_unsolicited_frame_control_get_header(
		&(scic_sds_phy_get_controller(this_phy)->uf_control),
		frame_index,
		(void **)&frame_words);

	if (result != SCI_SUCCESS) {
		return result;
	}

	frame_words[0] = SCIC_SWAP_DWORD(frame_words[0]);
	identify_frame = (struct sci_sas_identify_address_frame *)frame_words;

	if (identify_frame->address_frame_type == 0) {
		/*
		 * Byte swap the rest of the frame so we can make
		 * a copy of the buffer */
		frame_words[1] = SCIC_SWAP_DWORD(frame_words[1]);
		frame_words[2] = SCIC_SWAP_DWORD(frame_words[2]);
		frame_words[3] = SCIC_SWAP_DWORD(frame_words[3]);
		frame_words[4] = SCIC_SWAP_DWORD(frame_words[4]);
		frame_words[5] = SCIC_SWAP_DWORD(frame_words[5]);

		memcpy(
			&this_phy->phy_type.sas.identify_address_frame_buffer,
			identify_frame,
			sizeof(struct sci_sas_identify_address_frame)
			);

		if (identify_frame->protocols.u.bits.smp_target) {
			/*
			 * We got the IAF for an expander PHY go to the final state since
			 * there are no power requirements for expander phys. */
			sci_base_state_machine_change_state(
				scic_sds_phy_get_starting_substate_machine(this_phy),
				SCIC_SDS_PHY_STARTING_SUBSTATE_FINAL
				);
		} else {
			/* We got the IAF we can now go to the await spinup semaphore state */
			sci_base_state_machine_change_state(
				scic_sds_phy_get_starting_substate_machine(this_phy),
				SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SAS_POWER
				);
		}

		result = SCI_SUCCESS;
	} else
		dev_warn(sciphy_to_dev(this_phy),
			"%s: PHY starting substate machine recieved "
			"unexpected frame id %x\n",
			__func__,
			frame_index);

	/* Regardless of the result release this frame since we are done with it */
	scic_sds_controller_release_frame(
		scic_sds_phy_get_controller(this_phy), frame_index
		);

	return result;
}

/**
 *
 * @phy: This is struct scic_sds_phy object which is being requested to decode the
 *    frame data.
 * @frame_index: This is the index of the unsolicited frame which was received
 *    for this phy.
 *
 * This method decodes the unsolicited frame when the struct scic_sds_phy is in the
 * SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SIG_FIS_UF. - Get the UF Header - If
 * the UF is an SIGNATURE FIS - Copy IAF data to local phy object SIGNATURE FIS
 * data buffer. - else - log warning message of unexpected unsolicted frame -
 * release frame buffer enum sci_status SCI_SUCCESS Must decode the SIGNATURE FIS
 * data
 */
static enum sci_status scic_sds_phy_starting_substate_await_sig_fis_frame_handler(
	struct scic_sds_phy *this_phy,
	u32 frame_index)
{
	enum sci_status result;
	u32 *frame_words;
	struct sata_fis_header *fis_frame_header;
	u32 *fis_frame_data;

	result = scic_sds_unsolicited_frame_control_get_header(
		&(scic_sds_phy_get_controller(this_phy)->uf_control),
		frame_index,
		(void **)&frame_words);

	if (result != SCI_SUCCESS) {
		return result;
	}

	fis_frame_header = (struct sata_fis_header *)frame_words;

	if (
		(fis_frame_header->fis_type == SATA_FIS_TYPE_REGD2H)
		&& !(fis_frame_header->status & ATA_STATUS_REG_BSY_BIT)
		) {
		scic_sds_unsolicited_frame_control_get_buffer(
			&(scic_sds_phy_get_controller(this_phy)->uf_control),
			frame_index,
			(void **)&fis_frame_data
			);

		scic_sds_controller_copy_sata_response(
			&this_phy->phy_type.sata.signature_fis_buffer,
			frame_words,
			fis_frame_data
			);

		/* We got the IAF we can now go to the await spinup semaphore state */
		sci_base_state_machine_change_state(
			scic_sds_phy_get_starting_substate_machine(this_phy),
			SCIC_SDS_PHY_STARTING_SUBSTATE_FINAL
			);

		result = SCI_SUCCESS;
	} else
		dev_warn(sciphy_to_dev(this_phy),
			 "%s: PHY starting substate machine recieved "
			 "unexpected frame id %x\n",
			 __func__,
			 frame_index);

	/* Regardless of the result release this frame since we are done with it */
	scic_sds_controller_release_frame(
		scic_sds_phy_get_controller(this_phy), frame_index
		);

	return result;
}

/*
 * *****************************************************************************
 * * SCIC SDS PHY POWER_HANDLERS
 * ***************************************************************************** */

/**
 *
 * @phy: This is the struct sci_base_phy object which is cast into a struct scic_sds_phy
 *    object.
 *
 * This method is called by the struct scic_sds_controller when the phy object is
 * granted power. - The notify enable spinups are turned on for this phy object
 * - The phy state machine is transitioned to the
 * SCIC_SDS_PHY_STARTING_SUBSTATE_FINAL. enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_phy_starting_substate_await_sas_power_consume_power_handler(
	struct scic_sds_phy *this_phy)
{
	u32 enable_spinup;

	enable_spinup = SCU_SAS_ENSPINUP_READ(this_phy);
	enable_spinup |= SCU_ENSPINUP_GEN_BIT(ENABLE);
	SCU_SAS_ENSPINUP_WRITE(this_phy, enable_spinup);

	/* Change state to the final state this substate machine has run to completion */
	sci_base_state_machine_change_state(
		scic_sds_phy_get_starting_substate_machine(this_phy),
		SCIC_SDS_PHY_STARTING_SUBSTATE_FINAL
		);

	return SCI_SUCCESS;
}

/**
 *
 * @phy: This is the struct sci_base_phy object which is cast into a struct scic_sds_phy
 *    object.
 *
 * This method is called by the struct scic_sds_controller when the phy object is
 * granted power. - The phy state machine is transitioned to the
 * SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_PHY_EN. enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_phy_starting_substate_await_sata_power_consume_power_handler(
	struct scic_sds_phy *this_phy)
{
	u32 scu_sas_pcfg_value;

	/* Release the spinup hold state and reset the OOB state machine */
	scu_sas_pcfg_value = SCU_SAS_PCFG_READ(this_phy);
	scu_sas_pcfg_value &=
		~(SCU_SAS_PCFG_GEN_BIT(SATA_SPINUP_HOLD) | SCU_SAS_PCFG_GEN_BIT(OOB_ENABLE));
	scu_sas_pcfg_value |= SCU_SAS_PCFG_GEN_BIT(OOB_RESET);
	SCU_SAS_PCFG_WRITE(this_phy, scu_sas_pcfg_value);

	/* Now restart the OOB operation */
	scu_sas_pcfg_value &= ~SCU_SAS_PCFG_GEN_BIT(OOB_RESET);
	scu_sas_pcfg_value |= SCU_SAS_PCFG_GEN_BIT(OOB_ENABLE);
	SCU_SAS_PCFG_WRITE(this_phy, scu_sas_pcfg_value);

	/* Change state to the final state this substate machine has run to completion */
	sci_base_state_machine_change_state(
		scic_sds_phy_get_starting_substate_machine(this_phy),
		SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_PHY_EN
		);

	return SCI_SUCCESS;
}

/* --------------------------------------------------------------------------- */

struct scic_sds_phy_state_handler
scic_sds_phy_starting_substate_handler_table[SCIC_SDS_PHY_STARTING_MAX_SUBSTATES] =
{
	/* SCIC_SDS_PHY_STARTING_SUBSTATE_INITIAL */
	{
		{
			scic_sds_phy_default_start_handler,
			scic_sds_phy_default_stop_handler,
			scic_sds_phy_default_reset_handler,
			scic_sds_phy_default_destroy_handler
		},
		scic_sds_phy_default_frame_handler,
		scic_sds_phy_default_event_handler,
		scic_sds_phy_default_consume_power_handler
	},
	/* SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_OSSP_EN */
	{
		{
			scic_sds_phy_default_start_handler,
			scic_sds_phy_default_stop_handler,
			scic_sds_phy_default_reset_handler,
			scic_sds_phy_default_destroy_handler
		},
		scic_sds_phy_default_frame_handler,
		scic_sds_phy_starting_substate_await_ossp_event_handler,
		scic_sds_phy_default_consume_power_handler
	},
	/* SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SAS_SPEED_EN */
	{
		{
			scic_sds_phy_default_start_handler,
			scic_sds_phy_default_stop_handler,
			scic_sds_phy_default_reset_handler,
			scic_sds_phy_default_destroy_handler
		},
		scic_sds_phy_default_frame_handler,
		scic_sds_phy_starting_substate_await_sas_phy_speed_event_handler,
		scic_sds_phy_default_consume_power_handler
	},
	/* SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_IAF_UF */
	{
		{
			scic_sds_phy_default_start_handler,
			scic_sds_phy_default_stop_handler,
			scic_sds_phy_default_reset_handler,
			scic_sds_phy_default_destroy_handler
		},
		scic_sds_phy_starting_substate_await_iaf_uf_frame_handler,
		scic_sds_phy_starting_substate_await_iaf_uf_event_handler,
		scic_sds_phy_default_consume_power_handler
	},
	/* SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SAS_POWER */
	{
		{
			scic_sds_phy_default_start_handler,
			scic_sds_phy_default_stop_handler,
			scic_sds_phy_default_reset_handler,
			scic_sds_phy_default_destroy_handler
		},
		scic_sds_phy_default_frame_handler,
		scic_sds_phy_starting_substate_await_sas_power_event_handler,
		scic_sds_phy_starting_substate_await_sas_power_consume_power_handler
	},
	/* SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_POWER, */
	{
		{
			scic_sds_phy_default_start_handler,
			scic_sds_phy_default_stop_handler,
			scic_sds_phy_default_reset_handler,
			scic_sds_phy_default_destroy_handler
		},
		scic_sds_phy_default_frame_handler,
		scic_sds_phy_starting_substate_await_sata_power_event_handler,
		scic_sds_phy_starting_substate_await_sata_power_consume_power_handler
	},
	/* SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_PHY_EN, */
	{
		{
			scic_sds_phy_default_start_handler,
			scic_sds_phy_default_stop_handler,
			scic_sds_phy_default_reset_handler,
			scic_sds_phy_default_destroy_handler
		},
		scic_sds_phy_default_frame_handler,
		scic_sds_phy_starting_substate_await_sata_phy_event_handler,
		scic_sds_phy_default_consume_power_handler
	},
	/* SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_SPEED_EN, */
	{
		{
			scic_sds_phy_default_start_handler,
			scic_sds_phy_default_stop_handler,
			scic_sds_phy_default_reset_handler,
			scic_sds_phy_default_destroy_handler
		},
		scic_sds_phy_default_frame_handler,
		scic_sds_phy_starting_substate_await_sata_speed_event_handler,
		scic_sds_phy_default_consume_power_handler
	},
	/* SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SIG_FIS_UF, */
	{
		{
			scic_sds_phy_default_start_handler,
			scic_sds_phy_default_stop_handler,
			scic_sds_phy_default_reset_handler,
			scic_sds_phy_default_destroy_handler
		},
		scic_sds_phy_starting_substate_await_sig_fis_frame_handler,
		scic_sds_phy_starting_substate_await_sig_fis_event_handler,
		scic_sds_phy_default_consume_power_handler
	},
	/* SCIC_SDS_PHY_STARTING_SUBSTATE_FINAL */
	{
		{
			scic_sds_phy_default_start_handler,
			scic_sds_phy_default_stop_handler,
			scic_sds_phy_default_reset_handler,
			scic_sds_phy_default_destroy_handler
		},
		scic_sds_phy_default_frame_handler,
		scic_sds_phy_default_event_handler,
		scic_sds_phy_default_consume_power_handler
	}
};

