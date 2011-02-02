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
 * This file contains the implementation of the request handlers for the
 *    struct scic_sds_phy base state machine.
 *
 *
 */

#include "sci_environment.h"
#include "scic_sds_controller.h"
#include "scic_sds_port.h"
#include "scic_sds_phy_registers.h"
#include "scu_event_codes.h"

/*
 * ***************************************************************************
 * *  DEFAULT HANDLERS
 * *************************************************************************** */

/**
 *
 * @phy: This is the struct sci_base_phy object which is cast into a struct scic_sds_phy
 *    object.
 *
 * This is the default method for phy a start request.  It will report a
 * warning and exit. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_phy_default_start_handler(
	struct sci_base_phy *phy)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)phy;

	dev_warn(sciphy_to_dev(this_phy),
		 "%s: SCIC Phy 0x%p requested to start from invalid "
		 "state %d\n",
		 __func__,
		 this_phy,
		 sci_base_state_machine_get_state(
			 &this_phy->parent.state_machine));

	return SCI_FAILURE_INVALID_STATE;

}

/**
 *
 * @phy: This is the struct sci_base_phy object which is cast into a struct scic_sds_phy
 *    object.
 *
 * This is the default method for phy a stop request.  It will report a warning
 * and exit. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_phy_default_stop_handler(
	struct sci_base_phy *phy)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)phy;

	dev_warn(sciphy_to_dev(this_phy),
		 "%s: SCIC Phy 0x%p requested to stop from invalid "
		 "state %d\n",
		 __func__,
		 this_phy,
		 sci_base_state_machine_get_state(
			 &this_phy->parent.state_machine));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @phy: This is the struct sci_base_phy object which is cast into a struct scic_sds_phy
 *    object.
 *
 * This is the default method for phy a reset request.  It will report a
 * warning and exit. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_phy_default_reset_handler(
	struct sci_base_phy *phy)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)phy;

	dev_warn(sciphy_to_dev(this_phy),
		 "%s: SCIC Phy 0x%p requested to reset from invalid state "
		 "%d\n",
		 __func__,
		 this_phy,
		 sci_base_state_machine_get_state(
			 &this_phy->parent.state_machine));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @phy: This is the struct sci_base_phy object which is cast into a struct scic_sds_phy
 *    object.
 *
 * This is the default method for phy a destruct request.  It will report a
 * warning and exit. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_phy_default_destroy_handler(
	struct sci_base_phy *phy)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)phy;

	/* / @todo Implement something for the default */
	dev_warn(sciphy_to_dev(this_phy),
		 "%s: SCIC Phy 0x%p requested to destroy from invalid "
		 "state %d\n",
		 __func__,
		 this_phy,
		 sci_base_state_machine_get_state(
			 &this_phy->parent.state_machine));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @phy: This is the struct sci_base_phy object which is cast into a struct scic_sds_phy
 *    object.
 * @frame_index: This is the frame index that was received from the SCU
 *    hardware.
 *
 * This is the default method for a phy frame handling request.  It will report
 * a warning, release the frame and exit. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_phy_default_frame_handler(
	struct scic_sds_phy *this_phy,
	u32 frame_index)
{
	dev_warn(sciphy_to_dev(this_phy),
		 "%s: SCIC Phy 0x%p recieved unexpected frame data %d "
		 "while in state %d\n",
		 __func__,
		 this_phy,
		 frame_index,
		 sci_base_state_machine_get_state(
			 &this_phy->parent.state_machine));

	scic_sds_controller_release_frame(
		scic_sds_phy_get_controller(this_phy), frame_index);

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @phy: This is the struct sci_base_phy object which is cast into a struct scic_sds_phy
 *    object.
 * @event_code: This is the event code that was received from the SCU hardware.
 *
 * This is the default method for a phy event handler.  It will report a
 * warning and exit. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_phy_default_event_handler(
	struct scic_sds_phy *this_phy,
	u32 event_code)
{
	dev_warn(sciphy_to_dev(this_phy),
		"%s: SCIC Phy 0x%p received unexpected event status %x "
		"while in state %d\n",
		__func__,
		this_phy,
		event_code,
		sci_base_state_machine_get_state(
			&this_phy->parent.state_machine));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @phy: This is the struct sci_base_phy object which is cast into a struct scic_sds_phy
 *    object.
 *
 * This is the default method for a phy consume power handler.  It will report
 * a warning and exit. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_phy_default_consume_power_handler(
	struct scic_sds_phy *this_phy)
{
	dev_warn(sciphy_to_dev(this_phy),
		 "%s: SCIC Phy 0x%p given unexpected permission to consume "
		 "power while in state %d\n",
		 __func__,
		 this_phy,
		 sci_base_state_machine_get_state(
			 &this_phy->parent.state_machine));

	return SCI_FAILURE_INVALID_STATE;
}

/*
 * ******************************************************************************
 * * PHY STOPPED STATE HANDLERS
 * ****************************************************************************** */

/**
 *
 * @phy: This is the struct sci_base_phy object which is cast into a struct scic_sds_phy
 *    object.
 *
 * This method takes the struct scic_sds_phy from a stopped state and attempts to
 * start it. - The phy state machine is transitioned to the
 * SCI_BASE_PHY_STATE_STARTING. enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_phy_stopped_state_start_handler(
	struct sci_base_phy *phy)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)phy;

	sci_base_state_machine_change_state(
		scic_sds_phy_get_base_state_machine(this_phy),
		SCI_BASE_PHY_STATE_STARTING
		);

	return SCI_SUCCESS;
}

/**
 *
 * @phy: This is the struct sci_base_phy object which is cast into a struct scic_sds_phy
 *    object.
 *
 * This method takes the struct scic_sds_phy from a stopped state and destroys it. -
 * This function takes no action. Shouldnt this function transition the
 * struct sci_base_phy::state_machine to the SCI_BASE_PHY_STATE_FINAL? enum sci_status
 * SCI_SUCCESS
 */
static enum sci_status scic_sds_phy_stopped_state_destroy_handler(
	struct sci_base_phy *phy)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)phy;

	/* / @todo what do we actually need to do here? */
	return SCI_SUCCESS;
}

/*
 * ******************************************************************************
 * * PHY STARTING STATE HANDLERS
 * ****************************************************************************** */

/* All of these state handlers are mapped to the starting sub-state machine */

/*
 * ******************************************************************************
 * * PHY READY STATE HANDLERS
 * ****************************************************************************** */

/**
 *
 * @phy: This is the struct sci_base_phy object which is cast into a struct scic_sds_phy
 *    object.
 *
 * This method takes the struct scic_sds_phy from a ready state and attempts to stop
 * it. - The phy state machine is transitioned to the
 * SCI_BASE_PHY_STATE_STOPPED. enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_phy_ready_state_stop_handler(
	struct sci_base_phy *phy)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)phy;

	sci_base_state_machine_change_state(
		scic_sds_phy_get_base_state_machine(this_phy),
		SCI_BASE_PHY_STATE_STOPPED
		);

	return SCI_SUCCESS;
}

/**
 *
 * @phy: This is the struct sci_base_phy object which is cast into a struct scic_sds_phy
 *    object.
 *
 * This method takes the struct scic_sds_phy from a ready state and attempts to reset
 * it. - The phy state machine is transitioned to the
 * SCI_BASE_PHY_STATE_STARTING. enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_phy_ready_state_reset_handler(
	struct sci_base_phy *phy)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)phy;

	sci_base_state_machine_change_state(
		scic_sds_phy_get_base_state_machine(this_phy),
		SCI_BASE_PHY_STATE_RESETTING
		);

	return SCI_SUCCESS;
}

/**
 *
 * @phy: This is the struct scic_sds_phy object which has received the event.
 *
 * This method request the struct scic_sds_phy handle the received event.  The only
 * event that we are interested in while in the ready state is the link failure
 * event. - decoded event is a link failure - transition the struct scic_sds_phy back
 * to the SCI_BASE_PHY_STATE_STARTING state. - any other event recived will
 * report a warning message enum sci_status SCI_SUCCESS if the event received is a
 * link failure SCI_FAILURE_INVALID_STATE for any other event received.
 */
static enum sci_status scic_sds_phy_ready_state_event_handler(
	struct scic_sds_phy *this_phy,
	u32 event_code)
{
	enum sci_status result = SCI_FAILURE;

	switch (scu_get_event_code(event_code)) {
	case SCU_EVENT_LINK_FAILURE:
		/* Link failure change state back to the starting state */
		sci_base_state_machine_change_state(
			scic_sds_phy_get_base_state_machine(this_phy),
			SCI_BASE_PHY_STATE_STARTING
			);

		result = SCI_SUCCESS;
		break;

	case SCU_EVENT_BROADCAST_CHANGE:
		/* Broadcast change received. Notify the port. */
		if (scic_sds_phy_get_port(this_phy) != SCI_INVALID_HANDLE)
			scic_sds_port_broadcast_change_received(this_phy->owning_port, this_phy);
		else
			this_phy->bcn_received_while_port_unassigned = true;
		break;

	default:
		dev_warn(sciphy_to_dev(this_phy),
			 "%sP SCIC PHY 0x%p ready state machine recieved "
			 "unexpected event_code %x\n",
			 __func__,
			 this_phy,
			 event_code);

		result = SCI_FAILURE_INVALID_STATE;
		break;
	}

	return result;
}

/* --------------------------------------------------------------------------- */

/**
 *
 * @this_phy: This is the struct scic_sds_phy object which is receiving the event.
 * @event_code: This is the event code to be processed.
 *
 * This is the resetting state event handler. enum sci_status
 * SCI_FAILURE_INVALID_STATE
 */
static enum sci_status scic_sds_phy_resetting_state_event_handler(
	struct scic_sds_phy *this_phy,
	u32 event_code)
{
	enum sci_status result = SCI_FAILURE;

	switch (scu_get_event_code(event_code)) {
	case SCU_EVENT_HARD_RESET_TRANSMITTED:
		/* Link failure change state back to the starting state */
		sci_base_state_machine_change_state(
			scic_sds_phy_get_base_state_machine(this_phy),
			SCI_BASE_PHY_STATE_STARTING
			);

		result = SCI_SUCCESS;
		break;

	default:
		dev_warn(sciphy_to_dev(this_phy),
			 "%s: SCIC PHY 0x%p resetting state machine recieved "
			 "unexpected event_code %x\n",
			 __func__,
			 this_phy,
			 event_code);

		result = SCI_FAILURE_INVALID_STATE;
		break;
	}

	return result;
}

/* --------------------------------------------------------------------------- */

struct scic_sds_phy_state_handler
scic_sds_phy_state_handler_table[SCI_BASE_PHY_MAX_STATES] =
{
	/* SCI_BASE_PHY_STATE_INITIAL */
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
	/* SCI_BASE_PHY_STATE_STOPPED */
	{
		{
			scic_sds_phy_stopped_state_start_handler,
			scic_sds_phy_default_stop_handler,
			scic_sds_phy_default_reset_handler,
			scic_sds_phy_stopped_state_destroy_handler
		},
		scic_sds_phy_default_frame_handler,
		scic_sds_phy_default_event_handler,
		scic_sds_phy_default_consume_power_handler
	},
	/* SCI_BASE_PHY_STATE_STARTING */
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
	/* SCI_BASE_PHY_STATE_READY */
	{
		{
			scic_sds_phy_default_start_handler,
			scic_sds_phy_ready_state_stop_handler,
			scic_sds_phy_ready_state_reset_handler,
			scic_sds_phy_default_destroy_handler
		},
		scic_sds_phy_default_frame_handler,
		scic_sds_phy_ready_state_event_handler,
		scic_sds_phy_default_consume_power_handler
	},
	/* SCI_BASE_PHY_STATE_RESETTING */
	{
		{
			scic_sds_phy_default_start_handler,
			scic_sds_phy_default_stop_handler,
			scic_sds_phy_default_reset_handler,
			scic_sds_phy_default_destroy_handler
		},
		scic_sds_phy_default_frame_handler,
		scic_sds_phy_resetting_state_event_handler,
		scic_sds_phy_default_consume_power_handler
	},
	/* SCI_BASE_PHY_STATE_FINAL */
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

