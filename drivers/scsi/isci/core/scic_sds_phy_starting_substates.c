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
 * This file implements the starting substate machine for the struct scic_sds_phy.
 *
 *
 */

#include "scic_user_callback.h"
#include "scic_sds_controller.h"
#include "scic_sds_port.h"
#include "scic_sds_phy.h"
#include "scic_sds_phy_registers.h"

/**
 * scic_sds_phy_set_starting_substate_handlers() -
 *
 * This macro sets the starting substate handlers by state_id
 */
#define scic_sds_phy_set_starting_substate_handlers(phy, state_id) \
	scic_sds_phy_set_state_handlers(\
		(phy), \
		&scic_sds_phy_starting_substate_handler_table[(state_id)] \
		)

/*
 * ****************************************************************************
 * *  PHY STARTING SUBSTATE METHODS
 * **************************************************************************** */

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on
 * entering the SCIC_SDS_PHY_STARTING_SUBSTATE_INITIAL. - The initial state
 * handlers are put in place for the struct scic_sds_phy object. - The state is
 * changed to the wait phy type event notification. none
 */
static void scic_sds_phy_starting_initial_substate_enter(
	struct sci_base_object *object)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	scic_sds_phy_set_starting_substate_handlers(
		this_phy, SCIC_SDS_PHY_STARTING_SUBSTATE_INITIAL);

	/* This is just an temporary state go off to the starting state */
	sci_base_state_machine_change_state(
		scic_sds_phy_get_starting_substate_machine(this_phy),
		SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_OSSP_EN
		);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on
 * entering the SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_PHY_TYPE_EN. - Set the
 * struct scic_sds_phy object state handlers for this state. none
 */
static void scic_sds_phy_starting_await_ossp_en_substate_enter(
	struct sci_base_object *object)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	scic_sds_phy_set_starting_substate_handlers(
		this_phy, SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_OSSP_EN
		);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on
 * entering the SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SPEED_EN. - Set the
 * struct scic_sds_phy object state handlers for this state. none
 */
static void scic_sds_phy_starting_await_sas_speed_en_substate_enter(
	struct sci_base_object *object)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	scic_sds_phy_set_starting_substate_handlers(
		this_phy, SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SAS_SPEED_EN
		);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on
 * entering the SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_IAF_UF. - Set the
 * struct scic_sds_phy object state handlers for this state. none
 */
static void scic_sds_phy_starting_await_iaf_uf_substate_enter(
	struct sci_base_object *object)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	scic_sds_phy_set_starting_substate_handlers(
		this_phy, SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_IAF_UF
		);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on
 * entering the SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SAS_POWER. - Set the
 * struct scic_sds_phy object state handlers for this state. - Add this phy object to
 * the power control queue none
 */
static void scic_sds_phy_starting_await_sas_power_substate_enter(
	struct sci_base_object *object)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	scic_sds_phy_set_starting_substate_handlers(
		this_phy, SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SAS_POWER
		);

	scic_sds_controller_power_control_queue_insert(
		scic_sds_phy_get_controller(this_phy),
		this_phy
		);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on exiting
 * the SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SAS_POWER. - Remove the
 * struct scic_sds_phy object from the power control queue. none
 */
static void scic_sds_phy_starting_await_sas_power_substate_exit(
	struct sci_base_object *object)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	scic_sds_controller_power_control_queue_remove(
		scic_sds_phy_get_controller(this_phy), this_phy
		);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on
 * entering the SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_POWER. - Set the
 * struct scic_sds_phy object state handlers for this state. - Add this phy object to
 * the power control queue none
 */
static void scic_sds_phy_starting_await_sata_power_substate_enter(
	struct sci_base_object *object)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	scic_sds_phy_set_starting_substate_handlers(
		this_phy, SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_POWER
		);

	scic_sds_controller_power_control_queue_insert(
		scic_sds_phy_get_controller(this_phy),
		this_phy
		);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on exiting
 * the SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_POWER. - Remove the
 * struct scic_sds_phy object from the power control queue. none
 */
static void scic_sds_phy_starting_await_sata_power_substate_exit(
	struct sci_base_object *object)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	scic_sds_controller_power_control_queue_remove(
		scic_sds_phy_get_controller(this_phy),
		this_phy
		);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on
 * entering the SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_PHY_EN. - Set the
 * struct scic_sds_phy object state handlers for this state. none
 */
static void scic_sds_phy_starting_await_sata_phy_substate_enter(
	struct sci_base_object *object)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	scic_sds_phy_set_starting_substate_handlers(
		this_phy, SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_PHY_EN
		);

	scic_cb_timer_start(
		scic_sds_phy_get_controller(this_phy),
		this_phy->sata_timeout_timer,
		SCIC_SDS_SATA_LINK_TRAINING_TIMEOUT
		);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on exiting
 * the SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_SPEED_EN. - stop the timer
 * that was started on entry to await sata phy event notification none
 */
static void scic_sds_phy_starting_await_sata_phy_substate_exit(
	struct sci_base_object *object)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	scic_cb_timer_stop(
		scic_sds_phy_get_controller(this_phy),
		this_phy->sata_timeout_timer
		);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on
 * entering the SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_SPEED_EN. - Set the
 * struct scic_sds_phy object state handlers for this state. none
 */
static void scic_sds_phy_starting_await_sata_speed_substate_enter(
	struct sci_base_object *object)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	scic_sds_phy_set_starting_substate_handlers(
		this_phy, SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_SPEED_EN
		);

	scic_cb_timer_start(
		scic_sds_phy_get_controller(this_phy),
		this_phy->sata_timeout_timer,
		SCIC_SDS_SATA_LINK_TRAINING_TIMEOUT
		);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on exiting
 * the SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_SPEED_EN. - stop the timer
 * that was started on entry to await sata phy event notification none
 */
static void scic_sds_phy_starting_await_sata_speed_substate_exit(
	struct sci_base_object *object)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	scic_cb_timer_stop(
		scic_sds_phy_get_controller(this_phy),
		this_phy->sata_timeout_timer
		);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on
 * entering the SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SIG_FIS_UF. - Set the
 * struct scic_sds_phy object state handlers for this state. - Start the SIGNATURE FIS
 * timeout timer none
 */
static void scic_sds_phy_starting_await_sig_fis_uf_substate_enter(
	struct sci_base_object *object)
{
	bool continue_to_ready_state;
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	scic_sds_phy_set_starting_substate_handlers(
		this_phy, SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SIG_FIS_UF
		);

	continue_to_ready_state = scic_sds_port_link_detected(
		this_phy->owning_port,
		this_phy
		);

	if (continue_to_ready_state) {
		/*
		 * Clear the PE suspend condition so we can actually receive SIG FIS
		 * The hardware will not respond to the XRDY until the PE suspend
		 * condition is cleared. */
		scic_sds_phy_resume(this_phy);

		scic_cb_timer_start(
			scic_sds_phy_get_controller(this_phy),
			this_phy->sata_timeout_timer,
			SCIC_SDS_SIGNATURE_FIS_TIMEOUT
			);
	} else {
		this_phy->is_in_link_training = false;
	}
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on exiting
 * the SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SIG_FIS_UF. - Stop the SIGNATURE
 * FIS timeout timer. none
 */
static void scic_sds_phy_starting_await_sig_fis_uf_substate_exit(
	struct sci_base_object *object)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	scic_cb_timer_stop(
		scic_sds_phy_get_controller(this_phy),
		this_phy->sata_timeout_timer
		);
}

/**
 *
 * @object: This is the struct sci_base_object which is cast to a struct scic_sds_phy object.
 *
 * This method will perform the actions required by the struct scic_sds_phy on
 * entering the SCIC_SDS_PHY_STARTING_SUBSTATE_FINAL. - Set the struct scic_sds_phy
 * object state handlers for this state. - Change base state machine to the
 * ready state. none
 */
static void scic_sds_phy_starting_final_substate_enter(
	struct sci_base_object *object)
{
	struct scic_sds_phy *this_phy;

	this_phy = (struct scic_sds_phy *)object;

	scic_sds_phy_set_starting_substate_handlers(
		this_phy, SCIC_SDS_PHY_STARTING_SUBSTATE_FINAL
		);

	/*
	 * State machine has run to completion so exit out and change
	 * the base state machine to the ready state */
	sci_base_state_machine_change_state(
		scic_sds_phy_get_base_state_machine(this_phy),
		SCI_BASE_PHY_STATE_READY);
}

/* --------------------------------------------------------------------------- */

struct sci_base_state
scic_sds_phy_starting_substates[SCIC_SDS_PHY_STARTING_MAX_SUBSTATES] =
{
	{
		SCIC_SDS_PHY_STARTING_SUBSTATE_INITIAL,
		scic_sds_phy_starting_initial_substate_enter,
		NULL,
	},
	{
		SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_OSSP_EN,
		scic_sds_phy_starting_await_ossp_en_substate_enter,
		NULL,
	},
	{
		SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SAS_SPEED_EN,
		scic_sds_phy_starting_await_sas_speed_en_substate_enter,
		NULL,
	},
	{
		SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_IAF_UF,
		scic_sds_phy_starting_await_iaf_uf_substate_enter,
		NULL,
	},
	{
		SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SAS_POWER,
		scic_sds_phy_starting_await_sas_power_substate_enter,
		scic_sds_phy_starting_await_sas_power_substate_exit,
	},
	{
		SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_POWER,
		scic_sds_phy_starting_await_sata_power_substate_enter,
		scic_sds_phy_starting_await_sata_power_substate_exit
	},
	{
		SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_PHY_EN,
		scic_sds_phy_starting_await_sata_phy_substate_enter,
		scic_sds_phy_starting_await_sata_phy_substate_exit
	},
	{
		SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SATA_SPEED_EN,
		scic_sds_phy_starting_await_sata_speed_substate_enter,
		scic_sds_phy_starting_await_sata_speed_substate_exit
	},
	{
		SCIC_SDS_PHY_STARTING_SUBSTATE_AWAIT_SIG_FIS_UF,
		scic_sds_phy_starting_await_sig_fis_uf_substate_enter,
		scic_sds_phy_starting_await_sig_fis_uf_substate_exit
	},
	{
		SCIC_SDS_PHY_STARTING_SUBSTATE_FINAL,
		scic_sds_phy_starting_final_substate_enter,
		NULL,
	}
};


