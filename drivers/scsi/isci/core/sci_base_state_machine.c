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
 * This file contains all of the functionality common to all state machine
 *    object implementations.
 *
 *
 */

#include "sci_base_state_machine.h"

#define SCI_STATE_MACHINE_EXIT_STATE(state_machine) \
	if (\
		((state_machine)->state_table[(state_machine)->current_state_id]. \
		 exit_state != NULL) \
		) { \
		((state_machine)->state_table[(state_machine)->current_state_id]. \
		 exit_state((state_machine)->state_machine_owner)); \
	}

#define SCI_STATE_MACHINE_ENTER_STATE(state_machine) \
	((state_machine)->state_table[(state_machine)->current_state_id]. \
	 enter_state((state_machine)->state_machine_owner))

#define SCI_STATE_MACHINE_SET_STATE(state_machine, id) \
	((state_machine)->current_state_id = (id))

/*
 * ******************************************************************************
 * * P R O T E C T E D    M E T H O D S
 * ****************************************************************************** */

/**
 * This method will set the initial state and state table for the state
 *    machine. The caller should follow this request with the initialize
 *    request to cause the state machine to start.
 * @this_state_machine: This parameter provides the state machine object to be
 *    constructed.
 * @state_machine_owner: This parameter indicates the object that is owns the
 *    state machine being constructed.
 * @state_table: This parameter specifies the table of state objects that is
 *    managed by this state machine.
 * @initial_state: This parameter specifies the value of the initial state for
 *    this state machine.
 *
 */
void sci_base_state_machine_construct(
	struct sci_base_state_machine *this_state_machine,
	struct sci_base_object *my_state_machine_owner,
	struct sci_base_state *state_table,
	u32 initial_state)
{
#if defined(SCI_LOGGING)
	sci_base_subject_construct(&this_state_machine->parent);
#endif  /* defined(SCI_LOGGING) */

	this_state_machine->state_machine_owner = my_state_machine_owner;
	this_state_machine->initial_state_id  = initial_state;
	this_state_machine->previous_state_id = initial_state;
	this_state_machine->current_state_id  = initial_state;
	this_state_machine->state_table       = state_table;
}

/**
 * This method will cause the state machine to enter the initial state.
 * @this_state_machine: This parameter specifies the state machine that is to
 *    be started.
 *
 * sci_base_state_machine_construct() for how to set the initial state none
 */
void sci_base_state_machine_start(
	struct sci_base_state_machine *this_state_machine)
{
	SCI_STATE_MACHINE_SET_STATE(
		this_state_machine, this_state_machine->initial_state_id
		);

#if defined(SCI_BASE_ENABLE_SUBJECT_NOTIFICATION)
	sci_base_subject_notify(&this_state_machine->parent);
#endif

	SCI_STATE_MACHINE_ENTER_STATE(this_state_machine);
}

/**
 * This method will cause the state machine to exit it's current state only.
 * @this_state_machine: This parameter specifies the state machine that is to
 *    be stopped.
 *
 */
void sci_base_state_machine_stop(
	struct sci_base_state_machine *this_state_machine)
{
	SCI_STATE_MACHINE_EXIT_STATE(this_state_machine);

#if defined(SCI_BASE_ENABLE_SUBJECT_NOTIFICATION)
	sci_base_subject_notify(&this_state_machine->parent);
#endif
}

/**
 * This method performs an update to the current state of the state machine.
 * @this_state_machine: This parameter specifies the state machine for which
 *    the caller wishes to perform a state change.
 * @next_state: This parameter specifies the new state for the state machine.
 *
 */
void sci_base_state_machine_change_state(
	struct sci_base_state_machine *this_state_machine,
	u32 next_state)
{
	SCI_STATE_MACHINE_EXIT_STATE(this_state_machine);

	this_state_machine->previous_state_id = this_state_machine->current_state_id;
	SCI_STATE_MACHINE_SET_STATE(this_state_machine, next_state);

#if defined(SCI_BASE_ENABLE_SUBJECT_NOTIFICATION)
	/* Notify of the state change prior to entering the state. */
	sci_base_subject_notify(&this_state_machine->parent);
#endif

	SCI_STATE_MACHINE_ENTER_STATE(this_state_machine);
}

/**
 * This method simply returns the current state of the state machine to the
 *    caller.
 * @this_state_machine: This parameter specifies the state machine for which to
 *    retrieve the current state.
 *
 * This method returns a u32 value indicating the current state for the
 * supplied state machine.
 */
u32 sci_base_state_machine_get_state(
	struct sci_base_state_machine *this_state_machine)
{
	return this_state_machine->current_state_id;
}

