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
 * This file contains the base subject method implementations and any constants
 *    or structures private to the base subject object. A subject is a
 *    participant in the observer design pattern.  A subject represents the
 *    object being observed.
 *
 *
 */

#include "sci_types.h"
#include "sci_base_subject.h"
#include "sci_base_observer.h"

#if defined(SCI_LOGGING)

/*
 * ******************************************************************************
 * * P R O T E C T E D    M E T H O D S
 * ****************************************************************************** */

void sci_base_subject_construct(
	struct sci_base_subject *this_subject)
{
	this_subject->observer_list = NULL;
}

/* --------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------- */

void sci_base_subject_attach_observer(
	struct sci_base_subject *this_subject,
	struct sci_base_observer *observer)
{
	observer->next = this_subject->observer_list;

	this_subject->observer_list = observer;
}

/* --------------------------------------------------------------------------- */

void sci_base_subject_detach_observer(
	struct sci_base_subject *this_subject,
	struct sci_base_observer *observer)
{
	struct sci_base_observer *current_observer = this_subject->observer_list;
	struct sci_base_observer *previous_observer = NULL;

	/* Search list for the item to remove */
	while (
		current_observer != NULL
		&& current_observer != observer
		) {
		previous_observer = current_observer;
		current_observer = current_observer->next;
	}

	/* Was this observer in the list? */
	if (current_observer == observer) {
		if (previous_observer != NULL) {
			/* Remove from middle or end of list */
			previous_observer->next = observer->next;
		} else {
			/* Remove from the front of the list */
			this_subject->observer_list = observer->next;
		}

		/* protect the list so people dont follow bad pointers */
		observer->next = NULL;
	}
}

#endif /* defined(SCI_LOGGING) */
