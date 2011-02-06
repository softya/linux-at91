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

#ifndef _SCI_BASE_OBSERVER_H_
#define _SCI_BASE_OBSERVER_H_

/**
 * This file contains all of the structures, constants, and methods common to
 *    all base observer object definitions.
 *
 *
 */

#if defined(SCI_LOGGING)

struct sci_base_observer;
struct sci_base_subject;

/**
 * This type definition defines the format for the update method that is
 *    invoked for all observers participating in the observer design pattern.
 *
 * SCI_BASE_OBSERVER_UPDATE_T
 */
typedef void (*SCI_BASE_OBSERVER_UPDATE_T)(
	struct sci_base_observer *this_observer,
	struct sci_base_subject *the_subject
	);

/**
 * struct sci_base_observer - This structure defines the fields necessary for
 *    an object that intends to participate as an observer.
 *
 *
 */
struct sci_base_observer {
	/**
	 * This filed points to the next observer if there is one
	 */
	struct sci_base_observer *next;

	/**
	 * This field defines the function pointer that is invoked in order to
	 * notify the observer of a change in the subject (i.e. observed object).
	 */
	SCI_BASE_OBSERVER_UPDATE_T update;

};




#else /* defined(SCI_LOGGING) */

typedef u8 struct sci_base_observer;
#define sci_base_observer_construct
#define sci_base_observer_initialize
#define sci_base_observer_update

#endif  /* defined(SCI_LOGGING) */

#endif  /* _SCI_BASE_OBSERVER_H_ */
