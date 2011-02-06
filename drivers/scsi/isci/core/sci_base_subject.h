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

#ifndef _SCI_BASE_SUBJECT_H_
#define _SCI_BASE_SUBJECT_H_

/**
 * This file contains all of the structures, constants, and methods common to
 *    all subjects object definitions.  A subject is a participant in the
 *    observer pattern.  A subject represents the object being observed.
 *
 *
 */

#include "sci_types.h"

#if defined(SCI_LOGGING)

struct sci_base_observer;

/**
 * struct sci_base_subject - This structure defines the fields common to all
 *    subjects that participate in the observer design pattern
 *
 *
 */
struct sci_base_subject {
	struct sci_base_observer *observer_list;

};


/**
 * sci_base_subject_construct() - This method acts as the basic constructor for
 *    the subject.
 * @this_subject: This fields specifies the subject being constructed.
 *
 */
void sci_base_subject_construct(
	struct sci_base_subject *this_subject);


/**
 * sci_base_subject_attach_observer() - This method will add an observer to the
 *    subject.
 * @this_subject: This parameter specifies the subject for which an observer is
 *    being added.
 * @observer: This parameter specifies the observer that wishes it listen for
 *    notifications for the supplied subject.
 *
 */
void sci_base_subject_attach_observer(
	struct sci_base_subject *this_subject,
	struct sci_base_observer *observer);

/**
 * sci_base_subject_detach_observer() - This method will remove the observer
 *    from the subject.
 * @this_subject:
 * @my_observer:
 *
 */
void sci_base_subject_detach_observer(
	struct sci_base_subject *this_subject,
	struct sci_base_observer *my_observer);

#else /* defined(SCI_LOGGING) */

typedef u8 struct sci_base_subject;

#define sci_base_subject_construct
#define sci_base_subject_notify
#define sci_base_subject_attach_observer
#define sci_base_subject_detach_observer

#endif  /* defined(SCI_LOGGING) */

#endif  /* _SCI_BASE_SUBJECT_H_ */
