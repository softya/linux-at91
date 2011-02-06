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

#ifndef _SCI_BASE_LOGGER_H_
#define _SCI_BASE_LOGGER_H_

/**
 * This file contains all of the constants, structures, and methods common to
 *    all base logger objects.
 *
 *
 */


#include "sci_logger.h"
#include "sci_base_object.h"


#define SCI_BASE_LOGGER_MAX_VERBOSITY_LEVELS 5

/**
 * struct sci_base_logger - This structure contains the set of log objects and
 *    verbosities that are enabled for logging.  It's parent is the
 *    struct sci_base_object.
 *
 *
 */
struct sci_base_logger {
#if defined(SCI_LOGGING)
	/**
	 * The field specifies that the parent object for the base logger
	 * is the base object itself.
	 */
	struct sci_base_object parent;

	/**
	 * This filed specifies an array of objects mask.  There is one object
	 * mask for each verbosity level (e.g. ERROR, WARNING, etc.).  This
	 * allows for more flexible logging.
	 */
	u32 object_mask[SCI_BASE_LOGGER_MAX_VERBOSITY_LEVELS];

	/**
	 * This filed specifies which verbosity levels are currently enabled
	 * for logging
	 */
	u32 verbosity_mask;
#else   /* defined(SCI_LOGGING) */
	u8 dummy;
#endif /* defined(SCI_LOGGING) */

};

#if defined(SCI_LOGGING)


#else /* SCI_LOGGING */

#define sci_base_logger_construct

#endif

#endif /* _SCI_BASE_LOGGER_H_ */
