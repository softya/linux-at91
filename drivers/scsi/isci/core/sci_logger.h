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

#ifndef _SCI_LOGGER_H_
#define _SCI_LOGGER_H_

/**
 * This file contains all of the interface methods that can be called by an SCI
 *    user on the logger object.  These methods should be utilized to control
 *    the amount of information being logged by the SCI implementation.
 *
 *
 */


#include "sci_types.h"


/* The following is a list of verbosity levels that can be used to enable
 * logging for specific log objects.
 */

/** Enable/disable error level logging for the associated logger object(s). */
#define SCI_LOG_VERBOSITY_ERROR      0x00

/** Enable/disable warning level logging for the associated logger object(s). */
#define SCI_LOG_VERBOSITY_WARNING    0x01

/**
 *
 *
 * Enable/disable informative level logging for the associated logger object(s).
 */
#define SCI_LOG_VERBOSITY_INFO       0x02

/**
 *
 *
 * This constant is used to enable function trace (enter/exit) level logging
 * for the associated log object(s).
 */
#define SCI_LOG_VERBOSITY_TRACE      0x03

/**
 *
 *
 * This constant is used to enable state tracing information it will emit a log
 * message each time a state is entered for the associated log object(s).
 */
#define SCI_LOG_VERBOSITY_STATES     0x04

#ifdef SCI_LOGGING



/**
 * sci_logger_enable() - This method will enable each of the supplied log
 *    objects in log_object_mask for each of the supplied verbosities in
 *    verbosity_mask.  To enable all logging, simply set all bits in both the
 *    log_object_mask and verbosity_mask.
 * @logger: This parameter specifies the logger for which to disable the
 *    supplied objects/verbosities.  For example, the framework and core
 *    components have different loggers.
 * @log_object_mask: This parameter specifies the log objects for which to
 *    enable logging.
 * @verbosity_mask: This parameter specifies the verbosity levels at which to
 *    enable each log_object.
 *
 * Logging must be enabled at compile time in the driver, otherwise calling
 * this method has no affect. Reserved bits in both the supplied masks shall be
 * ignored. none
 */
void sci_logger_enable(
	SCI_LOGGER_HANDLE_T logger,
	u32 log_object_mask,
	u8 verbosity_mask);





#else /* SCI_LOGGING */

#define sci_logger_get_verbosity_mask(logger, log_object)
#define sci_logger_get_object_mask(logger, verbosity)
#define sci_logger_enable(logger, log_object_mask, verbosity_mask)
#define sci_logger_disable(logger, log_object_mask, verbosity_mask)
#define sci_logger_is_enabled(logger, log_object_mask, verbosity_level)

#endif /* SCI_LOGGING */


#endif  /* _SCI_LOGGER_H_ */

