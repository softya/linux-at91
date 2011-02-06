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

#ifndef _SCIC_LIBRARY_H_
#define _SCIC_LIBRARY_H_

/**
 * This file contains all of the interface methods that can be called by an SCI
 *    Core user on the library object.  The library is the container of all
 *    other objects being managed (i.e. controllers, target devices, sas ports,
 *    etc.).
 *
 *
 */


#include "sci_types.h"
#include "sci_status.h"


/**
 * enum _SCIC_LIBRARY_IO_MODE - This enumeration depicts the different IO modes
 *    in which the SCI library and it's controllers can operate.
 *
 *
 */
enum scic_library_io_mode {
	/**
	 * In this mode the SCI library will operate in a polling mode for
	 * operations.  In other words, the library will not return from a
	 * send io method until the completion for the IO has been received.
	 */
	SCIC_IO_MODE_POLLING,

	/**
	 * In this mode the SCI library returns after committing the IO request
	 * to the controller hardware.  Completion of the request will occur
	 * asynchronously.
	 */
	SCIC_IO_MODE_ASYNCHRONOUS

};


struct sci_pci_common_header;

/**
 * scic_library_construct() - This method will contsruct the core library based
 *    on the supplied parameter information.  By default, libraries are
 *    considered "ready" as soon as they are constructed.
 * @library_memory: a pointer to the memory at which the library object is
 *    located.
 * @max_controller_count: the maximum number of controllers that this library
 *    can manage.
 *
 * An opaque library handle to be used by the SCI user for all subsequent
 * library operations.
 */
SCI_LIBRARY_HANDLE_T scic_library_construct(
	void *library_memory,
	u8 max_controller_count);

/**
 * scic_library_set_pci_info() -
 * @library: the handle to the library object for which to allocate a
 *    controller.
 * @pci_header: a pointer to the pci header data for the pci device for which
 *    this library is being created.
 *
 * This method sets the PCI header information required for proper controller
 * object creation/allocation. none
 */
void scic_library_set_pci_info(
	SCI_LIBRARY_HANDLE_T library,
	struct sci_pci_common_header *pci_header);

/**
 * scic_library_get_object_size() - This method returns the size of the core
 *    library object.
 * @max_controller_count: the maximum number of controllers that this library
 *    can manage.
 *
 * a positive integer value indicating the size (in bytes) of the library
 * object.
 */
u32 scic_library_get_object_size(
	u8 max_controller_count);

/**
 * scic_library_get_pci_device_controller_count() -
 *
 *
 */
u8 scic_library_get_pci_device_controller_count(
	SCI_LIBRARY_HANDLE_T library);

/**
 * scic_library_allocate_controller() - This method will allocate the next
 *    available core controller object that can be managed by this core library.
 * @library: the handle to the library object for which to allocate a
 *    controller.
 * @new_controller: This parameter specifies a pointer to the controller handle
 *    that was added to the library.
 *
 * Indicate if the controller was successfully allocated or if iti failed in
 * some way. SCI_SUCCESS if the controller was successfully allocated.
 * SCI_FAILURE_INSUFFICIENT_RESOURCES if the library has no more available
 * controller objects to allocate.
 */
enum sci_status scic_library_allocate_controller(
	SCI_LIBRARY_HANDLE_T library,
	SCI_CONTROLLER_HANDLE_T *new_controller);








#endif  /* _SCIC_LIBRARY_H_ */

