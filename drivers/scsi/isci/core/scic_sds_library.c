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
 * This file contains the implementation of the public methods for a
 *    struct scic_sds_library object.
 *
 *
 */

#include "scic_sds_library.h"
#include "scic_sds_controller.h"
#include "scic_sds_request.h"
#include "scic_sds_remote_device.h"
#include "intel_pci.h"
#include "scic_sds_pci.h"
#include "scu_constants.h"

struct scic_sds_controller;

#define SCIC_LIBRARY_CONTROLLER_MEMORY_START(library) \
	((char *)(library) + sizeof(struct scic_sds_library))

/* --------------------------------------------------------------------------- */

u32 scic_library_get_object_size(
	u8 max_controller_count)
{
	return sizeof(struct scic_sds_library)
	       + scic_sds_controller_get_object_size() * max_controller_count;
}

/* --------------------------------------------------------------------------- */

SCI_LIBRARY_HANDLE_T scic_library_construct(
	void *library_memory,
	u8 max_controller_count)
{
	enum sci_status status;
	struct scic_sds_library *this_library;

	this_library = (struct scic_sds_library *)library_memory;

	this_library->max_controller_count = max_controller_count;

	this_library->controllers =
		(struct scic_sds_controller *)((char *)library_memory + sizeof(struct scic_sds_library));

	SCI_BASE_LIBRARY_CONSTRUCT(this_library,
				   &this_library->parent,
				   max_controller_count,
				   struct scic_sds_controller,
				   status);
	return this_library;
}

/* --------------------------------------------------------------------------- */

void scic_library_set_pci_info(
	SCI_LIBRARY_HANDLE_T library,
	struct sci_pci_common_header *pci_header)
{
	struct scic_sds_library *this_library;

	this_library = (struct scic_sds_library *)library;

	this_library->pci_device   = pci_header->device_id;
	this_library->pci_revision = pci_header->revision;
}

/* --------------------------------------------------------------------------- */

enum sci_status scic_library_allocate_controller(
	SCI_LIBRARY_HANDLE_T library,
	SCI_CONTROLLER_HANDLE_T *new_controller)
{
	enum sci_status status;
	struct scic_sds_library *this_library;

	this_library = (struct scic_sds_library *)library;

	if (
		((this_library->pci_device >= 0x1D60)
		 && (this_library->pci_device <= 0x1D62)
		)
		|| ((this_library->pci_device >= 0x1D64)
		    && (this_library->pci_device <= 0x1D65)
		    )
		|| ((this_library->pci_device >= 0x1D68)
		    && (this_library->pci_device <= 0x1D6F)
		    )
		) {
		SCI_BASE_LIBRARY_ALLOCATE_CONTROLLER(
			this_library, new_controller, &status);
	} else
		status = SCI_FAILURE_UNSUPPORTED_PCI_DEVICE_ID;

	return status;
}

/* --------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------- */

u8 scic_library_get_pci_device_controller_count(
	SCI_LIBRARY_HANDLE_T library)
{
	struct scic_sds_library *this_library;

	this_library = (struct scic_sds_library *)library;

	if (this_library->pci_revision == SCU_PBG_HBA_REV_B0)
		return 2;
	else
		return 1;
}

/* --------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------- */


/**
 * scic_sds_library_get_pci_revision() -
 *
 *
 */
enum SCU_CONTROLLER_PCI_REVISION_CODE scic_sds_library_get_pci_revision(
	struct scic_sds_library *library)
{
	return library->pci_revision;
}

/**
 * scic_sds_library_get_controller_index() -
 *
 *
 */
u8 scic_sds_library_get_controller_index(
	struct scic_sds_library *library,
	struct scic_sds_controller *controller)
{
	u8 index;

	for (index = 0; index < library->max_controller_count; index++) {
		if (controller == &library->controllers[index]) {
			return index;
		}
	}

	return 0xff;
}

