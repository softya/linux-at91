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

#ifndef _SCI_BASE_LIBRARY_H_
#define _SCI_BASE_LIBRARY_H_

/**
 * This file contains the protected interface structures, constants and
 *    interface methods for the struct sci_base_library object.
 *
 *
 */

#include "sci_library.h"
#include "sci_pool.h"
#include "sci_base_object.h"
#include "sci_base_logger.h"
#include "sci_controller_constants.h"

/**
 * struct sci_base_library - This structure contains all of the objects common
 *    to all library sub-objects.
 *
 *
 */
struct sci_base_library {
	/**
	 * This class derives directly from the base object class.  As a result,
	 * the field is named "parent" and is the first field contained in the
	 * structure.
	 */
	struct sci_base_object parent;

	/**
	 * This field provides the logger object to be utilized by all objects
	 * contained inside of a library.
	 */
	struct sci_base_logger logger;

	/* Create a pool structure to manage free controller indices. */
	SCI_POOL_CREATE(controller_id_pool, u16, SCI_MAX_CONTROLLERS);

};


/**
 * sci_base_library_construct() - This method will construct the base library
 *    object.
 * @this_library: This parameter specifies the library object to be constructed.
 * @max_controllers: This parameter specifies the maximum number of controllers
 *    to be supported by this library.
 *
 */
void sci_base_library_construct(
	struct sci_base_library *this_library,
	u32 max_controllers);

/**
 * SCI_BASE_LIBRARY_ALLOCATE_CONTROLLER() -
 *
 * This macro provides common code for allocating a controller from a library.
 * It will ensure that we successfully allocate an available controller index
 * and return SCI_FAILURE_INSUFFICIENT_RESOURCES if unsuccessful.
 */
#define SCI_BASE_LIBRARY_ALLOCATE_CONTROLLER(\
		library, \
		controller_ptr,	\
		rc \
		) \
	{ \
		u16 index; \
		*rc = SCI_SUCCESS; \
		if (!sci_pool_empty((library)->parent.controller_id_pool)) { \
			sci_pool_get((library)->parent.controller_id_pool, index); \
			*controller_ptr = (SCI_CONTROLLER_HANDLE_T) \
					  &(library)->controllers[index]; \
		} else \
			*rc = SCI_FAILURE_INSUFFICIENT_RESOURCES; \
	}

/**
 * SCI_BASE_LIBRARY_FREE_CONTROLLER() -
 *
 * This macro provides common code for freeing a controller to a library. It
 * calculates the index to the controller instance in the array by determining
 * the offset.
 */
#define SCI_BASE_LIBRARY_FREE_CONTROLLER(\
		library, \
		controller, \
		CONTROLLER_TYPE, \
		rc \
		) \
	{ \
		u16 index = (u16) \
			    ((((char *)(controller)) - ((char *)(library)->controllers)) \
			     / sizeof(CONTROLLER_TYPE)); \
		*rc = SCI_SUCCESS; \
		if ((index < SCI_MAX_CONTROLLERS) \
		    && (!sci_pool_full((library)->parent.controller_id_pool))) { \
			sci_pool_put((library)->parent.controller_id_pool, index); \
		} else \
			*rc = SCI_FAILURE_CONTROLLER_NOT_FOUND;	\
	}



/**
 * SCI_BASE_LIBRARY_CONSTRUCT() -
 *
 * This macro provides common code for constructing library. It It initialize
 * and fill the library's controller_id_pool.
 */
#define SCI_BASE_LIBRARY_CONSTRUCT(\
		library, \
		base_library, \
		max_controllers, \
		CONTROLLER_TYPE, \
		status \
		) \
	{ \
		u32 controller_index; \
		sci_base_object_construct(&(base_library)->parent, &(base_library)->logger); \
		sci_pool_initialize((base_library)->controller_id_pool); \
		for (controller_index = 0; controller_index < max_controller_count; controller_index++) {	\
			SCI_BASE_LIBRARY_FREE_CONTROLLER(\
				library, \
				&library->controllers[controller_index], \
				CONTROLLER_TYPE, \
				&status	\
				); \
		} \
	}



#endif /* _SCI_BASE_LIBRARY_H_ */
