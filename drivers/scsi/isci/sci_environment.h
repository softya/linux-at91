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

#ifndef _SCI_ENVIRONMENT_H_
#define _SCI_ENVIRONMENT_H_

#include "isci_request.h"

#ifdef SCI_SGL_OPTIMIZATION_ENABLED
/**
 * scic_cb_io_request_get_next_sge() - This macro gets address to where the
 *    next Scatter-Gather Element is located.
 *
 *
 */
#define scic_cb_io_request_get_next_sge(scic_user_io_request,	\
					current_sge_address, next_sge)	\
	{								\
		scic_cb_logger_log_trace(0, 0, "%s:\n", __func__);	\
		*next_sge = isci_request_io_request_get_next_sge(\
			scic_user_io_request,	\
			current_sge_address);	\
	}
#endif

#ifdef SCI_GET_PHYSICAL_ADDRESS_OPTIMIZATION_ENABLED
/**
 * scic_cb_io_request_get_physical_address() - This macro provides the physical
 *    address for the supplied virtual address when building an io request
 *    object.
 *
 *
 */
#define scic_cb_io_request_get_physical_address(controller, io_request,	  \
						virtual_address,	  \
						physical_address)	  \
	{									  \
		struct isci_host *isci_host					  \
			= (struct isci_host *)sci_object_get_association(controller);	 \
		struct isci_request *request					  \
			= (struct isci_request *)sci_object_get_association(io_request); \
		char *base_address = (char *)request;				 \
		char *requested_address = (char *)(virtual_address);		 \
		scic_cb_logger_log_trace(0, 0, "%s: isci_host = %p\n", __func__,  \
					 isci_host);				  \
		if (requested_address >= base_address				   \
		    && (requested_address - base_address) < request->request_alloc_size) { \
			*(physical_address) = request->request_daddr		  \
					      + (requested_address - base_address); \
		} else {							  \
			struct coherent_memory_info *mdl_struct;		  \
			struct list_head *curr_element;				  \
			list_for_each(curr_element, &isci_host->mdl_struct_list) {   \
				mdl_struct = list_entry(curr_element,		     \
							struct coherent_memory_info, \
							node);			  \
				base_address = (char *)mdl_struct->vaddr;	 \
				if (requested_address >= base_address		   \
				    && (requested_address - base_address)	   \
				    < mdl_struct->size) {			\
					*(physical_address) = mdl_struct->dma_handle \
							      + (requested_address    \
								 - base_address);  \
					return;					  \
				}						  \
			}							  \
			BUG();							  \
		}								  \
	}
#endif

#endif /*_SCI_ENVIRONMENT_H_ */

