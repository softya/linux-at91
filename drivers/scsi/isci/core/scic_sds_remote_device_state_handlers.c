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
 * This file contains the state handlers for the struct scic_sds_remote_device for the
 *    base state machine.
 *
 *
 */

#include "scic_user_callback.h"
#include "scic_controller.h"
#include "scic_sds_logger.h"
#include "scic_sds_remote_device.h"
#include "scic_sds_controller.h"
#include "scic_sds_port.h"
#include "scic_sds_request.h"
#include "scic_sds_remote_node_context.h"
#include "scu_event_codes.h"

/*
 * *****************************************************************************
 * *  PROTECTED METHODS
 * ***************************************************************************** */

/**
 *
 * @user_parameter: This is cast to a remote device object.
 *
 * This method is called once the remote node context is ready to be freed.
 * The remote device can now report that its stop operation is complete. none
 */
static void scic_sds_cb_remote_device_rnc_destruct_complete(
	void *user_parameter)
{
	struct scic_sds_remote_device *this_device;

	this_device = (struct scic_sds_remote_device *)user_parameter;

	ASSERT(this_device->started_request_count == 0);

	sci_base_state_machine_change_state(
		scic_sds_remote_device_get_base_state_machine(this_device),
		SCI_BASE_REMOTE_DEVICE_STATE_STOPPED
		);
}

/**
 *
 * @user_parameter: This is cast to a remote device object.
 *
 * This method is called once the remote node context has transisitioned to a
 * ready state.  This is the indication that the remote device object can also
 * transition to ready. none
 */
static void scic_sds_remote_device_resume_complete_handler(
	void *user_parameter)
{
	struct scic_sds_remote_device *this_device;

	this_device = (struct scic_sds_remote_device *)user_parameter;

	if (
		sci_base_state_machine_get_state(&this_device->parent.state_machine)
		!= SCI_BASE_REMOTE_DEVICE_STATE_READY
		) {
		sci_base_state_machine_change_state(
			&this_device->parent.state_machine,
			SCI_BASE_REMOTE_DEVICE_STATE_READY
			);
	}
}

/**
 *
 * @device: This parameter specifies the device for which the request is being
 *    started.
 * @request: This parameter specifies the request being started.
 * @status: This parameter specifies the current start operation status.
 *
 * This method will perform the STP request start processing common to IO
 * requests and task requests of all types. none
 */
void scic_sds_remote_device_start_request(
	struct scic_sds_remote_device *this_device,
	struct scic_sds_request *the_request,
	enum sci_status status)
{
	/* We still have a fault in starting the io complete it on the port */
	if (status == SCI_SUCCESS)
		scic_sds_remote_device_increment_request_count(this_device);
	else{
		this_device->owning_port->state_handlers->complete_io_handler(
			this_device->owning_port, this_device, the_request
			);
	}
}


/**
 *
 * @request: This parameter specifies the request being continued.
 *
 * This method will continue to post tc for a STP request. This method usually
 * serves as a callback when RNC gets resumed during a task management
 * sequence. none
 */
void scic_sds_remote_device_continue_request(
	struct scic_sds_remote_device *this_device)
{
	/* we need to check if this request is still valid to continue. */
	if (this_device->working_request != NULL) {
		struct scic_sds_request *this_request = this_device->working_request;

		this_request->owning_controller->state_handlers->parent.continue_io_handler(
			&this_request->owning_controller->parent,
			&this_request->target_device->parent,
			&this_request->parent
			);
	}
}

/**
 *
 * @user_parameter: This is cast to a remote device object.
 *
 * This method is called once the remote node context has reached a suspended
 * state. The remote device can now report that its suspend operation is
 * complete. none
 */

/**
 * This method will terminate all of the IO requests in the controllers IO
 *    request table that were targeted for this device.
 * @this_device: This parameter specifies the remote device for which to
 *    attempt to terminate all requests.
 *
 * This method returns an indication as to whether all requests were
 * successfully terminated.  If a single request fails to be terminated, then
 * this method will return the failure.
 */
static enum sci_status scic_sds_remote_device_terminate_requests(
	struct scic_sds_remote_device *this_device)
{
	enum sci_status status           = SCI_SUCCESS;
	enum sci_status terminate_status = SCI_SUCCESS;
	struct scic_sds_request *the_request;
	u32 index;
	u32 request_count    = this_device->started_request_count;

	SCIC_LOG_TRACE((
			       sci_base_object_get_logger(this_device),
			       SCIC_LOG_OBJECT_SMP_REMOTE_TARGET | SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
			       SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
			       "scic_sds_remote_device_terminate_requests(0x%x) enter\n",
			       this_device
			       ));

	for (index = 0;
	     (index < SCI_MAX_IO_REQUESTS) && (request_count > 0);
	     index++) {
		the_request = this_device->owning_port->owning_controller->io_request_table[index];

		if ((the_request != NULL) && (the_request->target_device == this_device)) {
			terminate_status = scic_controller_terminate_request(
				this_device->owning_port->owning_controller,
				this_device,
				the_request
				);

			if (terminate_status != SCI_SUCCESS)
				status = terminate_status;

			request_count--;
		}
	}

	return status;
}

/*
 * *****************************************************************************
 * *  DEFAULT STATE HANDLERS
 * ***************************************************************************** */

/**
 *
 * @device: The struct sci_base_remote_device which is then cast into a
 *    struct scic_sds_remote_device.
 *
 * This method is the default start handler.  It logs a warning and returns a
 * failure. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_remote_device_default_start_handler(
	struct sci_base_remote_device *device)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger((struct scic_sds_remote_device *)device),
				 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
				 "SCIC Remote Device requested to start while in wrong state %d\n",
				 sci_base_state_machine_get_state(
					 scic_sds_remote_device_get_base_state_machine((struct scic_sds_remote_device *)device))
				 ));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @device: The struct sci_base_remote_device which is then cast into a
 *    struct scic_sds_remote_device.
 *
 * This method is the default stop handler.  It logs a warning and returns a
 * failure. enum sci_status SCI_FAILURE_INVALID_STATE
 */
static enum sci_status scic_sds_remote_device_default_stop_handler(
	struct sci_base_remote_device *device)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger((struct scic_sds_remote_device *)device),
				 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
				 "SCIC Remote Device requested to stop while in wrong state %d\n",
				 sci_base_state_machine_get_state(
					 scic_sds_remote_device_get_base_state_machine((struct scic_sds_remote_device *)device))
				 ));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @device: The struct sci_base_remote_device which is then cast into a
 *    struct scic_sds_remote_device.
 *
 * This method is the default fail handler.  It logs a warning and returns a
 * failure. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_remote_device_default_fail_handler(
	struct sci_base_remote_device *device)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger((struct scic_sds_remote_device *)device),
				 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
				 "SCIC Remote Device requested to fail while in wrong state %d\n",
				 sci_base_state_machine_get_state(
					 scic_sds_remote_device_get_base_state_machine((struct scic_sds_remote_device *)device))
				 ));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @device: The struct sci_base_remote_device which is then cast into a
 *    struct scic_sds_remote_device.
 *
 * This method is the default destruct handler.  It logs a warning and returns
 * a failure. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_remote_device_default_destruct_handler(
	struct sci_base_remote_device *device)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger((struct scic_sds_remote_device *)device),
				 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
				 "SCIC Remote Device requested to destroy while in wrong state %d\n",
				 sci_base_state_machine_get_state(
					 scic_sds_remote_device_get_base_state_machine((struct scic_sds_remote_device *)device))
				 ));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @device: The struct sci_base_remote_device which is then cast into a
 *    struct scic_sds_remote_device.
 *
 * This method is the default reset handler.  It logs a warning and returns a
 * failure. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_remote_device_default_reset_handler(
	struct sci_base_remote_device *device)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger((struct scic_sds_remote_device *)device),
				 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
				 "SCIC Remote Device requested to reset while in wrong state %d\n",
				 sci_base_state_machine_get_state(
					 scic_sds_remote_device_get_base_state_machine((struct scic_sds_remote_device *)device))
				 ));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @device: The struct sci_base_remote_device which is then cast into a
 *    struct scic_sds_remote_device.
 *
 * This method is the default reset complete handler.  It logs a warning and
 * returns a failure. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_remote_device_default_reset_complete_handler(
	struct sci_base_remote_device *device)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger((struct scic_sds_remote_device *)device),
				 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
				 "SCIC Remote Device requested to complete reset while in wrong state %d\n",
				 sci_base_state_machine_get_state(
					 scic_sds_remote_device_get_base_state_machine((struct scic_sds_remote_device *)device))
				 ));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @device: The struct sci_base_remote_device which is then cast into a
 *    struct scic_sds_remote_device.
 *
 * This method is the default suspend handler.  It logs a warning and returns a
 * failure. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_remote_device_default_suspend_handler(
	struct scic_sds_remote_device *this_device,
	u32 suspend_type)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger(this_device),
				 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
				 "SCIC Remote Device 0x%x requested to suspend %d while in wrong state %d\n",
				 this_device, suspend_type,
				 sci_base_state_machine_get_state(
					 scic_sds_remote_device_get_base_state_machine(this_device))
				 ));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @device: The struct sci_base_remote_device which is then cast into a
 *    struct scic_sds_remote_device.
 *
 * This method is the default resume handler.  It logs a warning and returns a
 * failure. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_remote_device_default_resume_handler(
	struct scic_sds_remote_device *this_device)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger(this_device),
				 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
				 "SCIC Remote Device requested to resume while in wrong state %d\n",
				 sci_base_state_machine_get_state(
					 scic_sds_remote_device_get_base_state_machine(this_device))
				 ));

	return SCI_FAILURE_INVALID_STATE;
}

#if defined(SCI_LOGGING)
/**
 *
 * @[in]: this_device This is the device object that is receiving the event.
 * @[in]: event_code The event code to process.
 *
 * This is a private method for emitting log messages related to events
 * reported to the remote device from the controller object. None
 */
static void scic_sds_emit_event_log_message(
	struct scic_sds_remote_device *this_device,
	u32 event_code,
	char *message_guts,
	bool ready_state)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger(this_device),
				 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
				 "SCIC Remote device 0x%x (state %d) received %s %x while in the %sready %s%d\n",
				 this_device,
				 sci_base_state_machine_get_state(
					 scic_sds_remote_device_get_base_state_machine(this_device)),
				 message_guts, event_code,
				 (ready_state)
				 ? ""
				 : "not ",
				 (this_device->has_ready_substate_machine)
				 ? "substate "
				 : "",
				 (this_device->has_ready_substate_machine)
				 ? sci_base_state_machine_get_state(&this_device->ready_substate_machine)
				 : 0
				 ));
}
#else /* defined(SCI_LOGGING) */
#define scic_sds_emit_event_log_message(device, event_code, message, state)
#endif /* defined(SCI_LOGGING) */

/**
 *
 * @device: The struct sci_base_remote_device which is then cast into a
 *    struct scic_sds_remote_device.
 * @event_code: The event code that the struct scic_sds_controller wants the device
 *    object to process.
 *
 * This method is the default event handler.  It will call the RNC state
 * machine handler for any RNC events otherwise it will log a warning and
 * returns a failure. enum sci_status SCI_FAILURE_INVALID_STATE
 */
static enum sci_status  scic_sds_remote_device_core_event_handler(
	struct scic_sds_remote_device *this_device,
	u32 event_code,
	bool is_ready_state)
{
	enum sci_status status;

	switch (scu_get_event_type(event_code)) {
	case SCU_EVENT_TYPE_RNC_OPS_MISC:
	case SCU_EVENT_TYPE_RNC_SUSPEND_TX:
	case SCU_EVENT_TYPE_RNC_SUSPEND_TX_RX:
		status = scic_sds_remote_node_context_event_handler(this_device->rnc, event_code);
		break;
	case SCU_EVENT_TYPE_PTX_SCHEDULE_EVENT:

		if (scu_get_event_code(event_code) == SCU_EVENT_IT_NEXUS_TIMEOUT) {
			status = SCI_SUCCESS;

			/* Suspend the associated RNC */
			scic_sds_remote_node_context_suspend(this_device->rnc,
							      SCI_SOFTWARE_SUSPENSION,
							      NULL, NULL);

			scic_sds_emit_event_log_message(
				this_device, event_code,
				(is_ready_state)
				? "I_T_Nexus_Timeout event"
				: "I_T_Nexus_Timeout event in wrong state",
				is_ready_state);

			break;
		}
	/* Else, fall through and treat as unhandled... */

	default:
		scic_sds_emit_event_log_message(this_device, event_code,
						 (is_ready_state)
						 ? "unexpected event"
						 : "unexpected event in wrong state",
						 is_ready_state);
		status = SCI_FAILURE_INVALID_STATE;
		break;
	}

	return status;
}
/**
 *
 * @device: The struct sci_base_remote_device which is then cast into a
 *    struct scic_sds_remote_device.
 * @event_code: The event code that the struct scic_sds_controller wants the device
 *    object to process.
 *
 * This method is the default event handler.  It will call the RNC state
 * machine handler for any RNC events otherwise it will log a warning and
 * returns a failure. enum sci_status SCI_FAILURE_INVALID_STATE
 */
static enum sci_status  scic_sds_remote_device_default_event_handler(
	struct scic_sds_remote_device *this_device,
	u32 event_code)
{
	return scic_sds_remote_device_core_event_handler(this_device,
							  event_code,
							  false);
}

/**
 *
 * @device: The struct sci_base_remote_device which is then cast into a
 *    struct scic_sds_remote_device.
 * @frame_index: The frame index for which the struct scic_sds_controller wants this
 *    device object to process.
 *
 * This method is the default unsolicited frame handler.  It logs a warning,
 * releases the frame and returns a failure. enum sci_status
 * SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_remote_device_default_frame_handler(
	struct scic_sds_remote_device *this_device,
	u32 frame_index)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger(this_device),
				 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
				 "SCIC Remote Device requested to handle frame %x while in wrong state %d\n",
				 frame_index,
				 sci_base_state_machine_get_state(&this_device->parent.state_machine)
				 ));

	/* Return the frame back to the controller */
	scic_sds_controller_release_frame(
		scic_sds_remote_device_get_controller(this_device), frame_index
		);

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @device: The struct sci_base_remote_device which is then cast into a
 *    struct scic_sds_remote_device.
 * @request: The struct sci_base_request which is then cast into a SCIC_SDS_IO_REQUEST
 *    to start.
 *
 * This method is the default start io handler.  It logs a warning and returns
 * a failure. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_remote_device_default_start_request_handler(
	struct sci_base_remote_device *device,
	struct sci_base_request *request)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger((struct scic_sds_remote_device *)device),
				 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
				 "SCIC Remote Device requested to start io request %x while in wrong state %d\n",
				 request,
				 sci_base_state_machine_get_state(
					 scic_sds_remote_device_get_base_state_machine((struct scic_sds_remote_device *)device))
				 ));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @device: The struct sci_base_remote_device which is then cast into a
 *    struct scic_sds_remote_device.
 * @request: The struct sci_base_request which is then cast into a SCIC_SDS_IO_REQUEST
 *    to complete.
 *
 * This method is the default complete io handler.  It logs a warning and
 * returns a failure. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_remote_device_default_complete_request_handler(
	struct sci_base_remote_device *device,
	struct sci_base_request *request)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger((struct scic_sds_remote_device *)device),
				 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
				 "SCIC Remote Device requested to complete io_request %x while in wrong state %d\n",
				 request,
				 sci_base_state_machine_get_state(
					 scic_sds_remote_device_get_base_state_machine((struct scic_sds_remote_device *)device))
				 ));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @device: The struct sci_base_remote_device which is then cast into a
 *    struct scic_sds_remote_device.
 * @request: The struct sci_base_request which is then cast into a SCIC_SDS_IO_REQUEST
 *    to continue.
 *
 * This method is the default continue io handler.  It logs a warning and
 * returns a failure. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_remote_device_default_continue_request_handler(
	struct sci_base_remote_device *device,
	struct sci_base_request *request)
{
	SCIC_LOG_WARNING((
				 sci_base_object_get_logger((struct scic_sds_remote_device *)device),
				 SCIC_LOG_OBJECT_SSP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_SMP_REMOTE_TARGET |
				 SCIC_LOG_OBJECT_STP_REMOTE_TARGET,
				 "SCIC Remote Device requested to continue io request %x while in wrong state %d\n",
				 request,
				 sci_base_state_machine_get_state(
					 scic_sds_remote_device_get_base_state_machine((struct scic_sds_remote_device *)device))
				 ));

	return SCI_FAILURE_INVALID_STATE;
}

/**
 *
 * @device: The struct sci_base_remote_device which is then cast into a
 *    struct scic_sds_remote_device.
 * @request: The struct sci_base_request which is then cast into a SCIC_SDS_IO_REQUEST
 *    to complete.
 *
 * This method is the default complete task handler.  It logs a warning and
 * returns a failure. enum sci_status SCI_FAILURE_INVALID_STATE
 */

/*
 * *****************************************************************************
 * *  NORMAL STATE HANDLERS
 * ***************************************************************************** */

/**
 *
 * @device: The struct sci_base_remote_device which is then cast into a
 *    struct scic_sds_remote_device.
 * @frame_index: The frame index for which the struct scic_sds_controller wants this
 *    device object to process.
 *
 * This method is a general ssp frame handler.  In most cases the device object
 * needs to route the unsolicited frame processing to the io request object.
 * This method decodes the tag for the io request object and routes the
 * unsolicited frame to that object. enum sci_status SCI_FAILURE_INVALID_STATE
 */
enum sci_status scic_sds_remote_device_general_frame_handler(
	struct scic_sds_remote_device *this_device,
	u32 frame_index)
{
	enum sci_status result;
	struct sci_ssp_frame_header *frame_header;
	struct scic_sds_request *io_request;

	result = scic_sds_unsolicited_frame_control_get_header(
		&(scic_sds_remote_device_get_controller(this_device)->uf_control),
		frame_index,
		(void **)&frame_header
		);

	if (SCI_SUCCESS == result) {
		io_request = scic_sds_controller_get_io_request_from_tag(
			scic_sds_remote_device_get_controller(this_device), frame_header->tag);

		if ((io_request == SCI_INVALID_HANDLE)
		    || (io_request->target_device != this_device)) {
			/*
			 * We could not map this tag to a valid IO request
			 * Just toss the frame and continue */
			scic_sds_controller_release_frame(
				scic_sds_remote_device_get_controller(this_device), frame_index
				);
		} else {
			/* The IO request is now in charge of releasing the frame */
			result = io_request->state_handlers->frame_handler(
				io_request, frame_index);
		}
	}

	return result;
}

/**
 *
 * @[in]: this_device This is the device object that is receiving the event.
 * @[in]: event_code The event code to process.
 *
 * This is a common method for handling events reported to the remote device
 * from the controller object. enum sci_status
 */
enum sci_status scic_sds_remote_device_general_event_handler(
	struct scic_sds_remote_device *this_device,
	u32 event_code)
{
	return scic_sds_remote_device_core_event_handler(this_device,
							  event_code,
							  true);
}

/*
 * *****************************************************************************
 * *  STOPPED STATE HANDLERS
 * ***************************************************************************** */

/**
 *
 * @device:
 *
 * This method takes the struct scic_sds_remote_device from a stopped state and
 * attempts to start it.   The RNC buffer for the device is constructed and the
 * device state machine is transitioned to the
 * SCIC_BASE_REMOTE_DEVICE_STATE_STARTING. enum sci_status SCI_SUCCESS if there is
 * an RNC buffer available to construct the remote device.
 * SCI_FAILURE_INSUFFICIENT_RESOURCES if there is no RNC buffer available in
 * which to construct the remote device.
 */
static enum sci_status scic_sds_remote_device_stopped_state_start_handler(
	struct sci_base_remote_device *device)
{
	enum sci_status status;
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)device;

	status = scic_sds_remote_node_context_resume(
		this_device->rnc,
		scic_sds_remote_device_resume_complete_handler,
		this_device
		);

	if (status == SCI_SUCCESS) {
		sci_base_state_machine_change_state(
			scic_sds_remote_device_get_base_state_machine(this_device),
			SCI_BASE_REMOTE_DEVICE_STATE_STARTING
			);
	}

	return status;
}

/**
 *
 * @this_device: The struct sci_base_remote_device which is cast into a
 *    struct scic_sds_remote_device.
 *
 * This method will stop a struct scic_sds_remote_device that is already in a stopped
 * state.  This is not considered an error since the device is already stopped.
 * enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_remote_device_stopped_state_stop_handler(
	struct sci_base_remote_device *this_device)
{
	return SCI_SUCCESS;
}

/**
 *
 * @this_device: The struct sci_base_remote_device which is cast into a
 *    struct scic_sds_remote_device.
 *
 * This method will destruct a struct scic_sds_remote_device that is in a stopped
 * state.  This is the only state from which a destruct request will succeed.
 * The RNi for this struct scic_sds_remote_device is returned to the free pool and the
 * device object transitions to the SCI_BASE_REMOTE_DEVICE_STATE_FINAL.
 * enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_remote_device_stopped_state_destruct_handler(
	struct sci_base_remote_device *device)
{
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)device;

	scic_sds_controller_free_remote_node_context(
		scic_sds_remote_device_get_controller(this_device),
		this_device,
		this_device->rnc->remote_node_index
		);

	scic_sds_remote_node_context_set_remote_node_index(
		this_device->rnc,
		SCIC_SDS_REMOTE_NODE_CONTEXT_INVALID_INDEX
		);

	scic_sds_port_set_direct_attached_device_id(
		this_device->owning_port,
		SCIC_SDS_REMOTE_NODE_CONTEXT_INVALID_INDEX
		);

	sci_base_state_machine_change_state(
		scic_sds_remote_device_get_base_state_machine(this_device),
		SCI_BASE_REMOTE_DEVICE_STATE_FINAL
		);

	scic_sds_remote_device_deinitialize_state_logging(this_device);

	return SCI_SUCCESS;
}

/*
 * *****************************************************************************
 * *  STARTING STATE HANDLERS
 * ***************************************************************************** */

static enum sci_status scic_sds_remote_device_starting_state_stop_handler(
	struct sci_base_remote_device *device)
{
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)device;

	/*
	 * This device has not yet started so there had better be no IO requests
	 */
	ASSERT(this_device->started_request_count == 0);

	/*
	 * Destroy the remote node context
	 */
	scic_sds_remote_node_context_destruct(
		this_device->rnc,
		scic_sds_cb_remote_device_rnc_destruct_complete,
		this_device
		);

	/*
	 * Transition to the stopping state and wait for the remote node to
	 * complete being posted and invalidated.
	 */
	sci_base_state_machine_change_state(
		scic_sds_remote_device_get_base_state_machine(this_device),
		SCI_BASE_REMOTE_DEVICE_STATE_STOPPING
		);

	return SCI_SUCCESS;
}

/*
 * *****************************************************************************
 * *  INITIALIZING STATE HANDLERS
 * ***************************************************************************** */

/* There is nothing to do here for SSP devices */

/*
 * *****************************************************************************
 * *  READY STATE HANDLERS
 * ***************************************************************************** */

/**
 *
 * @this_device: The struct scic_sds_remote_device object to be suspended.
 *
 * This method is the resume handler for the struct scic_sds_remote_device object. It
 * will post an RNC resume to the SCU hardware. enum sci_status SCI_SUCCESS
 */

/**
 *
 * @device: The struct sci_base_remote_device object which is cast to a
 *    struct scic_sds_remote_device object.
 *
 * This method is the default stop handler for the struct scic_sds_remote_device ready
 * substate machine. It will stop the current substate machine and transition
 * the base state machine to SCI_BASE_REMOTE_DEVICE_STATE_STOPPING. enum sci_status
 * SCI_SUCCESS
 */
enum sci_status scic_sds_remote_device_ready_state_stop_handler(
	struct sci_base_remote_device *device)
{
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)device;
	enum sci_status status      = SCI_SUCCESS;

	/* Request the parent state machine to transition to the stopping state */
	sci_base_state_machine_change_state(
		scic_sds_remote_device_get_base_state_machine(this_device),
		SCI_BASE_REMOTE_DEVICE_STATE_STOPPING
		);

	if (this_device->started_request_count == 0) {
		scic_sds_remote_node_context_destruct(
			this_device->rnc,
			scic_sds_cb_remote_device_rnc_destruct_complete,
			this_device
			);
	} else
		status = scic_sds_remote_device_terminate_requests(this_device);

	return status;
}

/**
 *
 * @device: The struct sci_base_remote_device object which is cast to a
 *    struct scic_sds_remote_device object.
 *
 * This is the ready state device reset handler enum sci_status
 */
enum sci_status scic_sds_remote_device_ready_state_reset_handler(
	struct sci_base_remote_device *device)
{
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)device;

	/* Request the parent state machine to transition to the stopping state */
	sci_base_state_machine_change_state(
		scic_sds_remote_device_get_base_state_machine(this_device),
		SCI_BASE_REMOTE_DEVICE_STATE_RESETTING
		);

	return SCI_SUCCESS;
}

/**
 *
 * @device: The struct sci_base_remote_device object which is cast to a
 *    struct scic_sds_remote_device object.
 *
 * This is the default fail handler for the struct scic_sds_remote_device ready
 * substate machine.  It will stop the current ready substate and transition
 * the remote device object to the SCI_BASE_REMOTE_DEVICE_STATE_FAILED.
 * enum sci_status SCI_SUCCESS
 */

/**
 *
 * @device: The struct sci_base_remote_device which is cast to a
 *    struct scic_sds_remote_device for which the request is to be started.
 * @request: The struct sci_base_request which is cast to a SCIC_SDS_IO_REQUEST that
 *    is to be started.
 *
 * This method will attempt to start a task request for this device object. The
 * remote device object will issue the start request for the task and if
 * successful it will start the request for the port object then increment its
 * own requet count. enum sci_status SCI_SUCCESS if the task request is started for
 * this device object. SCI_FAILURE_INSUFFICIENT_RESOURCES if the io request
 * object could not get the resources to start.
 */
static enum sci_status scic_sds_remote_device_ready_state_start_task_handler(
	struct sci_base_remote_device *device,
	struct sci_base_request *request)
{
	enum sci_status result;
	struct scic_sds_remote_device *this_device  = (struct scic_sds_remote_device *)device;
	struct scic_sds_request *task_request = (struct scic_sds_request *)request;

	/* See if the port is in a state where we can start the IO request */
	result = scic_sds_port_start_io(
		scic_sds_remote_device_get_port(this_device), this_device, task_request);

	if (result == SCI_SUCCESS) {
		result = scic_sds_remote_node_context_start_task(
			this_device->rnc, task_request
			);

		if (result == SCI_SUCCESS) {
			result = scic_sds_request_start(task_request);
		}

		scic_sds_remote_device_start_request(this_device, task_request, result);
	}

	return result;
}

/**
 *
 * @device: The struct sci_base_remote_device which is cast to a
 *    struct scic_sds_remote_device for which the request is to be started.
 * @request: The struct sci_base_request which is cast to a SCIC_SDS_IO_REQUEST that
 *    is to be started.
 *
 * This method will attempt to start an io request for this device object. The
 * remote device object will issue the start request for the io and if
 * successful it will start the request for the port object then increment its
 * own requet count. enum sci_status SCI_SUCCESS if the io request is started for
 * this device object. SCI_FAILURE_INSUFFICIENT_RESOURCES if the io request
 * object could not get the resources to start.
 */
static enum sci_status scic_sds_remote_device_ready_state_start_io_handler(
	struct sci_base_remote_device *device,
	struct sci_base_request *request)
{
	enum sci_status result;
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)device;
	struct scic_sds_request *io_request  = (struct scic_sds_request *)request;

	/* See if the port is in a state where we can start the IO request */
	result = scic_sds_port_start_io(
		scic_sds_remote_device_get_port(this_device), this_device, io_request);

	if (result == SCI_SUCCESS) {
		result = scic_sds_remote_node_context_start_io(
			this_device->rnc, io_request
			);

		if (result == SCI_SUCCESS) {
			result = scic_sds_request_start(io_request);
		}

		scic_sds_remote_device_start_request(this_device, io_request, result);
	}

	return result;
}

/**
 *
 * @device: The struct sci_base_remote_device which is cast to a
 *    struct scic_sds_remote_device for which the request is to be completed.
 * @request: The struct sci_base_request which is cast to a SCIC_SDS_IO_REQUEST that
 *    is to be completed.
 *
 * This method will complete the request for the remote device object.  The
 * method will call the completion handler for the request object and if
 * successful it will complete the request on the port object then decrement
 * its own started_request_count. enum sci_status
 */
static enum sci_status scic_sds_remote_device_ready_state_complete_request_handler(
	struct sci_base_remote_device *device,
	struct sci_base_request *request)
{
	enum sci_status result;
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)device;
	struct scic_sds_request *the_request = (struct scic_sds_request *)request;

	result = scic_sds_request_complete(the_request);

	if (result == SCI_SUCCESS) {
		/* See if the port is in a state where we can start the IO request */
		result = scic_sds_port_complete_io(
			scic_sds_remote_device_get_port(this_device), this_device, the_request);

		if (result == SCI_SUCCESS) {
			scic_sds_remote_device_decrement_request_count(this_device);
		}
	}

	return result;
}

/*
 * *****************************************************************************
 * *  STOPPING STATE HANDLERS
 * ***************************************************************************** */

/**
 *
 * @this_device: The struct sci_base_remote_device which is cast into a
 *    struct scic_sds_remote_device.
 *
 * This method will stop a struct scic_sds_remote_device that is already in the
 * SCI_BASE_REMOTE_DEVICE_STATE_STOPPING state. This is not considered an error
 * since we allow a stop request on a device that is alreay stopping or
 * stopped. enum sci_status SCI_SUCCESS
 */
static enum sci_status scic_sds_remote_device_stopping_state_stop_handler(
	struct sci_base_remote_device *device)
{
	/*
	 * All requests should have been terminated, but if there is an
	 * attempt to stop a device already in the stopping state, then
	 * try again to terminate. */
	return scic_sds_remote_device_terminate_requests(
		       (struct scic_sds_remote_device *)device);
}


/**
 *
 * @device: The device object for which the request is completing.
 * @request: The task request that is being completed.
 *
 * This method completes requests for this struct scic_sds_remote_device while it is
 * in the SCI_BASE_REMOTE_DEVICE_STATE_STOPPING state. This method calls the
 * complete method for the request object and if that is successful the port
 * object is called to complete the task request. Then the device object itself
 * completes the task request. If struct scic_sds_remote_device started_request_count
 * goes to 0 and the invalidate RNC request has completed the device object can
 * transition to the SCI_BASE_REMOTE_DEVICE_STATE_STOPPED. enum sci_status
 */
static enum sci_status scic_sds_remote_device_stopping_state_complete_request_handler(
	struct sci_base_remote_device *device,
	struct sci_base_request *request)
{
	enum sci_status status = SCI_SUCCESS;
	struct scic_sds_request *this_request = (struct scic_sds_request *)request;
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)device;

	status = scic_sds_request_complete(this_request);
	if (status == SCI_SUCCESS) {
		status = scic_sds_port_complete_io(
			scic_sds_remote_device_get_port(this_device),
			this_device,
			this_request
			);

		if (status == SCI_SUCCESS) {
			scic_sds_remote_device_decrement_request_count(this_device);

			if (scic_sds_remote_device_get_request_count(this_device) == 0) {
				scic_sds_remote_node_context_destruct(
					this_device->rnc,
					scic_sds_cb_remote_device_rnc_destruct_complete,
					this_device
					);
			}
		}
	}

	return status;
}

/*
 * *****************************************************************************
 * *  RESETTING STATE HANDLERS
 * ***************************************************************************** */

/**
 *
 * @device: The struct sci_base_remote_device which is to be cast into a
 *    struct scic_sds_remote_device object.
 *
 * This method will complete the reset operation when the device is in the
 * resetting state. enum sci_status
 */
static enum sci_status scic_sds_remote_device_resetting_state_reset_complete_handler(
	struct sci_base_remote_device *device)
{
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)device;

	sci_base_state_machine_change_state(
		&this_device->parent.state_machine,
		SCI_BASE_REMOTE_DEVICE_STATE_READY
		);

	return SCI_SUCCESS;
}

/**
 *
 * @device: The struct sci_base_remote_device which is to be cast into a
 *    struct scic_sds_remote_device object.
 *
 * This method will stop the remote device while in the resetting state.
 * enum sci_status
 */
static enum sci_status scic_sds_remote_device_resetting_state_stop_handler(
	struct sci_base_remote_device *device)
{
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)device;

	sci_base_state_machine_change_state(
		&this_device->parent.state_machine,
		SCI_BASE_REMOTE_DEVICE_STATE_STOPPING
		);

	return SCI_SUCCESS;
}

/**
 *
 * @device: The device object for which the request is completing.
 * @request: The task request that is being completed.
 *
 * This method completes requests for this struct scic_sds_remote_device while it is
 * in the SCI_BASE_REMOTE_DEVICE_STATE_RESETTING state. This method calls the
 * complete method for the request object and if that is successful the port
 * object is called to complete the task request. Then the device object itself
 * completes the task request. enum sci_status
 */
static enum sci_status scic_sds_remote_device_resetting_state_complete_request_handler(
	struct sci_base_remote_device *device,
	struct sci_base_request *request)
{
	enum sci_status status = SCI_SUCCESS;
	struct scic_sds_request *this_request = (struct scic_sds_request *)request;
	struct scic_sds_remote_device *this_device = (struct scic_sds_remote_device *)device;

	status = scic_sds_request_complete(this_request);

	if (status == SCI_SUCCESS) {
		status = scic_sds_port_complete_io(
			scic_sds_remote_device_get_port(this_device), this_device, this_request);

		if (status == SCI_SUCCESS) {
			scic_sds_remote_device_decrement_request_count(this_device);
		}
	}

	return status;
}

/*
 * *****************************************************************************
 * *  FAILED STATE HANDLERS
 * ***************************************************************************** */

/**
 *
 * @device: The struct sci_base_remote_device which is to be cast into a
 *    struct scic_sds_remote_device object.
 *
 * This method handles the remove request for a failed struct scic_sds_remote_device
 * object. The method will transition the device object to the
 * SCIC_BASE_REMOTE_DEVICE_STATE_STOPPING. enum sci_status SCI_SUCCESS
 */

/* --------------------------------------------------------------------------- */

struct scic_sds_remote_device_state_handler
scic_sds_remote_device_state_handler_table[SCI_BASE_REMOTE_DEVICE_MAX_STATES] =
{
	/* SCI_BASE_REMOTE_DEVICE_STATE_INITIAL */
	{
		{
			scic_sds_remote_device_default_start_handler,
			scic_sds_remote_device_default_stop_handler,
			scic_sds_remote_device_default_fail_handler,
			scic_sds_remote_device_default_destruct_handler,
			scic_sds_remote_device_default_reset_handler,
			scic_sds_remote_device_default_reset_complete_handler,
			scic_sds_remote_device_default_start_request_handler,
			scic_sds_remote_device_default_complete_request_handler,
			scic_sds_remote_device_default_continue_request_handler,
			scic_sds_remote_device_default_start_request_handler,
			scic_sds_remote_device_default_complete_request_handler
		},
		scic_sds_remote_device_default_suspend_handler,
		scic_sds_remote_device_default_resume_handler,
		scic_sds_remote_device_default_event_handler,
		scic_sds_remote_device_default_frame_handler
	},
	/* SCI_BASE_REMOTE_DEVICE_STATE_STOPPED */
	{
		{
			scic_sds_remote_device_stopped_state_start_handler,
			scic_sds_remote_device_stopped_state_stop_handler,
			scic_sds_remote_device_default_fail_handler,
			scic_sds_remote_device_stopped_state_destruct_handler,
			scic_sds_remote_device_default_reset_handler,
			scic_sds_remote_device_default_reset_complete_handler,
			scic_sds_remote_device_default_start_request_handler,
			scic_sds_remote_device_default_complete_request_handler,
			scic_sds_remote_device_default_continue_request_handler,
			scic_sds_remote_device_default_start_request_handler,
			scic_sds_remote_device_default_complete_request_handler
		},
		scic_sds_remote_device_default_suspend_handler,
		scic_sds_remote_device_default_resume_handler,
		scic_sds_remote_device_default_event_handler,
		scic_sds_remote_device_default_frame_handler
	},
	/* SCI_BASE_REMOTE_DEVICE_STATE_STARTING */
	{
		{
			scic_sds_remote_device_default_start_handler,
			scic_sds_remote_device_starting_state_stop_handler,
			scic_sds_remote_device_default_fail_handler,
			scic_sds_remote_device_default_destruct_handler,
			scic_sds_remote_device_default_reset_handler,
			scic_sds_remote_device_default_reset_complete_handler,
			scic_sds_remote_device_default_start_request_handler,
			scic_sds_remote_device_default_complete_request_handler,
			scic_sds_remote_device_default_continue_request_handler,
			scic_sds_remote_device_default_start_request_handler,
			scic_sds_remote_device_default_complete_request_handler
		},
		scic_sds_remote_device_default_suspend_handler,
		scic_sds_remote_device_default_resume_handler,
		scic_sds_remote_device_general_event_handler,
		scic_sds_remote_device_default_frame_handler
	},
	/* SCI_BASE_REMOTE_DEVICE_STATE_READY */
	{
		{
			scic_sds_remote_device_default_start_handler,
			scic_sds_remote_device_ready_state_stop_handler,
			scic_sds_remote_device_default_fail_handler,
			scic_sds_remote_device_default_destruct_handler,
			scic_sds_remote_device_ready_state_reset_handler,
			scic_sds_remote_device_default_reset_complete_handler,
			scic_sds_remote_device_ready_state_start_io_handler,
			scic_sds_remote_device_ready_state_complete_request_handler,
			scic_sds_remote_device_default_continue_request_handler,
			scic_sds_remote_device_ready_state_start_task_handler,
			scic_sds_remote_device_ready_state_complete_request_handler
		},
		scic_sds_remote_device_default_suspend_handler,
		scic_sds_remote_device_default_resume_handler,
		scic_sds_remote_device_general_event_handler,
		scic_sds_remote_device_general_frame_handler,
	},
	/* SCI_BASE_REMOTE_DEVICE_STATE_STOPPING */
	{
		{
			scic_sds_remote_device_default_start_handler,
			scic_sds_remote_device_stopping_state_stop_handler,
			scic_sds_remote_device_default_fail_handler,
			scic_sds_remote_device_default_destruct_handler,
			scic_sds_remote_device_default_reset_handler,
			scic_sds_remote_device_default_reset_complete_handler,
			scic_sds_remote_device_default_start_request_handler,
			scic_sds_remote_device_stopping_state_complete_request_handler,
			scic_sds_remote_device_default_continue_request_handler,
			scic_sds_remote_device_default_start_request_handler,
			scic_sds_remote_device_stopping_state_complete_request_handler
		},
		scic_sds_remote_device_default_suspend_handler,
		scic_sds_remote_device_default_resume_handler,
		scic_sds_remote_device_general_event_handler,
		scic_sds_remote_device_general_frame_handler
	},
	/* SCI_BASE_REMOTE_DEVICE_STATE_FAILED */
	{
		{
			scic_sds_remote_device_default_start_handler,
			scic_sds_remote_device_default_stop_handler,
			scic_sds_remote_device_default_fail_handler,
			scic_sds_remote_device_default_destruct_handler,
			scic_sds_remote_device_default_reset_handler,
			scic_sds_remote_device_default_reset_complete_handler,
			scic_sds_remote_device_default_start_request_handler,
			scic_sds_remote_device_default_complete_request_handler,
			scic_sds_remote_device_default_continue_request_handler,
			scic_sds_remote_device_default_start_request_handler,
			scic_sds_remote_device_default_complete_request_handler
		},
		scic_sds_remote_device_default_suspend_handler,
		scic_sds_remote_device_default_resume_handler,
		scic_sds_remote_device_default_event_handler,
		scic_sds_remote_device_general_frame_handler
	},
	/* SCI_BASE_REMOTE_DEVICE_STATE_RESETTING */
	{
		{
			scic_sds_remote_device_default_start_handler,
			scic_sds_remote_device_resetting_state_stop_handler,
			scic_sds_remote_device_default_fail_handler,
			scic_sds_remote_device_default_destruct_handler,
			scic_sds_remote_device_default_reset_handler,
			scic_sds_remote_device_resetting_state_reset_complete_handler,
			scic_sds_remote_device_default_start_request_handler,
			scic_sds_remote_device_resetting_state_complete_request_handler,
			scic_sds_remote_device_default_continue_request_handler,
			scic_sds_remote_device_default_start_request_handler,
			scic_sds_remote_device_resetting_state_complete_request_handler
		},
		scic_sds_remote_device_default_suspend_handler,
		scic_sds_remote_device_default_resume_handler,
		scic_sds_remote_device_default_event_handler,
		scic_sds_remote_device_general_frame_handler
	},
	/* SCI_BASE_REMOTE_DEVICE_STATE_FINAL */
	{
		{
			scic_sds_remote_device_default_start_handler,
			scic_sds_remote_device_default_stop_handler,
			scic_sds_remote_device_default_fail_handler,
			scic_sds_remote_device_default_destruct_handler,
			scic_sds_remote_device_default_reset_handler,
			scic_sds_remote_device_default_reset_complete_handler,
			scic_sds_remote_device_default_start_request_handler,
			scic_sds_remote_device_default_complete_request_handler,
			scic_sds_remote_device_default_continue_request_handler,
			scic_sds_remote_device_default_start_request_handler,
			scic_sds_remote_device_default_complete_request_handler
		},
		scic_sds_remote_device_default_suspend_handler,
		scic_sds_remote_device_default_resume_handler,
		scic_sds_remote_device_default_event_handler,
		scic_sds_remote_device_default_frame_handler
	}
};

