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
 * This file contains the task queue implementation.
 *
 * isci_task.c
 */



#include <linux/completion.h>
#include "scic_task_request.h"
#include "scic_remote_device.h"
#include "scic_io_request.h"
#include "scic_sds_remote_device.h"
#include "scic_sds_remote_node_context.h"
#include "isci_module.h"
#include "isci_request.h"
#include "isci_sata.h"
#include "isci_task.h"



/**
 * isci_task_execute_task() - This function is one of the SAS Domain Template
 *    functions. This function is called by libsas to send a task down to
 *    hardware.
 * @task: This parameter specifies the SAS task to send.
 * @num: This parameter specifies the number of tasks to queue.
 * @gfp_flags: This parameter specifies the context of this call.
 *
 * status, zero indicates success.
 */
int isci_task_execute_task(
	struct sas_task *sas_task,
	int num,
	gfp_t gfp_flags)
{
	struct isci_host *isci_host;
	struct isci_request *request = NULL;
	struct isci_remote_device *device;
	struct list_head task_list = sas_task->list;
	struct sas_task *task, *tmp_task;
	unsigned long flags;
	unsigned long quiesce_flags = 0;
	int ret;
	enum sci_status status;


	isci_logger(trace, "num=%d\n", num);

	if (unlikely(sas_task->task_state_flags & SAS_TASK_STATE_ABORTED)) {

		isci_logger(warning, "task aborted!!\n", 0);

		isci_task_complete_for_upper_layer(
			sas_task,
			SAS_TASK_UNDELIVERED,
			SAM_STAT_TASK_ABORTED,
			isci_perform_normal_io_completion
			);
		return 0;  /* The I/O was accepted (and failed). */
	}
	if ((sas_task->dev == NULL) || (sas_task->dev->port == NULL)) {

		isci_logger(warning, "sas_task (%p) ->dev (%p) or "
			    "->dev->port (%p) == NULL\n",
			    sas_task, sas_task->dev, sas_task->dev->port);

		/* Indicate SAS_TASK_UNDELIVERED, so that the scsi midlayer
		 * removes the target.
		 */
		isci_task_complete_for_upper_layer(
			sas_task,
			SAS_TASK_UNDELIVERED,
			SAS_DEVICE_UNKNOWN,
			isci_perform_normal_io_completion
			);
		return 0;  /* The I/O was accepted (and failed). */
	}
	isci_host = isci_host_from_sas_ha(sas_task->dev->port->ha);

	/* Check if we have room for more tasks */
	ret = isci_host_can_queue(isci_host, num);

	if (unlikely(ret)) {

		isci_logger(warning, "queue full\n", 0);
		return ret;
	}
#define ISCI_DUMP_TASK_LIST
#ifdef ISCI_DUMP_TASK_LIST
	{
		int idx = num;
		int pass_count = 0;
		list_for_each_entry_safe(task, tmp_task, &task_list, list) {
			isci_logger(trace, "listdump: idx = %d; pass = %d; "
				    "task = %p, dev = %p; cmd = %p\n",
				    idx, pass_count++, task, task->dev,
				    task->uldd_task);
			--idx;

			if (!idx)
				break;
		}
	}
#endif  /* ISCI_DUMP_TASK_LIST */

	/* walk the list safely, libsas may delete tasks as we go. */
	list_for_each_entry_safe(task, tmp_task, &task_list, list) {
		/* we only do the number of tasks specified, break out even
		 * if there are more on the list.
		 */
		if (0 == num--)
			break;

		isci_logger(trace, "task = %p, num = %d; dev = %p; cmd = %p\n",
			    task, num, task->dev, task->uldd_task);

		if (unlikely((task->dev == NULL)
			     ||   (task->dev->port == NULL))) {

			isci_logger(warning, "task %p's port or dev == NULL!\n",
				    task);

			/* Indicate SAS_TASK_UNDELIVERED, so that the scsi
			 * midlayer removes the target.
			 */
			isci_task_complete_for_upper_layer(
				task,
				SAS_TASK_UNDELIVERED,
				SAS_DEVICE_UNKNOWN,
				isci_perform_normal_io_completion
				);
			/* We don't have a valid host reference, so we
			 * can't control the host queueing condition.
			 */
			continue;
		}
		device = isci_dev_from_domain_dev(task->dev);

		isci_host = isci_host_from_sas_ha(task->dev->port->ha);

		/* check if the controller hasn't started or if the device
		 * is ready but not accepting IO.
		 */
		if (device) {

			spin_lock_irqsave(&device->host_quiesce_lock,
					  quiesce_flags);
		}
		/* From this point onward, any process that needs to guarantee
		 * that there is no kernel I/O being started will have to wait
		 * for the quiesce spinlock.
		 */

		if (unlikely(
			    isci_host_get_state(isci_host) == isci_starting
			    || (device
				&& ((isci_remote_device_get_state(device)
				     == isci_ready)
				    || (isci_remote_device_get_state(device)
					== isci_host_quiesce)
				    )
				))) {

			/* Forces a retry from scsi mid layer. */
			isci_logger(warning,
				    "task %p: isci_host->status = %d, device = %p\n",
				    task,
				    isci_host_get_state(isci_host),
				    device
				    );
			if (device)
				isci_logger(warning,
					    "device->status = 0x%x\n",
					    isci_remote_device_get_state(device)
					    );

			/* Indicate QUEUE_FULL so that the scsi midlayer
			 * retries.
			 */
			isci_task_complete_for_upper_layer(
				task,
				SAS_TASK_COMPLETE,
				SAS_QUEUE_FULL,
				isci_perform_normal_io_completion
				);
			isci_host_can_dequeue(isci_host, 1);
		}
		/* the device is going down... */
		else if (unlikely(!device
				  /* Quicker way to device removal: */
				  || (isci_ready_for_io != isci_remote_device_get_state(device))
				  )) {

			isci_logger(warning,
				    "task %p: isci_host->status = %d, device = %p\n",
				    task, isci_host_get_state(isci_host), device
				    );
			if (device)
				isci_logger(warning,
					    "device->status = 0x%x\n",
					    isci_remote_device_get_state(device)
					    );

			/* Indicate SAS_TASK_UNDELIVERED, so that the scsi
			 * midlayer removes the target.
			 */
			isci_task_complete_for_upper_layer(
				task,
				SAS_TASK_UNDELIVERED,
				SAS_DEVICE_UNKNOWN,
				isci_perform_normal_io_completion
				);
			isci_host_can_dequeue(isci_host, 1);

		} else {

			/* Guard against abort requests while starting the
			 * I/O.
			 */
			spin_lock_irqsave(&task->task_state_lock, flags);

			if (unlikely(task->task_state_flags
				     & SAS_TASK_STATE_ABORTED)) {

				/* Task was aborted by the midlayers before
				 * it was dequeued here.
				 */
				spin_unlock_irqrestore(&task->task_state_lock,
						       flags
						       );

				isci_logger(warning,
					    "task %p already aborted!\n",
					    task
					    );

				isci_task_complete_for_upper_layer(
					task,
					SAS_TASK_UNDELIVERED,
					SAM_STAT_TASK_ABORTED,
					isci_perform_normal_io_completion
					);

				isci_host_can_dequeue(isci_host, 1);

			} else {

				spin_unlock_irqrestore(
					&task->task_state_lock, flags
					);

				/* build and send the request. */
				status = isci_request_execute(
					isci_host,
					task,
					&request,
					gfp_flags
					);

				spin_lock_irqsave(&task->task_state_lock, flags);

				if (likely(SCI_SUCCESS == status)) {

					task->task_state_flags
						|= SAS_TASK_AT_INITIATOR;

					spin_unlock_irqrestore(
						&task->task_state_lock, flags
						);

				} else {

					spin_unlock_irqrestore(
						&task->task_state_lock, flags
						);

					/* Indicate QUEUE_FULL so that the scsi
					 * midlayer retries. if the request
					 * failed for rmenote device reasons,
					 * it gets returned as
					 * SAS_TASK_UNDELIVERED next time
					 * through.
					 */
					isci_task_complete_for_upper_layer(
						task,
						SAS_TASK_COMPLETE,
						SAS_QUEUE_FULL,
						isci_perform_normal_io_completion
						);
					isci_host_can_dequeue(isci_host, 1);
				}
			}
		}
		if (device) {

			spin_unlock_irqrestore(&device->host_quiesce_lock,
					       quiesce_flags
					       );
		}
	}
	return 0;
}



/**
 * isci_task_request_build() - This function builds the task request object.
 * @isci_host: This parameter specifies the ISCI host object
 * @request: This parameter points to the isci_request object allocated in the
 *    request construct function.
 * @tmf: This parameter is the task management struct to be built
 *
 * SCI_SUCCESS on successfull completion, or specific failure code.
 */
static enum sci_status isci_task_request_build(
	struct isci_host *isci_host,
	struct isci_request **isci_request,
	struct isci_tmf *isci_tmf)
{
	SCI_REMOTE_DEVICE_HANDLE_T sci_device;
	enum sci_status status = SCI_FAILURE;
	struct isci_request *request;
	struct isci_remote_device *isci_device;
/*	struct sci_sas_identify_address_frame_protocols dev_protocols; */
	struct smp_discover_response_protocols dev_protocols;


	isci_logger(trace, "isci_tmf = %p\n", isci_tmf);

	isci_device = isci_tmf->device;
	sci_device = isci_device->sci_device_handle;

	/* do common allocation and init of request object. */
	status = isci_request_alloc_tmf(
		isci_host,
		isci_tmf,
		&request,
		isci_device,
		GFP_ATOMIC
		);

	if (unlikely(status != SCI_SUCCESS))
		goto out;

	/* let the core do it's construct. */
	status = scic_task_request_construct(
		isci_host->core_controller,
		sci_device,
		SCI_CONTROLLER_INVALID_IO_TAG,
		request,
		request->sci_request_mem_ptr,
		&request->sci_request_handle
		);

	if (unlikely(SCI_SUCCESS != status)) {

		isci_logger(warning,
			    "scic_task_request_construct failed - "
			    "status = 0x%x\n",
			    status
			    );
		goto errout;
	}

	sci_object_set_association(
		request->sci_request_handle,
		request
		);

	scic_remote_device_get_protocols(
		sci_device,
		&dev_protocols
		);

	/* let the core do it's protocol
	 * specific construction.
	 */
	if (dev_protocols.u.bits.attached_ssp_target) {

		isci_tmf->proto = SAS_PROTOCOL_SSP;
		status = scic_task_request_construct_ssp(
			request->sci_request_handle
			);
		if (unlikely(SCI_SUCCESS != status))
			goto errout;
	}

	if (dev_protocols.u.bits.attached_stp_target) {

		isci_tmf->proto = SAS_PROTOCOL_SATA;
		status = isci_sata_management_task_request_build(request);

		if (unlikely(SCI_SUCCESS != status))
			goto errout;
	}

	goto out;

 errout:

	/* release the dma memory if we fail. */
	isci_request_free(isci_host, request);
	request = NULL;

 out:
	*isci_request = request;
	return status;
}

/**
 * isci_tmf_timeout_cb() - This function is called as a kernel callback when
 *    the timeout period for the TMF has expired.
 *
 *
 */
static void isci_tmf_timeout_cb(void *tmf_request_arg)
{
	struct isci_request *request = (struct isci_request *)tmf_request_arg;
	struct isci_tmf *tmf = isci_request_access_tmf(request);
	enum sci_status status;

	ASSERT(tmf_task == request->ttype);

	/* This task management request has timed-out.  Terminate the request
	 * so that the request eventually completes to the requestor in the
	 * request completion callback path.
	 */
	/* Note - the timer callback function itself has provided spinlock
	 * exclusion from the start and completion paths.  No need to take
	 * the request->isci_host->scic_lock here.
	 */

	if (tmf->timeout_timer != NULL) {

		/* Call the users callback, if any. */
		if (tmf->cb_state_func != NULL)
			tmf->cb_state_func(isci_tmf_timed_out, tmf,
					   tmf->cb_data);

		/* Terminate the TMF transmit request. */
		status = scic_controller_terminate_request(
			request->isci_host->core_controller,
			request->isci_device->sci_device_handle,
			request->sci_request_handle
			);

		isci_logger(trace, "tmf_request = %p; tmf = %p; status = %d\n",
			    request, tmf, status
			    );

	} else {

		isci_logger(trace,
			    "timer already canceled! tmf_request = %p; tmf = %p\n",
			    request, tmf
			    );
	}
	/* No need to unlock since the caller to this callback is doing it for
	 * us.
	 * request->isci_host->scic_lock
	 */
}
/**
 * isci_task_execute_tmf() - This function builds and sends a task request,
 *    then waits for the completion.
 * @isci_host: This parameter specifies the ISCI host object
 * @tmf: This parameter is the pointer to the task management structure for
 *    this request.
 * @timeout_ms: This parameter specifies the timeout period for the task
 *    management request.
 *
 * TMF_RESP_FUNC_COMPLETE on successful completion of the TMF (this includes
 * error conditions reported in the IU status), or TMF_RESP_FUNC_FAILED.
 */
int isci_task_execute_tmf(
	struct isci_host *isci_host,
	struct isci_tmf *tmf,
	unsigned long timeout_ms)
{
	DECLARE_COMPLETION_ONSTACK(completion);
	enum sci_status status = SCI_FAILURE;
	SCI_REMOTE_DEVICE_HANDLE_T sci_device;
	struct isci_remote_device *isci_device = tmf->device;
	struct isci_request *request;
	int ret = TMF_RESP_FUNC_FAILED;
	unsigned long flags;

	/* sanity check, return TMF_RESP_FUNC_FAILED
	 * if the device is not there and ready.
	 */
	if (unlikely(!isci_device
		     || ((isci_ready_for_io != isci_remote_device_get_state(isci_device))
			 && (isci_host_quiesce != isci_remote_device_get_state(isci_device)))
		     )) {
		isci_logger(trace, "isci_device = %p not ready (%d)\n",
			    isci_device,
			    isci_remote_device_get_state(isci_device)
			    );
		goto out;

	} else
		isci_logger(trace, "isci_device = %p\n", isci_device);

	sci_device = isci_device->sci_device_handle;

	/* Assign the pointer to the TMF's completion kernel wait structure. */
	tmf->complete = &completion;

	isci_task_request_build(
		isci_host,
		&request,
		tmf
		);

	if (unlikely(NULL == request)) {

		isci_logger(warning, "isci_task_request_build failed\n", 0);
		goto out;
	}

	/* Allocate the TMF timeout timer. */
	tmf->timeout_timer = isci_timer_create(
		&isci_host->timer_list_struct,
		isci_host,
		request,
		isci_tmf_timeout_cb
		);

	spin_lock_irqsave(&isci_host->scic_lock, flags);

	/* Start the timer. */
	if (tmf->timeout_timer)
		isci_timer_start(tmf->timeout_timer, timeout_ms);
	else
		isci_logger(warning, "isci_timer_create failed!!!!\n", 0);

	/* start the TMF io. */
	status = scic_controller_start_task(
		isci_host->core_controller,
		sci_device,
		request->sci_request_handle,
		SCI_CONTROLLER_INVALID_IO_TAG
		);

	if (unlikely(SCI_SUCCESS != status)) {

		isci_logger(warning,
			    "start_io failed - status = 0x%x, request = %p\n",
			    status,
			    request
			    );
		goto cleanup_request;
	}

	/* Call the users callback, if any. */
	if (tmf->cb_state_func != NULL)
		tmf->cb_state_func(isci_tmf_started, tmf, tmf->cb_data);

	/* Change the state of the TMF-bearing request to "started". */
	isci_request_change_state(request, started);

	/* add the request to the remote device request list. */
	list_add(&request->dev_node, &isci_device->reqs_in_process);

	spin_unlock_irqrestore(&isci_host->scic_lock, flags);

	/* Wait for the TMF to complete, or a timeout. */
	wait_for_completion(&completion);

	isci_print_tmf(tmf);

	if (likely(SCI_SUCCESS == tmf->status)) {

		ret =  TMF_RESP_FUNC_COMPLETE;

	} else if (SCI_FAILURE_IO_RESPONSE_VALID == tmf->status) {

		isci_logger(trace, " tmf.status == "
			    "SCI_FAILURE_IO_RESPONSE_VALID\n", 0);
		ret =  TMF_RESP_FUNC_COMPLETE;
	}
	/* Else - leave the default "failed" status alone. */

	isci_logger(trace, "completed request = %p\n", request);

	if (request->io_request_completion != NULL) {

		/* The fact that this is non-NULL for a TMF request
		 * means there is a thread waiting for this TMF to
		 * finish.
		 */
		complete(request->io_request_completion);
	}

	spin_lock_irqsave(&isci_host->scic_lock, flags);

 cleanup_request:

	/* Clean up the timer if needed. */
	if (tmf->timeout_timer) {

		isci_timer_stop(tmf->timeout_timer);
		isci_timer_free(&isci_host->timer_list_struct,
				tmf->timeout_timer);
		tmf->timeout_timer = NULL;
	}
	spin_unlock_irqrestore(&isci_host->scic_lock, flags);

	isci_request_free(isci_host, request);
 out:
	return ret;
}

void isci_task_build_tmf(
	struct isci_tmf *tmf,
	struct isci_remote_device *isci_device,
	enum isci_tmf_function_codes code,
	void (*tmf_sent_cb)(enum isci_tmf_cb_state,
			    struct isci_tmf *,
			    void *),
	void *cb_data)
{

	isci_logger(trace, "isci_device = %p\n", isci_device);
	memset(tmf, 0, sizeof(*tmf));
/*	PRINT_TMF(*tmf); */

	tmf->device        = isci_device;
	tmf->tmf_code      = code;
	tmf->timeout_timer = NULL;
	tmf->cb_state_func = tmf_sent_cb;
	tmf->cb_data       = cb_data;
}


static struct isci_request *isci_task_get_request_from_task(
	struct sas_task *task,
	struct isci_host **isci_host,
	struct isci_remote_device **isci_device)
{

	struct isci_request *request = NULL;
	unsigned long flags;

	spin_lock_irqsave(&task->task_state_lock, flags);

	request = task->lldd_task;

	/* If task is already done, the request isn't valid */
	if (!(task->task_state_flags & SAS_TASK_STATE_DONE)
	    && (task->task_state_flags & SAS_TASK_AT_INITIATOR)
	    && (request != NULL)) {

		if (NULL != isci_host)
			*isci_host = request->isci_host;
		if (NULL != isci_device)
			*isci_device = request->isci_device;
	}
	spin_unlock_irqrestore(&task->task_state_lock, flags);

	return request;
}

/**
 * isci_task_validate_request_to_abort() - This function checks the given I/O
 *    against the "started" state.  If the request is still "started", it's
 *    state is changed to aborted. NOTE: isci_host->scic_lock MUST BE HELD
 *    BEFORE CALLING THIS FUNCTION.
 * @isci_request: This parameter specifies the request object to control.
 * @isci_host: This parameter specifies the ISCI host object
 * @isci_device: This is the device to which the request is pending.
 * @aborted_io_completion: This is a completion structure that will be added to
 *    the request in case it is changed to aborting; this completion is
 *    triggered when the request is fully completed.
 *
 * Either "started" on successful change of the task status to "aborted", or
 * "unallocated" if the task cannot be controlled.
 */
static enum isci_request_status isci_task_validate_request_to_abort(
	struct isci_request *isci_request,
	struct isci_host *isci_host,
	struct isci_remote_device *isci_device,
	struct completion *aborted_io_completion)
{
	enum isci_request_status old_state = unallocated;

	/* Only abort the task if it's in the
	 *  device's request_in_process list
	 */
	if (isci_request && !list_empty(&isci_request->dev_node)) {
		old_state = isci_request_change_started_to_aborted(
			isci_request, aborted_io_completion);

		/* Only abort requests in the started state. */
		if (old_state != started)
			old_state = unallocated;
	}

	return old_state;
}

static void isci_request_cleanup_completed_loiterer(
	struct isci_host *isci_host,
	struct isci_remote_device *isci_device,
	struct isci_request *isci_request)
{
	struct sas_task *task = isci_request_access_task(isci_request);
	unsigned long flags;

	isci_logger(warning, "isci_device=%p, request=%p, task=%p\n",
		    isci_device, isci_request, isci_request->ttype_ptr.io_task_ptr
		    );

	spin_lock_irqsave(&isci_host->scic_lock, flags);
	list_del_init(&isci_request->dev_node);
	if (task != NULL)
		task->lldd_task = NULL;
	spin_unlock_irqrestore(&isci_host->scic_lock, flags);

	isci_request_free(isci_host, isci_request);
}
/**
 * isci_terminate_request_core() - This function will terminate the given
 *    request, and wait for it to complete.  This function must only be called
 *    from a thread that can wait.  Note that the request is terminated and
 *    completed (back to the host, if started there).
 * @isci_host: This SCU.
 * @isci_device: The target.
 * @isci_request: The I/O request to be terminated.
 *
 *
 */
static void isci_terminate_request_core(
	struct isci_host *isci_host,
	struct isci_remote_device *isci_device,
	struct isci_request *isci_request,
	struct completion *request_completion)
{
	enum sci_status status                 = SCI_SUCCESS;
	bool was_terminated         = false;
	bool needs_cleanup_handling = false;
	enum isci_request_status request_status;
	unsigned long flags;

/*	isci_logger(trace, "device = %p; request = %p\n", */
	isci_logger(warning, "device = %p; request = %p\n",
		    isci_device, isci_request);

	/* Peek at the current status of the request.  This will tell
	 * us if there was special handling on the request such that it
	 * needs to be detached and freed here.
	 */
	spin_lock_irqsave(&isci_request->state_lock, flags);
	request_status = isci_request_get_state(isci_request);

	if ((isci_request->ttype == io_task) /* TMFs are in their own thread */
	    && ((request_status == aborted)
		|| (request_status == aborting)
		|| (request_status == terminating)
		)
	    ) {

		/* The completion routine won't free a request in
		 * the aborted/aborting/terminating state, so we do
		 * it here.
		 */
		needs_cleanup_handling = true;
	}
	spin_unlock_irqrestore(&isci_request->state_lock, flags);

	spin_lock_irqsave(&isci_host->scic_lock, flags);
	/* Make sure the request wasn't just sitting around signalling
	 * device condition (if the request handle is NULL, then the
	 * request completed but needed additional handling here).
	 */
	if (isci_request->sci_request_handle != NULL) {

		was_terminated = true;
		status = scic_controller_terminate_request(
			isci_host->core_controller,
			isci_device->sci_device_handle,
			isci_request->sci_request_handle
			);
	}
	spin_unlock_irqrestore(&isci_host->scic_lock, flags);

	if (unlikely(status != SCI_SUCCESS)) {

		isci_logger(error,
			    "scic_controller_terminate_request"
			    " returned = 0x%x\n",
			    status
			    );

		/* The only time the request to terminate will
		 * fail is when the io request is completed and
		 * being aborted.
		 */

	} else {

		if (was_terminated) {

			isci_logger(trace, "before completion wait (%p)\n",
				    request_completion);

			/* Wait here for the request to complete. */
			wait_for_completion(request_completion);

			isci_logger(trace, "after completion wait (%p)\n",
				    request_completion);

		}

		if (needs_cleanup_handling) {

			isci_request_cleanup_completed_loiterer(
				isci_host, isci_device, isci_request
				);
		}
	}
}
static void isci_terminate_request(
	struct isci_host *isci_host,
	struct isci_remote_device *isci_device,
	struct isci_request *isci_request,
	enum isci_request_status new_request_state)
{
	enum isci_request_status old_state;

	DECLARE_COMPLETION_ONSTACK(request_completion);
	unsigned long flags;

	spin_lock_irqsave(&isci_host->scic_lock, flags);

	/* Change state to "new_request_state" if it is currently "started" */
	old_state = isci_request_change_started_to_newstate(
		isci_request,
		&request_completion,
		new_request_state
		);
	spin_unlock_irqrestore(&isci_host->scic_lock, flags);

	if (old_state == started) {

		/* This request was not already being aborted. If it had been,
		 * then the aborting I/O (ie. the TMF request) would not be in
		 * the aborting state, and thus would be terminated here.  Note
		 * that since the TMF completion's call to the kernel function
		 * "complete()" does not happen until the pending I/O request
		 * terminate fully completes, we do not have to implement a
		 * special wait here for already aborting requests - the
		 * termination of the TMF request will force the request
		 * to finish it's already started terminate.
		 */
		isci_terminate_request_core(isci_host, isci_device,
					    isci_request, &request_completion
					    );
	}
}
/**
 * isci_terminate_pending_requests() - This function will change the all of the
 *    requests on the given device's state to "aborting", will terminate the
 *    requests, and wait for them to complete.  This function must only be
 *    called from a thread that can wait.  Note that the requests are all
 *    terminated and completed (back to the host, if started there).
 * @isci_host: This parameter specifies SCU.
 * @isci_device: This parameter specifies the target.
 *
 *
 */
void isci_terminate_pending_requests(
	struct isci_host *isci_host,
	struct isci_remote_device *isci_device,
	enum isci_request_status new_request_state)
{
	struct isci_request *isci_request;
	struct sas_task *task;
	bool done = false;
	unsigned long flags;

	isci_logger(trace, "isci_device = %p (new request state = %d)\n",
		    isci_device, new_request_state);

	#define ISCI_TERMINATE_SHOW_PENDING_REQUESTS
	#ifdef ISCI_TERMINATE_SHOW_PENDING_REQUESTS
	{
		struct isci_request *request;

		/* Only abort the task if it's in the
		 * device's request_in_process list
		 */
		list_for_each_entry(request,
				    &isci_device->reqs_in_process,
				    dev_node) {
			isci_logger(trace, "isci_device = %p; request is on "
				    "reqs_in_process list: %p\n",
				    isci_device, request);
		}
	}
	#endif /* ISCI_TERMINATE_SHOW_PENDING_REQUESTS */

	/* Clean up all pending requests. */
	do {

		spin_lock_irqsave(&isci_host->scic_lock, flags);
		if (list_empty(&isci_device->reqs_in_process)) {

			done = true;
			spin_unlock_irqrestore(&isci_host->scic_lock, flags);

			isci_logger(trace, "isci_device = %p; done.\n",
				    isci_device);

		} else {

			/* The list was not empty - grab the first request. */
			isci_request = list_first_entry(
				&isci_device->reqs_in_process,
				struct isci_request, dev_node
				);
			/* Note that we are not expecting to have to control
			 * the target to abort the request.
			 */
			isci_request->complete_in_target = true;

			spin_unlock_irqrestore(&isci_host->scic_lock, flags);

			/* Get the libsas task reference. */
			task = isci_request_access_task(isci_request);

			isci_logger(trace, "isci_device=%p request=%p; task=%p\n",
				    isci_device, isci_request, task);

			/* Mark all still pending I/O with the selected next
			 * state.
			 */
			isci_terminate_request(isci_host, isci_device,
					       isci_request, new_request_state
					       );

			/* Set the 'done' state on the task. */
			if (task)
				isci_task_all_done(task);
		}

	} while (!done);
}

/**
 * isci_task_send_lu_reset_sas() - This function is called by of the SAS Domain
 *    Template functions.
 * @lun: This parameter specifies the lun to be reset.
 *
 * status, zero indicates success.
 */
static int isci_task_send_lu_reset_sas(
	struct isci_host *isci_host,
	struct isci_remote_device *isci_device,
	u8 *lun)
{
	struct isci_tmf tmf;
	int ret = TMF_RESP_FUNC_FAILED;

	isci_logger(trace, "isci_host = %p, isci_device = %p\n",
		    isci_host, isci_device
		    );
	/* Send the LUN reset to the target.  By the time the call returns,
	 * the TMF has fully exected in the target (in which case the return
	 * value is "TMF_RESP_FUNC_COMPLETE", or the request timed-out (or
	 * was otherwise unable to be executed ("TMF_RESP_FUNC_FAILED").
	 */
	isci_task_build_tmf(&tmf, isci_device, isci_tmf_ssp_lun_reset, NULL,
			    NULL);

	#define ISCI_LU_RESET_TIMEOUT_MS 2000 /* 2 second timeout. */
	ret = isci_task_execute_tmf(isci_host, &tmf, ISCI_LU_RESET_TIMEOUT_MS);

	if (ret == TMF_RESP_FUNC_COMPLETE) {

		isci_logger(trace, "%p: TMF_LU_RESET passed\n", isci_device);

	} else {

		isci_logger(trace, "%p: TMF_LU_RESET failed (%x)\n",
			    isci_device, ret);
	}

	return ret;

}

/**
 * isci_task_lu_reset() - This function is one of the SAS Domain Template
 *    functions. This is one of the Task Management functoins called by libsas,
 *    to reset the given lun. Note the assumption that while this call is
 *    executing, no I/O will be sent by the host to the device.
 * @lun: This parameter specifies the lun to be reset.
 *
 * status, zero indicates success.
 */
int isci_task_lu_reset(
	struct domain_device *domain_device,
	u8 *lun)
{
	struct isci_host *isci_host = NULL;
	struct isci_remote_device *isci_device = NULL;
	int ret;
	bool device_stopping = false;

	if (domain_device == NULL) {

		isci_logger(warning, "domain_device == NULL\n", 0);
		return TMF_RESP_FUNC_FAILED;
	}

	isci_device = isci_dev_from_domain_dev(domain_device);

	if (domain_device->port != NULL)
		isci_host = isci_host_from_sas_ha(domain_device->port->ha);

	isci_logger(trace,
		    "domain_device=%p, isci_host=%p; isci_device=%p\n",
		    domain_device, isci_host, isci_device
		    );

	if (isci_device != NULL)
		device_stopping = (isci_device->status == isci_stopping)
				  || (isci_device->status == isci_stopped);

	/* If there is a device reset pending on any request in the
	 * device's list, fail this LUN reset request in order to
	 * escalate to the device reset.
	 */
	if ((isci_device == NULL)
	    || (isci_host == NULL)
	    || ((isci_host != NULL)
		&& (isci_device != NULL)
		&& (device_stopping
		    || (isci_device_is_reset_pending(isci_host, isci_device))
		    ))) {

		isci_logger(warning, "No dev (%p), no host (%p), or "
			    "RESET PENDING: domain_device=%p\n",
			    isci_device, isci_host, domain_device
			    );
		return TMF_RESP_FUNC_FAILED;
	}

	/* Stop I/O to the remote device. */
	isci_device_set_host_quiesce_lock_state(isci_device, true);

	/* Send the task management part of the reset. */
	if (ISCI_IS_PROTO_STP_OR_SATA(domain_device->tproto)) {

		ret = isci_task_send_lu_reset_sata(
			isci_host, isci_device, lun
			);

	} else {

		ret = isci_task_send_lu_reset_sas(isci_host, isci_device, lun);
	}
	/* If the LUN reset worked, all the I/O can now be terminated. */
	if (ret ==  TMF_RESP_FUNC_COMPLETE)
		/* Terminate all I/O now. */
		isci_terminate_pending_requests(isci_host, isci_device, terminating);


	/* Resume I/O to the remote device. */
	isci_device_set_host_quiesce_lock_state(isci_device, false);

	return ret;
}


/*	 int (*lldd_clear_nexus_port)(struct asd_sas_port *); */
int isci_task_clear_nexus_port(
	struct asd_sas_port *port)
{
	isci_logger(trace, "\n", 0);

	return TMF_RESP_FUNC_FAILED;
}



int isci_task_clear_nexus_ha(
	struct sas_ha_struct *ha)
{
	isci_logger(trace, "\n", 0);

	return TMF_RESP_FUNC_FAILED;
}

int isci_task_I_T_nexus_reset(
	struct domain_device *dev)
{
	isci_logger(trace, "\n", 0);

	return TMF_RESP_FUNC_FAILED;
}


/* Task Management Functions. Must be called from process context.	 */

/**
 * isci_abort_task_process_cb() - This is a helper function for the abort task
 *    TMF command.  It manages the request state with respect to the successful
 *    transmission / completion of the abort task request.
 * @cb_state: This parameter specifies when this function was called - after
 *    the TMF request has been started and after it has timed-out.
 * @tmf: This parameter specifies the TMF in progress.
 *
 *
 */
static void isci_abort_task_process_cb(
	enum isci_tmf_cb_state cb_state,
	struct isci_tmf *tmf,
	void *cb_data)
{
	struct isci_request *old_request;

	old_request = (struct isci_request *)cb_data;

	isci_logger(trace, "tmf=%p, old_request=%p\n", tmf, old_request);

	switch (cb_state) {

	case isci_tmf_started:
		/* The TMF has been started.  Nothing to do here, since the
		 * request state was already set to "aborted" by the abort
		 * task function.
		 */
		ASSERT(old_request->status == aborted);
		break;

	case isci_tmf_timed_out:

		/* Set the task's state to "aborting", since the abort task
		 * function thread set it to "aborted" (above) in anticipation
		 * of the task management request working correctly.  Since the
		 * timeout has now fired, the TMF request failed.  We set the
		 * state such that the request completion will indicate the
		 * device is no longer present.
		 */
		isci_request_change_state(old_request, aborting);
		break;

	default:
		isci_logger(error, "Bad cb_state (%d): tmf=%p, "
			    "old_request=%p\n",
			    cb_state, tmf, old_request);
		break;
	}
}

/**
 * isci_task_abort_task() - This function is one of the SAS Domain Template
 *    functions. This function is called by libsas to abort a specified task.
 * @task: This parameter specifies the SAS task to abort.
 *
 * status, zero indicates success.
 */
int isci_task_abort_task(
	struct sas_task *task)
{
	DECLARE_COMPLETION_ONSTACK(aborted_io_completion);
	struct isci_request *old_request = NULL;
	struct isci_remote_device *isci_device = NULL;
	struct isci_host *isci_host = NULL;
	struct isci_tmf tmf;
	int ret = TMF_RESP_FUNC_FAILED;
	unsigned long flags;
	bool any_dev_reset, device_stopping;

	isci_logger(trace, "task = %p\n", task);

	/* Get the isci_request reference from the task.  Note that
	 * this check does not depend on the pending request list
	 * in the device, because tasks driving resets may land here
	 * after completion in the core.
	 */
	old_request = isci_task_get_request_from_task(task, &isci_host,
						      &isci_device);


	/* Check if the device has been / is currently being removed.
	 * If so, no task management will be done, and the I/O will
	 * be terminated.
	 */
	device_stopping = (isci_device->status == isci_stopping)
			  || (isci_device->status == isci_stopped);

#ifdef NOMORE
	/* This abort task function is the first stop of the libsas error
	 * handler thread. Since libsas is executing in a thread with a
	 * referernce to the "task" parameter, that task cannot be completed
	 * directly back to the upper layers.  In order to make sure that
	 * the task is managed correctly if this abort task fails, set the
	 * "SAS_TASK_STATE_ABORTED" bit now such that completions up the
	 * stack will be intercepted and only allowed to happen in the
	 * libsas SCSI error handler thread.
	 */
	spin_lock_irqsave(&task->task_state_lock, flags);
	task->task_state_flags |= SAS_TASK_STATE_ABORTED;
	spin_unlock_irqrestore(&task->task_state_lock, flags);
#endif  /* NOMORE */

	/* This version of the driver will fail abort requests for
	 * SATA/STP.  Failing the abort request this way will cause the
	 * SCSI error handler thread to escalate to LUN reset
	 */
	if (ISCI_IS_PROTO_STP_OR_SATA(task->task_proto) && !device_stopping) {

		isci_logger(warning,
			    " task %p is for a STP/SATA device;"
			    " returning TMF_RESP_FUNC_FAILED\n"
			    " to cause a LUN reset...\n", task
			    );
		ret = TMF_RESP_FUNC_FAILED;

		goto out;
	}

	isci_logger(trace, " old_request == %p\n", old_request);

	spin_lock_irqsave(&task->task_state_lock, flags);

	/* Don't do resets to stopping devices. */
	if (device_stopping)
		task->task_state_flags &= ~SAS_TASK_NEED_DEV_RESET;

	/* See if there is a pending device reset for this device. */
	any_dev_reset = task->task_state_flags & SAS_TASK_NEED_DEV_RESET;

	spin_unlock_irqrestore(&task->task_state_lock, flags);

	if ((isci_device != NULL) && !device_stopping)
		any_dev_reset = any_dev_reset
				|| isci_device_is_reset_pending(isci_host,
								isci_device
								);

	/* If the extraction of the request reference from the task
	 * failed, then the request has been completed (or if there is a
	 * pending reset then this abort request function must be failed
	 * in order to escalate to the target reset).
	 */
	if ((old_request == NULL)
	    || ((old_request != NULL)
		&& (old_request->sci_request_handle == NULL)
		&& (old_request->complete_in_target))
	    || any_dev_reset
	    ) {

		spin_lock_irqsave(&task->task_state_lock, flags);

		/* If the device reset task flag is set, fail the task
		 * management request.  Otherwise, the original request
		 * has completed.
		 */
		if (any_dev_reset) {

			/* Turn off the task's DONE to make sure this
			 * task is escalated to a target reset.
			 */
			task->task_state_flags &= ~SAS_TASK_STATE_DONE;

			/* Fail the task management request in order to
			 * escalate to the target reset.
			 */
			ret = TMF_RESP_FUNC_FAILED;

			isci_logger(trace, "Failing task abort in order to "
				    "escalate to target reset because\n"
				    "SAS_TASK_NEED_DEV_RESET is set for "
				    "task %p on dev %p\n",
				    task, isci_device);

		} else {

			ret = TMF_RESP_FUNC_COMPLETE;

			isci_logger(trace, "abort task not needed for %p\n",
				    task);

			/* The request has already completed and there
			 * is nothing to do here other than to set the task
			 * done bit, and indicate that the task abort function
			 * was sucessful.
			 */
			isci_set_task_doneflags(task);

			/* Set the abort bit to make sure that libsas sticks the
			 * task in the completed task queue.
			 */
/*			task->task_state_flags |= SAS_TASK_STATE_ABORTED; */

			/* Check for the situation where the request was
			 * left around on the device list but the
			 * request already completed.
			 */
			if ((old_request != NULL)
			    && (old_request->sci_request_handle == NULL)) {

				isci_request_cleanup_completed_loiterer(
					isci_host, isci_device, old_request
					);
			}
		}
		spin_unlock_irqrestore(&task->task_state_lock, flags);

		goto out;
	}

	spin_lock_irqsave(&isci_host->scic_lock, flags);

	/* Sanity check the request status, and set the I/O kernel completion
	 * struct that will be triggered when the request completes.
	 */
	if (isci_task_validate_request_to_abort(
		    old_request,
		    isci_host,
		    isci_device,
		    &aborted_io_completion)
	    == unallocated) {

		isci_logger(trace,
			    " old_request not valid for device = %p\n",
			    isci_device
			    );
		old_request = NULL;
	}

	if (unlikely(old_request == NULL)) {

		/* There is no isci_request attached to the sas_task.
		 * It must have been completed and detached.
		 */
		isci_logger(trace, " old_request == NULL\n", 0);

		spin_unlock_irqrestore(&isci_host->scic_lock, flags);

		/* Set the state on the task. */
		isci_task_all_done(task);

		ret = TMF_RESP_FUNC_COMPLETE;
		goto out;
	}
	if (ISCI_IS_PROTO_SMP(task->task_proto) || device_stopping) {

		if (device_stopping) {
			isci_logger(trace, " device is stopping, thus no TMF\n", 0);
		} else {
			isci_logger(trace, " request is SMP, thus no TMF\n", 0);
		}

		old_request->complete_in_target = true;

		spin_unlock_irqrestore(&isci_host->scic_lock, flags);

		/* Set the state on the task. */
		isci_task_all_done(task);

		ret = TMF_RESP_FUNC_COMPLETE;

		/* Stopping and SMP devices are not sent a TMF, and are not
		 * reset, but the outstanding I/O request is terminated here.
		 *
		 * Clean up the request on our side, and wait for the aborted
		 * I/O to complete.
		 */
		isci_terminate_request_core(isci_host, isci_device, old_request,
					    &aborted_io_completion);
	} else {
		/* Fill in the tmf stucture */
		isci_task_build_tmf(&tmf, isci_device, isci_tmf_ssp_task_abort,
				    isci_abort_task_process_cb, old_request);

		tmf.io_tag = scic_io_request_get_io_tag(
			old_request->sci_request_handle
			);

		spin_unlock_irqrestore(&isci_host->scic_lock, flags);

		#define ISCI_ABORT_TASK_TIMEOUT_MS 500 /* half second timeout. */
		ret = isci_task_execute_tmf(isci_host, &tmf,
					    ISCI_ABORT_TASK_TIMEOUT_MS);

		if (ret == TMF_RESP_FUNC_COMPLETE) {
			old_request->complete_in_target = true;

			/* Clean up the request on our side, and wait for the aborted I/O to
			 * complete.
			 */
			isci_terminate_request_core(isci_host, isci_device, old_request,
						    &aborted_io_completion);

			/* Set the state on the task. */
			isci_task_all_done(task);
		} else
			isci_logger(error, "isci_task_send_tmf failed\n", 0);
	}
 out:

	return ret;
}

/**
 * isci_task_abort_task_set() - This function is one of the SAS Domain Template
 *    functions. This is one of the Task Management functoins called by libsas,
 *    to abort all task for the given lun.
 * @d_device: This parameter specifies the domain device associated with this
 *    request.
 * @lun: This parameter specifies the lun associated with this request.
 *
 * status, zero indicates success.
 */
int isci_task_abort_task_set(
	struct domain_device *d_device,
	u8 *lun)
{
	isci_logger(trace, "\n", 0);

	return TMF_RESP_FUNC_FAILED;
}


/**
 * isci_task_clear_aca() - This function is one of the SAS Domain Template
 *    functions. This is one of the Task Management functoins called by libsas.
 * @d_device: This parameter specifies the domain device associated with this
 *    request.
 * @lun: This parameter specifies the lun	 associated with this request.
 *
 * status, zero indicates success.
 */
int isci_task_clear_aca(
	struct domain_device *d_device,
	u8 *lun)
{
	isci_logger(trace, "\n", 0);

	return TMF_RESP_FUNC_FAILED;
}



/**
 * isci_task_clear_task_set() - This function is one of the SAS Domain Template
 *    functions. This is one of the Task Management functoins called by libsas.
 * @d_device: This parameter specifies the domain device associated with this
 *    request.
 * @lun: This parameter specifies the lun	 associated with this request.
 *
 * status, zero indicates success.
 */
int isci_task_clear_task_set(
	struct domain_device *d_device,
	u8 *lun)
{
	isci_logger(trace, "\n", 0);

	return TMF_RESP_FUNC_FAILED;
}


/**
 * isci_task_query_task() - This function is implemented to cause libsas to
 *    correctly escalate the failed abort to a LUN or target reset (this is
 *    because sas_scsi_find_task libsas function does not correctly interpret
 *    all return codes from the abort task call).  When TMF_RESP_FUNC_SUCC is
 *    returned, libsas turns this into a LUN reset; when FUNC_FAILED is
 *    returned, libsas will turn this into a target reset
 * @task: This parameter specifies the sas task being queried.
 * @lun: This parameter specifies the lun associated with this request.
 *
 * status, zero indicates success.
 */
int isci_task_query_task(
	struct sas_task *task)
{
	isci_logger(trace, "\n", 0);

	/* See if there is a pending device reset for this device. */
	if (task->task_state_flags & SAS_TASK_NEED_DEV_RESET)
		return TMF_RESP_FUNC_FAILED;
	else
		return TMF_RESP_FUNC_SUCC;
}

/**
 * isci_task_request_complete() - This function is called by the sci core when
 *    an task request completes.
 * @isci_host: This parameter specifies the ISCI host object
 * @request: This parameter is the completed isci_request object.
 * @completion_status: This parameter specifies the completion status from the
 *    sci core.
 *
 * none.
 */
void isci_task_request_complete(
	struct isci_host *isci_host,
	struct isci_request *request,
	enum sci_task_status completion_status)
{
	struct isci_remote_device *isci_device = request->isci_device;
	enum isci_request_status old_state;
	struct isci_tmf *tmf = isci_request_access_tmf(request);
	struct completion *tmf_complete;

	isci_logger(trace, "request = %p, status=%d\n",
		    request, completion_status);

	old_state = isci_request_change_state(request, completed);

	tmf->status = completion_status;
	request->complete_in_target = true;

	if (SAS_PROTOCOL_SSP == tmf->proto) {

		memcpy(&tmf->resp.resp_iu,
		       scic_io_request_get_response_iu_address(
			       request->sci_request_handle
			       ),
		       sizeof(struct sci_ssp_response_iu));

	} else if (SAS_PROTOCOL_SATA == tmf->proto) {

		memcpy(&tmf->resp.d2h_fis,
		       scic_stp_io_request_get_d2h_reg_address(
			       request->sci_request_handle
			       ),
		       sizeof(struct sata_fis_reg_d2h)
		       );
	}

	/* Manage the timer if it is still running. */
	if (tmf->timeout_timer) {

		isci_timer_stop(tmf->timeout_timer);
		isci_timer_free(&isci_host->timer_list_struct,
				tmf->timeout_timer);
		tmf->timeout_timer = NULL;
	}

	/* PRINT_TMF( ((struct isci_tmf *)request->task)); */
	tmf_complete = tmf->complete;

	scic_controller_complete_task(
		isci_host->core_controller,
		isci_device->sci_device_handle,
		request->sci_request_handle
		);
	/* NULL the request handle to make sure it cannot be terminated
	 *  or completed again.
	 */
	request->sci_request_handle = NULL;

	isci_request_change_state(request, unallocated);
	list_del_init(&request->dev_node);

	/* The task management part completes last. */
	complete(tmf_complete);
}


/**
 * isci_task_ssp_request_get_lun() - This function is called by the sci core to
 *    retrieve the lun for a given task request.
 * @request: This parameter is the isci_request object.
 *
 * lun for specified task request.
 */
u32 isci_task_ssp_request_get_lun(
	struct isci_request *request)
{
	struct isci_tmf *isci_tmf = isci_request_access_tmf(request);

	isci_logger(trace, "lun = %d\n", isci_tmf->lun[0]);
/* @todo: build lun from array of bytes to 32 bit */
	return isci_tmf->lun[0];
}

/**
 * isci_task_ssp_request_get_function() - This function is called by the sci
 *    core to retrieve the function for a given task request.
 * @request: This parameter is the isci_request object.
 *
 * function code for specified task request.
 */
u8 isci_task_ssp_request_get_function(
	struct isci_request *request)
{
	struct isci_tmf *isci_tmf = isci_request_access_tmf(request);

	isci_logger(trace, "func = %d\n", isci_tmf->tmf_code);

	return isci_tmf->tmf_code;
}

/**
 * isci_task_ssp_request_get_io_tag_to_manage() - This function is called by
 *    the sci core to retrieve the io tag for a given task request.
 * @request: This parameter is the isci_request object.
 *
 * io tag for specified task request.
 */
u16 isci_task_ssp_request_get_io_tag_to_manage(
	struct isci_request *request)
{
	u16 io_tag = SCI_CONTROLLER_INVALID_IO_TAG;

	if (tmf_task == request->ttype) {

		struct isci_tmf *tmf = isci_request_access_tmf(request);
		io_tag = tmf->io_tag;
	}
	isci_logger(trace,
		    "request = %p, io_tag = %d\n",
		    request,
		    io_tag
		    );

	return io_tag;
}

/**
 * isci_task_ssp_request_get_response_data_address() - This function is called
 *    by the sci core to retrieve the response data address for a given task
 *    request.
 * @request: This parameter is the isci_request object.
 *
 * response data address for specified task request.
 */
void *isci_task_ssp_request_get_response_data_address(
	struct isci_request *request)
{
	struct isci_tmf *isci_tmf = isci_request_access_tmf(request);

	isci_logger(trace, "\n", 0);
	return &isci_tmf->resp.resp_iu;
}

/**
 * isci_task_ssp_request_get_response_data_length() - This function is called
 *    by the sci core to retrieve the response data length for a given task
 *    request.
 * @request: This parameter is the isci_request object.
 *
 * response data length for specified task request.
 */
u32 isci_task_ssp_request_get_response_data_length(
	struct isci_request *request)
{
	struct isci_tmf *isci_tmf = isci_request_access_tmf(request);

	isci_logger(trace, "\n", 0);
	return sizeof(isci_tmf->resp.resp_iu);
}

/**
 * isci_queuecommand() - This function checks the state of the referenced
 *    device to see to make sure it is in a state in which commands can be sent
 *    to it.
 * @scsi_cmd: This parameter specifies the command to be queued.
 * @donefunc: This parameter is provided by the kernel to drive queueing if the
 *    device is in the correct state.
 *
 * SCSI_MLQUEUE_DEVICE_BUSY if the device is valid and not ready for I/O, else
 * return the value from sas_queuecommand.
 */
int isci_queuecommand(
	struct scsi_cmnd *scsi_cmd,
	void (*donefunc)(struct scsi_cmnd *))
{
	enum isci_status current_status;
	struct isci_remote_device *isci_remdev
		= isci_dev_from_domain_dev(
		sdev_to_domain_dev(scsi_cmd->device));

	if (unlikely(isci_remdev == NULL)) {

		isci_logger(warning, "isci_remdev is GONE!\n", 0);

		/* Queue the command so it can fail. */
		return sas_queuecommand(scsi_cmd, donefunc);
	}
	current_status = isci_remote_device_get_state(isci_remdev);

	/* If the device is stopping, queue the command so that the lldd
	 * can fail it.  This will cause libsas to stop queuing I/O for
	 * the device that is stopping.
	 */
	if ((current_status == isci_ready_for_io)
	    || (current_status == isci_stopping)) {

		return sas_queuecommand(scsi_cmd, donefunc);

	} else {

		isci_logger(warning, "isci_remdev %p is NOT READY\n",
			    isci_remdev);
		return SCSI_MLQUEUE_DEVICE_BUSY;
	}
}

/**
 * isci_bus_reset_handler() - This function performs a target reset of the
 *    device referenced by "cmd'.  This function is exported through the
 *    "struct scsi_host_template" structure such that it is called when an I/O
 *    recovery process has escalated to a target reset. Note that this function
 *    is called from the scsi error handler event thread, so may block on calls.
 * @scsi_cmd: This parameter specifies the target to be reset.
 *
 * SUCCESS if the reset process was successful, else FAILED.
 */
int isci_bus_reset_handler(struct scsi_cmnd *cmd)
{
	unsigned long flags = 0;
	struct isci_host *isci_host = NULL;
	enum sci_status status;
	int base_status;
	struct isci_remote_device *isci_dev
		= isci_dev_from_domain_dev(
		sdev_to_domain_dev(cmd->device));

	isci_logger(warning, "cmd %p, isci_dev %p\n", cmd, isci_dev);

	if (unlikely(isci_dev == NULL)) {

		isci_logger(warning, "isci_dev is GONE!\n", 0);

		return TMF_RESP_FUNC_COMPLETE; /* Nothing to reset. */
	}
	if (isci_dev->isci_port != NULL)
		isci_host = isci_dev->isci_port->isci_host;

	if (isci_host != NULL)
		spin_lock_irqsave(&isci_host->scic_lock, flags);

	status = scic_remote_device_reset(isci_dev->sci_device_handle);
	if (status != SCI_SUCCESS) {

		if (isci_host != NULL)
			spin_unlock_irqrestore(&isci_host->scic_lock, flags);

		isci_logger(warning, "scic_remote_device_reset(%p) "
			    "returned %d!\n",
			    isci_dev, status);

		return TMF_RESP_FUNC_FAILED;
	}
	if (isci_host != NULL)
		spin_unlock_irqrestore(&isci_host->scic_lock, flags);

	/* Stop I/O to the remote device. */
	isci_device_set_host_quiesce_lock_state(isci_dev, true);

	/* Make sure all pending requests are able to be fully terminated. */
	isci_device_clear_reset_pending(isci_dev);

	/* Terminate in-progress I/O now. */
	isci_remote_device_nuke_requests(isci_dev);

	/* Call into the libsas default handler (which calls sas_phy_reset). */
	base_status = sas_eh_bus_reset_handler(cmd);

	if (base_status != SUCCESS) {

		/* There can be cases where the resets to individual devices
		 * behind an expander will fail because of an unplug of the
		 * expander itself.
		 */
		isci_logger(warning,
			    "sas_eh_bus_reset_handler(%p) returned %d!\n",
			    cmd, base_status);
	}

	/* WHAT TO DO HERE IF sas_phy_reset FAILS? */

	if (isci_host != NULL)
		spin_lock_irqsave(&isci_host->scic_lock, flags);
	status
		= scic_remote_device_reset_complete(isci_dev->sci_device_handle);

	if (isci_host != NULL)
		spin_unlock_irqrestore(&isci_host->scic_lock, flags);

	if (status != SCI_SUCCESS) {

		isci_logger(warning,
			    "scic_remote_device_reset_complete(%p) "
			    "returned %d!\n",
			    isci_dev, status);
	}
	/* WHAT TO DO HERE IF scic_remote_device_reset_complete FAILS? */

	isci_logger(warning, "cmd %p, isci_dev %p complete.\n", cmd, isci_dev);

	/* Resume I/O to the remote device. */
	isci_device_set_host_quiesce_lock_state(isci_dev, false);

	return TMF_RESP_FUNC_COMPLETE;
}
