#TODO kill SCIC_SDS_4_ENABLED it is always true for this
#generation of silicon
EXTRA_CFLAGS += -DSCIC_SDS_4_ENABLED

#temporary until atapi support ready
EXTRA_CFLAGS += -DDISABLE_ATAPI

EXTRA_CFLAGS += -Idrivers/scsi/isci/core/ -Idrivers/scsi/isci/
obj-$(CONFIG_SCSI_ISCI) += isci.o
isci-objs := isci_module_init.o isci_phy.o isci_request.o isci_sata.o \
	     isci_remote_device.o isci_port.o isci_timers.o isci_module.o \
	     isci_host.o isci_task.o \
	     core/scic_sds_controller.o  \
	     core/scic_sds_controller_state_handlers.o \
	     core/scic_sds_phy_starting_substate_handlers.o \
	     core/scic_sds_remote_device.o    \
	     core/scic_sds_controller_states.o \
	     core/scic_sds_phy_starting_substates.o \
	     core/scic_sds_request.o \
	     core/scic_sds_stp_request.o \
	     core/scic_sds_stp_pio_request.o \
	     core/scic_sds_phy_state_handlers.o \
	     core/scic_sds_remote_node_context_state_handlers.o \
	     core/scic_sds_remote_node_context_states.o \
	     core/scic_sds_stp_packet_request.o \
	     core/scic_sds_stp_remote_device_ready_substate_handlers.o \
	     core/scic_sds_stp_remote_device_ready_substates.o \
	     core/scic_sds_stp_request_started_soft_reset_substate_handlers.o \
	     core/scic_sds_stp_request_started_soft_reset_substates.o \
	     core/scic_sds_request_state_handlers.o \
	     core/scic_sds_phy_states.o \
	     core/scic_sds_remote_device_state_handlers.o \
	     core/scic_sds_request_states.o \
	     core/scic_sds_port.o \
	     core/scic_sds_port_configuration_agent.o \
	     core/scic_sds_port_states.o \
	     core/scic_sds_remote_device_states.o \
	     core/scic_sds_port_ready_substate_handlers.o \
	     core/scic_sds_phy.o \
	     core/scic_sds_port_ready_substates.o \
	     core/scic_sds_port_state_handlers.o \
	     core/scic_sds_ssp_request_started_task_mgmt_substates.o \
	     core/scic_sds_ssp_request_started_task_mgmt_substate_handlers.o \
	     core/scic_sds_stp_request_started_packet_substates.o \
	     core/scic_sds_stp_request_started_packet_substate_handlers.o \
	     core/scic_sds_stp_request_started_udma_substate_handlers.o \
	     core/scic_sds_stp_request_started_udma_substates.o \
	     core/scic_sds_stp_request_started_non_data_substate_handlers.o \
	     core/scic_sds_stp_request_started_non_data_substates.o \
	     core/scic_sds_stp_request_started_pio_substates.o \
	     core/scic_sds_stp_request_started_pio_substate_handlers.o \
	     core/scic_sds_remote_node_context.o \
	     core/scic_sds_smp_request.o \
	     core/scic_sds_smp_request_started_substate_handlers.o \
	     core/scic_sds_smp_request_started_substates.o \
	     core/scic_sds_smp_remote_device_ready_substate_handlers.o \
	     core/scic_sds_smp_remote_device_ready_substates.o \
	     core/scic_sds_remote_node_table.o \
	     core/scic_sds_unsolicited_frame_control.o \
	     core/sci_base_controller.o \
	     core/sci_base_domain.o \
	     core/sci_base_memory_descriptor_list.o \
	     core/sci_base_object.o \
	     core/sci_base_phy.o \
	     core/sci_base_port.o \
	     core/sci_base_remote_device.o \
	     core/sci_base_request.o \
	     core/sci_base_state_machine.o \
	     core/sci_util.o
