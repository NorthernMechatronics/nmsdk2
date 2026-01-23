add_library(SWL2001 STATIC)

set(SWL2001_DIR ${PROJECT_SOURCE_DIR}/comms/SWL2001)
set(SWL2001_CORE_DIR ${SWL2001_DIR}/smtc_modem_core)

set(SWL2001_TARGET_DIR ${CMAKE_CURRENT_LIST_DIR})

target_compile_definitions(
    # SWL2001 PRIVATE
    SWL2001 PUBLIC
    -DRADIO=sx1262
    # SX126x transceiver
    -DSX126X
    -DSX1262

    # region
    -DREGION=all
    -DREGION_AS_923
    -DREGION_AU_915
    -DREGION_CN_470
    -DREGION_CN_470_RP_1_0
    -DREGION_EU_868
    -DREGION_IN_865
    -DREGION_KR_920
    -DREGION_RU_864
    -DREGION_US_915

    # region parameters
    -DRP_VERSION=RP2_103
    -DRP2_103

    #-DSMTC_D2D
    -DADD_FUOTA=1
    -DUSE_FLASH_READ_MODIFY_WRITE
	-DUSE_FUOTA
    -DFRAG_MAX_NB=1000
    -DSMTC_MULTICAST
    -DADD_CLASS_B
    -DADD_CLASS_C
    -DADD_SMTC_STREAM
    -DADD_SMTC_FILE_UPLOAD
    -DADD_SMTC_ALC_SYNC

    -DCRYPTO=SOFT

    -DMODEM_TRACE=yes
    -DMODEM_HAL_DBG_TRACE=1
    -DMODEM_HAL_DEEP_DBG_TRACE=0
    # -DMODEM_HAL_DBG_TRACE_RP=MODEM_HAL_FEATURE_ON

    -DUSE_GNSS=no

    -DMIDDLEWARE=no

    -DNUMBER_OF_STACKS=1
)

set(
    SWL2001_INCLUDES
    # SX126x transceiver
    ${SWL2001_CORE_DIR}/radio_drivers/sx126x_driver/src
    ${SWL2001_CORE_DIR}/smtc_modem_crypto/soft_secure_element
    

    ${SWL2001_DIR}
    ${SWL2001_DIR}/smtc_modem_api
	${SWL2001_CORE_DIR}
	${SWL2001_CORE_DIR}/modem_config
	${SWL2001_CORE_DIR}/modem_core
	${SWL2001_CORE_DIR}/modem_supervisor
	${SWL2001_CORE_DIR}/device_management
	${SWL2001_CORE_DIR}/modem_services
    ${SWL2001_CORE_DIR}/modem_services/stream_packages
    ${SWL2001_CORE_DIR}/modem_services/cloud_dm_package
	${SWL2001_CORE_DIR}/lorawan_api
	${SWL2001_CORE_DIR}/smtc_modem_services/headers
	${SWL2001_CORE_DIR}/smtc_modem_services/src
	${SWL2001_CORE_DIR}/smtc_modem_services/src/stream
	${SWL2001_CORE_DIR}/smtc_modem_services/src/file_upload
	${SWL2001_CORE_DIR}/smtc_modem_services/src/alc_sync
	${SWL2001_CORE_DIR}/smtc_modem_services
    ${SWL2001_CORE_DIR}/modem_utilities
	${SWL2001_CORE_DIR}/smtc_ral/src
	${SWL2001_CORE_DIR}/smtc_ralf/src
	${SWL2001_CORE_DIR}/lr1mac
	${SWL2001_CORE_DIR}/lr1mac/src
	${SWL2001_CORE_DIR}/lr1mac/src/services
	${SWL2001_CORE_DIR}/lr1mac/src/lr1mac_class_c
	${SWL2001_CORE_DIR}/lr1mac/src/lr1mac_class_b
    ${SWL2001_CORE_DIR}/lr1mac/src/services/smtc_multicast
	${SWL2001_CORE_DIR}/radio_planner/src
	${SWL2001_CORE_DIR}/smtc_modem_crypto
	${SWL2001_CORE_DIR}/smtc_modem_crypto/smtc_secure_element
	${SWL2001_CORE_DIR}/lorawan_api
    ${SWL2001_CORE_DIR}/lorawan_manager
    ${SWL2001_CORE_DIR}/lorawan_packages/lorawan_certification
    ${SWL2001_CORE_DIR}/lorawan_packages/application_layer_clock_synchronization
    ${SWL2001_CORE_DIR}/lorawan_packages/fragmented_data_block_transport
    ${SWL2001_CORE_DIR}/lorawan_packages/remote_multicast_setup
	${SWL2001_CORE_DIR}/lr1mac/src/smtc_real/src
    ${SWL2001_DIR}/smtc_modem_hal

    ${SWL2001_TARGET_DIR}
    ${SWL2001_TARGET_DIR}/radio_hal
    ${SWL2001_TARGET_DIR}/smtc_hal
    ${SWL2001_TARGET_DIR}/smtc_modem_hal

    CACHE PATH "LoRa Basic Modem includes"
)


target_include_directories(
    SWL2001 PRIVATE
    ${HAL_INCLUDES}
    ${SWL2001_INCLUDES}
)

target_sources(
    SWL2001 PRIVATE
    # SX126x transceiver
    ${SWL2001_CORE_DIR}/radio_drivers/sx126x_driver/src/sx126x.c
    ${SWL2001_CORE_DIR}/radio_drivers/sx126x_driver/src/sx126x_lr_fhss.c
    ${SWL2001_CORE_DIR}/radio_drivers/sx126x_driver/src/lr_fhss_mac.c
    ${SWL2001_CORE_DIR}/smtc_ral/src/ral_sx126x.c
    ${SWL2001_CORE_DIR}/smtc_ralf/src/ralf_sx126x.c
    ${SWL2001_CORE_DIR}/smtc_modem_crypto/soft_secure_element/aes.c
    ${SWL2001_CORE_DIR}/smtc_modem_crypto/soft_secure_element/cmac.c
    ${SWL2001_CORE_DIR}/smtc_modem_crypto/soft_secure_element/soft_se.c

    # regions
    ${SWL2001_CORE_DIR}/lr1mac/src/smtc_real/src/region_as_923.c
    ${SWL2001_CORE_DIR}/lr1mac/src/smtc_real/src/region_au_915.c
    ${SWL2001_CORE_DIR}/lr1mac/src/smtc_real/src/region_cn_470.c
    ${SWL2001_CORE_DIR}/lr1mac/src/smtc_real/src/region_cn_470_rp_1_0.c
    ${SWL2001_CORE_DIR}/lr1mac/src/smtc_real/src/region_eu_868.c
    ${SWL2001_CORE_DIR}/lr1mac/src/smtc_real/src/region_in_865.c
    ${SWL2001_CORE_DIR}/lr1mac/src/smtc_real/src/region_kr_920.c
    ${SWL2001_CORE_DIR}/lr1mac/src/smtc_real/src/region_ru_864.c
    ${SWL2001_CORE_DIR}/lr1mac/src/smtc_real/src/region_us_915.c

    # core
	${SWL2001_CORE_DIR}/lorawan_api/lorawan_api.c 
	# ${SWL2001_CORE_DIR}/device_management/dm_downlink.c 
    ${SWL2001_CORE_DIR}/modem_services/cloud_dm_package/cloud_dm_package.c
	# ${SWL2001_CORE_DIR}/device_management/modem_context.c
    ${SWL2001_CORE_DIR}/modem_utilities/modem_event_utilities.c
    ${SWL2001_CORE_DIR}/modem_utilities/modem_core.c
    ${SWL2001_CORE_DIR}/lorawan_manager/lorawan_join_management.c
    ${SWL2001_CORE_DIR}/lorawan_manager/lorawan_send_management.c
    ${SWL2001_CORE_DIR}/lorawan_manager/lorawan_dwn_ack_management.c
    ${SWL2001_CORE_DIR}/lorawan_manager/lorawan_cid_request_management.c
	${SWL2001_CORE_DIR}/smtc_modem.c
	${SWL2001_CORE_DIR}/smtc_modem_test.c
	${SWL2001_CORE_DIR}/modem_utilities/fifo_ctrl.c
	# ${SWL2001_CORE_DIR}/modem_utilities/modem_utilities.c 
	# ${SWL2001_CORE_DIR}/modem_services/smtc_modem_services_hal.c
	${SWL2001_CORE_DIR}/lorawan_packages/lorawan_certification/lorawan_certification.c
	${SWL2001_CORE_DIR}/modem_supervisor/modem_supervisor_light.c
    ${SWL2001_CORE_DIR}/modem_supervisor/modem_tx_protocol_manager.c

    # modem services
	# ${SWL2001_CORE_DIR}/modem_services/smtc_clock_sync.c
	# ${SWL2001_CORE_DIR}/smtc_modem_services/src/alc_sync/alc_sync.c
    ${SWL2001_CORE_DIR}/lorawan_packages/application_layer_clock_synchronization/v1.0.0/lorawan_alcsync_v1.0.0.c
    ${SWL2001_CORE_DIR}/lorawan_packages/application_layer_clock_synchronization/v2.0.0/lorawan_alcsync_v2.0.0.c
	# ${SWL2001_CORE_DIR}/smtc_modem_services/src/stream/stream.c
    ${SWL2001_CORE_DIR}/modem_services/stream_packages/stream.c
	# ${SWL2001_CORE_DIR}/smtc_modem_services/src/stream/rose.c
    ${SWL2001_CORE_DIR}/modem_services/stream_packages/rose.c
	# ${SWL2001_CORE_DIR}/smtc_modem_services/src/file_upload/file_upload.c
    ${SWL2001_CORE_DIR}/modem_services/lfu_service/file_upload.c

    # LR1MAC
	${SWL2001_CORE_DIR}/lr1mac/src/lr1_stack_mac_layer.c
	${SWL2001_CORE_DIR}/lr1mac/src/lr1mac_core.c
	${SWL2001_CORE_DIR}/lr1mac/src/lr1mac_utilities.c
	${SWL2001_CORE_DIR}/lr1mac/src/smtc_real/src/smtc_real.c
	${SWL2001_CORE_DIR}/lr1mac/src/services/smtc_duty_cycle.c
	${SWL2001_CORE_DIR}/lr1mac/src/services/smtc_lbt.c
	${SWL2001_CORE_DIR}/lr1mac/src/lr1mac_class_c/lr1mac_class_c.c
	${SWL2001_CORE_DIR}/lr1mac/src/lr1mac_class_b/smtc_beacon_sniff.c
	${SWL2001_CORE_DIR}/lr1mac/src/lr1mac_class_b/smtc_ping_slot.c

	${SWL2001_CORE_DIR}/lr1mac/src/services/smtc_multicast/smtc_multicast.c

    ${SWL2001_CORE_DIR}/lorawan_manager/lorawan_class_b_management.c
    #Ver. 1
    ${SWL2001_CORE_DIR}/lorawan_packages/fragmented_data_block_transport/v1.0.0/fragmentation_helper_v1.0.0.c
    ${SWL2001_CORE_DIR}/lorawan_packages/fragmented_data_block_transport/v1.0.0/lorawan_fragmentation_package_v1.0.0.c
    ${SWL2001_CORE_DIR}/lorawan_packages/application_layer_clock_synchronization/v1.0.0/lorawan_alcsync_v1.0.0.c
    ${SWL2001_CORE_DIR}/lorawan_packages/remote_multicast_setup/v1.0.0/lorawan_remote_multicast_setup_package_v1.0.0.c

    #Ver. 2
    # ${SWL2001_CORE_DIR}/lorawan_packages/fragmented_data_block_transport/v2.0.0/fragmentation_helper_v2.0.0.c
    # ${SWL2001_CORE_DIR}/lorawan_packages/fragmented_data_block_transport/v2.0.0/lorawan_fragmentation_package_v2.0.0.c
    # ${SWL2001_CORE_DIR}/lorawan_packages/application_layer_clock_synchronization/v2.0.0/lorawan_alcsync_v2.0.0.c
    # ${SWL2001_CORE_DIR}/lorawan_packages/remote_multicast_setup/v2.0.0/lorawan_remote_multicast_setup_package_v2.0.0.c
    # ${SWL2001_CORE_DIR}/smtc_modem_crypto/soft_secure_element/cmac.c
    # ${SWL2001_CORE_DIR}/smtc_modem_crypto/soft_secure_element/aes.c


	${SWL2001_CORE_DIR}/smtc_modem_crypto/smtc_modem_crypto.c

	${SWL2001_CORE_DIR}/radio_planner/src/radio_planner.c
	# ${SWL2001_CORE_DIR}/radio_planner/src/radio_planner_hal.c


    ${SWL2001_TARGET_DIR}/radio_hal/ral_sx126x_bsp.c
    ${SWL2001_TARGET_DIR}/radio_hal/sx126x_hal.c
    ${SWL2001_TARGET_DIR}/radio_hal/radio_utilities.c
    ${SWL2001_TARGET_DIR}/smtc_modem_hal/smtc_modem_hal.c

    ${SWL2001_TARGET_DIR}/smtc_hal/smtc_hal_flash.c
    ${SWL2001_TARGET_DIR}/smtc_hal/smtc_hal_gpio.c
    ${SWL2001_TARGET_DIR}/smtc_hal/smtc_hal_lp_timer.c
    ${SWL2001_TARGET_DIR}/smtc_hal/smtc_hal_mcu.c
    ${SWL2001_TARGET_DIR}/smtc_hal/smtc_hal_rng.c
    ${SWL2001_TARGET_DIR}/smtc_hal/smtc_hal_rtc.c
    ${SWL2001_TARGET_DIR}/smtc_hal/smtc_hal_spi.c
    ${SWL2001_TARGET_DIR}/smtc_hal/smtc_hal_trace.c
    ${SWL2001_TARGET_DIR}/smtc_hal/smtc_hal_watchdog.c
)

install (TARGETS SWL2001 DESTINATION ${PROJECT_SOURCE_DIR}/lib/${TARGET})