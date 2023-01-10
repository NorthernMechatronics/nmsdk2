add_library(ble STATIC)

set(BLE_DIR ${PROJECT_SOURCE_DIR}/comms/ble)
set(BLE_TARGET_DIR ${CMAKE_CURRENT_LIST_DIR})

target_compile_definitions(
    ble PRIVATE
	-DWDXS_INCLUDED=1
	-DSEC_CMAC_CFG=1
	-DSEC_ECC_CFG=2
	-DSEC_CCM_CFG=1
	-DHCI_TR_UART=1
#	-DWSF_CS_STATS=1
#	-DWSF_BUF_STATS=1
	-DWSF_TRACE_ENABLED=1
#	-DWSF_ASSERT_ENABLED=1
)

target_compile_options(
	ble PRIVATE
	-fomit-frame-pointer
)

set(
	BLE_INCLUDES
	${BLE_DIR}/ble-profiles/include
	${BLE_DIR}/ble-profiles/include/app
	${BLE_DIR}/ble-profiles/sources/apps/app

	${BLE_DIR}/ble-profiles/sources/profiles/include
	${BLE_DIR}/ble-profiles/sources/profiles

	${BLE_DIR}/ble-profiles/sources/services

	${BLE_DIR}/ble-host/include
	${BLE_DIR}/ble-host/sources/stack/att
	${BLE_DIR}/ble-host/sources/stack/cfg
	${BLE_DIR}/ble-host/sources/stack/dm
	${BLE_DIR}/ble-host/sources/stack/hci
	${BLE_DIR}/ble-host/sources/stack/l2c
	${BLE_DIR}/ble-host/sources/stack/smp
#	${BLE_DIR}/ble-host/sources/hci/dual_chip
 
	${BLE_DIR}/thirdparty/uecc

	${BLE_TARGET_DIR}/wsf/include
	${BLE_TARGET_DIR}/wsf/include/util
	${BLE_TARGET_DIR}/ble-host/sources/hci/nm180100
	${BLE_TARGET_DIR}/ble-host/sources/hci/nm180100/apollo3
	CACHE PATH "BLE includes"
)

target_include_directories(
    ble PRIVATE
    ${HAL_INCLUDES}
    ${BLE_INCLUDES}
)

target_sources(
    ble PRIVATE 
	${BLE_DIR}/ble-profiles/sources/apps/app/app_disc.c
	${BLE_DIR}/ble-profiles/sources/apps/app/app_main.c
	${BLE_DIR}/ble-profiles/sources/apps/app/app_master.c
	${BLE_DIR}/ble-profiles/sources/apps/app/app_master_ae.c
	${BLE_DIR}/ble-profiles/sources/apps/app/app_master_leg.c
	${BLE_DIR}/ble-profiles/sources/apps/app/app_server.c
	${BLE_DIR}/ble-profiles/sources/apps/app/app_slave.c
	${BLE_DIR}/ble-profiles/sources/apps/app/app_slave_ae.c
	${BLE_DIR}/ble-profiles/sources/apps/app/app_slave_leg.c
	${BLE_DIR}/ble-profiles/sources/apps/app/app_terminal.c
	${BLE_DIR}/ble-profiles/sources/apps/app/common/app_db.c
	${BLE_DIR}/ble-profiles/sources/apps/app/common/app_hw.c
	${BLE_DIR}/ble-profiles/sources/apps/app/common/app_ui.c
	${BLE_DIR}/ble-profiles/sources/apps/app/common/ui_console.c
	${BLE_DIR}/ble-profiles/sources/apps/app/common/ui_lcd.c
	${BLE_DIR}/ble-profiles/sources/apps/app/common/ui_main.c
	${BLE_DIR}/ble-profiles/sources/apps/app/common/ui_platform.c
	${BLE_DIR}/ble-profiles/sources/apps/app/common/ui_timer.c

	${BLE_DIR}/ble-profiles/sources/profiles/anpc/anpc_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/atpc/atpc_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/atps/atps_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/bas/bas_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/blpc/blpc_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/blps/blps_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/cpp/cpps_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/cscp/cscps_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/dis/dis_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/fmpl/fmpl_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/gap/gap_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/gatt/gatt_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/glpc/glpc_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/glps/glps_db.c
	${BLE_DIR}/ble-profiles/sources/profiles/glps/glps_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/hid/hid_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/hrpc/hrpc_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/hrps/hrps_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/htpc/htpc_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/htps/htps_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/paspc/paspc_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/plxpc/plxpc_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/plxps/plxps_db.c
	${BLE_DIR}/ble-profiles/sources/profiles/plxps/plxps_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/rscp/rscps_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/scpps/scpps_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/sensor/gyro_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/sensor/temp_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/tipc/tipc_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/udsc/udsc_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/uribeacon/uricfg_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/wdxc/wdxc_ft.c
	${BLE_DIR}/ble-profiles/sources/profiles/wdxc/wdxc_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/wdxc/wdxc_stream.c
	${BLE_DIR}/ble-profiles/sources/profiles/wdxs/wdxs_au.c
	${BLE_DIR}/ble-profiles/sources/profiles/wdxs/wdxs_dc.c
	${BLE_DIR}/ble-profiles/sources/profiles/wdxs/wdxs_ft.c
	${BLE_DIR}/ble-profiles/sources/profiles/wdxs/wdxs_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/wdxs/wdxs_phy.c
	${BLE_DIR}/ble-profiles/sources/profiles/wdxs/wdxs_stream.c
	${BLE_DIR}/ble-profiles/sources/profiles/wpc/wpc_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/wspc/wspc_main.c
	${BLE_DIR}/ble-profiles/sources/profiles/wsps/wsps_main.c

	${BLE_DIR}/ble-profiles/sources/services/svc_alert.c
	${BLE_DIR}/ble-profiles/sources/services/svc_batt.c
	${BLE_DIR}/ble-profiles/sources/services/svc_bps.c
	${BLE_DIR}/ble-profiles/sources/services/svc_core.c
	${BLE_DIR}/ble-profiles/sources/services/svc_cps.c
	${BLE_DIR}/ble-profiles/sources/services/svc_cscs.c
	${BLE_DIR}/ble-profiles/sources/services/svc_cte.c
	${BLE_DIR}/ble-profiles/sources/services/svc_dis.c
	${BLE_DIR}/ble-profiles/sources/services/svc_gls.c
	${BLE_DIR}/ble-profiles/sources/services/svc_gyro.c
	${BLE_DIR}/ble-profiles/sources/services/svc_hid.c
	${BLE_DIR}/ble-profiles/sources/services/svc_hrs.c
	${BLE_DIR}/ble-profiles/sources/services/svc_hts.c
	${BLE_DIR}/ble-profiles/sources/services/svc_ipss.c
	${BLE_DIR}/ble-profiles/sources/services/svc_plxs.c
	${BLE_DIR}/ble-profiles/sources/services/svc_px.c
	${BLE_DIR}/ble-profiles/sources/services/svc_rscs.c
	${BLE_DIR}/ble-profiles/sources/services/svc_scpss.c
	${BLE_DIR}/ble-profiles/sources/services/svc_temp.c
	${BLE_DIR}/ble-profiles/sources/services/svc_time.c
	${BLE_DIR}/ble-profiles/sources/services/svc_uricfg.c
	${BLE_DIR}/ble-profiles/sources/services/svc_wdxs.c
	${BLE_DIR}/ble-profiles/sources/services/svc_wp.c
	${BLE_DIR}/ble-profiles/sources/services/svc_wss.c


	${BLE_DIR}/ble-host/sources/stack/att/att_main.c
	${BLE_DIR}/ble-host/sources/stack/att/att_uuid.c
	${BLE_DIR}/ble-host/sources/stack/att/attc_disc.c
	${BLE_DIR}/ble-host/sources/stack/att/attc_main.c
	${BLE_DIR}/ble-host/sources/stack/att/attc_proc.c
	${BLE_DIR}/ble-host/sources/stack/att/attc_read.c
	${BLE_DIR}/ble-host/sources/stack/att/attc_sign.c
	${BLE_DIR}/ble-host/sources/stack/att/attc_write.c
	${BLE_DIR}/ble-host/sources/stack/att/atts_ccc.c
	${BLE_DIR}/ble-host/sources/stack/att/atts_csf.c
	${BLE_DIR}/ble-host/sources/stack/att/atts_dyn.c
	${BLE_DIR}/ble-host/sources/stack/att/atts_ind.c
	${BLE_DIR}/ble-host/sources/stack/att/atts_main.c
	${BLE_DIR}/ble-host/sources/stack/att/atts_proc.c
	${BLE_DIR}/ble-host/sources/stack/att/atts_read.c
	${BLE_DIR}/ble-host/sources/stack/att/atts_sign.c
	${BLE_DIR}/ble-host/sources/stack/att/atts_write.c
	${BLE_DIR}/ble-host/sources/stack/cfg/cfg_stack.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_adv.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_adv_ae.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_adv_leg.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_conn.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_conn_cte.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_conn_master.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_conn_master_ae.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_conn_master_leg.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_conn_slave.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_conn_slave_ae.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_conn_slave_leg.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_conn_sm.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_dev.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_dev_priv.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_main.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_past.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_phy.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_priv.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_scan.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_scan_ae.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_scan_leg.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_sec.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_sec_lesc.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_sec_master.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_sec_slave.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_sync_ae.c
	${BLE_DIR}/ble-host/sources/stack/hci/hci_main.c
	${BLE_DIR}/ble-host/sources/stack/l2c/l2c_coc.c
	${BLE_DIR}/ble-host/sources/stack/l2c/l2c_main.c
	${BLE_DIR}/ble-host/sources/stack/l2c/l2c_master.c
	${BLE_DIR}/ble-host/sources/stack/l2c/l2c_slave.c
	${BLE_DIR}/ble-host/sources/stack/smp/smp_act.c
	${BLE_DIR}/ble-host/sources/stack/smp/smp_db.c
	${BLE_DIR}/ble-host/sources/stack/smp/smp_main.c
	${BLE_DIR}/ble-host/sources/stack/smp/smp_non.c
	${BLE_DIR}/ble-host/sources/stack/smp/smp_sc_act.c
	${BLE_DIR}/ble-host/sources/stack/smp/smp_sc_main.c
	${BLE_DIR}/ble-host/sources/stack/smp/smpi_act.c
	${BLE_DIR}/ble-host/sources/stack/smp/smpi_sc_act.c
	${BLE_DIR}/ble-host/sources/stack/smp/smpi_sc_sm.c
	${BLE_DIR}/ble-host/sources/stack/smp/smpi_sm.c
	${BLE_DIR}/ble-host/sources/stack/smp/smpr_act.c
	${BLE_DIR}/ble-host/sources/stack/smp/smpr_sc_act.c
	${BLE_DIR}/ble-host/sources/stack/smp/smpr_sc_sm.c
	${BLE_DIR}/ble-host/sources/stack/smp/smpr_sm.c
	${BLE_DIR}/ble-host/sources/sec/common/sec_aes.c
	${BLE_DIR}/ble-host/sources/sec/common/sec_aes_rev.c
	${BLE_DIR}/ble-host/sources/sec/common/sec_ccm_hci.c
	${BLE_DIR}/ble-host/sources/sec/common/sec_cmac_hci.c
	${BLE_DIR}/ble-host/sources/sec/common/sec_ecc_debug.c
	${BLE_DIR}/ble-host/sources/sec/common/sec_ecc_hci.c
	${BLE_DIR}/ble-host/sources/sec/common/sec_main.c

	${BLE_DIR}/wsf/sources/util/bda.c
	${BLE_DIR}/wsf/sources/util/bstream.c
	${BLE_DIR}/wsf/sources/util/calc128.c
	${BLE_DIR}/wsf/sources/util/crc32.c
	${BLE_DIR}/wsf/sources/util/fcs.c
	${BLE_DIR}/wsf/sources/util/prand.c
	${BLE_DIR}/wsf/sources/util/print.c
	${BLE_DIR}/wsf/sources/util/terminal.c
	${BLE_DIR}/wsf/sources/util/wstr.c

	${BLE_TARGET_DIR}/ble-host/sources/hci/nm180100/hci_core.c
	${BLE_TARGET_DIR}/ble-host/sources/hci/nm180100/hci_tr.c
	${BLE_TARGET_DIR}/ble-host/sources/hci/nm180100/hci_cmd.c
	${BLE_TARGET_DIR}/ble-host/sources/hci/nm180100/hci_cmd_ae.c
	${BLE_TARGET_DIR}/ble-host/sources/hci/nm180100/hci_cmd_cte.c
	${BLE_TARGET_DIR}/ble-host/sources/hci/nm180100/hci_cmd_past.c
	${BLE_TARGET_DIR}/ble-host/sources/hci/nm180100/hci_cmd_phy.c
	${BLE_TARGET_DIR}/ble-host/sources/hci/nm180100/hci_core_ps.c
	${BLE_TARGET_DIR}/ble-host/sources/hci/nm180100/hci_evt.c
	${BLE_TARGET_DIR}/ble-host/sources/hci/nm180100/hci_vs_ae.c
	${BLE_TARGET_DIR}/ble-host/sources/hci/nm180100/apollo3/hci_vs_apollo3.c
	${BLE_TARGET_DIR}/ble-host/sources/hci/nm180100/apollo3/hci_drv_apollo3.c

	${BLE_TARGET_DIR}/wsf/sources/port/nm180100/wsf_assert.c
	${BLE_TARGET_DIR}/wsf/sources/port/nm180100/wsf_buf.c
#	${BLE_TARGET_DIR}/wsf/sources/port/nm180100/wsf_bufio.c
	${BLE_TARGET_DIR}/wsf/sources/port/nm180100/wsf_cs.c
	${BLE_TARGET_DIR}/wsf/sources/port/nm180100/wsf_detoken.c
	${BLE_TARGET_DIR}/wsf/sources/port/nm180100/wsf_efs.c
	${BLE_TARGET_DIR}/wsf/sources/port/nm180100/wsf_heap.c
	${BLE_TARGET_DIR}/wsf/sources/port/nm180100/wsf_msg.c
#	${BLE_TARGET_DIR}/wsf/sources/port/nm180100/wsf_nvm.c
	${BLE_TARGET_DIR}/wsf/sources/port/nm180100/wsf_os.c
	${BLE_TARGET_DIR}/wsf/sources/port/nm180100/wsf_queue.c
	${BLE_TARGET_DIR}/wsf/sources/port/nm180100/wsf_timer.c
	${BLE_TARGET_DIR}/wsf/sources/port/nm180100/wsf_trace.c

	${BLE_DIR}/thirdparty/uecc/uECC_ll.c
)

install (TARGETS ble DESTINATION ${PROJECT_SOURCE_DIR}/lib/${TARGET})