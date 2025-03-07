
set(BLE_DIR ${PROJECT_SOURCE_DIR}/comms/ble)
set(BLE_TARGET_DIR ${CMAKE_CURRENT_LIST_DIR})

set(
	BLE_INCLUDES
	${BLE_DIR}/ble-profiles/include
	${BLE_DIR}/ble-profiles/sources/af
	${BLE_DIR}/ble-profiles/sources/profiles
	${BLE_DIR}/ble-profiles/sources/profiles/anpc
	${BLE_DIR}/ble-profiles/sources/profiles/atpc
	${BLE_DIR}/ble-profiles/sources/profiles/atps
	${BLE_DIR}/ble-profiles/sources/profiles/bas
	${BLE_DIR}/ble-profiles/sources/profiles/blpc
	${BLE_DIR}/ble-profiles/sources/profiles/blps
	${BLE_DIR}/ble-profiles/sources/profiles/cpp
	${BLE_DIR}/ble-profiles/sources/profiles/cscp
	${BLE_DIR}/ble-profiles/sources/profiles/dis
	${BLE_DIR}/ble-profiles/sources/profiles/fmpl
	${BLE_DIR}/ble-profiles/sources/profiles/gap
	${BLE_DIR}/ble-profiles/sources/profiles/gatt
	${BLE_DIR}/ble-profiles/sources/profiles/glpc
	${BLE_DIR}/ble-profiles/sources/profiles/glps
	${BLE_DIR}/ble-profiles/sources/profiles/hid
	${BLE_DIR}/ble-profiles/sources/profiles/hrpc
	${BLE_DIR}/ble-profiles/sources/profiles/hrps
	${BLE_DIR}/ble-profiles/sources/profiles/htpc
	${BLE_DIR}/ble-profiles/sources/profiles/htps
	${BLE_DIR}/ble-profiles/sources/profiles/include
	${BLE_DIR}/ble-profiles/sources/profiles/paspc
	${BLE_DIR}/ble-profiles/sources/profiles/plxpc
	${BLE_DIR}/ble-profiles/sources/profiles/plxps
	${BLE_DIR}/ble-profiles/sources/profiles/rscp
	${BLE_DIR}/ble-profiles/sources/profiles/scpps
	${BLE_DIR}/ble-profiles/sources/profiles/sensor
	${BLE_DIR}/ble-profiles/sources/profiles/tipc
	${BLE_DIR}/ble-profiles/sources/profiles/udsc
	${BLE_DIR}/ble-profiles/sources/profiles/uribeacon
	${BLE_DIR}/ble-profiles/sources/profiles/wdxc
	${BLE_DIR}/ble-profiles/sources/profiles/wdxs
	${BLE_DIR}/ble-profiles/sources/profiles/wpc
	${BLE_DIR}/ble-profiles/sources/profiles/wspc
	${BLE_DIR}/ble-profiles/sources/profiles/wsps
	${BLE_DIR}/ble-profiles/sources/services

	${BLE_DIR}/ble-host/include
	${BLE_DIR}/ble-host/sources/sec/common
	${BLE_DIR}/ble-host/sources/stack/att
	${BLE_DIR}/ble-host/sources/stack/cfg
	${BLE_DIR}/ble-host/sources/stack/dm
	${BLE_DIR}/ble-host/sources/stack/hci
	${BLE_DIR}/ble-host/sources/stack/l2c
	${BLE_DIR}/ble-host/sources/stack/smp
 
	${BLE_DIR}/thirdparty/uecc

	${BLE_DIR}/platform/include

	${BLE_DIR}/ble-mesh-model/include
	${BLE_DIR}/ble-mesh-model/sources/include

	${BLE_DIR}/ble-mesh-profile/include
	${BLE_DIR}/ble-mesh-profile/sources/bearer/adv
	${BLE_DIR}/ble-mesh-profile/sources/bearer/gatt
	${BLE_DIR}/ble-mesh-profile/sources/ble-profiles/profiles
	${BLE_DIR}/ble-mesh-profile/sources/ble-profiles/services
	${BLE_DIR}/ble-mesh-profile/sources/stack/cfg
	${BLE_DIR}/ble-mesh-profile/sources/stack/access
	${BLE_DIR}/ble-mesh-profile/sources/stack/api
	${BLE_DIR}/ble-mesh-profile/sources/stack/bearer
	${BLE_DIR}/ble-mesh-profile/sources/stack/friend
	${BLE_DIR}/ble-mesh-profile/sources/stack/include
	${BLE_DIR}/ble-mesh-profile/sources/stack/local_config
	${BLE_DIR}/ble-mesh-profile/sources/stack/lpn
	${BLE_DIR}/ble-mesh-profile/sources/stack/network
	${BLE_DIR}/ble-mesh-profile/sources/stack/security
	${BLE_DIR}/ble-mesh-profile/sources/stack/seq_manager
	${BLE_DIR}/ble-mesh-profile/sources/stack/services/timers
	${BLE_DIR}/ble-mesh-profile/sources/stack/services/utils
	${BLE_DIR}/ble-mesh-profile/sources/stack/transports
	${BLE_DIR}/ble-mesh-profile/sources/provisioning

	${BLE_DIR}/ble-mesh-apps/include
	${BLE_DIR}/ble-mesh-apps/sources/common

	${BLE_TARGET_DIR}/wsf/include
	${BLE_TARGET_DIR}/wsf/include/util
	${BLE_TARGET_DIR}/ble-host/sources/hci/nm180100
	${BLE_TARGET_DIR}/ble-host/sources/hci/nm180100/apollo3
	CACHE PATH "BLE includes"
)

if (BUILD_BLE)

message("BLE development enabled")

add_library(ble STATIC)

target_compile_definitions(
	ble PRIVATE
	-DBT_VER=11
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

target_include_directories(
	ble PRIVATE
	${HAL_INCLUDES}
	${BLE_INCLUDES}
)

target_sources(
	ble PRIVATE 
	${BLE_DIR}/ble-profiles/sources/af/app_disc.c
	${BLE_DIR}/ble-profiles/sources/af/app_main.c
	${BLE_DIR}/ble-profiles/sources/af/app_master.c
	${BLE_DIR}/ble-profiles/sources/af/app_master_ae.c
	${BLE_DIR}/ble-profiles/sources/af/app_master_leg.c
	${BLE_DIR}/ble-profiles/sources/af/app_server.c
	${BLE_DIR}/ble-profiles/sources/af/app_slave.c
	${BLE_DIR}/ble-profiles/sources/af/app_slave_ae.c
	${BLE_DIR}/ble-profiles/sources/af/app_slave_leg.c
	${BLE_DIR}/ble-profiles/sources/af/app_terminal.c

	${BLE_DIR}/ble-profiles/sources/af/common/app_db.c
	${BLE_DIR}/ble-profiles/sources/af/common/app_hw.c
	${BLE_DIR}/ble-profiles/sources/af/common/app_ui.c
	${BLE_DIR}/ble-profiles/sources/af/common/ui_console.c
	${BLE_DIR}/ble-profiles/sources/af/common/ui_lcd.c
	${BLE_DIR}/ble-profiles/sources/af/common/ui_main.c
	${BLE_DIR}/ble-profiles/sources/af/common/ui_platform.c
	${BLE_DIR}/ble-profiles/sources/af/common/ui_timer.c

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
	${BLE_DIR}/ble-host/sources/stack/att/att_eatt.c
	${BLE_DIR}/ble-host/sources/stack/att/attc_disc.c
	${BLE_DIR}/ble-host/sources/stack/att/attc_main.c
	${BLE_DIR}/ble-host/sources/stack/att/attc_proc.c
	${BLE_DIR}/ble-host/sources/stack/att/attc_read.c
	${BLE_DIR}/ble-host/sources/stack/att/attc_sign.c
	${BLE_DIR}/ble-host/sources/stack/att/attc_write.c
	${BLE_DIR}/ble-host/sources/stack/att/attc_eatt.c
	${BLE_DIR}/ble-host/sources/stack/att/atts_ccc.c
	${BLE_DIR}/ble-host/sources/stack/att/atts_csf.c
	${BLE_DIR}/ble-host/sources/stack/att/atts_dyn.c
	${BLE_DIR}/ble-host/sources/stack/att/atts_ind.c
	${BLE_DIR}/ble-host/sources/stack/att/atts_main.c
	${BLE_DIR}/ble-host/sources/stack/att/atts_proc.c
	${BLE_DIR}/ble-host/sources/stack/att/atts_read.c
	${BLE_DIR}/ble-host/sources/stack/att/atts_sign.c
	${BLE_DIR}/ble-host/sources/stack/att/atts_write.c
	${BLE_DIR}/ble-host/sources/stack/att/atts_eatt.c
	${BLE_DIR}/ble-host/sources/stack/cfg/cfg_stack.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_adv.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_adv_ae.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_adv_leg.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_bis_master.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_bis_slave.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_cis.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_cis_master.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_cis_slave.c
	${BLE_DIR}/ble-host/sources/stack/dm/dm_cis_sm.c
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
	${BLE_DIR}/ble-host/sources/stack/dm/dm_iso.c
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
	${BLE_TARGET_DIR}/ble-host/sources/hci/nm180100/hci_cmd_bis.c
	${BLE_TARGET_DIR}/ble-host/sources/hci/nm180100/hci_cmd_cis.c
	${BLE_TARGET_DIR}/ble-host/sources/hci/nm180100/hci_cmd_cte.c
	${BLE_TARGET_DIR}/ble-host/sources/hci/nm180100/hci_cmd_iso.c
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
	${BLE_TARGET_DIR}/wsf/sources/port/nm180100/wsf_nvm.c
	${BLE_TARGET_DIR}/wsf/sources/port/nm180100/wsf_os.c
	${BLE_TARGET_DIR}/wsf/sources/port/nm180100/wsf_queue.c
	${BLE_TARGET_DIR}/wsf/sources/port/nm180100/wsf_timer.c
	${BLE_TARGET_DIR}/wsf/sources/port/nm180100/wsf_trace.c

	${BLE_DIR}/thirdparty/uecc/uECC_ll.c
	
	${BLE_DIR}/ble-mesh-model/sources/bindings/mmdl_bindings_main.c
	${BLE_DIR}/ble-mesh-model/sources/common/mmdl_common_main.c
	${BLE_DIR}/ble-mesh-model/sources/genbatterycl/mmdl_gen_battery_cl_main.c
	${BLE_DIR}/ble-mesh-model/sources/genbatterysr/mmdl_gen_battery_sr_main.c
	${BLE_DIR}/ble-mesh-model/sources/gendefaulttransitioncl/mmdl_gen_default_trans_cl_main.c
	${BLE_DIR}/ble-mesh-model/sources/gendefaulttransitionsr/mmdl_gen_default_trans_sr_main.c
	${BLE_DIR}/ble-mesh-model/sources/genlevelcl/mmdl_gen_level_cl_main.c
	${BLE_DIR}/ble-mesh-model/sources/genlevelsr/mmdl_gen_level_sr_main.c
	${BLE_DIR}/ble-mesh-model/sources/genonoffcl/mmdl_gen_onoff_cl_main.c
	${BLE_DIR}/ble-mesh-model/sources/genonoffsr/mmdl_gen_onoff_sr_main.c
	${BLE_DIR}/ble-mesh-model/sources/genpowerlevelcl/mmdl_gen_powerlevel_cl_main.c
	${BLE_DIR}/ble-mesh-model/sources/genpowerlevelsetupsr/mmdl_gen_powerlevelsetup_sr_main.c
	${BLE_DIR}/ble-mesh-model/sources/genpowerlevelsr/mmdl_gen_powerlevel_sr_main.c
	${BLE_DIR}/ble-mesh-model/sources/genpowonoffcl/mmdl_gen_powonoff_cl_main.c
	${BLE_DIR}/ble-mesh-model/sources/genpowonoffsetupsr/mmdl_gen_powonoffsetup_sr_main.c
	${BLE_DIR}/ble-mesh-model/sources/genpowonoffsr/mmdl_gen_powonoff_sr_main.c
	${BLE_DIR}/ble-mesh-model/sources/htcl/mesh_ht_cl_main.c
	${BLE_DIR}/ble-mesh-model/sources/htsr/mesh_ht_sr_main.c
	${BLE_DIR}/ble-mesh-model/sources/htsr/mesh_ht_sr_states.c
	${BLE_DIR}/ble-mesh-model/sources/lightctlcl/mmdl_light_ctl_cl_main.c
	${BLE_DIR}/ble-mesh-model/sources/lightctlsr/mmdl_light_ctl_setup_sr.c
	${BLE_DIR}/ble-mesh-model/sources/lightctlsr/mmdl_light_ctl_sr_main.c
	${BLE_DIR}/ble-mesh-model/sources/lightctlsr/mmdl_light_ctl_temp_sr_main.c
	${BLE_DIR}/ble-mesh-model/sources/lighthslcl/mmdl_light_hsl_cl_main.c
	${BLE_DIR}/ble-mesh-model/sources/lighthslhuesr/mmdl_light_hsl_hue_sr_main.c
	${BLE_DIR}/ble-mesh-model/sources/lighthslsatsr/mmdl_light_hsl_sat_sr_main.c
	${BLE_DIR}/ble-mesh-model/sources/lighthslsr/mmdl_light_hsl_setup_sr.c
	${BLE_DIR}/ble-mesh-model/sources/lighthslsr/mmdl_light_hsl_sr_main.c
	${BLE_DIR}/ble-mesh-model/sources/lightlightnesscl/mmdl_lightlightness_cl_main.c
	${BLE_DIR}/ble-mesh-model/sources/lightlightnesssetupsr/mmdl_lightlightnesssetup_sr_main.c
	${BLE_DIR}/ble-mesh-model/sources/lightlightnesssr/mmdl_lightlightness_sr_main.c
	${BLE_DIR}/ble-mesh-model/sources/scenecl/mmdl_scene_cl_main.c
	${BLE_DIR}/ble-mesh-model/sources/scenesr/mmdl_scene_setup_sr.c
	${BLE_DIR}/ble-mesh-model/sources/scenesr/mmdl_scene_sr_main.c
	${BLE_DIR}/ble-mesh-model/sources/schedulercl/mmdl_scheduler_cl_main.c
	${BLE_DIR}/ble-mesh-model/sources/schedulersr/mmdl_scheduler_setup_sr.c
	${BLE_DIR}/ble-mesh-model/sources/schedulersr/mmdl_scheduler_sr_main.c
	${BLE_DIR}/ble-mesh-model/sources/timecl/mmdl_time_cl_main.c
	${BLE_DIR}/ble-mesh-model/sources/timesr/mmdl_timesetup_sr_main.c
	${BLE_DIR}/ble-mesh-model/sources/timesr/mmdl_time_sr_main.c
	${BLE_DIR}/ble-mesh-model/sources/vendortestcl/mmdl_vendor_test_cl_main.c

	${BLE_DIR}/ble-mesh-profile/sources/bearer/adv/adv_bearer.c
	${BLE_DIR}/ble-mesh-profile/sources/bearer/gatt/gatt_bearer_cl.c
	${BLE_DIR}/ble-mesh-profile/sources/bearer/gatt/gatt_bearer_sr.c

	${BLE_DIR}/ble-mesh-profile/sources/ble-profiles/profiles/mprvc/mprvc_main.c
	${BLE_DIR}/ble-mesh-profile/sources/ble-profiles/profiles/mprvs/mprvs_main.c
	${BLE_DIR}/ble-mesh-profile/sources/ble-profiles/profiles/mprxc/mprxc_main.c
	${BLE_DIR}/ble-mesh-profile/sources/ble-profiles/profiles/mprxs/mprxs_main.c
	${BLE_DIR}/ble-mesh-profile/sources/ble-profiles/services/svc_mprvs.c
	${BLE_DIR}/ble-mesh-profile/sources/ble-profiles/services/svc_mprxs.c

	${BLE_DIR}/ble-mesh-profile/sources/stack/access/mesh_access_main.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/access/mesh_access_period_pub.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/api/mesh_api.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/api/mesh_cfg_mdl_cl_api.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/api/mesh_cfg_mdl_cl_main.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/api/mesh_cfg_mdl_main.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/api/mesh_cfg_mdl_messages.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/api/mesh_cfg_mdl_opcodes.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/api/mesh_cfg_mdl_sr_api.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/api/mesh_cfg_mdl_sr_app_key.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/api/mesh_cfg_mdl_sr_app_net_key.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/api/mesh_cfg_mdl_sr_main.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/api/mesh_cfg_mdl_sr_pub_subscr.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/bearer/mesh_adv_bearer.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/bearer/mesh_bearer.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/bearer/mesh_gatt_bearer.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/cfg/cfg_mesh_stack.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/friend/mesh_friend_act.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/friend/mesh_friend_data.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/friend/mesh_friend_main.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/friend/mesh_friend_queue.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/friend/mesh_friend_sm.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/local_config/mesh_local_config.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/lpn/mesh_lpn_act.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/lpn/mesh_lpn_main.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/lpn/mesh_lpn_sm.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/network/mesh_network_beacon.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/network/mesh_network_cache.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/network/mesh_network_if.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/network/mesh_network_main.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/network/mesh_network_mgmt.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/proxy/mesh_proxy_cl.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/proxy/mesh_proxy_main.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/proxy/mesh_proxy_sr.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/security/mesh_security_crypto_beacon.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/security/mesh_security_crypto_nwk.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/security/mesh_security_crypto_utr.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/security/mesh_security_deriv.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/security/mesh_security_main.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/security/mesh_security_toolbox.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/seq_manager/mesh_seq_manager.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/services/utils/mesh_utils.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/transports/mesh_lower_transport.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/transports/mesh_replay_protection.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/transports/mesh_sar_rx.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/transports/mesh_sar_rx_history.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/transports/mesh_sar_tx.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/transports/mesh_upper_transport.c
	${BLE_DIR}/ble-mesh-profile/sources/stack/transports/mesh_upper_transport_heartbeat.c

	${BLE_DIR}/ble-mesh-profile/sources/provisioning/mesh_prv_beacon.c
	${BLE_DIR}/ble-mesh-profile/sources/provisioning/mesh_prv_br_main.c
	${BLE_DIR}/ble-mesh-profile/sources/provisioning/mesh_prv_cl_act.c
	${BLE_DIR}/ble-mesh-profile/sources/provisioning/mesh_prv_cl_main.c
	${BLE_DIR}/ble-mesh-profile/sources/provisioning/mesh_prv_cl_sm.c
	${BLE_DIR}/ble-mesh-profile/sources/provisioning/mesh_prv_common.c
	${BLE_DIR}/ble-mesh-profile/sources/provisioning/mesh_prv_sr_act.c
	${BLE_DIR}/ble-mesh-profile/sources/provisioning/mesh_prv_sr_main.c
	${BLE_DIR}/ble-mesh-profile/sources/provisioning/mesh_prv_sr_sm.c

	${BLE_DIR}/ble-mesh-apps/sources/common/app_bearer.c
	${BLE_DIR}/ble-mesh-apps/sources/common/app_mesh_main.c
	${BLE_DIR}/ble-mesh-apps/sources/common/app_proxy.c
)

install(
	TARGETS ble
	DESTINATION ${PROJECT_SOURCE_DIR}/${TARGET_DIR}/comms/ble/lib
)

set(BLE_LIBS
	$<TARGET_FILE:ble>
	CACHE PATH "BLE libs"
)

else(BUILD_BLE)

set(BLE_LIBS
	${PROJECT_SOURCE_DIR}/${TARGET_DIR}/comms/ble/lib/libble.a
	CACHE PATH "BLE libs"
)

endif(BUILD_BLE)