/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2023-2024, Northern Mechatronics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _LRM_API_H_
#define _LRM_API_H_

#include <stdarg.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Represents an opaque type for the LoRa mesh context structure.
 */
typedef struct {} lrm_context_t;

/**
 * Represents error codes used throughout the LoRa mesh library.
 */
typedef enum
{
    /**
     * No error.
     */
    LRM_ERROR_NONE = 0,

    /**
     * Operation failed.
     */
    LRM_ERROR_FAILED = 1,

    /**
     * Insufficient buffers.
     */
    LRM_ERROR_NO_BUFS = 3,

    /**
     * Failed to parse message.
     */
    LRM_ERROR_PARSE = 6,

    /**
     * Input arguments are invalid.
     */
    LRM_ERROR_INVALID_ARGS = 7,

    /**
     * Cannot complete due to invalid state.
     */
    LRM_ERROR_INVALID_STATE = 13,

} lrm_error_e;

typedef struct
{
    char *networkName;
    uint16_t panId;
    const uint8_t *extendedPanId;
    const uint8_t *meshLocalPrefix;
    uint16_t channel;
    const uint8_t *networkKey;
    const uint8_t *pskc;
} lrm_network_config_t;

/**
 * Represents a device role.
 */
typedef enum
{
    LRM_DEVICE_ROLE_DISABLED = 0,   // The LoRa mesh network is disabled.
    LRM_DEVICE_ROLE_DETACHED = 1,   // Not currently attached to a LoRa mesh network.
    LRM_DEVICE_ROLE_CHILD    = 2,   // The Child role.
    LRM_DEVICE_ROLE_ROUTER   = 3,   // The Router role.
    LRM_DEVICE_ROLE_LEADER   = 4,   // The Leader role.
} lrm_device_role_e;

/**
 * Pointer is called to notify of device role changes.
 *
 * @param[out]  context         A pointer to a LoRa mesh context.
 * @param[out]  role            The new device role.
 */
typedef void (*lrm_role_changed_callback)(lrm_context_t *context, lrm_device_role_e role);

#define LRM_RADIO_RSSI_INVALID 127      // Invalid or unknown RSSI value

#define LRM_IP6_ADDRESS_STRING_SIZE 40  // Recommended size for string representation of an IPv6 address.

/**
 * Represents an opaque type for an IPv6 address.
 */
typedef struct {} lrm_ip6_address;

/**
 * Represents an opaque (and empty) type for a LoRa mesh message buffer.
 */
typedef struct {} lrm_message;

/**
 * Represents an opaque type for the local and peer IPv6 socket addresses.
 */
typedef struct {} lrm_message_info;

/**
 * Represents an opaque type for a UDP socket.
 */
typedef struct {uint8_t _reserved[64];} lrm_udp_socket;

/**
 * This callback allows LoRa mesh to inform the application of a received UDP message.
 */
typedef void (*lrm_udp_receive_callback)(void *context, lrm_message *message, const lrm_message_info *message_info);

typedef int (*lrm_cli_output_callback)(void *context, const char *fmt, va_list args);

/**
 * Initializes the static single instance of the LoRa mesh library and prepares it for subsequent API calls.
 * This function must be called before any other calls to LoRa mesh.
 *
 * @returns A pointer to the single LoRa mesh context.
 */
lrm_context_t *lrm_init(void);

/**
 * Disables the LoRa mesh library.
 *
 * Call this function when LoRa mesh is no longer in use.
 *
 * @param[in]  context          A pointer to a LoRa mesh context.
 */
void lrm_finalize(lrm_context_t *context);

/**
 * Registers a callback to indicate when the LoRa mesh device role changes.
 *
 * @param[in]  context          A pointer to a LoRa mesh context.
 * @param[in]  callback         A pointer to the application callback function.
 *
 * @retval LRM_ERROR_NONE     Added the callback.
 */
lrm_error_e lrm_set_role_changed_callback(lrm_context_t *context, lrm_role_changed_callback callback);

/**
 * Indicates whether a LoRa mesh network is configured or not.
 *
 * @param[in]  context          A pointer to a LoRa mesh context.
 *
 * @returns TRUE if a LoRa mesh network is configured, FALSE otherwise.
 *
 */
bool lrm_is_configured(lrm_context_t *context);

/**
 * Configures the LoRa mesh network.
 *
 * @param[in]  context          A pointer to a LoRa mesh context.
 * @param[in]  config           A pointer to a LoRa mesh configuration.
 *
 * @retval LRM_ERROR_NONE               Successfully configured the network.
 * @retval LRM_ERROR_NO_BUFS            Insufficient buffer space to set the new configuration.
 * @retval LRM_ERROR_FAILED             Failed to generate random values for the new configuration.
 */
lrm_error_e lrm_network_configure(lrm_context_t *context, lrm_network_config_t *config);

/**
 * Deletes all the settings stored on non-volatile memory, and then triggers a platform reset.
 *
 * @param[in]  context          A pointer to a LoRa mesh context.
 */
void lrm_factory_reset(lrm_context_t *context);

/**
 * The LoRa mesh provisioning credentials.
 */
extern char *lrm_prv_x509_cert;
extern char *lrm_prv_priv_key;
extern char *lrm_prv_trusted_root_cert;

/**
 * Start the LoRa mesh network.
 *
 * @param[in]  context          A pointer to a LoRa mesh context.
 *
 * @retval LRM_ERROR_NONE           Successfully started the network.
 * @retval LRM_ERROR_INVALID_STATE  IPv6 interface is not available.
 */
lrm_error_e lrm_network_start(lrm_context_t *context);

/**
 * Stop the LoRa mesh network.
 *
 * @param[in]  context          A pointer to a LoRa mesh context.
 *
 * @retval LRM_ERROR_NONE           Successfully stopped the network.
 * @retval LRM_ERROR_INVALID_STATE  The network interface was not not up.
 */
lrm_error_e lrm_network_stop(lrm_context_t *context);

/**
 * Indicates whether a LoRa mesh network is started or not.
 *
 * @param[in]  context          A pointer to a LoRa mesh context.
 *
 * @returns TRUE if a LoRa mesh network is start, FALSE otherwise.
 *
 */
bool lrm_is_network_started(lrm_context_t *context);

/**
 * Get the device role.
 *
 * @param[in]  context          A pointer to a LoRa mesh context.
 *
 * @retval LRM_DEVICE_ROLE_DISABLED The LoRa mesh network is disabled.
 * @retval LRM_DEVICE_ROLE_DETACHED The device is not currently attached to a LoRa mesh network.
 * @retval LRM_DEVICE_ROLE_CHILD    The device is currently operating as a LoRa mesh Child.
 * @retval LRM_DEVICE_ROLE_ROUTER   The device is currently operating as a LoRa mesh Router.
 * @retval LRM_DEVICE_ROLE_LEADER   The device is currently operating as a LoRa mesh Leader.
 */
lrm_device_role_e lrm_get_device_role(lrm_context_t *context);

/**
 * Open a LoRa mesh UDP socket.
 *
 * @param[in]  context          A pointer to a LoRa mesh context.
 * @param[in]  socket           A pointer to a UDP socket structure.
 * @param[in]  callback         A pointer to the application callback function.
 * @param[in]  callback_context A pointer to application-specific context.
 *
 * @retval LRM_ERROR_NONE   Successfully opened the socket.
 * @retval LRM_ERROR_FAILED Failed to open the socket.
 */
lrm_error_e lrm_udp_open(lrm_context_t *context, lrm_udp_socket *socket, lrm_udp_receive_callback callback, void *callback_context);

/**
 * Close a LoRa mesh UDP socket.
 *
 * @param[in]  context          A pointer to a LoRa mesh context.
 * @param[in]  socket           A pointer to a UDP socket structure.
 *
 * @retval LRM_ERROR_NONE   Successfully closed the socket.
 * @retval LRM_ERROR_FAILED Failed to close the Socket.
 */
lrm_error_e lrm_udp_close(lrm_context_t *context, lrm_udp_socket *socket);

/**
 * Bind a LoRa mesh UDP socket.
 *
 * @param[in]  context          A pointer to a LoRa mesh context.
 * @param[in]  socket           A pointer to a UDP socket structure.
 * @param[in]  host             The address to bind to.
 * @param[in]  port             The port to bind to.
 *
 * @retval LRM_ERROR_NONE   Bind operation was successful.
 * @retval LRM_ERROR_PARSE  Failed to parse host.
 * @retval LRM_ERROR_FAILED Failed to bind the socket.
 */
lrm_error_e lrm_udp_bind(lrm_context_t *context, lrm_udp_socket *socket, char *host, uint16_t port);

/**
 * Connect a LoRa mesh UDP socket.
 *
 * @param[in]  context          A pointer to a LoRa mesh context.
 * @param[in]  socket           A pointer to a UDP socket structure.
 * @param[in]  host             The address to connect to.
 * @param[in]  port             The port to connect to.
 *
 * @retval LRM_ERROR_NONE   Connect operation was successful.
 * @retval LRM_ERROR_PARSE  Failed to parse host.
 * @retval LRM_ERROR_FAILED Failed to connect the socket.
 */
lrm_error_e lrm_udp_connect(lrm_context_t *context, lrm_udp_socket *socket, char *host, uint16_t port);

/**
 * Send a LoRa mesh UDP message.
 *
 * @param[in]  context          A pointer to a LoRa mesh context.
 * @param[in]  socket           A pointer to a UDP socket structure.
 * @param[in]  host             The address to send to.
 * @param[in]  port             The port to send to.
 * @param[in]  buf              The message content.
 * @param[in]  len              The length of the message content.
 *
 * @retval LRM_ERROR_NONE           The message is successfully scheduled for sending.
 * @retval LRM_ERROR_INVALID_ARGS   Invalid arguments are given.
 * @retval LRM_ERROR_PARSE          Failed to parse host.
 * @retval LRM_ERROR_NO_BUFS        Insufficient available buffer to add the UDP and IPv6 headers.
 */
lrm_error_e lrm_udp_send(lrm_context_t *context, lrm_udp_socket *socket, char *host, uint16_t port, void *buf, size_t len);

/**
 * Converts a given IPv6 address to a human-readable string.
 *
 * The IPv6 address string is formatted as 16 hex values separated by ':' (i.e., "%x:%x:%x:...:%x").
 *
 * If the resulting string does not fit in @p buffer (within its @p size characters), the string will be truncated
 * but the outputted string is always null-terminated.
 *
 * @param[in]  address  A pointer to an IPv6 address (MUST NOT be NULL).
 * @param[out] buffer   A pointer to a char array to output the string (MUST NOT be NULL).
 * @param[in]  size     The size of @p buffer (in bytes). Recommended to use `LRM_IP6_ADDRESS_STRING_SIZE`.
 */
void lrm_ip6_address_to_string(const lrm_ip6_address *address, char *buffer, uint16_t size);

/**
 * Get the peer IPv6 address.
 *
 * @param[in]  message_info A pointer to the message information.
 *
 * @returns The peer IPv6 address.
 */
const lrm_ip6_address *lrm_message_info_get_peer_addr(const lrm_message_info *message_info);

/**
 * Get the peer transport-layer port.
 *
 * @param[in]  message_info A pointer to the message information.
 *
 * @returns The peer transport-layer port.
 */
uint16_t lrm_message_info_get_peer_port(const lrm_message_info *message_info);

/**
 * Get the message length in bytes.
 *
 * @param[in]  message  A pointer to a message buffer.
 *
 * @returns The message length in bytes.
 */
uint16_t lrm_message_get_length(const lrm_message *message);

/**
 * Get the message offset in bytes.
 *
 * @param[in]  message  A pointer to a message buffer.
 *
 * @returns The message offset value.
 */
uint16_t lrm_message_get_offset(const lrm_message *message);

/**
 * Returns the average RSS (received signal strength) associated with the message.
 *
 * @param[in]  message  A pointer to a message buffer.
 *
 * @returns The average RSS value (in dBm) or LRM_RADIO_RSSI_INVALID if no average RSS is available.
 */
int8_t lrm_message_get_rss(const lrm_message *message);

/**
 * Read bytes from a message.
 *
 * @param[in]  message  A pointer to a message buffer.
 * @param[in]  offset   An offset in bytes.
 * @param[in]  buf      A pointer to a buffer that message bytes are read to.
 * @param[in]  length   Number of bytes to read.
 *
 * @returns The number of bytes read.
 */
uint16_t lrm_message_read(const lrm_message *message, uint16_t offset, void *buf, uint16_t length);

typedef enum
{
    LRM_NETWORK_CFG_ROUTER_SELECTION_JITTER,
    LRM_NETWORK_CFG_ROUTER_UPGRADE_THRESHOLD,
    LRM_NETWORK_CFG_ROUTER_DOWNGRADE_THRESHOLD,
} lrm_network_cfg_e;

lrm_error_e lrm_network_get_cfg(lrm_context_t *context, lrm_network_cfg_e cfg, int32_t *value);
lrm_error_e lrm_network_set_cfg(lrm_context_t *context, lrm_network_cfg_e cfg, int32_t value);

typedef enum
{
    LRM_RADIO_CFG_TX_POWER,
    LRM_RADIO_CFG_FREQUENCY,
    LRM_RADIO_CFG_SPREADING_FACTOR,
    LRM_RADIO_CFG_BANDWIDTH,
} lrm_radio_cfg_e;

lrm_error_e lrm_radio_get_cfg(lrm_context_t *context, lrm_radio_cfg_e cfg, int32_t *value);
lrm_error_e lrm_radio_set_cfg(lrm_context_t *context, lrm_radio_cfg_e cfg, int32_t value);

void lrm_cli_init(lrm_context_t *context, lrm_cli_output_callback callback, void *callback_context);
void lrm_cli_input_line(lrm_context_t *context, bool no_echo, char *fmt, ...);

const char *lrm_error_to_string(lrm_error_e error);

void *lrm_get_instance(lrm_context_t *context);

void lrm_lock(lrm_context_t *context);
void lrm_unlock(lrm_context_t *context);

void lrm_sys_init(int argc, char *argv[]);
bool lrm_sys_pseudo_reset_was_requested(void);
void lrm_sys_process_drivers(lrm_context_t *context);
void lrm_tasklets_process(lrm_context_t *context);
bool lrm_tasklets_are_pending(lrm_context_t *context);

void lrm_plat_ble_gap_on_connected(lrm_context_t *context, uint16_t connection_id, uint16_t tx_handle);
void lrm_plat_ble_gap_on_disconnected(lrm_context_t *context, uint16_t connection_id);
uint16_t lrm_plat_ble_gatt_mtu_get_max(void);
void lrm_plat_ble_gatt_on_mtu_update(lrm_context_t *context, uint16_t mtu);
void lrm_plat_ble_gatt_server_indicate_cnf(lrm_context_t *context, uint16_t connection_id, uint16_t tx_handle);
void lrm_plat_ble_gatt_server_on_write_request(lrm_context_t *context, uint16_t handle, uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif  /* _LRM_API_H_ */
