/*!
 * \file      smtc_hal_flash.h
 *
 * \brief     FLASH Hardware Abstraction Layer definition
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __SMTC_HAL_FLASH_H__
#define __SMTC_HAL_FLASH_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include <am_mcu_apollo.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */
#define FLASH_PAGE_SIZE     AM_HAL_FLASH_PAGE_SIZE
#define FLASH_BASE          AM_HAL_FLASH_ADDR
#define FLASH_BANK_SIZE     AM_HAL_FLASH_INSTANCE_SIZE

#define FLASH_PAGE_NUMBER   ( AM_HAL_FLASH_INSTANCE_PAGES * AM_HAL_FLASH_NUM_INSTANCES ) //Total pages
#define FLASH_PAGE_PER_BANK AM_HAL_FLASH_INSTANCE_PAGES

// #define FLASH_USER_START_ADDR AM_HAL_FLASH_APPL_ADDR /* Start @ of user Flash area */
#define FLASH_USER_START_ADDR AM_HAL_FLASH_INSTANCE_SIZE /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR \
    FLASH_USER_START_ADDR + ( FLASH_PAGE_NUMBER * ADDR_FLASH_PAGE_SIZE ) - 1 /* End @ of user Flash area */

#define ADDR_FLASH_PAGE_SIZE AM_HAL_FLASH_PAGE_SIZE 

#define FLASH_BYTE_EMPTY_CONTENT ( ( uint8_t ) 0xFF )
#define FLASH_PAGE_EMPTY_CONTENT ( ( uint64_t ) 0xFFFFFFFFFFFFFFFF )

#define FLASH_PAGE_ADDR( page ) FLASH_USER_START_ADDR + ( ( page ) *ADDR_FLASH_PAGE_SIZE )

/* Base address of the Flash s */
// #define ADDR_FLASH_PAGE_0 ( ( uint32_t ) 0x08000000 )   /* Base @ of Page 0, 2 Kbytes */
// #define ADDR_FLASH_PAGE_1 ( ( uint32_t ) 0x08000800 )   /* Base @ of Page 1, 2 Kbytes */
// #define ADDR_FLASH_PAGE_2 ( ( uint32_t ) 0x08001000 )   /* Base @ of Page 2, 2 Kbytes */
// #define ADDR_FLASH_PAGE_x ( ( uint32_t ) 0x080xxx00 )   /* Base @ of Page x, 2 Kbytes */

#define FLASH_OPERATION_MAX_RETRY 4

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief Erase a given nb page to the FLASH at the specified address.
 *
 * \param[IN] addr FLASH address to start the erase
 * \param[IN] nb_page the number of page to erase.
 * \retval status [SUCCESS, FAIL]
 */
uint8_t hal_flash_erase_page( uint32_t addr, uint8_t nb_page );

/*!
 * \brief Writes the given buffer to the FLASH at the specified address.
 *
 * \param[IN] addr FLASH address to write to
 * \param[IN] buffer Pointer to the buffer to be written.
 * \param[IN] size Size of the buffer to be written.
 * \retval status [Real_size_written, FAIL]
 */
uint32_t hal_flash_write_buffer( uint32_t addr, const uint8_t* buffer, uint32_t size );

/*!
 * \brief Reads the FLASH at the specified address to the given buffer.
 *
 * \param[IN] addr FLASH address to read from
 * \param[OUT] buffer Pointer to the buffer to be written with read data.
 * \param[IN] size Size of the buffer to be read.
 * \retval status [SUCCESS, FAIL]
 */
void hal_flash_read_buffer( uint32_t addr, uint8_t* buffer, uint32_t size );

// uint32_t flash_get_page( uint32_t Addr );

#if defined( USE_FLASH_READ_MODIFY_WRITE ) || defined( MULTISTACK )
/**
 * @brief Reads a flash page, modify it, erase page and then write it
 *
 * @param [in] addr FLASH address
 * @param [in] buffer Pointer to the buffer to be written.
 * @param [in] size Size of the buffer to be written.
 */
void hal_flash_read_modify_write( uint32_t addr, const uint8_t* buffer, uint32_t size );
#endif  // USE_FLASH_READ_MODIFY_WRITE or MULTISTACK

#ifdef __cplusplus
}
#endif

#endif  // __SMTC_HAL_FLASH_H__

/* --- EOF ------------------------------------------------------------------ */
