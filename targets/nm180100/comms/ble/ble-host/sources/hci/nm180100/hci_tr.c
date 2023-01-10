/*************************************************************************************************/
/*!
 *  \file   hci_tr.c
 *
 *  \brief  HCI transport module.
 *
 *          $Date: 2017-03-10 14:08:37 -0600 (Fri, 10 Mar 2017) $
 *          $Revision: 11501 $
 *
 *  Copyright (c) 2011-2017 ARM Ltd., all rights reserved.
 *  ARM Ltd. confidential and proprietary.
 *
 *  IMPORTANT.  Your use of this file is governed by a Software License Agreement
 *  ("Agreement") that must be accepted in order to download or otherwise receive a
 *  copy of this file.  You may not use or copy this file for any purpose other than
 *  as described in the Agreement.  If you do not agree to all of the terms of the
 *  Agreement do not use this file and delete all copies in your possession or control;
 *  if you do not have a copy of the Agreement, you must contact ARM Ltd. prior
 *  to any use, copying or further distribution of this software.
 */
/*************************************************************************************************/

#include "wsf_types.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "wsf_assert.h"
#include "bstream.h"
#include "hci_api.h"
#include "hci_core.h"
#include "hci_drv.h"

/*************************************************************************************************/
/*!
 *  \fn     hciTrSendAclData
 *        
 *  \brief  Send a complete HCI ACL packet to the transport. 
 *
 *  \param  pContext Connection context.
 *  \param  pData    WSF msg buffer containing an ACL packet.
 *
 *  \return The length of ACL packet.
 */
/*************************************************************************************************/
uint16_t hciTrSendAclData(void *pContext, uint8_t *pData)
{
  uint16_t   len;

  /* get 16-bit length */
  BYTES_TO_UINT16(len, &pData[2]);
  len += HCI_ACL_HDR_LEN;

  /* transmit ACL header and data */
  if (hciDrvWrite(HCI_ACL_TYPE, len, pData) == len)
  {
      /* dump event for protocol analysis */
      HCI_PDUMP_TX_ACL(len, pData);
      return len;
  }
  else
  {
      return 0;
  }
}

/*************************************************************************************************/
/*!
 *  \fn     hciTrSendCmd
 *
 *  \brief  Send a complete HCI command to the transport.
 *
 *  \param  pData    WSF msg buffer containing an HCI command.
 *
 *  \return TRUE if packet sent, FALSE otherwise.
 */
/*************************************************************************************************/
bool_t hciTrSendCmd(uint8_t *pData)
{
  uint16_t   len;  // in case like LE set periodic advertising data, the maximum HCI command parameter length is 255

  /* get length */
  len = pData[2] + HCI_CMD_HDR_LEN;

  /* transmit ACL header and data */
  if (hciDrvWrite(HCI_CMD_TYPE, len, pData) == len)
  {
      /* dump event for protocol analysis */
      HCI_PDUMP_CMD(len, pData);
      return TRUE;
  }
  return FALSE;
}