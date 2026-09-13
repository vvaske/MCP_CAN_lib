/*
  mcp_can.h
  2012 Copyright (c) Seeed Technology Inc.  All right reserved.
  2017 Copyright (c) Cory J. Fowler  All Rights Reserved.

  Author:Loovee
  Contributor: Cory J. Fowler
  2017-09-25
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-
  1301  USA
*/
#ifndef _MCP2515_H_
#define _MCP2515_H_

#include "mcp_can_dfs.h"
#define MAX_CHAR_IN_MESSAGE 8

// Calls on an instance must be serialized; the shared message state is not
// safe for concurrent use from an interrupt and the main loop.
class MCP_CAN
{
    private:
    
    INT8U   m_nExtFlg;                                                  // Identifier Type
                                                                        // Extended (29 bit) or Standard (11 bit)
    INT32U  m_nID;                                                      // CAN ID
    INT8U   m_nDlc;                                                     // Data Length Code
    INT8U   m_nDta[MAX_CHAR_IN_MESSAGE];                                // Data array
    INT8U   m_nRtr;                                                     // Remote request flag
    INT8U   m_nfilhit;                                                  // The number of the filter that matched the message
    SPIClass *mcpSPI;                                                       // The SPI-Device used
    INT8U   MCPCS;                                                      // Chip Select pin number
    INT8U   mcpMode;                                                    // Last successfully selected mode, restored after configuration.
    

/*********************************************************************************************************
 *  mcp2515 driver function 
 *********************************************************************************************************/
   // private:
   private:

    void mcp2515_reset(void);                                           // Soft Reset MCP2515

    INT8U mcp2515_readRegister(const INT8U address);                    // Read MCP2515 register
    
    void mcp2515_readRegisterS(const INT8U address,                     // Read MCP2515 successive registers
                                     INT8U values[], 
                               const INT8U n);

    void mcp2515_setRegister(const INT8U address,                       // Set MCP2515 register
                             const INT8U value);

    void mcp2515_setRegisterS(const INT8U address,                      // Set MCP2515 successive registers
                              const INT8U values[],
                              const INT8U n);

    void mcp2515_initCANBuffers(void);

    void mcp2515_modifyRegister(const INT8U address,                    // Set specific bit(s) of a register
                                const INT8U mask,
                                const INT8U data);

    INT8U mcp2515_readStatus(void);                                     // Read MCP2515 Status
    INT8U mcp2515_setCANCTRL_Mode(const INT8U newmode);                 // Set mode
    INT8U mcp2515_requestNewMode(const INT8U newmode);                  // Set mode
    INT8U mcp2515_configRate(const INT8U canSpeed,                      // Set baudrate

                             const INT8U canClock);
                             
    INT8U mcp2515_init(const INT8U canIDMode,                           // Initialize Controller
                       const INT8U canSpeed,
                       const INT8U canClock);

    void mcp2515_write_mf( const INT8U mcp_addr,                        // Write CAN Mask or Filter
                           const INT8U ext,
                           const INT32U id );

    void mcp2515_write_id( const INT8U mcp_addr,                        // Write CAN ID
                           const INT8U ext,
                           const INT32U id );

    void mcp2515_read_id( const INT8U mcp_addr,                         // Read CAN ID
      INT8U* ext,
                                INT32U* id );

    void mcp2515_write_canMsg( const INT8U buffer_sidh_addr );          // Write CAN message
    void mcp2515_read_canMsg( const INT8U buffer_sidh_addr);            // Read CAN message
    INT8U mcp2515_getNextFreeTXBuf(INT8U *txbuf_n);                     // Find empty transmit buffer

/*********************************************************************************************************
 *  CAN operator function
 *********************************************************************************************************/

    INT8U setMsg(INT32U id, INT8U rtr, INT8U ext, INT8U len, INT8U *pData);        // Set message
    INT8U clearMsg();                                                   // Clear all message to zero
    INT8U readMsg();                                                    // Read message
    INT8U sendMsg();                                                    // Send message

public:
    MCP_CAN(INT8U _CS);
    MCP_CAN(SPIClass *_SPI, INT8U _CS);
    // Leaves the controller in LOOPBACK on success. Unsupported clock/rate
    // combinations (including 8 MHz / 1 Mbit/s) return CAN_FAILINIT.
    INT8U begin(INT8U idmodeset, INT8U speedset, INT8U clockset);       // Initialize controller parameters
    // Standard masks/filters use (uint32_t(SID) << 16) | (uint32_t(data0) << 8) | data1,
    // unlike sendMsgBuf() IDs. Example: SID 0x123 -> 0x01230000UL.
    // The low 16 bits support filtering the first two standard data bytes.
    // Extended masks/filters use a 29-bit value. In overloads without ext,
    // bit 31 selects extended format. Mask indices: 0..1; filters: 0..5.
    INT8U init_Mask(INT8U num, INT8U ext, INT32U ulData);               // Initialize Mask(s)
    INT8U init_Mask(INT8U num, INT32U ulData);                          // Initialize Mask(s)
    INT8U init_Filt(INT8U num, INT8U ext, INT32U ulData);               // Initialize Filter(s)
    INT8U init_Filt(INT8U num, INT32U ulData);                          // Initialize Filter(s)
    void setSleepWakeup(INT8U enable);                                  // Enable or disable the wake up interrupt (If disabled the MCP2515 will not be woken up by CAN bus activity)
    // Accepts MCP_NORMAL, MCP_SLEEP, MCP_LOOPBACK, MCP_LISTENONLY, MODE_CONFIG.
    // Invalid values return MCP2515_FAIL before SPI access; the cached mode
    // changes only on success.
    // A transition can wait up to 200 ms; waking may require two transitions.
    INT8U setMode(INT8U opMode);                                        // Set operational mode
    // len must be 0..8; buf may be null only for len == 0. Use an 11-bit
    // standard or 29-bit extended ID and ext=0/1. The overload without ext
    // uses ID bit 31 for extended format and bit 30 for a remote request.
    // These calls block until completion or a deadline. Adjust TX/abort
    // timeouts for slow bitrates (see mcp_can_dfs.h).
    // CAN_OK requires TXnIF success, not an application-level acknowledgement.
    // Timeout statuses guarantee no pending TX requests remain, but a frame
    // may have finished during abort. CAN_CTRLERROR requires explicit
    // abortTX()/controller recovery before another send attempt.
    INT8U sendMsgBuf(INT32U id, INT8U ext, INT8U len, INT8U *buf);      // Send message to transmit buffer
    INT8U sendMsgBuf(INT32U id, INT8U len, INT8U *buf);                 // Send message to transmit buffer
    // Output pointers must be valid and buf must hold 8 bytes. Read only *len
    // bytes after CAN_OK; CAN_NOMSG leaves outputs unchanged. RX DLC is capped
    // at 8. The ext overload returns a raw ID and does not expose RTR.
    // The other overload returns extended/RTR in ID bits 31/30 respectively;
    // remote frames contain no payload regardless of their requested DLC.
    INT8U readMsgBuf(INT32U *id, INT8U *ext, INT8U *len, INT8U *buf);   // Read message from receive buffer
    INT8U readMsgBuf(INT32U *id, INT8U *len, INT8U *buf);               // Read message from receive buffer
    INT8U checkReceive(void);                                           // Check for received data
    // checkError() reports EFLG & 0xF8. Use getError() to inspect all EFLG
    // bits, including EWARN/RXWAR/TXWAR warnings.
    INT8U checkError(void);                                             // Check for errors
    INT8U getError(void);                                               // Check for errors
    // Clears only RX0OVR/RX1OVR; does not flush RX buffers or reset the controller.
    void resetOverflowErrors(void);                                     // Reset overflow error bits
    INT8U errorCountRX(void);                                           // Get error count
    INT8U errorCountTX(void);                                           // Get error count
    INT8U enOneShotTX(void);                                            // Enable one-shot transmission
    INT8U disOneShotTX(void);                                           // Disable one-shot transmission
    INT8U abortTX(void);                                                // CAN_OK only after all TXREQ clear and ABAT cleared
    INT8U setGPO(INT8U data);                                           // Sets GPO
    INT8U getGPI(void);                                                 // Reads GPI
};

#endif
/*********************************************************************************************************
 *  END FILE
 *********************************************************************************************************/
