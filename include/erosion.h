#ifndef _H_EROSION_H_
#define _H_EROSION_H_

#include <linux/ioctl.h>

#define EROSION_DEV_NAME	"erosion"
#define EROSION_DEV_PATH	"/dev/"EROSION_DEV_NAME
#define EROSION_DEV_MAJOR	65
#define EROSION_IRQ		87
#define EROSION_ADDRESS	0x68E00000
#define EROSION_ADDR_SIZE	0x10000

#define FRAME_SIZE			0x200000
#define FRAME_OFFSET		FRAME_SIZE
#define FRAME_1_ADDRESS		0x1FC00000
#define FRAME_2_ADDRESS 	FRAME_1_ADDRESS + FRAME_SIZE

//Register address offsets
#define EROSION_ADDR_OFFSET_CTRL		0x00
#define EROSION_ADDR_OFFSET_GIE		0x04   	
#define EROSION_ADDR_OFFSET_IER		0x08   	
#define EROSION_ADDR_OFFSET_ISR		0x0c   	
#define EROSION_ADDR_OFFSET_ROWS		0x14
#define EROSION_ADDR_OFFSET_COLS		0x1C

//Register info for userspace accesses
#define EROSION_CTL_REG				EROSION_ADDR_OFFSET_CTRL
#define EROSION_ROWS_REG			EROSION_ADDR_OFFSET_ROWS
#define EROSION_COLS_REG			EROSION_ADDR_OFFSET_COLS

//IOCTL commands
#define EROSION_SELECT_REG	_IOW(EROSION_DEV_MAJOR, 0, int)
#define EROSION_ACCEL_START	_IO(EROSION_DEV_MAJOR, 1)

//CONTROL MASKS
#define EROSION_CTRL_START_BIT		1<<0
#define EROSION_CTRL_DONE_BIT		1<<1
#define EROSION_CTRL_IDLE_BIT		1<<2
#define EROSION_CTRL_READY_BIT		1<<3
#define EROSION_CTRL_ARESTART_BIT	1<<7

//Global interrupt enable masks
#define EROSION_IER_GLOBAL_BIT		1<<0

//IP Interrupt enable register masks
#define EROSION_IER_DONE_BIT	1<<0
#define EROSION IER_READY_BIT	1<<1

//IP Interrupt status register masks
#define EROSION_ISR_DONE_BIT	1<<0
#define EROSION_ISR_READY_BIT	1<<1

// VVVVVVV For reference VVVVVVV
// ==============================================================
// File generated by Vivado(TM) HLS - High-Level Synthesis from C, C++ and SystemC
// Version: 2012.4
// Copyright (C) 2012 Xilinx Inc. All rights reserved.
// 
// ==============================================================

// 0x00 : Control signals
//        bit 0  - ap_start (Read/Write/COH)
//        bit 1  - ap_done (Read/COR)
//        bit 2  - ap_idle (Read)
//        bit 3  - ap_ready (Read)
//        bit 7  - auto_restart (Read/Write)
//        others - reserved
// 0x04 : Global Interrupt Enable Register
//        bit 0  - Global Interrupt Enable (Read/Write)
//        others - reserved
// 0x08 : IP Interrupt Enable Register (Read/Write)
//        bit 0  - Channel 0 (ap_done)
//        bit 1  - Channel 1 (ap_ready)
//        others - reserved
// 0x0c : IP Interrupt Status Register (Read/TOW)
//        bit 0  - Channel 0 (ap_done)
//        bit 1  - Channel 1 (ap_ready)
//        others - reserved
// 0x10 : reserved
// 0x14 : Data signal of rows
//        bit 31~0 - rows[31:0] (Read/Write)
// 0x18 : reserved
// 0x1c : Data signal of cols
//        bit 31~0 - cols[31:0] (Read/Write)
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

//#define XERODE_CONTROL_BUS_ADDR_AP_CTRL   0x00
//#define XERODE_CONTROL_BUS_ADDR_GIE       0x04
//#define XERODE_CONTROL_BUS_ADDR_IER       0x08
//#define XERODE_CONTROL_BUS_ADDR_ISR       0x0c
//#define XERODE_CONTROL_BUS_ADDR_ROWS_DATA 0x14
//#define XERODE_CONTROL_BUS_BITS_ROWS_DATA 32
//#define XERODE_CONTROL_BUS_ADDR_COLS_DATA 0x1c
//#define XERODE_CONTROL_BUS_BITS_COLS_DATA 32
#endif
