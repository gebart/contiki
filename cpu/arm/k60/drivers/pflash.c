/*
 * Copyright (c) 2013, Lintech Embedded AB
 *
 * File:        pflash.c
 * Purpose:     Provide routines to program the flash memory
 *
 * Notes:       This file has been provided by Lintech Embedded AB.
 *
 * Author: Henrik Mäkitaavola (henrik@makitaavola.se)
 *
 */

#include "K60.h"
#include "pflash.h"
#include "interrupt.h"

enum
{
  CMD_READ_1S_SECTION = 0x01,
  CMD_PROGRAM_CHECK = 0x02,
  CMD_READ_RESOURCE = 0x03,
  CMD_PROGRAM_LONGWORD = 0x06,
  CMD_ERASE_SECTOR = 0x09,
  CMD_READ_1S_ALL_BLOCKS = 0x40,
  CMD_READ_ONCE = 0x41,
  CMD_PROGRAM_ONCE = 0x43,
  CMD_VERIFY_BACKDOOR_ACCESS_KEY = 0x45
};

#define EXECUTE() FTFL->FSTAT = 0x70; FTFL->FSTAT = 0x80 // execute command
#define FINISH() while (!(FTFL->FSTAT & FTFL_FSTAT_CCIF_MASK)) // Wait for command to execute

PFLASH_STATUS pflash_read_once(uint8_t offset, uint8_t* buf, uint8_t len)
{
  uint8_t i;
  if (len % 4 != 0 || offset % 4 != 0 || offset > 64 || len + offset > 64)
  {
    // len and offset must be a multiple of 4
    // IFR is 64 bytes so offset cant be larger than that and we cant read
    // past that boundary
    return PFLASH_INVAL;
  }
  FINISH();
  for (i = 0; i < (len / 4); ++i)
  {

    FTFL->FCCOB0 = CMD_READ_ONCE;
    FTFL->FCCOB1 = (offset / 4)+i;
    MK60_ENTER_CRITICAL_REGION();
    EXECUTE();
    FINISH();
    MK60_LEAVE_CRITICAL_REGION();
    buf[4*i] = FTFL->FCCOB4;
    buf[4*i + 1] = FTFL->FCCOB5;
    buf[4*i + 2] = FTFL->FCCOB6;
    buf[4*i + 3] = FTFL->FCCOB7;
    if (FTFL->FSTAT & 0x10)
    {
      return PFLASH_PROTECT_VIOLATION;
    }
    if (FTFL->FSTAT != 0x80)
    {
      return PFLASH_ERROR;
    }
  }

  return (FTFL->FSTAT == 0x80 ? PFLASH_OK : PFLASH_ERROR);
}

PFLASH_STATUS pflash_program_once(uint8_t offset, uint8_t* buf, uint8_t len)
{
  uint8_t i;
  if (len % 4 != 0 || offset % 4 != 0 || offset > 64 || len + offset > 64)
  {
    // len and offset must be a multiple of 4
    // IFR is 64 bytes so offset cant be larger than that and we cant write
    // past that boundary
    return PFLASH_INVAL;
  }

  FINISH();
  for (i = 0; i < (len / 4); ++i)
  {
    FTFL->FCCOB0 = CMD_PROGRAM_ONCE;
    FTFL->FCCOB1 = (offset / 4)+i;
    FTFL->FCCOB4 = buf[4*i];
    FTFL->FCCOB5 = buf[4*i + 1];
    FTFL->FCCOB6 = buf[4*i + 2];
    FTFL->FCCOB7 = buf[4*i + 3];
    MK60_ENTER_CRITICAL_REGION();
    EXECUTE();
    FINISH();
    MK60_LEAVE_CRITICAL_REGION();
    if (FTFL->FSTAT & 0x10)
    {
      return PFLASH_PROTECT_VIOLATION;
    }
    if (FTFL->FSTAT != 0x80)
    {
      return PFLASH_ERROR;
    }
  }

  return (FTFL->FSTAT == 0x80 ? PFLASH_OK : PFLASH_ERROR);
}

PFLASH_STATUS pflash_read(uint32_t addr, uint32_t* buf, uint32_t len)
{
  uint32_t i;
  uint32_t* raddr;
  FINISH();
  raddr = (uint32_t*) addr;
  for (i = 0; i < len; ++i)
  {
    buf[i] = *(raddr + i);
  }
  return PFLASH_OK;
}

PFLASH_STATUS pflash_write(uint32_t addr, uint32_t* buf, uint32_t len)
{
  uint32_t i;
  if ((addr % 4) != 0)
  {
    // Address not longword aligned.
    return PFLASH_INVAL;
  }

  // Dont write past flash end
  if (addr >= PFLASH_SECTOR_SIZE*PFLASH_NUM_SECTORS)
  {
    return PFLASH_INVAL;
  }
  FINISH();
  for (i = 0; i < len; ++i)
  {
    uint32_t waddr = addr + i*4;
    FTFL->FCCOB0 = CMD_PROGRAM_LONGWORD;
    FTFL->FCCOB1 = (waddr >> 16) & 0xff;
    FTFL->FCCOB2 = (waddr >> 8) & 0xff;
    FTFL->FCCOB3 = (waddr >> 0) & 0xff;
    FTFL->FCCOB4 = (buf[i] >> 24) & 0xff;
    FTFL->FCCOB5 = (buf[i] >> 16) & 0xff;
    FTFL->FCCOB6 = (buf[i] >> 8) & 0xff;
    FTFL->FCCOB7 = (buf[i] >> 0) & 0xff;
    MK60_ENTER_CRITICAL_REGION();
    EXECUTE();
    FINISH();
    MK60_LEAVE_CRITICAL_REGION();
    if (FTFL->FSTAT & 0x10)
    {
      return PFLASH_PROTECT_VIOLATION;
    }
    if (FTFL->FSTAT != 0x80)
    {
      return PFLASH_ERROR;
    }
  }

  return (FTFL->FSTAT == 0x80 ? PFLASH_OK : PFLASH_ERROR);
}

PFLASH_STATUS pflash_erase(uint32_t sector)
{
  uint32_t eaddr;
  if (sector > (PFLASH_NUM_SECTORS-1))
  {
    // sector must be 0 <= sector <= (NUM_SECTORS-1)
    return PFLASH_INVAL;
  }
  eaddr = PFLASH_SECTOR_SIZE*sector;
  FTFL->FCCOB0 = CMD_ERASE_SECTOR;
  FTFL->FCCOB1 = (eaddr >> 16) & 0xff;
  FTFL->FCCOB2 = (eaddr >> 8) & 0xff;
  FTFL->FCCOB3 = (eaddr >> 0) & 0xff;
  MK60_ENTER_CRITICAL_REGION();
  EXECUTE();
  FINISH();
  MK60_LEAVE_CRITICAL_REGION();
  if (FTFL->FSTAT & 0x10)
  {
    return PFLASH_PROTECT_VIOLATION;
  }
  return (FTFL->FSTAT == 0x80 ? PFLASH_OK : PFLASH_ERROR);
}
