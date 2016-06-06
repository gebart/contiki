/**
 * \file
 *         Bootloader application image routines.
 * \author
 *         Henrik Mäkitaavola <henrik@makitaavola.se>
 *
 *         Copyright (c) 2013, Eislab, Luleå University of Technology
 */

#define DEBUG 0

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "bootloader.h"
#include "bootloader-arch.h"
#include "bootloader_api.h"

uint8_t app_buffer[514];
uint16_t app_buffer_pos;
uint32_t total_app_size;
uint32_t bytes_written;
uint32_t addr;
uint32_t app_crc32;

/*static*/ void set_non_empty(void);

int32_t bl_app_size(const uint8_t* buf)
{
  return (int32_t)buf[0] << 24 | (int32_t)buf[1] << 16 | (int32_t)buf[2] << 8 | (int32_t)buf[3];
}

uint32_t bl_app_crc32(const uint8_t* buf)
{
  return (uint32_t)buf[4] << 24 | (uint32_t)buf[5] << 16 | (uint32_t)buf[6] << 8 | (uint32_t)buf[7];
}

uint32_t bl_app_header_size(void)
{
  return 8; // Only size and 32bit crc in header
}

uint32_t bl_app_max_block_size(void)
{
  return 514;
}

static unsigned int
calc_crc(unsigned int crc, unsigned char data)
{
  unsigned char lo8 = crc & 0x00FF;
  unsigned char hi8 = (crc >> 8) & 0x00FF;
  data ^= lo8;
  data ^= data << 4;

  return ((((unsigned int)data << 8) | hi8 /*hi8 (crc)*/) ^ (unsigned char)(data >> 4)
      ^ ((unsigned int)data << 3));
}

int
bl_app_validate_block(const uint8_t* buf, uint32_t len)
{
  int i;
  uint16_t crc = 0;
  uint16_t tcrc;
  for (i = 0; i < len-2; ++i)
  {
    crc = calc_crc(crc, buf[i]);
  }
  crc = calc_crc(crc, (unsigned char) ((app_crc32 >> 24) & 0xFF));
  crc = calc_crc(crc, (unsigned char) ((app_crc32 >> 16) & 0xFF));
  crc = calc_crc(crc, (unsigned char) ((app_crc32 >> 8) & 0xFF));;
  crc = calc_crc(crc, (unsigned char) ((app_crc32 >> 0) & 0xFF));
  tcrc = (buf[len-2] << 8) | buf[len-1];
  return tcrc == crc;
}

static int write_to_flash(const uint8_t* buf, int len)
{
  if (buf == NULL)
  {
    // app_buffer_pos should never be other than 0 or larger than 2
    // because a crc (2 bytes) is never sent alone so there can never
    // be only a crc in the buffer. But check smaller than 2 so there
    // wont be any pointers to strange address space.
    if (app_buffer_pos < 2)
    {
      // There is no more data in the app_buffer, we are done.
      return 1;
    }
    // Flush last
    if (!bl_app_validate_block(app_buffer, app_buffer_pos))
    {
      return 0;
    }
    if (!bl_arch_flash_write(addr, app_buffer, app_buffer_pos-2))
    {
      return 0;
    }
  }
  else
  {
    while(len != 0)
    {
      uint16_t write;
      if (bl_app_max_block_size()-app_buffer_pos < len)
      {
        write = bl_app_max_block_size()-app_buffer_pos;
      }
      else
      {
        write = len;
      }
      memcpy(app_buffer + app_buffer_pos, buf, write);
      app_buffer_pos += write;
      len -= write;
      buf += write;
      if (app_buffer_pos == bl_app_max_block_size())
      {
        if (!bl_app_validate_block(app_buffer, app_buffer_pos))
        {
          PRINTF("Block not valid\n");
          return 0;
        }
        if (!bl_arch_flash_write(addr, app_buffer, app_buffer_pos-2))
        {
          PRINTF("Unable to write\n");
          return 0;
        }
        addr += app_buffer_pos-2;
        app_buffer_pos = 0;
      }
    }
  }
  return 1;
}

uint32_t bl_app_data(uint32_t offset, const uint8_t* data, uint32_t len)
{
  if (offset == 0)
  {
    PRINTF("New application\n");
    if (!bl_arch_flash_erase())
    {
      return 0;
    }
    app_buffer_pos = 0;
    addr = USER_APP_FLASH_START;
    total_app_size = bl_app_size(data);
    app_crc32 = bl_app_crc32(data);
    PRINTF("Application size %lu crc %X\n", total_app_size, app_crc32);
    bytes_written = 0;
    data += bl_app_header_size();
    len -= bl_app_header_size();
  }
  if (!write_to_flash(data, len))
  {
    PRINTF("Failed to write to flash\n");
    return 0;
  }
  bytes_written += len;
  if (total_app_size == bytes_written)
  {
    PRINTF("No more data\n");
    if (!write_to_flash(NULL, 0))
    {
      PRINTF("Failed to write to flash\n");
      return 0;
    }
    set_non_empty();
  }
  return len;
}

/*static*/ void set_non_empty(void)
{
  uint8_t code[8] = CODE_NEW_APP;
  if (bl_app_is_empty())
  {
    if (!bl_arch_write_bl_mem(0, code, 8))
    {
    	PRINTF("Failed to set non empty!");
    }
  }
}

int bl_app_is_empty (void)
{
  int i;
  uint8_t cna[8] = CODE_NEW_APP;
  uint8_t buf[8];
  bl_arch_read_bl_mem(0, buf, 8);
  for (i = 0; i < 8; ++i)
  {
    if (buf[i] != cna[i])
    {
      return 1;
    }
  }
  return 0;
}

int bl_app_is_user_verified(void)
{
  int i;
  uint8_t cuv[8] = CODE_USER_VERIFICATION;
  uint8_t buf[8];
  bl_arch_read_bl_mem(8, buf, 8);
  for (i = 0; i < 8; ++i)
  {
    if (buf[i] != cuv[i])
    {
      return 0;
    }
  }
  return 1;
}

int bl_app_set_bootloader_verified(void)
{
  uint8_t code[8] = CODE_BOOTLOADER_VERIFICATION;
  return bl_arch_write_bl_mem(8*2, code, 8);
}

int bl_app_is_bootloader_verified(void)
{
  int i;
  uint8_t cblv[8] = CODE_BOOTLOADER_VERIFICATION;
  uint8_t buf[8];
  bl_arch_read_bl_mem(2*8, buf, 8);
  for (i = 0; i < 8; ++i)
  {
    if (buf[i] != cblv[i])
    {
      return 0;
    }
  }
  return 1;
}
