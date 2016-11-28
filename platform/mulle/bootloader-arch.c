/**
 * \file
 *         Implementation of arch specific functions required by the
 *         bootloader.
 * \author
 *         Henrik Mäkitaavola <henrik@makitaavola.se>
 */

#define DEBUG 0
#if DEBUG
#include <inttypes.h>
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

//#include "interrupt.h"
#include "bootloader-arch.h"
#include "pflash.h"
#include "mulle-nvram.h"
#include "process.h"

PROCESS(bootloader_flag_process, "Bootloader flag process");

#define _vector_ram_start 0x1FFF0000

static int read_flash_safetly(uint32_t addr, uint8_t* buf, uint32_t len);
static int write_flash_safetly(uint32_t addr, uint8_t* buf, uint32_t len);

int
bl_arch_flash_erase(void)
{
  int i;
  for (i = BL_MEM_FLASH_START / PFLASH_SECTOR_SIZE;
      i < PFLASH_NUM_SECTORS; ++i )
  {
    if (pflash_erase(i) != PFLASH_OK)
    {
    	return 0;
    }
  }
  return 1;
}

int
bl_arch_flash_write(unsigned long addr, unsigned char* buf, unsigned long len)
{
  // Pad the len to divide evenly with 4, we don't care about the data in the last bytes
  len = (len + (4 - 1)) & ~(4 - 1);

  return write_flash_safetly(addr, buf, len);
}

void bl_arch_run_app(void)
{
  void (*user_code_entry)(void);
  uint32_t* pdst=(uint32_t*)_vector_ram_start;
  uint32_t* psrc=(uint32_t*)USER_APP_FLASH_START;
  unsigned *p;
  int i;

  for (i=0; i < 0x100; i++, pdst++, psrc++)
  {
    *pdst = *psrc;
  }

  SCB->VTOR = (uint32_t)_vector_ram_start;

  // Load contents of second word of user flash - the reset handler address
  // in the applications vector table
  p = (unsigned *)(USER_APP_FLASH_START + 4);

  user_code_entry = (void (*)(void))(*p);

  // Jump to user application
  user_code_entry();

}

#define BOOTLOADER_FILE "bootloader"

static int
bl_arch_set_run_flag(uint16_t flag)
{
  PRINTF("bootloader: Set flag = 0x%04" PRIx16 "\n", flag);
  if (mulle_nvram == NULL) {
    PRINTF("bootloader: no nvram!\n");
    return 0;
  }
  if (mulle_nvram->write(mulle_nvram, &flag, MULLE_NVRAM_BOOT_LOADER_FLAG, sizeof(flag)) != sizeof(flag)) {
    PRINTF("bootloader: set flag write failed!\n");
    return 0;
  }
  return 1;
}

int
bl_arch_set_run_app(void)
{
  return bl_arch_set_run_flag(1);
}

int
bl_arch_set_run_bootloader(void)
{
  return bl_arch_set_run_flag(0);
}

int
bl_arch_is_run_app(void)
{
  uint16_t flag;

  if (mulle_nvram == NULL) {
    PRINTF("bootloader: no nvram!\n");
    return 0;
  }

  if (mulle_nvram->read(mulle_nvram, &flag, MULLE_NVRAM_BOOT_LOADER_FLAG, sizeof(flag)) != sizeof(flag)) {
    return 0;
  }

  return flag;
}

int bl_arch_read_bl_mem(uint32_t addr, uint8_t* buf, uint32_t len)
{
  return read_flash_safetly(BL_MEM_FLASH_START+addr, buf, len);
}

int bl_arch_write_bl_mem(uint32_t addr, uint8_t* buf, uint32_t len)
{
  return write_flash_safetly(BL_MEM_FLASH_START+addr, buf, len);
}

static int write_flash_safetly(uint32_t addr, uint8_t* buf, uint32_t len)
{
  int i;
  for (i = 0; i < len; i += 4)
  {
    int j;
    uint32_t word = 0xFFFFFFFF;
    for (j = 0; j < (len - i < 4 ? len - i : 4); ++j)
    {
      word &= ~(0xFF << j * 8);
      word |= (buf[i+j] << j * 8);
    }
    if (pflash_write(addr+i, &word, 1) != PFLASH_OK)
    {
      return 0;
    }
  }
  return 1;
}

static int read_flash_safetly(uint32_t addr, uint8_t* buf, uint32_t len)
{
  int i;
  for (i = 0; i < len; i += 4)
  {
    uint32_t word;
    int j;
    if (pflash_read(addr+i, &word, 1) != PFLASH_OK)
    {
      return 0;
    }
    for (j = 0; j < (len - i < 4 ? len - i : 4); ++j)
    {
      buf[i+j] = (word >> (j*8)) & 0xFF;
    }
  }
  return 1;
}

PROCESS_THREAD(bootloader_flag_process, ev, data)
{
  PROCESS_BEGIN();

  PRINTF("Set bootloader run_user flag\n");

  bl_arch_set_run_app();

  PROCESS_END();
}
