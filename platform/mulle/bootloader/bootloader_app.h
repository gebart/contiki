/**
 * \file
 *         Bootloader application image header.
 * \author
 *         Henrik Mäkitaavola <henrik@makitaavola.se>
 *
 *         Copyright (c) 2013, Eislab, Luleå University of Technology
 */

#ifndef BOOTLOADER_APP_H_
#define BOOTLOADER_APP_H_

/**
 * Returns the total size of the application image.
 * @param buf Buffer containing the first 64 bytes of the application image.
 * @return Size of the image including application and checksums.
 */
int32_t bl_app_size(const uint8_t* buf);

/**
 * Returns size of application image header.
 */
uint32_t bl_app_header_size(void);

/**
 * Returns the maximum size of a block. A block contains x bytes application
 * + 2 bytes crc
 */
uint32_t bl_app_max_block_size(void);

/**
 * Validates a block in the application image.
 * @param buf This contains len-2 bytes of the application and 2 (last bytes in
 *            buf) crc.
 * @param len The length of the block.
 * @return 1 if block is ok else 0.
 */
int bl_app_validate_block(const uint8_t* buf, uint32_t len);

uint32_t bl_app_data(uint32_t offset, const uint8_t* data, uint32_t len);
int bl_app_is_empty (void);

//Making this a define allows platforms to skip inclusion of bootloader_app.c
#define bl_app_set_user_verification(code) bl_arch_write_bl_mem(8, code, 8)
#endif /* BOOTLOADER_APP_H_ */
