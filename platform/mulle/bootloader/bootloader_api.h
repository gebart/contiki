/**
 * \file
 *         Header file for bootloader routines intended to be used by user
 *         application.
 * \author
 *         Henrik Mäkitaavola <henrik@makitaavola.se>
 *
 *         Copyright (c) 2013, Eislab, Luleå University of Technology
 */
#ifndef BOOTLOADER_API_H_
#define BOOTLOADER_API_H_

#include "bootloader-arch.h"

/**
 * Executes the bootloader. On most platform this function will probably not
 * return.
 */
void bootloader_run(void);

// TODO(henrik) Varification has never been used so it is not implemented in
//              bootloader.c. All verification codes are writen when a new
//              application has been successfully uploaded in bootloader_app.c.
/**
 * Checks whether the bootloader is verifying the user application.
 *
 * @returns 1 if verification process is on.
 */
int bootloader_is_verifying(void);

/**
 * Writes a verification code to be validated by the bootloader.
 * @param code The 8 byte verification code.
 */
int bootloader_write_user_verification(uint8_t code[8]);
#endif /* BOOTLOADER_API_H_ */
