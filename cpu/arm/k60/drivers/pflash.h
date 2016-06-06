#ifndef PFLASH_H_
#define PFLASH_H_

typedef enum
{
  PFLASH_OK = 0,
  PFLASH_INVAL = 1,
  PFLASH_PROTECT_VIOLATION = 2,
  PFLASH_ERROR,
} PFLASH_STATUS;

// TODO(henrik) There are multiple versions with different flash size. Move defines to platform code.
#define PFLASH_SECTOR_SIZE 2048 // Num sectors 524288/2048 = 256
#define PFLASH_NUM_SECTORS 524288/2048

PFLASH_STATUS pflash_read_once(uint8_t offset, uint8_t* buf, uint8_t len);
PFLASH_STATUS pflash_program_once(uint8_t offset, uint8_t* buf, uint8_t len);
PFLASH_STATUS pflash_read(uint32_t addr, uint32_t* buf, uint32_t len);
PFLASH_STATUS pflash_write(uint32_t addr, uint32_t* buf, uint32_t len);
PFLASH_STATUS pflash_erase(uint32_t sector);

#endif /* PFLASH_H_ */
