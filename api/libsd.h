#ifndef LIBSD_H_
#define LIBSD_H_

#include "libc/types.h"

#define SD_BUSY 2
#define SD_SUCCESS 1
#define SD_ERROR 0

/* libsd public API */
uint8_t sd_early_init(void);

uint32_t sd_init(void);

/**
 * sd_read - read data from the SD card
 * @buffer: Address of the buffer in which read data will be put. The size of
 * the buffer must be at least @len.
 * @addr: Address of the first block to read in the SD card.
 * @len: Number of words (32 bits) to read.
 * Return: This function returns 0 if success.
 */
int8_t  sd_read(uint32_t * buffer, uint32_t addr, uint32_t len);

/**
 * sd_write - write data to the SD card
 * @buffer: Address of the buffer from which data will be get. The size of the
 * buffer must be at least @len.
 * @addr: Address of the first block to write in the SD card.
 * @len: Number of words (32 bits) to write.
 * Return: This function returns 0 if success.
 */
int8_t  sd_write(uint32_t * buffer, uint32_t addr, uint32_t len);

uint32_t sd_get_capacity(void);

uint32_t sd_get_block_size(void);

uint32_t sd_get_block_number(void);

uint32_t sd_get_capacity(void);

uint8_t sd_get_cid(uint32_t **cid_p, uint32_t *cid_len);

extern volatile uint8_t sd_error;
extern uint32_t saver1;
extern uint32_t savestatus;
#endif /*!LIBSD_H_ */
