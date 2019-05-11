#ifndef SD_H_
#define SD_H_

#include "libsdio.h"
#include "libc/types.h"

struct sdio_cmd {
    union {
        struct {
            uint8_t     index:7;
            uint8_t     is_acmd:1;
        } cmd;
        uint8_t     cmd_value;
    };
    uint32_t    arg;
    enum sdio_resp_type response;
    /* For data commands */
    uint32_t   *buf;
    uint32_t    buf_len;
};

uint32_t    sd_init(void);

/**
 * sd_get_capacity - return the capacity of the SD card
 *
 * Return: This function returns the size in bytes of the SD card. The return
 * value is undefined is the SD card is not ready (see @sd_is_ready)
 */
uint32_t    sd_get_capacity(void);

/**
 * sd_get_block_size - return the block size of the SD card
 *
 * Return: This function returns the block size in bytes of the SD card. The
 * return value is undefined is the SD card is not ready (see @sd_is_ready)
 */
uint32_t    sd_get_block_size(void);

/**
 * sd_is_ready - return true if the SD card is ready
 *
 * Return: This function return 0 if the SD card is not ready (it is still in
 * identification mode) or a value different than 0 is the SD card is ready:
 * the SD card is in transfer mode (i.e. the identification is done) but an
 * operation can be in progress.
 */
uint8_t     sd_is_ready(void);

uint32_t    sd_card_csd_structure(void);

/**
 * sd_flush_commands - Send commands in SD queue
 */
void        sd_flush_commands(void);

#define CMD_TIMEOUT		0xffffff
#define SD_CMD_QUEUE_SIZE	32
#define SD_ERROR_MAX_RETRIES	255

/* Normal commands */
#define SD_GO_TO_IDLE_STATE		0
#define SD_ALL_SEND_CID		2
#define SD_SEND_RELATIVE_ADDR		3
#define SD_SELECT_DESELECT_CARD	7
#define SD_SEND_IF_COND		8
#	define SD_SEND_IF_COND_CHECK_PATTERN	0xaa
#	define SD_SEND_IF_COND_27V_36V		(1 << 8)
#define SD_SEND_CSD			9
#define SD_STOP_TRANSMISSION		12
#define SD_SEND_STATUS			13
#define SD_SET_BLOCKLEN		16
#define SD_READ_SINGLE_BLOCK		17
#define SD_READ_MULTIPLE_BLOCK		18
#define SD_WRITE_BLOCK			24
#define SD_WRITE_MULTIPLE_BLOCK	25
#define SD_APP_CMD			55

/* Application commands */
#define ACMD(cmd)			(0x80 | (cmd))
#define SD_SET_BUS_WIDTH_idx		6
//# define SD_SET_BUS_WIDTH     ACMD(SD_SET_BUS_WIDTH_idx)
#define SD_SET_BUS_WIDTH		SD_SET_BUS_WIDTH_idx
#define SD_SEND_OP_COND_idx		41
//# define SD_SEND_OP_COND      ACMD(SD_SEND_OP_COND_idx)
#define SD_SEND_OP_COND		SD_SEND_OP_COND_idx

#define R2_OR_R3	((1 << 6) - 1)  /* ACMD41 => R3, See spec section 4.9.4 */

/* Card status in response R1 */
#define CARD_STATUS_OUT_OF_RANGE	((uint32_t)1 << 31)
#define CARD_STATUS_ADDRESS_ERROR	(1 << 30)
#define CARD_STATUS_BLOCK_LEN_ERROR	(1 << 29)
#define CARD_STATUS_ERASE_SEQ_ERROR	(1 << 28)
#define CARD_STATUS_ERASE_PARAM		(1 << 27)
#define CARD_STATUS_WP_VIOLATION	(1 << 26)
#define CARD_STATUS_CARD_IS_LOCKED	(1 << 25)
#define CARD_STATUS_LOCK_UNLOCK_FAILED	(1 << 24)
#define CARD_STATUS_COM_CRC_ERROR	(1 << 23)
#define CARD_STATUS_R6_COM_CRC_ERROR	(1 << 15)
#define CARD_STATUS_ILLEGAL_COMMAND	(1 << 22)
#define CARD_STATUS_R6_ILLEGAL_COMMAND	(1 << 14)
#define CARD_STATUS_CARD_ECC_FAILED	(1 << 21)
#define CARD_STATUS_CC_ERROR		(1 << 20)
#define CARD_STATUS_ERROR		(1 << 19)
#define CARD_STATUS_R6_ERROR		(1 << 13)
#define CARD_STATUS_CSD_OVERWRITE	(1 << 16)
#define CARD_STATUS_WP_ERASE_SKIP	(1 << 15)
#define CARD_STATUS_CARD_ECC_DISABLED	(1 << 14)
#define CARD_STATUS_ERASE_RESET	(1 << 13)
#define CARD_STATUS_CURRENT_STATE	(0xf << 9)
#define CARD_STATUS_READY_FOR_DATA	(1 << 8)
#define CARD_STATUS_APP_CMD		(1 << 5)
#define CARD_STATUS_AKE_SEQ_ERROR	(1 << 3)
#define CARD_STATUS_R6_AKE_SEQ_ERROR	(1 << 3)

#define CARD_STATUS_ERROR_MASK (CARD_STATUS_OUT_OF_RANGE|CARD_STATUS_ADDRESS_ERROR\
                  |CARD_STATUS_BLOCK_LEN_ERROR |CARD_STATUS_ERASE_SEQ_ERROR\
                  |CARD_STATUS_ERASE_PARAM |CARD_STATUS_WP_VIOLATION\
                  |CARD_STATUS_CARD_IS_LOCKED |CARD_STATUS_LOCK_UNLOCK_FAILED\
                  |CARD_STATUS_COM_CRC_ERROR |CARD_STATUS_ILLEGAL_COMMAND \
                  |CARD_STATUS_CARD_ECC_FAILED |CARD_STATUS_CC_ERROR \
                  |CARD_STATUS_ERROR |CARD_STATUS_CSD_OVERWRITE \
                  |CARD_STATUS_AKE_SEQ_ERROR)

typedef struct __packed sd_csd_v1 {
    uint8_t     reserved0:1;
    uint8_t     crc:7;

    uint8_t     reserved8:2;
    uint8_t     file_format:2;
    uint8_t     tmp_write_protect:1;
    uint8_t     perm_write_protect:1;
    uint8_t     copy:1;
    uint8_t     file_format_grp:1;

    uint16_t     reserved16:5;
    uint16_t     write_bl_partial:1;
    uint16_t     write_bl_len:4;
    uint16_t     r2w_factor:3;
    uint16_t     reserved29:2;
    uint16_t     wp_grp_enable:1;

    uint32_t     wp_grp_size:7;
    uint32_t     sector_size:7;
    uint32_t     erase_blk_en:1;
    uint32_t     c_size_mult:3;
    uint32_t     vdd_w_curr_max:3;
    uint32_t     vdd_w_curr_min:3;
    uint32_t     vdd_r_curr_max:3;
    uint32_t     vdd_r_curr_min:3;
    uint32_t    c_size_high:2;
    uint16_t    c_size_low:10;
    uint16_t     reserved74:2;
    uint16_t     dsr_imp:1;
    uint16_t     read_blk_misalign:1;
    uint16_t     write_blk_misalign:1;
    uint16_t     read_bl_partial:1;

    uint16_t     read_bl_len:4;
    uint16_t    ccc:12;
    uint8_t     tran_speed;
    uint8_t     nsac;
    uint8_t     taac;
    uint8_t     reserved120:6;
    uint8_t     csd_structure:2;
} sd_csd_v1_t;

typedef struct __packed {
    uint8_t     reserved0:1;
    uint8_t     crc:7;
    uint8_t     reserved8:2;
    uint8_t     file_format:2;
    uint8_t     tmp_write_protect:1;
    uint8_t     perm_write_protect:1;
    uint8_t     copy:1;
    uint8_t     file_format_grp:1;
    uint16_t     reserved16:5;
    uint16_t     write_bl_partial:1;
    uint16_t     write_bl_len:4;
    uint16_t     r2w_factor:3;
    uint16_t     reserved29:2;
    uint16_t     wp_grp_enable:1;
    uint16_t     wp_grp_size:7;
    uint16_t     sector_size:7;
    uint16_t     erase_blk_en:1;
    uint16_t     reserved47:1;
    uint32_t    c_size:22;
    uint32_t    reserved70:6;
    uint32_t    dsr_imp:1;
    uint32_t     read_blk_misalign:1;
    uint32_t     write_blk_misalign:1;
    uint32_t     read_bl_partial:1;
    uint16_t     read_bl_len:4;
    uint16_t    ccc:12;
    uint8_t     tran_speed;
    uint8_t     nsac;
    uint8_t     taac;
    uint8_t     reserved120:6;
    uint8_t     csd_structure:2;
} sd_csd_v2_t;

enum sd_state {
    SD_IDLE = 0,
    SD_READY,
    SD_IDENT,
    SD_STBY,
    SD_TRAN,
    SD_DATA,
    SD_RCV,
    SD_PRG,
    SD_DIS,
    /* NON STANDARD USED FOR IMPLEMENTING INIT MACHINE */
    SD_ACMD41,
    SD_CMD11,
    SD_CMD2,
    SD_CMD3,
    SD_CMD8
};

static volatile struct {
    uint16_t    rca;
    uint8_t     version:1;
    uint8_t     s18r:1;
    uint8_t     s18a:1;
    uint8_t     hcs:1;
    uint8_t     ccs:1;
    uint8_t     send_if:1;
    uint8_t     ack_voltage:1;
    uint8_t     ACMD:1;
    volatile enum sd_state state;
    struct sd_csd_v1 csd;
    uint8_t     bus_width;
    uint8_t     timeout;
    uint8_t     error;
    struct sdio_cmd last_cmd;
    uint32_t    status_reg;

} g_sd_card;

enum card_status_bit_type {
    ERROR_BIT,
    STATUS_BIT
};

struct card_status_bit {
    uint32_t    mask;
    enum card_status_bit_type type;
};

struct card_status_bit card_status[] = {
    {CARD_STATUS_OUT_OF_RANGE, ERROR_BIT},
    {CARD_STATUS_ADDRESS_ERROR, ERROR_BIT},
    {CARD_STATUS_BLOCK_LEN_ERROR, ERROR_BIT},
    {CARD_STATUS_ERASE_SEQ_ERROR, ERROR_BIT},
    {CARD_STATUS_ERASE_PARAM, ERROR_BIT},
    {CARD_STATUS_WP_VIOLATION, ERROR_BIT},
    {CARD_STATUS_CARD_IS_LOCKED, STATUS_BIT},
    {CARD_STATUS_LOCK_UNLOCK_FAILED, ERROR_BIT},
    {CARD_STATUS_COM_CRC_ERROR, ERROR_BIT},
    {CARD_STATUS_ILLEGAL_COMMAND, ERROR_BIT},
    {CARD_STATUS_CARD_ECC_FAILED, ERROR_BIT},
    {CARD_STATUS_CC_ERROR, ERROR_BIT},
    {CARD_STATUS_ERROR, ERROR_BIT},
    {CARD_STATUS_CSD_OVERWRITE, ERROR_BIT},
    {CARD_STATUS_WP_ERASE_SKIP, ERROR_BIT},
    {CARD_STATUS_CARD_ECC_DISABLED, STATUS_BIT},
    {CARD_STATUS_ERASE_RESET, STATUS_BIT},
    {CARD_STATUS_CURRENT_STATE, STATUS_BIT},
    {CARD_STATUS_READY_FOR_DATA, STATUS_BIT},
    {CARD_STATUS_APP_CMD, STATUS_BIT},
    {CARD_STATUS_AKE_SEQ_ERROR, ERROR_BIT}
};

struct card_status_bit card_status_r6[] = {
    {CARD_STATUS_R6_COM_CRC_ERROR, ERROR_BIT},
    {CARD_STATUS_R6_ILLEGAL_COMMAND, ERROR_BIT},
    {CARD_STATUS_R6_ERROR, ERROR_BIT},
    {CARD_STATUS_CURRENT_STATE, STATUS_BIT},
    {CARD_STATUS_READY_FOR_DATA, STATUS_BIT},
    {CARD_STATUS_APP_CMD, STATUS_BIT},
    {CARD_STATUS_AKE_SEQ_ERROR, ERROR_BIT}
};

#if DEBUG_LVL >= 1
char        error_strings[][20] = {
    "",
    "",
    "",
    "AKE_SEQ_ERROR",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "WP_ERASE_SKIP",
    "CSD_OVERWRITE",
    "",
    "",
    "ERROR",
    "CC_ERROR",
    "CARD_ECC_FAILED",
    "ILLEGAL_COMMAND",
    "COM_CRC_ERROR",
    "LOCK_UNLOCK_FAILED",
    "",
    "WP_VIOLATION",
    "ERASE_PARAM",
    "ERASE_SEQ_ERROR",
    "BLOCK_LEN_ERROR",
    "ADDRESS_ERROR",
    "OUT_OF_RANGE",
};
#endif

#endif
