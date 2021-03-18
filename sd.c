#include "libsd.h"
#include "libsdio.h"
#include "sd.h"
#include "libc/syscall.h"
#include "libc/stdio.h"
#include "libc/nostd.h"
#include "libc/string.h"
#include "libc/regutils.h"
#include "libc/sanhandlers.h"

/*
 * BLOCK_SIZE - Size of block on GoodUSB
 *
 * This size should depend on the size of SD blocks. The minimum size is 512.
 * Increasing BLOCK_SIZE should force to decrease BIG_POOL_SIZE.
 *
 * The only tested value is 512.
 */
#define BLOCK_SIZE 512

#define SD_BUSY 2
#define SD_SUCCESS 1
#define SD_ERROR 0


/* libsd implementation, using libsdio public API */
dma_t   dma = {
    .in_addr = 0,
    .out_addr = 0,
    .in_prio = 0,
    .out_prio = 0,
    .size = 0,
    .dma = 0,
    .channel = 0,
    .stream = 0,
    .flow_control = 0,
    .dir = 0,
    .mode = 0,
    .mem_inc = false,
    .dev_inc = false,
    .datasize = 0,
    .mem_burst = 0,
    .dev_burst = 0,
    .in_handler = NULL,
    .out_handler = NULL
};                              /*dev to mem */

uint32_t saver1;
volatile uint8_t sd_error;

int     dma_descriptor;

#define DMA2 2

#define DMA2_CHANNEL_SDIO 4

#define DMA2_STREAM_SDIO_IN 3
#define DMA2_STREAM_SDIO_OUT 6

/* We use only one stream to avoid collision with other IP's DMA like CRYP */
#define DMA2_STREAM_SDIO_FD DMA2_STREAM_SDIO_IN

static volatile uint32_t *gbuffer;
static volatile uint32_t gsize;

/* Define card status bits for R1 response */
static void prepare_transfer(dma_dir_t dir, uint32_t * buffer,
                             uint32_t buf_len);

uint32_t sd_data_transfer_automaton();

static int8_t new_sd_rw(uint32_t * buffer, uint32_t addr, uint32_t size,
                        uint32_t op);

static uint32_t sd_getflags(uint32_t mask)
{
    return sdio_getflags(mask);
}

/*
 * Queue of commands
 */
static void print_cid(sd_cid_t * cid __attribute__((unused)))
{
#ifdef CONFIG_USR_LIB_SD_DEBUG
    log_printf("reserved0: %d\n", cid->reserved0);
    log_printf("crc: %x\n", cid->crc);
    log_printf("mdt: %x\n", cid->mdt);
    log_printf("reserved20: %d\n", cid->reserved20);
    log_printf("psn: %x\n", cid->psn);
    log_printf("prv: %x\n", cid->prv);
    log_printf("pnm: %lx\n", cid->pnm);
    log_printf("oid: %x\n",cid->oid);
    log_printf("mid: %x\n",cid->mid);
#endif
}

static void print_csd_v2(sd_csd_v2_t * csd __attribute__((unused)))
{
#ifdef CONFIG_USR_LIB_SD_DEBUG
    log_printf("reserved0: %d\n", csd->reserved0);
    log_printf("crc: %x\n", csd->crc);
    log_printf("reserved8: %d\n", csd->reserved8);
    log_printf("file_format: %d\n", csd->file_format);
    log_printf("tmp_write_protect: %d\n", csd->tmp_write_protect);
    log_printf("perm_write_protect: %d\n", csd->perm_write_protect);
    log_printf("copy: %d\n", csd->copy);
    log_printf("file_format_grp: %d\n", csd->file_format_grp);
    log_printf("reserved16: %d\n", csd->reserved16);
    log_printf("write_bl_partial: %d\n", csd->write_bl_partial);
    log_printf("write_bl_len: %d\n", csd->write_bl_partial);
    log_printf("r2w_factor: %d\n", csd->r2w_factor);
    log_printf("reserved29: %d\n", csd->reserved29);
    log_printf("wp_grp_enable: %d\n", csd->wp_grp_enable);
    log_printf("wp_grp_size: %d\n", csd->wp_grp_size);
    log_printf("sector_size: %d\n", csd->sector_size);
    log_printf("erase_blk_en: %d\n", csd->erase_blk_en);
    log_printf("reserved47: %d\n", csd->reserved47);
    log_printf("c_size: %x\n", csd->c_size);
    log_printf("reserved70: %d\n", csd->reserved70);
    log_printf("dsr_imp: %d\n", csd->dsr_imp);
    log_printf("read_blk_misalign: %d\n", csd->read_blk_misalign);
    log_printf("write_blk_misalign: %d\n", csd->write_blk_misalign);
    log_printf("read_bl_partial: %d\n", csd->read_bl_partial);
    log_printf("read_bl_len: %d\n", csd->read_bl_len);
    log_printf("ccc: %l\n", csd->ccc);
    log_printf("tran_speed: %d\n", csd->tran_speed);
    log_printf("nsac: %d\n", csd->nsac);
    log_printf("taac: %d\n", csd->taac);
    log_printf("reserved120: %d\n", csd->reserved120);
    log_printf("csd_structure: %d\n", csd->csd_structure);
#endif
}

static int8_t sd_send_cmd(struct sdio_cmd cmd)
{
    sdio_clear_flag(0xffffff);
    //printf("send cmd value %x arg %x\n",cmd.cmd_value,cmd.arg);
    sdio_hw_send_cmd(cmd.cmd_value, cmd.arg, cmd.response);
    return 0;
}

void sd_set_block_len(uint32_t blocksize)
{
    struct sdio_cmd cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_value = SD_SET_BLOCKLEN;
    cmd.arg = blocksize;
    cmd.response = SHORT_RESP;
    sd_send_cmd(cmd);
}

/*
* buffer is a pointer to the actual data to be read (uint32_t *) ???
* addr -> the numero of the block to be read
* size -> the number of blocks to be read
*/
static uint8_t lastcom;
static void send_cmd13_card(void)
{
    struct sdio_cmd cmd;

    memset(&cmd, 0, sizeof(cmd));
    g_sd_card.status_reg = 0;
    cmd.cmd_value = 13;
    cmd.arg = (g_sd_card.rca << 16);
    cmd.response = SHORT_RESP;
    sd_send_cmd(cmd);
}

int8_t sd_read(uint32_t * buffer, uint32_t addr, uint32_t len)
{
    if (len == 0) {
        return SD_SUCCESS;
    }
    if (lastcom == 24 || lastcom == 25) {
        do {
            send_cmd13_card();
        } while (!(g_sd_card.status_reg >> 8 & 1));
    }
    if (len > BLOCK_SIZE) {
        lastcom = 18;
        return new_sd_rw(buffer, addr, len, 18 /*SD_READ_MULTIPLE_BLOCK */ );
    } else {
        lastcom = 17;
        return new_sd_rw(buffer, addr, len, 17 /*SD_READ_SINGLE_BLOCK */ );
    }
}


int8_t sd_write(uint32_t * buffer, uint32_t addr, uint32_t len)
{
    if (len == 0) {
        return SD_SUCCESS;
    }
    if (lastcom == 24 || lastcom == 25) {
        do {
            send_cmd13_card();
        } while (!(g_sd_card.status_reg >> 8 & 1));
    }
    if (len > BLOCK_SIZE) {
        int     status =
            new_sd_rw(buffer, addr, len, 25 /*SD_WRITE_MULTIPLE_BLOCK */ );
        lastcom = 25;
        return status;
    } else {
        int     status = new_sd_rw(buffer, addr, len, 24 /*SD_WRITE_BLOCK */ );

        lastcom = 24;
        return status;
    }
}

static void sdio_dmacallback(uint8_t irq __UNUSED, uint32_t status __UNUSED)
{

}

/*
* Return the version number of the csd structure
*/
uint32_t sd_card_csd_structure(void)
{
    return (uint32_t) g_sd_card.csd.csd_structure;
}

/* The sd card capacity is calculated from CSD fields by using formulae in spec
 * part 5.3.2 in 'C_SIZE' paragraph
 * SD Specifications Part 1 Physical Layer Simplified Specification Version 2.00 September 25, 2006
 * SD_spec_part1_4.10.pdf
 * return capacity in Kblocks
 */
uint32_t sd_get_capacity(void)
{
    if (g_sd_card.csd.csd_structure == 0) {
        uint32_t mult = 1 << (g_sd_card.csd.c_size_mult + 2);
        uint32_t tmp = g_sd_card.csd.c_size_high;
        uint32_t blocknr = (tmp + (g_sd_card.csd.c_size_low << 2) + 1) * mult * 2;      //!!!!PHT *2

        return blocknr;
    } else {
        uint32_t c_size =
            ((uint32_t) ((volatile sd_csd_v2_t *) (&g_sd_card.csd))->c_size);
        return ((c_size + 1) * 1024);
    }
}


uint8_t sd_get_cid(uint32_t **cid_p, uint32_t *cid_len)
{
    if (cid_p == NULL || cid_len == NULL) {
        return 1;
    }

    *cid_p = (uint32_t*)&g_sd_card.cid; /* cid data access */
    *cid_len = 16; /* ... in bytes */

    return 0;
}

/*
 * Return the maximum block size
 * The maximum data block length is computed as 2^READ_BL_LEN.
 * The maximum block length might therefore be in the range 512...2048 bytes
 * However, block operation must be operated on  at most 512 blocks
 * So only 512 is returned here as it is the maximal value operable
 * See SD_SPEC 4.3.2 (2GByte Card) P30
 */
uint32_t sd_get_blocksize(void)
{
    return 512;
}

uint32_t sd_get_block_number(void)
{
/*
    if (g_sd_card.csd.csd_structure == 0) {
        uint32_t    mult = 1 << (g_sd_card.csd.c_size_mult + 2);
        uint32_t    blocknr = (g_sd_card.csd.c_size + 1) * mult;
        return blocknr;
    } else {
        uint32_t    c_size =
            ((uint32_t) ((sd_csd_v2_t *) (&g_sd_card.csd))->c_size);
        return ((c_size + 1));
    }
*/
    return 512;
}

/*
 * Return the maximum block size
 * The maximum data block length is computed as 2^READ_BL_LEN.
 * The maximum block length might therefore be in the range 512...2048 bytes
 * However, block operation must be operated on  at most 512 blocks
 * So only 512 is returned here as it is the maximal value operable
 * See SD_SPEC 4.3.2 (2GByte Card) P30
 */
uint32_t sd_get_block_size(void)
{
    uint32_t read_bl_len = g_sd_card.csd.read_bl_len;

    if (read_bl_len < 9) {
        printf
            ("INVALID csd.read_bl_len value: %d => Overiding with read_bl_len = 9 \n",
             read_bl_len);
        read_bl_len = 9;
    } else if (read_bl_len > 11) {
        printf
            ("INVALID csd.read_bl_len value: %d => Overiding with read_bl_len = 11 \n",
             read_bl_len);
        read_bl_len = 11;
    }

    return 512;
    //return 1 << read_bl_len;
}

/* SEND CMD13 */
static void get_card_status(void)
{
    struct sdio_cmd cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_value = SD_SEND_STATUS;
    cmd.response = SHORT_RESP;
    sdio_hw_send_cmd(cmd.cmd.index, cmd.arg, cmd.response);
}

uint8_t sd_is_ready(void)
{
    /* SD card is in data transfer mode */
    return g_sd_card.state == SD_TRAN || g_sd_card.state == SD_DATA ||
        g_sd_card.state == SD_RCV || g_sd_card.state == SD_PRG ||
        g_sd_card.state == SD_DIS;
}

/*
 * IRQ callbacks
 */

static int check_r6_error(void)
{
    uint32_t r6 = sdio_hw_get_short_resp();
    uint8_t i;

    return 0;                   //FIXME
    for (i = 0; i < ARRAY_SIZE(card_status_r6, struct card_status_bit); i++) {
        if (r6 & card_status_r6[i].mask && card_status_r6[i].type == ERROR_BIT) {
            log_printf("Unexpected card response\n");
            return -1;
        }
    }
    return 0;
}

static int send_relative_addr_response(void)
{
    if (check_r6_error())
        return -1;

    uint32_t resp = sdio_hw_get_short_resp();

    g_sd_card.rca = resp >> 16;
    g_sd_card.state = SD_STBY;

    return 0;
}

static void get_csd_sync(void)
{
    struct sdio_cmd cmd;

    /* CMD13 */
    do {
        memset(&cmd, 0, sizeof(cmd));
        cmd.cmd_value = SD_SEND_STATUS;
        cmd.arg = g_sd_card.rca << 16;
        cmd.response = SHORT_RESP;
        sd_send_cmd(cmd);
        while (!sd_getflags
               (SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND)) ;
    }
    while (!sd_getflags(SDIO_FLAG_CMDREND) ||
           !(sdio_hw_get_short_resp() & CARD_STATUS_READY_FOR_DATA));

    /* CMD9 */
    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_value = SD_SEND_CSD;
    cmd.arg = g_sd_card.rca << 16;
    cmd.response = LONG_RESP;
    sd_send_cmd(cmd);
    while (!sd_getflags
           (SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND)) ;
    /*FIXME: Do error checking/reporting here */
    sdio_hw_get_long_resp(&g_sd_card.csd);


}

static void select_card(void)
{
    struct sdio_cmd cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_value = SD_SELECT_DESELECT_CARD;
    cmd.arg = g_sd_card.rca << 16;
    cmd.response = SHORT_RESP;
    sd_send_cmd(cmd);
}

static void select_card_sync(void)
{
    struct sdio_cmd cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_value = SD_SELECT_DESELECT_CARD;
    cmd.arg = g_sd_card.rca << 16;
    cmd.response = SHORT_RESP;
    sd_send_cmd(cmd);
    while (!sd_getflags
           (SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND)) ;
}

uint8_t sd_set_bus_width_sync(uint8_t width)
{
    uint8_t err=SD_ERROR;
    /*TODO: check in SCR that we can use 4 wires */
    if (!(width == 4 || width == 1)) {
        return err;     // FIXME: should be a error return mecha. Do not use assert
    }
    struct sdio_cmd cmd;
    uint32_t localretries = 0;

    //ACMD command
    for (localretries = 1000; 0 != localretries; localretries--) {
        memset(&cmd, 0, sizeof(cmd));
        cmd.cmd_value = SD_APP_CMD;
        cmd.arg = (g_sd_card.rca << 16);
        cmd.response = SHORT_RESP;
        sd_send_cmd(cmd);
        while (!sd_getflags
               (SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND)) ;
        if (sd_getflags(SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CCRCFAIL)) {
            log_printf("error status %x\n", sd_getflags(0xffffffff));
            continue;
        }
        memset(&cmd, 0, sizeof(cmd));
        cmd.cmd_value = 6;      //SD_SET_BUS_WIDTH;
        if (width == 4)
            cmd.arg = 2;
        else
            cmd.arg = 0;
        cmd.response = SHORT_RESP;
        sd_send_cmd(cmd);
        while (!sd_getflags
               (SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND)) ;
        if (sd_getflags(SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CCRCFAIL)) {
            continue;
        }
        break;
    }
    if (!localretries) {
        log_printf("cannot set bus width\n");
        g_sd_card.ACMD=0;//reset an eventual flag ACMD
        g_sd_card.bus_width = 1;
        sdio_hw_set_bus_width(1);
        err=SD_ERROR;
    }
    else
    {
      g_sd_card.bus_width = width;
      sdio_hw_set_bus_width(width);
      err=SD_SUCCESS;
    }
return err;
}

uint32_t sd_card_reset()
{
    /* CMD 0:  GO_IDLE_STATE */
    struct sdio_cmd cmd = {
        .cmd_value = 0,
        .arg = 0,
        .response = 0,
        .buf = NULL,
        .buf_len = 0
    };
    cmd.cmd_value = SD_GO_TO_IDLE_STATE;
    cmd.response = NO_RESP;

    sdio_wait_for(SDIO_FLAG_CMDSENT);
    sd_send_cmd(cmd);
    g_sd_card.state = SD_IDLE;
    while (!sd_getflags(SDIO_FLAG_CMDSENT)) ;
    return sd_getflags(SDIO_FLAG_CTIMEOUT);     // Timeout is the only error condition possible
}

void sd_cmd8()
{
/* CMD8 (Send Interface Condition Command) is defined to initialize SD Memory Cards compliant to the
* Physical Layer Specification Version 2.00 or later. CMD8 is valid when the card is in Idle state. This
* command has two functions.
* - Voltage check: Checks whether the card can operate on the host supply voltage.
* - Enabling expansion of existing command and response:
*   Reviving CMD8 enables to expand new functionality to some existing commands by redefining
*   previously reserved bits. ACMD41 was expanded to support initialization of SDHC Card and the
*   expansion is also applied to SDXC Card.
 */
    struct sdio_cmd cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_value = SD_SEND_IF_COND;
    cmd.arg = SD_SEND_IF_COND_27V_36V;  //& 0x000F;
    cmd.arg |= SD_SEND_IF_COND_CHECK_PATTERN;
    cmd.response = SHORT_RESP;  //CARD_INTERFACE_CONDITION
    sdio_wait_for(SDIO_FLAG_CMD_RESP);
    sd_send_cmd(cmd);

}

static uint32_t sd_r7_response()
{
    //FIXME: pass timeout to >1s
    return sd_getflags(SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND |
                       SDIO_FLAG_CTIMEOUT);
}

static void send_acmd41()
{
    struct sdio_cmd cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_value = SD_APP_CMD;
    cmd.arg = g_sd_card.rca;
    cmd.response = SHORT_RESP;
    sdio_wait_for(SDIO_FLAG_CMD_RESP);
    sd_send_cmd(cmd);

}

static void _send_acmd41()
{
    struct sdio_cmd cmd;

    log_printf("_send_acmd41 %x\n",SD_SEND_OP_COND);
    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_value = SD_SEND_OP_COND;
    //HOST CAPACITY SUPPORT (1 means HC and XC supported)
    cmd.arg = (g_sd_card.hcs << 30) | (g_sd_card.s18r << 24);
    cmd.arg |= 1 << 17;         //OCR is arg[31:16]
    cmd.arg |= 1 << 28;         // XPC SDXC bit 1=max perf 0=power saving
    //cmd.arg=0x80100000;
    cmd.response = SHORT_RESP;
    sdio_wait_for(SDIO_FLAG_CMD_RESP);
    sd_send_cmd(cmd);

}

int sd_r3_busy()
{
    //Get the busy bit
    return !!(sdio_hw_get_short_resp() & ((uint32_t) 1 << 31u));
}

int sd_r3_ccs()
{
    //Get the Card Capacity Support bit
    return !!(sdio_hw_get_short_resp() & (1 << 30u));
}

static void send_cmd2()
{
    struct sdio_cmd cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_value = SD_ALL_SEND_CID;    //2;//SD_SEND_ALL_CID;
    cmd.arg = 0;
    cmd.response = LONG_RESP;
    sdio_set_timeout(0xffffffff);       //Set timout to max value
    sdio_wait_for(SDIO_FLAG_CMD_RESP);
    sd_send_cmd(cmd);

}

static void send_cmd3()
{

    struct sdio_cmd cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_value = 3;          //SD_RELATIVE_ADDR;
    cmd.arg = g_sd_card.rca;    //Stuff bits
    cmd.response = SHORT_RESP;
    sdio_wait_for(SDIO_FLAG_CMD_RESP);
    sd_send_cmd(cmd);

}

static void send_cmd12()
{
    struct sdio_cmd cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_value = 12;         //SD_STOP;
    cmd.arg = 0;                //Stuff bits
    cmd.response = SHORT_RESP;
    sdio_wait_for(SDIO_FLAG_CMD_RESP);
    sd_send_cmd(cmd);
}

// Voltage switch for MMC cards
// Arg is OCR see P128 of JEDEC std 4.41
static void send_cmd11()
{

    struct sdio_cmd cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_value = 11;         //MMC_VOLTAGE_SWITCH;
    cmd.arg = 0x1ff<<15|2<<29;  //OCR indicating only 2.7-3.6 voltage range & sector
                                //   mode access
    cmd.response = SHORT_RESP;
    sdio_wait_for(SDIO_FLAG_CMD_RESP);
    sd_send_cmd(cmd);
}
static void send_cmd1()
{
    struct sdio_cmd cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_value = 1;         //MMC_SEND_OP_COND;
    cmd.arg = 0;                //Stuff bits
    cmd.response = SHORT_RESP;
    sdio_wait_for(SDIO_FLAG_CMD_RESP);
    sd_send_cmd(cmd);
}
uint32_t mmc_init_automaton()
{
    //This is a MultimediaCard skip CMD11 and go CMD2
    //We start as in CMD1 as stated in SD SPEC 4.1 p41
    log_printf("card does not support voltage\n");
    g_sd_card.state = SD_IDLE;
    sdio_set_timeout(0xfffffff);
    send_cmd1();    //SEND_ALL_CID
    return 1;
}

/*
* State are named according to Physical Layer Simplified Spec p28 and p30
*
*/
uint32_t sdcard_init_automaton()
{
    uint32_t err = 0, tmp = 0;
    uint32_t lastcom = (*r_CORTEX_M_SDIO_CMD & 0x3f);

    switch (g_sd_card.state) {

        case SD_IDLE:
          switch(lastcom ) {
            case 0:
              // Reinit the card
              g_sd_card.send_if=0;
              //if (CS is asserted) //SPI operation Mode
              sd_cmd8();      //SD_SEND_IF_COND
              break;
            case 8:          //SEND_IF_COND
              tmp = sd_r7_response();
              if (tmp & SDIO_FLAG_CCRCFAIL) {
                err = 1;        //Card is unworkable
                break;
              }
              if ((tmp & SDIO_FLAG_CTIMEOUT)) {
                //FIXME: what to do if card does not support voltage?
                g_sd_card.hcs = 0;
                g_sd_card.state=SD_IDLE;
                log_printf("Card does not support Voltage or is MMC\n");
                sdio_set_timeout(0xfffffff);
                g_sd_card.state = SD_ACMD41;
                send_acmd41();  //check if ACMD41 is answered
                                //if no card must be mmc
              } else if (tmp & SDIO_FLAG_CMDREND) {
                //FIXME: check for compatible voltage
                g_sd_card.ack_voltage = 1;
                g_sd_card.hcs = 1;      //FIXME macro SD_BOARD_HCS;
                sdio_set_timeout(0xfffffff);
                g_sd_card.state = SD_ACMD41;
                send_acmd41();  //p29 set timeout > 1s
                err = 1;
              } else {
                __asm__ volatile ("bkpt\n");
              }
              break;
              case 41: /* we get here if card have returned BUSY fo last ACMD41
                       * Or host omitted voltage (we did not)
                       * just retry
                        */
                g_sd_card.state = SD_ACMD41;
                send_acmd41();  //p29 set timeout > 1s
                break;
              case 55: /*
                        we reach here if we detected multimedia and last CMD returned
                        busy or host has omitted voltage (we did not)
                        just retry
                      */
                log_printf("ACMD41 Failed in timeout\n");
                  g_sd_card.state=SD_READY;
                  send_cmd1();
                  break;
              case 1:
                  /* CMD1 returned busy
                  *  just retry
                  */

                log_printf("CMD1 Failed in timeout\n");
                  g_sd_card.state=SD_READY;
                send_cmd1();   //Voltage switch

                  break;

          }
          break;              //go back to idle state
        case SD_ACMD41:
          if (sd_getflags(SDIO_FLAG_CTIMEOUT)){
            /*
             * the card must be mmc card follow mmc init automaton at CMD1
             * as stated in SD SPEC 4.1 p41
             */
            g_sd_card.sd_or_mmc=0;//We detected an MMC card
            g_sd_card.state = SD_IDLE;
            break;
          }        //
          if ((*r_CORTEX_M_SDIO_CMD & 0x3f) == 55)    //CMD55)
          {
            //got the CMD_APP_CMD response
            g_sd_card.sd_or_mmc=1;//We detected an MMC card
            g_sd_card.state = SD_READY;
            _send_acmd41();
          }
          else {
              //busy ? retry
                send_acmd41();  //p29 set timeout > 1s
          }
          break;
        case SD_READY:
          if (sd_getflags(SDIO_FLAG_CTIMEOUT)){
            log_printf("timeout while in SD_READY\n");
          }
          if (sd_getflags(SDIO_FLAG_CCRCFAIL))
            //Command CRCfail because response is OCR
          {
            if (!sd_r3_busy()) {
              g_sd_card.state = SD_IDLE;
            } else {
              if (g_sd_card.ack_voltage) {
                //g_sd_card.state = SD_READY;
                g_sd_card.version = 1;  //FIXME add enum
                g_sd_card.ccs = sd_r3_ccs();    //CCS = Card Capacity status
                g_sd_card.s18a = 0;
                //FIXME : as we set s18r to 0 this field is always 0
              } else {
                //Ver 1.X Standard Capacity SD Memorycard
                g_sd_card.ccs = 0;
                g_sd_card.version = 0;  //FIXME add enum
              }
              /* Right after ACMD 41, clock can be upper than 400kHz */
              sdio_hw_set_clock_divisor(-1);

              if (g_sd_card.s18r && g_sd_card.s18a) {
                printf
                  ("wrong path ! voltage switch impossible with wookey!");
                send_cmd11();   //Voltage switch
              } else {
                g_sd_card.state = SD_IDENT;
                sdio_set_timeout(0xfffffff);
                send_cmd2();    //SEND_ALL_CID
              }
            }
            break;
          }
          break;
        case SD_IDENT:
          switch (lastcom) {
            case 11:
              send_cmd2();
              break;
            case 2:          //ALL_SEND_CID
              err = 0;            //FIXME error handling
              if (sd_getflags(SDIO_FLAG_CTIMEOUT)) {
                err = -1;
              }
              if (sd_getflags(SDIO_FLAG_CCRCFAIL)) {
                err = -1;
              }
              if (sd_getflags(SDIO_FLAG_CMDREND)) {
                /*
                * get Long response and store the CID
                */
                sdio_hw_get_long_resp(&g_sd_card.cid);

                g_sd_card.state = SD_STBY;
                send_cmd3();
              }
              break;
          }
          break;

        default:
          log_printf("unexecpted state %d\n",g_sd_card.state);
    }
    return err;
}

void sd_launch_dma(int i)
{
    volatile uint8_t ret;

    if (i == 1) {               /*IN */
        ret = sys_cfg(CFG_DMA_RECONF, &dma,
                      DMA_RECONF_BUFIN | DMA_RECONF_BUFOUT | DMA_RECONF_BUFSIZE
                      | DMA_RECONF_DIR, dma_descriptor);
        if (ret != 0) {         // FIXME - use something like SUCCESS, FAILURE...
            log_printf("error with DMA_RECONF: %s\n", strerror(ret));
        }
    } else {                    /* OUT */
        /* out is done in ISR mode, we use fastcalls here */
        ret = sys_cfg(CFG_DMA_RECONF, &dma,
                      DMA_RECONF_BUFIN | DMA_RECONF_BUFOUT | DMA_RECONF_BUFSIZE
                      | DMA_RECONF_DIR, dma_descriptor);
    }

    if (!ret) {
        sdio_launch_dma(i);
    }
    // FIXME: sd_launch_dma should return an error code
    return;
}

void handle_send_status()
{
    if (sd_getflags(SDIO_FLAG_CMDREND)) {
        uint8_t card_state;
        uint32_t resp = sdio_hw_get_short_resp();

        saver1=resp;
        g_sd_card.status_reg = resp;
        card_state = (resp & CARD_STATUS_CURRENT_STATE) >> 9;
    }
}

/*
 SD_IDLE state : P33 SD spec
 transition function as follow
          CMD7 -> SD_TRAN
          CMD4,9,10,3 -> SD_STBY
                saver1 = sdic_hw_get_short_resp();
*/

static inline void handle_mmc_sleep(const uint32_t lastcom)
{
    //What is the reason for awaking in this state
    switch (lastcom) {
        case 5:
            g_sd_card.state = SD_STBY;
            break;
        default:
            //TODO: error checking here
            log_printf("illegal command while in SD_STBY");
    }
}

/*
 SD_IDLE state : P33 SD spec
 transition function as follow
          CMD7 -> SD_TRAN
          CMD4,9,10,3 -> SD_STBY
*/

static inline void handle_sd_stby(const uint32_t lastcom)
{
    //What is the reason for awaking in this state
    switch (lastcom) {
        case 7:                //CMD7 = SELECT_CARD
            g_sd_card.state = SD_TRAN;
            break;
        case 3:
            g_sd_card.rca = sdio_hw_get_short_resp() >> 16;
            break;
        case 4:
            break;
        case 9:                //GET_CSD
            sdio_hw_get_long_resp(&g_sd_card.csd);
            break;
        case 10:
            break;
        case 39://MMC CMD
            break;
        default:
            //TODO: error checking here
            log_printf("illegal command while in SD_STBY");
    }
}

/*
 MMC_WaitIRQ state : P61 MMC spec
 transition function as follow
          No IRQ deteted -> SD_STBY
          card IRQ -> SD_STBY
*/

static inline void handle_mmc_wait_IRQ(const __attribute__((unused)) uint32_t lastcom)
{
    //What is the reason for awaking in this state
  g_sd_card.state = SD_STBY; //not implemented yet
}

/*
 SD_TRAN state : P33 SD spec
 transition function as follow
          CMD6,17,18,19,30,48,56(r),58 or ACMD13,22,51 -> SD_DATA
          CMD16,23,32,33 or ACMD6,42,23-> SD_tran
          CMD24,25,26,27,42,49,56(w),59 -> SD_RCV
          CMD20,28,29,38 -> SD_PRG
          CMD7 -> SD_STBY
*/
static inline int handle_sd_tran(const uint32_t lastcom, uint32_t * nevents)
{
    int     err = 0;

    //What is the reason for awaking in this state
    g_sd_card.error = 0;
    switch ((g_sd_card.ACMD << 31) | lastcom) {
        case 7:                //CMD7 = SELECT/DESELECT_CARD
            g_sd_card.state = SD_STBY;
            break;
        case 17:               //Read Single Block
        case 18:               //Read Multiple Blocks
            if (sd_getflags(SDIO_FLAG_CMDREND)) {
                // The command ends succesfully
                // The expected response is R1
                saver1 = sdio_hw_get_short_resp();
                //Let us see if an error is to be reported
                if (saver1 & CARD_STATUS_ERROR_MASK) {
                    sd_error = SD_ERROR;
                    break;
                }
                //Let us see if current internal state is TRAN as we expected
                //Check is not mandatory but ensures coherency between internal card state
                //And host automaton state
                if (((saver1 >> 9) & 0xf) != (int) SD_TRAN) {
                    sd_error = SD_ERROR;
                    break;
                }
                // No Error was reported for the command let us proceed the transition
                g_sd_card.state = SD_DATA;
                if (sd_getflags(SDIO_FLAG_DATA))        //Have the data transfer
                    //also ended?
                    (*nevents)++;
            } else {
                //Something went wrong during the command send
                //let us report the anomaly
#if 0
                sd_error = SD_ERROR;
                err = -1;
#endif
            }
            break;
        case 19:
        case 30:
        case 48:
            //case 56://r
        case 58:
            //case 58:
        case ACMD(13):
        case ACMD(22):
        case ACMD(51):
            g_sd_card.ACMD = 0;
            g_sd_card.state = SD_DATA;
            break;
        case 16:
        case 23:
        case 32:
        case 33:
        case ACMD(6):
        case ACMD(42):
        case ACMD(23):
            g_sd_card.ACMD = 0;
            break;
        case 24:
        case 25:
            if (sd_getflags(SDIO_FLAG_CMDREND)) {
                // The command ended succesfully
                // The expected response is R1
                saver1 = sdio_hw_get_short_resp();

                //Let us see if an error is  reported by the card
                if (saver1 & CARD_STATUS_ERROR_MASK) {
                    sd_error = SD_ERROR;
                    break;
                }
                //Let us see if current internal state is TRAN as we expected
                if (((saver1 >> 9) & 0xf) != (int) SD_TRAN) {
                    sd_error = SD_ERROR;
                    break;
                }
                // No Error was reported for the command let us proceed the transition
                g_sd_card.state = SD_RCV;
                sd_launch_dma(0);
                if (sd_getflags(SDIO_FLAG_DATA))        //Have the data transfer
                    //also ended?
                    (*nevents)++;
                break;
            } else {
                //Some other flags than the
                //sd_error=SD_ERROR;
                err = -1;
                break;
            }
        case 26:
        case 27:
        case 42: // LOCK UNLOCK COMMAND
            if(!sd_getflags(SDIO_FLAG_DATA)) {
                if (sd_getflags(SDIO_FLAG_CMDREND)) {
                saver1 = sdio_hw_get_short_resp();
                //printf("CMDREND %x\n",saver1);
                sd_launch_dma(0);//enable DPSM
                //sdio_launch_dma(0);//enable DPSM

             //   if (sd_getflags(SDIO_FLAG_DATA))        //Have the data transfer
             //       //also ended?
             //       (*nevents)++;
            }
            }
            if (sd_getflags(SDIO_FLAG_DATA)){

              /*Fixme : error handling*/
            }
          break;


        case 49:
        case 56:               //w
        case 59:
            g_sd_card.state = SD_RCV;
            break;
        case 20:
        case 28:
        case 29:
        case 38:
            g_sd_card.state = SD_PRG;
            break;
        default:
            log_printf
                ("illegal command while in SD_TRAN %x flags %x|\n",
                 (g_sd_card.ACMD << 31) | lastcom, sd_getflags(0xffffffff));
    }
    return err;
}

/*
 SD_RCV state : P33 SD spec
 transition function as follow
          CMD12 -> SD_PRG
          tranfer end -> SD_PRG
                In our understanding this operation complete can only occur for CMD24
                (write single block)
*/
static inline void handle_sd_rcv(const uint32_t lastcom, uint32_t * nevents)
{
    if ((lastcom == 25) && sdio_getflags(SDIO_FLAG_DATAEND)) {
        send_cmd12();
    } else if (((lastcom == 12) && sdio_getflags(SDIO_FLAG_CMDREND))
               || ((lastcom == 24) && sdio_getflags(SDIO_FLAG_DATAEND))) {
        g_sd_card.state = SD_PRG;
        (*nevents)++;
    } else {
        //FIXME: Error checking here
            log_printf
                ("illegal command while in SD_RCV %x flags %x|\n",
                 (g_sd_card.ACMD << 31) | lastcom, sd_getflags(0xffffffff));
    }
}

/*
 SD_DATA state : P33 SD spec
 transition function as follow
          CMD12 -> SD_TRAN
          CMD7 -> SD_STBY
          opration complete -> SD_TRAN
            in out understanding only CMD17(read single block) can create this case
*/
static inline void handle_sd_data(const uint32_t lastcom)
{
    //For Reading
    //Check for completion of _command_ 17/18(READ)
    if ((lastcom == 18) && sdio_getflags(SDIO_FLAG_DATAEND)) {
        send_cmd12();           //STOP_TRANSMISSION
    } else if (((lastcom == 12) && sdio_getflags(SDIO_FLAG_CMDREND))
               || ((lastcom == 17) && sdio_getflags(SDIO_FLAG_DATAEND))) {
        //error checking of R1 is performed before calling this function
        g_sd_card.state = SD_TRAN;
        //sdio_clear_flag(0xffffffff);
    } else if (lastcom == 7) {
        //Check if RCA == 0
        g_sd_card.state = SD_IDLE;
    } else {
        //FIXME: Error checking here
        log_printf("illegal command while in SD_DATA\n");
    }
}

/*
 SD_PRG state : P33 SD spec
 transition function as follow
          CMD12 -> SD_TRAN
          CMD7 -> SD_STBY
          operation complete -> SD_TRAN
*/
static inline void handle_sd_prg(const uint32_t lastcom)
{
    if ((lastcom == 12) || sdio_getflags(SDIO_FLAG_DBCKEND))    //DataBlock End
    {
        g_sd_card.state = SD_TRAN;
    } else if (lastcom == 7) {
        //Check that is is our RCA
        g_sd_card.state = SD_DIS;
    } else {
            printf
                ("illegal command while in SD_PRG %x flags %x|\n",
                 (g_sd_card.ACMD << 31) | lastcom, sd_getflags(0xffffffff));

    }
}

/*
 SD_DIS state : P33 SD spec
 transition function as follow
          CMD7 -> SD_SD_PRG
          operation complete -> SD_STBY
*/
static inline void handle_sd_dis(const uint32_t lastcom)
{
    if (sdio_getflags(SDIO_FLAG_DBCKEND)) {
        g_sd_card.state = SD_STBY;
    } else if (lastcom == 7) {
        //Check that is is our RCA
        g_sd_card.state = SD_PRG;
    } else {
        log_printf("illegal command while in SD_DIS");
    }
}

uint32_t sd_data_transfer_automaton()
{
    uint32_t err = 0, nevents;
    uint32_t lastcom = (*r_CORTEX_M_SDIO_CMD & 0x3f);

    /*
     * For the time beeing all error reported by the hw block
     *  Abort the current command and are reported
     */
    if (sd_getflags(SDIO_FLAG_CTIMEOUT | SDIO_FLAG_DTIMEOUT
                    | SDIO_FLAG_CCRCFAIL | SDIO_FLAG_DCRCFAIL
                    | SDIO_FLAG_TXUNDERR | SDIO_FLAG_RXOVERR)) {
        sd_error = SD_ERROR;
        return 1;
    }
    //CF: Page 35 STM-RM00090
    if (lastcom == 55)          //CMD_ACMD
    {
        g_sd_card.ACMD = 1;
        return 0;
    }
    if (lastcom == 13)          //SEND_STATUS
    {
        handle_send_status();   // Side effect : update internal state
        return 0;
    }
    nevents = 1;
    /*
     *there may have more than one event during
     *two successives calls of this automaton
     * e.g. when transfering data
     */
    while (nevents) {
        nevents--;
        switch (g_sd_card.state) {
            case SD_STBY:
                handle_sd_stby(lastcom);
                break;
            case SD_TRAN:
                err = handle_sd_tran(lastcom, &nevents);
                break;
            case SD_RCV:
                handle_sd_rcv(lastcom, &nevents);
                break;
            case SD_DATA:
                handle_sd_data(lastcom);
                break;
            case SD_PRG:
                handle_sd_prg(lastcom);
                break;
            case SD_DIS:
                handle_sd_dis(lastcom);
                break;
            default:
                log_printf("Unexpected state!");
                for(;;);/* do not go any further */
                break;

        }
    }
    //printf("on exit g_sd_card.state %x\n",g_sd_card.state);dbg_flush();
    return err;
}

static void prepare_transfer_customblock(dma_dir_t dir, uint32_t * buffer, uint32_t buf_len, uint32_t log2_blocksize)
{
    uint8_t ret = 0;

    if (dir == PERIPHERAL_TO_MEMORY) {
        dma.in_addr = (volatile physaddr_t) sdio_get_data_addr();
        dma.out_addr = (physaddr_t) buffer;
        dma.size = buf_len;
        dma.dir = PERIPHERAL_TO_MEMORY;
    } else {
        dma.in_addr = (physaddr_t) buffer;
        dma.out_addr = (volatile physaddr_t) sdio_get_data_addr();
        dma.size = buf_len;
        dma.dir = MEMORY_TO_PERIPHERAL;
    }
    if (ret != 0) {
        goto err;
    }

    if (dir == PERIPHERAL_TO_MEMORY) {
        sdio_prepare_dma(SDIO_READ_TIMEOUT, buf_len, log2_blocksize);
        sd_launch_dma(1);
        /* We launch the DMA transfer right now for reading */
    } else {
        sdio_prepare_dma(SDIO_WRITE_TIMEOUT, buf_len, log2_blocksize);
        /* for writing operation, DMA tranfer is to
           be launched after CMDREND is asserted, not now */
    }
    return;
 err:
    log_printf("Unable to reconfigure DMA ! : %s\n", strerror(ret));
}

static void prepare_transfer(dma_dir_t dir, uint32_t * buffer, uint32_t buf_len)
{
    uint8_t ret = 0;

    if (dir == PERIPHERAL_TO_MEMORY) {
        dma.in_addr = (volatile physaddr_t) sdio_get_data_addr();
        dma.out_addr = (physaddr_t) buffer;
        dma.size = buf_len;
        dma.dir = PERIPHERAL_TO_MEMORY;
    } else {
        dma.in_addr = (physaddr_t) buffer;
        dma.out_addr = (volatile physaddr_t) sdio_get_data_addr();
        dma.size = buf_len;
        dma.dir = MEMORY_TO_PERIPHERAL;
    }
    if (ret != 0) {
        goto err;
    }

    if (dir == PERIPHERAL_TO_MEMORY) {
        sdio_prepare_dma(SDIO_READ_TIMEOUT, buf_len, 9);
        sd_launch_dma(1);
        /* We launch the DMA transfer right now for reading */
    } else {
        sdio_prepare_dma(SDIO_WRITE_TIMEOUT, buf_len, 9);
        /* for writing operation, DMA tranfer is to
           be launched after CMDREND is asserted, not now */
    }
    return;
 err:
    log_printf("Unable to reconfigure DMA ! : %s\n", strerror(ret));
}

static int8_t new_sd_rw(uint32_t * buffer, uint32_t addr, uint32_t size,
                        uint32_t op)
{
    struct sdio_cmd cmd;

    //printf("[SD] read (buffer = %x, addr = %x, len = %x\n", buffer, addr,size);
    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_value = op;
    /*CF P106 SDSC use bytes unit address but SDHC and SDXC use block */
    //  log_printf("%s: %d BLOCK_SIZE %x\n",__func__,__LINE__,BLOCK_SIZE);
    cmd.arg = g_sd_card.ccs ? addr : BLOCK_SIZE * addr;
    cmd.response = SHORT_RESP;
    //send read/write cmd
    if (g_sd_card.state != SD_TRAN)
        return SD_BUSY;
    gbuffer = buffer;
    gsize = size;

    //arm the data automaton
    if ((op == 17) || (op == 18)) {
        prepare_transfer(PERIPHERAL_TO_MEMORY, buffer, size);
    } else {
        prepare_transfer(MEMORY_TO_PERIPHERAL, buffer, size);
    }
    sd_error = SD_SUCCESS;
    sd_send_cmd(cmd);
    while (g_sd_card.state != SD_TRAN && sd_error == SD_SUCCESS) ;
    return sd_error;
}

uint8_t sd_early_init(void)
{
    uint8_t ret = 0;

    /*******************************
     * Then DMA devices declaration
     *******************************/
    dma.channel = DMA2_CHANNEL_SDIO;
    dma.dir = MEMORY_TO_PERIPHERAL;     /* write by default */
    dma.in_addr = (physaddr_t) 0;       /* to set later via DMA_RECONF */
    dma.out_addr = (volatile physaddr_t) sdio_get_data_addr();
    dma.in_prio = DMA_PRI_HIGH;
    dma.dma = DMA2;
    dma.size = 0;               /* to set later via DMA_RECONF */

    dma.stream = DMA2_STREAM_SDIO_FD;

    dma.mode = DMA_FIFO_MODE;
    dma.mem_inc = 1;
    dma.dev_inc = 0;
    dma.datasize = DMA_DS_WORD;
    dma.mem_burst = DMA_BURST_INC4;
    dma.dev_burst = DMA_BURST_INC4;
    dma.flow_control = DMA_FLOWCTRL_DEV;
    dma.in_handler = (user_dma_handler_t) sdio_dmacallback;
    dma.out_handler = (user_dma_handler_t) sdio_dmacallback;

    ret = sys_init(INIT_DMA, &dma, &dma_descriptor);
    if (ret) {
        log_printf("%s:%d %d\n", __FILE__, __LINE__, ret);
    }

    ret |= sdio_early_init();
    if (ret) {
        log_printf("%s:%d %d\n", __FILE__, __LINE__, ret);
    }
    return ret;
}
static void prepare_transfer_nodma(dma_dir_t dir, uint32_t __attribute__((unused)) *buffer, uint32_t buf_len)
{
    if (dir == PERIPHERAL_TO_MEMORY) {
        sdio_prepare_nodma(SDIO_READ_TIMEOUT, buf_len, 9);
    } else {
        sdio_prepare_nodma(SDIO_WRITE_TIMEOUT, buf_len, 9);
    }
    return;
}

static void send_cmd42_nodma(uint8_t __attribute__((unused)) *block, uint32_t __attribute__((unused))blocklen)
{
    struct sdio_cmd cmd;

    memset(&cmd, 0, sizeof(cmd));
    g_sd_card.status_reg = 0;
    cmd.cmd_value = 42;
    cmd.arg=0x00;
    cmd.response = SHORT_RESP;
    prepare_transfer_nodma(MEMORY_TO_PERIPHERAL,(uint32_t*)block,blocklen);

    sd_send_cmd(cmd);

}
static void send_cmd42(uint8_t *block, uint32_t blocklen)
{
    struct sdio_cmd cmd;
    memset(&cmd, 0, sizeof(cmd));
    g_sd_card.status_reg = 0;
    cmd.cmd_value = 42;
    cmd.arg=0x00;
    cmd.response = SHORT_RESP;
    prepare_transfer(MEMORY_TO_PERIPHERAL,(uint32_t*)block,blocklen);

    sd_send_cmd(cmd);

}
static uint8_t block[512];
void sd_clear_password(uint8_t* oldpwd, uint8_t oldlen)
{
  //the card shall be selected before calling this function
  //send_cmd16_syncset block len to 512 (mandatory according SD Spec)
  //sd_set_block_len(512);

  //wait for completion
  while(!sd_getflags(SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND));

  memset(block,0,sizeof(block));
  block[0]=0x6; // means clear password and unlock the card
  block[1]=oldlen;

  for(int i=0;i<oldlen;i++)//first old password
    block[i+2]=oldpwd[i];

  send_cmd42(block, sizeof(block));
  while(!sd_getflags(SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CCRCFAIL|SDIO_FLAG_DCRCFAIL|SDIO_FLAG_DTIMEOUT|SDIO_FLAG_DBCKEND));
  log_printf("savestatus %x r1 %x\n",savestatus, saver1);
  while(savestatus);
  //Error checking
  /*FIXME*/
}

void sd_set_password(uint8_t* oldpwd, uint8_t oldlen,
                      uint8_t *pwd, uint8_t len)
{
  //wait for completion
  while(!sd_getflags(SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND));

  memset(block,0,sizeof(block));
  //block[0]=0x2; // means clear password and lock the card
  block[0]=0x5; // means set password and lock the card
  block[1]=oldlen+len;

  if(len > 16) {
    log_printf("Wrong password length\n");
    return ;
  }

  if(oldlen > 16) {
    log_printf("Wrong Old password length\n");
    return ;
  }
  for(int i=0;i<oldlen;i++)//first old password
    block[i+2]=oldpwd[i];

  for(int i=0;i<len;i++) //next new password
    block[i+2+oldlen]=pwd[i];
  send_cmd42(block, sizeof(block));
  while(!sd_getflags(SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND));
  while(!sd_getflags(SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CCRCFAIL|SDIO_FLAG_DCRCFAIL|SDIO_FLAG_DTIMEOUT|SDIO_FLAG_DBCKEND|SDIO_FLAG_DATAEND));
  log_printf("locking done savestatus %x r1 %x\n",savestatus, saver1);
  //Error checking
  /*FIXME*/
}
void sd_unlock_card(uint8_t *pwd,uint8_t len)
{
  //wait for completion
  while(!sd_getflags(SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND));

  memset(block,0,sizeof(block));
  block[0]=0x0;// means unlock the card
  block[1]=len;
  if(len > 16) {
    log_printf("Wrong password length\n");
    return ;
  }
  for(int i=0;i<len;i++) //next new password
    block[i+2]=pwd[i];
  send_cmd42(block,sizeof(block));
  while(!sd_getflags(SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DBCKEND));
  //Error checking

  if (sd_set_bus_width_sync(4) == SD_ERROR) {
      log_printf("ERR: failure while setting SD bus width to 4!\n");
  }
}
void sdio_hw_write_fifo(uint32_t * buf, uint32_t size);
void sd_forceerase_card()
{
  //wait for completion
  while(!sd_getflags(SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND));

  memset(block,0x08,sizeof(block));
  block[0]=0x08;// means unlock the card

  send_cmd42(block,sizeof(block));
  while(!sd_getflags(SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND));

  while(!sd_getflags(SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DBCKEND));
  do {
    send_cmd13_card();
  } while (!(g_sd_card.status_reg >> 8 & 1));

  //Error checking

}


uint32_t sd_init(void)
{
    int32_t err;

    memset((void *) &g_sd_card, 0, sizeof(g_sd_card));
    g_sd_card.bus_width = 1;

    //assume that the card is an sd_card
    g_sd_card.sd_or_mmc=1;// 1 means SD
    sdio_init();
    // From whatever state we are coming for perform a card reset and go to IDLE STATE
    err = sd_card_reset();
    g_sd_card.state = SD_IDLE;
    if (err)
        goto out;

    while (g_sd_card.state != SD_STBY) {
        /*
         *these two lines are a place holder for a proper sleep command
         */
        sdio_set_timeout(0xffffffff);
        (err = sdcard_init_automaton());
        if (err < 0)
            goto out;
        while (!sdio_finished_or_error()) ;

        //sys_yield();
        //This is the proper state machine
    }
    while (!sd_getflags
           (SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND)) {
        sys_yield();
    }
    if (send_relative_addr_response()) {
        err = -1;
        goto out;
    }
    get_csd_sync();
    select_card_sync();

    /* try to set bus width to 4. If it fails, the SDCard is probably locked. */
//    if (sd_set_bus_width_sync(4) == SD_ERROR) {
//        log_printf("INFO: setting SD bus width to 4 failed! Card locked?\n");
//    }

    g_sd_card.state = SD_TRAN;

    //
    //when we reach here card is initialized AND in STBY state!
    //
    /* Register our handler */
    ADD_LOC_HANDLER(sd_data_transfer_automaton)
    sdio_set_irq_handler(sd_data_transfer_automaton);

 out:
    return err;
}

uint8_t sd_is_locked()
{
  return !!(saver1&(1<<25));
}
