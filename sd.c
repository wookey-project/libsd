#include "libsd.h"
#include "libsdio.h"
//#include "sdio_regs.h"
//#include "sdio.h"               // FIXME: should not be included here
#include "sd.h"
#include "api/syscall.h"
#include "api/print.h"
#include "api/regutils.h"

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
dma_t       dma = {
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
};        /*dev to mem */

uint32_t saver1;
volatile uint8_t sd_error;

int         dma_descriptor;

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

uint32_t    sd_data_transfer_automaton();

static int8_t new_sd_rw(uint32_t * buffer, uint32_t addr, uint32_t size,
                        uint32_t op);

static uint32_t sd_getflags(uint32_t mask)
{
    return sdio_getflags(mask);
}

/*
 * Queue of commands
 */

static void print_csd_v2(sd_csd_v2_t * csd)
{
    printf("reserved0: %d\n", csd->reserved0);
    printf("crc: %x\n", csd->crc);
    printf("reserved8: %d\n", csd->reserved8);
    printf("file_format: %d\n", csd->file_format);
    printf("tmp_write_protect: %d\n", csd->tmp_write_protect);
    printf("perm_write_protect: %d\n", csd->perm_write_protect);
    printf("copy: %d\n", csd->copy);
    printf("file_format_grp: %d\n", csd->file_format_grp);
    printf("reserved16: %d\n", csd->reserved16);
    printf("write_bl_partial: %d\n", csd->write_bl_partial);
    printf("write_bl_len: %d\n", csd->write_bl_partial);
    printf("r2w_factor: %d\n", csd->r2w_factor);
    printf("reserved29: %d\n", csd->reserved29);
    printf("wp_grp_enable: %d\n", csd->wp_grp_enable);
    printf("wp_grp_size: %d\n", csd->wp_grp_size);
    printf("sector_size: %d\n", csd->sector_size);
    printf("erase_blk_en: %d\n", csd->erase_blk_en);
    printf("reserved47: %d\n", csd->reserved47);
    printf("c_size: %x\n", csd->c_size);
    printf("reserved70: %d\n", csd->reserved70);
    printf("dsr_imp: %d\n", csd->dsr_imp);
    printf("read_blk_misalign: %d\n", csd->read_blk_misalign);
    printf("write_blk_misalign: %d\n", csd->write_blk_misalign);
    printf("read_bl_partial: %d\n", csd->read_bl_partial);
    printf("read_bl_len: %d\n", csd->read_bl_len);
    printf("ccc: %l\n", csd->ccc);
    printf("tran_speed: %d\n", csd->tran_speed);
    printf("nsac: %d\n", csd->nsac);
    printf("taac: %d\n", csd->taac);
    printf("reserved120: %d\n", csd->reserved120);
    printf("csd_structure: %d\n", csd->csd_structure);
}

static int8_t sd_send_cmd(struct sdio_cmd cmd)
{
    sdio_clear_flag(0xffffff);
    //printf("send cmd value %x arg %x\n",cmd.cmd_value,cmd.arg);
    sdio_hw_send_cmd(cmd.cmd_value, cmd.arg, cmd.response);
    return 0;
}

static void sd_set_block_len(void)
{
    struct sdio_cmd cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_value = SD_SET_BLOCKLEN;
    cmd.arg = BLOCK_SIZE;
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
//  printf("[SD] read (buffer = %x, addr = %x, len = %x\n", buffer, addr, len);
//  dbg_flush();
#if 1
    if(lastcom == 24 || lastcom == 25)
    {
      do {
        send_cmd13_card();
        } while (!(g_sd_card.status_reg >> 8 & 1));

    }
#endif
    if (len > BLOCK_SIZE)
        {
          lastcom=18;
          return new_sd_rw(buffer, addr, len, 18 /*SD_READ_MULTIPLE_BLOCK */ );
        }
  else
        {
        lastcom=17;
       return new_sd_rw(buffer, addr, len, 17 /*SD_READ_SINGLE_BLOCK */ );
        }
}


int8_t sd_write(uint32_t * buffer, uint32_t addr, uint32_t len)
{
#if 1
    if(lastcom == 24 || lastcom == 25)
    {
      do {
        send_cmd13_card();
        } while (!(g_sd_card.status_reg >> 8 & 1));

    }
#endif
    if (len > BLOCK_SIZE)
    {
        int status=new_sd_rw(buffer, addr, len, 25 /*SD_WRITE_MULTIPLE_BLOCK */ );
        lastcom=25;
#if 0
         do {
            send_cmd13_card();
        } while (!(g_sd_card.status_reg >> 8 & 1));
#endif
          return status;
    }
    else
    {
        int status=new_sd_rw(buffer, addr, len, 24 /*SD_WRITE_BLOCK */ );
        lastcom=24;
#if 0
        do {
            send_cmd13_card();
        } while (!(g_sd_card.status_reg >> 8 & 1));
#endif
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
        uint32_t    mult = 1 << (g_sd_card.csd.c_size_mult + 2);
        uint32_t    blocknr = (((g_sd_card.csd.c_size_high<<10)
                                +g_sd_card.csd.c_size_low) + 1) * mult *2;//!!!!PHT *2
        return blocknr;
    } else {
        uint32_t    c_size =
            ((uint32_t) ((volatile sd_csd_v2_t *) (&g_sd_card.csd))->c_size);
        return ((c_size + 1) *1024);
    }
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
    uint32_t    read_bl_len = g_sd_card.csd.read_bl_len;
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
    uint32_t    r6 = sdio_hw_get_short_resp();
    uint8_t     i;
    return 0;                   //FIXME
    for (i = 0; i < ARRAY_SIZE(card_status_r6, struct card_status_bit); i++) {
        if (r6 & card_status_r6[i].mask && card_status_r6[i].type == ERROR_BIT) {
            printf("Unexpected card response\n");
            return -1;
        }
    }
    return 0;
}

#if 0

static int r2_or_r3_response(void)
{
    uint32_t    resp;
    switch (g_sd_card.state) {
    case SD_IDLE:
        /* ACMD41 */
        resp = sdio_hw_get_short_resp();
        if (!(resp >> 31)) {
            //printf("[SD] Card is busy, retrying ACMD41\n");
            resend_current_cmd();
            return -1;
        }
        /* Get CCS field, see spec Figure 4-4 */
        g_sd_card.high_capacity = (resp >> 30) & 1;
        g_sd_card.state = SD_READY;

        /* Right after ACMD 41, clock can be upper than 400kHz */
        sdio_hw_set_clock_divisor(-1);
        break;
    case SD_READY:
        /* R2 after CMD2 */
        /* TODO: get CID */
        g_sd_card.state = SD_IDENT;
        break;
    case SD_STBY:
        /* We assume that the cards CIDs have been
         * stored after the CMD2 (ALL_SEND_CID) in
         * identification mode. So there is no need for
         * CMD10 (SEND_CID) in data transfer mode.
         * Therefore, we receive a R2 response in
         * sand-by state only after a CMD9 (SEND_CSD).
         */
        sdio_hw_get_long_resp(&g_sd_card.csd);
        break;
    default:
        printf("R2_OR_R3 in wrong state\n");
    }
    return 0;
}

#endif
static int send_relative_addr_response(void)
{
    if (check_r6_error())
        return -1;

    uint32_t    resp = sdio_hw_get_short_resp();
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
    /*FIXME: Do error checking/reporting here*/
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

static void set_bus_width_sync(uint8_t width)
{
    /*TODO: check in SCR that we can use 4 wires */
    if (!(width == 4 || width == 1)) {
        return;                 // FIXME: should be a error return mecha. Do not use assert
    }
    struct sdio_cmd cmd;
    uint32_t    localretries = 0;
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
            printf("error status %x\n", sd_getflags(0xffffffff));
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
        printf("cannot set bus width\n");
    }
    g_sd_card.bus_width = width;
    sdio_hw_set_bus_width(width);
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
    return sd_getflags(SDIO_FLAG_CTIMEOUT); // Timeout is the only error condition possible
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
    return ! !(sdio_hw_get_short_resp() & ((uint32_t)1 << 31u));
}

int sd_r3_ccs()
{
    //Get the Card Capacity Support bit
    return ! !(sdio_hw_get_short_resp() & (1 << 30u));
}

static void send_cmd2()
{
    struct sdio_cmd cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_value = SD_ALL_SEND_CID;    //2;//SD_SEND_ALL_CID;
    cmd.arg = 0;
    cmd.response = LONG_RESP;
    sdio_set_timeout(0xffffffff);   //Set timout to max value
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

static void send_cmd11()
{
    //nothing yet implemented
}

/*
* State are named according to Physical Layer Simplified Spec p28 and p30
*
*/
uint32_t sdcard_init_automaton()
{
    uint32_t    err = 0, tmp = 0;
    switch (g_sd_card.state) {

        case SD_IDLE:
            if (!g_sd_card.send_if) {
                g_sd_card.state = SD_CMD8;
                sd_cmd8();          //SD_SEND_IF_COND
            } else {
                g_sd_card.state = SD_ACMD41;
                send_acmd41();      //p29 set timeout > 1s
            }
            break;
        case SD_CMD8:              //SEND_IF_COND
            tmp = sd_r7_response();
            if (tmp & SDIO_FLAG_CCRCFAIL) {
                err = 1;           //Card is unworkable
                break;
            }
            if ((tmp & SDIO_FLAG_CTIMEOUT)) {
                //FIXME: what to do if card does not support voltage?
                g_sd_card.hcs = 0;
                printf("Card does not support Voltage\n");
            } else if (tmp & SDIO_FLAG_CMDREND) {
                //FIXME: check for compatible voltage
                g_sd_card.ack_voltage = 1;
                g_sd_card.hcs = 1;  //FIXME macro SD_BOARD_HCS;
                g_sd_card.send_if = 1;
                g_sd_card.state = SD_IDLE;
                err = 1;
            } else {
                __asm__ volatile ("bkpt\n");
            }
            break;                  //go back to idle state
        case SD_ACMD41:
            if (sd_getflags(SDIO_FLAG_CTIMEOUT))
                printf("MMC Cards not supported card!");    //
            if ((*r_CORTEX_M_SDIO_CMD & 0x3f) == 55)    //CMD55)
            {
                //got the CMD_APP_CMD response
                _send_acmd41();
                break;
            }
            if (sd_getflags(SDIO_FLAG_CCRCFAIL))
                //Command CRCfail because response is OCR
            {
                if (!sd_r3_busy()) {
                    g_sd_card.state = SD_IDLE;
                } else {
                    if (g_sd_card.ack_voltage) {
                        g_sd_card.state = SD_READY;
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
                        printf("CMD2\n");
                        g_sd_card.state = SD_CMD2;
                        sdio_set_timeout(0xfffffff);
                        send_cmd2();    //SEND_ALL_CID
                    }
                }
            }
            break;
        case SD_CMD11:
            g_sd_card.state = SD_CMD2;
            send_cmd2();
            break;
        case SD_CMD2:              //ALL_SEND_CID
            err = 0;                //FIXME error handling
            if (sd_getflags(SDIO_FLAG_CTIMEOUT)) {
                err = 1;
            }
            if (sd_getflags(SDIO_FLAG_CCRCFAIL)) {
                err = 1;
            }
            if (sd_getflags(SDIO_FLAG_CMDREND)) {
                g_sd_card.state = SD_STBY;
                send_cmd3();
            }
            break;
        default:
            printf("unexecpted state\n");
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
            printf("error with DMA_RECONF: %s\n", strerror(ret));
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

uint32_t sd_data_transfer_automaton()
{
    uint32_t    err = 0, nevents;
    uint32_t    lastcom = (*r_CORTEX_M_SDIO_CMD & 0x3f);
    /*
    * For the time beeing all error reported by the block
    *  Abort the current command and are reported
    */
    if(sd_getflags(SDIO_FLAG_CTIMEOUT|SDIO_FLAG_DTIMEOUT
                        |SDIO_FLAG_CCRCFAIL|SDIO_FLAG_DCRCFAIL
                        |SDIO_FLAG_TXUNDERR|SDIO_FLAG_RXOVERR))
    {
      sd_error=SD_ERROR;
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
        if (sd_getflags(SDIO_FLAG_CMDREND)) {
            uint8_t     card_state;
            uint32_t    resp = sdio_hw_get_short_resp();
            g_sd_card.status_reg = resp;
            card_state = (resp & CARD_STATUS_CURRENT_STATE) >> 9;
            //if (card_state != g_sd_card.state) {
            /*  printf("sd.card.state is %d and card returned state %d\n",
               g_sd_card.state, card_state);
             */
            //g_sd_card.state = card_state;
            // }
        }
        return 0;
    }
    nevents = 1;                //there may have more than one event during
    //two successives calls of this automaton
    //e.g. when transfering data
    while (nevents) {
        nevents--;
        switch (g_sd_card.state) {
        case SD_STBY:
            //What is the reason for awaking in this state
            switch (lastcom) {
            case 7:            //CMD7 = SELECT_CARD
                g_sd_card.state = SD_TRAN;
                break;
            case 3:
                g_sd_card.rca = sdio_hw_get_short_resp() >> 16;
                break;
            case 4:
                break;
            case 9:            //GET_CSD
                sdio_hw_get_long_resp(&g_sd_card.csd);
                break;
            case 10:
                break;
            default:
                printf("illegal command while in SD_STBY");
            }
            break;
        case SD_TRAN:
//  printf("getting here for treating CMD %d ACMD %d\n",lastcom,g_sd_card.ACMD);
//  printf("flags %x\n",sd_getflags(0xffffffff));
            //What is the reason for awaking in this state
            g_sd_card.error = 0;
            switch ((g_sd_card.ACMD << 31) | lastcom) {
              case 7:            //CMD7 = SELECT/DESELECT_CARD
                g_sd_card.state = SD_STBY;
                break;
              case 17:           //Read Single Block
              case 18:           //Read Multiple Blocks
                if (sd_getflags(SDIO_FLAG_CMDREND)) {
                    // The command ends succesfully
                    // The expected response is R1
                    saver1=sdio_hw_get_short_resp();
                    //Let us see if an error is to be reported
                    if(saver1&CARD_STATUS_ERROR_MASK)
                    {
                        sd_error=SD_ERROR;
                        break;
                    }

                    //Let us see if current internal state is TRAN as we expected
                    if( ((saver1>>9)&0xf)!=(int)SD_TRAN) {
                        sd_error=SD_ERROR;
                        break;
                    }
                    // No Error was reported for the command let us proceed the transition
                    g_sd_card.state = SD_DATA;
                    if (sd_getflags(SDIO_FLAG_DATA))    //Have the data transfer
                        //also ended?
                        nevents++;
                    break;
                } else {
                    //Something went wrong during the command send
                    //let us report the anomaly
#if 0
                    sd_error=SD_ERROR;
                    err = -1;
#endif
                    break;
                }
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
                  saver1=sdio_hw_get_short_resp();

                  //Let us see if an error is  reported by the card
                  if(saver1&CARD_STATUS_ERROR_MASK)
                  {
                    sd_error=SD_ERROR;
                    break;
                  }
                  //Let us see if current internal state is TRAN as we expected
                  if( ((saver1>>9)&0xf)!=(int)SD_TRAN) {
                    sd_error=SD_ERROR;
                    break;
                  }


                  // No Error was reported for the command let us proceed the transition
                  g_sd_card.state = SD_RCV;
                  sd_launch_dma(0);
                  if (sd_getflags(SDIO_FLAG_DATA))    //Have the data transfer
                    //also ended?
                    nevents++;
                  break;
                } else {
                  //Some other flags than the
                  //sd_error=SD_ERROR;
                  err = 1;
                  break;
                }
              case 26:
              case 27:
              case 42:
              case 49:
              case 56:           //w
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
                printf("illegal command while in SD_TRAN %x flags %x|\n",
                    (g_sd_card.ACMD << 31) | lastcom,
                    sd_getflags(0xffffffff));
            }
            break;
        case SD_RCV:
            if ((lastcom == 25) && sdio_getflags(SDIO_FLAG_DATAEND)) {
              send_cmd12();
            } else if (((lastcom == 12) && sdio_getflags(SDIO_FLAG_CMDREND)) ||
                ((lastcom == 24) && sdio_getflags(SDIO_FLAG_DATAEND))) {
              g_sd_card.state = SD_PRG;
              nevents++;
            } else {
              //FIXME: Error chhecking here
            }
            break;
        case SD_DATA:
            //For Reading
            //Check for completion of _command_ 17/18(READ)
            if ((lastcom == 18) && sdio_getflags(SDIO_FLAG_DATAEND)) {
              send_cmd12();   //STOP_TRANSMISSION
            } else if (((lastcom == 12) && sdio_getflags(SDIO_FLAG_CMDREND)) ||
                ((lastcom == 17) && sdio_getflags(SDIO_FLAG_DATAEND))) {
              //TODO : error checking R1
              g_sd_card.state = SD_TRAN;
              //sdio_clear_flag(0xffffffff);
            } else if (lastcom == 7) {
              //Check if RCA == 0
              g_sd_card.state = SD_DIS;
            } else {
              //FIXME: Error checking here
            }

            break;
        case SD_PRG:
            if ((lastcom == 12) || sdio_getflags(SDIO_FLAG_DBCKEND)) //DataBlock End
            {
              g_sd_card.state = SD_TRAN;
            } else if (lastcom == 7) {
              //Check that is is our RCA
              g_sd_card.state = SD_DIS;
            } else {
              //printf("illegal command while in SD_PRG");
            }
            break;
        case SD_DIS:
            if (sdio_getflags(SDIO_FLAG_DBCKEND)) {
              g_sd_card.state = SD_STBY;
            } else if (lastcom == 7) {
              //Chek that is is our RCA
              g_sd_card.state = SD_PRG;
            } else {
              printf("illegal command while in SD_DIS");
            }
            break;
        default:
            break;

        }
    }
    //printf("on exit g_sd_card.state %x\n",g_sd_card.state);dbg_flush();
    return err;
}

static void prepare_transfer(dma_dir_t dir, uint32_t * buffer, uint32_t buf_len)
{
    uint8_t     ret = 0;

    if (dir == PERIPHERAL_TO_MEMORY) {
        dma.in_addr = (volatile physaddr_t)sdio_get_data_addr();
        dma.out_addr = (physaddr_t) buffer;
        dma.size = buf_len;
        dma.dir = PERIPHERAL_TO_MEMORY;
    } else {
        dma.in_addr = (physaddr_t) buffer;
        dma.out_addr = (volatile physaddr_t)sdio_get_data_addr();
        dma.size = buf_len;
        dma.dir = MEMORY_TO_PERIPHERAL;
    }
    if (ret != 0) {
        goto err;
    }

    if (dir == PERIPHERAL_TO_MEMORY) {
        sdio_prepare_dma(0xffffffff, buf_len, 9);
        sd_launch_dma(1);
        /* We launch the DMA transfer right now for reading */
    } else {
        sdio_prepare_dma(0xffffffff, buf_len, 9);
        /* for writing operation, DMA tranfer is to
           be launched after CMDREND is asserted, not now */
    }
    return;
 err:
    printf("Unable to reconfigure DMA ! : %s\n", strerror(ret));
}

static int8_t new_sd_rw(uint32_t * buffer, uint32_t addr, uint32_t size,
                        uint32_t op)
{
    struct sdio_cmd cmd;

    //printf("[SD] read (buffer = %x, addr = %x, len = %x\n", buffer, addr,size);
    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_value = op;
    /*CF P106 SDSC use bytes unit address but SDHC and SDXC use block */
    //  printf("%s: %d BLOCK_SIZE %x\n",__func__,__LINE__,BLOCK_SIZE);
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
    sd_error=SD_SUCCESS;
    sd_send_cmd(cmd);
    while (g_sd_card.state != SD_TRAN && sd_error == SD_SUCCESS) ;
    return sd_error;
}

uint8_t sd_early_init(void)
{
    uint8_t     ret = 0;
    /*******************************
     * Then DMA devices declaration
     *******************************/
    dma.channel = DMA2_CHANNEL_SDIO;
    dma.dir = MEMORY_TO_PERIPHERAL; /* write by default */
    dma.in_addr = (physaddr_t) 0;   /* to set later via DMA_RECONF */
    dma.out_addr = (volatile physaddr_t)sdio_get_data_addr();
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
        printf("%s:%d %d\n", __FILE__, __LINE__, ret);
    }

    ret |= sdio_early_init();
    if (ret) {
        printf("%s:%d %d\n", __FILE__, __LINE__, ret);
    }
    return ret;
}

uint32_t sd_init(void)
{
    int32_t     err;
    memset((void *)&g_sd_card, 0, sizeof(g_sd_card));
    g_sd_card.bus_width = 1;

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
        if (err != 0)
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
        err = 1;
        goto out;
    }
    get_csd_sync();
    select_card_sync();
    set_bus_width_sync(4);

    g_sd_card.state = SD_TRAN;
    //
    //when we reach here card is initialized AND in STBY state!
    //
    sdio_set_irq_handler(sd_data_transfer_automaton);
 out:
    return err;
}
