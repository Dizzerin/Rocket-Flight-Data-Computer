/*
 * SD_Card.c
 *
 * Generic SD card driver with card-detect interrupt support.
 *
 * States:
 *   NOT_PRESENT       — No card (detect pin HIGH, active-low).
 *   PRESENT_UNMOUNTED — Card just inserted; waiting 500 ms for power settle.
 *   MOUNTING          — Calling f_mount.
 *   MOUNTED           — Filesystem mounted; generic file ops available.
 *   ERROR             — Mount failed; stays here until card removal/re-insertion.
 *
 * The HAL_GPIO_EXTI_Callback override sets a flag on SD_CARD_DETECT changes.
 * SD_StateMachine() processes the flag and reads the pin to determine direction.
 *
 * File naming, CSV headers, and log file lifecycle are handled outside of this - in the caller (DataLogger).
 *
 * File operations (SD_FileOpen etc.) are thin wrappers that return FR_NOT_READY
 * when the filesystem is not mounted, so callers do not need to check state
 * before every FatFS call.
 */

#include <string.h>
#include "SD_Card.h"
#include "fatfs.h"
#include "user_diskio_spi.h" /* USER_SPI_deinitialize() */
#include "main.h"           /* SD_CARD_DETECT pin defines, myprintf() */
#include "stm32h7xx_hal.h"  /* HAL_GPIO_ReadPin, HAL_SPI_Abort */

/* =========================================================================
 * Module state
 * ========================================================================= */

static SD_State_t  state            = SD_NOT_PRESENT;
static FATFS       fatFs;

/* Set to 1 from EXTI ISR; cleared after SD_StateMachine() processes it */
static volatile uint8_t cardDetectChangedFlag = 0;

/* Tick captured when card is first detected — used for the CARD_SETTLE_MS wait */
static uint32_t mountWaitStartTick = 0;

#define CARD_SETTLE_MS  500U

/* =========================================================================
 * EXTI callback — called from HAL interrupt context
 * ========================================================================= */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == SD_CARD_DETECT_Pin) {
        cardDetectChangedFlag = 1;
    }
}

/* =========================================================================
 * Internal helpers
 * ========================================================================= */

/* Returns 1 if the SD card is physically present (pin LOW = active low). */
static uint8_t cardIsPresent(void)
{
    return (HAL_GPIO_ReadPin(SD_CARD_DETECT_GPIO_Port, SD_CARD_DETECT_Pin)
            == GPIO_PIN_RESET) ? 1 : 0;
}

/*
 * Perform low-level SD card teardown after physical removal.
 *
 * Must be called before f_mount(NULL, "", 0) on any removal path. Resets the
 * low-level SPI driver state so the next f_mount() forces a full disk_initialize()
 * sequence on re-insertion. Without this, FatFS sees no STA_NOINIT flag and skips
 * the CMD0/CMD8/ACMD41 init, so the new card is never put into SPI mode.
 *
 * Also aborts any in-progress SPI transaction (the HAL SPI state machine can get
 * stuck in HAL_SPI_STATE_BUSY if the card is yanked mid-transfer), and ensures
 * CS is deasserted so the next card's 80-dummy-clock init sequence runs correctly.
 */
static void sdCardTeardown(void)
{
    USER_SPI_deinitialize();   /* Reset Stat = STA_NOINIT, CardType = 0, FCLK_SLOW */
    HAL_SPI_Abort(&SD_SPI_HANDLE);
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);  /* Deassert CS */
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void SD_Init(void)
{
    cardDetectChangedFlag = 0;

    if (cardIsPresent()) {
        myprintf("SD: card present at boot — starting settle wait\r\n");
        mountWaitStartTick = HAL_GetTick();
        state = SD_PRESENT_UNMOUNTED;
    } else {
        myprintf("SD: no card at boot\r\n");
        state = SD_NOT_PRESENT;
    }
}

void SD_StateMachine(void)
{
    switch (state)
    {
        case SD_NOT_PRESENT:
            if (cardDetectChangedFlag) {
                cardDetectChangedFlag = 0;
                if (cardIsPresent()) {
                    myprintf("SD: card inserted — waiting for settle\r\n");
                    mountWaitStartTick = HAL_GetTick();
                    state = SD_PRESENT_UNMOUNTED;
                }
            }
            break;

        case SD_PRESENT_UNMOUNTED:
            /* Check for removal during the settle wait */
            if (cardDetectChangedFlag) {
                cardDetectChangedFlag = 0;
                if (!cardIsPresent()) {
                    myprintf("SD: card removed during settle\r\n");
                    state = SD_NOT_PRESENT;
                    break;
                }
            }
            if ((HAL_GetTick() - mountWaitStartTick) >= CARD_SETTLE_MS) {
                state = SD_MOUNTING;
            }
            break;

        case SD_MOUNTING:
            if (f_mount(&fatFs, "", 1) == FR_OK) {
                myprintf("SD: filesystem mounted\r\n");
                state = SD_MOUNTED;
            } else {
                myprintf("SD: f_mount failed\r\n");
                state = SD_ERROR;
            }
            break;

        case SD_MOUNTED:
            if (cardDetectChangedFlag) {
                cardDetectChangedFlag = 0;
                if (!cardIsPresent()) {
                    myprintf("SD: card removed — unmounting\r\n");
                    sdCardTeardown();
                    f_mount(NULL, "", 0);
                    state = SD_NOT_PRESENT;
                }
            }
            break;

        case SD_ERROR:
            if (cardDetectChangedFlag) {
                cardDetectChangedFlag = 0;
                if (!cardIsPresent()) {
                    myprintf("SD: card removed (was in error state)\r\n");
                    sdCardTeardown();
                    f_mount(NULL, "", 0);
                    state = SD_NOT_PRESENT;
                }
            }
            break;
    }
}

SD_State_t SD_GetState(void)
{
    return state;
}

uint8_t SD_IsMounted(void)
{
    return (state == SD_MOUNTED) ? 1 : 0;
}

/* =========================================================================
 * Generic file operations
 * ========================================================================= */

FRESULT SD_FileOpen(FIL *fp, const char *path, BYTE mode)
{
    if (state != SD_MOUNTED) return FR_NOT_READY;
    return f_open(fp, path, mode);
}

FRESULT SD_FileClose(FIL *fp)
{
    if (state != SD_MOUNTED) return FR_NOT_READY;
    return f_close(fp);
}

FRESULT SD_FileWrite(FIL *fp, const void *buff, UINT btw, UINT *bw)
{
    if (state != SD_MOUNTED) return FR_NOT_READY;
    return f_write(fp, buff, btw, bw);
}

/*
 * Flush all dirty file data and metadata (FAT, directory entry) to the SD card.
 *
 * This is intentionally separate from SD_FileWrite. Calling f_sync after every
 * write forces 2–3 sector writes per row (data sector + directory entry sector +
 * occasionally a FAT sector). Each sector write takes several milliseconds of SPI
 * time plus SD card internal write latency. At 100 Hz logging that would consume
 * far more than the available 10 ms budget per write.
 *
 * Call this periodically (e.g. once per second) rather than after every row.
 * On unexpected power-off you lose at most one sync interval worth of data.
 */
FRESULT SD_FileSync(FIL *fp)
{
    if (state != SD_MOUNTED) return FR_NOT_READY;
    return f_sync(fp);
}


/* =========================================================================
 * Directory operations
 * ========================================================================= */

FRESULT SD_DirOpen(DIR *dp, const char *path)
{
    if (state != SD_MOUNTED) return FR_NOT_READY;
    return f_opendir(dp, path);
}

// Each successive call to SD_DirRead returns the next item in the directory until it returns FR_OK with fno->fname[0] == '\0', which indicates the end of the directory listing.  So callers should loop calling SD_DirRead until they get FR_OK with fno->fname[0] == '\0' to read all items in the directory.
FRESULT SD_DirRead(DIR *dp, FILINFO *fno)
{
    return f_readdir(dp, fno);
}

FRESULT SD_DirClose(DIR *dp)
{
    return f_closedir(dp);
}
