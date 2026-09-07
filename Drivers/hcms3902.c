#include "hcms3902.h"
#include "spi.h"
#include "gio.h"

#define HCMS_RS_PORT     gioPORTA
#define HCMS_RS_PIN      2

#define HCMS_RESET_PORT  gioPORTB
#define HCMS_RESET_PIN   0

#define HCMS_BLANK_PORT  gioPORTB
#define HCMS_BLANK_PIN   1

#define HCMS_SPI         spiREG1

/*--------------------------------------------------*/
/* SPI configuration                                */
/*--------------------------------------------------*/

static spiDAT1_t spi_cfg_hold =
{
    TRUE,           /* CS_HOLD */
    FALSE,          /* WDEL */
    SPI_FMT_0,
    0xFE            /* CS0 active */
};

static spiDAT1_t spi_cfg_last =
{
    FALSE,          /* release CS */
    FALSE,
    SPI_FMT_0,
    0xFE
};

/*--------------------------------------------------*/
/* Fonts (5 columns each)                           */
/*--------------------------------------------------*/

static const uint8 font_blank[5] = {0x00,0x00,0x00,0x00,0x00};
static const uint8 font_O[5] = {0x3E,0x41,0x41,0x41,0x3E};
static const uint8 font_K[5] = {0x7F,0x08,0x14,0x22,0x41};
static const uint8 font_F[5] = {0x7F,0x09,0x09,0x09,0x01};
static const uint8 font_A[5] = {0x7E,0x11,0x11,0x11,0x7E};
static const uint8 font_I[5] = {0x41,0x41,0x7F,0x41,0x41};
static const uint8 font_L[5] = {0x7F,0x40,0x40,0x40,0x40};

/*--------------------------------------------------*/
/* Helper                                           */
/*--------------------------------------------------*/

static const uint8* HCMS_GetFont(char c)
{
    switch(c)
    {
        case 'O': return font_O;
        case 'K': return font_K;
        case 'F': return font_F;
        case 'A': return font_A;
        case 'I': return font_I;
        case 'L': return font_L;
        case ' ': return font_blank;

        default:
            return font_blank;
    }
}

/*--------------------------------------------------*/
/* Write one control byte                           */
/*--------------------------------------------------*/

static void HCMS_WriteControl(uint8 value)
{
    uint16 tx;
    gioSetBit(HCMS_RS_PORT, HCMS_RS_PIN, 1);
    tx = value;
    spiTransmitData(HCMS_SPI, &spi_cfg_last, 1, &tx);
}

/*--------------------------------------------------*/
/* Display update                                   */
/*--------------------------------------------------*/

void HCMS_DisplayString(char *str)
{
    uint16 txbuf[20];
    uint32 index = 0;
    uint32 ch;
    uint32 col;

    gioSetBit(HCMS_RS_PORT, HCMS_RS_PIN, 0);

    for(ch = 0; ch < 4; ch++)
    {
        const uint8 *font;
        if(str[ch] == '\0')
        {
            font = font_blank;
        }
        else
        {
            font = HCMS_GetFont(str[ch]);
        }

        for(col = 0; col < 5; col++)
        {
            txbuf[index++] = font[col];
        }
    }

    spiTransmitData(HCMS_SPI, &spi_cfg_hold, 20, txbuf);
}

/*--------------------------------------------------*/
/* Init                                             */
/*--------------------------------------------------*/

void HCMS_Init(void)
{
    gioSetBit(HCMS_BLANK_PORT, HCMS_BLANK_PIN, 0);
    gioSetBit(HCMS_RESET_PORT, HCMS_RESET_PIN, 0);

    for(volatile uint32 i = 0; i < 50000U; i++);
    gioSetBit(HCMS_RESET_PORT, HCMS_RESET_PIN, 1);
    for(volatile uint32 i = 0; i < 50000U; i++);
    /*
     * Control Word 0
     *
     * D7=0
     * D6=1 Normal mode
     * D5=1
     * D4=1 Max current
     * D3..D0=1111 Max PWM
     */
    HCMS_WriteControl(0x7F);
    HCMS_DisplayString("    ");
}
