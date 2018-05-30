/*
 * stk500.c
 *
 *  Created on: May 24, 2018
 *      Author: Marek Bel
 */

#include "stk500.h"
#include "command.h"
#include "write.h"
#include <avr/boot.h>
#include <avr/pgmspace.h>


/*
 * States used in the receive state machine
 */
#define ST_START        0
#define ST_GET_SEQ_NUM  1
#define ST_MSG_SIZE_1   2
#define ST_MSG_SIZE_2   3
#define ST_GET_TOKEN    4
#define ST_GET_DATA     5
#define ST_GET_CHECK    6

/*
 * Signature bytes are not available in avr-gcc io_xxx.h
 */
#if defined (__AVR_ATmega8__)
    #define SIGNATURE_BYTES 0x1E9307
#elif defined (__AVR_ATmega16__)
    #define SIGNATURE_BYTES 0x1E9403
#elif defined (__AVR_ATmega32__)
    #define SIGNATURE_BYTES 0x1E9502
#elif defined (__AVR_ATmega8515__)
    #define SIGNATURE_BYTES 0x1E9306
#elif defined (__AVR_ATmega8535__)
    #define SIGNATURE_BYTES 0x1E9308
#elif defined (__AVR_ATmega162__)
    #define SIGNATURE_BYTES 0x1E9404
#elif defined (__AVR_ATmega128__)
    #define SIGNATURE_BYTES 0x1E9702
#elif defined (__AVR_ATmega1280__)
    #define SIGNATURE_BYTES 0x1E9703
#elif defined (__AVR_ATmega2560__)
    #define SIGNATURE_BYTES 0x1E9801
#elif defined (__AVR_ATmega2561__)
    #define SIGNATURE_BYTES 0x1e9802
#elif defined (__AVR_ATmega1284P__)
    #define SIGNATURE_BYTES 0x1e9705
#elif defined (__AVR_ATmega640__)
    #define SIGNATURE_BYTES  0x1e9608
#elif defined (__AVR_ATmega64__)
    #define SIGNATURE_BYTES  0x1E9602
#elif defined (__AVR_ATmega169__)
    #define SIGNATURE_BYTES  0x1e9405
#elif defined (__AVR_AT90USB1287__)
    #define SIGNATURE_BYTES  0x1e9782
#elif defined (__AVR_ATmega32U4__)
    #define SIGNATURE_BYTES 0x1e9587
#else
    #error "no signature definition for MCU available"
#endif


/*
 * HW and SW version, reported to AVRISP, must match version of AVRStudio
 */
#define CONFIG_PARAM_BUILD_NUMBER_LOW   0
#define CONFIG_PARAM_BUILD_NUMBER_HIGH  0
#define CONFIG_PARAM_HW_VER             0x0F
#define CONFIG_PARAM_SW_MAJOR           2
#define CONFIG_PARAM_SW_MINOR           0x0A

/*
 * use 16bit address variable for ATmegas with <= 64K flash
 */
#if defined(RAMPZ)
    typedef uint32_t address_t;
#else
    typedef uint16_t address_t;
#endif


static unsigned char seqNum = 0;
static unsigned int  msgLength = 0;
static unsigned char msgBuffer[285];


/**
 * @brief Parse incoming message
 * @param c character received
 * @return msgParseState
 */
unsigned char parseMsg(unsigned char c)
{
    static unsigned int  ii = 0;
    static unsigned char msgParseState = ST_START;
    static unsigned char checksum = 0;
    if (ST_PROCESS == msgParseState) msgParseState = ST_START;

    switch (msgParseState)
    {
     case ST_START:
         if ( c == MESSAGE_START )
         {
             msgParseState   =   ST_GET_SEQ_NUM;
             checksum        =   MESSAGE_START^0;
         }
         break;

     case ST_GET_SEQ_NUM:
     #ifdef _FIX_ISSUE_505_
         seqNum          =   c;
         msgParseState   =   ST_MSG_SIZE_1;
         checksum        ^=  c;
     #else
         if ( (c == 1) || (c == seqNum) )
         {
             seqNum          =   c;
             msgParseState   =   ST_MSG_SIZE_1;
             checksum        ^=  c;
         }
         else
         {
             msgParseState   =   ST_START;
         }
     #endif
         break;

     case ST_MSG_SIZE_1:
         msgLength       =   c<<8;
         msgParseState   =   ST_MSG_SIZE_2;
         checksum        ^=  c;
         break;

     case ST_MSG_SIZE_2:
         msgLength       |=  c;
         msgParseState   =   ST_GET_TOKEN;
         checksum        ^=  c;
         break;

     case ST_GET_TOKEN:
         if ( c == TOKEN )
         {
             msgParseState   =   ST_GET_DATA;
             checksum        ^=  c;
             ii              =   0;
         }
         else
         {
             msgParseState   =   ST_START;
         }
         break;

     case ST_GET_DATA:
         msgBuffer[ii++] =   c;
         checksum        ^=  c;
         if (ii == msgLength )
         {
             msgParseState   =   ST_GET_CHECK;
         }
         break;

     case ST_GET_CHECK:
         if ( c == checksum )
         {
             msgParseState   =   ST_PROCESS;
         }
         else
         {
             msgParseState   =   ST_START;
         }
         break;
    }   //  switch
    return msgParseState;
}

/**
 * @brief Process the STK500 commands, see Atmel Appnote AVR068
 */
void processCommand(void)
{
    static unsigned char isLeave = 0;
    static address_t eraseAddress = 0;
    static address_t address = 0;
    switch (msgBuffer[0])
        {
    #ifndef REMOVE_CMD_SPI_MULTI
            case CMD_SPI_MULTI:
                {
                    unsigned char answerByte;
                    unsigned char flag=0;

                    if ( msgBuffer[4]== 0x30 )
                    {
                        unsigned char signatureIndex    =   msgBuffer[6];

                        if ( signatureIndex == 0 )
                        {
                            answerByte  =   (SIGNATURE_BYTES >> 16) & 0x000000FF;
                        }
                        else if ( signatureIndex == 1 )
                        {
                            answerByte  =   (SIGNATURE_BYTES >> 8) & 0x000000FF;
                        }
                        else
                        {
                            answerByte  =   SIGNATURE_BYTES & 0x000000FF;
                        }
                    }
                    else if ( msgBuffer[4] & 0x50 )
                    {
                    //* Issue 544:  stk500v2 bootloader doesn't support reading fuses
                    //* I cant find the docs that say what these are supposed to be but this was figured out by trial and error
                    //  answerByte  =   boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
                    //  answerByte  =   boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
                    //  answerByte  =   boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
                        if (msgBuffer[4] == 0x50)
                        {
                            answerByte  =   boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
                        }
                        else if (msgBuffer[4] == 0x58)
                        {
                            answerByte  =   boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
                        }
                        else
                        {
                            answerByte  =   0;
                        }
                    }
                    else
                    {
                        answerByte  =   0; // for all others command are not implemented, return dummy value for AVRDUDE happy <Worapoht>
                    }
                    if ( !flag )
                    {
                        msgLength       =   7;
                        msgBuffer[1]    =   STATUS_CMD_OK;
                        msgBuffer[2]    =   0;
                        msgBuffer[3]    =   msgBuffer[4];
                        msgBuffer[4]    =   0;
                        msgBuffer[5]    =   answerByte;
                        msgBuffer[6]    =   STATUS_CMD_OK;
                    }
                }
                break;
    #endif
            case CMD_SIGN_ON:
                msgLength       =   11;
                msgBuffer[1]    =   STATUS_CMD_OK;
                msgBuffer[2]    =   8;
                msgBuffer[3]    =   'A';
                msgBuffer[4]    =   'V';
                msgBuffer[5]    =   'R';
                msgBuffer[6]    =   'I';
                msgBuffer[7]    =   'S';
                msgBuffer[8]    =   'P';
                msgBuffer[9]    =   '_';
                msgBuffer[10]   =   '2';
                break;

            case CMD_GET_PARAMETER:
                {
                    unsigned char value;

                    switch(msgBuffer[1])
                    {
                    case PARAM_BUILD_NUMBER_LOW:
                        value   =   CONFIG_PARAM_BUILD_NUMBER_LOW;
                        break;
                    case PARAM_BUILD_NUMBER_HIGH:
                        value   =   CONFIG_PARAM_BUILD_NUMBER_HIGH;
                        break;
                    case PARAM_HW_VER:
                        value   =   CONFIG_PARAM_HW_VER;
                        break;
                    case PARAM_SW_MAJOR:
                        value   =   CONFIG_PARAM_SW_MAJOR;
                        break;
                    case PARAM_SW_MINOR:
                        value   =   CONFIG_PARAM_SW_MINOR;
                        break;
                    default:
                        value   =   0;
                        break;
                    }
                    msgLength       =   3;
                    msgBuffer[1]    =   STATUS_CMD_OK;
                    msgBuffer[2]    =   value;
                }
                break;

            case CMD_LEAVE_PROGMODE_ISP:
                isLeave =   1;
                //@suppress("No break at end of case")

            case CMD_SET_PARAMETER:
            case CMD_ENTER_PROGMODE_ISP:
                msgLength       =   2;
                msgBuffer[1]    =   STATUS_CMD_OK;
                break;

            case CMD_READ_SIGNATURE_ISP:
                {
                    unsigned char signatureIndex    =   msgBuffer[4];
                    unsigned char signature;

                    if ( signatureIndex == 0 )
                        signature   =   (SIGNATURE_BYTES >>16) & 0x000000FF;
                    else if ( signatureIndex == 1 )
                        signature   =   (SIGNATURE_BYTES >> 8) & 0x000000FF;
                    else
                        signature   =   SIGNATURE_BYTES & 0x000000FF;

                    msgLength       =   4;
                    msgBuffer[1]    =   STATUS_CMD_OK;
                    msgBuffer[2]    =   signature;
                    msgBuffer[3]    =   STATUS_CMD_OK;
                }
                break;

            case CMD_READ_LOCK_ISP:
                msgLength       =   4;
                msgBuffer[1]    =   STATUS_CMD_OK;
                msgBuffer[2]    =   boot_lock_fuse_bits_get( GET_LOCK_BITS );
                msgBuffer[3]    =   STATUS_CMD_OK;
                break;

            case CMD_READ_FUSE_ISP:
                {
                    unsigned char fuseBits;

                    if ( msgBuffer[2] == 0x50 )
                    {
                        if ( msgBuffer[3] == 0x08 )
                            fuseBits    =   boot_lock_fuse_bits_get( GET_EXTENDED_FUSE_BITS );
                        else
                            fuseBits    =   boot_lock_fuse_bits_get( GET_LOW_FUSE_BITS );
                    }
                    else
                    {
                        fuseBits    =   boot_lock_fuse_bits_get( GET_HIGH_FUSE_BITS );
                    }
                    msgLength       =   4;
                    msgBuffer[1]    =   STATUS_CMD_OK;
                    msgBuffer[2]    =   fuseBits;
                    msgBuffer[3]    =   STATUS_CMD_OK;
                }
                break;

    #ifndef REMOVE_PROGRAM_LOCK_BIT_SUPPORT
            case CMD_PROGRAM_LOCK_ISP:
                {
                    unsigned char lockBits  =   msgBuffer[4];

                    lockBits    =   (~lockBits) & 0x3C; // mask BLBxx bits
                    boot_lock_bits_set(lockBits);       // and program it
                    boot_spm_busy_wait();

                    msgLength       =   3;
                    msgBuffer[1]    =   STATUS_CMD_OK;
                    msgBuffer[2]    =   STATUS_CMD_OK;
                }
                break;
    #endif
            case CMD_CHIP_ERASE_ISP:
                eraseAddress    =   0;
                msgLength       =   2;
            //  msgBuffer[1]    =   STATUS_CMD_OK;
                msgBuffer[1]    =   STATUS_CMD_FAILED;  //* isue 543, return FAILED instead of OK
                break;

            case CMD_LOAD_ADDRESS:
    #if defined(RAMPZ)
                address =   ( ((address_t)(msgBuffer[1])<<24)|((address_t)(msgBuffer[2])<<16)|((address_t)(msgBuffer[3])<<8)|(msgBuffer[4]) )<<1;
    #else
                address =   ( ((msgBuffer[3])<<8)|(msgBuffer[4]) )<<1;      //convert word to byte address
    #endif
                msgLength       =   2;
                msgBuffer[1]    =   STATUS_CMD_OK;
                break;

            case CMD_PROGRAM_FLASH_ISP:
            case CMD_PROGRAM_EEPROM_ISP:
                {
                    unsigned int    size    =   ((msgBuffer[1])<<8) | msgBuffer[2];
                    unsigned char   *p  =   msgBuffer+10;
                    unsigned int    data;
                    unsigned char   highByte, lowByte;
                    address_t       tempaddress =   address;


                    if ( msgBuffer[0] == CMD_PROGRAM_FLASH_ISP )
                    {
                        // erase only main section (bootloader protection)
                        if (eraseAddress < BOOT_START_ADDR )
                        {
                            boot_page_erase(eraseAddress);  // Perform page erase
                            boot_spm_busy_wait();       // Wait until the memory is erased.
                            eraseAddress += SPM_PAGESIZE;   // point to next page to be erase
                        }

                        /* Write FLASH */
                        do {
                            lowByte     =   *p++;
                            highByte    =   *p++;

                            data        =   (highByte << 8) | lowByte;
                            boot_page_fill(address,data);

                            address =   address + 2;    // Select next word in memory
                            size    -=  2;              // Reduce number of bytes to write by two
                        } while (size);                 // Loop until all bytes written

                        boot_page_write(tempaddress);
                        boot_spm_busy_wait();
                        boot_rww_enable();              // Re-enable the RWW section
                    }
                    else
                    {
                    //* issue 543, this should work, It has not been tested.
                //  #if (!defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)  && !defined(__AVR_ATmega2561__)  && !defined(__AVR_ATmega1284P__)  && !defined(__AVR_ATmega640__))
                    #if (defined(EEARL) && defined(EEARH)  && defined(EEMWE)  && defined(EEWE)  && defined(EEDR))
                        /* write EEPROM */
                        do {
                            EEARL   =   address;            // Setup EEPROM address
                            EEARH   =   (address >> 8);
                            address++;                      // Select next EEPROM byte

                            EEDR    =   *p++;               // get byte from buffer
                            EECR    |=  (1<<EEMWE);         // Write data into EEPROM
                            EECR    |=  (1<<EEWE);

                            while (EECR & (1<<EEWE));   // Wait for write operation to finish
                            size--;                     // Decrease number of bytes to write
                        } while (size);                 // Loop until all bytes written
                    #endif
                    }
                        msgLength   =   2;
                    msgBuffer[1]    =   STATUS_CMD_OK;
                }
                break;

            case CMD_READ_FLASH_ISP:
            case CMD_READ_EEPROM_ISP:
                {
                    unsigned int    size    =   ((msgBuffer[1])<<8) | msgBuffer[2];
                    unsigned char   *p      =   msgBuffer+1;
                    msgLength               =   size+3;

                    *p++    =   STATUS_CMD_OK;
                    if (msgBuffer[0] == CMD_READ_FLASH_ISP )
                    {
                        unsigned int data;

                        // Read FLASH
                        do {
                    //#if defined(RAMPZ)
                    #if (FLASHEND > 0x10000)
                            data    =   pgm_read_word_far(address);
                    #else
                            data    =   pgm_read_word_near(address);
                    #endif
                            *p++    =   (unsigned char)data;        //LSB
                            *p++    =   (unsigned char)(data >> 8); //MSB
                            address +=  2;                          // Select next word in memory
                            size    -=  2;
                        }while (size);
                    }
                    else
                    {
                        /* Read EEPROM */
                        do {
                            EEARL   =   address;            // Setup EEPROM address
                            EEARH   =   ((address >> 8));
                            address++;                  // Select next EEPROM byte
                            EECR    |=  (1<<EERE);          // Read EEPROM
                            *p++    =   EEDR;               // Send EEPROM data
                            size--;
                        } while (size);
                    }
                    *p++    =   STATUS_CMD_OK;
                }
                break;

            default:
                msgLength       =   2;
                msgBuffer[1]    =   STATUS_CMD_FAILED;
                break;
        }
}
/**
 * @brief Send answer message back
 */
void replyMsg(void)
{
    unsigned char checksum =   MESSAGE_START^0;
    unsigned char c = ((msgLength>>8)&0xFF);

    WriteNextResponseByte(MESSAGE_START);

    WriteNextResponseByte(seqNum);
    checksum    ^=  seqNum;


    WriteNextResponseByte(c);
    checksum    ^=  c;

    c           =   msgLength&0x00FF;
    WriteNextResponseByte(c);
    checksum ^= c;

    WriteNextResponseByte(TOKEN);
    checksum ^= TOKEN;

    unsigned char *p;
    p   =   msgBuffer;
    while ( msgLength )
    {
        c   =   *p++;
        WriteNextResponseByte(c);
        checksum ^=c;
        msgLength--;
    }
    WriteNextResponseByte(checksum);
    seqNum++;

#ifndef REMOVE_BOOTLOADER_LED
    //* <MLS>   toggle the LED
    PROGLED_PORT    ^=  (1<<PROGLED_PIN);   // active high LED ON
#endif
}
