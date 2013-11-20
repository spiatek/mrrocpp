#include "mycrc.h"

uint8_t crcTable[256];

uint8_t crcSlow(const uint8_t message[], uint8_t nBytes)
{
    uint8_t  remainder = 0;
    uint8_t i, b;	//byte, bit

    /*
     * Perform modulo-2 division, a byte at a time.
     */
    for (i = 0; i < nBytes; ++i)
    {
        /*
         * Bring the next byte into the remainder.
         */
        remainder ^= (message[i] << (WIDTH - 8));

        /*
         * Perform modulo-2 division, a bit at a time.
         */
        for (b = 8; b > 0; --b)
        {
            /*
             * Try to divide the current data bit.
             */
            if (remainder & TOPBIT)
            {
                remainder = (remainder << 1) ^ POLYNOMIAL;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }
    }

    /*
     * The final remainder is the CRC result.
     */
    return (remainder);

}   /* crcSlow() */


void
crcInit(void)
{
    uint8_t  remainder;
	uint8_t b; //bit
	uint16_t dividend;

    /*
     * Compute the remainder of each possible dividend.
     */
    for (dividend = 0; dividend < 256; ++dividend)
    {
        /*
         * Start with the dividend followed by zeros.
         */
        remainder = dividend << (WIDTH - 8);

        /*
         * Perform modulo-2 division, a bit at a time.
         */
        for (b = 8; b > 0; --b)
        {
            /*
             * Try to divide the current data bit.
             */			
            if (remainder & TOPBIT)
            {
                remainder = (remainder << 1) ^ POLYNOMIAL;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }

        /*
         * Store the result into the table.
         */
        crcTable[dividend] = remainder;
    }

}   /* crcInit() */

uint8_t crcFast(const uint8_t message[], uint8_t nBytes)
{
    uint8_t mydata;
    uint8_t byte;
    uint8_t remainder = 0;


    /*
     * Divide the message by the polynomial, a byte at a time.
     */
    for (byte = 0; byte < nBytes; ++byte)
    {
        mydata = message[byte] ^ (remainder >> (WIDTH - 8));
        remainder = crcTable[mydata] ^ (remainder << 8);
    }

    /*
     * The final remainder is the CRC.
     */
    return (remainder);

}   /* crcFast() */
