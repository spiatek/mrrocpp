#ifndef _MYCRC_H_
#define _MYCRC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>

//typedef uint8_t crc;

#define POLYNOMIAL 0xD8  /* 11011 followed by 0's */
#define WIDTH  (8 * sizeof(uint8_t))
#define TOPBIT (1 << (WIDTH - 1))

uint8_t crcSlow(const uint8_t message[], uint8_t nBytes);
void crcInit(void);
uint8_t crcFast(const uint8_t message[], uint8_t nBytes);

#ifdef __cplusplus
}
#endif

#endif //_MYCRC_H_
