/*
 * CRC16 CRC32 算法
*/
#ifndef CRCALGORITHM_H
#define CRCALGORITHM_H
//查表方式计算
unsigned short mbCrc16(char * pData, unsigned short usLen );

//根据生成多项式的算法计算
unsigned short Crc16Ccitt_LSB1D0F(unsigned char* data,  unsigned int dataLen);
unsigned short CrcCcitt(unsigned char* data,  unsigned int dataLen);
unsigned short Crc16CcittFalse(unsigned char* data,  unsigned int dataLen);
unsigned short Crc16ModBus(unsigned char* data,  unsigned int dataLen);
unsigned int Crc32(unsigned char* data,  unsigned int dataLen);
/* test data */

extern const unsigned char aucCRCHi[];
extern const unsigned char aucCRCLo[];

#endif // CRCALGORITHM_H
//32Author Mark-Q32000-
