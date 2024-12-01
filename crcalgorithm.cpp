/*
 * CRC16 CRC32 算法
*/
#include "crcalgorithm.h"

/********************查表方式计算**************************/
//CRC高8位矩阵值
const unsigned char aucCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,//28Author Mark-Q-28121
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
};
//CRC低8位矩阵值
const unsigned char aucCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
};
/**
函数功能：计算Modbus CRC16值
输入输出：unsigned char * pucFrame, unsigned short usLen
返回值：unsigned short
说明：
**/
unsigned short mbCrc16(char * pucFrame, unsigned short usLen)
{
    unsigned char  ucCRCHi = 0xFF;//初始值
    unsigned char  ucCRCLo = 0xFF;
    int  iIndex;

    while( usLen-- )
    {
        iIndex = ucCRCLo ^ *( pucFrame++ );
        ucCRCLo = ( unsigned char )( ucCRCHi ^ aucCRCHi[iIndex] );
        ucCRCHi = aucCRCLo[iIndex];
    }
    return static_cast<unsigned short>( ucCRCHi << 8 | ucCRCLo );
}

/********************算法方式计算**************************/

static void InvertUint8(unsigned char* dst, unsigned char* src);//字节内位反序
static void InvertUint16(unsigned short* dBuf, unsigned short* srcBuf);//字内位反序
static void InvertUint32(unsigned int* dBuf, unsigned int* srcBuf);//双字内位反序
/*算法原理：
 * 1. 根据CRC16的标准选择初值 wCrcIn的值
 * 2. 将数据的第一个字节与CrcIn高8位异或。（将数据看成一个整体的字节流）
 * 3. 判断最高位，若该位为0左移一位，若为1左移一位再与多项式Hex码异或。
 * 4. 重复步骤3至8位数全部位移计算结束
 * 5. 重复将所有输入数据操作结束。
 * 6. 最后结果有异或初值的，也有的不做。
*/
/*
 * 各函数的不同，主要涉及到多项式不同、初始值不同，结果是否需要异或，输入数据是否需要位序倒转等。
*/
/**
函数功能：CRC16 LSB1D0F，输入字节内的位反序，输出不反序
输入输出：unsigned char* data 以字节方式输入,  unsigned int dataLen 长度
返回值：unsigned short 计算后的结果16位
说明：多项式0x1021,初始值为0x1D0F,输出结果不异或,输入LSB低位优先存放，
**/
unsigned short Crc16Ccitt_LSB1D0F(unsigned char* data,  unsigned int dataLen)
{
    unsigned short wCrcIn = 0x1D0F;//
    unsigned short wCPoly = 0x1021;
    unsigned char cChar = 0;
    while (dataLen--)
    {//28mark-Q28121
        cChar = *(data++);
        InvertUint8(&cChar,&cChar);
        wCrcIn ^= (cChar<<8);
        for(int i=0; i<8; i++)
        {
            if(wCrcIn&0x8000)
                wCrcIn = (wCrcIn<<1)^ wCPoly;
            else
                wCrcIn = wCrcIn<<1;
        }
    }
    return wCrcIn;
}
/**
函数功能：CRC16 CCITT，输入字节内的位反序，输出也反序
输入输出：unsigned char* data 以字节方式输入,  unsigned int dataLen 长度
返回值：unsigned short 计算后的结果16位
说明：多项式0x1021, 初始值为0x0000, 输出结果不异或
**/
unsigned short CrcCcitt(unsigned char* data,  unsigned int dataLen)
{
    unsigned short wCrcIn = 0x0000;
    unsigned short wCPoly = 0x1021;
    unsigned char cChar = 0;
    while (dataLen--)
    {
        cChar = *(data++);
        InvertUint8(&cChar,&cChar);
        wCrcIn ^= (cChar<<8);
        for(int i=0; i<8; i++)
        {
            if(wCrcIn&0x8000)
                wCrcIn = (wCrcIn<<1)^ wCPoly;
            else
                wCrcIn = wCrcIn<<1;
        }
    }
    InvertUint16(&wCrcIn,&wCrcIn);
    return wCrcIn;
}
/**
函数功能：CRC16 CCITT FALSE，输入字节不反序，输出也不反序
输入输出：unsigned char* data 以字节方式输入,  unsigned int dataLen 长度
返回值：unsigned short 计算后的结果16位
说明：多项式0x1021,初始值为0xFFFF,输出结果不异或
**/
unsigned short Crc16CittFalse(unsigned char* data,  unsigned int dataLen)
{
    unsigned short wCrcIn = 0xFFFF;//FFFF
    unsigned short wCPoly = 0x1021;
    unsigned char cChar = 0;
    while (dataLen--)
    {
        cChar = *(data++);
        wCrcIn ^= (cChar<<8);
        for(int i=0; i<8; i++)
        {
            if(wCrcIn&0x8000)
                wCrcIn = (wCrcIn<<1)^ wCPoly;
            else
                wCrcIn = wCrcIn<<1;
        }
    }
    return wCrcIn;
}
/**
函数功能：CRC16，输入字节不反序，输出也不反序
输入输出：unsigned char* data 以字节方式输入,  unsigned int dataLen 长度
返回值：unsigned short 计算后的结果16位
说明：多项式0x1021,初始值为0x0000,结果异或0,低位在前,高位在后
**/
unsigned short Crc16Xmodem(unsigned char* data,  unsigned int dataLen)
{
    unsigned short wCrcIn = 0x0000;//FFFF
    unsigned short wCPoly = 0x1021;
    unsigned char cChar = 0;
    while (dataLen--)
    {
        cChar = *(data++);
        wCrcIn ^= (cChar<<8);
        for(int i=0; i<8; i++)
        {
            if(wCrcIn&0x8000)
                wCrcIn = (wCrcIn<<1)^ wCPoly;
            else
                wCrcIn = wCrcIn<<1;
        }
    }
    return wCrcIn;
}
/**
函数功能：CRC16 X25，输入字节内的位反序，输出也反序
输入输出：unsigned char* data 以字节方式输入,  unsigned int dataLen 长度
返回值：unsigned short 计算后的结果16位
说明：多项式0x1021,初始值为0xFFFF,结果异或0xFFFF,低位在前,高位在后
**/
unsigned short Crc16X25(unsigned char* data,  unsigned int dataLen)
{
    unsigned short wCrcIn = 0xFFFF;//FFFF
    unsigned short wCPoly = 0x1021;
    unsigned char cChar = 0;
    while (dataLen--)
    {
        cChar = *(data++);
        InvertUint8(&cChar,&cChar);
        wCrcIn ^= (cChar<<8);
        for(int i=0; i<8; i++)
        {
            if(wCrcIn&0x8000)
                wCrcIn = (wCrcIn<<1)^ wCPoly;
            else
                wCrcIn = wCrcIn<<1;
        }
    }
    InvertUint16(&wCrcIn,&wCrcIn);
    return (wCrcIn ^ 0xFFFF);//结果再异或
}
/**
函数功能：CRC16 ModBus，输入字节内的位反序，输出也反序
输入输出：unsigned char* data 以字节方式输入,  unsigned int dataLen 长度
返回值：unsigned short 计算后的结果16位
说明：多项式0x8005,初始值为0xFFFF,结果异或0,低位在前,高位在后
**/
unsigned short Crc16ModBus(unsigned char* data,  unsigned int dataLen)
{
    unsigned short wCrcIn = 0xFFFF;//FFFF
    unsigned short wCPoly = 0x8005;
    unsigned char cChar = 0;
    while (dataLen--)
    {
        cChar = *(data++);
        InvertUint8(&cChar,&cChar);
        wCrcIn ^= (cChar<<8);
        for(int i=0; i<8; i++)
        {
            if(wCrcIn&0x8000)
                wCrcIn = (wCrcIn<<1)^ wCPoly;
            else
                wCrcIn = wCrcIn<<1;
        }
    }
    InvertUint16(&wCrcIn,&wCrcIn);
    return (wCrcIn);//
}
/**
函数功能：CRC16，输入字节内的位反序，输出也反序
输入输出：unsigned char* data 以字节方式输入,  unsigned int dataLen 长度
返回值：unsigned short 计算后的结果16位
说明：多项式0x8005,初始值为0x0000,结果异或0,低位在前,高位在后
**/
unsigned short Crc16IBM(unsigned char* data,  unsigned int dataLen)
{
    unsigned short wCrcIn = 0x0000;//FFFF
    unsigned short wCPoly = 0x8005;
    unsigned char cChar = 0;
    while (dataLen--)
    {
        cChar = *(data++);
        InvertUint8(&cChar,&cChar);
        wCrcIn ^= (cChar<<8);
        for(int i=0; i<8; i++)
        {
            if(wCrcIn&0x8000)
                wCrcIn = (wCrcIn<<1)^ wCPoly;
            else
                wCrcIn = wCrcIn<<1;
        }
    }
    InvertUint16(&wCrcIn,&wCrcIn);
    return (wCrcIn);//
}
/**
函数功能：CRC16 Maxim，输入字节内的位反序，输出也反序
输入输出：unsigned char* data 以字节方式输入,  unsigned int dataLen 长度
返回值：unsigned short 计算后的结果16位
说明：多项式0x8005,初始值为0x0000,结果异或0xFFFF,低位在前,高位在后
**/
unsigned short Crc16Maxim(unsigned char* data,  unsigned int dataLen)
{
    unsigned short wCrcIn = 0x0000;//FFFF
    unsigned short wCPoly = 0x8005;
    unsigned char cChar = 0;
    while (dataLen--)
    {
        cChar = *(data++);
        InvertUint8(&cChar,&cChar);
        wCrcIn ^= (cChar<<8);
        for(int i=0; i<8; i++)
        {
            if(wCrcIn&0x8000)
                wCrcIn = (wCrcIn<<1)^ wCPoly;
            else
                wCrcIn = wCrcIn<<1;
        }
    }
    InvertUint16(&wCrcIn,&wCrcIn);
    return (wCrcIn ^ 0xFFFF);//
}
/**
函数功能：CRC16 USB，输入字节内的位反序，输出也反序
输入输出：unsigned char* data 以字节方式输入,  unsigned int dataLen 长度
返回值：unsigned short 计算后的结果16位
说明：多项式0x8005,初始值为0xFFFF,结果异或0xFFFF,低位在前,高位在后
**/
unsigned short Crc16USB(unsigned char* data,  unsigned int dataLen)
{
    unsigned short wCrcIn = 0xFFFF;//FFFF
    unsigned short wCPoly = 0x8005;
    unsigned char cChar = 0;

    while (dataLen--)
    {
        cChar = *(data++);
        InvertUint8(&cChar,&cChar);
        wCrcIn ^= (cChar<<8);
        for(int i=0; i<8; i++)
        {
            if(wCrcIn&0x8000)
                wCrcIn = (wCrcIn<<1)^ wCPoly;
            else
                wCrcIn = wCrcIn<<1;
        }
    }
    InvertUint16(&wCrcIn,&wCrcIn);
    return (wCrcIn ^ 0xFFFF);//
}
/**
函数功能：CRC32，输入字节内的位反序，输出也反序
输入输出：unsigned char* data 以字节方式输入,  unsigned int dataLen 长度
返回值：unsigned int 计算后的结果32位
说明：多项式0x04C11DB7,初始值为0xFFFFFFFF,结果异或0xFFFFFFFF,低位在前,高位在后
**/
unsigned int Crc32(unsigned char* data,  unsigned int dataLen)
{
    unsigned int wCrcIn = 0xFFFFFFFF;//FFFF
    unsigned int wCPoly = 0x04C11DB7;
    unsigned int cChar = 0;
    while (dataLen--)
    {
        cChar = *(data++);
        InvertUint8((unsigned char*)&cChar,(unsigned char*)&cChar);
        wCrcIn ^= (cChar<<24);
        for(int i=0; i<8; i++)
        {
            if(wCrcIn&0x80000000)
                wCrcIn = (wCrcIn<<1)^ wCPoly;
            else
                wCrcIn = wCrcIn<<1;
        }
    }
    InvertUint32(&wCrcIn,&wCrcIn);
    return (wCrcIn ^ 0xFFFFFFFF);//
}
/**
函数功能：字节内位反序
输入输出：unsigned char* dBuf 输出, unsigned char* srcBuf 输入
返回值：void
说明：为了满足字节内低位在前，高位在后的要求
**/
static void InvertUint8(unsigned char* dst, unsigned char* src)
{
    int i;
    unsigned char tmp;
    tmp = 0;
    for(i=0; i<8; i++)
    {
        if(src[0]&(1<<i))
            tmp |= 1<<(7-i);
    }
    dst[0] = tmp;
}
/**
函数功能：字内位反序
输入输出：unsigned short* dBuf 输出, unsigned short* src 输入
返回值：void
说明：为了满足字内低位在前，高位在后的要求
**/
static void InvertUint16(unsigned short* dst, unsigned short* src)
{
    int i;
    unsigned short tmp;
    tmp = 0;
    for(i=0; i<16; i++)
    {
        if(src[0]&(1<<i))
            tmp |= 1<<(15-i);
    }
    dst[0] = tmp;
}
/**
函数功能：双字内位反序
输入输出：unsigned int* dBuf 输出, unsigned int* srcBuf 输入
返回值：void
说明：为了满足双字内低位在前，高位在后的要求
**/
static void InvertUint32(unsigned int* dst, unsigned int* src)
{
    int i;
    unsigned int tmp;
    tmp = 0;
    for(i=0; i<32; i++)
    {
        if(src[0]&(1<<i))
            tmp |= 1<<(32-i);//15?
    }
    dst[0] = tmp;
}
