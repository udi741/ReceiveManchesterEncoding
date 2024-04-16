#ifndef MANCHESTERENCODING_H
#define MANCHESTERENCODING_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#define ARRAY_LENGTH(arr) (sizeof(arr)/sizeof(arr[0]))
#define BYTE_MSB_POSITION 7
#define TWO_BYTES_MSB_BITS 14
#define BYTE_SIZE 8

enum ConversionStandard
{
    IEEE,
    THOMAS
};

enum StatusMessages
{
    OK,
    INVALID_ENCODE,
    ERROR_STANDARD,
    ERROR_SIZE,
    WORKING
};

enum StatusMessages encodeManchester(unsigned char* data, unsigned int lenArr, unsigned char* encodedData, enum ConversionStandard standard);
enum StatusMessages decodeManchester(unsigned char* data, unsigned int lenArr, unsigned char* decodedData, enum ConversionStandard standard);
#endif