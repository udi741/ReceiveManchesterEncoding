#include "ManchesterEncoding.h"

/**
 * @brief Encode bytes array using Manchester and returns status of the operation
 *
 * This function encodes Bytes array 'dataArr' using Manchester(THOMAS/IEEE standards) and store the encoded data on 'encodedData', it will return status of the decoding operation.
 *
 * @param dataArr The data to encode.
 * @param lenArr The size of the data array.
 * @param encodedData Pointer to store the enccoded data.
 * @param standard Conversion method.
 * @return The status of the operation.
 */
enum StatusMessages encodeManchester(unsigned char* dataArr, unsigned int lenArr, unsigned char* encodedData, enum ConversionStandard standard)
{
    unsigned int idxEncoded = 0;
    int shiftSize;
    uint16_t encodedByte; // 2Byte size, holds the encoded result for current Byte
    unsigned char oneConvert;
    unsigned char zeroConvert;

    switch(standard)
    {
        case THOMAS:
            oneConvert = 0b10;
            zeroConvert = 0b01;
            break;
        case IEEE:
            oneConvert = 0b01;
            zeroConvert = 0b10;
            break;
        default:
            return ERROR_STANDARD;
    }

    for (int byteIdx=0; byteIdx<lenArr; byteIdx++)
    {
        shiftSize = BYTE_MSB_POSITION;
        encodedByte = 0;

        for (unsigned char mask=0x80; mask>0; mask>>=1) // loops through each bit of the data Byte
        {
            encodedByte |= ((mask & dataArr[byteIdx]) >> shiftSize) == 1 ? oneConvert : zeroConvert;
            shiftSize--;
            if (mask > 1) // last bit means we finished to build the enccoded 2-bytes
                encodedByte <<= 2;
        }
        encodedData[idxEncoded] = (unsigned char)(encodedByte >> BYTE_SIZE);
        encodedData[++idxEncoded] = (unsigned char)encodedByte;
        idxEncoded++;
    }
    return OK;
}



/**
 * @brief Decode manchester encoded bytes array and returns status of the operation
 *
 * This function decodes Bytes array 'manchesterData' using Manchester(THOMAS/IEEE standard) and store the decoded data on 'decodedData', it will return status of the decoding operation.
 *
 * @param manchesterData Decoded data using manchester encoding.
 * @param lenArr The size of the data array.
 * @param decodedData Pointer to store the decoded data.
 * @param standard Conversion method.
 * @return The status of the operation.
 */
enum StatusMessages decodeManchester(unsigned char* manchesterData, unsigned int lenArr, unsigned char* decodedData, enum ConversionStandard standard)
{
    int idxDecoded = 0;
    uint16_t encodedByte;
    unsigned char decodedByte;
    int shiftSize;
    unsigned char oneConvert;
    unsigned char zeroConvert;

    if (lenArr % 2 != 0)
        return ERROR_SIZE;

    switch(standard)
    {
        case THOMAS:
            oneConvert = 0;
            zeroConvert = 1;
            break;
        case IEEE:
            oneConvert = 1;
            zeroConvert = 0;
            break;
        default:
            return ERROR_STANDARD;
    }

    for(int i=0; i<lenArr; i+=2)
    {
        decodedByte = 0;
        shiftSize = TWO_BYTES_MSB_BITS;
        encodedByte = manchesterData[i] << BYTE_SIZE;
        encodedByte |= manchesterData[i+1];
        for (uint16_t mask=0xC000; mask>0; mask>>=2)
        {
            if (((mask & encodedByte)>>shiftSize == 0b00) || ((mask & encodedByte)>>shiftSize == 0b11))
                return INVALID_ENCODE;
    
            decodedByte |= ((mask & encodedByte) >> shiftSize == 0b01) == 1 ? oneConvert : zeroConvert; // Because the shifted result is 2 bits, and the options are only 01 or 10 it's enough to check only the LSB
            if (mask > 3) // last two bits means we finished to build the decoded byte
                decodedByte <<= 1;
            shiftSize -= 2;
        }
        decodedData[idxDecoded] = decodedByte;
        idxDecoded++;
    }
    return OK;
}