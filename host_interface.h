#ifndef _HOST_INTERFACE_H_
#define _HOST_INTERFACE_H_

#include <Arduino.h>

class Host_Interface
{
private:
    void ParseMessage(uint8_t dID,uint8_t dLenght, uint8_t *data);
    char sBuffer[512];
    int sCount = 0;
    uint8_t deviceID;
    uint8_t dataLength;
    uint8_t data[255];
    uint8_t dataCount;
    uint8_t canCMD;
public:
    void BufferData(char c);
    Host_Interface(/* args */);
    ~Host_Interface();
};

struct _Host_message
{
    uint8_t proto:7;
    uint8_t config:1;
    uint8_t datalength;
    uint8_t data[255];
    uint8_t datalength2;
    uint8_t data2[255];
};



void handle_can(void);




#endif