#ifndef _HOST_INTERFACE_H_
#define _HOST_INTERFACE_H_

#include <Arduino.h>
#include "drakio_can_lib.h"
#include "pin_mapping_T4.h"

#define PACKED __attribute__ ((packed))

struct _Host_message
{
    uint8_t proto:7;
    uint8_t config:1;
    uint8_t datalength;
    uint8_t data[255];
    uint8_t datalength2;
    uint8_t data2[255];
}PACKED;

struct _usart_Data_pkt_t
{
    _Host_message   HM;
    uint8_t         buffer_full:1;
    uint8_t         buffer_cleared:1;
    uint8_t         buffer2:1;
    uint8_t         unused:5;
    uint16_t        count;
    unsigned long   sTime;

};


struct _host_pkt_t
{
    uint8_t msg_type:7;
    uint8_t flag:1;
    uint8_t dLength:8;
    uint8_t data[510];
}PACKED;

struct _adc_message
{
    uint8_t proto:7;
    uint8_t config:1;
    uint16_t data:16;
}PACKED;

struct _heartbeat_t
{    
    uint8_t msg_type:7;
    uint8_t flag:1;
}PACKED;

struct _sys_conf_pkt_t
{
    uint8_t proto:7;
    uint8_t flag:1;
    _sys_config_t sysConf;
}PACKED;



class Host_Interface
{
private:
    void ParseMessage(uint8_t dID,uint8_t dLenght, uint8_t *data);
    uint8_t getIOpktLength(_host_pkt_t pkt, uint8_t c);
    uint8_t getIOpktLength(_host_pkt_t pkt);
    char sBuffer[512];
    int sCount = 0;
    u_long baud = 115200;
    uint8_t deviceID;
    uint8_t dataLength;
    uint8_t data[255];
    uint8_t dataCount;
    //CAN_CMD canCMD;
    uint8_t Port = 0;
    _host_pkt_t hPKT;
    unsigned long sTime;
public:
    void processRequest(char *msg);
    void ResetBuffer(void);
    void BufferData(char c);
    void SetPort(uint8_t p);
    void SetBaud(u_long b);
    void write(const uint8_t *data,uint8_t count);
    Host_Interface(/* args */);
    ~Host_Interface();
};




void handle_can(void);


extern Host_Interface host_interface1;


#endif