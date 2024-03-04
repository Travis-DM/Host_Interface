#include <Arduino.h>
#include "host_interface.h" 
#include "string.h"

//from pin_mapping_T4
#include "teensy_map.h"
#include "pin_mapping_T4.h"
#include "analog.h"
#include "gpio.h"
#include "pwm.h"


#include "i2c.h"
#include "led.h"

#include "version.h"

//uint8_t dataCount;
char buffer[512];
uint8_t buffCount;


uint8_t Host_Interface::getIOpktLength(_host_pkt_t pkt)
{
    //GPIO Write
    if(pkt.msg_type <= 0X28)
    {
        if(pkt.flag == 1)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    if((pkt.msg_type == HEARTBEAT)||(pkt.msg_type == 0x2E)||(pkt.msg_type == 0x30)||((pkt.msg_type >= ADC0ID)&&(pkt.msg_type <= ADCFID))||(pkt.msg_type == I2C1ID)||(pkt.msg_type == I2C2ID)||(pkt.msg_type == I2C0ID))
    {
        return 0;
    }
    return 0XFF;

}

uint8_t Host_Interface::getIOpktLength(_host_pkt_t pkt, uint8_t c)
{    //USART Write
    if(((pkt.msg_type >= USART0ID)&&(pkt.msg_type <= USART7ID))||pkt.msg_type == USB_USARTID)
    {
        return c;
    }
        //GPIO Write
    if(pkt.msg_type <= 0X28)
    {
        
        dataCount = 1;
        hPKT.data[0] = c;
        SerialUSB2.printf(" c data %x : %x \n\r",c,pkt.data[0]);
        return 0;
    }
    //LED Write
    if((pkt.msg_type >= LED0ID)&&(pkt.msg_type <= LED3ID))
    {
        dataCount = 1;
        hPKT.data[0] = c;
        return 4;
    }
    //GPIO Binary Write
    if(pkt.msg_type == GPIOWID)
    {
        dataCount = 1;
        hPKT.data[0] = c;
        return 6;
    }
    //CAN Write
    if((pkt.msg_type >= CAN0ID)&&(pkt.msg_type <= CANFDID))
    {
        //Break down CAN Packet
        return 64;
    }
    //PWM Write
    if((pkt.msg_type >= PWM0ID)&&(pkt.msg_type <= PWM19ID))
    {
        dataCount = 1;
        hPKT.data[0] = c;
        return 1;
    }
    return 0XFF;

}


void Host_Interface::ResetBuffer(void)
{
    if(millis() > (sTime + 500))
    {
        sCount = 0;
        sTime = millis();
    }
    //SerialUSB2.printf("buffer reset: %d :: %x\n\r",sCount);
}

void Host_Interface::BufferData(char c)
{
    sTime = millis();
    SerialUSB2.printf("got data: %d :: %x\n\r",c,sCount);
    if (sCount == 0)
    {
        buffCount = 0;
        deviceID = c;
        hPKT.msg_type = c;
        if(c >= 0X80)
        {
            hPKT.flag = 1;
        }
        buffer[buffCount] = c;
        buffCount++;
        sCount++;
        dataLength = getIOpktLength(hPKT);
        SerialUSB2.printf("deviceID %x\n\r",deviceID);
        //Take care of single byte commands
        //if((deviceID <= 0x28)||(deviceID == HEARTBEAT)||(deviceID == 0x2E)||(deviceID == 0x30)||((deviceID >= ADC0ID)&&(deviceID <= ADCFID))||(deviceID == I2C1ID)||(deviceID == I2C2ID)||(deviceID == I2C0ID))
        if(dataLength == 0)
        {
            SerialUSB2.println("in one byte mode");
            ParseMessage(deviceID,0,data);
            dataCount = 0;
            data[dataCount] = 0;
            sCount = 0;
        }
        
    }
    else if (sCount == 1)
    {
        dataCount = 0;
        dataLength = getIOpktLength(hPKT,c);
        if (dataLength == 0)
        {
            SerialUSB2.println("in 2 byte mode");
            ParseMessage(hPKT.msg_type,1,hPKT.data);
        }
        sCount++;
        buffer[buffCount] = c;
        buffCount++;
        
        uint8_t temp = deviceID & 0X7F;
        switch (temp)
        {
        case CAN0ID:
        case CAN1ID:
        case CAN2ID:
        case CANFDID:
            {
                //canCMD = ((dataLength & 0x03) << 1) + ((deviceID & 0x80) >> 7);
                //dataLength = (dataLength & 0xFC) >> 2;
                dataLength = 64; 
                sCount = 3;
                break;
            }
        default:
            break;
        }
        
    }
    else if(sCount == 2)
    {
        //dataLength = dataLength + (c * 0X100);
        dataLength--;
        sCount++;
        buffer[buffCount] = c;
        buffCount++;
        //dataCount = 0;
        if(dataLength == 0)
        {
            ParseMessage(deviceID,dataLength,data);
            dataCount = 0;
            data[dataCount] = 0;
            sCount = 0;
        }
    }
    else
    {
        data[dataCount] = c;
        buffer[buffCount] = c;
        buffCount++;
        dataCount++;
        if(dataCount >= dataLength)
        {
            ParseMessage(deviceID,dataLength,data);
            dataCount = 0;
            data[dataCount] = 0;
            sCount = 0;
        }
    }


}
void Host_Interface::SetPort(uint8_t P)
{
    Port = P;
}

void Host_Interface::SetBaud(u_long b)
{
    baud = b;
}



void Host_Interface::ParseMessage(uint8_t _dID,uint8_t dLenght, uint8_t *data)
{
    //hostInterface2.printf("in parser %x\n\r",data[0]);
    uint8_t dID = _dID & 0X7F;
    SerialUSB2.printf("Parsing the Data\n\r");
    //SerialUSB2.printf("dID %x:: _dID::%x\n\r",dID,_dID);
    SerialUSB2.printf(" device flag data %x : %x : %x \n\r",hPKT.msg_type,hPKT.flag, hPKT.data[0]);
    if ((dID & 0X7F) <= 41 ) //Gpio pin function
    {
        if(hPKT.flag == 1)
        {
            SerialUSB2.printf("Writing pin %x : %x \n\r",dID,hPKT.data[0]);
            gpio[dID & 0X3F].Write(hPKT.data[0]);
        }
        else
        {    
            uint8_t packet[2];
            packet[1] = gpio[dID & 0X3F].Read();
            packet[0] = dID & 0X3F;
            write((byte*)packet,2);
        }
    }
    //What device
    switch (dID)
    {
    case HEARTBEAT:{
        _heartbeat_t pkt;
        SerialUSB2.printf("in HeartBeat::%d\n\r",sizeof(pkt));
        //received heartbeat
        //ToDo Add timestamp
        pkt.msg_type = HEARTBEAT;
        write((uint8_t *)&pkt,sizeof(pkt));
    }
        break;
    case LCDID:
        //write message to LCD
        break;
    case LED0ID:{
        if(led0EN())
        {
            uint8_t red = data[0];
            uint8_t green = data[1];
            uint8_t blue = data[2];
            int color = (red << 16) + (green << 8) + blue;
            for (int x = 0; x < 1; x++)
            {
                //setLED(x, color);
            //
            }
        }
        else
        {
            uint8_t packet[3];
            packet[0] = (LED0ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            write((byte*)packet,3);
        }
    }
        break;
    case LED1ID:{
        if(led1EN())
        {
            //set led color
        }
        else
        {
            uint8_t packet[3];
            packet[0] = (LED1ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            write((byte*)packet,3);
        }
    }
        break;

    case LED2ID:{
        if(led2EN())
        {
            //set led color
        }
        else
        {
            uint8_t packet[3];
            packet[0] = (LED2ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            write((byte*)packet,3);
        }
    }
        break;
    case LED3ID:{
        if(led3EN())
        {
            //set led color
        }
        else
        {
            uint8_t packet[3];
            packet[0] = (LED3ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            write((byte*)packet,3);
        }
    }
        break;
    case VERSIONID:
    {
        int dLength;
        _Host_message msg;
        msg.proto = VERSIONID;
        msg.config = 0;
        dLength = strlen(_VERSION_);
        if (dLength <= 255)
        {
            msg.datalength = dLength;
            memcpy(msg.data,_VERSION_,msg.datalength);
        }
        else
        {
            msg.datalength = 255;
            memcpy(msg.data,_VERSION_,msg.datalength);
            msg.datalength2 = dLength - 255;
            memcpy(msg.data,_VERSION_+255,msg.datalength);
        }
        write((byte*)&msg,dLength+3);
        break;
    }
    case RESETID:
    {   
        //This should take a code
        //ToDo Check Hash Value
        _Host_message msg;
        msg.proto = RESETID;
        msg.config = 0;
        write((byte*)&msg,1);
        delay(1000);
        _reboot_Teensyduino_();
        break;
    }
    case MEMORYID:{
            if(_dID > 0x80)
            {
                //write memory
            }
            else
            {
                //read memory
            }
    }
        break;
        /*
    case CAN0ID:
        {
            
            canCMD = ((dataLength & 0x03) << 1) + ((deviceID & 0x80) >> 7);
            dataLength = (dataLength & 0xFC) >> 2;
            if (can0EN())
            {
                switch (canCMD) {
                    case READ:
                        can0.read(cmsg);
                        umsg.h.proto = deviceID & 0x7F;
                        umsg.h.cmd = READ;
                        umsg.h.len = cmsg.len;
                        umsg.h.ts = cmsg.timestamp;
                        umsg.h.id = cmsg.id;
                        umsg.h.flags = cmsg.flags.extended | (cmsg.flags.remote << 8) | (cmsg.flags.overrun << 16);
                        memcpy(&umsg.payload, cmsg.buf, cmsg.len);
                        Serial.write((char *)&umsg, umsg.h.len + sizeof(USB_msg_header_t)); // USB message
                        Serial.send_now();
                    break;
                    case WRITE:
                        memcpy(&umsg,buffer,buffCount);
                        cmsg.id = *((uint32_t *) &umsg.h.id);
                        memcpy(cmsg.buf, &umsg.payload, umsg.h.len); 
                        can0.write(cmsg);
                    break;
                    case BUS_ON:
                        CAN_SETUP(0);
                    break;
                    case BUS_OFF:
                        // There is no end() method in the FlexCan library, so this becomes a no-op for compatibility with Kvaser CANlib.
                        // We can at least disable interrupts, although this needs fixing in the FlexCAN library, and unregister the callback.
                        CAN_DISABLE_INTR_HANDLER(0);
                    break;
                    case SET_PARMS:
                        handle = umsg.h.handle;
                        bitrate = *((uint32_t *)umsg.payload);
                        can0.setBaudRate(bitrate); 
                    break;
                    case OPEN:
                    break;
                    case CLOSE:
                    break;
                    default:
                    break;  
            }
            }
        }
        break;
    case CAN1ID:
        {
            
            canCMD = ((dataLength & 0x03) << 1) + ((deviceID & 0x80) >> 7);
            dataLength = (dataLength & 0xFC) >> 2;
            if (can1EN())
            {
                switch (canCMD) {
                    case READ:
                        can1.read(cmsg);
                        umsg.h.proto = deviceID & 0x7F;
                        umsg.h.cmd = READ;
                        umsg.h.len = cmsg.len;
                        umsg.h.ts = cmsg.timestamp;
                        umsg.h.id = cmsg.id;
                        umsg.h.flags = cmsg.flags.extended | (cmsg.flags.remote << 8) | (cmsg.flags.overrun << 16);
                        memcpy(&umsg.payload, cmsg.buf, cmsg.len);
                        Serial.write((char *)&umsg, umsg.h.len + sizeof(USB_msg_header_t)); // USB message
                        Serial.send_now();
                    break;
                    case WRITE:
                        memcpy(&umsg,buffer,buffCount);
                        cmsg.id = *((uint32_t *) &umsg.h.id);
                        memcpy(cmsg.buf, &umsg.payload, umsg.h.len); 
                        can1.write(cmsg);
                    break;
                    case BUS_ON:
                        CAN_SETUP(0);
                    break;
                    case BUS_OFF:
                        // There is no end() method in the FlexCan library, so this becomes a no-op for compatibility with Kvaser CANlib.
                        // We can at least disable interrupts, although this needs fixing in the FlexCAN library, and unregister the callback.
                        CAN_DISABLE_INTR_HANDLER(0);
                    break;
                    case SET_PARMS:
                        handle = umsg.h.handle;
                        bitrate = *((uint32_t *)umsg.payload);
                        can1.setBaudRate(bitrate); 
                    break;
                    case OPEN:
                    break;
                    case CLOSE:
                    break;
                    default:
                    break;  
            }
            }
        }
        break;
    case CAN2ID:
        {
            
            canCMD = ((dataLength & 0x03) << 1) + ((deviceID & 0x80) >> 7);
            dataLength = (dataLength & 0xFC) >> 2;
            if (can2EN())
            {
                switch (canCMD) {
                    case READ:
                        can2.read(cmsg);
                        umsg.h.proto = deviceID & 0x7F;
                        umsg.h.cmd = READ;
                        umsg.h.len = cmsg.len;
                        umsg.h.ts = cmsg.timestamp;
                        umsg.h.id = cmsg.id;
                        umsg.h.flags = cmsg.flags.extended | (cmsg.flags.remote << 8) | (cmsg.flags.overrun << 16);
                        memcpy(&umsg.payload, cmsg.buf, cmsg.len);
                        Serial.write((char *)&umsg, umsg.h.len + sizeof(USB_msg_header_t)); // USB message
                        Serial.send_now();
                    break;
                    case WRITE:
                        memcpy(&umsg,buffer,buffCount);
                        cmsg.id = *((uint32_t *) &umsg.h.id);
                        memcpy(cmsg.buf, &umsg.payload, umsg.h.len); 
                        can2.write(cmsg);
                    break;
                    case BUS_ON:
                        CAN_SETUP(0);
                    break;
                    case BUS_OFF:
                        // There is no end() method in the FlexCan library, so this becomes a no-op for compatibility with Kvaser CANlib.
                        // We can at least disable interrupts, although this needs fixing in the FlexCAN library, and unregister the callback.
                        CAN_DISABLE_INTR_HANDLER(0);
                    break;
                    case SET_PARMS:
                        handle = umsg.h.handle;
                        bitrate = *((uint32_t *)umsg.payload);
                        can2.setBaudRate(bitrate); 
                    break;
                    case OPEN:
                    break;
                    case CLOSE:
                    break;
                    default:
                    break;  
            }
            }
        }
        break;
    case CANFDID:
        break;
    case USART0ID:
        if (usart0EN())
        {
            if(_dID > 0x80)
            {
                //configure serial port
                //is the package the right size

            }
            else
            {
                debugger.msg(3,"Sending data to USART0");
                for (int x = 0; x < dLenght; x++)
                {
                    debugger.msg(3,"%c",data[x]);
                    USART0.printf("%c",data[x]);
                }
            }
        }
        else
        {
            debugger.msg(3,"USART0 is not configured as USART");
            uint8_t packet[3];
            packet[0] = (USART0ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            write((byte*)packet,3);
        }
        break;*/
    case USART1ID:
        if (usart1EN())
        {            
            if(_dID > 0x80)
            {
                //configure serial port
            }
            else
            {
                for (int x = 0; x < dLenght; x++)
                {
                    USART1.printf("%c",data[x]);
                }
            }
        }
        else
        {
            uint8_t packet[3];
            packet[0] = (USART1ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            write((byte*)packet,3);
        }
        break;
    case USART2ID:
        if (usart2EN())
        {     
            if(_dID > 0x80)
            {
                
            }
            else
            {       
                for (int x = 0; x < dLenght; x++)
                {
                    USART2.printf("%c",data[x]);
                }
            }
        }
        else
        {
            uint8_t packet[3];
            packet[0] = (USART2ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            write((byte*)packet,3);
        }
        break;
    case USART3ID:
        if (usart3EN())
        {      
            if(_dID > 0x80)
            {
                //configure serial port
            }
            else
            {      
                for (int x = 0; x < dLenght; x++)
                {
                    USART3.printf("%c",data[x]);
                }
            }
        }
        else
        {
            uint8_t packet[3];
            packet[0] = (USART3ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            write((byte*)packet,3);
        }
        break;  
    case USART4ID:
        if (usart4EN())
        {            
            if(_dID > 0x80)
            {
                //configure serial port
            }
            else
            {
                for (int x = 0; x < dLenght; x++)
                {
                    USART4.printf("%c",data[x]);
                }
            }
        }
        else
        {
            uint8_t packet[3];
            packet[0] = (USART4ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            write((byte*)packet,3);
        }
        break;  
    case USART5ID:
        if (usart5EN())
        {        
            if(_dID > 0x80)
            {
                //configure serial port
            }
            else
            {    
                for (int x = 0; x < dLenght; x++)
                {
                    USART5.printf("%c",data[x]);
                }
            }
        }
        else
        {
            uint8_t packet[3];
            packet[0] = (USART5ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            write((byte*)packet,3);
        }
        break;
    case USART6ID:
        if (usart6EN())
        {    
            if(_dID > 0x80)
            {
                //configure serial port
            }
            else
            {        
                for (int x = 0; x < dLenght; x++)
                {
                    USART6.printf("%c",data[x]);
                }
            }
        }
        else
        {
            uint8_t packet[3];
            packet[0] = (USART6ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            write((byte*)packet,3);
        }
        break;
    case USART7ID:
        if (usart7EN())
        {    
            if(_dID > 0x80)
            {
                //configure serial port
            }
            else
            {        
                for (int x = 0; x < dLenght; x++)
                {
                    USART7.printf("%c",data[x]);
                }
            }
        }
        else
        {
            uint8_t packet[3];
            packet[0] = (USART7ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            write((byte*)packet,3);
        }
        break;
    case I2C0ID:
        {
            uint8_t bus = 0;
            if (i2c0EN())
            {
                uint8_t* devices;
                uint8_t dcount = busScan(bus ,devices);
                for(int x =0; x < dcount; x++)
                {
                }
            }
            else
            {
                uint8_t packet[3];
                packet[0] = (I2C0ID+0x80);
                packet[1] = 0;
                packet[2] = 0;
                Serial1.write((byte*)packet,3);
            }
        break;
        }
    case I2C1ID:
        {
        uint8_t bus = 1;
        if (i2c1EN())
        {
            uint8_t* devices;
            uint8_t dcount = busScan(bus ,devices);
        }
        else
        {
            uint8_t packet[3];
            packet[0] = (I2C0ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            write((byte*)packet,3);
        }
        break;
        }
    case I2C2ID:     
        {   
        uint8_t bus = 2;
        if (i2c2EN())
        {
            uint8_t devices[128];
            uint8_t dcount = busScan(bus , devices);
        }
        else
        {
            uint8_t packet[3];
            packet[0] = (I2C0ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            write((byte*)packet,3);
        }
        break;
        }
    case ADC0ID:
    case ADC1ID:
    case ADC2ID:
    case ADC3ID:
    case ADC4ID:
    case ADC5ID:
    case ADC6ID:
    case ADC7ID:
    case ADC8ID:
    case ADC9ID:
    case ADCAID:
    case ADCBID:
    case ADCCID:
    case ADCDID:
    case ADCEID:
    case ADCFID:
    case ADC10ID:
    case ADC11ID:
        uint16_t value = analog[dID - ADC0ID].Read();
        _adc_message msg;
        msg.proto = dID;
        msg.config = 0;
        msg.data = value;
        write((uint8_t *)&msg,sizeof(msg));
        break;
    case PWM0ID:
    case PWM1ID:
    case PWM2ID:
    case PWM3ID:
    case PWM4ID:
    case PWM5ID:
    case PWM6ID:
    case PWM7ID:
    case PWM8ID:
    case PWM9ID:
    case PWMAID:
    case PWMBID:
    case PWMCID:
    case PWMDID:
    case PWMEID:
    case PWMFID:
    case PWM10ID:
    case PWM11ID:
    case PWM12ID:
    case PWM13ID:
    case PWM14ID:
    case PWM15ID:
    case PWM16ID:
    case PWM17ID:
    case PWM18ID:
    case PWM19ID:{
        uint16_t lvalue = 0;
        lvalue = (hPKT.data[1] << 8) + hPKT.data[0];
        SerialUSB2.printf("PWM:%x:%x::%x\n\r",hPKT.data[0],hPKT.data[1]);
        pwm[dID - PWM0ID].Write(lvalue);
    }
            break;
    default:
        break;
    }
}
    

void Host_Interface::write(const uint8_t *data,uint8_t count)
{
    switch (Port)
    {
    case 0:
        hostInterface.write(data,count);
        break;
    case 1:
        hostInterface1.write(data,count);
        break;
    case 2:
        hostInterface2.write(data,count);
        break;
    
    default:
        break;
    }
}

Host_Interface::Host_Interface(/* args */)
{
}

Host_Interface::~Host_Interface()
{
}
/*
void canDump(const CAN_message_t &msg) {
  
  static USB_msg_t hmsg;  

#ifdef DEBUG
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
#endif

  // Construct a hmsg to return to host.
  // Here, we only care about the payload carrying a CAN message.
  // NB: Should be PROTO_CAN0 if bus is 0 or PROTO_CAN1 if bus is 1.
  // We subtract 1 from msg.bus as the callback handler assumes msg.bus starts from 1. Go figure!
  hmsg.h.proto = CAN0ID + msg.bus - 1; 
  hmsg.h.cmd = READ;
  hmsg.h.len = msg.len;
  hmsg.h.ts = msg.timestamp;
  hmsg.h.handle = 0; // Default for now
  hmsg.h.id = msg.id;
  hmsg.h.flags = msg.flags.extended | (msg.flags.remote << 8) | (msg.flags.overrun << 16);
  memcpy(hmsg.payload, msg.buf, msg.len);
  if (msg.bus == 1) { // Interface acm0
    Serial.write((byte *)&hmsg, msg.len+sizeof(USB_msg_header_t));
    Serial.send_now();
  }
  else if (msg.bus == 2) { // Interface acm1
    SerialUSB1.write((byte *)&hmsg, msg.len+sizeof(USB_msg_header_t));
    SerialUSB1.send_now();
  }
}

void handle_can(void)
{
  //Handle CAN Interfaces
  if (can0EN())
  {
    can0.events();
  }
  if (can1EN())
  {
    can1.events();
  }
  if (can2EN())
  {
    can2.events();
  }
}
*/
void LED::setLEDs(uint8_t red, uint8_t blue, uint8_t green)
{

    int color = (red << 16) + (green << 8) + blue;
    for (int x = 0; x < ledCount; x++)
    {
        
    //
    }
}

LED::LED(/* args */)
{
}

LED::~LED()
{
}

void LED::Setup(int s_pin, uint8_t s_pID, uint8_t mode, uint8_t s_ledCount, uint8_t red, uint8_t blue, uint8_t green)
{
    pID = s_pID;
    lpin = s_pin;
    ledCount = s_ledCount;
    setLEDs(red,blue,green);
}




void LED::setLEDs(int color)
{
    for (int x = 0; x < ledCount; x++)
    {
        
    }
}

