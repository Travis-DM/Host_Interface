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

void Host_Interface::BufferData(char c)
{
    if (sCount == 0)
    {
        buffCount = 0;
        debug.printf("got id %x\n",c);
        deviceID = c;
        buffer[buffCount] = c;
        buffCount++;
        sCount++;
        //Take care of single byte commands
        if((deviceID <= 0x28)||(deviceID == 0x2E)||(deviceID == 0x30)||((deviceID >= ADC0ID)&&(deviceID <= ADCFID))||(deviceID == I2C1ID)||(deviceID == I2C2ID)||(deviceID == I2C0ID))
        {
            debug.println("Single Byte");
            ParseMessage(deviceID,0,data);
            dataCount = 0;
            data[dataCount] = 0;
            sCount = 0;
        }
        
    }
    else if (sCount == 1)
    {
        debug.printf("got length %d\n",c);
        dataLength = c;
        sCount++;
        buffer[buffCount] = c;
        buffCount++;
        dataCount = 0;
        uint8_t temp = deviceID & 0X7F;
        switch (temp)
        {
        case CAN0ID:
        case CAN1ID:
        case CAN2ID:
        case CANFDID:
            {
                //canCMD = ((dataLength & 0x03) << 1) + ((deviceID & 0x80) >> 7);
                dataLength = (dataLength & 0xFC) >> 2; 
                sCount = 3;
                break;
            }
        default:
            break;
        }
        
    }
    else if(sCount == 2)
    {
        debug.printf("got length 2%d\n",c);
        dataLength = dataLength + (c * 0X100);
        sCount++;
        buffer[buffCount] = c;
        buffCount++;
        dataCount = 0;
        debug.printf("data length %d\n",dataLength);
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
        debug.printf("data : %x ::: data count : %d ::: dataLenght : %d\n",c,dataCount,dataLength);
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

void Host_Interface::ParseMessage(uint8_t _dID,uint8_t dLenght, uint8_t *data)
{
    uint8_t dID = _dID & 0X7F;
    if ((dID & 0X7F) <= 41 ) //Gpio pin function
    {
        debug.printf("dID : %d\n",_dID);
        if((_dID & 0X80) >= 1)
        {
            debug.printf("Writing pin : %d\n", dID & 0X7F);
            gpio[dID & 0X3F].Write(data[0]);
        }
        else
        {
            debug.printf("Reading pin : %d\n",dID);
            gpio[dID & 0X3F].Read();
        }
    }
    //What device
    debug.printf("_dID = %x\n",_dID);
    switch (_dID)
    {
    case HEARTBEAT:{
        uint8_t packet[3];
        //received heartbeat
        //ToDo Add timestamp
        debug.println("Received Heartbeat");
        packet[0] = HEARTBEAT;
        packet[1] = 0;
        packet[2] = 0;
        hostInterface.write((byte*)packet,3);
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
            debug.printf("Setting LED string to 0x%x red, 0x%x green, 0x%x blue\n",red,green,blue);
            int color = (red << 16) + (green << 8) + blue;
            debug.printf("LED Color = 0x%x\n",color);
            for (int x = 0; x < 1; x++)
            {
                setLED(x, color);
            //
            }
        }
        else
        {
            debug.println("LED0 is not configured as LED");
            uint8_t packet[3];
            packet[0] = (LED0ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            hostInterface.write((byte*)packet,3);
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
            debug.println("LED1 is not configured as LED");
            uint8_t packet[3];
            packet[0] = (LED1ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            hostInterface.write((byte*)packet,3);
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
            debug.println("LED2 is not configured as LED");
            uint8_t packet[3];
            packet[0] = (LED2ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            hostInterface.write((byte*)packet,3);
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
            debug.println("LED3 is not configured as LED");
            uint8_t packet[3];
            packet[0] = (LED3ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            hostInterface.write((byte*)packet,3);
        }
    }
        break;
    case VERSIONID:
    {
        int dLength;
        _Host_message msg;
        msg.proto = VERSIONID;
        msg.config = 0;
        dLength = strlen(VERSION);
        debug.printf("msg.datalength : %d\n",dLength);
        if (dLength <= 255)
        {
            msg.datalength = dLength;
            memcpy(msg.data,VERSION,msg.datalength);
        }
        else
        {
            msg.datalength = 255;
            memcpy(msg.data,VERSION,msg.datalength);
            msg.datalength2 = dLength - 255;
            memcpy(msg.data,VERSION+255,msg.datalength);
        }
        hostInterface.write((byte*)&msg,dLength+3);
        break;
    }
    case RESETID:
    {   
        //This should take a code
        debug.println("Resetting CPU");
        //ToDo Check Hash Value
        _Host_message msg;
        msg.proto = RESETID;
        msg.config = 0;
        hostInterface.write((byte*)&msg,1);
        delay(1000);
        _reboot_Teensyduino_();
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
                debug.println("Sending data to USART0");
                for (int x = 0; x < dLenght; x++)
                {
                    debug.printf("%c",data[x]);
                    USART0.printf("%c",data[x]);
                }
            }
        }
        else
        {
            debug.println("USART0 is not configured as USART");
            uint8_t packet[3];
            packet[0] = (USART0ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            hostInterface.write((byte*)packet,3);
        }
        break;
    case USART1ID:
        if (usart1EN())
        {            
            if(_dID > 0x80)
            {
                //configure serial port
            }
            else
            {
                debug.println("Sending data to USART1");
                for (int x = 0; x < dLenght; x++)
                {
                    debug.printf("%c",data[x]);
                    USART1.printf("%c",data[x]);
                }
            }
        }
        else
        {
            debug.println("USART1 is not configured as USART");
            uint8_t packet[3];
            packet[0] = (USART1ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            hostInterface.write((byte*)packet,3);
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
                debug.println("Sending data to USART2");
                for (int x = 0; x < dLenght; x++)
                {
                    debug.printf("%c",data[x]);
                    USART2.printf("%c",data[x]);
                }
            }
        }
        else
        {
            debug.println("USART2 is not configured as USART");
            uint8_t packet[3];
            packet[0] = (USART2ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            hostInterface.write((byte*)packet,3);
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
                debug.println("Sending data to USART3");
                for (int x = 0; x < dLenght; x++)
                {
                    debug.printf("%c",data[x]);
                    USART3.printf("%c",data[x]);
                }
            }
        }
        else
        {
            debug.println("USART3 is not configured as USART");
            uint8_t packet[3];
            packet[0] = (USART3ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            hostInterface.write((byte*)packet,3);
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
                debug.println("Sending data to USART4");
                for (int x = 0; x < dLenght; x++)
                {
                    debug.printf("%c",data[x]);
                    USART4.printf("%c",data[x]);
                }
            }
        }
        else
        {
            debug.println("USART4 is not configured as USART");
            uint8_t packet[3];
            packet[0] = (USART4ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            hostInterface.write((byte*)packet,3);
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
                debug.println("Sending data to USART5");
                for (int x = 0; x < dLenght; x++)
                {
                    debug.printf("%c",data[x]);
                    USART5.printf("%c",data[x]);
                }
            }
        }
        else
        {
            debug.println("USART5 is not configured as USART");
            uint8_t packet[3];
            packet[0] = (USART5ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            hostInterface.write((byte*)packet,3);
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
                debug.println("Sending data to USART6");
                for (int x = 0; x < dLenght; x++)
                {
                    debug.printf("%c",data[x]);
                    USART6.printf("%c",data[x]);
                }
            }
        }
        else
        {
            debug.println("USART6 is not configured as USART");
            uint8_t packet[3];
            packet[0] = (USART6ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            hostInterface.write((byte*)packet,3);
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
                debug.println("Sending data to USART7");
                for (int x = 0; x < dLenght; x++)
                {
                    debug.printf("%c",data[x]);
                    USART7.printf("%c",data[x]);
                }
            }
        }
        else
        {
            debug.println("USART7 is not configured as USART");
            uint8_t packet[3];
            packet[0] = (USART7ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            hostInterface.write((byte*)packet,3);
        }
        break;
    case I2C0ID:
        {
            uint8_t bus = 0;
            if (i2c0EN())
            {
                uint8_t* devices;
                uint8_t dcount = busScan(bus ,devices);
                debug.printf("%d Found on I2C Bus %d\n", dcount, bus);
                for(int x =0; x < dcount; x++)
                {
                    debug.printf("::0X%X:",devices[dcount]);
                }
                debug.println(" ");
            }
            else
            {
                debug.printf("I2C%d is not enabled",bus);
                uint8_t packet[3];
                packet[0] = (I2C0ID+0x80);
                packet[1] = 0;
                packet[2] = 0;
                hostInterface.write((byte*)packet,3);
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
            debug.printf("%d Found on I2C Bus %d\n", dcount, bus);
            for(int x =0; x < dcount; x++)
            {
                debug.printf("::0X%X:",devices[dcount]);
            }
            debug.println(" ");
        }
        else
        {
            debug.printf("I2C%d is not enabled",bus);
            uint8_t packet[3];
            packet[0] = (I2C0ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            hostInterface.write((byte*)packet,3);
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
            //debug.printf("Devices Adress == 0x%x\n", *devices);
            debug.printf("%d Found on I2C Bus %d\n", dcount, bus);
            for(int x =0; x < dcount; x++)
            {
                debug.printf("::0X%X:",devices[dcount]);
            }
            debug.println(" ");
        }
        else
        {
            debug.printf("I2C%d is not enabled",bus);
            uint8_t packet[3];
            packet[0] = (I2C0ID+0x80);
            packet[1] = 0;
            packet[2] = 0;
            hostInterface.write((byte*)packet,3);
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
        analog[dID - ADC0ID].Read();
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
    case PWM19ID:
    case PWM1AID:
    case PWM1BID:{
        uint16_t lvalue = 0;
        lvalue = (data[0] << 8) + data[1];
        pwm[dID - PWM0ID].Write(lvalue);
    }
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

