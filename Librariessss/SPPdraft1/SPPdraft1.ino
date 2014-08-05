



/*
Example sketch for the RFCOMM/SPP Bluetooth library - developed by Kristian Lauszus
For more information visit my blog: http://blog.tkjelectronics.dk/ or
send me an e-mail:  kristianl@tkjelectronics.com
*/
#include <SPP.h>
#include <usbhub.h>
// Satisfy IDE, which only needs to see the include statement in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

USB Usb;
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
SPP *SerialBT;  

void setup() 
{
       SerialBT = new SPP(&Btd);  
??      Serial.begin(115200);
??      while (!Serial);
??      if (Usb.Init() == -1) 
       {
????          Serial.print(F("\r\nOSC did not start"));
????          while (1); // Halt
??      }
??      Serial.print(F("\r\nSPP Bluetooth Library Started"));
}


void loop() 
{
       int BTinput;
??      Usb.Task(); 
       if (SerialBT->connected) 
       {
??????        if (SerialBT->available())
             {
????????             BTinput = SerialBT->read();
                    Serial.write(BTinput);
                    if( BTinput == ?f? )
                        Serial.println(?forward?);
                    if( BTinput == ?b? )
                        Serial.println(?backward?);
                    if( BTinput == ?l? )
                        Serial.println(?left?);
                    if( BTinput == ?r? )
                        Serial.println(?right?);
             }
??      }
}


