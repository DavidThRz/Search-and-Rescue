#include <MotorWheel.h> 
#include <Omni3WD.h> 
#include <Omni4WD.h> 
#include <PID_Beta6.h> 
#include <PinChangeInt.h> 
#include <PinChangeIntConfig.h>
//#include "sml_nexus_common.h"
 // Include the header files 
 /*  
 \ /  wheel1 \ / wheel4  
 Left \ / Right      
 / \  wheel2 / \ wheel3  
 Right / \ Left 
 */ 
irqISR(irq1,isr1);   // Intterrupt function.on the basis of the pulse,work for wheel1 
 MotorWheel wheel1(11,12,5,4,&irq1); //UL
 irqISR(irq2,isr2); 
 MotorWheel wheel2(3,2,15,14,&irq2); //3,2,14,15
 irqISR(irq3,isr3); 
 MotorWheel wheel3(10,7,17,16,&irq3); //10,7,18,19
 irqISR(irq4,isr4); 
 MotorWheel wheel4(9,8,19,18,&irq4); //9,8,16,17
 Omni4WD Omni(&wheel1,&wheel2,&wheel3,&wheel4); // This will create a Omni4WD object called Omni4WD. //You can then use any of its methods; for instance,   // to control a Omni4WD attached to pins, you could write   

 void setup() { 

  Serial.begin(115200);
  
  String r;
  String substring;

  while (!Serial);  
  //TCCR0B=TCCR0B&0xf8|0x01; 
  // warning!! it will change millis()   
  TCCR1B=TCCR1B&0xf8|0x01; // Pin9,Pin10 PWM 31250Hz   
  TCCR2B=TCCR2B&0xf8|0x01; // Pin3,Pin11 PWM 31250Hz  
  Omni.PIDEnable(0.31,0.01,0,10); // Enable PID  
  } 
  void loop() {

 // ALBERTO - TEST IDENTIFICACIÓN DE LAS RUEDAS
/*
wheel1.setSpeedRPM(4000);
Omni.delayMS(5000);

wheel1.setSpeedRPM(0);
Omni.delayMS(5000);

wheel1.setSpeedRPM(-4000);
Omni.delayMS(5000);

wheel1.setSpeedRPM(0);
Omni.delayMS(5000);

wheel2.setSpeedRPM(4000);
Omni.delayMS(5000);

wheel2.setSpeedRPM(0);
Omni.delayMS(5000);

wheel2.setSpeedRPM(-4000);
Omni.delayMS(5000);

wheel2.setSpeedRPM(0);
Omni.delayMS(5000);

wheel3.setSpeedRPM(4000);
Omni.delayMS(5000);

wheel3.setSpeedRPM(0);
Omni.delayMS(5000);

wheel3.setSpeedRPM(-4000);
Omni.delayMS(5000);

wheel3.setSpeedRPM(0);
Omni.delayMS(5000);

wheel4.setSpeedRPM(4000);
Omni.delayMS(5000);

wheel4.setSpeedRPM(0);
Omni.delayMS(5000);

wheel4.setSpeedRPM(-4000);
Omni.delayMS(5000);

wheel4.setSpeedRPM(0);
Omni.delayMS(5000);

// ALBERTO - TEST IDENTIFICACIÓN DE LAS REVOLUCIONES

/*
wheel1.setSpeedRPM(500);
Omni.delayMS(4000);
Serial.println(wheel1.getPWM());

wheel1.setSpeedRPM(1000);
Omni.delayMS(4000);
Serial.println(wheel1.getPWM());

wheel1.setSpeedRPM(1500);
Omni.delayMS(4000);
Serial.println(wheel1.getPWM());
*/

// controle clavier

 if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n');
    Serial.println("Received command: " + message);
   if (message == "R1") {
        wheel1.setSpeedRPM(1000);
        Omni.delayMS(1000);
    } else if (message == "R2") {
        wheel2.setSpeedRPM(1000);
        Omni.delayMS(1000);
    } else if (message == "R3") {
        wheel3.setSpeedRPM(1000);
        Omni.delayMS(1000);
    } else if (message == "R4") {
        wheel4.setSpeedRPM(1000);
        Omni.delayMS(1000);    
    } else {
        wheel1.setSpeedRPM(0);
        wheel2.setSpeedRPM(0);
        wheel3.setSpeedRPM(0);
        wheel4.setSpeedRPM(0);                       
        Omni.delayMS(4000);
    }
 }
}