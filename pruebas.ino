#include <ros.h>
#include <std_msgs/UInt8.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <EEPROM.h>
#define _NAMIKI_MOTOR	 //for Namiki 22CL-103501PG80:1
#include <fuzzy_table.h>
#include <PID_Beta6.h>
#include <MotorWheel.h>
#include <Omni4WD.h>



ros::NodeHandle node_handle;

std_msgs::UInt8 movement_msg;

/*

            \                    /
   wheel1   \                    /   wheel4
   Left     \                    /   Right
    
    
                              power switch
    
            /                    \
   wheel2   /                    \   wheel3
   Right    /                    \   Left

 */

/*
irqISR(irq1,isr1);
MotorWheel wheel1(5,4,12,13,&irq1);

irqISR(irq2,isr2);
MotorWheel wheel2(6,7,14,15,&irq2);

irqISR(irq3,isr3);
MotorWheel wheel3(9,8,16,17,&irq3);

irqISR(irq4,isr4);
MotorWheel wheel4(10,11,18,19,&irq4);
 */

irqISR(irq1,isr1);
MotorWheel wheel1(3,2,4,5,&irq1);

irqISR(irq2,isr2);
MotorWheel wheel2(11,12,14,15,&irq2);

irqISR(irq3,isr3);
MotorWheel wheel3(9,8,16,17,&irq3);

irqISR(irq4,isr4);
MotorWheel wheel4(10,7,18,19,&irq4);


Omni4WD Omni(&wheel1,&wheel2,&wheel3,&wheel4);


void subscriberCallback(const std_msgs::UInt8& movement_msg) {
  if(movement_msg.data == 1)
  {
    Omni.setCarAdvance(0);
    Omni.setCarSpeedMMPS(200, 500);
    Omni.delayMS(5000);
    Omni.setCarSlow2Stop(500);
  }
  else if(movement_msg.data == 2)
    Omni.setCarSpeedMMPS(0, 500);
}

//ros::Publisher button_publisher("button_press", &button_msg);
ros::Subscriber<std_msgs::UInt8> move_subscriber("movement", &subscriberCallback);

void setup() {
	//TCCR0B=TCCR0B&0xf8|0x01;    // warning!! it will change millis()
	TCCR1B=TCCR1B&0xf8|0x01;    // Pin9,Pin10 PWM 31250Hz
	TCCR2B=TCCR2B&0xf8|0x01;    // Pin3,Pin11 PWM 31250Hz
    
	Omni.PIDEnable(0.31,0.01,0,10);

  //Serial.begin(57600);
  node_handle.getHardware()->setBaud(57600);
  node_handle.initNode();
  //node_handle.advertise(button_publisher);
  node_handle.subscribe(move_subscriber);
    
}

void loop() {
	//Omni.demoActions(30,1500,500,false);

  //button_publisher.publish( &button_msg );
  node_handle.spinOnce();
    
  delay(50);
}