/**
 * Author Teemu Mäntykallio and Kushagra Keshari
 * Initializes the library and runs the stepper motor with AccelStepper for testing sensorless homing.
 */

#include <TMCStepper.h>

#define EN_PIN           4 // Enable
#define DIR_PIN          3 // Direction
#define STEP_PIN         2 // Step
#define STALL_PIN_X      5 // Connected to DIAG pin on the TMC2209

#define ENB_PIN           8 // Enable
#define DIRB_PIN          7 // Direction
#define STEPB_PIN         6 // Step
#define STALLB_PIN_X      9
//#define CS_PIN           42 // Chip select
//#define SW_MOSI          66 // Software Master Out Slave In (MOSI)
//#define SW_MISO          44 // Software Master In Slave Out (MISO)
//#define SW_SCK           64 // Software Slave Clock (SCK)
//#define SW_RX            63 // TMC2208/TMC2224 SoftwareSerial receive pin
//#define SW_TX            40 // TMC2208/TMC2224 SoftwareSerial transmit pin

int aMin = 50;
int aMax = 9500;
int bMin = 50;
int bMax = 8000;

#define SERIAL_PORT Serial1 // TMC2209 HardwareSerial port

//The numbers after 'b' are determined by the state of pins MS2 and MS1 pins of
//TMC2209 respectively. 1->VCC 2->GND
//Both the driver's UART pins are connected to the same UART pins of the microcontroller.
//The distinct addresses are used to indentify each and control the parameters individually.
#define driverA_ADDRESS 0b00 //Pins MS1 and MS2 connected to GND.
#define driverB_ADDRESS 0b01// Pin MS1 connected to VCC and MS2 connected to GND.

//Stallguard values for each driver(0-255), higher number -> higher sensitivity.
#define STALLA_VALUE 25
#define STALLB_VALUE 23

#define RA_SENSE 0.11f // Sense resistor value, match to your driverA
#define RB_SENSE 0.11f


TMC2209Stepper driverA(&SERIAL_PORT, RA_SENSE, driverA_ADDRESS);
TMC2209Stepper driverB(&SERIAL_PORT, RB_SENSE, driverB_ADDRESS);

constexpr uint32_t steps_per_round = 200*(60/16);//Claculated for the belt and pulley system.

#include <AccelStepper.h>
AccelStepper stepperA = AccelStepper(stepperA.DRIVER, STEP_PIN, DIR_PIN);
AccelStepper stepperB = AccelStepper(stepperB.DRIVER, STEPB_PIN, DIRB_PIN);


bool startup = true; // set false after homing
bool stalled_A = false;
bool stalled_B = false;

void stallInterruptX(){ // flag set for motor A when motor stalls
stalled_A = true;
}

void stallInterruptB(){ // flag set for motor B when motor stalls
stalled_B = true;
}

void motorBHome()
{
  stepperB.move(-100*steps_per_round);
  while(1)
    {
      stepperB.run();

      if(stalled_B){
        stepperB.setCurrentPosition(0);
        break; 
      }
      
    }

    digitalWrite(LED_BUILTIN, HIGH);

    delay(250);

    digitalWrite(LED_BUILTIN, LOW);

    stepperB.setMaxSpeed(10*steps_per_round);
    stepperB.setAcceleration(50*steps_per_round); // 2000mm/s^2
    stepperB.moveTo(1250);

    while(stepperB.distanceToGo())
    stepperB.run();
}

void motorAHome()
{
  stepperA.move(-100*steps_per_round); // Move 100mm
  while(1)
    {
      stepperA.run();

      if(stalled_A){
        stepperA.setCurrentPosition(0);
        break; 
      }
      
    }

    digitalWrite(LED_BUILTIN, HIGH);

    delay(250);

    digitalWrite(LED_BUILTIN, LOW);

    stepperA.setMaxSpeed(10*steps_per_round);
    stepperA.setAcceleration(50*steps_per_round); // 2000mm/s^2
    stepperA.moveTo(2000);

    while(stepperA.distanceToGo())
    stepperA.run();
}


void setup() {
    
    //Serial.begin(9600);
    //while(!Serial);
    //Serial.println("Start...");
    
    SERIAL_PORT.begin(115200);      // HW UART driverAs

    pinMode(LED_BUILTIN, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(STALL_PIN_X), stallInterruptX, RISING);
    attachInterrupt(digitalPinToInterrupt(STALLB_PIN_X), stallInterruptB, RISING);
    
    driverA.begin();             // Initiate pins and registeries
    driverA.rms_current(700);    // Set stepperA current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    //driverA.en_pwm_mode(1);      // Enable extremely quiet stepping
    driverA.pwm_autoscale(1);
    driverA.microsteps(16);
    driverA.TCOOLTHRS(0xFFFFF); // 20bit max
    driverA.SGTHRS(STALLA_VALUE);

    driverB.begin();             // Initiate pins and registeries
    driverB.rms_current(700);    // Set stepperA current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    //driverA.en_pwm_mode(1);      // Enable extremely quiet stepping
    driverB.pwm_autoscale(1);
    driverB.microsteps(16);
    driverB.TCOOLTHRS(0xFFFFF); // 20bit max
    driverB.SGTHRS(STALLB_VALUE);

    stepperA.setMaxSpeed(1.25*steps_per_round); // 100mm/s @ 80 steps/mm
    stepperA.setAcceleration(10*steps_per_round); // 2000mm/s^2
    stepperA.setEnablePin(EN_PIN);
    stepperA.setPinsInverted(false, false, true);
    stepperA.enableOutputs();

    stepperB.setMaxSpeed(1.25*steps_per_round); // 100mm/s @ 80 steps/mm
    stepperB.setAcceleration(10*steps_per_round); // 2000mm/s^2
    stepperB.setEnablePin(ENB_PIN);
    stepperB.setPinsInverted(false, false, true);
    stepperB.enableOutputs();

    
    motorAHome();
    motorBHome();
    
    //stepperB.moveTo(2000);
    
}

void loop() {

  
    /*
    if(Serial.available()>0){
      char readVal = Serial.read();
      if (readVal == 'a'){
      int steps = Serial.parseInt();
      stepperA.moveTo(steps);
      
      } else if (readVal == 'A'){
      motorAHome();
      }

      if (readVal == 'b'){
      int steps = Serial.parseInt();
      stepperB.moveTo(steps);
      
      } else if (readVal == 'B'){
      motorBHome();
      }
    }
    */

  

  stepperA.run();
  stepperB.run();
    
}
