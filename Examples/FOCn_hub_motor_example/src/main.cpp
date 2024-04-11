#include <Arduino.h>
#include "SimpleFOC.h"

#define usr_led_pin 46
#define aux1_pin 10
#define pullup_en_pin 48

// ########## set motor parameters here ##########
// power supply voltage
#define drv_power_sply 15.0f
// limits max voltage at the motor (set to input voltage if not needed)
#define drv_driver_volt_limit 8.0f
// voltage used for hall sensor alignment. Start low and increase until align is reliable
#define drv_align_voltage 2.0f
#define drv_polePairs 15
// ########## end motor parameters #########


#define drv_pwmA_pin 12
#define drv_pwmB_pin 13
#define drv_pwmC_pin 14
#define drv_enable_pin 41
#define drv_snsA_pin 4
#define drv_snsB_pin 6
#define drv_snsC_pin 5
#define drv_hallA_pin 38
#define drv_hallB_pin 37
#define drv_hallC_pin 36
#define drv_cal_pin 40
#define drv_shunt_res 0.003f
#define drv_shunt_gain (-20.0f)

#define hvpwm_pin 35
#define lvpwm_pin 47


// ########## set max motor target value here ##########
#define drv_target_max 15.0f




//simplefoc motor
static BLDCMotor drv_motor = BLDCMotor(drv_polePairs);
//simplefoc driver
static BLDCDriver3PWM drv_driver = BLDCDriver3PWM(drv_pwmA_pin, drv_pwmB_pin, drv_pwmC_pin, drv_enable_pin);
//simplefoc hall sensor
static HallSensor drv_sensor = HallSensor(drv_hallA_pin, drv_hallB_pin, drv_hallC_pin, drv_polePairs);
//simplefoc current sensor
static LowsideCurrentSense drv_current_sense = LowsideCurrentSense(drv_shunt_res, drv_shunt_gain, drv_snsA_pin, drv_snsB_pin, drv_snsC_pin);

// HALL interrupt routine initialization
void drv_int_hall_A(){drv_sensor.handleA();}
void drv_int_hall_B(){drv_sensor.handleB();}
void drv_int_hall_C(){drv_sensor.handleC();}



// RC PWM stuff ---------------------
volatile uint32_t PwmRiseTime = 0;
volatile uint32_t PwmTimeNow = 0;
volatile uint32_t PwmPulseTime = 0;
volatile uint32_t PwmLastEdge = 0;
volatile bool PwmValid = false;
// map RC PWM pulse time to motor target value
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
float rcPwmToMotorTarget(float rcPwm){
  return mapFloat(rcPwm, 1100.0, 1800.0, -drv_target_max, drv_target_max);
}

// PWM interrupt
void IRAM_ATTR pwmInInterrupt() {
  PwmTimeNow = micros();
  PwmLastEdge = PwmTimeNow;
  if(digitalRead(aux1_pin) == HIGH){
    //rising edge
    PwmRiseTime = PwmTimeNow;
  }
  else{
    //falling edge
    PwmPulseTime = PwmTimeNow - PwmRiseTime;
    if(PwmPulseTime > 1000 && PwmPulseTime < 2000){
      PwmValid = true;
    }
    else{
      PwmValid = false;
    }
  }
}
// End RC PWM stuff ---------------------


void setup() {
  Serial.begin(115200);
  delay(4000);
  Serial.println("Hello FOCn boot ok!!");
  // put your setup code here, to run once:
  pinMode(usr_led_pin, OUTPUT);
  pinMode(drv_cal_pin, OUTPUT);
  pinMode(pullup_en_pin, OUTPUT);


  //enable hall pullups
  digitalWrite(pullup_en_pin, LOW);

  // gate driver can calibrate shunt amp offsets. If needed do this after every gate driver enable:
  // digitalWrite(drv_cal_pin, HIGH);
  // delay(2); //calibration takes 100us as per datasheet
  // digitalWrite(drv_cal_pin, LOW);
  // delay(2); //settle back to normal amplification

  // if analogRead is not called on ADC pins, ESP crashes when SimpleFOC starts sampling
  // This seems to be a ESP32-S3 SimpleFOC issue
  analogRead(drv_snsA_pin);
  analogRead(drv_snsB_pin);
  analogRead(drv_snsC_pin);

  //enable simplefoc debug
  SimpleFOCDebug::enable();

  // initialize hall sensor hardware
  drv_sensor.init();
  Serial.println("sensor init done!");

  //register callbacks for hall interrupts
  drv_sensor.enableInterrupts(drv_int_hall_A, drv_int_hall_B, drv_int_hall_C);

  //link sensor to motor
  drv_motor.linkSensor(&drv_sensor);

  // set power supply voltage param
  drv_driver.voltage_power_supply = drv_power_sply;
  // limit max voltage driver can set
  drv_driver.voltage_limit = drv_driver_volt_limit;

  //init driver
  drv_driver.init();
  Serial.println("driver init done!");

  //link motor and driver
  drv_motor.linkDriver(&drv_driver);

  //voltage for hall align
  drv_motor.voltage_sensor_align = drv_align_voltage;

  //select FOC modulation
  drv_motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  //link current sense to driver
  drv_current_sense.linkDriver(&drv_driver);

  // ########## SELECT MOTION CONTROLLER HERE ##########
  // voltage is most reliable - it does not depend on current sensing
  // if set to voltage, target is voltage [V] applied to the motor
  // if set to dc/foc current, target is phase current [A] (DC or FOC q component)
  // ###
  // drv_motor.torque_controller = TorqueControlType::voltage;
  // drv_motor.torque_controller = TorqueControlType::dc_current;
  drv_motor.torque_controller = TorqueControlType::foc_current;
  // ###

  //select motion controller
  drv_motor.controller = MotionControlType::torque;

  //init motor
  drv_motor.init();
  Serial.println("M: motor init done!");

  // init current sense
  if (drv_current_sense.init())  Serial.println("Current sense init success!");
  else{
      Serial.println("Current sense init failed!");
      while(1){
        //stop here if failed
        delay(100);
      }
  }

  //link current sense to motor
  drv_motor.linkCurrentSense(&drv_current_sense);

  // ########## select if you want to skip current sense alignment ##########
  // pins and gains need to be set correctly to do this. For basic FOCn you can skip this,
  // ### 
  drv_current_sense.skip_align = true;
  // ###

  // ########## select if you want to skip hall sensor alignment ##########
  // direction and offset angle needs to be set correctly to do this. Use automatic alignment when starting to get values.
  // ###
  // select one direction
  // drv_motor.sensor_direction = Direction::CW;
  // drv_motor.sensor_direction = Direction::CCW;
  // set angle offset (in RAD)
  // drv_motor.zero_electric_angle = 4.20;
  // ###

  //init FOC
  drv_motor.initFOC();
  Serial.println("FOC init done!");

  // uncomment to print hall alignment params
  // Serial.println("automatic hall align got data:");
  // Serial.printf("Hall calib: zero: %f, dir: %d \n", drv_motor.zero_electric_angle, drv_motor.sensor_direction);
  // Serial.println("help: dir vals: 1:CW  -1:CCW  0:UNKNOWN");


  // ##### aditional hardware shunt amplifier calibration #####
  // gate driver can calibrate shunt amp offsets. If needed do this after every gate driver enable:
  // digitalWrite(drv_cal_pin, HIGH);
  // delay(2); //calibration takes 100us as per datasheet
  // digitalWrite(drv_cal_pin, LOW);
  // delay(2); //settle back to normal amplification




  // attach interrupt for RC PWM input
  attachInterrupt(digitalPinToInterrupt(aux1_pin), pwmInInterrupt, CHANGE);

  //enable driver
  drv_driver.enable();




}

void loop() {
  //timing variables for periodic events
  static uint32_t last_t = millis();
  static uint32_t last_t2 = millis();


  // SimpleFOC FOC and move handlers run in loop
  drv_motor.loopFOC();
  drv_motor.move();



  // Every 50ms event
  if(millis() - last_t2 > 50){
    last_t2 = millis();

    // parse RC PWM input and set motor target
    if(micros() - PwmLastEdge > 200000){
      Serial.println("No PWM signal");
      PwmValid = false;
    }
    else{
      // Serial.printf("PWM signal: %d us\n", PwmPulseTime);
    }


    //bldc ctrl
    if(PwmValid){
      if(PwmPulseTime > 1435 && PwmPulseTime < 1465){
        drv_motor.target = 0;
      }
      else{
        drv_motor.target = rcPwmToMotorTarget(PwmPulseTime);
        Serial.println("set to " + String(drv_motor.target));
      }
      // BLDC.set_pwm(rcPwmToMotorPwm(PwmPulseTime));
      // Serial.printf("BLDC set to %d\n", rcPwmToMotorPwm(PwmPulseTime));
    }
    else{
      drv_motor.target = 0;
    }


  }

  // Every 1s event
  if(millis() - last_t > 1000){
    last_t = millis();

    // print phase current and velocity
    Serial.println("phase current: " + String(drv_current_sense.getDCCurrent()));
    Serial.println("velocity: " + String(drv_motor.shaftVelocity()));
  }





}

