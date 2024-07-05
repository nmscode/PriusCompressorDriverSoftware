#include <Arduino.h>
#include "pinout.h"
#include "SimpleFOC.h"

HFIBLDCMotor motor = HFIBLDCMotor(3,0.35,42.0f,2.2e-3f);

#if defined(STM32F4xx) || defined(STM32F7xx)
BLDCDriver6PWM driver = BLDCDriver6PWM(U_H,U_L,W_H,W_L,V_H,V_L);
LowsideCurrentSense currentsense = LowsideCurrentSense(0.01f, 20.0f, CS_A, CS_B);
#endif

#ifdef ARDUINO_B_G431B_ESC1
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentsense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
#endif

#ifdef ESP32
#include <soc/adc_periph.h>
#include <driver/adc.h>
// BLDCDriver3PWM driver = BLDCDriver3PWM(33, 32, 25); // c-> b1
// LowsideCurrentSense currentsense = LowsideCurrentSense(0.01f, 50.0f, 39, 36);
BLDCDriver6PWM driver = BLDCDriver6PWM(33, 32, 14, 27, 26, 25);
LowsideCurrentSense currentsense = LowsideCurrentSense(0.010f, 20.00f,39,36);
#endif

Commander command = Commander(Serial);
void onMotor(char* cmd){command.motor(&motor,cmd);}

void process_hfi(){motor.process_hfi();}



float target_vel_ramp = 0;
float target_velocity = 0;
float rampRate=20.0;
float driver_temp=0;
bool ocp_tripped=false;
static float temp_array[23]={134.59,106.61,85.00,68.20,55.06,44.71,36.51,29.97,24.74,20.52,17.10,14.32,12.04,10.17,8.63,7.35,6.28,5.39,4.64,4.01,3.48,3.02,2.64};
int cnt=0;
void doTarget(char* cmd) {
  command.scalar(&target_velocity, cmd);
}
void setup() {
  #ifdef ESP32
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  //pinMode(2,OUTPUT);
  #else
  pinMode(PC10,OUTPUT);
  Serial.begin(115200);
	#endif
  SimpleFOCDebug::enable(&Serial);
  
  

  driver.voltage_power_supply = 200.0f;
  driver.pwm_frequency = 10000;
  driver.voltage_limit = driver.voltage_power_supply;
  driver.dead_zone=0.04;
  driver.init();

  while (!Serial.available()) {}

  command.add('M',&onMotor,"motor");
  command.add('T', doTarget, "target velocity");
  motor.linkDriver(&driver);

  motor.linkCurrentSense(&currentsense);
  currentsense.linkDriver(&driver);
  // don't skip current sense align with sfoc shield
  // currentsense.skip_align = true;
  currentsense.init();
  motor.voltage_sensor_align = 1.0+8.0;//Add 8V to compensate deadtime
  motor.deadtime_compensation=4.0; //Compensate 4V on each side
  motor.current_limit = 5.5;
  motor.P_angle.P = 3.0f;
  motor.P_angle.I = 1.0f;
  motor.P_angle.D = 0.1f;
  motor.P_angle.output_ramp = 0;
  motor.LPF_angle.Tf =  1/(25*_2PI);

  motor.PID_velocity.P = 0.1f;
  motor.PID_velocity.I = 0.5f;
  motor.PID_velocity.D = 0.0005f;
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 1/(25*_2PI); 
  

  motor.LPF_current_d.Tf = 1/(2000*_2PI);
  motor.LPF_current_q.Tf = 1/(2000*_2PI);
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;
  //motor.controller = MotionControlType::velocity_openloop;

  // 2804 140kv
  // motor.Ld = 2200e-6f;
  // motor.Lq = 3100e-6f;
  
  //5208 80kv
  // motor.Ld = 4000e-6f;
  // motor.Lq = 6500e-6f;

  //3548 790kv
  motor.phase_resistance=0.35;
  motor.Ld = 1.400e-3f;
  motor.Lq = 3.000e-3f;
  motor.error_saturation_limit=0.3;
  motor.hfi_v = 35.0;
  motor.polarity_alignment_voltage=3.0;
  motor.bemf_threshold=20.0;
  motor.fo_hysteresis_threshold=500;
  motor.voltage_limit=90;

  motor.init();
  motor.initFOC();

  Serial.println(motor.enabled);
  motor.hfi_on = true;
  motor.sensor_direction = Direction::CCW;
  motor.current_setpoint.d = 0.00f;
  motor.ocp_protection_limit=7.0;
  motor.ocp_protection_maxcycles=4;
  Serial.println(motor.enabled);
  delay(1000);
  motor.start_polarity_alignment=true;
  delay(1000);
  Serial.println(motor.enabled);
}

uint32_t time_prev=0;
void loop() {

  motor.move(target_vel_ramp);
  if(!abs(target_velocity-target_vel_ramp)<0.001){

  if(target_velocity>target_vel_ramp){
    target_vel_ramp+=(rampRate/1000000)*(micros() - time_prev);
  }
  if(target_velocity<target_vel_ramp){
    target_vel_ramp-=(rampRate/1000000)*(micros() - time_prev);
  }
  }
  time_prev=micros();
  if(cnt>=1000 && !ocp_tripped){
    driver_temp=_readADCVoltageLowSide(34,currentsense.params);
    driver_temp=(driver_temp*10.0)/(3.3-driver_temp);
    int i=0;
    for(int x=0;x<23 && driver_temp>=temp_array[x];x++){
      i=x;
    }
    driver_temp=map(driver_temp,temp_array[i],temp_array[i+1],i*5+15.0,(i+1)*5+15.0);
    if(motor.enabled){
      Serial.printf("%f,%f\n",motor.sensorless_velocity,driver_temp);
    }
    else{
      Serial.println("OCP TRIPPED");
      ocp_tripped=true;
    }
    cnt=0;
  }
  cnt+=1;
  command.run();

}
