#include "mbed.h"
#include "nRF24L01.h"
#include "RF24.h"
#include "RF24_config.h"
#include "VL53L0X.h"
#include "PID_Gain.h"

PwmOut PWM1(D9);
PwmOut PWM2(D1);
PwmOut PWM3(D0);
PwmOut PWM4(D10);


I2C i2c(PB_4,PA_7); //(I2C_SDA,I2C_SCL); //sda, scl I2C_SDA,I2C_SCL D4, D5 A4, A5 for MPU9250
RF24 NRF24L01(PA_12, PA_6, PA_5, PA_4, PA_0); // NRF24L01
VL53L0X vl53l0x(D12, A6); // Distance Sensor

Serial pc(USBTX, USBRX); //D1D0

DigitalOut led1(D11);
DigitalOut led_switch(LED4);
DigitalIn switch_on(PB_1,PullDown);
Ticker loops;

Timer timer1, timer2;   
//-----------------------------------------------------------------------------------------
//mpu9250 register      
int   MPU9250_ADDRESS                    = (0x68<<1), //0b11010000
      WHO_AM_I_MPU9250                   = 0x75,
      FIFO_EN                            = 0x23,
      SMPLRT_DIV                         = 0x19,
      CONFIG                             = 0x1A,
      GYRO_CONFIG                        = 0x1B,
      ACCEL_CONFIG                       = 0x1C,
      ACCEL_CONFIG2                      = 0x1D,     
      ACCEL_XOUT_H                       = 0x3B,
      TEMP_OUT_H                         = 0x41,
      GYRO_XOUT_H                        = 0x43,
      
      PWR_MGMT_1                         = 0x6B,
      PWR_MGMT_2                         = 0x6C,  
      INT_PIN_CFG                        = 0x37,     
      
      AK8963_ADDRESS                     = (0x0C<<1), //0b00001100 0x0C<<1
      WHO_AM_I_AK8963                    = 0x00; // should return 0x48
     
      
float gyro_bias[3]={0.0,0.0,0.0};
//------------------------------------------------------------------------------------------------
//RF
const uint64_t pipe = 0x1212121212LL;
int8_t recv[30];
int16_t PITCH = 0, ROLL = 0, YAW = 0, THROTTLE = 0;
int8_t ackData[30]; 
int8_t flip1 = 1;   
int rcCommand[3];
int errorAngle[3];
//------------------------------------------------------------------------------------------------
//VL53L0X
uint16_t distance_mm;
int8_t distance_data[4];

float error_alt, initial_alt, d_alt, error_integral, initial_throttle;
float prev_height_cm, height_cm_new;
float alt_motor_val = 0;

float prev_alt_motor_val;
int16_t loop_count4_max = 100;
int16_t error_alt2;
float prev_acc_z, acc_z_gain = 30.0 * 9.80665 / 4096.0f, acc_z_d_gain = 2.0 * 9.80665 / 4096.0f;
//------------------------------------------------------------------------------------------------
// ETC
float roll_controller, pitch_controller, yaw_controller = 0.0;
char BUT1, BUT2, prev_BUT2;
int rf_fail_count = 0;

int constrain(int16_t x, int min, int max)
{
    if (x > max) x = max;
    else if (x < min) x = min;
    
    return x;
}

//------------------------------------------------------------------------------------------------

//taken from MPU9250 library    
int l_count=0;
int prev_throttle = 0;
int time_r = 0;

float dt = 0.0025;
float accel_f[3],gyro_f[3];
float gyro_angle[3];
float angle_accel[3];
float roll_f, roll_g_n, roll_g_old, roll_a_n, roll_a_old;
float pitch_f, pitch_g_n, pitch_g_old, pitch_a_n, pitch_a_old;
float yaw_f, yaw_g_n, yaw_g_old, yaw_a_n, yaw_a_old;
float tau=1.0;
float roll_cont = 0.0, pitch_cont = 0.0, yaw_cont = 0.0;
float roll_c = 0.0, pitch_c = 0.0, yaw_c = 0.0, thrust_c = 0.0;
float roll_err, roll_rate_err;
float roll_ref = 0.0, roll_rate_ref = 0.0;
float pitch_err, pitch_rate_err;
float pitch_ref = 0.0, pitch_rate_ref = 0.0;
float yaw_err, yaw_rate_err;
float yaw_ref = 0.0, yaw_rate_ref = 0.0;
 
float thrust_in = 0.5;

float pre_pwm1_pw, pre_pwm2_pw, pre_pwm3_pw, pre_pwm4_pw;
float pwm1_pw, pwm2_pw, pwm3_pw, pwm4_pw;
float ROLL_ERR_MAX = 50.0, ROLL_RATE_ERR_MAX = 50.0; // maximum errors considered
float PITCH_ERR_MAX = 50.0, PITCH_RATE_ERR_MAX = 50.0;
float YAW_ERR_MAX = 50.0, YAW_RATE_ERR_MAX = 50.0;
int16_t gyro[3];

void RF_READ(void);
void control(void);
void pwm_drive(void);
void control_loop(void);
void control_loop2(void);
void WHO_AM_I(void);
void MPU9250_INIT(void);
void MPU9250_RESET(void);
void MPU9250_GET_GYRO(int16_t * destination);
void MPU9250_GET_ACCEL(int16_t * destination);
void gyro_bias_f(void);
void VL53L0X_READ(void);
void altitude_control(void);
    

int main() 
{
    
    PWM1.period(0.0002);// 5 kHz PWM for PWM1~PWM4
    i2c.frequency(400000);
    pc.baud(115200);
  
    
    //RF
    NRF24L01.begin();
    NRF24L01.setDataRate(RF24_2MBPS); //RF24_2MBPS
    NRF24L01.setChannel(76);
    NRF24L01.setPayloadSize(28);
    NRF24L01.setAddressWidth(5);
    NRF24L01.setRetries(2,4); //1,3 2,8 1,8
    NRF24L01.enableAckPayload();
    NRF24L01.openReadingPipe(0, pipe);
    NRF24L01.startListening();
    
    //MPU9250
    WHO_AM_I();
   
    MPU9250_INIT();
    
    gyro_angle[0] = 0.0;
    gyro_angle[1] = 0.0;
    gyro_angle[2] = 0.0;
     
    gyro_bias_f();
    pc.printf("gyro biases(deg/sec) %f %f %f \n\r", gyro_bias[0], gyro_bias[1], gyro_bias[2]); 
    wait(3);
    
   //loops.attach(&control_loop, 0.01);
    
    
    //measurement
    //find error
    //find control output
    // pwm dirve
    timer1.start();
    int count_led = 0;
    
    while(1)    
       {
        RF_READ();          
        control_loop();
      
        //pc.printf("l_count= %d \n\r",l_count);
        //wait(1.0);
        count_led = count_led + 1;
        
        if (count_led >= 100){
            led1=!led1;
            pc.printf("%ld \n\r", timer1.read_us());
            pc.printf("ROLL: %d, PITCH: %d, YAW: %d, THROTTLE: %d \n\r", ROLL, PITCH, YAW, THROTTLE);
            pc.printf("Motor1: %f, Motor2: %f, Motor3: %f, Motor4: %f \n\r", pwm1_pw, pwm2_pw, pwm3_pw, pwm4_pw);
            pc.printf("Gyro x: %f, y: %f, z: %f \n\r", gyro_angle[0], gyro_angle[1], gyro_angle[2]);
            pc.printf("l_count= %d, roll_accel= %f, roll_f= %f, roll_c= %f \n\r", l_count, angle_accel[0], roll_f, roll_c);
            pc.printf("l_count= %d, pitch_accel= %f, pitch_f= %f, pitch_c= %f \n\r", l_count, angle_accel[1], pitch_f, pitch_c);
            pc.printf("l_count= %d, yaw_accel= %f, yaw_f= %f, yaw_c= %f \n\r", l_count, angle_accel[2], yaw_f, yaw_c);
            count_led = 0;
            }
        while (timer1.read_us() < 2500){
           }
        timer1.reset(); 
        
       }    
}
    
 
    
void control_loop(void)
{    
    int16_t gyro[3];
    int16_t accel[3]; 
    float angle_accel_1[3];   
    float accel_fu[3];
    static float accel_tau = 5.0;
    static float accel_f1[3], accel_f2[3];
   //   pc.printf("YesYES\n\r");
   //   timer2.reset();
  //    timer2.start();
   // 
   
    MPU9250_GET_GYRO(gyro);
   //  timer2.stop(); 
   //pc.printf("timer2= %ld \n\r", timer2.read_us()); 
    gyro_f[0] = gyro[1]/32.8 - gyro_bias[0];  //deg/sec
    gyro_f[1] = -(gyro[0]/32.8 - gyro_bias[1]);
    gyro_f[2] = (gyro[2]/32.8 - gyro_bias[2]);
    //gyro_angle[0]=gyro_angle[0]+(float)((gyro_f[0]-gyro_bias[0]))*dt;
    gyro_angle[0] = gyro_angle[0] + (gyro_f[0])*dt;
    gyro_angle[1] = gyro_angle[1] + (gyro_f[1])*dt;
    gyro_angle[2] = gyro_angle[2] + (gyro_f[2])*dt;
    
    MPU9250_GET_ACCEL(accel);
    accel_fu[0] = accel[1]/8192.0; //%Navigation frame reference (NED) unit in G (9.8 m/sec^2)
    accel_fu[1] = accel[0]/8192.0*(1);
    accel_fu[2] = accel[2]/8192.0*(-1);
    
  
    // lowpass filtering accel data
    // low pass filter for the accel data is inserted to reduce vibration noise
    accel_f2[0] = (1 - dt/accel_tau)*accel_f1[0] + dt/accel_tau*accel_fu[0];
    accel_f2[1] = (1 - dt/accel_tau)*accel_f1[1] + dt/accel_tau*accel_fu[1];
    accel_f2[2] = (1 - dt/accel_tau)*accel_f1[2] + dt/accel_tau*accel_fu[2];
    
    accel_f1[0] = accel_f2[0];
    accel_f1[1] = accel_f2[1];
    accel_f1[2] = accel_f2[2];
    
    accel_f[0] = accel_f2[0];
    accel_f[1] = accel_f2[1];
    accel_f[2] = accel_f2[2];
 
    angle_accel_1[0] = atan(accel_f[1]/sqrt(pow(accel_f[0], 2) + pow(accel_f[2], 2)))*180.0f/3.14f;
    angle_accel_1[1] = atan(-accel_f[0]/sqrt(pow(accel_f[1], 2) + pow(accel_f[2], 2)))*180.0f/3.14f;
    
    if(angle_accel_1[0] < 85 && angle_accel_1[0] > -85) 
    {
        angle_accel[0] = angle_accel_1[0];
        }
    if(angle_accel_1[1] < 85 && angle_accel_1[1] > -85) 
    {
        angle_accel[1] = angle_accel_1[1];
        }  
     
    roll_g_n = (1 - dt/tau)*roll_g_old + dt*gyro_f[0];
    roll_a_n = (1 - dt/tau)*roll_a_old + dt/tau*angle_accel[0];
    pitch_g_n = (1 - dt/tau)*pitch_g_old + dt*gyro_f[1];
    pitch_a_n = (1 - dt/tau)*pitch_a_old + dt/tau*angle_accel[1];
    yaw_g_n = (1 - dt/tau)*yaw_g_old + dt*gyro_f[2];
    yaw_a_n = (1 - dt/tau)*yaw_a_old + dt/tau*angle_accel[2];    
     
    roll_g_old = roll_g_n;
    roll_a_old = roll_a_n;
    pitch_g_old = pitch_g_n;
    pitch_a_old = pitch_a_n;
    yaw_g_old = yaw_g_n;
    yaw_a_old = yaw_a_n;     
         
    roll_f = roll_g_old + roll_a_old;
    pitch_f = pitch_g_old + pitch_a_old; 
    yaw_f = yaw_g_old + yaw_a_old;     
   
    
    control();
    
  
    pwm_drive();
 
    l_count = l_count + 1;
 
    // ;
                             
  }
    
    
    
    
    
void control(void)
{
    /*  IMU   */  
    roll_err = roll_ref - roll_f;
    roll_rate_err = roll_rate_ref - gyro_f[0];
    if(roll_err > ROLL_ERR_MAX) roll_err = ROLL_ERR_MAX; 
    else if(roll_err < -(ROLL_ERR_MAX)) roll_err = -(ROLL_ERR_MAX);
    if(roll_rate_err > ROLL_RATE_ERR_MAX) roll_rate_err = ROLL_RATE_ERR_MAX; 
    else if(roll_rate_err < -(ROLL_RATE_ERR_MAX)) roll_rate_err = -(ROLL_RATE_ERR_MAX);
    
    pitch_err = pitch_ref - pitch_f;
    pitch_rate_err = pitch_rate_ref - gyro_f[1];
    if(pitch_err > PITCH_ERR_MAX) pitch_err = PITCH_ERR_MAX; 
    else if(pitch_err < -(PITCH_ERR_MAX)) pitch_err = -(PITCH_ERR_MAX);
    if(pitch_rate_err > PITCH_RATE_ERR_MAX) pitch_rate_err = PITCH_RATE_ERR_MAX; 
    else if(pitch_rate_err < -(PITCH_RATE_ERR_MAX)) pitch_rate_err = -(PITCH_RATE_ERR_MAX);
    
       
    yaw_err = yaw_ref - yaw_f;
    yaw_rate_err = yaw_rate_ref - gyro_f[2];
    if(yaw_err > YAW_ERR_MAX) yaw_err = YAW_ERR_MAX; 
    else if(yaw_err < -(YAW_ERR_MAX)) yaw_err = -(YAW_ERR_MAX);
    if(yaw_rate_err > YAW_RATE_ERR_MAX) yaw_rate_err = YAW_RATE_ERR_MAX; 
    else if(yaw_rate_err < -(YAW_RATE_ERR_MAX)) yaw_rate_err = -(YAW_RATE_ERR_MAX);
    
    /* Controller   */
    roll_controller = -ROLL*0.117f;         // opposite direction in controller
    pitch_controller = PITCH*0.117f;
    yaw_controller = -YAW*0.117f;           // opposite direction in controller
       
    roll_cont = K_scale*(Kp_roll*roll_err + Kd_roll*roll_rate_err) + roll_controller;          
    pitch_cont = K_scale*(Kp_pitch*pitch_err + Kd_pitch*pitch_rate_err) + pitch_controller;
    yaw_cont = K_scale*(Kp_yaw*yaw_err + Kd_yaw*yaw_rate_err) + yaw_controller;                
       
    if(roll_cont > 50.0f) roll_cont = 50.0f;
    else if (roll_cont < -50.0f) roll_cont = -50.0f;
    if(pitch_cont > 50.0f) pitch_cont = 50.0f;
    else if (pitch_cont < -50.0f) pitch_cont = -50.0f;
    if(yaw_cont > 50.0f) yaw_cont = 50.0f;
    else if (yaw_cont < -50.0f) yaw_cont = -50.0f;
       
    roll_c = roll_cont;
    pitch_c = pitch_cont;
    yaw_c = yaw_cont;
             
    //thrust_c = 50; //Thrust by controller              
    }
        
        
        
    
void pwm_drive(void)
{
    
    
    if(prev_throttle < 250) time_r = 0;
    time_r++;
    
    if (time_r >= 400) {
        time_r = 400;
        }        
    if (time_r < 400){
        if ((THROTTLE - prev_throttle) >= 10) 
            {    
                THROTTLE = prev_throttle + 10;
            }
        }
    prev_throttle = THROTTLE; 
    thrust_c = ((float)THROTTLE)*0.165;
    
    pre_pwm1_pw = +roll_c - pitch_c + yaw_c + thrust_c;
    pre_pwm2_pw = +roll_c + pitch_c - yaw_c + thrust_c;
    pre_pwm3_pw = -roll_c + pitch_c + yaw_c + thrust_c;
    pre_pwm4_pw = -roll_c - pitch_c - yaw_c + thrust_c;
    
    /*  truncation  */
    pwm1_pw = constrain(pre_pwm1_pw, 0, 200);
    pwm2_pw = constrain(pre_pwm2_pw, 0, 200);
    pwm3_pw = constrain(pre_pwm3_pw, 0, 200);
    pwm4_pw = constrain(pre_pwm4_pw, 0, 200);
    
    
     
      /*  if(!switch_on)
           {
            pwm1_pw=0.0;
            pwm2_pw=0.0;
            pwm3_pw=0.0;  
            pwm4_pw=0.0;
           }*/
           
           
    /*  Lowest Throttle */ 
    if(THROTTLE < 10) { 
        pwm1_pw = 0;
        pwm2_pw = 0;
        pwm3_pw = 0;
        pwm4_pw = 0; 
    } 
     
      
    PWM1.pulsewidth_us(pwm1_pw);
    PWM2.pulsewidth_us(pwm2_pw);
    PWM3.pulsewidth_us(pwm3_pw);
    PWM4.pulsewidth_us(pwm4_pw);
   
    
   
       
    /*PWM1.write( pwm1_pw);
      PWM2.write( pwm2_pw);
       PWM3.write( pwm3_pw);
        PWM4.write( pwm4_pw);
      */   
    
                    
   
    }
  
void MPU9250_RESET(void) 
{
  // reset device
  //writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  
    char cmd[2];
    cmd[0] = PWR_MGMT_1; //status
    cmd[1] = 0x80;
    i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
    wait(0.1);
    }

void WHO_AM_I(void)
{
  char cmd[2]; 
  cmd[0] = WHO_AM_I_MPU9250; 
  i2c.write(MPU9250_ADDRESS, cmd, 1, 1);
  i2c.read(MPU9250_ADDRESS, cmd, 1, 0);
  wait(0.1);
  uint8_t DEVICE_ID = cmd[0];
  pc.printf("IMU device id is  %d \n\r", DEVICE_ID);
  wait(1);
 /* cmd[0] = WHO_AM_I_AK8963; 
  i2c.write(AK8963_ADDRESS, cmd, 1);
  i2c.read(AK8963_ADDRESS, cmd, 0);//
  wait(0.1);
  uint8_t DEVICE_ID2 = cmd[0];
  pc.printf("MAG  id is %d \n\r", DEVICE_ID2);*/
    }


void MPU9250_INIT(void)
{  
 // Initialize MPU9250 device
 // wake up device
  //and clear sleep mode bit (6), enable all sensors 
  //wait(0.1); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  
  
 // get stable time source
  //writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01); // Auto selects the best available clock source PLL if ready, else use the Internal oscillator

 // Configure Gyro and Accelerometer
 // Disable FSYNC and set gyro bandwidth to 41 Hz, delay 5.9 ms
 // DLPF_CFG = bits 2:0 = 011; this sets gyro bandwidth to 41 Hz, delay 5.9 ms and internal sampling rate at 1 kHz
 // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
  //writeByte(MPU9250_ADDRESS, CONFIG, 0x03); //page 13 register map  
 
 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  //writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

    char cmd[3];
    cmd[0] = PWR_MGMT_1; //reset
    cmd[1] = 0x80;
    i2c.write(MPU9250_ADDRESS, cmd, 2);
    pc.printf("MPU 1 \n\r");
    wait(0.1);


    cmd[0] = PWR_MGMT_1; // Auto selects the best available clock source PLL if ready, else use the Internal oscillator
    cmd[1] = 0x01;
    i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
    pc.printf("MPU 2 \n\r");
    wait(0.1);
  
    cmd[0] = CONFIG;
    cmd[1] = 0x03;// 41Hz gyro bandwidth, 1kHz internal sampling
    i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
    pc.printf("MPU 3 \n\r");
    wait(0.1);
 
 // sample rate divisor for all sensors, 1000/(1+4)=200 Hz for Gyro 
  cmd[0] = SMPLRT_DIV; 
  cmd[1] = 0x04;//0x04 
  i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
  pc.printf("MPU 4 \n\r");
  wait(0.1);
  
 
 /*// Set gyroscope full scale range //page 14
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c;
  cmd[0] = GYRO_CONFIG; //status
  i2c.write(MPU9250_ADDRESS, cmd, 1, 1);
  i2c.read(MPU9250_ADDRESS, cmd, 1, 0);
  pc.printf("MPU 5 \n\r");
  c = cmd[0];
  */
  
  //writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 0b1110000
  //writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3] 0b00011000
  //writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c | 0b00010011); // Set scale range for the gyro
    
  cmd[0] = GYRO_CONFIG; 
  cmd[1] = 0b00010000;// Gyro full scale 1000 deg/sec; Gyro DLPF Enable
  i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
  pc.printf("MPU 6 \n\r");
  wait(0.1);
   
 // Set accelerometer configuration
 // Accel fulll sacle range +/- 4g
  cmd[0] = ACCEL_CONFIG; 
  cmd[1] = 0b00001000;// Accel 
  i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
  pc.printf("MPU 8 \n\r");
  wait(0.1);
  
 // Set accelerometer sample rate configuration (Fast sampling)
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
 
  cmd[0] = ACCEL_CONFIG2; 
  cmd[1] = 0b00001100;
  i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
  pc.printf("MPU 10 \n\r");
  wait(0.1);
  // XYZ Gyro accel enable (default)
  cmd[0] = PWR_MGMT_2; 
  cmd[1] = 0x00;
  i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
  pc.printf("MPU 11 \n\r");
  wait(0.1);
  
  //cmd[0] = FIFO_EN; 
  //cmd[1] = 0b11111000;
  //i2c.write(MPU9250_ADDRESS, cmd, 2);
  //wait(0.1);
  
 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the mbed as master
  //writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
  //writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  //page 29
//I2c bypass mode
  cmd[0] = INT_PIN_CFG; 
  cmd[1] = 0x22; //0x02  
  i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
  pc.printf("MPU 12 \n\r");
  wait(0.1);
  }
  
  

void gyro_bias_f(void)
{
    
    int16_t gyro1[3];
    pc.printf("Please keep still 5 seconds\n\r");
    for(int i=0;i<100;i++){
        timer2.reset();
        timer2.start();
      
        MPU9250_GET_GYRO(gyro1);
        timer2.stop(); 
        pc.printf("%ld \n\r", timer2.read_us()); 
        
        gyro_bias[0] = gyro_bias[0]+gyro1[0]/32.8;
        gyro_bias[1] = gyro_bias[1]+gyro1[1]/32.8;
        gyro_bias[2] = gyro_bias[2]+gyro1[2]/32.8;
        pc.printf("bias finding i= %d\n\r",i);
        }
        
    gyro_bias[0] = gyro_bias[0]/100.0f;
    gyro_bias[1] = gyro_bias[1]/100.0f; 
    gyro_bias[2] = gyro_bias[2]/100.0f;
    pc.printf("bias finding completed %\n\r");
    }


void MPU9250_GET_GYRO(int16_t * destination)
{
  //uint8_t rawData[6];  // x/y/z gyro register data stored here
  //readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  //destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
  //destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);  
  //destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]); 
  
    char cmd[6];
    cmd[0] = GYRO_XOUT_H;
    i2c.write(MPU9250_ADDRESS, cmd, 1, 1);
    i2c.read(MPU9250_ADDRESS, cmd, 6, 0);
    destination[0] = (int16_t)(((int16_t)cmd[0] << 8) | cmd[1]);
    destination[1] = (int16_t)(((int16_t)cmd[2] << 8) | cmd[3]);
    destination[2] = (int16_t)(((int16_t)cmd[4] << 8) | cmd[5]);
  
    //pc.printf("gyro_raw %d, %d, %d \n\r", destination[0], destination[1], destination[2]); 
    }
  


void MPU9250_GET_ACCEL(int16_t * destination)
{
  //uint8_t rawData[6];  // x/y/z gyro register data stored here
  //readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  //destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
  //destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);  
  //destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]); 
  
    char cmd[6];
    cmd[0] = ACCEL_XOUT_H;
    i2c.write(MPU9250_ADDRESS, cmd, 1, 1);
    i2c.read(MPU9250_ADDRESS, cmd, 6, 0);
    destination[0] = (int16_t)(((int16_t)cmd[0] << 8) | cmd[1]);
    destination[1] = (int16_t)(((int16_t)cmd[2] << 8) | cmd[3]);
    destination[2] = (int16_t)(((int16_t)cmd[4] << 8) | cmd[5]);
  
    //pc.printf("gyro_raw %d, %d, %d \n\r", destination[0], destination[1], destination[2]); 
    }

void RF_READ(){
     
     if (NRF24L01.available())   
     {
        NRF24L01.read(recv, 10); 

/*
        //PQR
        ackData[0] = ((char *)&ROLL)[0]; //roll
        ackData[1] = ((char *)&ROLL)[1];
        ackData[2] = ((char *)&PITCH)[0]; //pitch
        ackData[3] = ((char *)&PITCH)[1];
        ackData[4] = ((char *)&YAW)[0]; //yaw
        ackData[5] = ((char *)&YAW)[1];      
        //gyro
        ackData[6] = ((char *)&acc_raw[0])[0]; // acc_raw / 16384 g (+-2g)
        ackData[7] = ((char *)&acc_raw[0])[1];
        ackData[8] = ((char *)&acc_raw[1])[0];
        ackData[9] = ((char *)&acc_raw[1])[1];
        ackData[10] = ((char *)&acc_raw[2])[0];
        ackData[11] = ((char *)&acc_raw[2])[1];
        //acc
        ackData[12] = ((char *)&gyro_raw[0])[0]; // gyro_raw / 65.536f degree/s (+-500)
        ackData[13] = ((char *)&gyro_raw[0])[1];
        ackData[14] = ((char *)&gyro_raw[1])[0];
        ackData[15] = ((char *)&gyro_raw[1])[1];
        ackData[16] = ((char *)&gyro_raw[2])[0];
        ackData[17] = ((char *)&gyro_raw[2])[1];
        //mag
        ackData[18] = ((char *)&mag_raw[0])[0]; // * 0.6 uT
        ackData[19] = ((char *)&mag_raw[0])[1];
        ackData[20] = ((char *)&mag_raw[1])[0];
        ackData[21] = ((char *)&mag_raw[1])[1];
        ackData[22] = ((char *)&mag_raw[2])[0];
        ackData[23] = ((char *)&mag_raw[2])[1];
        //laser
        ackData[24] = ((char *)&distance_mm)[0]; // distance mm
        ackData[25] = ((char *)&distance_mm)[1];
        //mpu temp /100
        ackData[26] = ((char *)&mpu_temp_read2)[0]; // / 100 degree C
        ackData[27] = ((char *)&mpu_temp_read2)[1];


        NRF24L01.writeAckPayload(0, ackData, 28); 
*/        
        
        //ack_count = 0;
        //}
        
        ROLL  = *(int16_t*)(&recv[0]); //ROLL = ROLL; 
        PITCH = *(int16_t*)(&recv[2]); //flip pitch and roll
        PITCH = PITCH;
        YAW = *(int16_t*)(&recv[4]);
        THROTTLE = *(int16_t*)(&recv[6]); 
        BUT1 = recv[8];
        BUT2 = recv[9]; //should hold value here
        
        //pc.printf("%d, %d, %d, %d \n\r", PITCH, ROLL, YAW, THROTTLE); //pc.printf("%d\n\r", THROTTLE);        
        
        rf_fail_count = 0;
      }
      
      else {
          
          rf_fail_count = rf_fail_count + 1;
          
            if(rf_fail_count >= 20 && rf_fail_count < 100){
                THROTTLE = THROTTLE - 2;
                THROTTLE = constrain(THROTTLE, 0, 1023);
            }
          
                if(rf_fail_count >= 100){
                THROTTLE = 0;
                
            }
          if(rf_fail_count >= 100) {
            rf_fail_count = 100;
          }
      }
}
