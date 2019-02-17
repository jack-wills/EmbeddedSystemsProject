#include "mbed.h"

PinName echo2Pin = PC_3;
PinName trigger2Pin = PC_2;
PinName echo1Pin = PC_12;
PinName trigger1Pin = PC_10;
PinName sensor1_pin = A5;
PinName sensor2_pin = A4;
PinName sensor3_pin = A3;
PinName sensor4_pin = A2;
PinName sensor5_pin = A1;
PinName sensor6_pin = A0;
PinName sensor1power_pin = D7;
PinName sensor2power_pin = D6;
PinName sensor3power_pin = D5;
PinName sensor4power_pin = D4;
PinName sensor5power_pin = D3;
PinName sensor6power_pin = D2;
PinName PWM_PIN1 = D12;
PinName PWM_PIN2 = D9;
PinName ENABLE_PIN = D8;
PinName UNIPOLAR_PIN1 = D15;
PinName UNIPOLAR_PIN2 = D11;
PinName DIRECTION_PIN1 = D13;
PinName DIRECTION_PIN2 = D10;
PinName test1_pin = PC_5;
PinName test2_pin = PB_1;
PinName test3_pin = PC_4;

AnalogIn p_gain(test1_pin);
AnalogIn i_gain(test2_pin);
AnalogIn d_gain(test3_pin);

AnalogIn sensor1(sensor1_pin);
AnalogIn sensor2(sensor2_pin);
AnalogIn sensor3(sensor3_pin);
AnalogIn sensor4(sensor4_pin);
AnalogIn sensor5(sensor5_pin);
AnalogIn sensor6(sensor6_pin);

DigitalOut sensor1power(sensor1power_pin);
DigitalOut sensor2power(sensor2power_pin);
DigitalOut sensor3power(sensor3power_pin);
DigitalOut sensor4power(sensor4power_pin);
DigitalOut sensor5power(sensor5power_pin);
DigitalOut sensor6power(sensor6power_pin);

DigitalOut enable(ENABLE_PIN);
DigitalOut uni_left(UNIPOLAR_PIN1);
DigitalOut uni_right(UNIPOLAR_PIN2);
DigitalOut dir_left(DIRECTION_PIN1);
DigitalOut dir_right(DIRECTION_PIN2);

DigitalIn Button(USER_BUTTON);

InterruptIn sensor1_echo(echo1Pin);
DigitalOut sensor1_trigger(trigger1Pin);
InterruptIn sensor2_echo(echo2Pin);
DigitalOut sensor2_trigger(trigger2Pin);
Timer ultrasound1_timer;
Timer ultrasound2_timer;
Timeout sensor1_triggerTimeout, sensor1_echoTimeout;
Timeout sensor2_triggerTimeout, sensor2_echoTimeout;
Timeout ultrasound2_timeout;

PwmOut pwm_left(PWM_PIN1);
PwmOut pwm_right(PWM_PIN2);

Ticker ultrasound_ticker;
Timer sensor_timer;
Timer stop_timer;
Timeout stop_timeout;

Serial pc(USBTX, USBRX);

float dt;
float sensor_list[6];
float sensor1_offset, sensor2_offset, sensor3_offset, sensor4_offset, sensor5_offset, sensor6_offset = 0.0f;
//const float pid_p = 1.818f;
//const float pid_p = 50.0f;
const float pid_p = 3.35f;
const float pid_i = 0.0f;//0.8
const float pid_d = 0.05f;// 0.0375
float pid_error, pid_intergral, pid_old_error, pid_value;
//float Vlow, Vhigh;
float row1_error, row2_error, angle;
float sensor1_distance;
bool sensor1_timerStarted;
float sensor2_distance;
bool sensor2_timerStarted;
bool stop_bool = 0;
bool timer_started = 0;
bool saved_bool = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////MAP FUNCTION/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float map(float in, float inMin, float inMax, float outMin, float outMax)
{
    // check it's within the range
    if (inMin<inMax) {
        if (in <= inMin)
            return outMin;
        if (in >= inMax)
            return outMax;
    } else {
        if (in >= inMin)
            return outMin;
        if (in <= inMax)
            return outMax;
    }
    // calculate how far into the range we are
    float scale = (in-inMin)/(inMax-inMin);
    // calculate the output.
    return outMin + scale*(outMax-outMin);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////Enable Motors////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void configure_motordrive(void)
{
    enable = 1;
    uni_left = 1;
    uni_right = 1;
    dir_left = 0;
    dir_right = 0;
}

void enable_PWM(void)
{
    pwm_left.period_us(100);
    pwm_right.period_us(100);
    pwm_left.write(0.5);
    pwm_right.write(0.5);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////Sensor Processing////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void read_sensors(void)
{
    sensor1power = 1;
    wait_us(500);
    sensor_list[5] = (float)sensor1.read_u16();
    sensor1power = 0;
    sensor2power = 1;
    wait_us(500);
    sensor_list[4] = (float)sensor2.read_u16();
    sensor2power = 0;
    sensor3power = 1;
    wait_us(500);
    sensor_list[3] = (float)sensor3.read_u16();
    sensor3power = 0;
    sensor4power = 1;
    wait_us(500);
    sensor_list[2] = (float)sensor4.read_u16();
    sensor4power = 0;
    sensor5power = 1;
    wait_us(500);
    sensor_list[1] = (float)sensor5.read_u16();
    sensor5power = 0;
    sensor6power = 1;
    wait_us(500);
    sensor_list[0] = (float)sensor6.read_u16();
    sensor6power = 0;
}
void calibrate_sensors(void)
{
    while(Button);
    for(int i=0; i<50; i++) {
        read_sensors();
        sensor1_offset += sensor_list[0];
        sensor2_offset += sensor_list[1];
        sensor3_offset += sensor_list[2];
        sensor4_offset += sensor_list[3];
        sensor5_offset += sensor_list[4];
        sensor6_offset += sensor_list[5];
    }
    sensor1_offset /= 50;
    sensor2_offset /= 50;
    sensor3_offset /= 50;
    sensor4_offset /= 50;
    sensor5_offset /= 50;
    sensor6_offset /= 50;
    wait(1);
    while(Button);
    /*enable = 0;
    while(!Button);
    for(int i=0;i<50;i++){
        sensor1power = 1;
        Vhigh += sensor1.read();
        sensor1power = 0;
        sensor2power = 1;
        Vhigh += sensor2.read();
        sensor2power = 0;
        sensor3power = 1;
        Vhigh += sensor3.read();
        sensor3power = 0;
    }
    Vhigh /= 150;
    Vhigh -= 0.05;
    enable = 1;*/
}

void process_sensors(void)
{
    float min_value = 65536.0;
    sensor_list[0] -= sensor1_offset;
    sensor_list[1] -= sensor2_offset;
    sensor_list[2] -= sensor3_offset;
    sensor_list[3] -= sensor4_offset;
    sensor_list[4] -= sensor5_offset;
    sensor_list[5] -= sensor6_offset;
    for(int i = 0; i<6; i++) {
        if(sensor_list[i] < 0) sensor_list[i] = 0;
        if(sensor_list[i]<min_value) min_value = sensor_list[i];
    }
    for(int i = 0; i<6; i++) {
        sensor_list[i] -= min_value;
    }
}

void calculate_error(void){
    float avg = 0;
    float sum = 0;
    //row 1
    avg += sensor_list[1]*-1024.0f; //Weighted average
    avg += sensor_list[2]*-512.0f; //Weighted average
    avg += sensor_list[3]*512.0f; //Weighted average
    avg += sensor_list[4]*1024.0f; //Weighted average
    sum = sensor_list[1] +sensor_list[2] +sensor_list[3] +sensor_list[4]; //sum of values
    row1_error = avg/(sum+0.000001f);
/*
    //row 2
    for(int i = 3; i<6; i++) {
        avg += sensor_list[i]*(float)(i-4)*740.0f; //Weighted average
        sum += sensor_list[i]; //sum of values
    }
    row2_error = avg/(sum+0.000001f);
    angle = atan((row1_error-row2_error)/1024)*57.29f;*/
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////Ultrasound 2/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void turnOffTrigger2(void){
    sensor2_trigger = 0; //Make trigger low
}
void stopUltrasound2Timer(void); //reference function before startUltrasoundTimer()

void startUltrasound2Timer(void){
    if (!sensor2_timerStarted) {
        ultrasound2_timer.start(); // start the timer
        sensor2_timerStarted = true;
        sensor2_echoTimeout.attach_us(&stopUltrasound2Timer, 15000); // in case echo fall does not occur
        sensor2_echo.fall(&stopUltrasound2Timer); //Trigger stopUltrasoundTimer() when echo pin rises
        sensor2_echo.rise(NULL); //Detach interrupt on echo
    }
}

void start_ultrasound2(void){
    sensor2_trigger = 1; //Make trigger high
    sensor2_triggerTimeout.attach_us(&turnOffTrigger2, 10); //Make trigger low after 10us by calling turnOffTrigger()
    sensor2_echo.rise(&startUltrasound2Timer); //When echo pin rises trigger startUltrasoundTimer()
}

void stopUltrasound2Timer(void){
    ultrasound2_timer.stop(); // stop the timer
    if (sensor2_timerStarted) {
        sensor2_distance = ultrasound2_timer.read() * (float)1e6 / 58.0f; //Convert echo pulse to distance
        if (sensor2_distance < 2){
            sensor2_distance = 2;
        }
        if (sensor2_distance > 150){ ///0.087s
            sensor2_distance = 150;
        }
        //pc.printf("Distance = %5.1f mm \n", sensor2_distance*10);
    }
    ultrasound2_timer.reset(); //Reset timer
    sensor2_timerStarted = false;
    sensor2_echoTimeout.detach(); //Detach timeout on echo
    sensor2_echo.fall(NULL); //Detach interrupt on echo
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////Ultrasound 1/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void turnOffTrigger1(void){
    sensor1_trigger = 0; //Make trigger low
}
void stopUltrasound1Timer(void); //reference function before startUltrasoundTimer()

void startUltrasound1Timer(void){
    if (!sensor1_timerStarted) {
        ultrasound1_timer.start(); // start the timer
        sensor1_timerStarted = true;
        sensor1_echoTimeout.attach_us(&stopUltrasound1Timer, 15000); // in case echo fall does not occur
        sensor1_echo.fall(&stopUltrasound1Timer); //Trigger stopUltrasoundTimer() when echo pin rises
        sensor1_echo.rise(NULL); //Detach interrupt on echo
    }
}

void start_ultrasound1(void){
    sensor1_trigger = 1; //Make trigger high
    sensor1_triggerTimeout.attach_us(&turnOffTrigger1, 10); //Make trigger low after 10us by calling turnOffTrigger()
    sensor1_echo.rise(&startUltrasound1Timer); //When echo pin rises trigger startUltrasoundTimer()
}

void stopUltrasound1Timer(void)
{
    ultrasound1_timer.stop(); // stop the timer
    if (sensor1_timerStarted) {
        sensor1_distance = ultrasound1_timer.read() * (float)1e6 / 58.0f; //Convert echo pulse to distance
        if (sensor1_distance < 2) {
            sensor1_distance = 2;
        }
        if (sensor1_distance > 400) {
            sensor1_distance = 400;
        }
        //pc.printf("Distance = %5.1f mm \n", sensor1_distance*10);
    }
    ultrasound1_timer.reset(); //Reset timer
    sensor1_timerStarted = false;
    sensor1_echoTimeout.detach(); //Detach timeout on echo
    sensor1_echo.fall(NULL); //Detach interrupt on echo
    ultrasound2_timeout.attach_us(&start_ultrasound2, 1000);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////CONTROL//////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void calculate_pid(void)
{
    //float pid_p = p_gain.read();
    //float pid_i = i_gain.read();
    //float pid_d = d_gain.read()/10;
    pid_error = row1_error;
    pid_intergral += pid_error;
    pid_value = (pid_p * pid_error) + (pid_i * pid_intergral * dt) + (pid_d * (pid_error - pid_old_error)/dt);
    pid_old_error = pid_error;
}

void stop(void)
{
    stop_bool = 1;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////MAIN LOOP////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main()
{
    configure_motordrive();
    enable_PWM();
    //float saved_error = 0;
    ultrasound_ticker.attach(&start_ultrasound1, 0.031);
    calibrate_sensors();
    //sensor2_distance = 100; ////////////////
    sensor_timer.start();
    //saved_bool = 0;
    while(1) {
        read_sensors();
        /*pc.printf("\n\n%4.2f %4.2f %4.2f\n%4.2f %4.2f %4.2f", sensor_list[0], sensor_list[1], sensor_list[2], sensor_list[3], sensor_list[4], sensor_list[5]);
        pc.printf("\n\npid = %4.2f", map(pid_value, -60, 60, 0.2, 0.8)+0.13f);
        pc.printf(" %4.2f", map(-1*pid_value, -60, 60, 0.2, 0.8)+0.13f);
        pc.printf("\nDistance = %4.2f", distance);
        pc.printf("\nAngle = %4.2f", angle);
        pc.printf("\nRow 1 = %4.2f", row1_error);
        pc.printf("\nRow 2 = %4.2f", row2_error);*/
        process_sensors();
        float sum = 0;
        for(int i = 1; i < 5; i++) {
            sum += sensor_list[i];
        }
        calculate_error();
        //pc.printf("\n\n%4.2f %4.2f %4.2f %4.2f %4.2f %4.2f", sensor_list[0], sensor_list[1], sensor_list[2], sensor_list[3], sensor_list[4], sensor_list[5]);
        dt = sensor_timer.read();
        sensor_timer.reset();
        calculate_pid();
        //pc.printf("\nRow 1 = %4.2f", row1_error);

        if (sensor1_distance < 15) {
            pwm_left.write(0.7);
            pwm_right.write(0.2);
            wait_ms(300);
            sensor2power = 1;
            wait_us(500);
            while((sensor2.read_u16()-sensor2_offset) < 10000); //need to test this
            sensor2power = 0;
            pwm_left.write(0.4);
            pwm_right.write(0.6);
            wait_ms(100);//100ms
        }else if(sensor_list[5] > 6750){
            pwm_left.write(0.8);
            pwm_right.write(0.3);
            sensor2power = 1;
            wait_us(500);
            while((sensor2.read_u16()-sensor2_offset) < 10000); //need to test this
            sensor2power = 0;
        }else if(sensor_list[0] > 4000){
            pwm_left.write(0.3);
            pwm_right.write(0.8);
            sensor5power = 1;
            wait_us(500);
            while((sensor5.read_u16()-sensor5_offset) < 10000); //need to test this
            sensor5power = 0;
        }else if (sum >= 2500.0f) {
            float left_speed = map(pid_value, -1024.0f, 1024.0f, 0.2f, 0.8f)+map(sensor2_distance, 14.0f, 120.0f, 0.1f, 0.3f);//0.25+map(abs(pid_value), 0.0f, 30.0f, 0.15f, 0.05f)
            float right_speed = map(pid_value, 1024.0f, -1024.0f, 0.2f, 0.8f)+map(sensor2_distance, 14.0f, 120.0f, 0.1f, 0.3f);
            pwm_left.write(left_speed);
            pwm_right.write(right_speed);
        }else if(sum < 2500.0f){
            pwm_left.write(0.5);
            pwm_right.write(0.5);
        }
    }
}


