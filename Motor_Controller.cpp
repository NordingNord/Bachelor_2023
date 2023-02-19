// --Includes--
#include <iostream>
#include <wiringPi.h>
#include <softPwm.h>

using namespace std;

// --Global Variables--
int Motor_R_DIR1 = 28; //20; 
int Motor_R_DIR2 = 29; //21; 
int Motor_R_EN = 26; //12; 

int Motor_L_DIR1 = 21; //5; 
int Motor_L_DIR2 = 22; //6; 
int Motor_L_EN = 23; //13; 

//Some variables for easy reading
int M_Right = 2;
int M_Left = 1;
int nothing = 0;
int PWM_Max = 100;

// --Functions--
// The motor controller that takes enable, pwm and direction inputs for both motors
void Motor_Control(bool EN_MR = false, bool EN_ML = false, int DIR_MR = nothing, int DIR_ML = nothing, int PWM_MR = 0, int PWM_ML = 0){
    // Motor right has direction right
    if(DIR_MR == M_Right){
        cout << "Right motor right" << endl;
        digitalWrite(Motor_R_DIR1, 1);
        digitalWrite(Motor_R_DIR2, 0);
    }
    // Motor right has direction left
    else if(DIR_MR == M_Left){
        cout << "Right motor left" << endl;
        digitalWrite(Motor_R_DIR1, 0);
        digitalWrite(Motor_R_DIR2, 1);
    }
    // Motor right has no direction
    else{
        cout << "Right motor nothing" << endl;
        digitalWrite(Motor_R_DIR1, 0);
        digitalWrite(Motor_R_DIR2, 0);
    }

    // Motor left has direction right
    if(DIR_ML == M_Right){
        cout << "Left motor right" << endl;
        digitalWrite(Motor_L_DIR1, 1);
        digitalWrite(Motor_L_DIR2, 0);
    }
    // Motor left has direction left
    else if(DIR_ML == M_Left){
        cout << "Left motor left" << endl;
        digitalWrite(Motor_L_DIR1, 0);
        digitalWrite(Motor_L_DIR2, 1);
    }
    // Motor left has no direction
    else{
        cout << "Left motor nothing" << endl;
        digitalWrite(Motor_L_DIR1, 0);
        digitalWrite(Motor_L_DIR2, 0);
    }

    // If motor R enable output PWM
    if(EN_MR == true){
        cout << "Right motor enable" << endl;
        //pwmWrite(Motor_R_EN,PWM_MR);
        softPwmWrite(Motor_R_EN,PWM_MR);
    }
    else{
        //pwmWrite(Motor_R_EN,0);
        cout << "Right motor disabled" << endl;
        softPwmWrite(Motor_R_EN,nothing);
    }

    // If motor L enable output PWM
    if(EN_ML == true){
        cout << "Left motor enable" << endl;
        //pwmWrite(Motor_L_EN,PWM_ML);
        softPwmWrite(Motor_L_EN,PWM_ML);
    }
    else{
        //pwmWrite(Motor_L_EN,0);
        cout << "Left motor disabled" << endl;
        softPwmWrite(Motor_L_EN,nothing);
    }

}

// --Main--
int main(){
    // --Pin Setup--
    // We use gpio namespace
    wiringPiSetup();
    // We set DIR pins to output
    pinMode(Motor_R_DIR1,OUTPUT);
    pinMode(Motor_R_DIR2,OUTPUT);
    pinMode(Motor_L_DIR1,OUTPUT);
    pinMode(Motor_L_DIR2,OUTPUT);
    // We set EN pins to PWM output for speed control
    //pinMode(Motor_R_EN,PWM_OUTPUT);
    //pinMode(Motor_L_EN,PWM_OUTPUT);
    //pinMode(Motor_R_EN,OUTPUT);
    //pinMode(Motor_L_EN,OUTPUT);
    
    softPwmCreate(Motor_R_EN, nothing, PWM_Max);
    softPwmCreate(Motor_L_EN, nothing, PWM_Max);
    cout << "Starting motors" << endl;
    Motor_Control(false, false, M_Right, M_Right, nothing, nothing);
    Motor_Control(false, true, M_Left, M_Left, PWM_Max, 100);
    
    while(true){}
    return 0;
}
