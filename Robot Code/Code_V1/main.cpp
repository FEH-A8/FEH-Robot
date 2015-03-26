#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHRPS.h>
#include <math.h>

//Declarations for encoders & motors
ButtonBoard buttons(FEHIO::Bank3);
FEHEncoder right_encoder(FEHIO::P1_0);
FEHEncoder left_encoder(FEHIO::P2_0);
FEHMotor right_motor(FEHMotor::Motor0);
FEHMotor left_motor(FEHMotor::Motor1);
FEHServo servo(FEHServo::Servo7);
FEHServo servoSalt(FEHServo::Servo4);
AnalogInputPin CdS(FEHIO::P0_0);


//declares RPS location constants for key locations
const float START_LIGHT_X = 18;
const float START_LIGHT_Y = 30;
const float CRANK_X = 28.8;
const float CRANK_Y = 58.3;
const float SALT_X = 27.9;
const float SALT_Y = 8.4;
const float BUTTONS_X_RB = 15.099;
const float BUTTONS_Y = 64.099;
const float BUTTONS_X_W = 13.8;
const float BUTTONS_Y_W = 62.8;
const float GARAGE_X = 6.4;
const float GARAGE_Y = 59.099;
const float SWITCH_X = 13.669;
const float SWITCH_Y = 9.9;

const int percent = 60; //sets the motor percent for the rest of the code
const int toSlow = 15; //this int will be the fix required for the robot to travel among the course
const float cts_per_in= 3.704; //counts per inch
const float cts_per_deg = .1859; //counts per degree

//declares prototypes for functions
void goToCrank();
void goToButtons();
void goToSalt();
void goToGarage();
void goToSwitch();

void turnCrank();
void pushButtons();
void getSalt();
void depositSalt();
void toggleSwitch();

void check_heading(float heading);

void move(int percent, int counts);
void move_adjusted(int percent, int counts);
void brake(int percent);
void turn_left(int percent, int counts);
void turn_right(int percent, int counts);


void move(int percent, int counts) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    //Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);


    //While the average of the left and right encoder are less than counts,
    //keep running motors
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts){
        LCD.Write("Right Encoder Counts: ");
        LCD.WriteLine(right_encoder.Counts());
        LCD.Write("Left Encoder Counts: ");
        LCD.WriteLine(left_encoder.Counts());
        LCD.Write("CdS Value: ");
        LCD.WriteLine(CdS.Value());
    }

    brake(percent);

    Sleep(1000);
} //move


/*
 *This method allows the robot to move for a certain number of counts
 * @param counts
 *              the number of counts for which the robot should move forward
 * @param percent
 *              the motor power percentage at which the robot will travel.
 * @convention
 *              if @param percent is negative, robot will move backwards
 *              else robot will move forwards
 */
void move_adjusted(int percent, int counts) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    //Set both motors to desired percent
    right_motor.SetPercent((percent*2)/3);
    left_motor.SetPercent((percent*2)/3);


    //While the average of the left and right encoder are less than counts,
    //keep running motors
    int current_counts = (left_encoder.Counts() + right_encoder.Counts()) / 2.;
    while(current_counts < counts/3){
        LCD.Write("Right Encoder Counts: ");
        LCD.WriteLine(right_encoder.Counts());
        LCD.Write("Left Encoder Counts: ");
        LCD.WriteLine(left_encoder.Counts());

        //Scales from half percent to full percent over the first third of the path
        right_motor.SetPercent((percent*2)/3 + percent * current_counts/counts);     //Don't ask
        left_motor.SetPercent((percent*2)/3 + percent * current_counts/counts);      //Don't ask
        current_counts = (left_encoder.Counts() + right_encoder.Counts()) / 2.;
    }

    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);

    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts);

    brake(percent);

    Sleep(1000);
} //move_adjusted


/*
 * This method will allow the robot to brake more quickly
 */
void brake(int percent){
    if (percent>0){
        right_motor.SetPercent(-100);
        left_motor.SetPercent(-100);
    } else {
        right_motor.SetPercent(100);
        left_motor.SetPercent(100);
    }

    Sleep(200);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();

} //brake

/*
 *This method allows the robot to turn right for a certain number of counts
 * @param counts
 *              the number of counts for which the robot should be turning
 * @param percent
 *              the percent at which the motor will be powered during turning
 */
void turn_right(int percent, int counts) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    //Set both motors to desired percent
    //hint: set right motor forward, left motor backward

    right_motor.SetPercent(-percent);
    left_motor.SetPercent(percent);


    //While the average of the left and right encoder are less than counts,
    //keep running motors

    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts);

    //brake
    right_motor.SetPercent(90);
    left_motor.SetPercent(-90);
    Sleep(200);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();

    Sleep(1000);
} //turn_right

/*
 *This method allows the robot to turn left for a certain number of counts
 * @param counts
 *              the number of counts for which the robot should be turning
 * @param percent
 *              the percent at which the motor will be powered during turning
 */
void turn_left(int percent, int counts) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    //Set both motors to desired percent

    right_motor.SetPercent(percent);
    left_motor.SetPercent(-percent);

    //While the average of the left and right encoder are less than counts,
    //keep running motors

    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts);


    //brake
    right_motor.SetPercent(-90);
    left_motor.SetPercent(90);
    Sleep(150);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();

    Sleep(1000);
} //turn_left

/*
 *This method will allow the robot to push the buttons on the side of the BOO in order
 */
void pushButtons(){
    int counts = cts_per_in * 1;
    if (RPS.RedButtonOrder() == 1){
        servo.SetDegree(25); //prepare to hit red button
        Sleep(100);
        move(percent-toSlow, counts); //drive forward and push button
        Sleep(100);
        move(-(percent-toSlow), counts); //back up
        if (RPS.WhiteButtonOrder() == 2){
            servo.SetDegree(90); //prepare to hit white button
            Sleep(100);
            move(percent-toSlow, counts); //drive forward and push button
            Sleep(100);
            move(-(percent-toSlow), counts); //back up
            Sleep(100);
            servo.SetDegree(150); //prepare to hit blue button
            Sleep(100);
            move(percent-toSlow, counts); //drive forward and push button
            Sleep(100);
            move(-(percent-toSlow), counts); //back up
        } else{ //if blue is second
            servo.SetDegree(150); //prepare to hit blue button
            Sleep(100);
            move(percent-toSlow, counts); //drive forward and push button
            Sleep(100);
            move(-(percent-toSlow), counts); //back up
            Sleep(100);
            servo.SetDegree(90); //prepare to hit white button
            Sleep(100);
            move(percent-toSlow, counts); //drive forward and push button
            Sleep(100);
            move(-(percent-toSlow), counts); //back up
        } //else
    } else if (RPS.WhiteButtonOrder() == 1){
        servo.SetDegree(90); //prepare to hit white button
        Sleep(100);
        move(percent-toSlow, counts); //drive forward and push button
        Sleep(100);
        move(-(percent-toSlow), counts); //back up
        if (RPS.RedButtonOrder() == 2){
            servo.SetDegree(25); //prepare to hit red button
            Sleep(100);
            move(percent-toSlow, counts); //drive forward and push button
            Sleep(100);
            move(-(percent-toSlow), counts); //back up
            Sleep(100);
            servo.SetDegree(150); //prepare to hit blue button
            Sleep(100);
            move(percent-toSlow, counts); //drive forward and push button
            Sleep(100);
            move(-(percent-toSlow), counts); //back up
        } else{ //if blue is second
            servo.SetDegree(150); //prepare to hit blue button
            Sleep(100);
            move(percent-toSlow, counts); //drive forward and push button
            Sleep(100);
            move(-(percent-toSlow), counts); //back up
            Sleep(100);
            servo.SetDegree(25); //prepare to hit red button
            Sleep(100);
            move(percent-toSlow, counts); //drive forward and push button
            Sleep(100);
            move(-(percent-toSlow), counts); //back up
        } //else
    } else if (RPS.BlueButtonOrder() == 1){
        servo.SetDegree(150); //prepare to hit blue button
        Sleep(100);
        move(percent-toSlow, counts); //drive forward and push button
        Sleep(100);
        move(-(percent-toSlow), counts); //back up
        if (RPS.RedButtonOrder() == 2){
            servo.SetDegree(25);
            Sleep(100);
            move(percent-toSlow, counts); //drive forward and push button
            Sleep(100);
            move(-(percent-toSlow), counts); //back up
            Sleep(100);
            servo.SetDegree(90); //prepare to hit white button
            Sleep(100);
            move(percent-toSlow, counts); //drive forward and push button
            Sleep(100);
            move(-(percent-toSlow), counts); //back up
        } else{ //if white is second
            servo.SetDegree(90); //prepare to hit white button
            Sleep(100);
            move(percent-toSlow, counts); //drive forward and push button
            Sleep(100);
            move(-(percent-toSlow), counts); //back up
            Sleep(100);
            servo.SetDegree(25); //prepare to hit red button
            Sleep(100);
            move(percent-toSlow, counts); //drive forward and push button
            Sleep(100);
            move(-(percent-toSlow), counts); //back up
        } //else
    }
} //pushButtons

/*
 * This method will turn the crank
 */
void turnCrank(float cds_value){
    if (cds_value > .3){ //the light is blue
        servo.SetDegree(120);
        Sleep(1200);
        move(percent, cts_per_in); //move forward an inch
        Sleep(500);
        servo.SetDegree(0);
        Sleep(1200);
        move(-percent, cts_per_in); //move backwkards an inch
        Sleep(500);

        servo.SetDegree(120);
        Sleep(1200);
        move(percent, cts_per_in); //move forward an inch
        Sleep(500);
        servo.SetDegree(0);
        Sleep(1200);
        move(-percent, cts_per_in); //move backwkards an inch
        Sleep(500);

        servo.SetDegree(120);
        Sleep(1200);
        move(percent, cts_per_in); //move forward an inch
        Sleep(500);
        servo.SetDegree(0);
        Sleep(1200);
        move(-percent, cts_per_in); //move backwards an inch

    } else{ //the light is red
        servo.SetDegree(0);
        Sleep(1200);
        move(percent, cts_per_in); //move forward an inch
        Sleep(500);
        servo.SetDegree(120);
        Sleep(1200);
        move(-percent, cts_per_in); //move backwkards an inch
        Sleep(500);

        servo.SetDegree(0);
        Sleep(1200);
        move(percent, cts_per_in); //move forward an inch
        Sleep(500);
        servo.SetDegree(120);
        Sleep(1200);
        move(-percent, cts_per_in); //move backwkards an inch
        Sleep(500);

        servo.SetDegree(0);
        Sleep(1200);
        move(percent, cts_per_in); //move forward an inch
        Sleep(500);
        servo.SetDegree(120);
        Sleep(1200);
        move(-percent, cts_per_in); //move backwards an inch
    }
} //turnCrank

/*
 * This method will pick up the salt bag.
 * The salt bag is to be dragged by the robot for the remainder of the course,
 * until it is deposited in the garage.
 */
void getSalt(){
    servoSalt.SetDegree(172);
} //getSalt

/*
 * This method will deposit the salt into the garage.
 */
void depositSalt(){
    servoSalt.SetDegree(0);
    move(-(percent-toSlow), cts_per_in*1); //back up and push salt into garage
} //depositSalt

/*
 * This method will toggle the oil switch.
 * @convention
 *          if RPS.OilDirec() is 0, then switch must be pushed to the left
 *          else the switch must be pushed to the right.
 */
void toggleSwitch(){
    if (RPS.OilDirec()==0){
        turn_right(percent-toSlow, cts_per_deg*45);
        servoSalt.SetDegree(45);
        turn_left(percent-toSlow, cts_per_deg*50);
    } else{
        turn_left(percent-toSlow, cts_per_deg*45);
        servoSalt.SetDegree(45);
        turn_right(percent-toSlow, cts_per_deg*50);
    }
} //toggleSwitch

/*
 * This method will take the robot from the start light, to the salt bag.
 * @pre
 *      the robot will be at the start light
 */
void goToSalt(){
    move(-percent, cts_per_in*12);
    turn_left(percent-toSlow, cts_per_deg*45); //angle robot towards salt
    move(-percent, cts_per_in*19); //move to the salt
    //check x and y
    check_heading(135);
} //goToSalt

/*
 * This method will take the robot from the salt bag to the crank.
 * @pre
 *      the robot will be at the salt bag
 */
void goToCrank(){

} //goToCrank

/*
 * This method will take the robot from the crank to the buttons
 * @pre
 *      the robot will be at the crank
 */
void goToButtons(){

} //goToButtons

/*
 * This method will take the robot from the buttons, to the garage.
 * After this method is called, the robot should be in good position
 * to deposit the salt into the garage.
 * @pre
 *      the robot will be at the buttons
 */
void goToGarage(){

} //goToGarage

/*
 * Finally, this method will take the robot from the garage, to the oil switch.
 * After this method is called, the robot should go down the avalanche ramp and
 * be placed in a good position to toggle the switch.
 * @pre
 *      the robot will be at the garage
 */
void goToSwitch(){

} //goToSwitch

/*
 * This method will check the current heading of the robot, and set the heading of the robot to @param heading
 * @param heading
 *          the heading at which the robot wants to be
 */
void check_heading(float heading){
    const int turnPercent = 50;
    float change = (int)(abs(heading-RPS.Heading()));
    if(change>180){ change = 360-change; }
    while(change > 2){
        float curr_Heading = RPS.Heading();
        if(curr_Heading < heading || (curr_Heading > 270 && heading < 90)){ //Turn right
            right_motor.SetPercent(-turnPercent);
            left_motor.SetPercent(turnPercent);
        } //if
        else { //Turn left
            right_motor.SetPercent(turnPercent);
            left_motor.SetPercent(-turnPercent);
        } //else
        Sleep(30);
        change = (int)abs(heading-RPS.Heading());
        if(change>180){
            change = 360-change;
        } //if
    } //while
    right_motor.SetPercent(0);
    left_motor.SetPercent(0);
} //check_heading

/*
 * This method was created to efficiently complete performance test 5
 */
void performanceTest5(){
    float begin_heading = RPS.Heading();

    move(-percent, cts_per_in*13.5);
    turn_right(percent-8, cts_per_deg*90);
    move(percent, cts_per_in*11);
    turn_left(percent-8, cts_per_deg*90);
    servoSalt.SetDegree(174);

    //Split up the up ramp movement so it RPS checks before it gets there
    move(percent, cts_per_in*38);
    float cds_value = CdS.Value();
    //check_heading((float)(((int)begin_heading+180) % 360));
    move(percent, cts_per_in*6);

    servoSalt.SetDegree(180);
    Sleep(1000);
    turnCrank(cds_value);
    Sleep(1000);
    servoSalt.SetDegree(82);
    move(-percent, cts_per_in*43);
    turn_right(percent-8, cts_per_deg*90);
    move(-percent, cts_per_in*14);
    servoSalt.SetDegree(126);
    Sleep(1000);
    turn_left(percent+8, cts_per_deg*90);
    /*
    move(-percent, cts_per_in*8);
    turn_right(percent+8, cts_per_deg*90);
    move(-percent, cts_per_in*7);
    toggleSwitch();
*/


} //performanceTest5

/*
 * The main method
 */
int main(void)
{
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

    const int arrayLength = 5; //sets length of task array

    /*
     * Initialize task array.
     * Elements of which are to be used in switch case in main.
     */
    int taskArray[arrayLength] = {0, 1, 2, 3, 4};

    //initialize positions of servo motors
    servoSalt.SetDegree(82);
    servo.SetDegree(0);

    //initialize encoder thresholds
    right_encoder.SetThresholds(.5, 2);
    left_encoder.SetThresholds(.5, 2);

    //RPS.InitializeMenu();

    while(CdS.Value()>1); //start on the light

    performanceTest5();

//    for (int i=0; i<arrayLength; i++){
//        switch (taskArray[i]){
//        case 0:
//            goToSalt();
//            getSalt();
//            break;
//        case 1:
//            goToCrank();
//            turnCrank();
//            break;
//        case 2:
//            goToButtons();
//            turnButtons();
//            break;
//        case 3:
//            goToGarage();
//            depositSalt();
//        case 4:
//            goToSwitch();
//            toggleSwitch();
//        } //switch
//    } //for

    return 0;
} //main
