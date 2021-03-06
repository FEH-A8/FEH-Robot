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
DigitalInputPin bump (FEHIO::P2_3);

//declares RPS location constants for key locations
//Course D was used for these values
const float START_LIGHT_X = 18.7;
const float START_LIGHT_Y = 28.2;

const float BEFORE_RAMP_X = 30.6;
const float BEFORE_RAMP_Y = 20.1;
const float BEFORE_RAMP_HEADING = 90;

const float CRANK_X = 30.7;
const float CRANK_Y = 54.5;
const float CRANK_HEADING = 87;

const float SALT_X = 26.2;
const float SALT_Y = 11.1;
const float SALT_HEADING = 134.;

const float BUTTONS_X = 16.4;
const float BUTTONS_Y = 64.5;
const float BUTTONS_HEADING = 130;

const float GARAGE_X = 6.1;
const float GARAGE_Y = 57.8;
const float GARAGE_HEADING = 310;

const float SWITCH_X = 10.6;
const float SWITCH_Y = 15.4;
const float SWITCH_HEADING = 86;

const int percent = 60; //sets the motor percent for the rest of the code
const int toSlow = 15; //this int will be the fix required for the robot to travel among the course
const float cts_per_in= 3.704; //counts per inch
const float cts_per_deg = .1859; //counts per degree

const int startDegree = 82;
const int downDegree = 174;

//declares prototypes for functions
float goToCrank();
void goToButtons();
void goToSalt();
void goToGarage();
void goToSwitch();

void turnCrank(float light);
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
    right_motor.SetPercent((percent>0)?(percent+1):(percent-1));
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

    right_motor.SetPercent(-percent-1);
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

    right_motor.SetPercent(percent+1);
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

    check_heading(90);

    if (cds_value > .3){ //the light is blue

        for (int i=0; i<3; i++){
            check_heading(CRANK_HEADING);
            servo.SetDegree(120);
            Sleep(1200);
            move(percent, cts_per_in); //move forward an inch
            Sleep(500);
            double now = TimeNow();
            double timeout = 12.0;
            while (bump.Value() || TimeNow()-now > timeout){
              move(-percent, cts_per_in); //move backwkards an inch
              check_heading(CRANK_HEADING);
              //check x
              //check y
              move(percent, cts_per_in); //move forwards an inch
            }
            servo.SetDegree(0);
            Sleep(1200);
            move(-percent, cts_per_in); //move backwkards an inch
            Sleep(500);
        }

    } else{ //the light is red

        for (int i=0; i<3; i++){
            check_heading(CRANK_HEADING);
            servo.SetDegree(0);
            Sleep(1200);
            move(percent, cts_per_in); //move forward an inch
            Sleep(500);
            double now = TimeNow();
            double timeout = 12.0;
            while(bump.Value() || TimeNow()-now > timeout){
                move(-percent, cts_per_in); //move backwards an inch
                check_heading(CRANK_HEADING);
                //check x
                //check y
                move(percent, cts_per_in);
            }
            servo.SetDegree(120);
            Sleep(1200);
            move(-percent, cts_per_in); //move backwkards an inch
            Sleep(500);
        }
    }
} //turnCrank

/*
 * This method will pick up the salt bag.
 * The salt bag is to be dragged by the robot for the remainder of the course,
 * until it is deposited in the garage.
 */
void getSalt(){
    servoSalt.SetDegree(downDegree);
    Sleep(1000);
} //getSalt

/*
 * This method will deposit the salt into the garage.
 */
void depositSalt(){
    servoSalt.SetDegree(startDegree);
    move(-(percent-toSlow), cts_per_in*3); //back up and push salt into garage
} //depositSalt

/*
 * This method will toggle the oil switch.
 * @convention
 *          if RPS.OilDirec() is 0, then switch must be pushed to the left
 *          else the switch must be pushed to the right.
 */
void toggleSwitch(){
    check_heading(270);
    if (RPS.OilDirec()==0)/*Switch needs to be toggled to the left*/{
        servoSalt.SetDegree(140);
        move(percent, cts_per_in*5);
    } else /*Switch needs to be toggled to the right*/{
        servoSalt.SetDegree(140);
        move(percent, cts_per_in*5);
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
    move(-percent, cts_per_in*10); //move to the salt
    //check x and y
    check_heading(SALT_HEADING);
    Sleep(1000);
} //goToSalt

/*
 * This method will take the robot from the salt bag to the crank.
 * @pre
 *      the robot will be at the salt bag
 */
float goToCrank(){
    turn_right(percent-toSlow, cts_per_deg*90);
    move(percent, cts_per_in*2);
    turn_left(percent-toSlow, cts_per_deg*45);
    //check x
    //check y
    check_heading(BEFORE_RAMP_HEADING);
    move(percent, cts_per_in*46);
    //check x
    //check y
    check_heading(CRANK_HEADING);
    return CdS.Value();
} //goToCrank

/*
 * This method will take the robot from the crank to the buttons
 * @pre
 *      the robot will be at the crank
 */
void goToButtons(){
    servoSalt.SetDegree(startDegree);
    turn_left(percent-toSlow, cts_per_deg*90);
    move(percent, cts_per_in*3);
    turn_left(percent-toSlow, cts_per_deg*90);
    check_heading(BUTTONS_HEADING);
} //goToButtons

/*
 * This method will take the robot from the buttons, to the garage.
 * After this method is called, the robot should be in good position
 * to deposit the salt into the garage.
 * @pre
 *      the robot will be at the buttons
 */
void goToGarage(){
    turn_right(percent-toSlow, cts_per_deg*90);
    move(-percent, cts_per_in*2);
    turn_right(percent - toSlow, cts_per_deg*45);
    check_heading(GARAGE_HEADING);
} //goToGarage

/*
 * Finally, this method will take the robot from the garage, to the oil switch.
 * After this method is called, the robot should go down the avalanche ramp and
 * be placed in a good position to toggle the switch.
 * @pre
 *      the robot will be at the garage
 */
void goToSwitch(){
    servoSalt.SetDegree(82);
    turn_left(percent-toSlow, cts_per_deg*45);
    check_heading(180);
    move(-percent, cts_per_in*29);
    servoSalt.SetDegree(120);
    move(percent, cts_per_in*4);
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
    while(change > 4){
        float curr_Heading = RPS.Heading();
        if(curr_Heading < heading || (curr_Heading > 270 && heading < 90)){ //Turn right
            right_motor.SetPercent(turnPercent);
            left_motor.SetPercent(-turnPercent);
        } //if
        else { //Turn left
            right_motor.SetPercent(-turnPercent);
            left_motor.SetPercent(turnPercent);
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
 * The main method
 */
int main(void)
{

    RPS.InitializeMenu();

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

    float light;

    while(CdS.Value()> .6); //start on the light

    for (int i=0; i<arrayLength; i++){
        switch (taskArray[i]){
        case 0:
            goToSalt();
            getSalt();
            break;
        case 1:
            light = goToCrank();
            turnCrank(light);
            break;
        case 3:
            goToButtons();
            pushButtons();
            break;
        case 2:
            goToGarage();
            depositSalt();
        case 4:
            goToSwitch();
            toggleSwitch();
        } //switch
    } //for

    return 0;
} //main




#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHRPS.h>
#include <math.h>


///////////////////////////////////////////
//    NATIONAL ANTHEM GLOBAL VARIABLE
      long _GLOBAL_START;	// = TimeNowMSec() after RPS Initialization
      char freedom[20][50]{
        "ERROR LINE",
        "Oh say can you see",
        "By the dawn's early light",
        "What so proudly we hailed",
        "at the twighlight's last gleaming",
        "Whose broad stripes and bright starts",
        "Through the perilous fight",
        "O'er the ramparts we watched",
        "Were so gallantly streaming?",
        "And the rockets' red glare",
        "The bombs bursting in air",
        "Gave proof through the night",
        "That our flag was still there",
        "Oh Say does that Star Spangled",
        "Banner yet wave",
        "O'er the land of the free",
        "And the HOME",
        "OF.",
        "THE.",
        "BRAVE!!"
      };//line
                                         //
///////////////////////////////////////////


///////////////////////////////////////////
//    NATIONAL ANTHEM METHOD HEADS
    int beatAtLine(int);
    int lineAtBeat(int);
    int sincestartMSec();
    int beatNow();
    void AnthemDisplay(int);
    void recallPatriotism(int);
                                         //
///////////////////////////////////////////


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
const float cts_per_deg = .1776; //counts per degree

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
void turn_left(int percent, int counts);
void turn_right(int percent, int counts);

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
       /* LCD.Write("Right Encoder Counts: ");
        LCD.WriteLine(right_encoder.Counts());
        LCD.Write("Left Encoder Counts: ");
        LCD.WriteLine(left_encoder.Counts());
    */

        recallPatriotism(35);
    }

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();

    recallPatriotism(1000);
} //move

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

    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts){

        recallPatriotism(5);
    }


    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();

    recallPatriotism(1000);
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

    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts){

        recallPatriotism(5);
    }

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();

    recallPatriotism(1000);
} //turn_left

/*
 *This method will allow the robot to push the buttons on the side of the BOO in order
 */
void pushButtons(){
    int counts = cts_per_in * 1;
    if (RPS.RedButtonOrder() == 1){
        servo.SetDegree(25); //prepare to hit red button
        recallPatriotism(100);
        move(percent-toSlow, counts); //drive forward and push button
        recallPatriotism(100);
        move(-(percent-toSlow), counts); //back up
        if (RPS.WhiteButtonOrder() == 2){
            servo.SetDegree(90); //prepare to hit white button
            recallPatriotism(100);
            move(percent-toSlow, counts); //drive forward and push button
            recallPatriotism(100);
            move(-(percent-toSlow), counts); //back up
            recallPatriotism(100);
            servo.SetDegree(150); //prepare to hit blue button
            recallPatriotism(100);
            move(percent-toSlow, counts); //drive forward and push button
            recallPatriotism(100);
            move(-(percent-toSlow), counts); //back up
        } else{ //if blue is second
            servo.SetDegree(150); //prepare to hit blue button
            recallPatriotism(100);
            move(percent-toSlow, counts); //drive forward and push button
            recallPatriotism(100);
            move(-(percent-toSlow), counts); //back up
            recallPatriotism(100);
            servo.SetDegree(90); //prepare to hit white button
            recallPatriotism(100);
            move(percent-toSlow, counts); //drive forward and push button
            recallPatriotism(100);
            move(-(percent-toSlow), counts); //back up
        } //else
    } else if (RPS.WhiteButtonOrder() == 1){
        servo.SetDegree(90); //prepare to hit white button
        recallPatriotism(100);
        move(percent-toSlow, counts); //drive forward and push button
        recallPatriotism(100);
        move(-(percent-toSlow), counts); //back up
        if (RPS.RedButtonOrder() == 2){
            servo.SetDegree(25); //prepare to hit red button
            recallPatriotism(100);
            move(percent-toSlow, counts); //drive forward and push button
            recallPatriotism(100);
            move(-(percent-toSlow), counts); //back up
            recallPatriotism(100);
            servo.SetDegree(150); //prepare to hit blue button
            recallPatriotism(100);
            move(percent-toSlow, counts); //drive forward and push button
            recallPatriotism(100);
            move(-(percent-toSlow), counts); //back up
        } else{ //if blue is second
            servo.SetDegree(150); //prepare to hit blue button
            recallPatriotism(100);
            move(percent-toSlow, counts); //drive forward and push button
            recallPatriotism(100);
            move(-(percent-toSlow), counts); //back up
            recallPatriotism(100);
            servo.SetDegree(25); //prepare to hit red button
            recallPatriotism(100);
            move(percent-toSlow, counts); //drive forward and push button
            recallPatriotism(100);
            move(-(percent-toSlow), counts); //back up
        } //else
    } else if (RPS.BlueButtonOrder() == 1){
        servo.SetDegree(150); //prepare to hit blue button
        recallPatriotism(100);
        move(percent-toSlow, counts); //drive forward and push button
        recallPatriotism(100);
        move(-(percent-toSlow), counts); //back up
        if (RPS.RedButtonOrder() == 2){
            servo.SetDegree(25);
            recallPatriotism(100);
            move(percent-toSlow, counts); //drive forward and push button
            recallPatriotism(100);
            move(-(percent-toSlow), counts); //back up
            recallPatriotism(100);
            servo.SetDegree(90); //prepare to hit white button
            recallPatriotism(100);
            move(percent-toSlow, counts); //drive forward and push button
            recallPatriotism(100);
            move(-(percent-toSlow), counts); //back up
        } else{ //if white is second
            servo.SetDegree(90); //prepare to hit white button
            recallPatriotism(100);
            move(percent-toSlow, counts); //drive forward and push button
            recallPatriotism(100);
            move(-(percent-toSlow), counts); //back up
            recallPatriotism(100);
            servo.SetDegree(25); //prepare to hit red button
            recallPatriotism(100);
            move(percent-toSlow, counts); //drive forward and push button
            recallPatriotism(100);
            move(-(percent-toSlow), counts); //back up
        } //else
    }
} //pushButtons

/*
 * This method will turn the crank
 */
void turnCrank(){
    if (CdS.Value() > .3){ //the light is blue
        servo.SetDegree(180);
        recallPatriotism(500);
        move(percent, cts_per_in); //move forward an inch
        servo.SetDegree(0);
        recallPatriotism(500);
        move(-percent, cts_per_in); //move backwkards an inch
        servo.SetDegree(180);
        recallPatriotism(500);
        move(percent, cts_per_in); //move forward an inch
        servo.SetDegree(0);
        recallPatriotism(500);
        move(-percent, cts_per_in); //move backwards an inch

    } else{ //the light is red
        servo.SetDegree(0);
        recallPatriotism(500);
        move(percent, cts_per_in); //move forward an inch
        servo.SetDegree(180);
        recallPatriotism(500);
        move(-percent, cts_per_in); //move backwkards an inch
        servo.SetDegree(0);
        recallPatriotism(500);
        move(percent, cts_per_in); //move forward an inch
        servo.SetDegree(180);
        recallPatriotism(500);
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
        recallPatriotism(30);
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
    move(-percent, cts_per_in*13.5);
    turn_right(percent, cts_per_deg*90);
    move(percent, cts_per_in*11);
    turn_left(percent, cts_per_deg*90);
    servoSalt.SetDegree(174);
    move(percent, cts_per_in*38);
    recallPatriotism(1000);
    turnCrank();
    recallPatriotism(1000);
    move(-percent, cts_per_in*38);
    turn_right(percent, cts_per_deg*90);
    move(-percent, cts_per_in*14);
    turn_left(percent, cts_per_in*90);
    move(-percent, cts_per_in*8);
    turn_right(-percent, cts_per_deg*90);
    move(-percent, cts_per_in*7);
    toggleSwitch();

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
    servoSalt.SetDegree(74);
    servo.SetDegree(0);

    //initialize encoder thresholds
    right_encoder.SetThresholds(.5, 2);
    left_encoder.SetThresholds(.5, 2);

    while(CdS.Value()>1); //start on the light

    _GLOBAL_START = TimeNowMSec();
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




///////////////////////////////////////////
//    METHOD DEFINITIONS
  int beatAtLine(int line){
    if(line > 0 && line <= 20)
      return((6 * line) - 3);
    else
      return(-3);		//Return error line if something goes wrong
  }//beatAtLine

  int lineAtBeat(int beat){
    if(beat > 111)	//If after the last beat, assume it is last line to avoid index error
      return(19);
    else if(beat < 0)
      return(0);
    else
      return((beat + 3) / 6);
  }//lineAtBeat


  //gets millisecs since beginning time
  int sincestartMSec(){
      return((int)(TimeNowMSec() - _GLOBAL_START));
  }

  //Gives the current beat based on time since beginning
  int beatNow(){
      return(sincestartMSec()/560);
  }//beatNow


  void AnthemDisplay(int line){	//Displays the given line to the Proteus in the correct spot
    LCD.WriteLine(freedom[line]);\
  }


  //Acts like the recallPatriotism(int) function, but better, because AMERICA NEVER WOULD NEVER JUST SIT THERE.
  //Param: (int) duration --> the duration (in milliseconds) to be standing by in great anticipation
  //  for not only the next operation of our apparatus of freedom (robot), but also the next incantation to
  //  to let ring (anthem line).
  void recallPatriotism(int duration){
    //Should I prepare to sing the next line?
    //  If the beat marking of the next line falls within the duration
    //  of this function call, then you can bet there will be singing.
    long end_time = TimeNowMSec() + duration;
    int base_beat = beatNow();
    int base_lines = lineAtBeat(base_beat);

    if(lineAtBeat(base_beat + duration / 560) == lineAtBeat(base_beat)){		//If the line at the end of the pause is the same as it is now, line won't be changed, so just recallPatriotism like normal
      Sleep(duration);
    }//if
    else {									//else, plan when to sing the next line(s)
      int lines_passed = 0;					//measures how many lines have passed
      Sleep(duration % 35);					//recallPatriotisms the remainder time if the recallPatriotism isn't a multiple of the 35 ms beat
      while( TimeNowMSec() < end_time ){				//loop until recallPatriotism is over
        if(lineAtBeat(beatNow()) > base_lines + lines_passed){	//If the new line has come
          lines_passed++;
          AnthemDisplay(base_lines + lines_passed);
        }//end if
        Sleep(35);						//recallPatriotism the beat length, so as to not wear down the battery as much
      }//end while
    }//end else

  }

                                         //
///////////////////////////////////////////



/*const int beat_at_line[20] = {  //The (64th note) beat of each starting line, counting the pickup as the third beat of a full "first" measure
  -3,		//ERROR LINE
  3,		//Oh say can you see
  9,		//By the dawn's early light
  15,		//What so proudly we hailed
  21,		//at the twighlight's last gleaming
  27,		//Whose broad stripes and bright starts
  33,		//Through the perious fight
  39,		//O'er the ramparts we watched
  45,		//Were so gallantly streaming?
  51,		//And the rockets' red glare
  57,		//The bombs bursting in air
  63,		//Gave proof through the night
  69,		//That our flag was still there
  75,		//Oh Say does that Star Spangled
  82,		//Banner yet wave
  87,		//O'er the land of the free
  93,		//And the HOME
  99,		//OF.
  105,		//THE.
  111		//BRAVE!!
}//Actually, represents te formula 6i-3
*/

#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHRPS.h>
#include <math.h>
using namespace std;


//Declarations for encoders & motors
ButtonBoard buttons(FEHIO::Bank3);
FEHEncoder right_encoder(FEHIO::P1_0);
FEHEncoder left_encoder(FEHIO::P2_0);
FEHMotor right_motor(FEHMotor::Motor0);
FEHMotor left_motor(FEHMotor::Motor1);
FEHServo servo(FEHServo::Servo7);
FEHServo servoSalt(FEHServo::Servo4);
AnalogInputPin CdS(FEHIO::P0_0);


//////////////////////////////////////


//Tuple coordinate data structure
//Stores an X and Y cartesian coordinate
struct coordinate {
  float x;
  float y;
  void zero(){ x = 0; y = 0; }
  coordinate(){x = 0; y = 0;}
  coordinate(float ex, float why){ x = ex; y = why; }
};

class Boundary {
  coordinate * tail;
  coordinate * head;
  float x_comp;
  float y_comp;
  float slope;

  public:
    Boundary (coordinate, coordinate);              	//constructor
    coordinate * getTail();								//get tail
    coordinate * getHead();
    void setTail(coordinate);
    void setHead(coordinate);
    float length();			//finds the distance between the head and the tail
    int compareto_bound(coordinate); 	//returns -1 if < line, 1 if > line, 0 if on line
    int compareto_slab(coordinate, float); 		//"slab" is a boundary line extending in the positive direction with spacing, width. Returns -1 if < both lines, 1 if > both lines, 0 if on or between lines
    bool intersection(Boundary *);
};


/* Boundary Member Methods */
Boundary::Boundary(coordinate first, coordinate second){
  *tail = first;
  *head = second;
  x_comp = (head->x - tail->x);
  y_comp = (head->y - tail->y);
  slope = y_comp / x_comp;
}

coordinate * Boundary::getTail(){
  return (tail);
}
coordinate * Boundary::getHead(){
  return (head);
}
void Boundary::setTail(coordinate T){
  *tail = T;
}
void Boundary::setHead(coordinate H){
  *head = H;
}
float Boundary::length(){
  return(sqrt(pow(x_comp,2) + pow(y_comp,2)));
}
int Boundary::compareto_bound(coordinate pt){

  if (pt.y == (slope * pt.x) + tail->y){ 	//if y = mx + b, the point is on the boundary line
    return(0);
  }
  else if (pt.y > (slope * pt.x) + tail->y){	//if the slope is too small, i.e. the coordinate is past the line
    return(1);
  }
  else if (pt.y < (slope * pt.x) + tail->y){	//if the slope is too big, i.e. the coordinate falls before the line
    return(-1);
  }
  else{						//If you get here, something done went stupid
    return(0);
  }

}


int Boundary::compareto_slab(coordinate pt, float width){
  //calculate tail point of second boundary line
  //Derivation in side notes
  float m = -1 / slope;
  float x_prime = width/(pow((1 + m * m), .5));
  float y_prime = m * x_prime + pt.y;
  x_prime += pt.x;

  coordinate second_tail(x_prime, y_prime);

  //if compare to bound on both lines is negative 1, the point lies between them.
  //if it is positive 1, it could be above or below.
  //It can never be zero...nevermind, i cant do that
  //i'll just copy most of the code from compareto_bound()

  if (pt.y >= (slope * pt.x) + tail->y && pt.y <= (slope * pt.x) + second_tail.y){ 	//if y = mx + b, the point is on the boundary line
    return(0);
  }
  else if (pt.y > (slope * pt.x) + second_tail.y){	//if the slope is too small, i.e. the coordinate is past the line
    return(1);
  }
  else if (pt.y < (slope * pt.x) + tail->y){	//if the slope is too big, i.e. the coordinate falls before the line
    return(-1);
  }
  else{						//Something done went stupid
    return(0);
  }
}

bool Boundary::intersection(Boundary * crossing){
  //Check if the crossing points lie on either sides of the reference line
  //  if slope of reference line is > 0
  //  	check if top crossing point
  //Dang this method is hard
  //Nvm I got it
  //if the crossing points lie on either side of the reference line, and the reference line points lie on either side of the crossing line, they intersect
  //  to check if the points lie on either side of the line, see if the product of their point compareto's is -1, (1 * -1)


  if (compareto_bound(*((*crossing).getHead())) * compareto_bound(*((*crossing).getTail())) == -1 && (*crossing).compareto_bound(*(getHead())) * (*crossing).compareto_bound(*(getTail())) == -1){
    return(true);
  }
  else{
    return(false);
  }
}



class Mapper{
  Boundary course[];
  public:
    Mapper();
};


/* Mapper class member functions */
Mapper::Mapper(){
/*course =
    (new Boundary(new coordinate(0, 0), new coordinate(0, 0))),
    (new Boundary(new coordinate(0, 0), new coordinate(0, 0))),
    (new Boundary(new coordinate(0, 0), new coordinate(0, 0))),
    (new Boundary(new coordinate(0, 0), new coordinate(0, 0))),
    (new Boundary(new coordinate(0, 0), new coordinate(0, 0))),

*/

}

class RPSChecker{
  float begin_X;
  float begin_Y;
  float begin_Heading;

  public:
    RPSChecker();
    void check_x_plus(float);
    void check_x_minus(float);
    void check_y_plus(float);
    void check_y_minus(float);
    void check_45(float);
    void check_heading(float);
    void check_any(float[]);		//param is end state array
    float check_distance(float, float);

} Checker;



/* Checker class member functions */
RPSChecker::RPSChecker(){
  //Initialize RPS values at start
  begin_X = RPS.X();
  begin_Y = RPS.Y();
  begin_Heading = RPS.Heading();
}

void RPSChecker::check_x_plus(float end_x){
  //Taken from lab 2
  //Assumes that robot is aligned so it travels forward along positive X axis
  //float state[3] = current_state();
  float closeness;
  int pos_neg_multiplier;

  do { //while( (closeness = abs(RPS.X() - end_x)) > 1. ){
    closeness = (RPS.X() - end_x);
    pos_neg_multiplier = ((closeness < 0) << 1) - 1;		//gives +1 if you need to travel forward, -1 if backward

    closeness = abs(closeness);
    //strong pulse motor if farther away
    if( closeness > 2){
      right_motor.SetPercent(65 * pos_neg_multiplier);
      left_motor.SetPercent(65 * pos_neg_multiplier);
    }
    else{
      right_motor.SetPercent(45 * pos_neg_multiplier);
      left_motor.SetPercent(45 * pos_neg_multiplier);
    }
  } while (closeness > .3);
  right_motor.SetPercent(0);
  left_motor.SetPercent(0);
}


void RPSChecker::check_x_minus(float end_x){
  //Taken from lab 2
  //Assumes that robot is aligned so it travels forward along negative X axis

  float closeness;
  int pos_neg_multiplier;

  do { //while( (closeness = abs(RPS.X() - end_x)) > 1. ){
    closeness = (end_x - RPS.X());
    pos_neg_multiplier = ((closeness < 0) << 1) - 1;		//gives +1 if you need to travel forward, -1 if backward

    closeness = abs(closeness);
    //strong pulse motor if farther away
    if( closeness > 2){
      right_motor.SetPercent(65 * pos_neg_multiplier);
      left_motor.SetPercent(65 * pos_neg_multiplier);
    }
    else{
      right_motor.SetPercent(45 * pos_neg_multiplier);
      left_motor.SetPercent(45 * pos_neg_multiplier);
    }
  } while (closeness > .3);
  right_motor.SetPercent(0);
  left_motor.SetPercent(0);
}

void RPSChecker::check_y_plus(float end_y){
  //Taken from lab 2
  //Assumes that robot is aligned so it travels forward along positive X axis
  //float state[3] = current_state();
  float closeness;
  int pos_neg_multiplier;

  do { //while( (closeness = abs(RPS.X() - end_x)) > 1. ){
    closeness = (RPS.Y() - end_y);
    pos_neg_multiplier = (((int)(closeness < 0)) << 1) - 1;		//gives +1 if you need to travel forward, -1 if backward

    closeness = abs(closeness);
    //strong pulse motor if farther away
    if( closeness > 2){
      right_motor.SetPercent(65 * pos_neg_multiplier);
      left_motor.SetPercent(65 * pos_neg_multiplier);
    }
    else{
      right_motor.SetPercent(45 * pos_neg_multiplier);
      left_motor.SetPercent(45 * pos_neg_multiplier);
    }
  } while (closeness > .3);
  right_motor.SetPercent(0);
  left_motor.SetPercent(0);
}

void RPSChecker::check_y_minus(float end_y){
  //Taken from lab 2
  //Assumes that robot is aligned so it travels forward along positive X axis
  //float state[3] = current_state();
  float closeness;
  int pos_neg_multiplier;

  do { //while( (closeness = abs(RPS.X() - end_x)) > 1. ){
    closeness = (end_y - RPS.Y());
    pos_neg_multiplier = (((int)(closeness < 0)) << 1) - 1;		//gives +1 if you need to travel forward, -1 if backward

    closeness = abs(closeness);
    //strong pulse motor if farther away
    if( closeness > 2){
      right_motor.SetPercent(65 * pos_neg_multiplier);
      left_motor.SetPercent(65 * pos_neg_multiplier);
    }
    else{
      right_motor.SetPercent(45 * pos_neg_multiplier);
      left_motor.SetPercent(45 * pos_neg_multiplier);
    }
  } while (closeness > .3);
  right_motor.SetPercent(0);
  left_motor.SetPercent(0);
}

void RPSChecker::check_45(float end_x){


}


// //////////////////////////////
// Taken from working code, edited to no longer sway around 90 degrees
void RPSChecker::check_heading(float heading){

    //IMPLEMENT RELATVE HEADINGS

    const int turnPercent = 50;
        float change = (int)(abs(heading-RPS.Heading()));
        if(change>180){ change = 360-change; }
        while(change > 3){
            float curr_Heading = RPS.Heading();
            if(curr_Heading < heading || (curr_Heading + change) > 360){ //Turn left
                right_motor.SetPercent(turnPercent);
                left_motor.SetPercent(-turnPercent);
            } //if
            else { //Turn right
                right_motor.SetPercent(-turnPercent);
                left_motor.SetPercent(turnPercent);
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
// ///////////////////////////////////


void RPSChecker::check_any(float destination[]){


}

float RPSChecker::check_distance(float x, float y){
  return ( (*(new Boundary(*(new coordinate(RPS.X(), RPS.Y() ) ), *(new coordinate(x, y) ) ))).length() );	//Returns distance between current RPS coordinate and destination coordinate
}


///////////////////////////////////////



int main(void)
{
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

    servoSalt.SetDegree(82);
    RPS.InitializeMenu();

                    RPSChecker check;
                    Sleep(400);
    //TEST RPS STUFF
    //Drive to arb y pos from start, check y pos, print RPS stuff and wait for button input
    //Turn 90, check heading, print, and wait
    //Drive backwards on x, check x, print, and wait

                    /*left_motor.SetPercent(65);
                    right_motor.SetPercent(65);
                    Sleep(500);
*/
                    check.check_y_minus(15);

                    LCD.WriteLine(RPS.X());
                    LCD.WriteLine(RPS.Y());
                    LCD.WriteLine(RPS.Heading());
                    while(!buttons.MiddlePressed());
                    while(buttons.MiddlePressed());

                    //Turn left
  /*                left_motor.SetPercent(-65);
                    right_motor.SetPercent(65);
                    Sleep(400);
*/
                    check.check_heading(0);
                    LCD.WriteLine(RPS.X());
                    LCD.WriteLine(RPS.Y());
                    LCD.WriteLine(RPS.Heading());
                    while(!buttons.MiddlePressed());
                    while(buttons.MiddlePressed());


                    //Run X

  /*                  left_motor.SetPercent(-65);
                    right_motor.SetPercent(-65);
                    Sleep(1000);
*/
                    check.check_x_plus(12);

                    LCD.WriteLine(RPS.X());
                    LCD.WriteLine(RPS.Y());
                    LCD.WriteLine(RPS.Heading());
                    while(!buttons.MiddlePressed());
                    while(buttons.MiddlePressed());


    return 0;
}

