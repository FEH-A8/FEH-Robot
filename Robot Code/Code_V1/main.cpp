////////////////////////////////////////////////////////
//  CODE OUTLINE                                      //
//  I.   Included Libraries.................GOTO 0015 //
//  II.  Sensor and Motor declarations......GOTO 0026 //
//  III. Global Constants...................GOTO 0038 //
//  IV.  Function prototypes................GOTO 0063 //
//  V.   int main(void).....................GOTO 0097 //
//  vi.  Function declarations..............GOTO 0125 //
//       a. Drive and check functions.......GOTO 0127 //
//       b. Task functions..................GOTO 0405 //
//       c. Start functions.................GOTO 0748 //
////////////////////////////////////////////////////////


//INCLUDED LIBRARIES
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHRPS.h>
#include <math.h>
#include <FEHBattery.h>


//SENSOR AND MOTOR DECLARATIONS
ButtonBoard buttons(FEHIO::Bank3);	//FEH button board
FEHEncoder right_encoder(FEHIO::P1_0);	//right shaft encoder
FEHEncoder left_encoder(FEHIO::P2_0);	//left shaft encoder
FEHMotor right_motor(FEHMotor::Motor0);	//right VEX drive motor
FEHMotor left_motor(FEHMotor::Motor1);	//left VEX drive motor
FEHServo servo(FEHServo::Servo7);	//button & crank servo
FEHServo servoSalt(FEHServo::Servo4);	//salt bag servo
AnalogInputPin CdS(FEHIO::P0_0);	//CdS cell
DigitalInputPin bump (FEHIO::P1_3);	//button & crank bump switch


//GLOBAL CONSTANTS
float CRANK_X = 30.6;
float CRANK_Y = 55.9;
float CRANK_HEADING = 90;
float CRANK_X_offset = 0;

float BUTTON_X = 14.3;
float BUTTON_Y = 63.4;
float BUTTON_HEADING = 134;

const int SERVO_RED = 25;
const int SERVO_BLUE = 150;
const int SERVO_WHITE = 90;


const float saltStart = 62;
const float saltUp = 92;
const float saltDown = 158;
const float saltRamp = 172;
const float saltGarage = 135;
const float saltSwitch = 116;
const float oil_push = 106;
const float oil_pull = 120;


//FUNCTION PROTOTYPES
//  drive and check functions
void forward(float, int);
void forward(float);
void backward(float, int);
void backward(float);
void turn_right(int, int);
void turn_right(int);
void turn_left(int, int);
void turn_left(int);
void check_heading(float, double);
void check_heading(float);
void check_x(float);
void check_y(float);

//  task functions
void task_runner(int);
void drivetoCrank();
void turnCrank();		
bool pressedButton(int);	//Determines which button check to run
bool pressedRed(int);		//Assumes robot is on wall at red. returns true if the red button was successfully pressed after 3 tries
bool pressedBlue(int);		//Assumes robot is on wall at blue. returns true if the blue button was successfully pressed after 3 tries
bool pressedWhite(int);		//Assumes robot is on wall at white. returns true if the white button was successfully pressed after 3 tries
void pushButtons(int);

//  start functions
void calibrateServos();
void setThresholds();
void setUpScreen();
void InitializeCoordinates();
void setServos();
void waitToStart();


int main(void)
{
  float heading;
  calibrateServos();
  setThresholds();
  setUpScreen();
 
  //Initialize RPS
  RPS.InitializeMenu();
  
  InitializeCoordinates();

  setServos();

  //Wait for start light
  waitToStart();

  //Run the tasks for the course
  for (int i = 1; i<=9; i++C){
    task_runner(i);
  }   


  return 0;
} //main



//FUNCTION DEFINITIONS

//DRIVE AND CHECK FUNCTIONS
void forward(float inches){
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();

    left_motor.SetPercent(69);
    right_motor.SetPercent(70);

    while(left_encoder.Counts() + right_encoder.Counts() / 2. < (inches * 5.25)){

        LCD.WriteLine(left_encoder.Counts());
        LCD.WriteLine(right_encoder.Counts());
    }


    right_motor.Stop();
    left_motor.Stop();


    Sleep(700);

}

void forward(int percent, float inches){
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();

    left_motor.SetPercent(percent);
    right_motor.SetPercent(percent);

    while(left_encoder.Counts() + right_encoder.Counts() / 2. < (inches * 5.25)){

        LCD.WriteLine(left_encoder.Counts());
        LCD.WriteLine(right_encoder.Counts());
    }

    right_motor.SetPercent(-100);
    left_motor.SetPercent(-100);
    Sleep(170);

    right_motor.Stop();
    left_motor.Stop();

    Sleep(700);
}

void backward(float inches){
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();

    left_motor.SetPercent(-69);
    right_motor.SetPercent(-70);

    while(left_encoder.Counts() + right_encoder.Counts() / 2. < (inches * 5.25)){

        LCD.WriteLine(left_encoder.Counts());
        LCD.WriteLine(right_encoder.Counts());
    }

    right_motor.SetPercent(100);
    left_motor.SetPercent(100);
    Sleep(170);

    right_motor.Stop();
    left_motor.Stop();

    Sleep(700);

}


void backward(int percent, float inches){
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();

    left_motor.SetPercent(percent);
    right_motor.SetPercent(percent);

    while(left_encoder.Counts() + right_encoder.Counts() / 2. < (inches * 5.25)){

        LCD.WriteLine(left_encoder.Counts());
        LCD.WriteLine(right_encoder.Counts());
    }

    right_motor.SetPercent(100);
    left_motor.SetPercent(100);
    Sleep(170);

    right_motor.Stop();
    left_motor.Stop();

    Sleep(700);
//Counts per inch is 3.4??

}

void turn_right(int percent, int degrees){
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();

    left_motor.SetPercent(percent);
    right_motor.SetPercent(-percent);
    while(left_encoder.Counts() + right_encoder.Counts() / 2. < degrees * .1859){

        LCD.WriteLine(left_encoder.Counts());
        LCD.WriteLine(right_encoder.Counts());
    }

    right_motor.Stop();
    left_motor.Stop();

    //NO SLEEP HERe

}


void turn_right(int degrees){
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();

    left_motor.SetPercent(65);
    right_motor.SetPercent(-65);
    while(left_encoder.Counts() + right_encoder.Counts() / 2. < degrees * .1859){

        LCD.WriteLine(left_encoder.Counts());
        LCD.WriteLine(right_encoder.Counts());
    }

    right_motor.Stop();
    left_motor.Stop();

    Sleep(700);

}


void turn_left(int degrees){
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();

    left_motor.SetPercent(-65);
    right_motor.SetPercent(65);
    while(left_encoder.Counts() + right_encoder.Counts() / 2. < degrees * .1859){

        LCD.WriteLine(left_encoder.Counts());
        LCD.WriteLine(right_encoder.Counts());
    }

    right_motor.Stop();
    left_motor.Stop();

    Sleep(700);


}

void turn_left(int percent, int degrees){
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();

    left_motor.SetPercent(-percent);
    right_motor.SetPercent(percent);
    while(left_encoder.Counts() + right_encoder.Counts() / 2. < degrees * .1859){

        LCD.WriteLine(left_encoder.Counts());
        LCD.WriteLine(right_encoder.Counts());
    }

    right_motor.Stop();
    left_motor.Stop();

    //NO SLEEP IN HERe

}


void check_heading(float heading, double timeout){

    const int turnPercent = 43;
        float change = (int)(abs(heading-RPS.Heading()));
        if(change>180){ change = 360-change; }
        double ahora = TimeNow();
        while(change > 2 && TimeNow() - ahora < timeout ){
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
            right_motor.SetPercent(0);
            left_motor.SetPercent(0);
            change = (int)abs(heading-RPS.Heading());
            if(change>180){
                change = 360-change;
            } //if
        } //while
        right_motor.SetPercent(0);
        left_motor.SetPercent(0);

        LCD.WriteLine("Heading checked");
        Sleep(700);
} //check_heading

void check_heading(float heading){

    check_heading(heading, 120);

} //check_heading

void check_y(float end_y){
    //Assumes that robot is aligned so it travels forward along positive Y axis
    float closeness;
    int pos_neg_multiplier;

    //Turn toward positive y axis
    float start_heading = RPS.Heading();
    //check_heading(90);

    closeness = (RPS.Y() - end_y);
     while (closeness > .7) { //while( (closeness = abs(RPS.X() - end_x)) > 1. ){
      pos_neg_multiplier = (((int)(closeness < 0)) << 1) - 1;		//gives +1 if you need to travel forward, -1 if backward

      closeness = abs(closeness);
      //strong pulse motor if farther away
      if( closeness > 3){
        right_motor.SetPercent(65 * pos_neg_multiplier);
        left_motor.SetPercent(65 * pos_neg_multiplier);
      }
      else{
        right_motor.SetPercent(40 * pos_neg_multiplier);
        left_motor.SetPercent(40 * pos_neg_multiplier);
      }

      closeness = (RPS.Y() - end_y);
    }
    right_motor.SetPercent(0);
    left_motor.SetPercent(0);

    if(abs(RPS.Heading() - start_heading) >= 2){
        check_heading(start_heading);
    }
}

void check_x(float end_x){
    //Assumes that robot is aligned so it travels forward along positive X axis
    float closeness;
    int pos_neg_multiplier;

     closeness = (RPS.X() - end_x);
     while (closeness > .7){ //while( (closeness = abs(RPS.X() - end_x)) > 1. ){

      pos_neg_multiplier = (((int)(closeness < 0)) << 1) - 1;		//gives +1 if you need to travel forward, -1 if backward

      closeness = abs(closeness);
      //strong pulse motor if farther away
      if( closeness > 3){
        right_motor.SetPercent(65 * pos_neg_multiplier);
        left_motor.SetPercent(65 * pos_neg_multiplier);
      }
      else{
        right_motor.SetPercent(40 * pos_neg_multiplier);
        left_motor.SetPercent(40 * pos_neg_multiplier);
      }

      closeness = (RPS.X() - end_x);
    }
    right_motor.SetPercent(0);
    left_motor.SetPercent(0);

    //NO CEHCK HEADING
}



// TASK FUNCTIONS
void task_runner(int task){	//Runs through switch case to pick task
  switch(task){
    case 1:
      heading = RPS.Heading();
      servoSalt.SetDegree(saltUp);
      Sleep(600);

      backward(11);
      turn_left(43);
      check_heading(135);

      LCD.Write("PART 1");
      break;

    case 2:
      backward(8.1);
      check_heading(137);
      servoSalt.SetDegree(saltDown);
      Sleep(800);
      forward(2.5);
      servoSalt.Off();
      LCD.Write("PART 2");

      break;

    case 3:
      turn_right(116);
      forward(11);
      check_x(CRANK_X);
      turn_left(53)  ;

      LCD.Write("PART 3");
      break;
    
    case 4:
      check_heading(CRANK_HEADING);
      servoSalt.SetDegree(saltRamp);  //Salt into ground
      driveToCrank();   //gets it in front of crank
      servoSalt.SetDegree(saltDown);
      check_y(CRANK_Y);
      if(RPS.X() >= CRANK_X-.2){
          check_heading(CRANK_HEADING);
      }
      LCD.Write("PART 4");
  
      break;

    case 5:
      turnCrank();
      check_heading(heading);
      check_y(CRANK_Y);
      LCD.Write("PART 5");
      break;


    case 6:
      backward(2.2);
      float save_y = RPS.Y();

      //Clamp on to snow and swing it right
      servoSalt.SetDegree(saltUp);
      turn_right(102);
      check_heading(0);
      backward(1.5);
      servoSalt.SetDegree(165);
      Sleep(1000);
      turn_right(55,90);
      servoSalt.SetDegree(saltSwitch);
      turn_right(50,30);
      Sleep(1000);
      //and left
      turn_left(50,70);
      servoSalt.SetDegree(saltDown);
      Sleep(250);
      turn_left(50,45);
      servoSalt.SetDegree(saltSwitch);
      Sleep(400);
      servoSalt.SetDegree(saltUp);
      //go back and get the salt bag
      forward(2);
      turn_left(90);
      check_heading(heading);
      check_y(save_y);
      check_heading(heading);
      servoSalt.SetDegree(saltDown);

      Sleep(600);
      servoSalt.Off();
      forward(.5);
      turn_right(90);
      check_heading(5);
      backward(19);
      turn_right(41);
      backward(3.2);
      break;

    case 7:
      servoSalt.SetDegree(saltGarage);
      Sleep(1000);
      forward(6.2);

      servoSalt.SetDegree(saltUp);
      turn_left(150);
      break;

    case 8:
      forward(8);
      check_y(BUTTON_Y);
      check_heading(180);
      check_x(BUTTON_X);
      check_heading(BUTTON_HEADING);
      pushButtons();
      break;

    case 9:
      turn_right(80);
      backward(13);
      turn_left(30);
      servoSalt.SetDegree(130);
      backward(16);

      if(RPS.OilDirec() == 1){ //turning right
          servoSalt.SetDegree(oil_pull);
          backward(3);
          servoSalt.SetDegree(oil_pull-10);
          Sleep(500);
          forward(100, 5);
      } else /*turning left*/{
          servoSalt.SetDegree(oil_push);
          Sleep(600);
          backward(100, 8);
      }
}



void driveToCrank(){	//Assumes that robot is below ramp lined up with crank
  //This is how:
    //Take x position
    //theta difference = arcsin(desired distance y_offset / x_offset)
    //inches to be travelled = desired incehs (32.4) * ||inch RPS vector||/RPS dist
    //if RPS.X > x, add to original angle (90)
    //else subtract
    //go inch distance

    //Mathy stuff
    float my_x = RPS.X(), my_y = RPS.Y();
    float dest_x = CRANK_X + CRANK_X_offset;
    float dest_y = CRANK_Y;
    float delta_x = dest_x- my_x, delta_y = dest_y - my_y;
    float theta = atan((delta_y)/(abs(delta_x)));
    float inches = 33 * (sqrt((delta_y * delta_y) + (delta_x*delta_x)))/delta_y;
    float angle = 90;
    if(delta_x < 0){
        angle+=theta;
    }
    else{
        angle-=theta;
    }

    //Drive stuff
    int counts = inches * 5.25;
    int left_motor_adjust = 0, right_motor_adjust = 0;
    float heading;
    int cycle_spent_off_track = 0;

    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    right_motor.SetPercent(60);
    left_motor.SetPercent(60);

    while( left_encoder.Counts() + right_encoder.Counts() / 2. < counts){
      heading = RPS.Heading();
      cycle_spent_off_track++;	//Assume robot is off track unbtil proven otherwise

      if(heading <= angle - cycle_spent_off_track/2){		//needs minor correction
        left_motor_adjust = 0;
        right_motor_adjust++;
        if(heading <= angle-2 - cycle_spent_off_track){	//needs major correction
          right_motor_adjust += 2;
        }
        right_motor.SetPercent(60 + right_motor_adjust/3);
        left_motor.SetPercent(60 + left_motor_adjust/3);
      }
      else if(heading >= angle + cycle_spent_off_track/2){	//needs minor correction
        right_motor_adjust = 0;
        left_motor_adjust++;
        if(heading >= angle+2 + cycle_spent_off_track){	//needs major correction
          left_motor_adjust += 2;
        }
          right_motor.SetPercent(60 + right_motor_adjust/3);
          left_motor.SetPercent(60 + left_motor_adjust/3);
        }
      else{			//on track, reset cycles spent off track
        cycle_spent_off_track -= 2;
      }
      Sleep(50);
    }

    //Stop Motors
    right_motor.Stop();
    left_motor.Stop();

}


void turnCrank(){

    float cds_value = CdS.Value();
    float inital_reading = cds_value;
    int cds_miscount = 0;       //counts the changes to CdS reading
    for (int i=0; i<4; i++){
        if (cds_value > .35){ //the light is blue

                            //CHANGED SOME STUFF HERE
            LCD.Write("THE LIGHT IS BLUE");
            servo.SetDegree(180);
            Sleep(500);
            forward(2.1); //move forward into the crank
            //Sleep(500);
            servo.SetDegree(60);
            Sleep(500);
            backward(1.8); //move backwkards an inch
            //Sleep(500);

            //read new value and increment if light reading changed
            if(cds_value != 5) {cds_value = CdS.Value();}         //If not forced to stay in the loop, see if we changed our mind
            if (cds_value <= .35 && cds_miscount<2){
                cds_miscount++;
                i = 0;
            }
            if(cds_miscount >= 2){
                cds_value = 5;      //stay within this loop if the robot changes its mind more than twice
            }


        } else{ //the light is red

            LCD.Write("THE LIGHT IS RED");
            servo.SetDegree(60);
            Sleep(500);
            forward(2.1); //move forward into the crank
            //Sleep(500);
            servo.SetDegree(180);
            Sleep(500);
            backward(1.9); //move backwkards an inch
            //Sleep(500);

            cds_value = -1;      //stay within this loop if the robot changes its mind more than twice

        }
    }
}//turn crank



/*
 *These methods will allow the robot to push the buttons on the side of the BOO in order
 */

  bool pressedRed(int num){
    for(int i = 0; i<num; i++){
      if(RPS.RedButtonPressed()){ return( true ); }

      //move back and try again
      //move back and try again
      //check_heading(137);
      forward(2+i);
      backward(2);
    }
    if(RPS.RedButtonPressed()){ return( true ); }

    return(false);
  }
  bool pressedWhite(int num){
    for(int i = 0; i<num; i++){
      if(RPS.WhiteButtonPressed()){ return( true ); }

      //move back and try again
      //move back and try again
      //check_heading(137);
      forward(2+i);
      backward(2);
    }
    if(RPS.WhiteButtonPressed()){ return( true ); }

    return(false);
  }
  bool pressedBlue(int num){
    for(int i = 0; i<num; i++){
      if(RPS.BlueButtonPressed()){ return( true ); }

      //move back and try again
      //check_heading(137);
      forward(2+i);
      backward(2);
    }
    if(RPS.BlueButtonPressed()){ return( true ); }

    return(false);
  }

  bool pressedButton(int button, int num){

    switch(button){
      case 0:
        servo.SetDegree(SERVO_RED);
        return(pressedRed(num));
        break;
      case 1:
        servo.SetDegree(SERVO_WHITE);
        return(pressedWhite(num));
    break;
      case 2:
        servo.SetDegree(SERVO_BLUE);
        return(pressedBlue(num));
    break;
      default:
    return(false);
    }
  }

 void pushButtons(){	//Assumes robot is two inches away from buttons
   int rwb[3] = { RPS.RedButtonOrder(), RPS.WhiteButtonOrder(), RPS.BlueButtonOrder() };
   int order[3];
   int rwb_degree[3] = { SERVO_RED, SERVO_WHITE, SERVO_BLUE };

   //0 = red, 1 = white, 2 = blue
   //order has the order that the buttons need to be pressed
   for(int i = 0; i<3; i++){
     order[rwb[i] - 1] = i;
   }

   for(int j = 0; j < 3; j++){
     servo.SetDegree(rwb_degree[order[j]]);
     pressedButton(order[j], (j==0?3:1));
   }

}//push buttons


// START FUNCTIONS
void calibrateServos(){
    //initialize positions and calibrate servo motors
       servoSalt.SetMin(521);
       servoSalt.SetMax(2341);
       servo.SetMin(500);
       servo.SetMax(2260);
}

void waitToStart(){
    double start = TimeNow();
    double CdSTimeOut = 30;
    while(CdS.Value() > .6 && TimeNow() - start < CdSTimeOut );
}

void setThresholds(){
    //initialize encoder thresholds
    right_encoder.SetThresholds(.5, 2);
    left_encoder.SetThresholds(.5, 2);
}

void setUpScreen(){
    //Print the current battery state to the screen
    LCD.SetOrientation(FEHLCD::East);
    LCD.Clear( FEHLCD::Black );
}

void InitializeCoordinates(){

    //Ask to initialize necessary coordinates?
    LCD.WriteLine("Initailize coordinates?");
    LCD.WriteLine("L - yes");
    LCD.WriteLine("R - no");
    while(true){
        if(buttons.RightPressed()){
            break;
        }
        else if(buttons.LeftPressed()){
            LCD.Clear( FEHLCD::Black);
            LCD.WriteLine("Place the robot in front of the crank and then press the middle/left button to grab the coordinate. Press R to exit.");
            while(true){
                if(buttons.RightPressed()){
                    break;
                }
                else if(buttons.MiddlePressed()){
                    CRANK_X = RPS.X();
                    CRANK_Y = RPS.Y();
                    CRANK_HEADING = RPS.Heading();
                    LCD.Write("Crank Position recorded is (");
                    LCD.Write(CRANK_X); LCD.Write(", "); LCD.Write(CRANK_Y); LCD.WriteLine(").");
                }
                else if(buttons.LeftPressed()){
                    BUTTON_X = RPS.X();
                    BUTTON_Y = RPS.Y();
                    BUTTON_HEADING = RPS.Heading();
                    LCD.Write("BUTTON Position recorded is (");
                    LCD.Write(BUTTON_X); LCD.Write(", "); LCD.Write(BUTTON_Y); LCD.Write(", "); LCD.Write(BUTTON_HEADING);
                }
            }

        }

    }
    LCD.Clear( FEHLCD::Black );
    LCD.Write("Press middle when I am ready to start my run.");

    while(true){
        if(buttons.MiddlePressed()){
            while(!buttons.MiddleReleased());
            break;
        }
    }
}


void setServos(){
    //initialize positions of servo motors'
    LCD.Clear(FEHLCD::Black);
    servo.SetDegree(180);
    servoSalt.SetDegree(saltStart);
    Sleep(1000);
}

