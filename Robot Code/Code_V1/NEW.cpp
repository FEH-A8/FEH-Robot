#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHRPS.h>
#include <math.h>
#include <FEHBattery.h>

//Declarations for encoders & motors
ButtonBoard buttons(FEHIO::Bank3);
FEHEncoder right_encoder(FEHIO::P1_0);
FEHEncoder left_encoder(FEHIO::P2_0);
FEHMotor right_motor(FEHMotor::Motor0);
FEHMotor left_motor(FEHMotor::Motor1);
FEHServo servo(FEHServo::Servo7);
FEHServo servoSalt(FEHServo::Servo4);
AnalogInputPin CdS(FEHIO::P0_0);
DigitalInputPin bump (FEHIO::P1_3);


float CRANK_X = 55;


void forward(float);
void backward(float);

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
            forward(2.5); //move forward into the crank
            Sleep(500);
            servo.SetDegree(60);
            Sleep(500);
            backward(0.7); //move backwkards an inch
            Sleep(500);

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
            forward(2.5); //move forward into the crank
            Sleep(500);
            servo.SetDegree(180);
            Sleep(500);
            backward(0.7); //move backwkards an inch
            Sleep(500);

            //read new value and increment if light reading changed
            if(cds_value!=-1){cds_value = CdS.Value();}         //If not forced to stay in the loop, see if we changed our mind
            if (cds_value <= .35 && cds_miscount<2){
                cds_miscount++;
                i = 0;
            }
            if(cds_miscount >= 2){
                cds_value = -1;      //stay within this loop if the robot changes its mind more than twice
            }
        }
    }
}//turn crank

/*
 * New Button Methods
 */
const int SERVO_RED = 25;
const int SERVO_BLUE = 150;
const int SERVO_WHITE = 90;

/*
 *This method will allow the robot to push the buttons on the side of the BOO in order
 */

// New Button Function

  bool pressedButton(int);	//Determines which button check to run
  bool pressedRed();	//Assumes robot is on wall at red. returns true if the red button was successfully pressed after 3 tries
  bool pressedBlue();	//Assumes robot is on wall at blue. returns true if the blue button was successfully pressed after 3 tries
  bool pressedWhite();	//Assumes robot is on wall at white. returns true if the white button was successfully pressed after 3 tries
  void pushButtons();

  bool pressedRed(){
    for(int i = 0; i<3; i++){
      if(RPS.RedButtonPressed()){ return( true ); }

      //move back and try again
      //move back and try again
      forward(2+i);
      backward(2);
    }
    if(RPS.RedButtonPressed()){ return( true ); }

    return(false);
  }
  bool pressedWhite(){
    for(int i = 0; i<3; i++){
      if(RPS.WhiteButtonPressed()){ return( true ); }

      //move back and try again
      //move back and try again
      forward(2+i);
      backward(2);
    }
    if(RPS.WhiteButtonPressed()){ return( true ); }

    return(false);
  }
  bool pressedBlue(){
    for(int i = 0; i<3; i++){
      if(RPS.BlueButtonPressed()){ return( true ); }

      //move back and try again
      forward(2+i);
      backward(2);
    }
    if(RPS.BlueButtonPressed()){ return( true ); }

    return(false);
  }

  bool pressedButton(int button){
    switch(button){
      case 0:
        servo.SetDegree(SERVO_RED);
    return(pressedRed());
    break;
      case 1:
        servo.SetDegree(SERVO_WHITE);
    return(pressedWhite());
    break;
      case 2:
        servo.SetDegree(SERVO_BLUE);
    return(pressedBlue());
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
     pressedButton(order[j]);
   }

}//push buttons





void forward(float inches){
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();

    left_motor.SetPercent(68);
    right_motor.SetPercent(70);

    while(left_encoder.Counts() + right_encoder.Counts() / 2. < (inches * 5.25)){

        LCD.WriteLine(left_encoder.Counts());
        LCD.WriteLine(right_encoder.Counts());
    }


    right_motor.Stop();
    left_motor.Stop();


    Sleep(1000);

//Counts per inch is 3.4??

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

    Sleep(1000);
//Counts per inch is 3.4??

}

void backward(float inches){
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();

    left_motor.SetPercent(-68);
    right_motor.SetPercent(-70);

    while(left_encoder.Counts() + right_encoder.Counts() / 2. < (inches * 5.25)){

        LCD.WriteLine(left_encoder.Counts());
        LCD.WriteLine(right_encoder.Counts());
    }


    right_motor.Stop();
    left_motor.Stop();

    Sleep(1000);
//Counts per inch is 3.4??

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


    right_motor.Stop();
    left_motor.Stop();

    Sleep(1000);
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

    Sleep(1000);
//Counts per inch is 3.4??


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

    Sleep(1000);
//Counts per inch is 3.4??


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

    Sleep(1000);
//Counts per inch is 3.4??


}

void turn_left(int percent, int degrees){
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

    Sleep(1000);
//Counts per inch is 3.4??


}


// //////////////////////////////
// Taken from working code, edited to no longer sway around 90 degrees
void check_heading(float heading){

    const int turnPercent = 40;
        float change = (int)(abs(heading-RPS.Heading()));
        if(change>180){ change = 360-change; }
        while(change > 2){
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
        Sleep(1000);
    } //check_heading
// ////////////////////////////////


void driveToCrank(){	//Assumes that robot is below ramp lined up with crank
  //TODO: Incorporate RPS X check?

  float inches = 31.5;
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

    if(heading <= 90 - cycle_spent_off_track/2){		//needs minor correction
      left_motor_adjust = 0;
      right_motor_adjust++;
      if(heading <= 88 - cycle_spent_off_track){	//needs major correction
    right_motor_adjust += 2;
      }
      right_motor.SetPercent(60 + right_motor_adjust/3);
      left_motor.SetPercent(60 + left_motor_adjust/3);
    }
    else if(heading >= 90 + cycle_spent_off_track/2){	//needs minor correction
      right_motor_adjust = 0;
      left_motor_adjust++;
      if(heading >= 92 + cycle_spent_off_track){	//needs major correction
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


int main(void)
{
float heading;
float saltStart = 62, saltUp = 92, saltDown = 162, saltRamp = 172, saltGarage = 130, saltSwitch = 116, oil_push = 106, oil_pull = 120;

    //initialize positions and calibrate servo motors
       servoSalt.SetMin(521);
       servoSalt.SetMax(2341);
       servo.SetMin(500);
       servo.SetMax(2260);


    //initialize encoder thresholds
    right_encoder.SetThresholds(.5, 2);
    left_encoder.SetThresholds(.5, 2);

    //Print the current battery state to the screen
    LCD.SetOrientation(FEHLCD::East);
    LCD.Clear( FEHLCD::Black );

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

        }

    }

   //Initialize RPS
    RPS.InitializeMenu();

    //initialize positions of servo motors
    servo.SetDegree(180);
    servoSalt.SetDegree(saltStart);
    Sleep(1000);


    while(CdS.Value() > .6);

    //Order of steps from sheet
    //1
    heading = RPS.Heading();
    servoSalt.SetDegree(saltUp);
    Sleep(400);

    backward(11);
    turn_left(43);

    LCD.Write("PART 1");

    //2
    backward(8);
    check_heading(heading+45);
    servoSalt.SetDegree(saltDown);
    Sleep(1200);
    forward(2.5);
    LCD.Write("PART 2");

    //3
    turn_right(105);
    forward(11.5);
    turn_left(52);
    LCD.Write("PART 3");

    //4 (before ramp)
    check_heading(heading);
    servoSalt.SetDegree(saltRamp);  //Salt into ground
    driveToCrank();                 //gets it in front of crank
    servoSalt.SetDegree(saltDown);
    check_heading(90);
    LCD.Write("PART 4");

    //5  at the crank
    turnCrank();
    LCD.Write("PART 5");


    //6-10  moving from crank to garage
    backward(2);

    //Clamp on to snow and swing it right
    servoSalt.SetDegree(saltUp);
    turn_right(90);
    backward(1.5);
    servoSalt.SetDegree(160);
    Sleep(1000);
    turn_right(75);
    //and left
    servoSalt.SetDegree(saltUp);
    turn_left(45);
    servoSalt.SetDegree(saltDown);
    Sleep(500);
    turn_left(40);
    turn_right(10);
    servoSalt.SetDegree(saltUp);
    //go back and get the salt bag
    forward(2);
    turn_left(90);
    check_heading(90);
    servoSalt.SetDegree(saltDown);

    turn_right(90);
    check_heading(0);
    backward(19);
    turn_right(45);
    check_heading(318);
    backward(3);


    //11  push salt into garage
    servoSalt.SetDegree(saltGarage);
    Sleep(1000);
    forward(6);
    servoSalt.SetDegree(saltDown);
    Sleep(1000);
    backward(6);
    forward(3);
    turn_left(120);

    //12 go to buttons
    servoSalt.SetDegree(130);
    forward(7);
    turn_left(87);
    check_heading(heading+45);
    forward(3);
    pushButtons();

    //13 go to top of avalanche
    turn_right(80);
    backward(16);
    turn_left(45);
    backward(16);

    return 0;
} //main
