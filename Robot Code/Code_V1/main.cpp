#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHRPS.h>
#include <math.h>
#include <FEHBattery.h>


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
  coordinate tail;
  coordinate head;
  float x_comp;
  float y_comp;
  float slope;

  public:
    Boundary (coordinate, coordinate);              	//constructor
    ~Boundary();
    coordinate * getTail();				//get tail
    coordinate * getHead();				//get head
    float getSlope();
    void setTail(coordinate);
    void setHead(coordinate);
    float length();		        //finds the distance between the head and the tail
    float getAngle();
    int compareto_bound(coordinate); 	//returns -1 if < line, 1 if > line, 0 if on line
    int compareto_slab(coordinate, float); 		//"slab" is a boundary line extending in the positive direction with spacing, width. Returns -1 if < both lines, 1 if > both lines, 0 if on or between lines
    bool intersection(Boundary *);
};


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
    void check_any_plus(float, float, float);
    void check_any_minus(float, float, float);
    float check_distance(float, float);
};


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
RPSChecker garmin;


//declares RPS location constants for key locations
//Course D was used for these values
float START_LIGHT_X = 18;
float START_LIGHT_Y = 30;
float START_LIGHT_HEADING = 30;

float SALT_X = 27.9;
float SALT_Y = 8.4;
float SALT_HEADING = 30;

float BEFORE_RAMP_X = 30.6;
float BEFORE_RAMP_Y = 20.1;
float BEFORE_RAMP_HEADING = 92;

float CRANK_X = 28.8;
float CRANK_Y = 58.3;
float CRANK_HEADING = 30;

float BUTTONS_X = 15.099;
float BUTTONS_Y = 64.099;
float BUTTONS_HEADING = 30;

float GARAGE_X = 6.4;
float GARAGE_Y = 59.099;
float GARAGE_HEADING = 30;

float ABOVE_AVALANCHE_X = 6;
float ABOVE_AVALANCHE_Y = 40;
float ABOVE_AVALANCHE_HEADING = 180;

float SWITCH_X = 13.669;
float SWITCH_Y = 9.9;
float SWITCH_HEADING = 30;

const float NORTH = 90;
const float EAST  = 0;
const float SOUTH = 180;
const float WEST  = 270;

const int percent = 60; //sets the motor percent for the rest of the code
const int toSlow = 15; //this int will be the fix required for the robot to travel among the course
const float cts_per_in= 3.704; //counts per inch
const float cts_per_deg = .1859; //counts per degree

const int START_DEGREE = 82;
const int DOWN_DEGREE = 174;

//declares prototypes for functions
void goToCrank();
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

void Record_RPS();

//////////////////////////////////////

/* Boundary Member Methods */
Boundary::Boundary(coordinate first, coordinate second){
  tail = first;
  head = second;
  x_comp = (head.x - tail.x);
  y_comp = (head.y - tail.y);
  slope = y_comp / x_comp;
}

Boundary::~Boundary(){
    //delete tail;
    //delete head;
}

coordinate * Boundary::getTail(){
  return (&tail);
}
coordinate * Boundary::getHead(){
  return (&head);
}
float Boundary::getSlope(){
  return(slope);
}
void Boundary::setTail(coordinate T){
  tail = T;
}
void Boundary::setHead(coordinate H){
  head = H;
}
float Boundary::getAngle(){
  return(atan(1/slope) * 180. / 3.14159);
}
float Boundary::length(){
  return(sqrt(pow(x_comp,2) + pow(y_comp,2)));
}
int Boundary::compareto_bound(coordinate pt){

  if (pt.y == (slope * pt.x) + tail.y){ 	//if y = mx + b, the point is on the boundary line
    return(0);
  }
  else if (pt.y > (slope * pt.x) + tail.y){	//if the slope is too small, i.e. the coordinate is past the line
    return(1);
  }
  else if (pt.y < (slope * pt.x) + tail.y){	//if the slope is too big, i.e. the coordinate falls before the line
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

  if (pt.y >= (slope * pt.x) + tail.y && pt.y <= (slope * pt.x) + second_tail.y){ 	//if y = mx + b, the point is on the boundary line
    return(0);
  }
  else if (pt.y > (slope * pt.x) + second_tail.y){	//if the slope is too small, i.e. the coordinate is past the line
    return(1);
  }
  else if (pt.y < (slope * pt.x) + tail.y){	//if the slope is too big, i.e. the coordinate falls before the line
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

/* Checker class member functions */
RPSChecker::RPSChecker(){
  //Initialize RPS values at start
  begin_X = RPS.X();
  begin_Y = RPS.Y();
  begin_Heading = RPS.Heading();
}

void RPSChecker::check_x_plus(float end_x){
  //Makes robot travel along x axis

  float closeness;
  int pos_neg_multiplier;

  //Turn toward positive x axis
  float start_heading = RPS.Heading();
  check_heading(0);

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
  } while (closeness > .25);
  right_motor.SetPercent(0);
  left_motor.SetPercent(0);

  check_heading(start_heading);
}


void RPSChecker::check_x_minus(float end_x){
  //Taken from lab 2
  //Assumes that robot is aligned so it travels forward along negative X axis

  float closeness;
  int pos_neg_multiplier;

  //Turn toward positive x axis
  float start_heading = RPS.Heading();
  check_heading(180);

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
  } while (closeness > .25);
  right_motor.SetPercent(0);
  left_motor.SetPercent(0);

  check_heading(start_heading);
}

void RPSChecker::check_y_plus(float end_y){
  //Taken from lab 2
  //Assumes that robot is aligned so it travels forward along positive Y axis
  //float state[3] = current_state();
  float closeness;
  int pos_neg_multiplier;

  //Turn toward positive y axis
  float start_heading = RPS.Heading();
  check_heading(90);

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
  } while (closeness > .25);
  right_motor.SetPercent(0);
  left_motor.SetPercent(0);

  check_heading(start_heading);
}

void RPSChecker::check_y_minus(float end_y){
  //Taken from lab 2
  //Assumes that robot is aligned so it travels forward along negative Y axis
  float closeness;
  int pos_neg_multiplier;

  //Turn toward positive x axis
  float start_heading = RPS.Heading();
  check_heading(270);

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
  } while (closeness > .25);
  right_motor.SetPercent(0);
  left_motor.SetPercent(0);

  check_heading(start_heading);
}

void RPSChecker::check_45(float end_x){


}


// //////////////////////////////
// Taken from working code, edited to no longer sway around 90 degrees
void RPSChecker::check_heading(float heading){

    const int turnPercent = 50;
        float change = (int)(abs(heading-RPS.Heading()));
        if(change>180){ change = 360-change; }
        while(change > .7){
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
// ////////////////////////////////


void RPSChecker::check_any_plus(float dest_x, float dest_y, float dest_heading){   //PLUS designates that front stays in the front when it moves

  // NOTE: This method turns toward the final coordinate before moving, then the final desired heading at the end so be careful when to use this
  //       I mean to say that If you are facing up and you want to check a point 2 inches to the left, make sure you have room to swing through quadrant IV

  coordinate start(RPS.X(), RPS.Y());
  coordinate end(dest_x, dest_y);
  Boundary path(start, end);

  //turn to appropriate angle
  float to_heading = path.getAngle();
  check_heading(to_heading);   //Send salt bag arm side toward heading

  //Modified from check_x and check_y functions
  float closeness = (check_distance(dest_x, dest_y));
  float last_closeness = closeness;
  int pos_neg_multiplier = 1;

  do {
    closeness = (check_distance(dest_x, dest_y));
    pos_neg_multiplier = (((int)(closeness > last_closeness)) << 1) - 1;		//gives +1 if you need to travel forward, -1 if backward (because distance increases if you pass the point)

    last_closeness = closeness;

    //strong pulse motor if farther away
    if( closeness > 4){
      right_motor.SetPercent(65 * pos_neg_multiplier);
      left_motor.SetPercent(65 * pos_neg_multiplier);
    }
    else{
      right_motor.SetPercent(45 * pos_neg_multiplier);
      left_motor.SetPercent(45 * pos_neg_multiplier);
    }

  } while (closeness > 1);
  right_motor.SetPercent(0);
  left_motor.SetPercent(0);

  check_heading(dest_heading);

}

void RPSChecker::check_any_minus(float dest_x, float dest_y, float dest_heading){   //PLUS designates that front stays in the front when it moves

  // NOTE: This method turns toward the final coordinate before moving, then the final desired heading at the end so be careful when to use this
  //       I mean to say that If you are facing up and you want to check a point 2 inches to the left, make sure you have room to swing through quadrant IV

  coordinate start(RPS.X(), RPS.Y());
  coordinate end(dest_x, dest_y);
  Boundary path(start, end);

  //turn to appropriate angle
  float to_heading = path.getAngle();
  if(to_heading >= 180){ to_heading -= 180;} else{ to_heading += 180;}   //flip the heading to the back of the robot while keeping it under 360
  check_heading(to_heading);   //Send salt bag arm side toward heading

  //Modified from check_x and check_y functions
  float closeness = (check_distance(dest_x, dest_y));
  float last_closeness = closeness;
  int pos_neg_multiplier = 1;

  do {
    closeness = (check_distance(dest_x, dest_y));
    pos_neg_multiplier = (((int)(closeness > last_closeness)) << 1) - 1;		//gives +1 if you need to travel forward, -1 if backward (because distance increases if you pass the point)

    last_closeness = closeness;

    //strong pulse motor if farther away
    if( closeness > 4){
      right_motor.SetPercent(-65 * pos_neg_multiplier);
      left_motor.SetPercent(-65 * pos_neg_multiplier);
    }
    else{
      right_motor.SetPercent(-45 * pos_neg_multiplier);
      left_motor.SetPercent(-45 * pos_neg_multiplier);
    }

  } while (closeness > 1);
  right_motor.SetPercent(0);
  left_motor.SetPercent(0);

  check_heading(dest_heading);

}

float RPSChecker::check_distance(float x, float y){
  coordinate tail(RPS.X(), RPS.Y());
  coordinate head(x,y);
  Boundary line(tail, head);
  return (line.length());
}

//////////////////////////////////////



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
      move(-50, cts_per_inch*2);
      garmin.check_heading(BUTTONS_HEADING);
      move(55, cts_per_inch*2);
    }
    if(RPS.RedButtonPressed()){ return( true ); }

    return(false);
  }
  bool pressedWhite(){
    for(int i = 0; i<3; i++){
      if(RPS.WhiteButtonPressed()){ return( true ); }

      //move back and try again
      move(-50, cts_per_inch*2);
      garmin.check_heading(BUTTONS_HEADING);
      move(55, cts_per_inch*2);
    }
    if(RPS.WhiteButtonPressed()){ return( true ); }

    return(false);
  }
  bool pressedBlue(){
    for(int i = 0; i<3; i++){
      if(RPS.BlueButtonPressed()){ return( true ); }

      //move back and try again
      move(-50, cts_per_inch*2);
      garmin.check_heading(BUTTONS_HEADING);
      move(55, cts_per_inch*2);
    }
    if(RPS.BlueButtonPressed()){ return( true ); }

    return(false);
  }

  bool pressedButton(int button){
    switch(button){
      case 0:
    return(pressedRed());
    break;
      case 1:
    return(pressedWhite());
    break;
      case 2:
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
     servo.SetDegree(rwb_degree[order[i]]);
     move(55, cts_per_inch*2);
     pressedButton[order[i]];
     move(-50, cts_per_inch * 2);
   }

}


/*
 * This method will turn the crank
 */
void turnCrank(){

    garmin.check_heading(CRANK_HEADING);
    float cds_value = CdS.Value();
    int cds_miscount = 0;       //counts changes to CdS miscount
    for (int i=0; i<3; i++){
        if (cds_value > .35){ //the light is blue

                            //CHANGED SOME STUFF HERE
            garmin.check_heading(CRANK_HEADING);
            servo.SetDegree(0);
            Sleep(1200);
            move(percent, 2.5*cts_per_in); //move forward into the crank
            Sleep(1000);
            servo.SetDegree(120);
            Sleep(1200);
            move(-percent, 1.2*cts_per_in); //move backwkards an inch
            Sleep(1000);

            //read new value and increment if light reading changed
            cds_value = CdS.Value();
            if (cds_value <= .35 && cds_miscount<2){
                cds_miscount++;
                i = 0;
            }
            if(cds_miscount >= 2){
                cds_value = 1;      //stay within this loop if the robot changes its mind more than twice
            }


        } else{ //the light is red

            garmin.check_heading(CRANK_HEADING);
            servo.SetDegree(120);
            Sleep(1200);
            move(percent, 2.5*cts_per_in); //move forward into the crank
            Sleep(1000);
            servo.SetDegree(0);
            Sleep(1200);
            move(-percent, 1.3*cts_per_in); //move backwkards an inch
            Sleep(1000);

            //read new value and increment if light reading changed
            cds_value = CdS.Value();
            if (cds_value <= .35 && cds_miscount<2){
                cds_miscount++;
                i = 0;
            }
            if(cds_miscount >= 2){
                cds_value = 1;      //stay within this loop if the robot changes its mind more than twice
            }
        }
    }
} //turnCrank

/*
 * This method will pick up the salt bag.
 * The salt bag is to be dragged by the robot for the remainder of the course,
 * until it is deposited in the garage.
 */
void getSalt(){
    servoSalt.SetDegree(DOWN_DEGREE);
    Sleep(1000);
} //getSalt

/*
 * This method will deposit the salt into the garage.
 */
void depositSalt(){
    servoSalt.SetDegree(START_DEGREE);
    move(-(percent-toSlow), cts_per_in*3); //back up and push salt into garage
} //depositSalt

/*
 * This method will toggle the oil switch.
 * @convention
 *          if RPS.OilDirec() is 0, then switch must be pushed to the left
 *          else the switch must be pushed to the right.
 */
void toggleSwitch(){
    garmin.check_heading(270);
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
    move(-percent, cts_per_in*10);
    turn_left(percent-toSlow, cts_per_deg*45); //angle robot towards salt
    garmin.check_heading(SALT_HEADING);
    move(-percent, cts_per_in*9); //move to the salt    THIS VALUE IS LOWER NOW
    garmin.check_x_minus(SALT_X);
    garmin.check_y_plus(SALT_Y);
    garmin.check_heading(SALT_HEADING);
    //garmin.check_any_minus(26.1, 11.3, 132);
    Sleep(1000);
} //goToSalt

/*
 * This method will take the robot from the salt bag to the crank.
 * @pre
 *      the robot will be at the salt bag
 */
void goToCrank(){
    move(percent, cts_per_in*2);
    turn_right(percent-toSlow, cts_per_deg*90);
    move(percent, cts_per_in*10);
    turn_left(percent-toSlow, cts_per_deg*45);

    garmin.check_x_plus(BEFORE_RAMP_X);
    garmin.check_y_plus(BEFORE_RAMP_Y);
    garmin.check_heading(BEFORE_RAMP_HEADING);
    //garmin.check_any_plus(BEFORE_RAMP_X, BEFORE_RAMP_Y, BEFORE_RAMP_HEADING);

    move(percent, cts_per_in*38);   //DIFFERENT VALUE
    garmin.check_x_plus(CRANK_X);
    garmin.check_y_plus(CRANK_Y);
    garmin.check_heading(CRANK_HEADING);
} //goToCrank

/*
 * This method will take the robot from the crank to the buttons
 * @pre
 *      the robot will be at the garage
 */
void goToButtons(){
    servoSalt.SetDegree(START_DEGREE);
    turn_left(percent-toSlow, cts_per_deg*90);
    move(percent, cts_per_in*3);
    turn_left(percent-toSlow, cts_per_deg*90);
    garmin.check_x_minus(BUTTONS_X);
    garmin.check_y_plus(BUTTONS_Y);
    garmin.check_heading(BUTTONS_HEADING);
} //goToButtons

/*
 * This method will take the robot from the buttons, to the garage.
 * After this method is called, the robot should be in good position
 * to deposit the salt into the garage.
 * @pre
 *      the robot will be at the crank
 */
void goToGarage(){
    move(-percent, cts_per_in*5);
    turn_right(percent-toSlow, cts_per_deg*90);
    move(-percent, cts_per_in*20);
    turn_right(percent - toSlow, cts_per_deg*30);
    move(-percent, cts_per_in*5);
    garmin.check_x_plus(GARAGE_X);
    garmin.check_y_minus(GARAGE_Y);
    garmin.check_heading(GARAGE_HEADING);
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
    garmin.check_heading(180);
    move(-percent, cts_per_in*29);
    servoSalt.SetDegree(120);
    move(percent, cts_per_in*4);
} //goToSwitch

void Record_RPS()
{
    int menu = 1;
    int run = 1;

    while( run == 1 )
    {
        if  (buttons.LeftPressed())
            {
            menu--;
            }
        else if (buttons.RightPressed())
            {
                menu++;
            }

        switch (menu)
        {
        case 0:
            menu = 13;
            break;
        case 1:
            LCD.Clear( FEHLCD::Black );
            LCD.WriteLine( "Salt Bag" );
            Sleep(250);
            if( buttons.MiddlePressed() )
            {
                SALT_X = RPS.X();
                SALT_Y = RPS.Y();
                SALT_HEADING = RPS.Heading();
                LCD.Write( "X = ");  LCD.WriteLine(SALT_X);
                LCD.Write( "Y = ");  LCD.WriteLine(SALT_Y);
                LCD.Write( "Heading = ");  LCD.WriteLine(SALT_HEADING);
                Sleep(1000);
                menu++;
            }//Press Middle Button
            break;
        case 3:
            LCD.Clear( FEHLCD::Black );
            LCD.WriteLine( "Crank" );
            Sleep(250);
            if( buttons.MiddlePressed() )
            {
                CRANK_X = RPS.X();
                CRANK_Y = RPS.Y();
                CRANK_HEADING = RPS.Heading();
                LCD.Write( "X = ");  LCD.WriteLine(CRANK_X);
                LCD.Write( "Y = ");  LCD.WriteLine(CRANK_Y);
                LCD.Write( "Heading = ");  LCD.WriteLine(CRANK_HEADING);
                Sleep(1000);
                menu++;
            }//Press Middle Button
            break;
        case 2:
            LCD.Clear( FEHLCD::Black );
            LCD.WriteLine( "Before Ramp" );
            Sleep(250);
            if( buttons.MiddlePressed() )
            {
                BEFORE_RAMP_X = RPS.X();
                BEFORE_RAMP_Y = RPS.Y();
                BEFORE_RAMP_HEADING = RPS.Heading();
                LCD.Write( "X = ");  LCD.WriteLine(BEFORE_RAMP_X);
                LCD.Write( "Y = ");  LCD.WriteLine(BEFORE_RAMP_Y);
                LCD.Write( "Heading = ");  LCD.WriteLine(BEFORE_RAMP_HEADING);
                Sleep(1000);
                menu++;
            }//Press Middle Button
            break;
        case 4:
            LCD.Clear( FEHLCD::Black );
            LCD.WriteLine( "Buttons" );
            Sleep(250);
            if( buttons.MiddlePressed() )
            {
                BUTTONS_X = RPS.X();
                BUTTONS_Y = RPS.Y();
                BUTTONS_HEADING = RPS.Heading();
                LCD.Write( "X = ");  LCD.WriteLine(BUTTONS_X);
                LCD.Write( "Y = ");  LCD.WriteLine(BUTTONS_Y);
                LCD.Write( "Heading = ");  LCD.WriteLine(BUTTONS_HEADING);
                Sleep(1000);
                menu++;
            }//Press Middle Button
            break;
        case 5:
            LCD.Clear( FEHLCD::Black );
            LCD.WriteLine( "Garage" );
            Sleep(250);
            if( buttons.MiddlePressed() )
            {
                GARAGE_X = RPS.X();
                GARAGE_Y = RPS.Y();
                GARAGE_HEADING = RPS.Heading();
                LCD.Write( "X = ");  LCD.WriteLine(GARAGE_X);
                LCD.Write( "Y = ");  LCD.WriteLine(GARAGE_Y);
                LCD.Write( "Heading = ");  LCD.WriteLine(GARAGE_HEADING);
                Sleep(1000);
                menu++;
            }//Press Middle Button
            break;
        case 6:
            LCD.Clear( FEHLCD::Black );
            LCD.WriteLine( "Oil Pump" );
            Sleep(250);
            if( buttons.MiddlePressed() )
            {
                SWITCH_X = RPS.X();
                SWITCH_Y = RPS.Y();
                SWITCH_HEADING = RPS.Heading();
                LCD.Write( "X = ");  LCD.WriteLine(SWITCH_X);
                LCD.Write( "Y = ");  LCD.WriteLine(SWITCH_Y);
                LCD.Write( "Heading = ");  LCD.WriteLine(SWITCH_HEADING);
                Sleep(1000);
                menu++;
            }//Press Middle Button
            break;
        case 7:
            LCD.Clear( FEHLCD::Black );
            LCD.WriteLine( "Starting Location" );
            Sleep(250);
            if( buttons.MiddlePressed() )
            {
                START_LIGHT_X = RPS.X();
                START_LIGHT_Y = RPS.Y();
                START_LIGHT_HEADING = RPS.Heading();
                LCD.Write( "X = ");  LCD.WriteLine(START_LIGHT_X);
                LCD.Write( "Y = ");  LCD.WriteLine(START_LIGHT_Y);
                LCD.Write( "Heading = ");  LCD.WriteLine(START_LIGHT_HEADING);
                Sleep(1000);
                menu++;
            }//Press Middle Button
            break;

        case 8:
            LCD.Clear( FEHLCD::Black );
            LCD.WriteLine( "Starting Location" );
            Sleep(250);
            if( buttons.MiddlePressed() )
            {
                ABOVE_AVALANCHE_X = RPS.X();
                ABOVE_AVALANCHE_Y = RPS.Y();
                ABOVE_AVALANCHE_HEADING = RPS.Heading();
                LCD.Write( "X = ");  LCD.WriteLine(ABOVE_AVALANCHE_X);
                LCD.Write( "Y = ");  LCD.WriteLine(ABOVE_AVALANCHE_Y);
                LCD.Write( "Heading = ");  LCD.WriteLine(ABOVE_AVALANCHE_HEADING);
                Sleep(1000);
                menu++;
            }//Press Middle Button
            break;
        case 9:
            LCD.Clear( FEHLCD::Black );
            LCD.WriteLine( "North" );
            Sleep(250);
            if( buttons.MiddlePressed() )
            {
                NORTH = RPS.Heading();
                LCD.Write( "Heading = ");  LCD.WriteLine(NORTH);
                Sleep(1000);
                menu++;
            }//Press Middle Button
            break;
        case 10:
            LCD.Clear( FEHLCD::Black );
            LCD.WriteLine( "East" );
            Sleep(250);
            if( buttons.MiddlePressed() )
            {
                EAST = RPS.Heading();
                LCD.Write( "Heading = ");  LCD.WriteLine(EAST);
                Sleep(1000);
                menu++;
            }//Press Middle Button
            break;
        case 11:
            LCD.Clear( FEHLCD::Black );
            LCD.WriteLine( "South" );
            Sleep(250);
            if( buttons.MiddlePressed() )
            {
                SOUTH = RPS.Heading();
                LCD.Write( "Heading = ");  LCD.WriteLine(SOUTH);
                Sleep(1000);
                menu++;
            }//Press Middle Button
            break;
        case 12:
            LCD.Clear( FEHLCD::Black );
            LCD.WriteLine( "West" );
            Sleep(250);
            if( buttons.MiddlePressed() )
            {
                WEST = RPS.Heading();
                LCD.Write( "Heading = ");  LCD.WriteLine(WEST);
                Sleep(1000);
                menu++;
            }//Press Middle Button
            break;
        case 13:
            LCD.Clear( FEHLCD::Black );
            LCD.WriteLine("Press Middle Button to End.");
            Sleep(250);
            if( buttons.MiddlePressed() )
            {
                run = 0;
                LCD.Clear( FEHLCD::Black );
            }
            break;
        case 14:
            menu=1;
            break;

         }//Switch Case
    }// Menu Loop
}//Record_RPS


/*
 * The main method
 */
int main(void)
{
    //initialize positions and calibrate servo motors
       servoSalt.SetMin(521);
       servoSalt.SetMax(2341);
       servo.SetMin(500);
       servo.SetMax(2266);

    //initialize positions of servo motors

    servoSalt.SetDegree(82);
    servo.SetDegree(90);

    //initialize encoder thresholds
    right_encoder.SetThresholds(.5, 2);
    left_encoder.SetThresholds(.5, 2);

    //Print the current battery state to the screen
    LCD.SetOrientation(FEHLCD::East);
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );
    LCD.Write("The current battery power is "); LCD.Write(100*Battery.Voltage()/11.7); LCD.Write("%.");
    Sleep(1000);
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

   //Initialize RPS
    RPS.InitializeMenu();

   //Read In RPS Values
    Record_RPS();

    /*
     * Initialize task array.
     * Elements of which are to be used in switch case in main.
     */

    const int arrayLength = 5   ; //sets length of task array
    int taskArray[arrayLength] = {0, 1, 3, 2, 4};


    double currentTime = TimeNow();
    double startTimeout = 32.;

    while(CdS.Value()> .6 || TimeNow()-currentTime > startTimeout); //start on the light

    for (int i=0; i<arrayLength; i++){
        switch (taskArray[i]){
        case 0:
            goToSalt();
            getSalt();
            break;
        case 1:
            goToCrank();
            turnCrank();
            break;
        case 2:
            goToButtons();
            pushButtons();
            break;
        case 3:
            goToGarage();
            depositSalt();
        case 4:
            goToSwitch();
            toggleSwitch();
        } //switch
    } //for

    return 0;
} //main
