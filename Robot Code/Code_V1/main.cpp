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
float Boundary::getSlope(){
  return(slope);
}
void Boundary::setTail(coordinate T){
  *tail = T;
}
void Boundary::setHead(coordinate H){
  *head = H;
}
float Boundary::getAngle(){
  return(atan(1/slope) * 180. / 3.14159);
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

};



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


void RPSChecker::check_any(float destination[]){

  // NOTE: This method turns toward the final coordinate before moving, then the final desired heading at the end so be careful when to use this
  //       I mean to say that If you are facing up and you want to check a point 2 inches to the left, make sure you have room to swing through quadrant IV

  coordinate start(RPS.X(), RPS.Y());
  coordinate end(destination[0], destination[1]);
  Boundary path(start, end);

  //turn to appropriate angle
  float to_heading = path.getAngle();
  check_heading(to_heading);

  //Stolen from check_x and check_y functions
  float closeness;
  int pos_neg_multiplier;

  do { //while( (closeness = abs(RPS.X() - end_x)) > 1. ){
    closeness = (check_distance(destination[0], destination[1]));
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

  check_heading(destination[2]);




}

float RPSChecker::check_distance(float x, float y){
  return ( (*(new Boundary(*(new coordinate(RPS.X(), RPS.Y() ) ), *(new coordinate(x, y) ) ))).length() );	//Returns distance between current RPS coordinate and destination coordinate
}


//////////////////////////////////////


//declares RPS location constants for key locations
//Course D was used for these values
const float START_LIGHT_X = 18.7;
const float START_LIGHT_Y = 28.2;

const float BEFORE_RAMP_X = 30.6;
const float BEFORE_RAMP_Y = 20.1;
const float BEFORE_RAMP_HEADING = 92;

const float CRANK_X = 30.7;
const float CRANK_Y = 54.5;
const float CRANK_HEADING = 92;

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
void turnCrank(){

    check_heading(CRANK_HEADING);
    float cds_value = CdS.Value();
    for (int i=0; i<3; i++){
    if (cds_value > .3){ //the light is blue

                            //CHANGED SOME STUFF HERE
            check_heading(CRANK_HEADING);
            servo.SetDegree(120);
            Sleep(1200);
            move(percent, 3*cts_per_in); //move forward an inch
            Sleep(1000);
            double now = TimeNow();
            double timeout = 6.0;
            /*while (!bump.Value() || TimeNow()-now > timeout){
              move(-percent, 2*cts_per_in); //move backwkards an inch
              check_heading(CRANK_HEADING);
              //check x
              //check y
              move(percent, 2*cts_per_in); //move forwards an inch
            }*/
            servo.SetDegree(0);
            Sleep(1200);
            move(-percent, cts_per_in); //move backwkards an inch
            Sleep(1000);
            cds_value = CdS.Value();


    } else{ //the light is red

            check_heading(CRANK_HEADING);
            servo.SetDegree(0);
            Sleep(1200);
            move(percent, 3*cts_per_in); //move forward an inch
            Sleep(1000);
            double now = TimeNow();
            double timeout = 6.0;
            /*while(!bump.Value() || TimeNow()-now > timeout){
                move(-percent, 2*cts_per_in); //move backwards an inch
                check_heading(CRANK_HEADING);
                //check x
                //check y
                move(percent, 2*cts_per_in);
            }*/
            servo.SetDegree(120);
            Sleep(1200);
            move(-percent, cts_per_in); //move backwkards an inch
            Sleep(1000);
            cds_value = CdS.Value();
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
    move(-percent, cts_per_in*10);
    turn_left(percent-toSlow, cts_per_deg*45); //angle robot towards salt
    move(-percent, cts_per_in*9); //move to the salt    THIS VALUE IS LOWER NOW
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
    RPSChecker checker;
    move(percent, cts_per_in*2);
    turn_right(percent-toSlow, cts_per_deg*90);
    move(percent, cts_per_in*10);
    turn_left(percent-toSlow, cts_per_deg*45);
    check_heading(BEFORE_RAMP_HEADING);
    //check x
    //check y
    float blah[3] = {BEFORE_RAMP_X, BEFORE_RAMP_Y, BEFORE_RAMP_HEADING};
    //checker.check_any(blah);
    move(percent, cts_per_in*38);   //DIFFERENT VALUE
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
    LCD.SetOrientation(FEHLCD::East);                //NEW STUFF
    //initialize positions of servo motors
    servoSalt.SetDegree(82);
    servo.SetDegree(0);
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );
    LCD.Write("The current battery power is ");
    LCD.Write(100*Battery.Voltage()/11.7);
    LCD.Write("%.");
            Sleep(3000);

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
    double currentTime = TimeNow();
    double startTimeout = 32.;

    while(CdS.Value()> .6 || TimeNow()-currentTime > startTimeout); //start on the light

    for (int i=0; i<arrayLength; i++){
        switch (taskArray[i]){
        case 0:
            //goToSalt();
            //getSalt();
            break;
        case 1:
            //goToCrank();
            turnCrank();
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

