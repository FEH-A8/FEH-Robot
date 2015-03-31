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
    pos_neg_multiplier = ((closeness > 0) << 1) - 1;		//gives +1 if you need to travel forward, -1 if backward

    //strong pulse motor if farther away
    if( closeness > 4){
      right_motor.SetPercent(65 * pos_neg_multiplier);
      left_motor.SetPercent(65 * pos_neg_multiplier);
    }
    else{
      right_motor.SetPercent(45 * pos_neg_multiplier);
      left_motor.SetPercent(45 * pos_neg_multiplier);
    }
  } while (closeness > .75);

}


void RPSChecker::check_x_minus(float end_x){
  //Taken from lab 2
  //Assumes that robot is aligned so it travels forward along negative X axis

  float closeness;
  int pos_neg_multiplier;

  do { //while( (closeness = abs(RPS.X() - end_x)) > 1. ){
    closeness = (RPS.X() - end_x);
    pos_neg_multiplier = ((closeness < 0) << 1) - 1;		//gives +1 if you need to travel forward, -1 if backward

    //strong pulse motor if farther away
    if( closeness > 4){
      right_motor.SetPercent(65 * pos_neg_multiplier);
      left_motor.SetPercent(65 * pos_neg_multiplier);
    }
    else{
      right_motor.SetPercent(45 * pos_neg_multiplier);
      left_motor.SetPercent(45 * pos_neg_multiplier);
    }
  } while (closeness > .75);

}

void RPSChecker::check_y_plus(float end_y){
  //Taken from lab 2
  //Assumes that robot is aligned so it travels forward along positive X axis
  //float state[3] = current_state();
  float closeness;
  int pos_neg_multiplier;

  do { //while( (closeness = abs(RPS.X() - end_x)) > 1. ){
    closeness = (RPS.Y() - end_y);
    pos_neg_multiplier = (((int)(closeness > 0)) << 1) - 1;		//gives +1 if you need to travel forward, -1 if backward

    //strong pulse motor if farther away
    if( closeness > 4){
      right_motor.SetPercent(65 * pos_neg_multiplier);
      left_motor.SetPercent(65 * pos_neg_multiplier);
    }
    else{
      right_motor.SetPercent(45 * pos_neg_multiplier);
      left_motor.SetPercent(45 * pos_neg_multiplier);
    }
  } while (closeness > .75);

}

void RPSChecker::check_y_minus(float end_y){
  //Taken from lab 2
  //Assumes that robot is aligned so it travels forward along positive X axis
  //float state[3] = current_state();
  float closeness;
  int pos_neg_multiplier;

  do { //while( (closeness = abs(RPS.X() - end_x)) > 1. ){
    closeness = (RPS.Y() - end_y);
    pos_neg_multiplier = (((int)(closeness < 0)) << 1) - 1;		//gives +1 if you need to travel forward, -1 if backward

    //strong pulse motor if farther away
    if( closeness > 4){
      right_motor.SetPercent(65 * pos_neg_multiplier);
      left_motor.SetPercent(65 * pos_neg_multiplier);
    }
    else{
      right_motor.SetPercent(45 * pos_neg_multiplier);
      left_motor.SetPercent(45 * pos_neg_multiplier);
    }
  } while (closeness > .75);

}

void RPSChecker::check_45(float end_x){


}
// //////////////////////////////
void RPSChecker::check_heading(float heading){
  //Taken from lab 2
  float change = (int)abs(heading-RPS.Heading());
    if(change>180){ change = 360-change; }
    while(change > 2)
    {
        float curr_Heading = RPS.Heading();
        if(curr_Heading < heading || (curr_Heading > 270 && heading < 90)){    //Turn right
            right_motor.SetPercent(-50);
            left_motor.SetPercent(50);
        }
        else {  //Left
            right_motor.SetPercent(50);
            left_motor.SetPercent(-50);
        }

        Sleep(30);					//make sure it gets enough SLEEP!!
        change = (int)abs(heading-RPS.Heading());
        if(change>180){ change = 360-change; }


    }
    right_motor.SetPercent(0);
    left_motor.SetPercent(0);
}
// ///////////////////////////////////


void RPSChecker::check_any(float destination[]){


}

float RPSChecker::check_distance(float x, float y){
  return ( (*(new Boundary(*(new coordinate(RPS.X(), RPS.Y() ) ), *(new coordinate(x, y) ) ))).length() );	//Returns distance between current RPS coordinate and destination coordinate
}


///////////////////////////////////////



int main(void)
{
    ButtonBoard buttons( FEHIO::Bank3 );

    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

    while( true )
    {
        if( buttons.MiddlePressed() )
        {
            LCD.WriteLine( "Hello World!" );
            Sleep( 100 );
        }
    }
    return 0;
}
