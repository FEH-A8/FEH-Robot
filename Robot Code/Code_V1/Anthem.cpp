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
    void FlagDisplay();
    void AnthemDisplay(int);
    void recallPatriotism(int);
                                         //
///////////////////////////////////////////





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


  //Acts like the Sleep(int) function, but better, because AMERICA NEVER WOULD NEVER JUST SIT THERE.
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

    if(lineAtBeat(base_beat + duration / 560) == lineAtBeat(base_beat)){		//If the line at the end of the pause is the same as it is now, line won't be changed, so just sleep like normal
      Sleep(duration);
    }//if
    else {									//else, plan when to sing the next line(s)
      int lines_passed = 0;					//measures how many lines have passed
      Sleep(duration % 35);					//Sleeps the remainder time if the sleep isn't a multiple of the 35 ms beat
      while( TimeNowMSec() < end_time ){				//loop until sleep is over
        if(lineAtBeat(beatNow()) > base_lines + lines_passed){	//If the new line has come
          lines_passed++;
          AnthemDisplay(base_lines + lines_passed);
        }//end if
        Sleep(35);						//Sleep the beat length, so as to not wear down the battery as much
      }//end while
    }//end else

  }

                                         //
///////////////////////////////////////////


void FlagDisplay(){
    LCD.SetFontColor( FEHLCD::Blue );
    LCD.FillRectangle(0,0,100,60);

    LCD.SetFontColor( FEHLCD::White );
    LCD.FillRectangle(100,10,140,10);
    LCD.FillRectangle(100,30,140,10);
    LCD.FillRectangle(100,50,140,10);
    LCD.FillRectangle(0,70,240,10);
    LCD.FillRectangle(0,90,240,10);
    LCD.FillRectangle(0,110,240,10);

    LCD.SetFontColor( FEHLCD::Red );
    LCD.FillRectangle(100,0,140,10);
    LCD.FillRectangle(100,20,140,10);
    LCD.FillRectangle(100,40,140,10);
    LCD.FillRectangle(0,60,240,10);
    LCD.FillRectangle(0,80,240,10);
    LCD.FillRectangle(0,100,240,10);
    LCD.FillRectangle(0,120,240,10);
    
    LCD.SetFontColor( FEHLCD::White );
}



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

void forward(int);
void backward(int);

void driveToCrank(){	//Assumes that robot is lined up with crank
  int inches = 33;
  int counts = inches * 3.5;
  int left_motor_adjust = 0, right_motor_adjust = 0;
  float heading;
  int cycle_spent_off_track = 0;

  right_encoder.ResetCounts();
  left_encoder.ResetCounts();
  
  right_motor.SetPercent(70);
  left_motor.SetPercent(70);

  while( left_encoder.Counts() + right_encoder.Counts() / 2. < counts){
    heading = RPS.Heading();
    cycle_spent_off_track++;	//Assume robot is off track unbtil proven otherwise

    if(heading <= 90 - cycle_spent_off_track){		//needs minor correction
      left_motor_adjust = 0;
      right_motor_adjust++;
      if(heading <= 88 - cycle_spent_off_track){	//needs major correction		
	right_motor_adjust += 2;
      }
      right_motor.SetPercent(55 + right_motor_adjust);
      left_motor.SetPercent(55 + left_motor_adjust);
    }
    else if(heading >= 90 + cycle_spent_off_track){	//needs minor correction
      right_motor_adjust = 0;
      left_motor_adjust++;
      if(heading >= 92 + cyle_spent_off_track){	//needs major correction		
	left_motor_adjust += 2;
      } 
      right_motor.SetPercent(55 + right_motor_adjust);
      left_motor.SetPercent(55 + left_motor_adjust);
    }
    else{			//on track, reset cycles spent off track
      cycle_spent_off_track--;
    }
    Sleep(50);
  }

  //Stop Motors
  right_motor.Stop();
  left_motor.Stop();



}


void turnCrank(){

    float cds_value = CdS.Value();
    int cds_miscount = 0;       //counts changes to CdS miscount
    for (int i=0; i<4; i++){
        if (cds_value > .35){ //the light is blue

                            //CHANGED SOME STUFF HERE
            LCD.Write("THE LIGHT IS BLUE");
            servo.SetDegree(180);
            Sleep(1200);
            forward(2.5); //move forward into the crank
            Sleep(1000);
            servo.SetDegree(60);
            Sleep(1200);
            backward(1.2); //move backwkards an inch
            Sleep(1000);

            //read new value and increment if light reading changed
            cds_value = CdS.Value();
            if (cds_value <= .35 && cds_miscount<2){
                cds_miscount++;
                i = 0;
            }
            if(cds_miscount >= 2){
                cds_value = 0;      //stay within this loop if the robot changes its mind more than twice
            }


        } else{ //the light is red

            LCD.Write("THE LIGHT IS RED");
            servo.SetDegree(60);
            Sleep(1200);
            forward(2.5); //move forward into the crank
            Sleep(1000);
            servo.SetDegree(180);
            Sleep(1200);
            backward(1.2); //move backwkards an inch
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
      forward(2.4);
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
      forward(2.4);
      backward(2);
    }
    if(RPS.WhiteButtonPressed()){ return( true ); }

    return(false);
  }
  bool pressedBlue(){
    for(int i = 0; i<3; i++){
      if(RPS.BlueButtonPressed()){ return( true ); }

      //move back and try again
      forward(2.4);
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
     forward(2);
     pressedButton(order[j]);
     backward(2);
   }

}//push buttons





void forward(float inches){
    forward(55, inches);
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

    left_motor.SetPercent(-70);
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


void backward(int percent, int inches){
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

    left_motor.SetPercent(68);
    right_motor.SetPercent(-68);
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

    const int turnPercent = 50;
        float change = (int)(abs(heading-RPS.Heading()));
        if(change>180){ change = 360-change; }
        while(change > 4){
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

        LCD.WriteLine("Heading checked");
        Sleep(1000);
    } //check_heading
// ////////////////////////////////


int main(void)
{
float heading;
float saltStart = 62, saltUp = 92, saltDown = 162, saltRamp = 172, saltSwitch = 116, oil_push = 106, oil_pull = 120;

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

   //Initialize RPS
    RPS.InitializeMenu();

    //initialize positions of servo motors
    servo.SetDegree(180);
    servoSalt.SetDegree(saltStart);
    Sleep(1000);


    while(CdS.Value() > .6);

    //Order of steps from sheet
    //1
    servoSalt.SetDegree(saltUp);
    heading = RPS.Heading();
    backward(11);
    turn_left(43);

    LCD.Write("PART 1");

    //2
    backward(8);
    servoSalt.SetDegree(saltDown);
    Sleep(1200);
    forward(4);
    LCD.Write("PART 2");

    //3
    turn_right(100);
    forward(14);
    turn_left(47);
    LCD.Write("PART 3");

    //4 (before ramp)
    check_heading(heading);
    servoSalt.SetDegree(saltRamp);
    driveToCrank();
    check_heading(90);
    LCD.Write("PART 4");

    //5  at the crank
    turnCrank();
    LCD.Write("PART 5");




    //6-10  moving from crank to garage
    turn_right(90);
    check_heading(0);
    servoSalt.SetDegree(saltDown);
    backward(20);
    turn_right(45);
    check_heading(315);
    backward(5);


    //11  push salt into garage
    servoSalt.SetDegree(150);
    Sleep(1000);
    forward(6);
    servoSalt.SetDegree(saltDown);
    Sleep(1000);
    backward(6);
    forward(4);
    turn_left(95);

    //12 go to buttons
    forward(8);
    turn_left(87);
    check_heading(heading+45);
    pushButtons();

    //13 go to top of avalanche
    turn_right(90);
    backward(16);
    turn_left(45);
    servoSalt.SetDegree(82);
    backward(16);













    return 0;
} //main
