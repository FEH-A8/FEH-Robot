#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>


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
            Sleep( 1000 );
            _GLOBAL_START = TimeNowMSec();
            LCD.Clear( FEHLCD::Black );
            recallPatriotism(60000);
        }
    }
    return 0;
}



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

