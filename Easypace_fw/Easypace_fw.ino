/*

Firmware for Arduino-based pulsegenerator, applicable for, e.g., pacing of cardiac tissues
written by Oliver Schneider (ghilorevilo@posteo.de), September 2021

follow the instructions for assembling the hardware components and upload the firmware to your Arduino Nano

The pulsegenerator offers the output of 2 independently controllable biphasic pulse signals characterized by their:
- frequency (Hz)
- pulsewidth (ms)
- pulsevoltage (V)
- polarity (positive pulse or negative pulse first)
Each pulse signal is further output by the Opamp in 2 independent lines,
such that you can pace 4 samples simultaneously:
- DAC1 -> S1, S2; DAC2 -> S3, S4

sample output pulse:

Voltage

^        w
|      <-->
|      ####               #### ^ 
|      #  #               #  # | U
|      #  #               #  # |
|      #  #               #  # v
|#######--#--##############--#--##########----> time
|         #  #               #  #
|         #  #               #  #
|         #  #               #  #
|         ####               ####
|
|               f
|      <------------------>

the pulsegenerator can be controlled either 
- manually with the rotary encoder or
- remotely via Serial commands when connecting the Arduino via USB to a computer (baudrate set to 9600)
    supported serial commands:
    - GETPAR -> returns currently set pacing parameter
    - SETREM ON/OFF -> set pacer into remote (ON) or manual (OFF) mode
    - SETF f1 f2 -> sets frequency of channel 1 to f1, of channel 2 to f2
    - SETU U1 U2 -> sets voltage of channel 1 to U1, of channel 2 to U2
    - SETW w1 w2 -> sets pulsewidth of channel 1 to w1, of channel 2 to w2
    - SETPOL Pol1 Pol2 -> sets polarity of channel 1 to Pol1 (POS or NEG), of channel 2 to Pol2 (POS or NEG)
    - SETON ONstate1 ONstate2 -> sets pacing on channel 1 to Onstate1 (ON or OFF), of channel 2 to ONstate2 (ON or OFF)


notes: 
- due to individual variations in hardware components a slight drift of the zero-value might occur
  you can check for a drift by connecting a voltmeter to a signal-output (connect signal + ground),
  switch on the pacer and record the voltage in the idle state (pacing not turned on) U_idle.
  Then you can set the Variable DAC_correction to U_idle/0.0062
  (e.g. offset of +0.024 V -> DAC_correction = 0.024/0.0062 = 3.87 --> change line 74 to int DAC_correction = 4)
- timing is currently controlled by calling millis() which did not pose any issues for our applications (pulsewidths > 5 ms used)
  if timing is more crucial, calls to millis() might be replaced with micros()

    
*/

#include <MCP492X.h>            // DAC control
#include <LiquidCrystal_I2C.h>  // display control
#include <CommandParser.h>      // helper library for parsing of serial commands

#define PIN_SPI_CHIP_SELECT 10 // CS Pin 10
#define PIN_LED2 6
#define PIN_LED1 7
#define PIN_SHDN 8 // shutdown pin of DAC

#define PinA 2
#define PinB 3
#define PushButtonPin 4

int DAC_correction = 0;

//#######################################
// rotary encoder variables

#define rightTurn 2 // possible user interactions
#define leftTurn 3
#define pushButton 1
#define nothingPressed 0

volatile byte aFlag = 0; // control variables of encoder, based on https://www.instructables.com/Easy-Arduino-Menus-for-Rotary-Encoders/
volatile byte bFlag = 0;
volatile byte encoderPos = 0;
volatile byte oldEncPos = 0; // stores the last encoder position value 
volatile byte reading = 0;   // reading of interrupt pins  
boolean RotLeft_Flag = 0;    // flag to indicate leftwards rotation (-> false indicates right rotation)

bool newEncMovement = 0;
byte EncMovement = 0;

byte inputAction = nothingPressed;

bool PushBounceLock = LOW;  // for pushbutton debouncing
long PushLastPress = millis();
bool buttonPushed = LOW;

//#######################################
// variables for handling serial input (Parser)

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

boolean newData = false;

typedef CommandParser<7, 2, 6, 8, 32> MyCommandParser; 
// adjust parser to nr of expected commands to minimize memory consumption:
// parameters: 
// 7 stored commands,
// 2 allowed args for each command,
// 6 byte (= 6 chars) command name length,
// 8 byte max argument size, (e.g. 64-bit int would be 8 byte)
// 32 char max length of response message

MyCommandParser parser;

const char* POS = "POS";
const char* NEG = "NEG";
const char* ON = "ON";
const char* OFF = "OFF";

//#######################################
// menu options

// general control modes manual/ remote, stored in variable control_mode
#define manual 0
#define remote 1
byte control_mode = manual;

// menu options for manual control
#define mainMenu 0
#define adjustParameter 1

byte menustate = mainMenu;
byte menupage = 0;
byte cursor = 0;  // can be 0 or 1 (2 lines of display)
byte menuselection = 0; // = menupage + cursor

//#######################################
// variables for pacing control

// Signal 1 parameters
float S1_f = 1.0;       // frequency in Hz
float S1_w = 50.0;      // pulsewdth in ms
float S1_U = 5.0;       // voltage of biphasic pulse
float S1_T = (1/S1_f)*1000 - 2*S1_w;    // calculated period length in ms
bool S1_on = false;     // on/off pacing toggle
bool S1_pol = false;      // polarity of first pulse: false -> - first, true -> + first

// Signal 2 parameters
float S2_f = 1.0;       // frequency in Hz
float S2_w = 50.0;      // pulsewdth in ms
float S2_U = 5.0;       // voltage of biphasic pulse
float S2_T = (1/S2_f)*1000 - 2*S2_w;    // calculated period length in ms
bool S2_on = false;     // on/off pacing toggle
bool S2_pol = false;      // polarity of first pulse: false -> - first, true -> + first

// variables used for adjusting signal parameters
// specify range for allowed parameter values
float adj_f = S1_f; // (use just one buffer variable here in future?)
float min_f = 0.3;  // (use define here in future?)
float max_f = 5.0;
float step_f = 0.1;

float adj_w = S1_w;
float min_w = 1.0;
float max_w = 300;
float step_w = 1.0;

float adj_U = S1_U;
float min_U = 0.0;
float max_U = 12.5;
float step_U = 0.1;

bool adj_on = S1_on;
bool adj_pol = S1_pol;

// timing variables 
long currtime = millis();
long switchtime = currtime;     // switchtime used for blinking animation in ui during setting of param
long updatetime = switchtime;   // updatetime used to control how often parameterchanges are updated on the display
bool blinking = HIGH;

//##############################
// variables for pulse states

#define state_zero 0
#define state_p1 1
#define state_p2 2

byte S1_pulse_state = state_zero;
byte S2_pulse_state = state_zero;

long S1_statetime = 0;
long S2_statetime = 0;

// specifies DAC-value for pulse states (p1, p2, zero)
// is calculated later from set voltage and polarity order
int S1_DAC_p1 = 0;
int S2_DAC_p1 = 0;

int S1_DAC_p2 = 0;
int S2_DAC_p2 = 0;

int S1_DAC_zero = 0;
int S2_DAC_zero = 0;


//##############################
// variables for adjustment menu

char parameter_to_adjust;
const int refreshtime = 50;     // msec for refreshing currently adjusted parameter
const int blinktime_on = 800;   // msec for blinking mode in menuselection
const int blinktime_off = 400;  // msec for blinking mode in menuselection
int blinktime = blinktime_on;

//################################
// display

LiquidCrystal_I2C lcd(0x27, 16, 2); // set up display type: 16x2
MCP492X myDac(PIN_SPI_CHIP_SELECT); // init DAC

//################################
// DAC control functions

// calculate needed DAC output to get desired voltage V
// 12-bit DAC -> 2^12 = 4096 values -> can take values from 0-4095
// Opamp converts signal: (U_DAC-2.5V)*(56kO/11kO)
// output 0 - 4095 gets converted to -12.727 V - 12.727 V
// theoret. output: 4095 to get 12.727 V
int calc_DAC(float V) {
    int DACOut = int(((V + 12.727)/25.454) * 4095.0)-DAC_correction;
    return DACOut;
}

// sets values S1_DAC_p1, S1_DAC_p2, S1_DAC_zero (= DAC output for specific state)
// corresponding to desired signal voltage

// so far does calculation for both channels
// todo: can be definitely improved to do on specified channel also
// as done only once not considered too time critical right now
// arrays, e.g. U[channel] would definitely help
void set_DACvals(){
    
    // pulsestates for channel 1
    if (S1_pol == true) {   // -> first state S1_DAC_p1 positive voltage, second S1_DAC_p2 negative voltage
    S1_DAC_p1 = calc_DAC(-S1_U);
    S1_DAC_p2 = calc_DAC(S1_U); }
    
    else { // -> first pulse negative
    S1_DAC_p1 = calc_DAC(S1_U);
    S1_DAC_p2 = calc_DAC(-S1_U); }
    
    // pulsestates for channel 2, similar to above
    if (S2_pol == true) {
    S2_DAC_p1 = calc_DAC(-S2_U);
    S2_DAC_p2 = calc_DAC(S2_U); }
    
    else {
    S2_DAC_p1 = calc_DAC(S2_U);
    S2_DAC_p2 = calc_DAC(-S2_U); }    
    
    // set zerostates for both channels (should be same)
    S1_DAC_zero = calc_DAC(0.0);
    S2_DAC_zero = S1_DAC_zero;
}

// sets DAC output of channel chan to specified state
// uses precalculated DAC-values S1_DAC_p1, S1_DAC_p2, S1_DAC_zero

// chan is 1 or 2, state is state_zero, state_p1, state_p2
// for correct writing to DAC via library:
// set gain + enable bits explicitely, otherwise gain would be 1
void write_DAC(int chan, byte state) {
    switch(state) {
        case state_zero:
            if (chan == 1) {myDac.analogWrite(0, 0, 0, 1, S1_DAC_zero);}
            else if (chan == 2) {myDac.analogWrite(1, 0, 0, 1, S2_DAC_zero);}
            // todo: ugly solution, can be simplified to array DAC_zero[channel]
        break;

        case state_p1:
            if (chan == 1) {myDac.analogWrite(0, 0, 0, 1, S1_DAC_p1);}
            else if (chan == 2) {myDac.analogWrite(1, 0, 0, 1, S2_DAC_p1);}
         
        break;
        
        case state_p2:
            if (chan == 1) {myDac.analogWrite(0, 0, 0, 1, S1_DAC_p2);}
            else if (chan == 2) {myDac.analogWrite(1, 0, 0, 1, S2_DAC_p2);}
        break;
    }
}

//################################
// Arduino setup function

void setup() {
    myDac.begin();  // take care, as VOut = 0 V -> output to pacer will be -12.7 V, leading to an initial pulse during startup
    setupPins();
    updatePacer();  // get DAC as quickly as possible to put out 2.5 V -> 0V after OPAMP    
    lcd.begin();
    
    startscreen();
    print_MainMenu(menupage,cursor);
    
    Serial.begin(9600); // todo: think about where to place, start here already or enable manually? 
    
    // link input commands to callback functions
    // "dd" means 2 doubles as input, e.g. "SETF 1.0 1.5"
    parser.registerCommand("SETREM", "s", &setrem);  //
    parser.registerCommand("GETPAR", "", &getpar);  //
    parser.registerCommand("SETF", "dd", &setf);    //
    parser.registerCommand("SETU", "dd", &setu);    //
    parser.registerCommand("SETW", "dd", &setw);    //
    parser.registerCommand("SETPOL", "ss", &setpol);//
    parser.registerCommand("SETON", "ss", &seton);  //
}

//################################
// Arduino loop

void loop() {
    readInput();    // continuously check for input
    handleInput();
    runPacer();     // constantly perform required DAC state changes for correct pacing
}

// setting up pins on first startup
void setupPins()  {
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_SHDN, OUTPUT);
  digitalWrite(PIN_LED1, HIGH);
  digitalWrite(PIN_LED2, HIGH);
  digitalWrite(PIN_SHDN, HIGH); // enable DAC
  
  pinMode(PinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage  
  pinMode(PinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage  
  attachInterrupt(0,PinA_INT,RISING); // set interrupt on PinA, looking for a rising edge 
  attachInterrupt(1,PinB_INT,RISING);
  pinMode(PushButtonPin,INPUT_PULLUP);
  }

// interrupt for PinA
void PinA_INT(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and 
                                      //that we are expecting detent on this pin's rising edge
  EncMovement = rightTurn;
  newEncMovement = 1;
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal 
                                            //the transition to detent from free rotation
  sei(); //restart interrupts
}

// interrupt for PinB
void PinB_INT(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and 
                                       //that we are expecting detent on this pin's rising edge
  EncMovement = leftTurn;
  newEncMovement = 1;
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal 
                                            //the transition to detent from free rotation
  sei(); //restart interrupts
}
  
// startup screen
void startscreen()  {
  lcd.setCursor(0,0);
  lcd.print("  EasyPace 2.0  ");
  lcd.setCursor(0,1);
  lcd.print(" ready to pace  ");

  for (int i=17; i>2; i--)
  {
    
    int delaytime = i*i*2;
    delay(delaytime);
    digitalWrite(PIN_LED1, HIGH);
    delay(70);
    digitalWrite(PIN_LED2, HIGH);
    delay(30);
    digitalWrite(PIN_LED1, LOW);
    delay(30);
    digitalWrite(PIN_LED2, LOW);
  }
  
  lcd.clear();
}

// print menu structure upon navigation
void print_MainMenu(byte menupage, byte Cursor){
  lcd.clear();
  //or do it as array char mainMenu_Entries[][17] = {"running:", "set par", "quick U:", "set U"};
  switch(menupage) {
    case 0:
    lcd.setCursor(0,0);
    lcd.print(" running:");
  if(S1_on == true) {lcd.print("ON");} else {lcd.print("OFF");}
    lcd.setCursor(0,1);
    lcd.print(" set f:"); lcd.print(S1_f);
    break;
    case 1:
    lcd.setCursor(0,0);
    lcd.print(" set f:"); lcd.print(S1_f);
    lcd.setCursor(0,1);
    lcd.print(" set W:"); lcd.print(S1_w);
    break;
    case 2:
    lcd.setCursor(0,0);
    lcd.print(" set W:"); lcd.print(S1_w);
    lcd.setCursor(0,1);
    lcd.print(" set U:"); lcd.print(S1_U);
    break;

    case 3:
    lcd.setCursor(0,0);
    lcd.print(" set U:"); lcd.print(S1_U);
    lcd.setCursor(0,1);
    lcd.print(" set polar:");
    if(S1_pol == true) {lcd.print("+");} else {lcd.print("-");}
    break;

  }
  
  lcd.setCursor(0,Cursor);
  lcd.print(">");
}

// actions to perform on encoder input left/right/push
// perhaps easier: array of function pointer? No switch needed anymore -> action[0/1/2]
void mainMenu_actions()  {
  
  switch (inputAction)  {
    case leftTurn:    // decrease counter of program step    
        if (menupage > 0 && cursor == 0) {menupage--; print_MainMenu(menupage,cursor);}
    else if (cursor == 1) {cursor--; print_MainMenu(menupage,cursor);}// decrement cursor by 1 
    break;
    
    case rightTurn: // increase counter of program step
    if (menupage < 3 && cursor == 1)  {menupage++; print_MainMenu(menupage,cursor);}
    else if (cursor == 0)  {cursor++; print_MainMenu(menupage,cursor);}      
    break;
    
    case pushButton:
    menuselection = menupage + cursor;
    switchtime = millis(); updatetime = millis();
    lcd.clear();
      
        switch (menuselection)  {
        case 0: // running
          parameter_to_adjust = 'o';
          adj_on = S1_on;
          menustate = adjustParameter;
        break; 
        
        case 1: // set f
          parameter_to_adjust = 'f';
          adj_f = S1_f;
          menustate = adjustParameter;    
        break;
        
        case 2: // set W
          parameter_to_adjust = 'w';
          adj_w = S1_w;
          menustate = adjustParameter;
        break;
          
        case 3: // set U
          parameter_to_adjust = 'U';
          adj_U = S1_U;
          menustate = adjustParameter;      
        break;
        
        case 4: // set pulse polarity
          parameter_to_adjust = 'p';
          adj_pol = S1_pol;
          menustate = adjustParameter;      
        break;        
          } 
      break;
    
    case nothingPressed:
      break;
  }  
}

void adjustParameter_actions(char parameter)  {
// update visible variable
// possible parameters: 'o' (on/off), 'f', 'w', 'U'
// longer char as input?
   
  print_adjustParameter(parameter);
  
  switch (inputAction)  {
    case leftTurn:
    if (parameter == 'o') {
        if (adj_on == true)  {adj_on = false;}
      }
  
    else if (parameter == 'f')  {
        if (adj_f > min_f)  {adj_f = adj_f-step_f;}
    }
    
    else if (parameter == 'w')  {
    if (adj_w > min_w)  {adj_w = adj_w-step_w;}
    }
    
    else if (parameter == 'U')  {
    if (adj_U > min_U)  {adj_U = adj_U-step_U;}
    }    
    
    else if (parameter == 'p')  {
    if (adj_pol == true)  {adj_pol = false;}
    }
    
    break;
    
  case rightTurn:
    if (parameter == 'o') {
        if (adj_on == false)  {adj_on = true;}
      }
  
    else if (parameter == 'f')  {
        if (adj_f < max_f)  {adj_f = adj_f+step_f;}
    }
    
    else if (parameter == 'w')  {
        if (adj_w < max_w)  {adj_w = adj_w+step_w;}
    }
    
    else if (parameter == 'U')  {
        if (adj_U < max_U)  {adj_U = adj_U+step_U;}
    }
    
    else if (parameter == 'p') {
        if (adj_pol == false)  {adj_pol = true;}
      }
      
    break; 
    
    case pushButton:
    updatePacer();
    menustate = mainMenu;
    print_MainMenu(menupage, cursor);
    break;
  
    case nothingPressed:
      break;  
  }

}

// function called to update ui during parameter change
void print_adjustParameter(char parameter)  {

  currtime = millis();
  if (parameter == 'o') {
    if(currtime-switchtime > blinktime) {
    switchtime = currtime;
    if (blinking == LOW) {blinking = HIGH; blinktime = blinktime_on; lcd.setCursor(0,cursor); lcd.print("> running:"); lcd.print(adj_on);}
    else {blinking = LOW; blinktime = blinktime_off; lcd.clear(); lcd.setCursor(0,cursor); lcd.print("> running:"); }
    }
    
    if (blinking == HIGH)
    {
      if(currtime-updatetime > refreshtime) {updatetime = currtime; lcd.setCursor(0,cursor); lcd.print("> running:"); lcd.print(adj_on);}
    }   
  }

  else if (parameter == 'p') {
    if(currtime-switchtime > blinktime) {
    switchtime = currtime;
    if (blinking == LOW) {blinking = HIGH; blinktime = blinktime_on; lcd.setCursor(0,cursor); lcd.print("> set polar:"); 
        if (adj_pol == true)  {lcd.print("+");} else {lcd.print("-");}}
    else {blinking = LOW; blinktime = blinktime_off; lcd.clear(); lcd.setCursor(0,cursor); lcd.print("> set polar:"); }
    }
    
    if (blinking == HIGH)
    {
      if(currtime-updatetime > refreshtime) {updatetime = currtime; lcd.setCursor(0,cursor); lcd.print("> set polar:");
        if (adj_pol == true)  {lcd.print("+");} else {lcd.print("-");}}
    }   
  }
 
  else if (parameter == 'f'){
    if(currtime-switchtime > blinktime) {
    switchtime = currtime;
    if (blinking == LOW) {blinking = HIGH; blinktime = blinktime_on; lcd.setCursor(0,cursor); lcd.print("> set f:"); lcd.print(adj_f);}
    else {blinking = LOW; blinktime = blinktime_off; lcd.clear(); lcd.setCursor(0,cursor); lcd.print("> set f:"); }
    }

    if (blinking == HIGH)
    {
      if(currtime-updatetime > refreshtime) {updatetime = currtime; lcd.setCursor(0,cursor); lcd.print("> set f:"); lcd.print(adj_f);}
    }
  }
  
  else if (parameter == 'w'){
    if(currtime-switchtime > blinktime) {
    switchtime = currtime;
    if (blinking == LOW) {blinking = HIGH; blinktime = blinktime_on; lcd.setCursor(0,cursor); lcd.print("> set w:"); lcd.print(adj_w);}
    else {blinking = LOW; blinktime = blinktime_off; lcd.clear(); lcd.setCursor(0,cursor); lcd.print("> set w:"); }
    }

    if (blinking == HIGH)
    {
      if(currtime-updatetime > refreshtime) {updatetime = currtime; lcd.setCursor(0,cursor); lcd.print("> set w:"); lcd.print(adj_w);}
    }
  }
  
  else if (parameter == 'U'){
    if(currtime-switchtime > blinktime) {
    switchtime = currtime;
    if (blinking == LOW) {blinking = HIGH; blinktime = blinktime_on; lcd.setCursor(0,cursor); lcd.print("> set U:"); lcd.print(adj_U);}
    else {blinking = LOW; blinktime = blinktime_off; lcd.clear(); lcd.setCursor(0,cursor); lcd.print("> set U:"); }
    }

    if (blinking == HIGH)
    {
      if(currtime-updatetime > refreshtime) {updatetime = currtime; lcd.setCursor(0,cursor); lcd.print("> set U:"); lcd.print(adj_U);}
    }
  }

}

// provide structured start of pacer when parameters are changed, also run once on start
void updatePacer(){
  
  // assign changed "adj_"-variables to param
  // todo: some repetitions as all param are reassigned, however not too time-critical
  if (control_mode == manual) {

      S1_f = adj_f;
      S1_w = adj_w;
      S1_on = adj_on;
      S1_U = adj_U;
      S1_T = (1/adj_f)*1000 - 2*S1_w ;
      S1_pol = adj_pol;
      
      // manual control does not discriminate between signals
      // broadcast values of signal 1 to signal 2
      S2_f = S1_f;
      S2_w = S1_w;
      S2_on = S1_on;
      S2_U = S1_U;
      S2_T = S1_T;
      S2_pol = S1_pol;
      }
  
  // in remote control mode variables are directly set, calculate only T
  else if (control_mode == remote){
      S1_T = (1/S1_f)*1000 - 2*S1_w ;
      S2_T = (1/S2_f)*1000 - 2*S2_w ;
      }
  
  S1_pulse_state = state_zero;
  S2_pulse_state = state_zero;            
  
  set_DACvals(); // calc DAC values, assign depending on polarity
  write_DAC(1, state_zero); // set DAC to zerostate
  write_DAC(2, state_zero); 
  
  digitalWrite(PIN_LED1, LOW); 
  digitalWrite(PIN_LED2, LOW);
  S1_statetime = millis();
  S2_statetime = S1_statetime;
}

// function is continuously checked to perform pacing tasks on correct timing
void runPacer(){
    
  // set pacingstate for signal 1
  // todo: repetition super ugly again, can be circumvented by changing state parameters 
  // into array on[channel], switch pulse_state[channel], ...
  if(S1_on == true) {
    currtime = millis();    // todo: use micros() if better accuracy wanted
    
    switch(S1_pulse_state) { // change voltage/ transition to next state depending on individual times
        case state_zero:
          if (currtime-S1_statetime > S1_T) {   // transfer to first pulse if singal period elapsed
              S1_pulse_state = state_p1; 
              digitalWrite(PIN_LED1, HIGH); 
              //digitalWrite(PIN_LED2, HIGH); 
              S1_statetime = currtime;
              write_DAC(1, state_p1);}
        break;
        
        case state_p1:
          if (currtime-S1_statetime > S1_w) {   // transfer to second pulse
              S1_pulse_state = state_p2; 
              S1_statetime = currtime;
              write_DAC(1, state_p2);}
        break;
        
        case state_p2:
          if (currtime-S1_statetime > S1_w) {   // transfer to state_zero
              S1_pulse_state = state_zero; 
              digitalWrite(PIN_LED1, LOW); 
              //digitalWrite(PIN_LED2, LOW); 
              S1_statetime = currtime;
              write_DAC(1, state_zero);}
        break;   
    }
  }
  
  if(S2_on == true) {
    currtime = millis();
    
    switch(S2_pulse_state) { // change voltage/ transition to next state depending on individual times
        case state_zero:
          if (currtime-S2_statetime > S2_T) {   // transfer to first pulse if singal period elapsed
              S2_pulse_state = state_p1; 
              digitalWrite(PIN_LED2, HIGH);
              S2_statetime = currtime;
              write_DAC(2, state_p1);}
        break;
        
        case state_p1:
          if (currtime-S2_statetime > S2_w) {   // transfer to second pulse
              S2_pulse_state = state_p2; 
              S2_statetime = currtime;
              write_DAC(2, state_p2);}
        break;
        
        case state_p2:
          if (currtime-S2_statetime > S2_w) {   // transfer to state_zero
              S2_pulse_state = state_zero; 
              digitalWrite(PIN_LED2, LOW);
              S2_statetime = currtime;
              write_DAC(2, state_zero);}
        break;   
    }
  }
  
}

// function is polled in every menu to check for user input; sets variable inputAction afterwards
void readInput()  {

  // handle rotary encoder input
  inputAction = nothingPressed;

  // read encoder turns
  if (newEncMovement == 1)  {inputAction = EncMovement; newEncMovement = 0;}
  
  // debounced reaading of PushButton
  if (PushBounceLock){if (millis()-PushLastPress > 300) PushBounceLock = LOW;}  // shut down PushBounceLock after 300 ms
  else{
    buttonPushed = !digitalRead(PushButtonPin);
    if (buttonPushed == HIGH) {PushBounceLock = HIGH; PushLastPress = millis(); inputAction = pushButton;}
  }

  // handle serial input
  // both inputs should work simultaneously
  read_unblocked_serial();

}

// unblocked reading of serial port, read until newline detected
// finished reading indicated by newData = true, receivedChars filled
// taken from https://forum.arduino.cc/t/serial-input-basics/278284
// corresponds to recvWithEndMarker
void read_unblocked_serial() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void handleInput()  {

  // perform action selected with rotary encoder if manual mode active
  if (control_mode == manual){
    switch(menustate)  {
    
    case mainMenu:
      mainMenu_actions(); // interpret input actionsperform actions for mainMenu
      break;
    
    case adjustParameter:
      adjustParameter_actions(parameter_to_adjust);
      break;
    }
  }

// handle serial commands
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    // because strtok() used in parseInput() replaces the commas with \0
    
    char response[MyCommandParser::MAX_RESPONSE_SIZE];
    parser.processCommand(tempChars, response);
    Serial.println(response);
    
    newData = false;
  }
}

//##############################
// serial callback functions

// setf -> set frequency
void setf(MyCommandParser::Argument *args, char *response) {
    
    // check if device in remote control mode, return if not
    if (control_mode != remote) {
        strlcpy(response, "> can't set, not in remote mode", MyCommandParser::MAX_RESPONSE_SIZE);
        return;
    }
    
    // get inputargs, round to 1 decimal
    float f1 = round(args[0].asDouble*10)/10.0;
    float f2 = round(args[1].asDouble*10)/10.0;
    
    // check if input value in correct range
    // todo: syntax correct? can be quite probably improved..
    if ((f1 <= max_f) and (f1 >= min_f) and (f2 <= max_f) and (f2 >= min_f)) {
        S1_f = f1;  // check: can double be assigned to float?.. for instance declared f1 and f2 as floats
        S2_f = f2;
        Serial.print("> set f1: "); Serial.print(S1_f);
        Serial.print(" f2: "); Serial.println(S2_f);
        strlcpy(response, "> setf succeeded", MyCommandParser::MAX_RESPONSE_SIZE);
        updatePacer(); // important, restart pacer with newly set parameters
    }
    else {strlcpy(response, "> setf failed", MyCommandParser::MAX_RESPONSE_SIZE);}
}

// setu -> set voltage
void setu(MyCommandParser::Argument *args, char *response) {

    // check if device in remote control mode, return if not
    if (control_mode != remote) {
        strlcpy(response, "> can't set, not in remote mode", MyCommandParser::MAX_RESPONSE_SIZE);
        return;
    }
    
    // get inputargs, round to 1 decimal
    float u1 = round(args[0].asDouble*10)/10.0;
    float u2 = round(args[1].asDouble*10)/10.0;
    
    // check if input value in correct range
    // todo: syntax correct? can be quite probably improved..
    if ((u1 <= max_U) and (u1 >= min_U) and (u2 <= max_U) and (u2 >= min_U)) {
        S1_U = u1;
        S2_U = u2;
        Serial.print("> set u1: "); Serial.print(S1_U);
        Serial.print(" u2: "); Serial.println(S2_U);
        strlcpy(response, "> setu succeeded", MyCommandParser::MAX_RESPONSE_SIZE);
        updatePacer();
    }
    else {strlcpy(response, "> setu failed", MyCommandParser::MAX_RESPONSE_SIZE);}
}

// setw -> set pulsewidth
void setw(MyCommandParser::Argument *args, char *response) {

    // check if device in remote control mode, return if not
    if (control_mode != remote) {
        strlcpy(response, "> can't set, not in remote mode", MyCommandParser::MAX_RESPONSE_SIZE);
        return;
    }
    
    // get inputargs, round to 1 decimal
    float w1 = round(args[0].asDouble)*1.0; // check if *1.0 needed
    float w2 = round(args[1].asDouble)*1.0;
    
    // check if input value in correct range
    // todo: syntax correct? can be quite probably improved..
    // or use && as and...
    if ((w1 <= max_w) and (w1 >= min_w) and (w2 <= max_w) and (w2 >= min_w)) {
        S1_w = w1;
        S2_w = w2;
        Serial.print("> set w1: "); Serial.print(S1_w);
        Serial.print(" w2: "); Serial.println(S2_w);
        strlcpy(response, "> setw succeeded", MyCommandParser::MAX_RESPONSE_SIZE);
        updatePacer();
    }
    else {strlcpy(response, "> setw failed", MyCommandParser::MAX_RESPONSE_SIZE);}
}

// setpol -> set polarity
void setpol(MyCommandParser::Argument *args, char *response) {

    // check if device in remote control mode, return if not
    if (control_mode != remote) {
        strlcpy(response, "> can't set, not in remote mode", MyCommandParser::MAX_RESPONSE_SIZE);
        return;
    }
    
    // validate input, POS, NEG as arg allowed
    // check: save "POS" in variable/ const to save memory?
    // or use || for or operator

    if ((strcmp(args[0].asString, POS) == 0) or (strcmp(args[0].asString, NEG) == 0) and
        (strcmp(args[1].asString, POS) == 0) or (strcmp(args[1].asString, NEG) == 0)) {
            
        S1_pol = (strcmp(args[0].asString, POS) == 0);  // if arg == POS -> pos. pulse first, indicated by S1_pol = true
        S2_pol = (strcmp(args[1].asString, POS) == 0);
        Serial.print("> set pol1: "); Serial.print(S1_pol);
        Serial.print(" pol2: "); Serial.println(S2_pol);
        strlcpy(response, "> setpol succeeded", MyCommandParser::MAX_RESPONSE_SIZE);
        updatePacer();        
        }
    else {strlcpy(response, "> setpol failed", MyCommandParser::MAX_RESPONSE_SIZE);}
}

// seton -> set channel on/ off
void seton(MyCommandParser::Argument *args, char *response) {

    // check if device in remote control mode, return if not
    if (control_mode != remote) {
        strlcpy(response, "> can't set, not in remote mode", MyCommandParser::MAX_RESPONSE_SIZE);
        return;
    }
    
    // validate input, ON, OFF as arg allowed
    if ((strcmp(args[0].asString, ON) == 0) or (strcmp(args[0].asString, OFF) == 0) and
        (strcmp(args[1].asString, ON) == 0) or (strcmp(args[1].asString, OFF) == 0)) {
            
        S1_on = (strcmp(args[0].asString, ON) == 0);  // if arg == ON -> S1_on = true
        S2_on = (strcmp(args[1].asString, ON) == 0);
        Serial.print("> set on1: "); Serial.print(S1_on);
        Serial.print(" on2: "); Serial.println(S2_on);
        strlcpy(response, "> seton succeeded", MyCommandParser::MAX_RESPONSE_SIZE);
        updatePacer();        
        }
    else {strlcpy(response, "> seton failed", MyCommandParser::MAX_RESPONSE_SIZE);}
}

// setrem -> place pacer into remote mode
void setrem(MyCommandParser::Argument *args, char *response) {
    
    // check if valid arg specified
    // ON as arg
    if (strcmp(args[0].asString, ON) == 0) {
        strlcpy(response, "> changing to remote mode", MyCommandParser::MAX_RESPONSE_SIZE);
        
        lcd.setCursor(0,0);
        lcd.print("################");
        lcd.setCursor(0,1);        
        lcd.print("#remote control#");
    
        control_mode = remote;
        updatePacer();
    }
    
    // OFF as arg
    else if (strcmp(args[0].asString, OFF) == 0){
        strlcpy(response, "> changing to manual mode", MyCommandParser::MAX_RESPONSE_SIZE);
        control_mode = manual;
        
        // assign adj_ variables for correct transfer of remote values to manual
        adj_f = S1_f;
        adj_w = S1_w;
        adj_on = S1_on;
        adj_U = S1_U;
        adj_pol = S1_pol;      
        
        updatePacer();
        // update display
        menupage = 0;
        cursor = 0;
        print_MainMenu(menupage, cursor);
    }
    
    // unknown arg
    else {strlcpy(response, "> setrem failed", MyCommandParser::MAX_RESPONSE_SIZE);       
    }
}

// displays currently set pace parameters
void getpar(MyCommandParser::Argument *args, char *response) {
    Serial.println("> Currently set pacing param:");
    Serial.print("> f [Hz]:"); Serial.print(S1_f); Serial.print(" "); Serial.println(S2_f);
    Serial.print("> U [V]:"); Serial.print(S1_U); Serial.print(" "); Serial.println(S2_U);
    Serial.print("> w [ms]:"); Serial.print(S1_w); Serial.print(" "); Serial.println(S2_w);
    Serial.print("> polarity:"); Serial.print(S1_pol); Serial.print(" "); Serial.println(S2_pol);
    Serial.print("> on:"); Serial.print(S1_on); Serial.print(" "); Serial.println(S2_on);
    
    strlcpy(response, ">", MyCommandParser::MAX_RESPONSE_SIZE);
}