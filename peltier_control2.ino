/**
   Peltier controller arduino project.
   History:
   2015.??.?? - Initial version created
   2018.07.29 - Revised to open this project - 

 */

#define REVISION_STRING         "Rev 2.00"
#include <PV_RTD_RS232_RS485_Memory_Map.h>
#include <PV_RTD_RS232_RS485_Shield.h>

// include the library code:
#include <math.h>
#include <Wire.h>
#include <wiring_private.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include <PID_v1.h>
#include <EEPROM.h>
#include "peltier_control.h"

// ---------------------- funtion prototype
// ----------------------------------------------LCD DISPLAY
//   DISPLAY: STARTUP SCREEN to LCD ( version etc. )
void show_startup();
//   MAIN MENU ENTRY - control periodic update and common button handling.
void menu(void);
//   context menu function typedef
//   CONTEXT MENU MAIN UI TARGET TEMPERATURE / CURRENT TEMPERATURE / ERROR REPORT
void main_ui(uint8_t button_now, uint8_t button_prev, uint16_t button_keep_pressed_counter);
//   DISPLAY: MAIN UI to LCD
void display_main_lcd();
//   CONTEXT MENU - temperature detail
void temperature_detail_ui(uint8_t button_now, uint8_t button_prev, uint16_t button_keep_pressed_counter);
//   DISPLAY: ALL TEMPERATURE OUTPUT to LCD
void display_temperature_detail_to_lcd(void);
//   CONTEXT MENU - PID
void pid_ui(uint8_t button_now, uint8_t button_prev, uint16_t button_keep_pressed_counter);
void version_ui(uint8_t button_now, uint8_t button_prev, uint16_t button_keep_pressed_counter);
void stability_ui(uint8_t button_now, uint8_t button_prev, uint16_t button_keep_pressed_counter);
//   output float "%f.1" to LCD
void print_float(float f);
void float_to_int(float f, int* integer_part, int* mantissa);

// common button handling
eButtonAction common_button_handling( uint8_t button_to_be_checked, uint8_t button_now, uint8_t button_prev, uint16_t button_keep_pressed_counter , uint16_t* context);


void updateLCDLine( int line , const char str[16+1] );
void updateLCD( const char display_lines[16*2+1]);

// ----------------------------------------------LCD DISPLAY

// ----------------------------------------------CONTROL/SENSE

//   PSU control
void power_supply_control();
//   update peltier PWM output and current sense.
void update_intensity();
//   set peltier PWN from global status.
void output_peltier_intensity();
//   convert ANALOG READ value ( 0-1023 ) to VNH2SP30 CS VALUE ( amp ).
float convert_current_sense2amp(int cs_analogread_value);
//   I2C temperature sensor read
void get_thermocouple_temp();
float getTemperature(int i2c_addr);
//   READ THERMISTOR RESISTANCE FROM PINs and UPDATE it to GLOBAL STATUS
void get_thermistor_temp();
//   record temp history and get average
void record_history();
//   update pid target temperature according to temperature record.
void update_pid_target();
//   RTD temp measurement
void init_rtd_temp();
void get_rtd_temp();


//   for PID UI
void serial_processing(void);
void SerialReceive();
void SerialSend();
// ----------------------------------------------CONTROL/SENSE

// ----------------------------------------------STATUS MANAGEMENT
//   check condition is OK and if not, set error to global status.
void check_condition();
//   reset error status
void reset_error();
// ----------------------------------------------STATUS MANAGEMENT
// ----------------------------------------------EEPROM READ/WRITE
void updatePIDforEEPROM(double p, double i, double d);
void readSaveDataFromEEPROM(eeprom_save_data_t* buf);
void updateSaveDataToEEPROM(const eeprom_save_data_t* buf);
double ReadDoubleEEPROM( int addr );
void   WriteDoubleEEPROM( int addr, double value );
// ----------------------------------------------EEPROM READ/WRITE
//
void setup();
void loop();
// ---------------------- end function prototype
  


// ##############################################CONSTANTS
 
#if 0
const int reference_voltage = 33; //3.3v
#else
const int reference_voltage = 50; //5.0v
#endif


//Default PID value
const float DEFAULT_PID_P = 3.0;
const float DEFAULT_PID_I = 1.0;
const float DEFAULT_PID_D = 0.0;


// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

const int fan_output_pin = 6;
const int tmp_0_addr=0x48;
const int tmp_1_addr=0x49;
const int tmp_2_addr=0x4A;

// PSU control pin
const int PSON_PIN = 11;

//FAN PWN CONTROL --- place holder....

//VNHS2P30
const int INA_PIN = 13;
const int ENA_PIN = 12;
const int PWM_PIN = 3;
const int peltier_cool_output_pin = PWM_PIN;
const int CS_PIN = 3;/*ANALOG PIN3*/
const int ENB_PIN = 8;
const int INB_PIN = 7;

// What pin to connect the sensor to
#define NUM_THERMISTOR 2
#define THERMISTORPIN0 A0 
#define THERMISTORPIN1 A1 
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 12
// dealy per sample
//#define DELAY_PER_SAMPLE (10/NUM_THERMISTOR)
#define DELAY_PER_SAMPLE 5
// The beta coefficient of the thermistor (usually 3000-4000)
//#define BCOEFFICIENT 3950
#define BCOEFFICIENT 3435 //103-AT/103-JT
// the value of the 'other' resistor
#define SERIESRESISTOR 10000


// ##############################################END CONSTANTS

// ########MAIN STATUS STRUCT######################################
typedef struct {
  //--for PID--
  double setpoint;
  double input;
  double output;
  //---
  float target_temp; /* RTD - connected to ProtoVolatics board. */
  float env_temp; /* thermo couple - Air environment temp - new for 2.0. */
  float temp_out[NUM_THERMISTOR]; /* cooler side and hot side */
#define HIGH_SIDE_TEMP 0
#define LOW_SIDE_TEMP 1
  int	intensity; /* current pwm output value -255 -> 255*/
  int   last_intensity;
  int   current_sense;
  float current_sense_amp;
  bool  over_current;
  bool  over_temp;
  int   ps_on; /* LOW / HI */
  //#define MAX_TEMP_HISTORY 6
#define MAX_TEMP_HISTORY 12 /* 6 was too short */
#define TEMP_SAMPLING_PERIOD 1 //sec
#define TEMP_AVG_DURATION 10 //sec
#define TEMP_ADJUST_FACTOR	(0.8f)
  float peltier_temp_hist[MAX_TEMP_HISTORY];
  float target_temp_hist[MAX_TEMP_HISTORY];
  int   hist_count; // history counter
  double pid_target_temp; // PID target temperature - calculation result -
  
}status_t;

//////////////////////////////////////////////////////////////////////////////////
//---- global variables ----
status_t g_status={0};// status
eeprom_save_data_t g_save_data={0};// save data on EEPROM
PV_RTD_RS232_RS485 rtds( 82, 100.0 ); // RTD
PID peltierPID(&g_status.input, &g_status.output, &g_status.pid_target_temp,2,5,1, REVERSE); // PID
//---- global variables ----
//////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
//
//	SETUP
//
////////////////////////////////////////////////////////////////////////////////
void setup() {
  //PSU control pin
  g_status.ps_on = HIGH;
  digitalWrite( PSON_PIN, g_status.ps_on );
  pinMode( PSON_PIN, OUTPUT );
  digitalWrite( PSON_PIN, g_status.ps_on );
  g_status.over_current = false;
  //setup PWM
  //for PIN5,6 
#if 0
  // devide by 8
  cbi(TCCR0B, CS22); //CS22=b0
  sbi(TCCR0B, CS21); //CS21=b1
  cbi(TCCR0B, CS20); //CS20=b0  
#endif

#if 0
  // devide by 1
  cbi(TCCR0B, CS22); //CS22=b0
  sbi(TCCR0B, CS21); //CS21=b0
  cbi(TCCR0B, CS20); //CS20=b1  
#endif

#define PIN3_16K	0
#define PIN3_4K		1
#define PIN3_MODE	PIN3_16K
  
  //TCCR2B is PIN3 control register
  // ASSUME PHASE CORRECT
  // PWM freq = clock freq / ( division ratio * 256 * 2 )
#if PIN3_MODE==PIN3_16K
  //clock divider = 1
  //for PIN3,11 - 32KHz 
  cbi(TCCR2B, CS22); //CS22=b0
  cbi(TCCR2B, CS21); //CS21=b0
  sbi(TCCR2B, CS20); //CS20=b1
  //set to Phase correct - 16KHz
  //WGM22,21,20 = 001 or 101
  cbi(TCCR2B, WGM22); //WGM22=b0
  cbi(TCCR2B, WGM21); //WGM21=b0
  sbi(TCCR2B, WGM20); //WGM20=b1
#elif PIN3_MODE==PIN3_4K
  //clock divider = 8
  //for PIN3,11 - 4KHz - VNH2SP30 - MAX PWN FREQ<20KHz
  cbi(TCCR2B, CS22); //CS22=b0
  sbi(TCCR2B, CS21); //CS21=b1
  cbi(TCCR2B, CS20); //CS20=b0
#else
# error UNKNOWN PIN3 PWM FREQUENCY
#endif
  //sbi(TCCR2B, CS20); //CS20=b1  
  
  if ( reference_voltage == 33 ){
    // connect AREF to 3.3V and use that as VCC, less noisy!
    analogReference(EXTERNAL);
  }
  
  // Debugging output
  Serial.begin(9600);
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);

  // Print a message to the LCD. We track how long it takes since
  // this library has been optimized a bit and we're proud of it :)
  lcd.setBacklight(WHITE);
  show_startup(); 

  pinMode(peltier_cool_output_pin, OUTPUT);
  pinMode(fan_output_pin, OUTPUT);

  analogWrite(peltier_cool_output_pin,0);
  if ( peltier_cool_output_pin == 3 ||
       peltier_cool_output_pin == 5 ||
       peltier_cool_output_pin == 6 ){
    g_status.intensity = 20;
  }else{
    g_status.intensity = 255;
  }

  pinMode(INA_PIN,OUTPUT);
  pinMode(INB_PIN,OUTPUT);
  pinMode(ENA_PIN,INPUT_PULLUP);
  pinMode(ENB_PIN,INPUT_PULLUP);
  //pinMode(CS_PIN,INPUT_PULLUP);

  init_rtd_temp();

  
  analogWrite(fan_output_pin,255);

  get_thermistor_temp();
  get_thermocouple_temp();
  get_rtd_temp();
  

  peltierPID.SetMode(AUTOMATIC);
  peltierPID.SetSampleTime(1000);/*per 1sec*/
  peltierPID.SetOutputLimits(-255,255);
  peltierPID.SetControllerDirection(REVERSE);

  
  readSaveDataFromEEPROM(&g_save_data);
  if ( isnan(g_save_data.setpoint) ){
    g_save_data.setpoint = 16.0;
  }
  g_status.pid_target_temp = g_status.setpoint = g_save_data.setpoint;
  if ( isnan(g_save_data.p) || g_save_data.p == 0.0f){
    g_save_data.p = DEFAULT_PID_P;
    g_save_data.i = DEFAULT_PID_I;
    g_save_data.d = DEFAULT_PID_D;
    updatePIDforEEPROM( g_save_data.p, g_save_data.i, g_save_data.d );
  }
  peltierPID.SetTunings( g_save_data.p, g_save_data.i, g_save_data.d );

  
  //  delay(2000);
  lcd.clear();
}
////////////////////////////////////////////////////////////////////////////////
//
//	MAIN LOOP
//
////////////////////////////////////////////////////////////////////////////////
void loop() {
  //read sensor
  get_thermistor_temp();
  get_thermocouple_temp();
  get_rtd_temp();
  // record history and decide PID target temperature
  record_history();
  update_pid_target();
  

  //set target temp to PID input
  // for peltier low side target
  g_status.input = g_status.temp_out[LOW_SIDE_TEMP];
  // for termocouple target
  // g_status.input = g_status.target_temp;
  peltierPID.Compute();
  g_status.intensity = (int)g_status.output;
  update_intensity();
  check_condition();

  //menu UI
  menu();

  //communicate with PID front end.
  serial_processing();
}


////////////////////////////////////////////////////////////////////////////////
//
//         UP                               SELECT<== PSU POWER ON/OFF
//    LEFT    RIGHT <==change menu context
//         DOWN
//         A                                              RESET<=??
//         |
//        Behavior change between context
//-------------------------- MENU UIs
#define LONG_PRESS_DETECTION_MS 500
#define MENU_SCHEDULING_MS   100
void menu(void){
  static uint8_t button_prev=0;
  static unsigned long menu_time=0;
  static uint8_t menu_context=0;
  static const menu_context_proc_t menu_context_proc[] = {
							  main_ui,
							  temperature_detail_ui,
							  pid_ui,
							  stability_ui,
							  version_ui
  };
  static const uint8_t num_context=sizeof(menu_context_proc)/sizeof(menu_context_proc_t); 

  if ( menu_time == 0 ){
    menu_time = millis();
  }

  unsigned long current_time = millis();
  if ( current_time > menu_time ){
    menu_time += MENU_SCHEDULING_MS;
    uint8_t button_now=lcd.readButtons();
    static uint16_t kept_press_button = 0;
    static unsigned long start_press_button = 0;
    //up down is context sensitive button
    //left/right/select is common amoung context to change context.
    //here we handle left/right/select button.
    if ( button_now ){
      if (button_now == button_prev){
	if ( kept_press_button == 0 ){
	  start_press_button = current_time-MENU_SCHEDULING_MS;
	  kept_press_button = MENU_SCHEDULING_MS;
	}else{
	  kept_press_button = current_time - start_press_button;
	}
      }else{
	kept_press_button = 0;
      }

      if ((button_now & BUTTON_SELECT) && (button_prev & BUTTON_SELECT) == 0){
	g_status.ps_on = g_status.ps_on == LOW ? HIGH : LOW;
	power_supply_control();
      }
      if ((button_now & BUTTON_LEFT) && (button_prev & BUTTON_LEFT) == 0){
	menu_context = menu_context==0 ? (num_context-1): (menu_context-1);
      }
      if ((button_now & BUTTON_RIGHT) && (button_prev & BUTTON_RIGHT) == 0){
	menu_context = menu_context==(num_context-1) ? 0 : (menu_context+1);
      }
    }
    menu_context_proc[menu_context](button_now,button_prev,kept_press_button);

    button_prev = button_now;//save button state

    if ( g_status.setpoint != g_save_data.setpoint ){
      g_save_data.setpoint = g_status.setpoint;
      g_status.pid_target_temp = g_status.setpoint;
      updateSaveDataToEEPROM(&g_save_data);
    }
  }
}

void main_ui(uint8_t button_now, uint8_t button_prev, uint16_t button_keep_pressed_counter){
  if (button_now) {
    static uint16_t button_context=0;
    switch( common_button_handling(BUTTON_UP, button_now, button_prev, button_keep_pressed_counter, &button_context ) ){
    case FIRST_PRESS:
      g_status.setpoint += .1;
      break;
    case LONG_PRESS:
      g_status.setpoint += 1.0;
      break;
    default:
      break;
    }
    switch( common_button_handling(BUTTON_DOWN, button_now, button_prev, button_keep_pressed_counter, &button_context ) ){
    case FIRST_PRESS:
      g_status.setpoint -= .1;
      break;
    case LONG_PRESS:
      g_status.setpoint -= 1.0;
      break;
    default:
      break;
    }
  }
  display_main_lcd();
}


const char clear_line[]={"                "};
const char progress_char[]={"-\\|/"};

void display_main_lcd(){
  static int display_count=0;
  static int Hz=0;
  static unsigned long start_time=0;

  display_count++;
  unsigned long current_time = millis();
  if ( start_time == 0 ){
    start_time = current_time;
  }else{
    if ( current_time - start_time > 1000 ){
      Hz = display_count;
      display_count=0;
      start_time = current_time;
    }
  }
#if 0
  //lcd update performance -- 5.6 milli sec / one character or command .
  lcd.setCursor(0,0);//5600usec
  //lcd.print("12345678901234561234567890123456");//179800usec
  //  lcd.print("a");//5640usec
  lcd.print("SET:");//22500usec
  print_float((float)g_status.setpoint);//22680usec
  lcd.print(clear_line);//89900usec
  lcd.setCursor(0,1);
  if ( g_status.over_current || g_status.over_temp ){
    lcd.print("ERROR=");
    if ( g_status.over_current ){
      lcd.print("OC,");
    }
    if ( g_status.over_temp ){
      lcd.print("OT,");
    }
  }
  lcd.print("/MENU Hz=");
  lcd.print(Hz);
  lcd.print(clear_line);
#else
  char line[16*2+1];
  //  int set_integer = (int)g_status.setpoint;
  //  int set_float = (int)(g_status.setpoint*10 - set_integer*10);
  int set_integer, set_float, t_integer, t_float;
  float_to_int(g_status.setpoint,&set_integer,&set_float);
  float_to_int(g_status.target_temp, &t_integer, &t_float );
  int roomtemp_integer, roomtemp_float;
  float_to_int(g_status.env_temp, &roomtemp_integer, &roomtemp_float);
  char error_str[4][10+1]={
			   /*
			     0123456789
			   */
			   "NONE",
			   "OC", // over current
			   "OT", // over temp
			   "OC&OT"};
  uint8_t error_val = (g_status.over_current!=0) |  (g_status.over_temp!=0)<<1;

  //            0123  4567  89 AB CD  EF01234  56  789ABCDEF
  sprintf(line,"SET:%02d.%1d [%02d.%1d] RT:%02d.%1d|ER:%-5s",
	  set_integer, set_float, t_integer, t_float, roomtemp_integer, roomtemp_float,
	  error_str[error_val] );
  updateLCD(line);
#endif
}


eButtonAction common_button_handling( uint8_t button_to_be_checked, uint8_t button_now, uint8_t button_prev, uint16_t button_keep_pressed_counter , uint16_t* context){
  eButtonAction ret = NOACTION;
  if (button_now & button_to_be_checked){
    if ( (button_prev & button_to_be_checked) == 0 ) {
      ret = FIRST_PRESS;
      *context = 0;
    }else if ( button_keep_pressed_counter - *context > LONG_PRESS_DETECTION_MS ) {
      ret = LONG_PRESS;
      *context = button_keep_pressed_counter;
    }
  }
  return ret;
}


void temperature_detail_ui(uint8_t button_now, uint8_t button_prev, uint16_t button_keep_pressed_counter){
  if (button_now) {
  }
  display_temperature_detail_to_lcd();
}
void display_temperature_detail_to_lcd(void){
#if 0
  //line1

  lcd.setCursor(0,0 );
  if ( g_status.over_current || g_status.over_temp ){
    lcd.print("E=");
    if ( g_status.over_current ){
      lcd.print("OC,");
    }
    if ( g_status.over_temp ){
      lcd.print("OT,");
    }
  }else{
    lcd.print("PWM=");
    lcd.print(g_status.intensity);
  }
  lcd.print(",CS=");
  //lcd.print(g_status.current_sense);
  print_float(g_status.current_sense_amp);
  // if( g_status.ps_on == LOW ){
  //   lcd.print(",ON");
  // }else{
  //   lcd.print(",OF");
  // }
  static uint8_t progress=0;
  lcd.write(progress_char[progress]);
  progress=(progress+1)&(4-1);
  

  //line2
  lcd.setCursor(0,1);
  lcd.print("T");
  print_float(g_status.target_temp);
  lcd.print("H");
  print_float(g_status.temp_out[HIGH_SIDE_TEMP]);
  lcd.print("L");
  print_float(g_status.temp_out[LOW_SIDE_TEMP]);
  lcd.print(clear_line);
#else
  char line[16*2+1]={0};
  int cs_integer,cs_float;
  int t_integer,t_float;
  int h_integer,h_float;
  int l_integer,l_float;
  float_to_int(g_status.current_sense_amp, &cs_integer, &cs_float );
  float_to_int(g_status.target_temp, &t_integer, &t_float );
  float_to_int(g_status.temp_out[HIGH_SIDE_TEMP], &h_integer, &h_float );
  float_to_int(g_status.temp_out[LOW_SIDE_TEMP], &l_integer, &l_float );
  
  //            012345 678 9A BCDEF0123456789ABCDEF
  sprintf(line,"PWM%03d CS%02d.%1d   T%2d.%1dH%2d.%1dL%2d.%1d",
	  g_status.intensity, cs_integer, cs_float, t_integer,t_float, h_integer,h_float, l_integer,l_float );
  updateLCD(line);
#endif
}

// PID UI
void pid_ui(uint8_t button_now, uint8_t button_prev, uint16_t button_keep_pressed_counter){
#if 0
  lcd.setCursor(0,0);
  lcd.print("P:");
  print_float(peltierPID.GetKp());   
  lcd.print(" I:");
  print_float(peltierPID.GetKi());
  lcd.print(clear_line);
  
  lcd.setCursor(0,1);
  lcd.print("D:");
  print_float(peltierPID.GetKd());   
  lcd.print(clear_line);
#else
  char line[16*2+1]={0};
  int p_integer,p_float;
  int i_integer,i_float;
  int d_integer,d_float;
  float_to_int(peltierPID.GetKp(), &p_integer, &p_float );
  float_to_int(peltierPID.GetKi(), &i_integer, &i_float );
  float_to_int(peltierPID.GetKd(), &d_integer, &d_float );
  //            012345 678 9A BCDEF0123456789ABCDEF
  sprintf(line,"P:%03d.%d I:%03d.%d D:%03d.%d",
	  p_integer,p_float,i_integer,i_float,d_integer,d_float);
  updateLCD(line);
#endif
}
void version_ui(uint8_t button_now, uint8_t button_prev, uint16_t button_keep_pressed_counter){
  char line[16*2+1]={0};
  sprintf(line,"%16s%16s","Peltier Cooler",REVISION_STRING);
  updateLCD(line);
}

void stability_ui(uint8_t button_now, uint8_t button_prev, uint16_t button_keep_pressed_counter){
  float peltier_max=.0,peltier_min=100.0f;
  float target_max=.0,target_min=100.0f;
  char line[16*2+1]={0};
  
  for ( int i = 0 ; i <  g_status.hist_count ; i++ ){
    peltier_max = (peltier_max < g_status.peltier_temp_hist[i]) ? g_status.peltier_temp_hist[i] : peltier_max;
    peltier_min = (peltier_min > g_status.peltier_temp_hist[i]) ? g_status.peltier_temp_hist[i] : peltier_min;
    target_max = (target_max < g_status.target_temp_hist[i]) ? g_status.target_temp_hist[i] : target_max;
    target_min = (target_min > g_status.target_temp_hist[i]) ? g_status.target_temp_hist[i] : target_min;
  }
  int diff_peltier_integer,diff_peltier_float;
  float diff;
  diff = peltier_max - peltier_min;
  float_to_int00(diff,&diff_peltier_integer, &diff_peltier_float);
  
  int diff_target_integer,diff_target_float;
  diff = target_max - target_min;
  float_to_int00(diff,&diff_target_integer, &diff_target_float);

  sprintf(line,"[%d]PELTIER:%01d.%02d TARGET:%01d.%02d",
	  g_status.hist_count,
	  diff_peltier_integer, diff_peltier_float,
	  diff_target_integer,diff_target_float );
  updateLCD(line);
}




void show_startup(){
  lcd.clear();
  version_ui(0,0,0);
}



void print_float(float f){
  int i=(int)f;
  lcd.print( i );
  lcd.print( "."    );
  int ii=( f-(float)i )*100.0f;
  lcd.print( ii/10 );
  //lcd.print( ii%10 );
}
void float_to_int(float f, int* integer_part, int* mantissa){
  *integer_part=(int)f;
  *mantissa=( f-*integer_part )*10.0f;
}
void float_to_int00(float f, int* integer_part, int* mantissa){
  *integer_part=(int)f;
  *mantissa=( f-*integer_part )*100.0f;
}

//////////////////////////////////////////////////////////////////////////////////
// NEW LCD PRINT 
//////////////////////////////////////////////////////////////////////////////////
char lcd_buf[2][16+1];
void updateLCD( const char display_lines[16*2+1]){
  updateLCDLine(0, display_lines);
  updateLCDLine(1, &display_lines[16]);
}
void updateLCDLine( int line , const char str[16] ){
  for ( int col = 0 ; col < 16 ; col++ ){
    char c = (str[col]==0?' ':str[col]);
    if ( lcd_buf[line][col] != c ){
      lcd.setCursor(col,line);
      lcd.write(c);
      lcd_buf[line][col]=c;
      for ( int col2 = col+1 ; col2 < 16 ; col2++ ){
	c = (str[col2]==0?' ':str[col2]);
	if ( lcd_buf[line][col2] != c ){
	  lcd.write(c);
	  lcd_buf[line][col2]=c;
	}else{
	  col = col2;
	  break;
	}
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////
//-------------------------- Thermister temperature sensor
//////////////////////////////////////////////////////////////////////////////////
void get_thermistor_temp(){
  uint8_t i;
  float average[NUM_THERMISTOR];
  static int samples[NUM_THERMISTOR][NUMSAMPLES];
  int pins[NUM_THERMISTOR] = {THERMISTORPIN0,THERMISTORPIN1};
  static int current_sample=0;
  static unsigned long sampling_time=0;
  
  if ( sampling_time == 0 ){
    sampling_time = millis();
    for ( int j = 0 ; j < NUM_THERMISTOR ; j++ ){
      int val = analogRead(pins[j]);
      for (int i=0; i< NUMSAMPLES; i++) {
	samples[j][i] = val;
      }
    }
  }
  unsigned long current_time = millis();
  if ( sampling_time < current_time ) {
    int thermistor = current_sample%NUM_THERMISTOR;
    // take N samples in a row, with a slight delay
    samples[thermistor][current_sample/NUM_THERMISTOR] = analogRead(pins[thermistor]);
#if DELAY_PER_SAMPLE != 0
    sampling_time += DELAY_PER_SAMPLE;
#endif
    current_sample = current_sample == NUM_THERMISTOR*NUMSAMPLES ? 0 : current_sample+1;
    // average all the samples out
    int sample_count;
    average[thermistor] = 0;
    sample_count=0;
    for( i=0; i< NUMSAMPLES; i++) {
      average[thermistor] += samples[thermistor][i];
      sample_count++;
    }
    average[thermistor] /= sample_count;
#ifdef SERIAL_DEBUG
    Serial.print("Average analog reading "); 
    Serial.println(average[thermistor]);
#endif
    // convert the value to resistance
    average[thermistor] = 1023 / average[thermistor] - 1;
    average[thermistor] = SERIESRESISTOR / average[thermistor];
#ifdef SERIAL_DEBUG
    Serial.print("Thermistor resistance "); 
    Serial.println(average[thermistor]);
#endif
    
    float steinhart;
    steinhart = average[thermistor] / THERMISTORNOMINAL;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;                         // convert to C
    g_status.temp_out[thermistor] = steinhart;
#ifdef SERIAL_DEBUG
    Serial.print("Temperature "); 
    Serial.print(steinhart);
    Serial.println(" C");
#endif
  }
}



//////////////////////////////////////////////////////////////////////////////////
//-------------------------- I2C temperature sensor - thermo couple
//////////////////////////////////////////////////////////////////////////////////
float getTemperature(int addr){
  Wire.requestFrom(addr,2); 

  byte MSB = Wire.read();
  byte LSB = Wire.read();

  //it's a 12bit int, using two's compliment for negative
  int TemperatureSum = ((MSB << 8) | LSB) >> 4; 

  float celsius = TemperatureSum*0.0625;
  return celsius;
}
 
void get_thermocouple_temp(){
  static unsigned long interval_timer = 0;
  if ( interval_timer == 0 ){
    interval_timer = millis();
  }
  if ( interval_timer < millis() ){
    g_status.env_temp = getTemperature( tmp_0_addr );
    interval_timer += 1000;
  }
}


//////////////////////////////////////////////////////////////////////////////////
//   RTD temp measurement
//////////////////////////////////////////////////////////////////////////////////
unsigned long rtd_interval_timer = 0;
void init_rtd_temp(){
  rtds.Factory_Reset();
  rtds.Disable_All_RTD_Channels();
  rtds.Enable_RTD_Channel( 3, 1 ); // enable 3wire channel 1
  rtds.Set_RTD_SPS( 10 ); // Samples-per-second = 10. 
  rtds.Set_RTD_Idac( 3, 1, 0.000250 );
  rtds.Set_RTD_PGA( 3, 1, 64 );

#define RTD_READ_INTERVAL_MS_INITIAL 6000 // 3000 for 20sps. 10000 for 5sps.
#define RTD_READ_INTERVAL_MS 3200 // 1600 for 20sps. 6600 for 5sps.
  rtd_interval_timer = millis() + RTD_READ_INTERVAL_MS_INITIAL;
}
void get_rtd_temp(){
  if ( rtd_interval_timer < millis() ){
    g_status.target_temp = rtds.Get_RTD_Temperature_degC( 3, 1 );
    rtd_interval_timer += RTD_READ_INTERVAL_MS;
  }
}

//////////////////////////////////////////////////////////////////////////////////
//  Temp HISTORY - for check???
//////////////////////////////////////////////////////////////////////////////////
//   record temp history and get average
void record_history(){
  static unsigned long sampling_time=0;
  static float peltier_lowside_sum = 0;
  static float target_sum = 0;
  static int num_sample = 0;
    
  if ( sampling_time == 0 ){
    sampling_time = millis() + TEMP_SAMPLING_PERIOD*1000;
    g_status.hist_count = 0;
  }

  unsigned long current_time = millis();
  if ( sampling_time < current_time ) {
    sampling_time += TEMP_SAMPLING_PERIOD*1000;
    peltier_lowside_sum += g_status.temp_out[LOW_SIDE_TEMP];
    target_sum += g_status.target_temp;
    num_sample ++;
    if ( num_sample == (TEMP_AVG_DURATION/TEMP_SAMPLING_PERIOD) ){
      g_status.peltier_temp_hist[g_status.hist_count] = peltier_lowside_sum/num_sample;
      g_status.target_temp_hist[g_status.hist_count] = target_sum/num_sample;
      g_status.hist_count++;
      peltier_lowside_sum = 0;
      target_sum = 0;
      num_sample = 0;
    }
  }
}
//   update pid target temperature from temperature record.
void update_pid_target(){
  float peltier_max=.0,peltier_min=100.0f;
  float target_max=.0,target_min=100.0f;
  
  if ( g_status.hist_count == MAX_TEMP_HISTORY ){
    g_status.hist_count = 0;
    for ( int i = 0 ; i < MAX_TEMP_HISTORY ; i++ ){
      peltier_max = (peltier_max < g_status.peltier_temp_hist[i]) ? g_status.peltier_temp_hist[i] : peltier_max;
      peltier_min = (peltier_min > g_status.peltier_temp_hist[i]) ? g_status.peltier_temp_hist[i] : peltier_min;
      target_max = (target_max < g_status.target_temp_hist[i]) ? g_status.target_temp_hist[i] : target_max;
      target_min = (target_min > g_status.target_temp_hist[i]) ? g_status.target_temp_hist[i] : target_min;
    }
    if ( (peltier_max - peltier_min) <= 0.1
	 &&
	 ( fabs((peltier_max+peltier_min)/2.0 - g_status.pid_target_temp) < 0.1 )
	 &&
	 (target_max - target_min) <= 0.1 ){
      // now temperature is stable.
      // update target temperature --- kind of integral control???
      g_status.pid_target_temp += (g_status.setpoint - ((target_max+target_min)/2.0f))*TEMP_ADJUST_FACTOR;
      // update temperature hist to avoid successive false update
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////
//--------------------------for PID UI serial communication
//////////////////////////////////////////////////////////////////////////////////
void serial_processing(void){
  static unsigned long serialTime=0; //this will help us know when to talk with processing
  if ( serialTime == 0 ){
    serialTime = millis();
  }
  //send-receive with processing if it's time
  if(millis()>serialTime)
    {
      SerialReceive();
      SerialSend();
      serialTime+=10000;
    }
}
 
//////////////////////////////////////////////////////////////////////////////////
/** PID UI **/
//////////////////////////////////////////////////////////////////////////////////
union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
  foo;                   // float array

// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2-5: float setpoint
//  6-9: float input
//  10-13: float output  
//  14-17: float P_Param
//  18-21: float I_Param
//  22-245: float D_Param
void SerialReceive()
{

  // read the bytes sent from Processing
  int index=0;
  byte Auto_Man = -1;
  byte Direct_Reverse = -1;
  while(Serial.available()&&index<26)
    {
      if(index==0) Auto_Man = Serial.read();
      else if(index==1) Direct_Reverse = Serial.read();
      else foo.asBytes[index-2] = Serial.read();
      index++;
    } 
  
  // if the information we got was in the correct format, 
  // read it into the system
  if(index==26  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1))
    {
      g_status.setpoint=double(foo.asFloat[0]);
      //Input=double(foo.asFloat[1]);       // * the user has the ability to send the 
      //   value of "Input"  in most cases (as 
      //   in this one) this is not needed.
      if(Auto_Man==0)                       // * only change the output if we are in 
	{                                     //   manual mode.  otherwise we'll get an
	  g_status.output=double(foo.asFloat[2]);      //   output blip, then the controller will 
	}                                     //   overwrite.
    
      double p, i, d;                       // * read in and set the controller tunings
      p = double(foo.asFloat[3]);           //
      i = double(foo.asFloat[4]);           //
      d = double(foo.asFloat[5]);           //
      peltierPID.SetTunings(p, i, d);            //
    
      if(Auto_Man==0) peltierPID.SetMode(MANUAL);// * set the controller mode
      else peltierPID.SetMode(AUTOMATIC);             //
    
      if(Direct_Reverse==0) peltierPID.SetControllerDirection(DIRECT);// * set the controller Direction
      else peltierPID.SetControllerDirection(REVERSE);          //

      //update eeprom
      updatePIDforEEPROM(p,i,d);
    }
  
  Serial.flush();                         // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
  Serial.print("PID ");
  Serial.print(g_status.pid_target_temp);   
  Serial.print(" ");
  Serial.print(g_status.input);   
  Serial.print(" ");
  Serial.print(g_status.output);   
  Serial.print(" ");
  Serial.print(peltierPID.GetKp());   
  Serial.print(" ");
  Serial.print(peltierPID.GetKi());   
  Serial.print(" ");
  Serial.print(peltierPID.GetKd());   
  Serial.print(" ");
  if(peltierPID.GetMode()==AUTOMATIC) Serial.print("Automatic");
  else Serial.print("Manual");  
  Serial.print(" ");
  if(peltierPID.GetDirection()==DIRECT) Serial.print("Direct");
  else Serial.print("Reverse");
  Serial.print(" ");
  Serial.print(g_status.setpoint);
  Serial.print(" ");
  Serial.println(g_status.target_temp);
}

//------------------------------------------ IO

/* returns amp*1000 */
#define CS_VOLTAGE_REF 5 /*V*/
#define CS_READ_PRECISION 1023 /*VALUE for 5V*/
#define CS_COEF 0.13f

float convert_current_sense2amp(int cs_analogread_value){
  return ((CS_VOLTAGE_REF/CS_COEF)/CS_READ_PRECISION)*cs_analogread_value;
}



//--------------------------------------------------------------------------
// UPDATE PWM update
void update_intensity(){
  output_peltier_intensity();

  g_status.current_sense = analogRead(CS_PIN);
  g_status.current_sense_amp = convert_current_sense2amp(g_status.current_sense);
}

void output_peltier_intensity(){
  int out= g_status.intensity > 0 ? g_status.intensity : -1*g_status.intensity;
  out = out < 256 ? out : 255; /* cap to 255 */
  int ina = g_status.intensity > 0 ? HIGH : LOW;
  int inb = g_status.intensity < 0 ? HIGH : LOW;
  // if intensity = 0 -> ina = 0 and inb = 0 -> brake to ground
  //update
  analogWrite(peltier_cool_output_pin,out);
  digitalWrite(INA_PIN,ina);
  digitalWrite(INB_PIN,inb);
}
    

void check_condition(){
  if ( g_status.current_sense_amp > CURRENT_LIMIT_VALUE ){
    g_status.over_current = true;
  }

  if ( g_status.temp_out[HIGH_SIDE_TEMP] > HIGH_TEMP_GUARD ){
    g_status.over_temp = true;
  }

  if ( g_status.over_current || g_status.over_temp ){
    g_status.intensity = 0;
    output_peltier_intensity();
  }
  return;
}

void reset_error(){
  g_status.over_current = false;
  g_status.over_temp = false;
}

void power_supply_control(){
  if ( g_status.ps_on == LOW ){
    reset_error();
  }
  digitalWrite( PSON_PIN, g_status.ps_on );
}


// ---------------------------------------------- Start of EEPROM READ/WRITE
void updatePIDforEEPROM(double p, double i, double d){
  g_save_data.p = p;
  g_save_data.i = i;
  g_save_data.d = d;
  updateSaveDataToEEPROM( &g_save_data );
}

void readSaveDataFromEEPROM(eeprom_save_data_t* psave_data){
  uint8_t* buf = (uint8_t*)psave_data;
  for ( int i = 0 ; i < (int)sizeof(double) ; i++ ){
    buf[i] = EEPROM.read(EEPROM_SAVE_DATA_START_ADDR+i);
  }
}
void updateSaveDataToEEPROM(const eeprom_save_data_t* psave_data){
  uint8_t* buf = (uint8_t*)psave_data;
  for ( int i = 0 ; i < (int)sizeof(double) ; i++ ){
    EEPROM.write(EEPROM_SAVE_DATA_START_ADDR+i,buf[i]);
  }
}

double ReadDoubleEEPROM( int addr ){
  uint8_t buf[sizeof(double)];
  for ( int i = 0 ; i < (int)sizeof(double) ; i++ ){
    buf[i] = EEPROM.read(addr+i);
  }
  return *(double*)buf;
}
void   WriteDoubleEEPROM( int addr, double value ){
  uint8_t buf[sizeof(double)];
  *(double*)buf = value;
  for ( int i = 0 ; i < (int)sizeof(double) ; i++ ){
    EEPROM.write(addr+i,buf[i]);
  }
}
// ---------------------------------------------- END of EEPROM READ/WRITE

