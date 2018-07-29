//#define SERIAL_DEBUG
#define CONTROLLER_NAME		"PELTIER CONTROLLER"

#define CURRENT_LIMIT_VALUE	6.0f /* Current limit - Amp */
#define HIGH_TEMP_GUARD		50.0f /* Temperature Limit - celcius */

// structures / types
//    for save data
#define EEPROM_SAVE_DATA_START_ADDR 0x00
typedef struct {
  double setpoint;
  double p;
  double i;
  double d;
}eeprom_save_data_t;
//    for button processing
typedef enum{
  NOACTION,  
  FIRST_PRESS,
  LONG_PRESS,
}eButtonAction;
//    for menu call
typedef void (*menu_context_proc_t)(uint8_t button_now, uint8_t button_prev, uint16_t button_keep_pressed_counter);


