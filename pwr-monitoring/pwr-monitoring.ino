/* 
 Power monitoring
 Arduino Mega 1280
 E. JENN  

 --------------------------------------------------------------

 Digital outputs:
  // Analog multiplexer input selection (3 bits)
      ANAMUX0_PIN 47
      ANAMUX1_PIN 48
      ANAMUX2_PIN 49

  // Relays control
      PWR_CHANNEL1_PIN 43
      PWR_CHANNEL2_PIN 44
      PWR_CHANNEL3_PIN 45
      PWR_CHANNEL4_PIN 46

  // LEDs
      LED_R_PIN 40
      LED_G_PIN 41
      LED_B_PIN 42
      LED_PIN 13

  // Buzzer
      BUZZER_PIN 35
 
 Digital inputs:
  // Keys
      KEY_G_PIN 36
      KEY_Y1_PIN 37
      KEY_Y2_PIN 38
      KEY_R_PIN 39

 Analog inputs:
    Input from analog multiplexer 1 (battery voltages)
      ANA_MUX_1_PIN 13 (from analog multiplexer, battery voltages)
      ANA_I1_PIN 14 (from ACS712 battery 0)
      ANA_I2_PIN 15 (from ACS712 battery 0)

 CAN messages:
      K_CAN_BAT_V_ID        0xA0 (battery voltages, bat 0 and bat 1, 2*floats)
      K_CAN_BAT_I_ID        0xA1 (battery current, bat 0 and bat 1, 2*floats)
      K_CAN_BAT_LVL_ID      0xA2 (PWR LVL, bat0 and bat1, uint8_t)
      K_CAN_POSTPONE_ID     0xA3 (channel change postpone)
      K_CAN_BAT_SETLVL_ID   0xA4 (channel change postpone)
 SPI interface:
  - MISO : pin 50
  - MOSI : pin 51
  - SCK  : Pin 52
  - SS   : pin 53


*/

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <mcp_can.h>
#include <SPI.h>


// Key debounce time in ms
#define kDEBOUNCE_MS 10

/* 
 * Page display duration 
 * Duration of a page display
 */
#define kPAGE_DSP_DURATION 5000

/*
 * Channel change delays
 */
// Delay between a PWR LVL change and the channel control 
#define k_CHANNEL_CHANGE_DELAY 10000
// Additional delay in case a postpone message is received
#define k_CHANNEL_POSTPONE_DELAY 5000


/*
 * Sound frequencies
 */
#define kALERT_FREQ 1100
#define kUP_FREQ 1100
#define kDN_FREQ 900
#define kVALID_FREQ 440
#define kCANCEL_FREQ 220
#define kMANUAL_FREQ 2000
#define kAUTO_FREQ 2000
#define kCHANNEL_UPDATE_FREQ 3000
#define kPOSTPONE_FREQ 4000
#define kBOOT_FREQ 4000
/*
 * CAN message configuration
 */
// Voltage battery 0 and 1 (out)
#define K_CAN_BAT_V_ID 0xA0
// Current battery 0 and 1 (out)
#define K_CAN_BAT_I_ID 0xA1
// State data (out)
#define K_CAN_BAT_LVL_ID 0xA2 
// Channel set messages (in)
#define K_CAN_BAT_POSTPONE_ID 0xA3 
// Set battery level (bat id: uint8_t, bat lvl: uint8_t)
#define K_CAN_BAT_SETLVL_ID 0xA4 

 
/*
 * LED configuration
 */

// Blinking periods in ms (0 is always on) for the Slow blink, Fast blink and Very fast blink

const uint16_t kBLINK_HALF_PERIOD[]={1000,125,50};


/*
 * Digital pins
 */

// Analog multiplexer
#define ANAMUX0_PIN 47
#define ANAMUX1_PIN 48
#define ANAMUX2_PIN 49

// Relays
#define PWR_CHANNEL1_PIN 43
#define PWR_CHANNEL2_PIN 44
#define PWR_CHANNEL3_PIN 45
#define PWR_CHANNEL4_PIN 46

// LEDs
#define LED_R_PIN 40
#define LED_G_PIN 41
#define LED_B_PIN 42
#define LED_PIN 13

// Keys
#define KEY_G_PIN 36
#define KEY_Y1_PIN 37
#define KEY_Y2_PIN 38
#define KEY_R_PIN 39

// Buzzer
#define BUZZER_PIN 35

/*
 * Analog pins
 */
// Input from analog multiplexer 1 (battery voltages)
#define ANA_MUX_1_PIN 13 
// Input from ACS712 battery 1
#define ANA_I1_PIN 14  
// Input from ACS712 battery 2
#define ANA_I2_PIN 15  


/*
 * CAN bus configuration
 */
#define SPI_CS_PIN 53
#define K_INTERRUPT_PIN 2



MCP_CAN CAN(SPI_CS_PIN); 

/*
 * System states
 */
typedef enum {  e_sys_pwr_on,
                e_sys_wait,
                e_sys_nominal,
                e_sys_error,
                e_alert} sys_state_t;

/*
 * Control states
 */
typedef enum  { e_ctrl_auto,
                e_ctrl_manual,
                e_ctrl_confirm } ctrl_state_t;

/*
 * Display states
 */
typedef enum {  e_dsp_start, 
                e_dsp_va, 
                e_dsp_cells,
                e_dsp_status,
                e_dsp_alert, 
                e_dsp_chg_lvl,
                e_dsp_sel_bat,
                e_dsp_error} dsp_state_t;

/*
 * Power level
 */
typedef uint8_t  pwr_lvl_t;


/*
 * Keyboard events
 */
typedef struct {
    uint8_t red_pressed_evt;
    uint8_t yellow1_pressed_evt;
    uint8_t yellow2_pressed_evt;
    uint8_t green_pressed_evt;
} key_evts_t;

/*
 * Sound event
 */
typedef struct {
    uint8_t snd_evt;
    unsigned int freq;
    uint8_t repeat;
    unsigned int duration;
    unsigned int interval;
} snd_evts_t;


/*
 * LED commands
 */
typedef enum { eOff=0, eOn, eSlowBlk, eFastBlk, eVeryFastBlk } led_cmd_t;


/*
 * FSM state variables
 */
// System state variable
sys_state_t g_sys_state = e_sys_pwr_on;

// Control state variable
ctrl_state_t g_ctrl_state[2] = { e_ctrl_auto, e_ctrl_auto };

// Display state variable
dsp_state_t g_dsp_state = e_dsp_start;

/*
 * Power levels
 */
pwr_lvl_t g_pwr_lvl[2] ={5,5};          // Current power levels
pwr_lvl_t g_pwr_lvl_user[2];            // Power levels set by the user
pwr_lvl_t g_pwr_lvl_bat[2]  = {5,5};    // Power levels set from the battery levels
pwr_lvl_t g_pwr_lvl_prev[2] = {5,5};    // Last power levels

/*
 * Overcurrent
 */
pwr_lvl_t g_ovrcur[2] = { false, false};

// Current selected battery
uint8_t g_sel_bat;

// Postpone event
uint8_t g_postpone_evt = 0;
uint8_t g_send_pwr_lvl_msg_evt = 0;

// Timers
unsigned long g_sys_next_time, g_dsp_next_time, g_ctrl_next_time[0];
  
// Power channels status
uint8_t g_channels_status[]= {HIGH, LOW, LOW, LOW};

// LEDs commands
led_cmd_t g_led_cmd[3] = {eOff, eOff, eSlowBlk};

// Voltage data
float g_vcell[8];
float g_vbat[2];
float g_vmin[2];

// Current data
float g_ibat[2];

// Current channel
int g_ch = 0;
  
/*
 * Battery levels to string mapping
 */
const char* kBAT_EVTS_TO_STR[] = {"Lvl 0", "Lvl 1", "Lvl 2", "Lvl 3", "Lvl 4", "Lvl 5", "AUTO" };

/*
 * Battery levels to LED commands mapping.
 */
const led_cmd_t kRED_LED_LVLS[] = { eOn, eOn, eFastBlk, eSlowBlk, eSlowBlk, eOff };
const led_cmd_t kGREEN_LED_LVLS[] = { eOff, eOff, eSlowBlk, eSlowBlk, eFastBlk, eOn };

/*
 * Sound management global variables
 */

snd_evts_t g_snd = { 0, 0, 0, 0, 0};

// String manipulation buffer
char str[20];

// CAN Reception management
uint8_t g_flagRecv = 0;


// LCD display driver
LiquidCrystal_I2C lcd(0x27,20,4); 


/*
 * --------------------------------------------------------------------
 * Select the analog mux input for a given
 * battery pack and battery in the pack.
 * --------------------------------------------------------------------
 */
void setMux(int pack, int bat) 
{
    uint8_t v = (pack<<2)+bat;
    digitalWrite(ANAMUX0_PIN,(v & 0b001));
    digitalWrite(ANAMUX1_PIN,(v & 0b010)>>1);
    digitalWrite(ANAMUX2_PIN,(v & 0b100)>>2);   
}

/*
 * --------------------------------------------------------------------
 * Set the power channels state (relays)
 * --------------------------------------------------------------------
 */
void  drive_pwr_channels() {
  digitalWrite(PWR_CHANNEL1_PIN, !g_channels_status[0]);
  digitalWrite(PWR_CHANNEL2_PIN, !g_channels_status[1]);
  digitalWrite(PWR_CHANNEL3_PIN, !g_channels_status[2]);
  digitalWrite(PWR_CHANNEL4_PIN, !g_channels_status[3]);
}

/*
 * --------------------------------------------------------------------
 * SETUP function
 * --------------------------------------------------------------------
 */
void setup() {

  /*
   * Setup LCD display
   */

  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Setup serial port 
  Serial.begin(115200);
  
  // Setup digital pins
  pinMode (ANAMUX0_PIN, OUTPUT);
  pinMode (ANAMUX1_PIN, OUTPUT);
  pinMode (ANAMUX2_PIN, OUTPUT);
  pinMode (PWR_CHANNEL1_PIN, OUTPUT);
  pinMode (PWR_CHANNEL2_PIN, OUTPUT);
  pinMode (PWR_CHANNEL3_PIN, OUTPUT);
  pinMode (PWR_CHANNEL4_PIN, OUTPUT);
  pinMode (LED_R_PIN, OUTPUT);
  pinMode (LED_G_PIN, OUTPUT);
  pinMode (LED_B_PIN, OUTPUT);
  pinMode (KEY_R_PIN, INPUT_PULLUP);  
  pinMode (KEY_Y1_PIN, INPUT_PULLUP);  
  pinMode (KEY_Y2_PIN, INPUT_PULLUP);  
  pinMode (KEY_G_PIN, INPUT_PULLUP);  
  pinMode (BUZZER_PIN, OUTPUT);

  
  /*
   * Analog pins
   */
  pinMode (ANA_MUX_1_PIN, INPUT); // Voltage all batteries
  pinMode (ANA_I1_PIN, INPUT);    // Current battery 1
  pinMode (ANA_I2_PIN, INPUT);    // Current battery 2

  // Select ana multiplexer channel 0
  setMux(0, 0) ;

  /*
   * Setup CAN
   */

  while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz))     
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        lcd.print("CAN init failure");  
        delay(100);
    }
  Serial.println("CAN BUS Shield init ok!");

  // Set masks 
  // there are 2 mask in mcp2515, you need to set both of them
  CAN.init_Mask(0, 0, 0x3ff);   
  CAN.init_Mask(1, 0, 0x3ff);

  // Set filters 
  // There are 6 filters in the mcp2515
  CAN.init_Filt(0, 0, K_CAN_BAT_POSTPONE_ID);   // Receive the channel set messages
  CAN.init_Filt(1, 0, K_CAN_BAT_SETLVL_ID);   // Receive the channel set messages
  // Attach interrupt routine
  attachInterrupt(digitalPinToInterrupt(K_INTERRUPT_PIN), MCP2515_ISR, FALLING); // start interrupt

  // Set the analog multiplexer
  setMux(0, 0) ;  

  
  // Set initial state
  g_sys_state = e_sys_pwr_on;
  g_ctrl_state[0] = e_ctrl_auto;
  g_ctrl_state[1] = e_ctrl_auto;
  g_dsp_state = e_dsp_start;
   
}


/*
 * -------------------------------------------------------------------- 
 * CAN ISR handler
 * --------------------------------------------------------------------
 */
void MCP2515_ISR()
{
    g_flagRecv = 1;
}


/*
 * --------------------------------------------------------------------
 * Sound definitions
 * --------------------------------------------------------------------
 */

const snd_evts_t auto_snd = {
  .snd_evt  = 1,
  .freq     = kAUTO_FREQ,
  .repeat   = 4,
  .duration = 100,
  .interval = 100
};

const snd_evts_t manual_snd = {
  .snd_evt  = 1, 
  .freq     = kMANUAL_FREQ,
  .repeat   = 8,
  .duration = 50,
  .interval = 50
};

snd_evts_t up_snd = {
  .snd_evt  = 1, 
  .freq     = kUP_FREQ,
  .repeat   = 1,
  .duration = 100,
  .interval = 100
};

snd_evts_t dn_snd = {
  .snd_evt  = 1, 
  .freq     = kDN_FREQ,
  .repeat   = 1,
  .duration = 100,
  .interval = 100
};

snd_evts_t alert_snd = {
  .snd_evt  = 1,
  .freq     = kALERT_FREQ,
  .repeat   = 3,
  .duration = 100,
  .interval = 100
};

snd_evts_t valid_snd = {
  .snd_evt  = 1,
  .freq     = kVALID_FREQ,
  .repeat   = 2,
  .duration = 100,
  .interval = 100
};

snd_evts_t cancel_snd = {
  .snd_evt  = 1,  
  .freq     = kCANCEL_FREQ,
  .repeat   = 2,
  .duration = 100,
  .interval = 100
};

snd_evts_t chanset_snd = {
  .snd_evt  = 1, 
  .freq     = kCHANNEL_UPDATE_FREQ,
  .repeat   = 4,
  .duration = 200,
  .interval = 100
};

snd_evts_t boot_snd = {
  .snd_evt  = 1, 
  .freq     = kBOOT_FREQ,
  .repeat   = 10,
  .duration = 50,
  .interval = 50
};

snd_evts_t postpone_snd = {
  .snd_evt  = 1, 
  .freq     = kPOSTPONE_FREQ,
  .repeat   = 2,
  .duration = 500,
  .interval = 500
};

/*
 * --------------------------------------------------------------------
 * Drive the buzzer
 * --------------------------------------------------------------------
 */
void drive_buzzer()
{
  typedef enum { e_sys_wait_buz, e_play_buz, e_end_buz, e_silence_buz} buz_state_t;
  static uint8_t buz_state = e_sys_wait_buz;
  static unsigned long timer = 0;


 switch ( buz_state ) {
  // Waiting for a sound to emit...
  case e_sys_wait_buz:
    if ( g_snd.snd_evt )
    {
      g_snd.repeat--;
      timer = millis()+g_snd.duration;
      tone(BUZZER_PIN,g_snd.freq);     
      buz_state = e_play_buz;
    }
    break;
    
  // Play the sound
  case e_play_buz:
      if ( millis() > timer ) {
        noTone(BUZZER_PIN);
        buz_state = e_end_buz;
      }
    break;
    
  // Stop the sound
  case e_end_buz:
    if ( g_snd.repeat != 0 ) { 
        g_snd.repeat--;
        timer = millis()+g_snd.interval;
        buz_state = e_silence_buz;
    }
    else
      buz_state = e_sys_wait_buz;
    break;
    
  // Silence interval between two sounds
  case e_silence_buz:
    if ( millis() > timer ) {
      tone(BUZZER_PIN,g_snd.freq); 
      timer = millis()+g_snd.duration;
      buz_state = e_play_buz;
    }
    break;
  default:
    break;
 }
}

/* 
 * --------------------------------------------------------------------
 * Drive LEDs
 * cmd=0: OFF
 * cmd=1: ON
 * cmd=2: slow blink
 * cmd=3: fast blink
 * --------------------------------------------------------------------
 */
void drive_leds()
{
  static unsigned long timer[3] = {0,0,0};
  static uint8_t state[3] = {0,0,0};
  unsigned long t = millis();
 
  for ( uint8_t i = 0; i < 3; i++) 
  {
    if ( g_led_cmd[i] == eOff) 
       state[i]=0; 
    else
    if ( g_led_cmd[i] == eOn) 
       state[i]=1;       
    else
      if ( t>= timer[i]) 
      {
        state[i]=1-state[i];
        timer[i] = t+kBLINK_HALF_PERIOD[g_led_cmd[i]-2];
      }
  }

  digitalWrite(LED_R_PIN,state[0]);
  digitalWrite(LED_G_PIN,state[1]);
  digitalWrite(LED_B_PIN,state[2]);
  
}  

 
/* 
 * --------------------------------------------------------------------
 * Display the state data
 * --------------------------------------------------------------------
 */
void dsp_status(sys_state_t g_sys_state,  pwr_lvl_t  pwr_lvl[]) 
{
    lcd.clear();
    switch (g_sys_state) 
    {
      // Power on
      case e_sys_pwr_on:
        lcd.print("Powering on");
        break;
      // Nominal mode
      case e_sys_nominal:
        lcd.setCursor(0,0);       
        lcd.print(( g_ctrl_state[0] == e_ctrl_manual ) ? "B0: (man)" : "B0: (auto)");   
        lcd.setCursor(11,0);
        lcd.print(kBAT_EVTS_TO_STR[pwr_lvl[0]]);   
        lcd.setCursor(0,1);                 
        lcd.print(( g_ctrl_state[1] == e_ctrl_manual ) ? "B1: (man)" : "B1: (auto)");  
        lcd.setCursor(11,1);
        lcd.print(kBAT_EVTS_TO_STR[pwr_lvl[1]]);            
        break;                   
      default:
        break;
    }
 }

/* 
 * --------------------------------------------------------------------
 * Display voltages and currents
 * --------------------------------------------------------------------
 */
void dsp_va() {
    lcd.clear();
    // Display battery voltages 
    lcd.setCursor(0,0);
    lcd.print("B0:");
    lcd.setCursor(0,1);
    lcd.print("B1:");     
     
    dtostrf(g_vbat[0], 4, 1, str);  //4 is mininum width, 2 is precision
    strcat(str,"V");
    lcd.setCursor(3,0);
    lcd.print(str);     
    dtostrf(g_vbat[1], 4, 1, str);  //4 is mininum width, 2 is precision
    strcat(str,"V");
    lcd.setCursor(3,1);
    lcd.print(str);

    // Display battery currents
    dtostrf(g_ibat[0], 3, 2, str);  //4 is mininum width, 2 is precision
    strcat(str,"A");
    lcd.setCursor(10,0);
    lcd.print(str);
    dtostrf(g_ibat[1], 3, 2, str);  //4 is mininum width, 2 is precision
    strcat(str,"A");
    lcd.setCursor(10,1);       
    lcd.print(str);  
}

/* 
 * --------------------------------------------------------------------
 * Display cell voltages
 * --------------------------------------------------------------------
 */
void dsp_cells() {
    lcd.clear();
    // Display battery voltages 
    lcd.setCursor(0,0);
    sprintf(str,"B%i:", g_sel_bat);
    lcd.print(str);
    
    dtostrf(g_vcell[0+4*g_sel_bat], 4, 1, str); 
    strcat(str,"V");
    lcd.setCursor(3,0);
    lcd.print(str);  

    dtostrf(g_vcell[1+4*g_sel_bat], 4, 1, str); 
    strcat(str,"V");
    lcd.setCursor(3,1);
    lcd.print(str);  
    
    dtostrf(g_vcell[2+4*g_sel_bat], 4, 1, str); 
    strcat(str,"V");
    lcd.setCursor(10,0);
    lcd.print(str);  

    dtostrf(g_vcell[3+4*g_sel_bat], 4, 1, str); 
    strcat(str,"V");
    lcd.setCursor(10,1);
    lcd.print(str);  
    
}     

/* 
 * --------------------------------------------------------------------
 * Display pwr lvl change page
 * --------------------------------------------------------------------
 */
void dsp_chg_lvl()
{
  lcd.clear();
  sprintf(str,"B%i:", g_sel_bat);
  lcd.print(str);
  lcd.setCursor(4,0);
  lcd.print("cur");
  lcd.setCursor(11,0);
  lcd.print(kBAT_EVTS_TO_STR[g_pwr_lvl[g_sel_bat]]);
  lcd.setCursor(4,1);
  lcd.print("nxt");
  lcd.setCursor(11,1);  
  lcd.print(kBAT_EVTS_TO_STR[g_pwr_lvl_user[g_sel_bat]]);
}

/* 
 * --------------------------------------------------------------------
 * Display pwr lvl change page
 * --------------------------------------------------------------------
 */
void dsp_g_sel_bat()
{
  lcd.clear();
  sprintf(str,"B%i:", g_sel_bat);
  lcd.print(str);
  lcd.setCursor(4,0);
  lcd.print("cur");
  lcd.setCursor(11,0);
  lcd.print(kBAT_EVTS_TO_STR[g_pwr_lvl[g_sel_bat]]);
}

/*
 * --------------------------------------------------------------------
 * Compute the bat events vector
 * --------------------------------------------------------------------
 */
void get_key_events ( key_evts_t *key_evts )
{
  static uint16_t timer = 0;
  static uint8_t prev_v=0, prev_filt_v;
  uint8_t v = 0;
  
  v = (~((digitalRead(KEY_R_PIN)  << 3) |
         (digitalRead(KEY_Y1_PIN) << 2) |
         (digitalRead(KEY_Y2_PIN) << 1) |
         (digitalRead(KEY_G_PIN)))) & 0b1111 ;
 
          
  // Debounce (kDEBOUNCE_MS confirmation)
  if ( v != prev_v ) 
  {
    prev_v = v;
    timer = millis();

  }
  else
  {
    if ( millis()-timer > kDEBOUNCE_MS)
    {
      v = ( ~prev_filt_v & prev_v) ;
      key_evts->red_pressed_evt =      (v & 0b1000) >> 3;
      key_evts->yellow1_pressed_evt =  (v & 0b0100) >> 2;
      key_evts->yellow2_pressed_evt =  (v & 0b0010) >> 1;
      key_evts->green_pressed_evt =    (v & 0b0001);
      prev_filt_v=prev_v;
    }
  }
}

/*
 * --------------------------------------------------------------------
 * Compute the pwr lvl
 * --------------------------------------------------------------------
 */
void update_pwr_lvl_bat ( )
{
  const float kBAT_THRESHOLDS[] = { 3.5, 3.7, 3.9, 4.1, 4.3, 4.8 };


  // Events for battery levels (Bat 0)
  for (int i=0; i< 6; i++) 
    if ( g_vmin[0] < kBAT_THRESHOLDS[i] ) 
    {
        if ( i < g_pwr_lvl_bat[0] )
         g_pwr_lvl_bat[0] = i; 
       break;
    }  
  
  // Events for battery levels (Bat 1)
  for (int i=0; i< 6; i++) 
    if ( g_vmin[1] < kBAT_THRESHOLDS[i] )
    {  
        if ( i < g_pwr_lvl_bat[1] )
        g_pwr_lvl_bat[1] = i;         
        break;
    }
}

/*
 * --------------------------------------------------------------------
 * Compute the overcurrent conditions
 * --------------------------------------------------------------------
 */
void update_ovrcur ()
{
  // Overcurrent (Bat 1)
  if ( g_ibat[0] > 3.0 ) 
    g_ovrcur[0] = true; 
    
  if ( g_ibat[1] > 3.0 ) 
    g_ovrcur[1] = true; 

  
}

/*
 * --------------------------------------------------------------------
 * Set the channels states according to the PWR LVL
 * --------------------------------------------------------------------
 */
void set_pwr_channels() {
   // Manage channels for battery 0
   switch (  g_pwr_lvl[0] )
    {
       case 0:
        g_channels_status[0] = LOW;
        g_channels_status[1] = LOW;
        g_channels_status[2] = LOW;
        g_channels_status[3] = LOW;
        break;
       case 1:
        g_channels_status[0] = HIGH;
        g_channels_status[1] = LOW;
        g_channels_status[2] = LOW;
        g_channels_status[3] = LOW;  
        break;
       case 2:
        g_channels_status[0] = HIGH;
        g_channels_status[1] = HIGH;
        g_channels_status[2] = LOW;
        g_channels_status[3] = LOW;
        break;
       case 3:
        g_channels_status[0] = HIGH;
        g_channels_status[1] = HIGH;
        g_channels_status[2] = LOW;
        g_channels_status[3] = LOW;
        break;
       case 4:
        g_channels_status[0] = HIGH;
        g_channels_status[1] = HIGH;
        g_channels_status[2] = HIGH;
        g_channels_status[3] = LOW;        
        break;
       case 5: 
        // Power up all channels
        g_channels_status[0] = HIGH;
        g_channels_status[1] = HIGH;
        g_channels_status[2] = HIGH;
        g_channels_status[3] = HIGH;           
    }

    // Manage channels for battery 1
    // Nothing to do for the moment...
}
/*
 * --------------------------------------------------------------------
 * Transitions of the DSP FSM
 * --------------------------------------------------------------------
 */

// Transition to the voltages and currents page
void to_dsp_va() {
  dsp_va();      
  g_dsp_state = e_dsp_va;
  g_dsp_next_time = millis()+kPAGE_DSP_DURATION;
}

// Transition to the battery cell voltage page
void to_dsp_cells() {
  dsp_cells();
  g_dsp_state = e_dsp_cells;
  g_dsp_next_time = millis()+kPAGE_DSP_DURATION;
}

// Transition to the global status page
void to_dsp_status() {
  dsp_status(g_sys_state, g_pwr_lvl);
  g_dsp_state = e_dsp_status;
  g_dsp_next_time = millis()+kPAGE_DSP_DURATION;
}

// Transition to the PWR LVL change page
void to_dsp_chg_lvl() {
  // Initialize the requested level
  g_pwr_lvl_user[0] = g_pwr_lvl[0];
  g_pwr_lvl_user[1] = g_pwr_lvl[1];
  dsp_chg_lvl();
  g_dsp_state = e_dsp_chg_lvl;
}

// Transition to the PWR LVL change page (2)
void to_dsp_chg_lvl_1() {
  dsp_chg_lvl();
  g_dsp_state = e_dsp_chg_lvl;
}

// Transition to battery selection page
void to_dsp_sel_bat() {
  dsp_g_sel_bat();
  g_dsp_state = e_dsp_sel_bat;    
}


/*
 * --------------------------------------------------------------------
 * LEDs management
 * --------------------------------------------------------------------
 */
 
/*
 * Set the LEDs comands in case of overcurrent
 */
void set_leds_ovrcur()
{
  g_led_cmd[0] = eOn;
  g_led_cmd[1] = eOff;   
}

/*
 * Set the LEDs commands according to the PWR LVL of BAT0
 */
void set_leds_lvl()
{

  g_led_cmd[0] = kRED_LED_LVLS[g_pwr_lvl[0]];
  g_led_cmd[1] = kGREEN_LED_LVLS[g_pwr_lvl[0]];   
}

/*
 * --------------------------------------------------------------------
 * Acquisition of voltages and currents for the two batteries
 * --------------------------------------------------------------------
 */
void get_vi()
{
  float v;
  uint32_t a;

  /*
   * Voltage acquisition
   */

  a = 0;
  // Measure voltage on the channel g_ch.
  // Do 50 acquisitions...
  for ( int i = 0; i < 50 ; i++)
    a += analogRead(ANA_MUX_1_PIN);  

  // Compute voltage for the current cell
  g_vcell[g_ch] = (a/50.0/1024.0)*5.0*4.0;

  //  Save bat voltage (last cell)
  if ( g_ch == 3 ) g_vbat[0] = g_vcell[3]; 
  else
  if ( g_ch == 7 ) g_vbat[1] = g_vcell[7]; 

  // Compute actual V for each cell
  if ( g_ch >= 4)
    for (int i=g_ch-1; i>=4; i--)
      g_vcell[g_ch] -= g_vcell[i];
  else
    for (int i=g_ch-1; i>=0; i--)
      g_vcell[g_ch] -= g_vcell[i];

  /*
   * Current acquisition
   */
  
  // From the ACS712 x05B: sensitivity is 185 mV/A
  // Do 50 acquisitions...
  a = 0;
  for ( int i = 0; i < 50 ; i++)
    a += analogRead(ANA_I1_PIN);
    
  v = -((a/50.0/1024.0)*5.0-2.5) / 185e-3;
  g_ibat[0] = (v <= 0.0) ? 0.0 : v ;
      
  a = 0;
  for ( int i = 0; i < 50 ; i++)
    a += analogRead(ANA_I2_PIN);

  v = -((a/50.0/1024.0)*5.0-2.5) / 185e-3;
  g_ibat[1] = (v <= 0.0) ? 0.0 : v ;


  // Compute vmin
  g_vmin[0]=1000.0; // Arbitrary value greater than any voltage  
  for (int i=0; i<4; i++)
      if ( g_vcell[i] < g_vmin[0]) g_vmin[0]=g_vcell[i];
  g_vmin[1]=1000.0; // Arbitrary value greater than any voltage       
  for (int i=4; i<8; i++)
      if ( g_vcell[i] < g_vmin[1]) g_vmin[1]=g_vcell[i];

}

/* 
 * --------------------------------------------------------------------
 * Get can messages
 * --------------------------------------------------------------------
 */
void get_can_msgs()
{
  uint8_t len = 0;
  unsigned char buf[8];
  
  // Process CAN commands, if any
  if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
  {

    switch ( CAN.getCanId() ){
      case K_CAN_BAT_POSTPONE_ID:
          g_postpone_evt = 1;
          break;
      case K_CAN_BAT_SETLVL_ID:
          CAN.readMsgBuf(&len, buf);   // read data,  len: data length, buf: data buf
          // Set the new PWR LEVEL for BAT 0
          if (buf[1] == 6)
          {
            g_snd  = auto_snd;        
            g_ctrl_state[buf[0]] = e_ctrl_auto;  
          }
          else
          {
           // Set the mode to MANUAL
            g_ctrl_state[g_sel_bat] = e_ctrl_manual;
            // Generate a sound signal
            g_snd  = manual_snd; 
            // Set the power level            
            g_pwr_lvl[buf[0]] = buf[0];
            // Send the new PWR LVL via CAN
            g_send_pwr_lvl_msg_evt = 1;
            // Set the power channels according to the power level
            set_pwr_channels();   
          }
          break;
      default:
        break;
    }
  }
}
/* 
 * --------------------------------------------------------------------
 * Main loop
 * --------------------------------------------------------------------
 */
void loop() {
  static uint16_t clk = 0;
  key_evts_t key_evts; 

/*
 * Clear all events
 */
  key_evts.red_pressed_evt = 0;
  key_evts.yellow1_pressed_evt = 0;
  key_evts.yellow2_pressed_evt = 0;
  key_evts.green_pressed_evt = 0;
  g_snd.snd_evt = 0;
  g_postpone_evt = 0;
  g_send_pwr_lvl_msg_evt = 0;

/*
 * Get can messages
 */
get_can_msgs();

/*
 * Acquire V and I
 * (in any mode)
 */
get_vi();

/*
 * Update the key events
 */
get_key_events (&key_evts);

/*
* Manage sounds
* UP and DN keys always generate the same sounds.
*/
if ( key_evts.yellow1_pressed_evt ) g_snd  = up_snd;
if ( key_evts.yellow2_pressed_evt ) g_snd  = dn_snd;


/*
 * Compute ovr current conditions
 */
update_ovrcur();
      
/*
 * Overcurrent protection
 * (in any mode)
 */
if ( g_ovrcur[0] )
{
    // Stop all channels (this also stop the processor)
    g_channels_status[0] = LOW;
    g_channels_status[1] = LOW;
    g_channels_status[2] = LOW;
    g_channels_status[3] = LOW;
    set_leds_ovrcur();  // For test purposes (when the channel control is inhibited)
}

/*
 * Mode management
 */
  switch (g_sys_state) {
    // Normal power on
    case e_sys_pwr_on:
      // Switch all LEDs on
      g_led_cmd[0] = eOn;    
      g_led_cmd[1] = eOn;    
      g_led_cmd[2] = eOn;    
      g_sys_state = e_sys_wait;
      g_sys_next_time = millis()+1000;
      g_snd  = boot_snd;
      break;
      
    // Error case
    case e_sys_error:
      g_led_cmd[0] = eSlowBlk;    
      g_led_cmd[1] = eSlowBlk;    
      g_led_cmd[2] = eSlowBlk;
      break;     
    
    case e_sys_wait:
      if (millis() >= g_sys_next_time ) 
         g_sys_state = e_sys_nominal;
      break;

    case e_sys_nominal:

      // Compute the pwr lvl
      update_pwr_lvl_bat();
      
      /*
       * Battery control management BAT0
       */
      switch (g_ctrl_state[0]) {
    
        case e_ctrl_manual:
          g_led_cmd[2] = eFastBlk;
          break;

        // Wait for a power level mode change
        case e_ctrl_auto:
          g_led_cmd[2] = eSlowBlk;
          // Set the power level to the value given from the battery measurements
          g_pwr_lvl[0] = g_pwr_lvl_bat[0];     
          // Check if there has been a power level change
          if (g_pwr_lvl[0] != g_pwr_lvl_prev[0] ) 
          {
            // Yes. Wait until all sub-systems have been given the chance to postpone the action.
            g_ctrl_next_time[0] = millis()+k_CHANNEL_CHANGE_DELAY;
            g_led_cmd[2] = eVeryFastBlk;
            g_send_pwr_lvl_msg_evt = 1;
            g_ctrl_state[0] = e_ctrl_confirm;
          }
          break;

       // Wait for confirmation of power level change
       case e_ctrl_confirm:
          // Set the power level to the value given from the battery measurements
          g_pwr_lvl[0] = g_pwr_lvl_bat[0];  
          // Check if there has been a power level change
          if (g_pwr_lvl[0] != g_pwr_lvl_prev[0] ) 
          {
            // Yes. Wait again.
            g_ctrl_next_time[0] = millis()+k_CHANNEL_POSTPONE_DELAY;
            g_led_cmd[2] = eVeryFastBlk; 
            g_send_pwr_lvl_msg_evt = 1;
          } 
          else
          // Check if we have received a postpone message
          if ( g_postpone_evt ) 
          {
            // YES: play the postpone sound
            g_snd  = postpone_snd;            
            // And wait for another delay
            g_ctrl_next_time[0] = millis()+k_CHANNEL_CHANGE_DELAY;
          }
          else
          // Is the wait period completed?
          if ( millis() >= g_ctrl_next_time[0] )
          {
            // YES: play the channel update sound
            g_snd  = chanset_snd;
            // Set the power channels according to the power level
            set_pwr_channels();
            // And go back to the auto mode.
            g_ctrl_state[0] = e_ctrl_auto; 
          }
           
          break;
                   
        default:
          break;    
      }
    
    /*
     * Battery control management BAT1
     */
      switch (g_ctrl_state[1]) {
    
        case e_ctrl_manual:
          break;
    
        case e_ctrl_auto:
          g_pwr_lvl[1] = g_pwr_lvl_bat[1];        
          break;
              
        default:
          break;    
      }
    
      break;
          
    default:
      break;
  }
     

  /*
   * DISPLAY MANAGEMENT
   */
  switch ( g_dsp_state ) {      
    case e_dsp_start:
      to_dsp_va();
      break;

    // Display voltage and current
    case e_dsp_va:
      if (( millis() >= g_dsp_next_time ) || (key_evts.yellow1_pressed_evt))
      {
        g_sel_bat = 0;
        to_dsp_cells();
      }
      else
      if (key_evts.yellow2_pressed_evt)  
        to_dsp_status();
      else
      if (key_evts.red_pressed_evt)
      {
        g_snd  = valid_snd;        
        to_dsp_sel_bat();
      }
      else
      if (key_evts.green_pressed_evt) {
          g_snd  = auto_snd;        
          g_ctrl_state[0] = e_ctrl_auto;  
          g_ctrl_state[1] = e_ctrl_auto;      
          to_dsp_status();
      }
      break;

    // Display cells voltages
    case e_dsp_cells:
      if ( (millis() >= g_dsp_next_time ) || (key_evts.yellow1_pressed_evt)) {
        if ( g_sel_bat == 0 ) {
          g_sel_bat=1;
          to_dsp_cells();
        }
        else
          to_dsp_status();
      }
      else
      if (key_evts.yellow2_pressed_evt) {
        if ( g_sel_bat == 1 ){
          g_sel_bat = 0;
          to_dsp_cells();
        }
        else
          to_dsp_va();
      }
      else
      if (key_evts.red_pressed_evt)
      {
        g_snd  = valid_snd;
        to_dsp_sel_bat();
      }
      else
      if (key_evts.green_pressed_evt) {
          g_snd  = auto_snd;        
          g_ctrl_state[0] = e_ctrl_auto;  
          g_ctrl_state[1] = e_ctrl_auto;       
          to_dsp_status();
      }        
      break;

    // Display PWR LEVEL
    case e_dsp_status:
      if ( (millis() >= g_dsp_next_time ) || (key_evts.yellow1_pressed_evt)) 
        to_dsp_va();
      else
      if (key_evts.yellow2_pressed_evt) 
      {
        g_sel_bat = 1;
        to_dsp_cells();  
      }
      else
      if (key_evts.red_pressed_evt)
      {
        g_snd  = valid_snd;
        g_sel_bat = 0;
        to_dsp_sel_bat();
      }
      else
      if (key_evts.green_pressed_evt) {
          g_snd  = auto_snd;        
          g_ctrl_state[0] = e_ctrl_auto;  
          g_ctrl_state[1] = e_ctrl_auto;                   
          to_dsp_status();
      }        
      break;

    // Select battery
    case e_dsp_sel_bat:
      if ((key_evts.yellow1_pressed_evt) || (key_evts.yellow2_pressed_evt)) {
          g_sel_bat = 1-g_sel_bat;
          to_dsp_sel_bat();
      }
      else   
      if( key_evts.green_pressed_evt )
      {
        g_snd  = cancel_snd;
        to_dsp_va();
      }
      else
      if (key_evts.red_pressed_evt)
      {
        g_snd  = valid_snd;
        to_dsp_chg_lvl();      
      }
      break;

    case e_dsp_chg_lvl:
     if ( key_evts.yellow1_pressed_evt )
     {
        if (g_pwr_lvl_user[g_sel_bat] < 6) 
          g_pwr_lvl_user[g_sel_bat]++;
        else 
          g_pwr_lvl_user[g_sel_bat] = 0;
        to_dsp_chg_lvl_1();
     }
     else
     if ( key_evts.yellow2_pressed_evt )
     {
        if (g_pwr_lvl_user[g_sel_bat] > 1) 
          g_pwr_lvl_user[g_sel_bat]--;
        else 
          g_pwr_lvl_user[g_sel_bat] = 6;
        to_dsp_chg_lvl_1();
     }
     else 
     if( key_evts.green_pressed_evt ) 
     {
        g_snd  = cancel_snd;
        to_dsp_va();
     }
     else 
     if( key_evts.red_pressed_evt ) 
     {
      // If AUTO was selected
      if ( g_pwr_lvl_user[g_sel_bat] == 6 )
      {
        // Set the mode to AUTO
        g_ctrl_state[g_sel_bat] = e_ctrl_auto;
        // Generate a sound signal        
        g_snd  = auto_snd;
      }
      else
      {
        // Set the mode to MANUAL
        g_ctrl_state[g_sel_bat] = e_ctrl_manual;
        // Generate a sound signal
        g_snd  = manual_snd; 
        // Set the new PWR LEVEL
        g_pwr_lvl[g_sel_bat] = g_pwr_lvl_user[g_sel_bat];
        // Send the new PWR LVL via CAN
        g_send_pwr_lvl_msg_evt = 1;
        // Set the power channels according to the power level
        set_pwr_channels();        
      }
      to_dsp_status();      
     }
     break;
      
    default:
      break;
  }

  
  /*
   * CAN MANAGEMENT
   */
  if ( clk == 0) { 
    // Transmit cells voltage for bat 0
    CAN.sendMsgBuf(K_CAN_BAT_V_ID, 0, 8, (uint8_t *)&g_vcell[0]);
  }
  else
  if ( clk == 100 ) {
    //  Transmit cells voltage for bat 1
    CAN.sendMsgBuf(K_CAN_BAT_V_ID, 0, 8, (uint8_t *)&g_vcell[4]);
  }
  else  
  if ( (clk == 333 ) || g_send_pwr_lvl_msg_evt ) 
  { 
    // Transmit pwr lvl      
    CAN.sendMsgBuf(K_CAN_BAT_LVL_ID, 0, 2, (uint8_t *) g_pwr_lvl);
  }
  else
  if ( clk == 666 ) {
    // Transmit the current data for bat 0 and bat 1
    CAN.sendMsgBuf(K_CAN_BAT_I_ID, 0, 8, (uint8_t *)g_ibat);
  }

  // Set the LEDs
  set_leds_lvl();
        
  // Drive LEDs
  drive_leds();

  // Drive the power channels
  drive_pwr_channels();

  // Drive the buzzer
  drive_buzzer();

  // Select the analog multiplexer channel
  g_ch = (g_ch+1) & 0b111; 
  setMux(0, g_ch) ;

  // Save previous pwr lvl
  g_pwr_lvl_prev[0] = g_pwr_lvl[0];
  g_pwr_lvl_prev[1] = g_pwr_lvl[1];  

  // Update the clock
  clk = ( clk < 1000 ) ? clk+1: 0;

  // Wait till next cycle
  delay(10);


}
