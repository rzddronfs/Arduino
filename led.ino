#include <arduino.h>
#include <ctc_timers.h>
#include <spi_assert.h>


// globals
volatile static int n_pulses_of_fan_reed_switch;
volatile static int n_servo_counter_pulse_ticks = 46; // 1500 mcs for clk/256 timer resolution


// logic
struct Led_fade_in_network_data_t {
  enum STATE {
    ST_INIT,
    ST_TRG_WAIT,
    ST_BTN_PUSHED,
    ST_LOW_SIG,
    ST_HIGH_SIG,
    ST_NEXT_RATIO,  
    ST_FINAL
  };

  int state;
  int lo_sig_part;
  int hi_sig_part;
  int delta;
  unsigned long time;      
};

int run_led_fade_in_network( int st_trg, struct Led_fade_in_network_data_t* state ) // st_trg - state of triggering pin
{    
  typedef struct Led_fade_in_network_data_t LFINDT;
  typedef LFINDT STATE;
  
  int st_ret = LOW;
  
  ASSERT( state );

  switch( state->state )
  {
    case STATE::ST_INIT:
      state->lo_sig_part = 500;
      state->hi_sig_part = 500;     
      state->delta = 10000 / state->lo_sig_part;
      state->time = 0;              
        
      state->state = STATE::ST_TRG_WAIT;
    break;
    
    case STATE::ST_TRG_WAIT:
      if( st_trg == LOW )
      {
        state->state = STATE::ST_BTN_PUSHED;
      }    
    break;
    
    case STATE::ST_BTN_PUSHED:
      --state->lo_sig_part;
      ++state->hi_sig_part;
      state->time = micros();
      state->state = STATE::ST_LOW_SIG;
    break;
    
    case STATE::ST_LOW_SIG:
      st_ret = LOW;
      if( micros() - state->time > state->lo_sig_part * state->delta )
      {        
        state->state = STATE::ST_HIGH_SIG;
      }
    break;
    
    case STATE::ST_HIGH_SIG:
      st_ret = HIGH;
      if( micros() - state->time > state->hi_sig_part * state->delta )
      {
        state->state = STATE::ST_NEXT_RATIO;
      }    
    break;
    
    case STATE::ST_NEXT_RATIO:
      state->state = STATE::ST_FINAL;
      if( state->lo_sig_part > 0 )
      {
        state->state = STATE::ST_BTN_PUSHED;
      }
    break;
    
    case STATE::ST_FINAL:
      state->state = STATE::ST_INIT;
    break;
  }// switch
  
  return st_ret;
}


struct Led_fade_out_network_data_t {
  enum STATE {
    ST_INIT,
    ST_TRG_WAIT,
    ST_BTN_PUSHED,
    ST_LOW_SIG,
    ST_HIGH_SIG,
    ST_NEXT_RATIO,  
    ST_FINAL
  }; 
 
  int state;
  int lo_sig_part;
  int hi_sig_part;
  int delta;
  unsigned long time;  
};

int run_led_fade_out_network( int st_trg, struct Led_fade_out_network_data_t* state ) // st_trg - state of triggering pin
{  
  typedef struct Led_fade_out_network_data_t LFONDT;
  typedef LFONDT STATE;
  
  int st_ret = LOW;
  
  ASSERT( state );

  switch( state->state )
  {
    case STATE::ST_INIT:
      state->lo_sig_part = 0;
      state->hi_sig_part = 1000;
      state->delta = 10000 / state->hi_sig_part;
      state->time = 0;
      
      state->state = STATE::ST_TRG_WAIT;
    break;
    
    case STATE::ST_TRG_WAIT:
      if( st_trg == LOW )
      {
        state->state = STATE::ST_BTN_PUSHED;
      }
    break;
    
    case STATE::ST_BTN_PUSHED:
      ++state->lo_sig_part;
      --state->hi_sig_part;
      state->time = micros();
      state->state = STATE::ST_LOW_SIG;
    break;
    
    case STATE::ST_LOW_SIG:
      st_ret = LOW;
      if( micros() - state->time > state->lo_sig_part * state->delta )
      {        
        state->state = STATE::ST_HIGH_SIG;
      }
    break;
    
    case STATE::ST_HIGH_SIG:
      st_ret = HIGH;
      if( micros() - state->time > state->hi_sig_part * state->delta )
      {
        state->state = STATE::ST_NEXT_RATIO;
      }    
    break;
    
    case STATE::ST_NEXT_RATIO:
      state->state = STATE::ST_FINAL;
      if( state->hi_sig_part > 0 )
      {
        state->state = STATE::ST_BTN_PUSHED;
      }
    break;
    
    case STATE::ST_FINAL:
      state->state = STATE::ST_INIT;
    break;
  }// switch
  
  return st_ret;
}


int run_pwm_pulse_st_network( unsigned long* ps_hi_start_mcs, long period_mcs, long ps_duration_mcs )
{
 int result_st = LOW;
 unsigned long actual_time = micros(); 
  
 ASSERT( ps_hi_start_mcs && period_mcs >= 0 && ps_duration_mcs >= 0 && ps_duration_mcs <= period_mcs );
 
 if( actual_time - *ps_hi_start_mcs <= ps_duration_mcs )
 {
   result_st = HIGH;
 }
 
 if( actual_time - *ps_hi_start_mcs >= period_mcs )
 {
   *ps_hi_start_mcs = actual_time;
 }  
 
 return result_st;
}


int is_in_timeout( unsigned long* tout_from_mcs, unsigned long tout_duration_mcs )
{
  int result = 1;
  
  ASSERT( tout_from_mcs != 0 && tout_duration_mcs >=0 );
    
  if( micros() - *tout_from_mcs > tout_duration_mcs )
  {
    result = 0;
  }
    
  return result;
}


struct Speed_change_detection_data_t {
  enum STATE {
    ST_INIT,
    ST_TRG_WAIT,
    ST_ACCEL_BTN_PUSHED,
    ST_SLOWDN_BTN_PUSHED,
    ST_TOUT_INIT,
    ST_TOUT,
    ST_FINAL
  };
  
  int state;
  unsigned long time_buf;
  unsigned long tout_mcs;
};

int detect_speed_change( int accel, int slowdn, struct Speed_change_detection_data_t* state ) 
{
  typedef struct Speed_change_detection_data_t SCDDT; 
  typedef SCDDT STATE; 
  
  int result = 0;
  
  ASSERT( state );

  switch( state->state )  
  {
    case STATE::ST_INIT: 
      state->time_buf = 0;
      state->tout_mcs = 10000;
      
      state->state = STATE::ST_TRG_WAIT; 
    break;
    
    case STATE::ST_TRG_WAIT:
      if( accel == LOW ) state->state = STATE::ST_ACCEL_BTN_PUSHED;
      if( slowdn == LOW ) state->state = STATE::ST_SLOWDN_BTN_PUSHED;   
    break;
    
    case STATE::ST_ACCEL_BTN_PUSHED:
      ++result;
      state->state = STATE::ST_TOUT_INIT;
    break;
    
    case STATE::ST_SLOWDN_BTN_PUSHED:
      --result;
      state->state = STATE::ST_TOUT_INIT;
    break;
    
    case STATE::ST_TOUT_INIT:
      state->time_buf = micros();
      state->state = STATE::ST_TOUT;
    break;
    
    case STATE::ST_TOUT:    
      if( !is_in_timeout( &state->time_buf, state->tout_mcs ) ) state->state = STATE::ST_FINAL;
    break;
    
    case STATE::ST_FINAL:
      state->state = STATE::ST_TRG_WAIT;
    break;
  }

  ASSERT( result == -1 || result == 0 || result == 1 );
  return result;
}

int is_detect_speed_change_timeout( struct Speed_change_detection_data_t* scdd )
{     
  typedef struct Speed_change_detection_data_t SCDDT; 
  typedef SCDDT STATE;   
  
  ASSERT( scdd );
  
  return (scdd->state != STATE::ST_TRG_WAIT);
}


struct Fan_slowdown_measurement_data_t {
  enum STATE {
    ST_INIT,
    ST_MEASURE,
    ST_FINAL
  }; 

  enum {
    NORMAL_RATE = 4,
    DEFAULT_TIME_DIF_MS = 500
  };
  
  int state;
  int last_n_ps;
  int result;
  unsigned long time_buf;
  int last_ps_dif;  
  int time_dif;
};

// returns the number of pulses absent from the second measure of the measure_interval_n_ps provided,  
// when the first measure determines measure_interval_n_ps time span.
int get_slowdn_rate( struct Fan_slowdown_measurement_data_t* state )
{
  typedef struct Fan_slowdown_measurement_data_t FSMDT;  
  typedef FSMDT STATE;  
  
  const int n_ps = n_pulses_of_fan_reed_switch;  
  const unsigned long actual_time = millis();
  
  ASSERT( state );
  
  switch( state->state )
  {
    case STATE::ST_INIT:      
      state->result = 0;
      state->last_ps_dif = 0;        
      state->time_buf = actual_time;
      state->last_n_ps = n_ps;
      state->time_dif = STATE::DEFAULT_TIME_DIF_MS;
      
      state->state = STATE::ST_MEASURE;
    break;
    
    case STATE::ST_MEASURE:      
      if( actual_time - state->time_buf >= state->time_dif )
      {
        int actual_diff = n_ps - state->last_n_ps;
        state->result = state->last_ps_dif - actual_diff;   
        state->last_ps_dif = actual_diff;
        state->last_n_ps = n_ps;    
        state->time_buf = actual_time; 
        
        state->state = STATE::ST_FINAL;
      }
    break;  
    
    case STATE::ST_FINAL:  
      state->result = 0;    
      state->state = STATE::ST_MEASURE;
    break;    
  }
  
  return state->result;
}

int run_fan_startup_phase_1( unsigned long*, long, long )
{
  return HIGH;
}

int run_fan_startup_phase_2( unsigned long*, long, long )
{
  return HIGH;
}

int run_fan_startup_phase_3( unsigned long*, long, long )
{
  return HIGH;
}

struct Fan_control_data_t {
  enum STATE {
    ST_INIT,
    ST_SETPOINT_CONTROL,
    ST_INPUT_CONTROL,
    ST_OUTPUT_CONTROL,
    ST_EMERG_STOP,
    ST_FINAL
  };
  
  enum {
    DEFAULT_PERIOD_MCS = 1000
  };
  
  typedef int (*Output_control_func_t)( unsigned long*, long, long );
  
  int state;
  unsigned long ps_hi_start_mcs;
  int period_mcs;
  int ps_duration_mcs;
  int is_startup;  
  Output_control_func_t ocf;
  struct Speed_change_detection_data_t scdd;
  struct Fan_slowdown_measurement_data_t fsmd;
};

int control_fan( int accel, int slowdn, struct Fan_control_data_t* state )
{
  typedef struct Fan_control_data_t FCDT;
  typedef FCDT STATE;
  typedef struct Speed_change_detection_data_t SCDDT;
  typedef struct Fan_slowdown_measurement_data_t FSMDT;  
  
  int st_ret = LOW;
  
  ASSERT( state );
  
  switch( state->state )
  {
    case STATE::ST_INIT:      
      {                   
        state->scdd = {0};
        state->fsmd = {0};

        state->ps_hi_start_mcs = 0;
        state->period_mcs = STATE::DEFAULT_PERIOD_MCS;
        state->ps_duration_mcs = 0;  
        state->is_startup = 0;
        state->ocf = run_pwm_pulse_st_network; 
        
        state->state = STATE::ST_SETPOINT_CONTROL;
      }
    break;
   
    case STATE::ST_INPUT_CONTROL: 
      {  
        int slowdn_rate = get_slowdn_rate( &state->fsmd );
        
        state->state = STATE::ST_SETPOINT_CONTROL; 
        
        if( slowdn_rate > FSMDT::NORMAL_RATE )
        {
          state->state = STATE::ST_EMERG_STOP;         
          break;
        }
 
        if( state->ocf == run_fan_startup_phase_1 && slowdn_rate != 0 )
        {
          state->ocf = run_fan_startup_phase_3;
        }
        
        /*
        if( state->ocf == run_fan_startup_phase_2 && slowdn_rate > 0 )
        {
          state->ocf = run_fan_startup_phase_3;
        }
        */

        if( state->ocf == run_fan_startup_phase_3 && slowdn_rate == 0 && accel == HIGH )
        {
          state->is_startup = 0;
          state->ocf = run_pwm_pulse_st_network;
          state->ps_hi_start_mcs = micros();
        }       
      }    
   
    case STATE::ST_SETPOINT_CONTROL:
      {      
        int speed_change = detect_speed_change( accel, slowdn, &state->scdd );        
        state->is_startup = ( state->is_startup || speed_change > 0 && state->ps_duration_mcs == 0 );
          
        state->ps_duration_mcs += speed_change;
        state->ps_duration_mcs = (state->ps_duration_mcs > state->period_mcs)? state->period_mcs : state->ps_duration_mcs;
        state->ps_duration_mcs = (state->ps_duration_mcs < 0)? 0 : state->ps_duration_mcs;
        
        if( state->is_startup && state->ocf == run_pwm_pulse_st_network )
        {        
          state->ocf = run_fan_startup_phase_1;        
        }
        
        state->state = STATE::ST_OUTPUT_CONTROL;                              
      }           

    case STATE::ST_OUTPUT_CONTROL:
      {               
        //st_ret = run_pwm_pulse_st_network( &state->ps_hi_start_mcs, state->period_mcs, state->ps_duration_mcs );
        st_ret = state->ocf( &state->ps_hi_start_mcs, state->period_mcs, state->ps_duration_mcs );
        state->state = STATE::ST_INPUT_CONTROL;
      }     
    break;        

    case STATE::ST_EMERG_STOP:
      ASSERT( 0 );
    break;
  }//switch 
  
  return st_ret;
}


struct Sound_params_t {
  enum STATE {
    ST_INIT,
    ST_PLAY,
    ST_FINAL
  };  
  
  int state;
  unsigned long time_buf;
  unsigned long pulse_time_buf;
};

int make_sound( int f, int duration_ms, struct Sound_params_t* sp )
{  
  typedef struct Sound_params_t SPT;
  typedef SPT STATE;
  
  int result_st = LOW;
  
  if( f >= 0, duration_ms > 0 && sp )
  {
    long pulse_width_mcs = (f > 0) ? 1000000/f : 0;
    long pulse_period_mcs = 2 * pulse_width_mcs;

    switch( sp->state )
    {
      case STATE::ST_FINAL:
        sp->state = STATE::ST_INIT;
        
      case STATE::ST_INIT:
        sp->time_buf = millis();
        result_st = HIGH;
        
        sp->state = STATE::ST_PLAY;
      break;
      
      case STATE::ST_PLAY:
        result_st = run_pwm_pulse_st_network( &(sp->pulse_time_buf), pulse_period_mcs, pulse_width_mcs );
        
        if( millis() -  sp->time_buf > duration_ms )
        {  
          sp->state = STATE::ST_FINAL;
        }
      break;     
    }
  }// defensive if
   
  return result_st;
}


struct Melody_network_data_t {
  enum STATE {
    ST_INIT,
    ST_TRG_WAIT,
    ST_PLAY,
    ST_FINAL
  };  
  
  int state;
  int* sounds;
  int* times;
  int n_vals;  
  int idx;  
  struct Sound_params_t sp;  
};

int run_melody_network( int st_trg, struct Melody_network_data_t* state )
{
  typedef struct Melody_network_data_t MNDT;
  typedef struct Sound_params_t SPT;
  typedef MNDT STATE;
  
  int st_ret = LOW;
  
  ASSERT( state );
  
  switch( state->state )
  {
    case STATE::ST_INIT:
    {
      // taat - taat - taam - tat - tat - tat - tat - taam
      static int fs[] =    { 262,   0, 292,   0, 330, 0,   349,   0, 392,   0, 440,   0, 494,   0, 262,   0 };
      static int times[] = { 600, 400, 600, 400, 800, 200, 300, 200, 300, 200, 300, 200, 300, 200, 800, 200 };
      state->n_vals = sizeof( fs ) / sizeof( fs[0] ); 
      state->sounds = fs;
      state->times = times;
      state->sp = { 0 };          
      
      state->state = STATE::ST_TRG_WAIT;
    }
    break;
    
    case STATE::ST_TRG_WAIT:      
      if( st_trg == LOW )
      {
        state->idx = 0;
        state->state = STATE::ST_PLAY;
      }    
    break;
    
    case STATE::ST_PLAY:
      st_ret = make_sound( state->sounds[state->idx], state->times[state->idx], &state->sp );
      
      if( SPT::ST_FINAL == state->sp.state )
      {
        ++state->idx;
      }
      
      if( state->idx >= state->n_vals )
      {
        state->state = STATE::ST_FINAL;
      }
    break;
    
    case STATE::ST_FINAL:
      state->state = STATE::ST_TRG_WAIT;
    break;
  }
    
  return st_ret;
}


int run_toggle_led_n_pulse_network( int n )
{  
  static int last_n;
  static int led_st = LOW;

  int result_st = led_st;
  
  if( n > 0 )
  {
    int n_pulses = n_pulses_of_fan_reed_switch;

    if( last_n != n_pulses && n_pulses % n == 0 )
    {
      result_st = (led_st == LOW)? HIGH : LOW;
    }      
    
    led_st = result_st;
    last_n = n_pulses;
    
  }// defensive if
  
  return result_st;
}


void isr_0( void )
{
  ++n_pulses_of_fan_reed_switch;
}


int control_servodrv( int st_right, int st_left )
{
  int st_ret = LOW;
  static unsigned long time_buf = millis();  
  
  if( millis() - time_buf > 250 )
  {    
    int n_ticks = n_servo_counter_pulse_ticks;
    
    if( st_right == LOW )
    {
      ++n_ticks;
    }
    
    if( st_left == LOW )
    {
      --n_ticks;
    }
    
    if( n_ticks > 78 )
    {
      n_ticks = 78;
    }
  
    if( n_ticks < 16 )
    {
      n_ticks = 16;
    }
    
    n_servo_counter_pulse_ticks = n_ticks;
    
    time_buf = millis();   
  }
  
  return st_ret;
}


ISR( TIMER2_COMPA_vect )
{
  static int n;
  
  if( n == 1204 )
  {
    digitalWrite( 12, HIGH );
    n = 0;
  }
  
  if( n == n_servo_counter_pulse_ticks )
  {
    digitalWrite( 12, LOW );
  }
  
  ++n;
}


ISR( TIMER1_COMPA_vect )
{
  static int n;  
  int st = digitalRead( 13 );
    
  if( n == 4 )
  {      
    st = (st == LOW)? HIGH : LOW;
    n = 0;
  }
  
  digitalWrite( 13, st );
  
  ++n;
}

void synchronization_barrier( int dest_diff_mcs )
{
  static unsigned long last_time_mcs = micros() - dest_diff_mcs / 2;
  
  int diff = micros() - last_time_mcs;
  int wait_time = dest_diff_mcs - diff;
  
  ASSERT( dest_diff_mcs > diff );
  
  delayMicroseconds( wait_time );
  
  last_time_mcs = micros();
}


// hw config
void setup() {
  Serial.begin(9600);
  
  // put your setup code here, to run once:
  pinMode( 9, OUTPUT );
  pinMode( 8, OUTPUT );
  pinMode( 10, OUTPUT );
  pinMode( 11, OUTPUT );
  pinMode( 12, OUTPUT );
  pinMode( 13, OUTPUT );

  pinMode( 5, INPUT_PULLUP );
  pinMode( 6, INPUT_PULLUP );
  pinMode( 2, INPUT_PULLUP );
  pinMode( 3, INPUT_PULLUP );
  pinMode( 4, INPUT_PULLUP );
  pinMode( 7, INPUT_PULLUP );
  
  attachInterrupt( 0, isr_0, RISING ); 
  setup_timer_1( 62500, T1_DIVIDER_CLK_64 );
  setup_timer_2( 1, T2_DIVIDER_CLK_1024 );
}


// entry
void loop() {
  // put your main code here, to run repeatedly:
  static struct Led_fade_out_network_data_t lfoutd;
  static struct Fan_control_data_t fcd;
  static struct Melody_network_data_t mnd;
  const int target_timing_mcs = 250;
  
  int st_0 = digitalRead( 5 );
  int st_1 = digitalRead( 6 );
  int st_2 = digitalRead( 2 );
  int st_3 = digitalRead( 3 );
  int st_4 = digitalRead( 4 );
  int st_7 = digitalRead( 7 );

  int st_9 = run_toggle_led_n_pulse_network( 10 ); //run_st_9_network( st_2 );
  int st_8 = run_led_fade_out_network( st_3, &lfoutd );
  int st_10 = control_fan( st_1, st_0, &fcd );
  int st_11 = run_melody_network( st_4, &mnd );
  int st_12 = control_servodrv( st_1, st_0 );  

  synchronization_barrier( target_timing_mcs );

  digitalWrite( 9, st_9 );
  digitalWrite( 8, st_8 );
  digitalWrite( 10, st_10 );
  digitalWrite( 11, st_11 );
  //digitalWrite( 12, st_12 );
   
  /*
  ASSERT( loop_dur < 200 );  
  ASSERT( loop_dur > 100 );
  */
}

