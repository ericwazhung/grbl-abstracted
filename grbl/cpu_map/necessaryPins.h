//THIS FILE DOES NOT DO ANYTHING
// It's merely for determining the pinout...
// THE ACTUAL definitions are in cpu_map_pic32...h



//I count 22 pins...
//However, in some configurations some of these are either not used or
//combined..
// e.g. FEED_HOLD and SAFETY_DOOR may be the same pin
// e.g. SPINDLE_PWM and SPINDLE_ENABLE may be the same pin
//Also, some pins may not be necessary, or even disableable
// e.g. If not using Z, could map those pins to unavailable bits...?
// e.g. COOLANT_FLOOD/COOLANT_MIST
// e.g. PROBE

//PIC32MX170F256B pins:
//Pin   (toupee2) grbl              PPS output-compares
//------------------------------------------------------------
//RA0   (Tx0)     Tx0               (OC1)
//RA1   (Heart)   X_LIMIT                 (OC2)
//RA2   (Rx0)     Rx0                                 (OC4/5)
//RA3   /WTF??/   Y_LIMIT                       (OC3)
//RA4             Z_LIMIT                             (OC4/5)
//------------------------------------------------------------
//RB0   /PGED1/   RESET                         (OC3)
//RB1   /PGEC1/   FEED_HOLD/SAFETY_DOOR   (OC2)
//RB2             CYCLE_START                         (OC4/5)
//RB3             PROBE             (OC1)
//RB4             X_STEP            (OC1)
//RB5             X_DIR                   (OC2)
//RB6             SPINDLE_PWM                         (OC4/5)
//RB7<- (jTDI)    /Z_STEP/          (OC1)....................
//RB8<- (jTCK)    /Z_DIR/           ......(OC2)..............
//RB9-> (jTDO)    /STEPPERS_DISABLE/............(OC3)........
//RB10            SPINDLE_ENABLE                (OC3)
//RB11<-(jTMS)                      ......(OC2)..............
//RB12            SPINDLE_DIRECTION .........................
//RB13            Y_STEP                              (OC4/5)
//RB14            Y_DIR                         (OC3)
//RB15            COOLANT_FLOOD     (OC1)

//JTAG: 33.2.3.1:
// a "soft" TAP reset ... using the TMS and TCK pins...
//    Apply HIGH to TMS for at least five rising edges of TCK
// (was contemplating whether it'd be possible to use the jtag pins for
//  other purposes while maintaining jtag functionality... probably not)

#define SERIAL_RX                i  d0    RA2   //1

#define SERIAL_TX                o  d1    RA0   //2

#define X_STEP_BIT               o  d2    RB4   //3  (OC1)
#define Y_STEP_BIT               o  d3    RB13  //4  (OC4)
#define Z_STEP_BIT               o  d4    RB7   //5  (UNUSED, JTAG)

#define X_DIRECTION_BIT          o  d5    RB5   //6  (OC2)
#define Y_DIRECTION_BIT          o  d6    RB14  //7  (OC3)
#define Z_DIRECTION_BIT          o  d7    RB8   //8  (UNUSED, JTAG)

#define STEPPERS_DISABLE_BIT     o  b0    RB9   //9  (UNUSED, JTAG)

//Not with other inputs... (was with STEPPERS_DISABLE)
#define X_LIMIT_BIT              i  b1    RA1   //10
#define Y_LIMIT_BIT              i  b2    RA3   //11
#define Z_LIMIT_BIT	            i  b3/4  RA4   
   //if VARIABLE_SPINDLE   (TRUE)   -> b4*      //12
   //else                           -> b3x


#define SPINDLE_ENABLE_BIT       o  b5/3/4   RB10
   //if VARIABLE_SPINDLE (TRUE)
      //if USE_SPINDLE_DIR_AS_ENABLE_PIN  (FALSE)  -> b5x
      //else                                       -> b3*   //13
   //else                                          -> b4x


//if NOT USE_SPINDLE_DIR_AS_ENABLE_PIN !(FALSE)==TRUE
#define SPINDLE_DIRECTION_BIT    o  b5    RB12     //14

#define COOLANT_FLOOD_BIT        o  c3    RB15     //15

//if ENABLE_M7 (FALSE)
#define COOLANT_MIST_BIT         o  c4x

//Not with other inputs
#define RESET_BIT                i  c0    RB0      //16
#define FEED_HOLD_BIT            i  c1*   RB1      //17
#define CYCLE_START_BIT          i  c2    RB2      //18
#define SAFETY_DOOR_BIT          i  c1*   RB1      //(17)

#define PROBE_BIT                i  c5    RB3      //19

//if VARIABLE_SPINDLE   (OC2A=PB3)  (TRUE)
#define SPINDLE_PWM_BIT	         o  b3    RB6     //(13) (OC5)
#error "SPINDLE_PWM and SPINDLE_ENABLE on same pin???"
