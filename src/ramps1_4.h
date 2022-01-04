#ifndef ramps1_4

#define ramps1_4

// For RAMPS 1.4
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_LIM_MAX          3
#define X_LIM_MIN          2

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_LIM_MAX          0
#define Y_LIM_MIN          0

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_LIM_MAX          0
#define Z_LIM_MIN          0

#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24
#define E0_LIM_MAX         0
#define E0_LIM_MIN         0

#define E1_STEP_PIN        36
#define E1_DIR_PIN         34
#define E1_ENABLE_PIN      30 
#define E1_LIM_MAX         0
#define E1_LIM_MIN         0

#define RAMPS_MAX_MOTORS    5

#define RAMPS_NOP __asm__ __volatile__ ("nop\n\t")

const byte RAMPS_PIN_S[]  = {X_STEP_PIN,Y_STEP_PIN,Z_STEP_PIN,E0_STEP_PIN,E1_STEP_PIN};
const byte RAMPS_PIN_D[]  = {X_DIR_PIN,Y_DIR_PIN,Z_DIR_PIN,E0_DIR_PIN,E1_DIR_PIN};
const byte RAMPS_PIN_E[]  = {X_ENABLE_PIN,Y_ENABLE_PIN,Z_ENABLE_PIN,E0_ENABLE_PIN,E1_ENABLE_PIN};
const byte RAMPS_PIN_MAX[]  = {X_LIM_MAX,Y_LIM_MAX,Z_LIM_MAX,E0_LIM_MAX,E1_LIM_MAX};
const byte RAMPS_PIN_MIN[]  = {X_LIM_MIN,Y_LIM_MIN,Z_LIM_MIN,E0_LIM_MIN,E1_LIM_MIN};
volatile long RAMPS_Abs[] = {0,0,0,0,0};


#define RAMPS_Step(M) { digitalWrite(RAMPS_PIN_S[M],HIGH); RAMPS_Abs[M]+=RAMPS_GetDir(M)?1:-1;  digitalWrite(RAMPS_PIN_S[M],LOW); }
#define RAMPS_SetDir(M,D) { digitalWrite(RAMPS_PIN_D[M],D); }
#define RAMPS_GetDir(M) digitalRead(RAMPS_PIN_D[M])
#define RAMPS_IsMax(M) digitalRead(RAMPS_PIN_MAX[M])
//(RAMPS_PIN_MAX[M]?digitalRead(RAMPS_PIN_MAX[M]):0) 
#define RAMPS_IsMin(M) digitalRead(RAMPS_PIN_MIN[M])
//(RAMPS_PIN_MIN[M]?digitalRead(RAMPS_PIN_MIN[M]):0) 
#define RAMPS_IsEnabledMotor(M) (!digitalRead(RAMPS_PIN_E[M]))
#define RAMPS_EnableMotor(M) { digitalWrite(RAMPS_PIN_E[M],LOW); }
#define RAMPS_DisableMotor(M) { digitalWrite(RAMPS_PIN_E[M],HIGH); }

void RAMPS_Init() {
 for(int i=0;i<RAMPS_MAX_MOTORS;i++) {
    digitalWrite(RAMPS_PIN_S[i],LOW);
    digitalWrite(RAMPS_PIN_D[i],LOW);
    digitalWrite(RAMPS_PIN_E[i],HIGH);
    pinMode(RAMPS_PIN_S[i],OUTPUT);
    pinMode(RAMPS_PIN_D[i],OUTPUT);
    pinMode(RAMPS_PIN_E[i],OUTPUT);
    pinMode(RAMPS_PIN_MAX[i],INPUT);
    Serial.println(RAMPS_PIN_MAX[i]);
    pinMode(RAMPS_PIN_MIN[i],INPUT);
    RAMPS_Abs[i]=0;
  }
}

#endif
