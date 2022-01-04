#include "ramps1_4.h"

// StepperOS - Interactive text based stepper controller OS for Arduino.
// Jason Bolbach - JBolbach@gmail.com
// Initial development 2019
//
// Compiles for Nano, Uno, Mega.  Change pin settings in ramps1_4.h.
// Send ASCII text commands at 115200 baud
// # = Motor Number.

// Error Response is prefixed with @:
// Non-Error responses are prefixed with $:
// Interrupt (out of band data) is prefixed with I:

// R#       Reset Absolute Position Counter
// E#       Enable Motor Driver
// D#       Disable Motor Driver

// M#,[+|-][STEPS]  Move Relative. Ex:  M0,500 = Move Motor 0 +500  M2,-3000= Move Motor 2 -3000
// A#,[+|-][STEPS]  Move Absolute. Ex:  A0,500 = Move Motor 0 To Position +500
// C#,[+|-][NUMBER] Move Continuous in +/- direction specified by any pos/neg number.

// ?P#        Query Absolute Position of Motor (use A for All)
// ?S#        Query State of Motor (use A for All)
// ?I#        Query Interrupt Conditions for Motor (use A for All)

// V#,[Min Velocity],[Max Velocity],[Steps]
//          Velocities can be 0 - 255 (0 being fastest, 255 being slowest)
//          Steps can be 0-15.  This is the number of steps used in the acceleration curve.

// I#       This is used to tie an interupt pin to a stop condition for a motor.
//          Go ahead and ask me if I remember what I was doing here, don't expect a good answer.
//          It worked once or twice, and for the life of me, I can't recall how to use it.
//          Someday when I've had enough coffee, I will work it out and document it.
//          It's some kind of a Motor vs Interrupt Pin matrix specifying stop on High/Low.

#define M_MAX_VELOCITIES    16
#define M_MODE_DIS          0 //Disabled - Driver Disabled
#define M_MODE_STP          1 //Stopped  - Driver Enabled, Not Moving
#define M_MODE_ACC          2 //Accelerating
#define M_MODE_DEC          3 //Decelerating
#define M_MODE_RUN          4 //Running At Max Speed
#define M_MODE_CNT          5 //Continuous At Max Speed
#define M_MODE_CNA          6 //Continuous (Acceleration Phase)

volatile long     M_ABS_Pos[]   = {0,0,0,0,0};  //Absolute Position
volatile byte     M_Cur_Vel[]   = {0,0,0,0,0};  //Current Velocity Index
volatile byte     M_Max_Vel[]   = {0,0,0,0,0};  //Max Velocity Index

volatile byte     M_Vel[RAMPS_MAX_MOTORS][M_MAX_VELOCITIES];   
volatile byte     M_AD_Steps[]  = {0,0,0,0,0};    
volatile long     M_Run_Steps[] = {0,0,0,0,0};   

volatile long     M_Steps[]     = {0,0,0,0,0};  //AD Level Steps Remaining
volatile byte     M_Mode[]      = {0,0,0,0,0};  //Motor Mode
volatile byte     M_Vel_Wait[]  = {0,0,0,0,0};  

const unsigned int Sin[] = {0,105,208,309,407,500,588,669,743,809,866,914,951,978,995,1000}; 
char Buffer[256];

void SetVelocity(byte Motor,byte MinVelocity,byte MaxVelocity,byte ADSteps) {
 M_AD_Steps[Motor]=ADSteps;
 long A = MinVelocity-MaxVelocity;
 long B = 0;
 for(int i=0;i<M_MAX_VELOCITIES;i++)
    {
     B=A;
     B*=Sin[i];
     B/=1000;
     M_Vel[Motor][i]=MinVelocity-B;
    }
}

void Reset(byte Motor) {
  M_Mode[Motor]=RAMPS_IsEnabledMotor(Motor)?M_MODE_STP:M_MODE_DIS;
  M_Cur_Vel[Motor]=0;
  M_Steps[Motor]=0;
  M_Run_Steps[Motor]=0;
  M_Vel_Wait[Motor]=0;
}

void MoveCon(byte Motor,int Dir,byte MaxSpeed) {
  Reset(Motor);
  if(MaxSpeed<1) MaxSpeed=1;
  if(MaxSpeed>=M_MAX_VELOCITIES) MaxSpeed=M_MAX_VELOCITIES-1;
  M_Max_Vel[Motor]=MaxSpeed;
  if(Dir<0) { RAMPS_SetDir(Motor,LOW);  }
  else if(Dir>0) { RAMPS_SetDir(Motor,HIGH); }
  else { MoveRel(Motor,0,0); return; }
  RAMPS_EnableMotor(Motor);
  M_Steps[Motor]=M_AD_Steps[Motor];
  M_Mode[Motor]=M_MODE_CNA;
}

void MoveAbs(byte Motor,long Position,byte MaxSpeed) {
  long Rel=Position;
  Rel-=RAMPS_Abs[Motor];
  MoveRel(Motor,Rel,MaxSpeed);
}

void MoveRel(byte Motor,long Steps,byte MaxSpeed) {
  Reset(Motor);
  if(MaxSpeed<1) MaxSpeed=1;
  if(MaxSpeed>=M_MAX_VELOCITIES) MaxSpeed=M_MAX_VELOCITIES-1;
  if(!Steps) return;
  if(Steps<0) { RAMPS_SetDir(Motor,LOW); Steps=-Steps; }
  else RAMPS_SetDir(Motor,HIGH);

  RAMPS_EnableMotor(Motor);
  long Compare=0;
  
  Compare=M_AD_Steps[Motor];
  Compare*=2;
    if(Steps<=Compare) {          //Move too short for Accel/Decel, Decel Only
    M_Max_Vel[Motor]=0;
    M_Steps[Motor]=Steps;
    M_Mode[Motor]=M_MODE_DEC;
    return;
  }
  
  Compare=M_AD_Steps[Motor];
  Compare*=MaxSpeed;
  Compare*=2;
  if(Steps<=Compare) { //Move too short to leave Accel/Decel range
     M_Max_Vel[Motor]=Steps/(M_AD_Steps[Motor]*2);
     M_Run_Steps[Motor]=Steps%(M_AD_Steps[Motor]*2);
     M_Steps[Motor]=M_AD_Steps[Motor];
     M_Mode[Motor]=M_MODE_ACC;
     return;
    }
    M_Max_Vel[Motor]=MaxSpeed;
    Compare=M_AD_Steps[Motor];
    Compare*=MaxSpeed;
    Compare*=2;
    M_Run_Steps[Motor]=Steps-Compare;
    M_Steps[Motor]=M_AD_Steps[Motor];
    M_Mode[Motor]=M_MODE_ACC;
}

char MotorName[][3] = { "X","Y","Z","E0","E1" };
char ModeName[][4] = { "DIS","STP","ACC","DEC","RUN","CNT","ACC" };

#define MAX_INPUT 64
char Input[MAX_INPUT];
byte Index=0;

void WipeInput() { for(byte i=0;i<MAX_INPUT;i++) Input[i]=0; Index=0; Buffer[0]=0; }

byte PC_INT_NUM[] = {2,3,18,19,20,21};
byte PC_INT_EN[] = {0,0,0,0,0,0};
byte PC_INT_HL[] = {0,0,0,0,0,0};

void Pin2Change() {
 Serial.println("I:0,MAX");
}

void Pin3Change() {
 Serial.println("I:0,MIN");
}

void Pin18Change() {
 //Serial.println("I8");
}

void Pin19Change() {
 //Serial.println("I19");
}

void Pin20Change() {
 //Serial.println("I20");
}

void Pin21Change() {
 //Serial.println("I21");
}

void ProcessInput() {
  bool Error=0;
  int M=0;
  long Params[5]={0,0,0,0,0};
  int NumParams=0;
  byte Temp=0;
  
  switch(Input[(Input[0]=='?')?2:1]) {
    case('0'): M=0; break; 
    case('1'): M=1; break; 
    case('2'): M=2; break; 
    case('3'): M=3; break; 
    case('4'): M=4; break;
    case('a'): M=5; break;
    case('A'): M=5; break;
    default:
      sprintf(Buffer,"@:Bad_Motor_ID:%c",Input[(Input[0]=='?')?2:1]);
      Serial.println(Buffer);
      return; 
    break;
  }
  if(Input[0]!='?') {
    int Sign=1;
    for(int i=2;i<MAX_INPUT && Input[2];i++) {
      if(Input[i]==' ') continue;
      if(Input[i]==0) {
        Params[NumParams++]*=Sign;
        Sign=1;
        break;
      }
      if(Input[i]==',') {
        Params[NumParams++]*=Sign;
        Sign=1;
      }
      if(Input[i]=='-') Sign=-1;
      if(Input[i]=='+') Sign=+1;
      if(Input[i]>='0' && Input[i]<='9') {
        Params[NumParams]*=10;
        Params[NumParams]+=Input[i]-'0';
      }
    }
  }

  switch(Input[0]) {
    case('?'):
      switch(Input[1]) {
        case('p'):
        case('P'):
          if(M<5) {
            sprintf(Buffer,"%ld",RAMPS_Abs[M]); 
          }else {
            sprintf(Buffer,"%ld,%ld,%ld,%ld,%ld",RAMPS_Abs[0],RAMPS_Abs[1],RAMPS_Abs[2],RAMPS_Abs[3],RAMPS_Abs[4]); 
          }
        break;

        case('s'):
        case('S'):
          if(M<5) {
            sprintf(Buffer,"%d:%s",M,ModeName[M_Mode[M]]); 
          }else {
            sprintf(Buffer,"%s,%s,%s,%s,%s",ModeName[M_Mode[0]],ModeName[M_Mode[1]],ModeName[M_Mode[2]],ModeName[M_Mode[3]],ModeName[M_Mode[4]]); 
          }
        break;

        case('i'):
        case('I'):
          if(M<5) {
            Temp=1<<M;
            sprintf(Buffer," , , , , , ");
            for(int i=0;i<6;i++) Buffer[i*2]=(PC_INT_EN[i]&Temp)?((PC_INT_HL[i]&Temp)?'H':'L'):'0';
          }else {
          }
        break;

        default:
          sprintf(Buffer,"Unknown_Query_Argument:%c",Input[1]); 
      }
    break;

    case('e'):
    case('E'):
      if(M<5) {
        RAMPS_EnableMotor(M);
        sprintf(Buffer,"Enable:%d",M); 
      }else {
        for(int i=0;i<RAMPS_MAX_MOTORS;i++) RAMPS_EnableMotor(i); 
        sprintf(Buffer,"Enable:All"); 
      }
    break;

    case('d'):
    case('D'):
      if(M<5) {
        RAMPS_DisableMotor(M);
        sprintf(Buffer,"Disable:%d",M); 
      }else {
        for(int i=0;i<RAMPS_MAX_MOTORS;i++) RAMPS_DisableMotor(i); 
        sprintf(Buffer,"Disable_All"); 
      }
    break;

    case('r'):
    case('R'):
      if(M<5) {
        RAMPS_Abs[M]=0;
        sprintf(Buffer,"Reset:%d",M); 
      }else {
        for(int i=0;i<RAMPS_MAX_MOTORS;i++) RAMPS_Abs[i]=0; 
        sprintf(Buffer,"Reset All"); 
      }
    break;

    case('m'):
    case('M'):
      MoveRel(M,Params[1],(NumParams>2)?Params[2]:M_MAX_VELOCITIES-1);
      sprintf(Buffer,"Move_Rel:%d:%ld:%ld",M,Params[1],(NumParams>2)?Params[2]:M_MAX_VELOCITIES-1);
    break;

    case('a'):
    case('A'):
      MoveAbs(M,Params[1],(NumParams>2)?Params[2]:M_MAX_VELOCITIES-1);
      sprintf(Buffer,"Move_Abs:%d:%ld:%ld",M,Params[1],(NumParams>2)?Params[2]:M_MAX_VELOCITIES-1);
    break;

    case('c'):
    case('C'):
      MoveCon(M,Params[1],(NumParams>2)?Params[2]:M_MAX_VELOCITIES-1);
      sprintf(Buffer,"Move_Con:%d:%c:%ld",M,Params[1]<0?'-':'+',(NumParams>2)?Params[2]:M_MAX_VELOCITIES-1);
    break;

    case('v'):
    case('V'):
      if(NumParams!=4) {
        Error=1;
        sprintf(Buffer,"Invalid_Number_Of_Velocity_Parameters:%d",NumParams);
      }else {
        SetVelocity(M,Params[1],Params[2],Params[3]);
        sprintf(Buffer,"Set_Vel:%d:%ld:%ld:%ld",M,Params[1],Params[2],Params[3]);
      }
    break;

    case('i'):
    case('I'):
      switch(Params[0]) {
        case(2):
          if(Params[1]) PC_INT_EN[0]|=(1<<M);
          else PC_INT_EN[0]&=(255-(1<<M));
          if(Params[2]) PC_INT_HL[0]|=(1<<M);
          else PC_INT_HL[0]&=(255-(1<<M));
          sprintf(Buffer,"%x:%x",PC_INT_EN[0],PC_INT_HL[0]);
        break;
        case(3):
        break;
        case(18):
        break;
        case(19):
        break;
        case(20):
        break;
        case(21):
        break;
          
        default:
          Error=1;
          sprintf(Buffer,"Invalid_Pin:%ld",Params[0]);
      }
    break;

    default:
      sprintf(Buffer,"Unknown Command %c",Input[0]); 
  }

  if(Error) Serial.print("@:"); 
  else Serial.print("$:");
  Serial.println(Buffer);
}

void setup() {
  Serial.begin(115200);
  RAMPS_Init();

  cli();
  for(int i=0;i<RAMPS_MAX_MOTORS;i++) {
    Reset(i);
    SetVelocity(i,150,20,5);
  }
 
  // Setup TIMER1 @ 20KHz
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  OCR1A = 799; // = 16000000 / (1 * 20000) - 1 (must be <65536)
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);

  attachInterrupt(digitalPinToInterrupt(2),  Pin2Change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3),  Pin3Change, CHANGE);
  /*
  attachInterrupt(digitalPinToInterrupt(18), Pin18Change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(19), Pin19Change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(20), Pin20Change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(21), Pin21Change, CHANGE); 
  */
  sei();
  Serial.println("$:StepperOS v1.1 (Jason Bolbach 2019)");
}

byte M_Mode_Prime[] = {0,0,0,0,0}; 

void loop() {
 for(int i=0;i<RAMPS_MAX_MOTORS;i++) {
  if(M_Mode[i]!=M_Mode_Prime[i]) {
    M_Mode_Prime[i]=M_Mode[i];
    sprintf(Buffer,"I:%d:%s",i,ModeName[M_Mode[i]]);
    Serial.println(Buffer);
  }
 }
 while(Serial.available()) {
  char i=Serial.read();
  if(i=='`') { WipeInput(); continue; }
  if(i=='~') { Oscillate(); }
  if(i==0x0d || i==0x0a) { if(Index) ProcessInput(); WipeInput(); continue; }
  Input[Index++]=i;
 }
}

void Oscillate()
{
 sprintf(Input,"v0,40,2,8");
 ProcessInput();
 while(1) {
  sprintf(Input,"a0,12137");
  ProcessInput();
  while(M_Mode[0]!=M_MODE_STP);
  delay(150);
  
  sprintf(Input,"a0,6068");
  ProcessInput();
  while(M_Mode[0]!=M_MODE_STP);
  delay(150);

  sprintf(Input,"a0,18205");
  ProcessInput();
  while(M_Mode[0]!=M_MODE_STP);
  delay(150);

  sprintf(Input,"a0,12137");
  ProcessInput();
  while(M_Mode[0]!=M_MODE_STP);
  delay(150);

  sprintf(Input,"a0,24275");
  ProcessInput();
  while(M_Mode[0]!=M_MODE_STP);
  delay(150);

  sprintf(Input,"a0,0");
  ProcessInput();
  while(M_Mode[0]!=M_MODE_STP);
  delay(150);
 }
}

ISR(TIMER1_COMPA_vect) {
  cli();
  byte IsMin,IsMax;
  for(byte M=0;M<RAMPS_MAX_MOTORS;M++)
     {
      if(M_Mode[M]>1) { 
      if(M_Vel_Wait[M]<=0) 
        {
         IsMin=RAMPS_IsMin(M);
         IsMax=RAMPS_IsMax(M);
         if(IsMax|IsMin) {
          if(!RAMPS_GetDir(M) && IsMax) {  M_Mode[M]=M_MODE_STP; continue; }
          if(RAMPS_GetDir(M) && IsMin) {  M_Mode[M]=M_MODE_STP; continue; }
         }
         RAMPS_Step(M);
         if(--M_Steps[M]<=0)
           {
            if(M_Mode[M]==M_MODE_ACC)
              {
               if(M_Cur_Vel[M]+1==M_Max_Vel[M])
                 {
                  M_Cur_Vel[M]++;   
                  if(!M_Run_Steps[M])
                    {
                     M_Cur_Vel[M]--;
                     M_Steps[M]=M_AD_Steps[M];
                     M_Mode[M]=M_MODE_DEC;
                    }
                  else
                    {
                     M_Steps[M]=M_Run_Steps[M];
                     M_Mode[M]=M_MODE_RUN;
                    }
                 }
               else 
                 {
                  M_Cur_Vel[M]++;
                  M_Steps[M]=M_AD_Steps[M];
                 }
              }
            else if(M_Mode[M]==M_MODE_CNA)
              {
               if(M_Cur_Vel[M]+1==M_Max_Vel[M])
                 {
                  M_Cur_Vel[M]++;   
                  M_Mode[M]=M_MODE_CNT;
                 }
               else 
                 {
                  M_Cur_Vel[M]++;
                  M_Steps[M]=M_AD_Steps[M];
                 }
              }
            else if(M_Mode[M]==M_MODE_DEC)
              {
               if(!M_Cur_Vel[M])
                 {
                  M_Mode[M]=M_MODE_STP;
                 }
               else
                 {
                  M_Cur_Vel[M]--;
                  M_Steps[M]=M_AD_Steps[M];
                 }
              }
            else if(M_Mode[M]==M_MODE_RUN)
              {
               M_Cur_Vel[M]--;
               M_Steps[M]=M_AD_Steps[M];
               M_Mode[M]=M_MODE_DEC;
              }
            else if(M_Mode[M]==M_MODE_CNT)
              {
              }
           }
       M_Vel_Wait[M]=M_Vel[M][M_Cur_Vel[M]];
      }
    else {M_Vel_Wait[M]--;}
  }}
  sei();
}
