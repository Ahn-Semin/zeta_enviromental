
#define __DEBUG
#define _DEBUG

// Define commands
#define SEND_USDATA           0x20
#define SEND_USDATA_ONCE      0x21
#define SEND_USDATA_CONT      0x22
#define STOP_USDATA_CONT      0x23

// Define Sonar Sensor I/O
#define FRONT_EN              A1
#define FRONT_AN              A0
#define LEFT_EN               A2
#define LEFT_PW               3
#define RIGHT_EN              A3
#define RIGHT_PW              5
#define REARLEFT_EN           A4
#define REARLEFT_PW           6
#define REARLCENTER_EN        A5
#define REARLCENTER_PW        9
#define REARRCENTER_EN        A6
#define REARRCENTER_PW        10
#define REARRIGHT_EN          A7
#define REARRIGHT_PW          11

#define LED_PORT              13

#define USNUM                 7  
#define USFILTERNUM           5

#define ONOFF_DELAY           5
#define READ_DELAY            1

#define FR                    0
#define LT                    1
#define RT                    2
#define RL                    3
#define RLC                   4
#define RRC                   5
#define RR                    6


union INTVAL {
  char cval[4];
  int ival;
} int_val;

union UINTVAL {
  char cval[4];
  uint32_t uival;
} uint_val;

union SHORTVAL {
  char cval[2];
  short sval;
} short_val;

union WNTVAL {
  char cval[2];
  uint16_t wval;
} word_val;

union FLOATVAL {
  char cval[4];
  float fval;
} float_val;

byte startcnt = 0;
byte b_StartFlag = 0;
byte rsize = 0;

char RData[50]={0};
char WData[65] = {0};


unsigned long sortData[USNUM][USFILTERNUM] = {0};
unsigned long USRawData[USNUM][USFILTERNUM] = {0};
float USRawDSum = 0;
float USFilteredData[USNUM];

uint16_t USLoopFreq = 5;    // Hz
uint32_t USStartTime;
uint32_t USEndTime;
uint32_t USLoopTime;
uint32_t _max_time = 0;

bool b_USSendCont = false;

float distance[7] = {0};
int adval;
int cnt = 0;

void setup() {

  uint32_t i;
  uint32_t pre_time;

  Serial.begin(115200);

  //delay(500);
  
  //Serial.println("Connect to pc...");

  b_StartFlag = 0;
  startcnt = 0;
  rsize = 0;
  
  USLoopTime = 1000000/USLoopFreq;

  pinMode(LED_PORT, OUTPUT);

}


#if 1
void serialEvent()
{
  if(!Serial.available())
    return;
    
  byte buffer = Serial.read();

  if(buffer==0xAA && b_StartFlag==0) {
    startcnt++;
    
    if(startcnt==1 && b_StartFlag==0) {
       rsize = 0;
    }

    if(startcnt>=3) {
      startcnt = 0;
      b_StartFlag =  1;
      RData[rsize++] = buffer;  
     
      if(rsize>3) {
        rsize = 0;
        RData[0] = 0xAA;
        RData[1] = 0xAA;
        RData[2] = 0xAA;
        rsize = 3;
      }
    } else {
      RData[rsize++] = buffer;  
    }
  } else {
    RData[rsize++] = buffer;
  }
  
  if(b_StartFlag==1 && RData[rsize-1]==0x3B) {
    rsize = 0;
    b_StartFlag = 0;
          
    switch(RData[3]) {
      case SEND_USDATA_ONCE:

        ReadUSSensor();

        SendUSData();
        
        break;        

      case SEND_USDATA_CONT:
        word_val.cval[0] = RData[4];
        word_val.cval[1] = RData[5];

        USLoopFreq = word_val.wval;
        USLoopTime = 1000000/USLoopFreq;
        
        b_USSendCont = true;

        USStartTime = micros();
        
        break;       

      case STOP_USDATA_CONT:
        if(b_USSendCont == true)
          b_USSendCont = false;
        
        break; 
    }    
  } 
   
}

#endif

void InitSonar(void)
{
  pinMode(FRONT_EN, OUTPUT);
  pinMode(LEFT_EN, OUTPUT);
  pinMode(RIGHT_EN, OUTPUT);
  pinMode(REARLEFT_EN, OUTPUT);
  pinMode(REARLCENTER_EN, OUTPUT);
  pinMode(REARRCENTER_EN, OUTPUT);
  pinMode(REARRIGHT_EN, OUTPUT);

#if 0
  // Inactive all sensors
  digitalWrite(FRONT_EN, LOW);
  digitalWrite(LEFT_EN, LOW);
  digitalWrite(RIGHT_EN, LOW);
  digitalWrite(REARLEFT_EN, LOW);
  digitalWrite(REARLCENTER_EN, LOW);
  digitalWrite(REARRCENTER_EN, LOW);
  digitalWrite(REARRIGHT_EN, LOW);
#endif

#if 1
  // Active all sensors
  digitalWrite(FRONT_EN, HIGH);
  digitalWrite(LEFT_EN, HIGH);
  digitalWrite(RIGHT_EN, HIGH);
  digitalWrite(REARLEFT_EN, HIGH);
  digitalWrite(REARLCENTER_EN, HIGH);
  digitalWrite(REARRCENTER_EN, HIGH);
  digitalWrite(REARRIGHT_EN, HIGH);
 #endif

 #if 1
  pinMode(FRONT_AN, INPUT);
  pinMode(LEFT_PW, INPUT);
  pinMode(RIGHT_PW, INPUT);
  pinMode(REARLEFT_PW, INPUT);
  pinMode(REARLCENTER_PW, INPUT);
  pinMode(REARRCENTER_PW, INPUT);
  pinMode(REARRIGHT_PW, INPUT);
#endif  
  
}

void SendUSData(void)
{
  for(int i=0; i<65; i++)
    WData[i] = 0;
  
  WData[0] = 0xAA;
  WData[1] = 0xAA;
  WData[2] = 0xAA;  
  WData[3] = SEND_USDATA;
  
  float_val.fval = distance[0];
  WData[4] = float_val.cval[0];
  WData[5] = float_val.cval[1];
  WData[6] = float_val.cval[2];
  WData[7] = float_val.cval[3];   

  float_val.fval = distance[1];
  WData[8] = float_val.cval[0];
  WData[9] = float_val.cval[1];
  WData[10] = float_val.cval[2];
  WData[11] = float_val.cval[3];   

  float_val.fval = distance[2];
  WData[12] = float_val.cval[0];
  WData[13] = float_val.cval[1];
  WData[14] = float_val.cval[2];
  WData[15] = float_val.cval[3];   

  float_val.fval = distance[3];
  WData[16] = float_val.cval[0];
  WData[17] = float_val.cval[1];
  WData[18] = float_val.cval[2];
  WData[19] = float_val.cval[3];   

  float_val.fval = distance[4];
  WData[20] = float_val.cval[0];
  WData[21] = float_val.cval[1];
  WData[22] = float_val.cval[2];
  WData[23] = float_val.cval[3];   

  float_val.fval = distance[5];
  WData[24] = float_val.cval[0];
  WData[25] = float_val.cval[1];
  WData[26] = float_val.cval[2];
  WData[27] = float_val.cval[3];   

  float_val.fval = distance[6];
  WData[28] = float_val.cval[0];
  WData[29] = float_val.cval[1];
  WData[30] = float_val.cval[2];
  WData[31] = float_val.cval[3];   

  WData[32] = 0x3B;

   for(int i=0; i<33; i++)
    Serial.write(WData[i]); 
}

#if 1
void ReadUSSensor()
{ 
  int i,j;
  
  /*digitalWrite(FRONT_EN, HIGH);
  digitalWrite(LEFT_EN, HIGH);
  digitalWrite(REARLCENTER_EN, HIGH);
  digitalWrite(REARRIGHT_EN, HIGH);

  delay(ONOFF_DELAY);*/
  
  for(i=0; i<USFILTERNUM; i++) {
    USRawData[FR][i] = (unsigned long)analogRead(FRONT_AN);
    USRawData[LT][i] = pulseIn(LEFT_PW, HIGH);
    USRawData[RLC][i] = pulseIn(REARLCENTER_PW, HIGH);
    USRawData[RR][i] = pulseIn(REARRIGHT_PW, HIGH);

    USRawData[RT][cnt] = pulseIn(RIGHT_PW, HIGH);
    USRawData[RRC][cnt] = pulseIn(REARRCENTER_PW, HIGH);
    USRawData[RL][cnt] = pulseIn(REARLEFT_PW, HIGH);

    delay(READ_DELAY);
  }

  /*digitalWrite(FRONT_EN, LOW);
  digitalWrite(LEFT_EN, LOW);
  digitalWrite(REARLCENTER_EN, LOW);
  digitalWrite(REARRIGHT_EN, LOW);
  
  digitalWrite(RIGHT_EN, HIGH);
  digitalWrite(REARLEFT_EN, HIGH);
  digitalWrite(REARRCENTER_EN, HIGH);

  delay(ONOFF_DELAY);
  
  for(i=0; i<USFILTERNUM; i++) {
    USRawData[RT][cnt] = pulseIn(RIGHT_PW, HIGH);
    USRawData[RRC][cnt] = pulseIn(REARRCENTER_PW, HIGH);
    USRawData[RL][cnt] = pulseIn(REARLEFT_PW, HIGH);

    delay(READ_DELAY);
  }*/
  
  USFilter2(); 
}
#endif

#if 0
void ReadUSSensor()
{ 
  int i,j;

  if(cnt>USFILTERNUM-1) {
    //Serial.print("cnt1:");
    //Serial.print(cnt);
    cnt = USFILTERNUM-1;
    //Serial.print(" cnt2:");
    //Serial.print(cnt);

    //Serial.print("USRawData all:");

    for(i=0; i<USNUM; i++) {
      for(j=0; j<(USFILTERNUM-1); j++) {
        USRawData[i][j] = USRawData[i][j+1];
        //Serial.print(" " );
        //Serial.print(USRawData[i][j]);
      }
      //Serial.println(" ");
    }
  }
  
  //digitalWrite(FRONT_EN, HIGH);
  /*digitalWrite(LEFT_EN, HIGH);
  digitalWrite(REARLCENTER_EN, HIGH);
  digitalWrite(REARRIGHT_EN, HIGH);*/

  //delay(5);
    
  USRawData[FR][cnt] = (unsigned long)analogRead(FRONT_AN);

  //Serial.print(" fr raw:");
  //Serial.print(USRawData[FR][cnt]);
    
  /*USRawData[LT][cnt] = pulseIn(LEFT_PW, HIGH);
  USRawData[RLC][cnt] = pulseIn(REARLCENTER_PW, HIGH);
  USRawData[RR][cnt] = pulseIn(REARRIGHT_PW, HIGH);*/

  //digitalWrite(FRONT_EN, LOW);
  /*digitalWrite(LEFT_EN, LOW);
  digitalWrite(REARLCENTER_EN, LOW);
  digitalWrite(REARRIGHT_EN, LOW);*/
  
  /*digitalWrite(RIGHT_EN, HIGH);
  digitalWrite(REARLEFT_EN, HIGH);
  digitalWrite(REARRCENTER_EN, HIGH);

   delay(5);

  USRawData[RT][cnt] = pulseIn(RIGHT_PW, HIGH);
  USRawData[RRC][cnt] = pulseIn(REARRCENTER_PW, HIGH);
  USRawData[RL][cnt] = pulseIn(REARLEFT_PW, HIGH);*/

  cnt++;

  /*digitalWrite(RIGHT_EN, LOW);
  digitalWrite(REARLEFT_EN, LOW);
  digitalWrite(REARRCENTER_EN, LOW);*/

  USFilter2();
  //USFilter1();
  
#if 0

  //FrontAD.getRange();
  distance[FR] = FrontAD.getRange();
  //distance[FR] = FrontAD.getSampleBest();

  //RSLeft.getRange();
  distance[LT] = RSLeft.getRange();
  //distance[LT] = RSLeft.getSampleBest();

  //RSRearRight.getRange();
  distance[RR] = RSRearRight.getRange();
  //distance[RR] = RSRearRight.getSampleBest();

  //RSRearLCenter.getRange();
  distance[RLC] = RSRearLCenter.getRange();
  //distance[RLC] = RSRearLCenter.getSampleBest();
#endif


#if 0  
  //RSRight.getRange();
  distance[RT] = RSRight.getRange();
  //distance[RT] = RSRight.getSampleBest();

  //RSRearLeft.getRange();
  distance[RL] = RSRearLeft.getRange();
  //distance[RL] = RSRearLeft.getSampleBest();

  //RSRearRCenter.getRange();
  distance[RRC] = RSRearRCenter.getRange();
  //distance[RRC] = RSRearRCenter.getSampleBest();
#endif
  
}
#endif

void USFilter1()
{
  int i,j;

  //USRawData[snum][cnt] = duration[snum];

  USRawDSum = 0;

  for(i=0; i<USNUM; i++) {
    USRawDSum = 0;
    for(j=0; j<USFILTERNUM; j++)
      USRawDSum += USRawData[i][j];

    USFilteredData[i] = USRawDSum/USFILTERNUM;

    /*Serial.print("USRawDSum:");
    Serial.print(USRawDSum);
    Serial.print(" USFilteredData[0]:");
    Serial.print(USFilteredData[0]);*/
    
  }
  
  distance[0] = USFilteredData[0]*2.54*5.0/10.0;
  for(i=1; i<USNUM; i++)  
    distance[i] = USFilteredData[i]*2.54/147.0;

}

void USFilter2()
{
  int i,j;

  sortUSRawData();

  //for(i=0; i<USNUM; i++)
  //  distance[i] = sortData[i][USFILTERNUM-1];
   
  distance[0] = sortData[0][USFILTERNUM-1]*2.54*5.0/10.0;
  for(i=1; i<USNUM; i++)  
    distance[i] = sortData[i][USFILTERNUM-1]*2.54/147.0;

}

void sortUSRawData()
{
  for (int i = 0; i < USNUM; i++)
    for (int j = 0; j < USFILTERNUM; j++)
      sortData[i][j] = USRawData[i][j];

    /*Serial.print(sortData[0][0]);
    Serial.print(",");
    Serial.print(sortData[0][1]);
    Serial.print(",");
    Serial.print(sortData[0][2]);
    Serial.print(",");
    Serial.print(sortData[0][3]);
    Serial.print(",");
    Serial.print(sortData[0][4]);
    Serial.print(",");
    Serial.println(" ");*/
      
  for (int i = 0; i < USNUM; i++) {
    for (int j = 1; j < USFILTERNUM; j++) {
        unsigned long val = sortData[i][j];
        int k;

        for (k = j - 1; (k >= 0) && (val < sortData[i][k]); k--)
            sortData[i][k + 1] = sortData[i][k];

        sortData[i][k + 1] = val;
    }

    /*Serial.print(sortData[i][0]);
    Serial.print(",");
    Serial.print(sortData[i][1]);
    Serial.print(",");
    Serial.print(sortData[i][2]);
    Serial.print(",");
    Serial.print(sortData[i][3]);
    Serial.print(",");
    Serial.print(sortData[i][4]);
    Serial.print(",");
    Serial.print(sortData[i][5]);
    Serial.print(",");
    Serial.print(sortData[i][6]);
    Serial.print(",");
    Serial.print(sortData[i][7]);
    Serial.print(",");
    Serial.print(sortData[i][8]);
    Serial.println(" ");*/
  }

}

void loop() {

#if 0
  digitalWrite(LED_PORT, HIGH);

  //delay(1000);
  ReadUSSensor();


  Serial.print(distance[0]);
  Serial.print(",");
  Serial.print(distance[1]);
  Serial.print(",");
  Serial.print(distance[2]);
  Serial.print(",");
  Serial.print(distance[3]);
  Serial.print(",");
  Serial.print(distance[4]);
  Serial.print(",");
  Serial.print(distance[5]);
  Serial.print(",");
  Serial.print(distance[6]);
  Serial.println(" ");

  /*Serial.print("FR:");
  Serial.print(distance[0]);
  Serial.print(" LT:");
  Serial.print(distance[1]);
  Serial.print(" RT:");
  Serial.print(distance[2]);
  Serial.print(" RL:");
  Serial.print(distance[3]);
  Serial.print(" RLC:");
  Serial.print(distance[4]);
  Serial.print(" RRC:");
  Serial.print(distance[5]);
  Serial.print(" RR:");
  Serial.print(distance[6]);
  Serial.println(" ");*/

  digitalWrite(LED_PORT, LOW);
#endif  

#if 1
   if(b_USSendCont == true) {
    USEndTime = micros();
    if((USEndTime - USStartTime) > USLoopTime) {
      digitalWrite(LED_PORT, HIGH);
      
      ReadUSSensor();
      
      SendUSData();
      
      digitalWrite(LED_PORT, LOW);
      
      USStartTime = micros();
    }
  }
#endif
 
  
  delay(1);
  //delay(500);

}
