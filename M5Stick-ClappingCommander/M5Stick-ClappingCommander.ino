#include <M5StickCPlus.h>
#include <driver/i2s.h>

// --- I2S MIC address and command
#define PIN_CLK  0
#define PIN_DATA 34
#define SAMPLE_RATE 44100
#define DATA_POINTS 128
#define READ_LEN (2 * DATA_POINTS)
uint8_t BUFFER[READ_LEN] = {0};
int16_t *adcBuffer = NULL;
float totalPower = 0;

// --- DISO COLOR LED LAMP address and command
#define BUTTONA 37 // M BUTTON
#define BUTTONB 39 // Side BUTTON
#define DATAPIN 9   // M5StackC (IR PIN)
#define HBYTE(x) ((x >> 8) & 0xFF)
#define LBYTE(x) (x & 0xFF)

uint16_t IRAddr = 0x807F ;
uint8_t IRCmdPowerON  = 0x12 ;
uint8_t IRCmdPowerOFF = 0x1A ;
uint8_t IRCmdMode = 0x1E ;
uint8_t IRCmdColor = 0x03 ;
int IRCmdCOLORNum = 9 ;
uint8_t IRCmdCOLOR[] = {0x04,0x05,0x06 ,
                        0x07,0x08,0x09 ,
                        0x0A,0x1B,0x1F ,
                        0x0C,0x0D,0x0E } ;

// ▲▼▲▼ MIC　▲▼▲▼
void i2sInit()
{
   i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    .sample_rate =  SAMPLE_RATE,                  // sampling frequency
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
    .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,  // monaural right channel
    .communication_format = I2S_COMM_FORMAT_I2S,  // I2S Format
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
    .dma_buf_count = 2,                           // number of buffers
    .dma_buf_len = 128,                           // sampling buffer length
   };

   i2s_pin_config_t pin_config;
   pin_config.bck_io_num   = I2S_PIN_NO_CHANGE;
   pin_config.ws_io_num    = PIN_CLK;
   pin_config.data_out_num = I2S_PIN_NO_CHANGE;
   pin_config.data_in_num  = PIN_DATA;
   
   i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
   i2s_set_pin(I2S_NUM_0, &pin_config);
   i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}

void mic_record_task (void* arg)
{   
  size_t bytesread;
  while(1){
    i2s_read(I2S_NUM_0,(char*) BUFFER, READ_LEN, &bytesread, (100 / portTICK_RATE_MS));
    adcBuffer = (int16_t *)BUFFER;
    for (int n = 0; n < DATA_POINTS; n++){
      totalPower += adcBuffer[n];
    }
    totalPower /= DATA_POINTS;
  }
}

// ▲▼▲▼ IR COMMANDS
// Send command in NEC format
void SendIRCommand(uint16_t Addr,uint8_t cmd) {
  int i ,n ,m ;
  uint8_t data[4] = { HBYTE(Addr) , LBYTE(Addr) , cmd , ~cmd } ;

  // 1ms = 38 Loops : T=(562μS)21.356 Loops
  for (i=0;i<341;i++) { // Send 16T(8992μS) - HIGH
      digitalWrite(DATAPIN,HIGH) ;
      delayMicroseconds(13);
      digitalWrite(DATAPIN,LOW) ;
      delayMicroseconds(12);
  }
  for (i=0;i<170;i++) { // Send 8T(4496μS) - LOW
      digitalWrite(DATAPIN,LOW) ;
      delayMicroseconds(13);
      digitalWrite(DATAPIN,LOW) ;
      delayMicroseconds(12);
  }
  // SWND : ADDRESS + DATA + ~DATA
  for (n=0;n<8;n++) {
    uint8_t mask = 0x01 ;
    for (m=0;m<8;m++) {
      int t = ((data[n] & mask) == 0) ? 21 : 64 ;
      for (i=0;i<21;i++) { // Send T - HIGH
        digitalWrite(DATAPIN,HIGH) ;
        delayMicroseconds(13);
        digitalWrite(DATAPIN,LOW) ;
        delayMicroseconds(12);
      }
      for (i=0;i<t;i++) { // Send t - LOW
        digitalWrite(DATAPIN,LOW) ;
        delayMicroseconds(13);
        digitalWrite(DATAPIN,LOW) ;
        delayMicroseconds(12);
      }
      mask <<=1 ;
    }
  }
  // end mark. probably not necessary
  for (i=0;i<21;i++) { // Send T - HIGH
    digitalWrite(DATAPIN,HIGH) ;
    delayMicroseconds(13);
    digitalWrite(DATAPIN,LOW) ;
    delayMicroseconds(12);
  }
}

// ▲▼▲▼ Control
bool readStat ;
unsigned long startTime ;
unsigned long checkTime ;
unsigned long passtTime ;
int clapCommandNum ;
unsigned long clapCommand[4] ;

int cnt = 0 ;
int color = 0 ;
bool switchStatus = false ; 

void setup() {
  M5.begin();
  Serial.begin(9600) ;

  i2sInit();
  xTaskCreate(mic_record_task, "mic_record_task", 2048, NULL, 1, NULL);

  pinMode(10, OUTPUT);
  pinMode(26, OUTPUT);
  digitalWrite(10,HIGH) ;
  digitalWrite(26,HIGH) ;
  startTime = millis();
  clapCommandNum = 0 ;
  readStat = true ;

  pinMode(BUTTONA,INPUT) ;
  pinMode(BUTTONB,INPUT) ;
  pinMode(DATAPIN, OUTPUT);
  digitalWrite(DATAPIN,LOW) ;
  
  M5.Axp.ScreenBreath(8);
  M5.Lcd.setRotation(1);
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setTextFont(4);
  M5.Lcd.setTextSize(1); //Set the character size (1 (minimum) to 7 (maximum))
  M5.Lcd.setTextColor(BLACK);
  String str1 = "CLAPING" ; 
  String str2 = "COMMANDER" ; 
  M5.Lcd.drawString(str1, 25, 10);
  M5.Lcd.drawString(str2, 60, 40);
  M5.Lcd.setTextColor(ORANGE);
  M5.Lcd.drawString(str1, 23, 8);
  M5.Lcd.drawString(str2, 58, 38);

  M5.Lcd.drawRect(  49, 84, 32, 32, BLACK);
  M5.Lcd.drawRect( 109, 84, 32, 32, BLACK);
  M5.Lcd.drawRect( 169, 84, 32, 32, BLACK);
  M5.Lcd.fillRect(  50, 85, 30, 30, GREEN);
  M5.Lcd.fillRect( 110, 85, 30, 30, GREEN);
  M5.Lcd.fillRect( 170, 85, 30, 30, GREEN);
}

void loop() {
  digitalWrite(26,(readStat?HIGH:LOW)) ;
  int p = totalPower ;
  p = abs(p - 1500) ;
  if (p > 150) {
    passtTime = millis();
    Serial.print("Level=") ;
    Serial.print(p) ;
    Serial.print("  readStat=") ;
    Serial.print(readStat) ;
    Serial.print("  clapCommandNum=") ;
    Serial.println(clapCommandNum) ;

    if (readStat) {
      readStat = false ;
      startTime = passtTime ;
      checkTime = passtTime ;
      M5.Lcd.fillRect(  50, 85, 30, 30, BLUE);
    } else {
      clapCommand[clapCommandNum] = passtTime - checkTime ;
      checkTime = passtTime ;
      clapCommandNum ++ ;
      switch(clapCommandNum) {
        case 1: M5.Lcd.fillRect( 110, 85, 30, 30, BLUE); break ;
        case 2: M5.Lcd.fillRect( 170, 85, 30, 30, BLUE); break ;
      }

      if (clapCommandNum >= 2) {
        // Command parsing
        Serial.print("COMMAND=") ;
        Serial.print(clapCommand[0]) ;
        Serial.print(" / ") ;
        Serial.println(clapCommand[1]) ;

        int cmdNo = 0;
        if (350 <= clapCommand[0] && clapCommand[0] <= 500) {
          if (clapCommand[1] <= clapCommand[0] * 1.3 && clapCommand[0] * 0.7 <= clapCommand[0]) {
            cmdNo = 1 ;
          } 
        }
        if (450 <= clapCommand[0] && clapCommand[0] <= 600) {
          if (clapCommand[0] / 3 <= clapCommand[1] && clapCommand[1] <= clapCommand[0] / 2 ) {
            cmdNo = 2 ;
          }
          Serial.print("COMMAND-2:") ;
          Serial.print(clapCommand[0] / 3) ;
          Serial.print("-") ;
          Serial.println(clapCommand[0] / 2) ;
        } else
        if (140 <= clapCommand[0] && clapCommand[0] <= 270) {
          if (clapCommand[0] * 2 <= clapCommand[1] && clapCommand[1] <= clapCommand[0] * 3) {
            cmdNo = 3 ;
          }
          Serial.print("COMMAND-3:") ;
          Serial.print(clapCommand[0] * 3) ;
          Serial.print("-") ;
          Serial.println(clapCommand[0] * 4) ;
        }
        Serial.print("COMMAND-") ;
        Serial.println(cmdNo) ;
        if (cmdNo != 0) {
          if (cmdNo == 1) {
            if (switchStatus) {
              switchStatus = false ;
              SendIRCommand(IRAddr,IRCmdPowerOFF) ;
            } else {
              switchStatus = true ;
              SendIRCommand(IRAddr,IRCmdPowerON) ;
            }
          } else
          if (cmdNo == 2) {
            SendIRCommand(IRAddr,IRCmdColor) ;
          } else
          if (cmdNo == 3) {
              SendIRCommand(IRAddr,IRCmdMode) ;
          }
          delay(200) ;
          M5.Lcd.fillRect(  50, 85, 30, 30, MAGENTA);
          M5.Lcd.fillRect( 110, 85, 30, 30, MAGENTA);
          M5.Lcd.fillRect( 170, 85, 30, 30, MAGENTA);
          delay(500) ;
        }

        clapCommandNum = 0 ;
        readStat = true ;
        M5.Lcd.fillRect(  50, 85, 30, 30, GREEN);
        M5.Lcd.fillRect( 110, 85, 30, 30, GREEN);
        M5.Lcd.fillRect( 170, 85, 30, 30, GREEN);
      }
    }

    // Wait to ignore noise
    digitalWrite(10,LOW) ;
    delay(50) ;
    digitalWrite(10,HIGH) ;

  } else {
    if (!readStat) {
      passtTime = millis() - checkTime ;
      if (passtTime > 1000) {
        clapCommandNum = 0 ;
        readStat = true ;
        M5.Lcd.fillRect(  50, 85, 30, 30, GREEN);
        M5.Lcd.fillRect( 110, 85, 30, 30, GREEN);
        M5.Lcd.fillRect( 170, 85, 30, 30, GREEN);
        Serial.println("Command Cancel") ;
      }
    } else {
      // manual operation
      if (digitalRead(BUTTONA) == LOW) {
        if (switchStatus) {
          switchStatus = false ;
          SendIRCommand(IRAddr,IRCmdPowerOFF) ;
        } else {
          switchStatus = true ;
          SendIRCommand(IRAddr,IRCmdPowerON) ;
        }
        delay(500) ;
      }
      if (digitalRead(BUTTONB) == LOW) {
        cnt = (cnt + 1) % 10 ; 
        if (cnt == 0) {
          SendIRCommand(IRAddr,IRCmdCOLOR[color]) ;
          color = (color + 1) % IRCmdCOLORNum ;
        }
        delay(500) ;
      }
    }
  }
}
