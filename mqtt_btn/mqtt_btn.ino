/**
 * Brief : 
 */
#include <WiFi.h>
#include <ESPmDNS.h>
#include "Adafruit_MQTT.h"  // Add ChangeServer() to switch HostName->Ip after connecting WIFI. 
                            // use MQTT broker HostName when create MQTT instance, get MQTT broker IP from hostname after complete WIFI connection.
#include "Adafruit_MQTT_Client.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

//#define DEBUG_
#ifdef DEBUG_ 
  #define d_printf(fmt, args...)   Serial.printf(fmt, ## args) 
#else
  #define d_printf(fmt, args...)
#endif 

#include "queue.h"
#include "color_arr.h"

#define WLAN_SSID       "my-router"
#define WLAN_PASS       "xxxxxxxx"
#define AIO_SERVER      "raspberrypi" //"10.0.0.33"  // MQTT broker
char hostIP[15];  // store MQTT Broker IP. getting from Hostname.
#define AIO_SERVERPORT  1883 // MQTT port#
#define AIO_USERNAME    ""
#define AIO_KEY         ""

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
void MQTT_connect();

enum {DIMMER, COLOR_TEMP, COLOR_XY, CHANGE_GRP, END_MODE};
uint16_t switch_mode=DIMMER;

#define LED_R   26
#define LED_G   18

#define BTN_PIN 33
#define BTN_PRESSED   LOW
#define BTN_RELEASED  HIGH
#define CLICK_THRES 400  // msec  LongPress, dbl_clk interval between clicks. ( for the double click)

unsigned long clk_dn_start; // click staring tick
unsigned long cur;     // current tick
unsigned long cur_sec; // for secCount
unsigned long deque_tm; // key event dequeing timing

uint8_t clk_count; // click count
enum {None, CLK, RELEASE_EDGE, DBL_CLK, LONG_PRESSING, LONG_PRESS_END};  // must reset to 'None', after reading 'CLK', 'DBL_CLK'.
                                                                         // 'LONG_PRESSING' will be reset inside of ISR.
volatile uint8_t clk_evt; // Button Event.
hw_timer_t * clkTimer = NULL;

//portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// about MPU6050 start
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
// uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
xz_mv readDmpFIFO();
// about MPU6050 end




bool ramp_mode=true;
#define LOWEST_TEMP 150    // coolest
#define HIGHEST_TEMP 400 //440  warmest
uint16_t color_temp;

int16_t color_idx;
int16_t color_idx_max;

#define MAX_GRP_NO 8
uint8_t grpNo=7; // default Zigbee Group Number.  0:unused currently. use between 1~8
#define RGB_GRP 7

bool btn_prev=true;
void btn_polling()
{
  cur=millis();

  uint8_t btn=digitalRead(BTN_PIN);
  if(btn==btn_prev){
    return;
  } else {
    btn_prev=btn;
    delay(20); // for remove debounce
  }
  
  if(btn==BTN_RELEASED && clk_dn_start!=0){  // click_release.
    if(clk_evt!=LONG_PRESSING){ // clkTimer start when button release.
      clk_count++;
      timerWrite(clkTimer,0);
      timerAlarmEnable(clkTimer);
      
      if(timerAlarmEnabled(clkTimer))
        Serial.print("st-");
      Serial.println("Rel");

    }else{ // end of longPressing will processe inside loop()
      clk_evt=LONG_PRESS_END; 
      Serial.println("eol");
    }
    clk_dn_start=0;

  }else if(clk_dn_start==0){//  btn==BTN_PRESSED  
    clk_dn_start=cur;
    //clk_dn_end=0;
    if(timerAlarmEnabled(clkTimer)){ // if clkTimer active then Stop it.
      timerAlarmDisable(clkTimer);   // Stop
      
      Serial.print("stp-");
    }
    Serial.println("push");    
  }else {
    // need to find a reason to reach here.
    Serial.println("else"); 
  }
}

/**
 *  @brief  timer interrupt
 *          figuring single click, double.
 */
void IRAM_ATTR onTimerAfterClickReleased() 
{
//  portENTER_CRITICAL_ISR(&timerMux);
  switch(clk_count){
  case 1:
    clk_evt=CLK;
    break;
  case 2:
    clk_evt=DBL_CLK;
    break;
  default:
    clk_evt=None;
    break;
  }
  clk_count=0;
  clk_dn_start=0;  
//  portEXIT_CRITICAL_ISR(&timerMux);
  clk_count=0;
}


void setup() 
{
    Serial.begin(115200);
 
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    pinMode(LED_G, OUTPUT);
    pinMode(LED_R, OUTPUT);
    digitalWrite(LED_R, HIGH);
    pinMode(BTN_PIN, INPUT);

    // hw Timer.
    clk_dn_start=0;
    clkTimer = timerBegin(0, 80, true); // timer0, systemClock(80MHz)/80, true:countUp/false:countDn
    timerAttachInterrupt(clkTimer, &onTimerAfterClickReleased, true);   // add CB,  true:edge/false:flat 
    timerAlarmWrite(clkTimer, CLICK_THRES*1000, false);
    timerAlarmDisable(clkTimer);
    
    mpu.initialize();
    // while(!mpu.testConnection()) Serial.println( F("MPU6050 connection failed"));

    mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    mpu.setDMPEnabled(true);

    mpu.CalibrateAccel(3);//6);
    mpu.CalibrateGyro(3);//6);
    
    // packetSize = mpu.dmpGetFIFOPacketSize();
    mpu.setSleepEnabled(true);  
    color_temp=(LOWEST_TEMP+HIGHEST_TEMP)/2;
    
    // connect WIFI, MQTT broker
    WiFi.begin(WLAN_SSID, WLAN_PASS);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      d_printf(".");
    }

    int err;
    IPAddress  resultIP;
    if (!MDNS.begin("esp32")) {
      Serial.println("Error setting up MDNS responder!");
      while(1) {
          delay(1000);
      }
    }
    for (int i=0; i<5; i++)
    {
      //err = WiFi.hostByName(AIO_SERVER, result);
      resultIP = MDNS.queryHost(AIO_SERVER);
      if(resultIP.toString().length()>7)
      {
        Serial.printf("rasp IP is %s\n",resultIP.toString().c_str()); //resultIP가 stack에 있기때문에 string을 바로 mqtt에 주면 안됨.
        strcpy(hostIP, resultIP.toString().c_str());  //전역이나 static으로 만들어놓은 hostIP 버퍼로 옮겨놓고, 그걸 줘야 IP가 유지됨.
        mqtt.changeServer((const char*)hostIP);       // adafruit_MQTT.h에 직접 changeServer 를 추가시킴. mqtt생성때 넣은 IP(비어있음)를 바꾸는게 없음.
        break;                                        // 생성자때 hostname을 넣고, 이걸로 IP로 다시 고쳐줌.
      }else{
        Serial.print(i);
        Serial.println(resultIP.toString().c_str());
      }
      delay(500);
    } // end of for()
    if(resultIP.toString().length()<=7){
      Serial.print("\nFail to get host IP!");
      while(1);
    }
    MQTT_connect();
    for(color_idx_max=0; color_arr[color_idx_max]!=0; color_idx_max++)
      ;
    color_idx_max--;  // '0'값이 아니고 실제 마지막 값을 가리키도록함.  
    digitalWrite(LED_R, LOW);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
#define A_COUNT 15 // 센서에서 읽은값을 평균내기 위한 읽기 횟수값.
#define DEQUE_ITV 250
#define Z_RATE  60
#define X_RATE  60

int ax,az; // A_COUNT 횟수만큼 읽은 값을 계속 더한 후 A_COUNT로 나눠서 여기에 넣을것임. Y값은 사용안할꺼라 빼버림.
int x_pv,z_pv; // 실제 X,Y이동을 위해서는 센서값이 변화하는 값을 이용할것이기 때문에 위 ax, az값을 저장해놓기위한 공간.
int8_t prev_z=127; // enque하기 전에 두개이상 같은값인지 확인용.(dimmer, temp에서만 사용함. 갑자기 반대방향 툭 튀어나오는것 방지용)

uint32_t secCount, read_count;  // reset후 맨처음 몇개 값은 버려야함.(크게 흔들림) 이를위한 sensor reading count
int a_count=A_COUNT;  // ax, ax값을 구하기 위한 counter
int8_t z_rate;
int8_t x_rate;
uint8_t stopCnt; // x,z {0,0}이 연속적으로 몇개 오는지 count. 3개연속이면 stop시킬것임.

void loop() 
{
    btn_polling();
    cur=millis();
    
    if(cur-cur_sec>1000){
      cur_sec=cur;
      secCount++;
      
      MQTT_connect();
      if(secCount%10==0){ // ping every 10sec
        if(! mqtt.ping()) {
          mqtt.disconnect();
        }
      } 
    }// end of secCount
        
    switch(clk_evt){
    case CLK:
    {
      //Adafruit_MQTT_Publish mqttPub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "zigbee2mqtt/grp1/set");
      //mqttPub.publish("toggle");
      //bool publish(const char *topic, const char *payload, uint8_t qos = 0);
      char buf_tp[30];
      //sprintf(buf_tp,"zigbee2mqtt/grp%d/set",grpNo);
      if(grpNo!=RGB_GRP){
        sprintf(buf_tp,"zigbee2mqtt/grp%d/set", grpNo);
      }else{
        sprintf(buf_tp,"zigbee2mqtt/sengledRGB/set");  
      }        
      mqtt.publish((const char*)buf_tp, "toggle");
      d_printf("Clicked!\n");
      clk_evt=None;
      break;
    }

    case DBL_CLK:
      d_printf("Double Clicked!\n");
      //enum {DIMMER, COLOR_TEMP, COLOR_XY, CHANGE_GRP, END_MODE}
      switch_mode++;
      if(switch_mode==END_MODE)
        switch_mode=DIMMER;
      clk_evt=None;

      digitalWrite(LED_G, HIGH);
      delay(500);
      for(int i=0; i<switch_mode; i++){
        digitalWrite(LED_G, LOW);
        delay(250);
        digitalWrite(LED_G, HIGH);
        delay(500);
      }
      digitalWrite(LED_G, LOW);
      break;
    case LONG_PRESSING: // holding button
      if(switch_mode!=CHANGE_GRP){ // CHANGE_GRP만 MPU6050 센서값 필요없음.
        xz_mv ret = readDmpFIFO();
        
        if(ret.x==127 && ret.z==127){ // 준비안된값임(평균값 구해지는 과정). 그냥 switch pass시킴.
          break;
        }
        if(ret.x!=0 || ret.z!=0){
          // color바꿀땐 그냥 바로 enque시키고, dimmer or temp바꿀땐 이전 z값과 비교해서 같을때(2개이상 같은값이 들어올 때만 enque)
          if(switch_mode==COLOR_XY || prev_z==ret.z){ 
            enQue(ret);
          }
          prev_z=ret.z;
          stopCnt=0;
        }
      }
      break;
    case LONG_PRESS_END:
      xz_mv stop_mv={0,0};
      emptyQue(); // 이전에 있던거 다 없애고
      enQue(stop_mv); // 바로 stop처리함.
      z_rate=stopCnt=0;
      prev_z=127;
      d_printf("End-Long\n");
      if(switch_mode==CHANGE_GRP){ // CHANGE_GRP mode에선 Long Press로 grpNo값을 변경함.
        grpNo++;// 1~MAX_GRP_NO (현재 8)
        if(grpNo>=MAX_GRP_NO)
          grpNo=1;
        digitalWrite(LED_R, HIGH);  // CHANGE_GRP에서 long press로 GRP값 바꿨을때 RED LED로 GRP# 알림.
        delay(500);
        for(int i=1; i<grpNo; i++){
          digitalWrite(LED_R, LOW);
          delay(250);
          digitalWrite(LED_R, HIGH);
          delay(500);
        }
        digitalWrite(LED_R, LOW);          
      }
      x_pv=z_pv=0;
      read_count=0;
      mpu.setSleepEnabled(true);
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_R, LOW);
      delay(30);// 이때도 noise때문에 필요함.     
      clk_evt=None;
      break;
    }
      
    cur=millis();
    if( (clk_dn_start!=0) && (cur-clk_dn_start>CLICK_THRES) )           // Long Pressing이 시작되는 moment
    {
      if((digitalRead(BTN_PIN)==BTN_PRESSED) && (clk_evt!=LONG_PRESSING) ) 
      {
        //Serial.println("lstart");
        clk_count=0; // 혹 여러번 click하다가 Long Pressing으로 들어오면 이전 clk_count 무시함.
        clk_evt=LONG_PRESSING; // LONG_PRESSING 완료는 btn isr에서 release때 처리.

        if(switch_mode!=CHANGE_GRP){ // CHANGE_GRP만 MPU6050사용 안함.
          mpu.setSleepEnabled(false);
          digitalWrite(LED_G, HIGH);
          //cur_xz=;
        }
      }
    }

    if(cntQue()>0 && (cur-deque_tm)>DEQUE_ITV)
    {
      deque_tm=cur;

      xz_mv dq;
      dq=deQue();
      if(dq.z!=0){  // 
        if(dq.z != z_rate){ // 움직임 시작(변화)에 해당함(버튼 pressed와 틀림). 방향 변화가 있을때만 brightness_move 보냄. 
          //Adafruit_MQTT_Publish mqttPub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "zigbee2mqtt/grp1/set");
          char buf_tp[30]; // topic for publish          
          char payload[70];
          z_rate=dq.z; //Serial.print("UP,");
          if(switch_mode==DIMMER){
            sprintf(payload, "{\"brightness_move\":%d}", z_rate);
            //mqttPub.publish(payload);
            //sprintf(buf_tp,"zigbee2mqtt/grp%d/set",grpNo);
            if(grpNo!=RGB_GRP){
              sprintf(buf_tp,"zigbee2mqtt/grp%d/set", grpNo);
            }else{
              sprintf(buf_tp,"zigbee2mqtt/sengledRGB/set");  
            }            
            mqtt.publish((const char*)buf_tp, payload);
            d_printf(payload);
            d_printf("\n");            
          }else if(switch_mode==COLOR_TEMP){
            ramp_mode=true;
            deque_tm=cur;//+DEQUE_ITV; // 빨리 시작되게끔.
          }else if(switch_mode==COLOR_XY){
            deque_tm=cur;//+DEQUE_ITV; // 빨리 시작되게끔.
          }
        }else{
          d_printf("skip\n");
        }
      }else if(dq.x==0 && dq.z==0){  // 0,0 이면 longpress end
        //Adafruit_MQTT_Publish mqttPub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "zigbee2mqtt/grp1/set");
        char payload[50];
        char buf_tp[30];
        //sprintf(buf_tp,"zigbee2mqtt/grp%d/set",grpNo);
        if(grpNo!=RGB_GRP){
          sprintf(buf_tp,"zigbee2mqtt/grp%d/set", grpNo);
        }else{
          sprintf(buf_tp,"zigbee2mqtt/sengledRGB/set");  
        }
        z_rate=0;

        if(switch_mode==DIMMER){
          sprintf(payload, "{\"brightness_move\":%d}", z_rate);
        }else { //if(switch_mode==COLOR_TEMP ||switch_mode==COLOR_XY){
          sprintf(payload, ""); // 딱히 stop이 없음. 그냥 같은값 한번 더.
        }
        //mqttPub.publish(payload);
        if(strlen(payload))
          mqtt.publish((const char*)buf_tp, payload);
          
        d_printf("stop\n");
      }
    }else if(((switch_mode==COLOR_TEMP&& (cur-deque_tm)>(DEQUE_ITV*2)) || // 움직임 시작되고, que와 상관없는 timer Interval에 의한 동작.
              (switch_mode==COLOR_XY&& (cur-deque_tm)>(DEQUE_ITV*1)) 
             ) && z_rate!=0)     // interval은 deque의 두배, btn release가 되면 z_rate=0가 됨.
    {
      //Adafruit_MQTT_Publish mqttPub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "zigbee2mqtt/grp1/set");
      char payload[50];
      char buf_tp[30];
      if(grpNo!=RGB_GRP){
        sprintf(buf_tp,"zigbee2mqtt/grp%d/set", grpNo);
      }else{
        sprintf(buf_tp,"zigbee2mqtt/sengledRGB/set");  
      }
      deque_tm=cur;
      if(switch_mode==COLOR_TEMP || (switch_mode==COLOR_XY && ramp_mode==true)){ // color_temp변화 또는 color_xy변화하기위해 warmest로 변화.
        if(switch_mode==COLOR_TEMP){
          color_temp+=z_rate/4;
        }else{  // switch_mode==COLOR_XY && ramp_mode==true
          color_temp-=Z_RATE/4; // COLOR_XY모드에선 COOLEST로 가기위해 값을 계속 -함.
        }
  
        if(color_temp>HIGHEST_TEMP){    // HIGHEST => WARMEST
          color_temp=HIGHEST_TEMP;
        }else if(color_temp<LOWEST_TEMP){  // LOWEST => COOLEST
          color_temp=LOWEST_TEMP;
          if(switch_mode==COLOR_XY && ramp_mode==true){
            ramp_mode=false; // 다음부턴 실제 color값이 바뀔것임.
            color_idx=60; // 0x00f Blue
          }          
        }        
        sprintf(payload, "{\"color_temp\":%d}", color_temp);
        Serial.println(payload);
        //mqttPub.publish(payload);
        mqtt.publish((const char*)buf_tp, payload);
      } else if(switch_mode==COLOR_XY && ramp_mode==false) { // 실제 color xy변화하는 
        uint8_t r,g,b;
        r = (uint8_t)((color_arr[color_idx]&0xf00)>>4);   // 0xRvv => 0xR0
        g = (uint8_t) (color_arr[color_idx]&0x0f0);       // 0xvGv => 0xG0
        b = (uint8_t)((color_arr[color_idx]&0x00f)<<4);   // 0xvvB => 0xB0
        //sprintf(payload, "{\"color\":{\"rgb\":\"%d,%d,%d\"}}",r,g,b);
        sprintf(payload, "{\"color\":{\"hex\":\"#%.2x%.2x%.2x\"}}",r,g,b);
        
        //sprintf(payload, "{\"color\":{\"x\":%f,\"y\":%f}}",color_arr[color_idx].x, color_arr[color_idx].y);
        //Serial.print(color_idx);
        if(z_rate>0){
          color_idx+=1;
          if(color_idx>color_idx_max)
            color_idx=0;
        }else{
          color_idx-=1;
          if(color_idx<0)
            color_idx=color_idx_max;
        }
        
        Serial.println(payload);
        //mqttPub.publish(payload);
        mqtt.publish((const char*)buf_tp, payload,0);
      }
    }
}

/**
 * @brief Read a packet from FIFO
 * 
 */
xz_mv readDmpFIFO()
{
  xz_mv xz_dat={127,127}; // 아직 준비 안된값. 받는측에서 무시해버려야함.
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  
    if(a_count==0) { // A_COUNT회수 만큼읽은후 평균값을 사용함. 10msec* (A_COUNT+1)=160 msec임.
      a_count=A_COUNT;
      read_count++;
      if(read_count<3){ // 버튼 누른후 3개까지 값은 무시해버림.
        x_pv=ax/a_count; 
        z_pv=az/a_count;
      }else{
        digitalWrite(LED_R, LOW);
        digitalWrite(LED_G, HIGH);
      }
//      d_printf("%d,\t%d\n", ax/a_count - x_pv, az/a_count -z_pv);  //ax/a_count, az/a_count);
#define X_THRESHOLD 350
#define Z_THRESHOLD 350  
      if(ax/a_count-x_pv > X_THRESHOLD){  // 이전 값과의 차이를 관찰함.
        xz_dat.x = X_RATE; 
      }else if(ax/a_count-x_pv < (-1*X_THRESHOLD)){
        xz_dat.x = -X_RATE;
      }else{
        xz_dat.x=0;
      }

      if(az/a_count-z_pv > Z_THRESHOLD){
        d_printf("UP");
        xz_dat.z= Z_RATE;
      }else if(az/a_count-z_pv < (-1*Z_THRESHOLD)){
        d_printf("DN");
        xz_dat.z= -Z_RATE;
      }else{
        xz_dat.z=0;
      }
      
      x_pv=ax/a_count;// ax/a_count; 
      z_pv=az/a_count;// az/a_count;
      
      ax=az=0;
    }else{
      ax+= (100*ypr[0] * (int)(180/M_PI));   
      //ay+= ypr[1] * 180/M_PI;
      az+= (int)((-200)*ypr[2] * 180/M_PI);
      a_count--;
    }
  }
  return xz_dat;
}  


void MQTT_connect() {
  int8_t ret;

  if (mqtt.connected()) {
    return;
  }
/*  // for checking WIFI Signal strenth. ESP32 WIFI antenna sucks.  
  Serial.printf("Wifi Signal Strength(RSSI): %ddbm\n", WiFi.RSSI());
  Serial.println("(-30dbm:Great, -67:very Good, -70:OK, -80:Not Good)");
  Serial.print("Connecting to MQTT... ");
*/
  #define RETRY_NUM 5
  uint8_t retries = RETRY_NUM;
  while ((ret = mqtt.connect()) != 0)  // connect will return 0 for connected
  {
     d_printf("%s\n", mqtt.connectErrorString(ret));
//     Serial.printf("Retrying MQTT connection in %d seconds...\n", (RETRY_NUM-retries+1)*2);
     mqtt.disconnect();
     delay((RETRY_NUM-retries+1)*2000);  // wait 2,4,8,10,12
     retries--;
     if (retries == 0) {
       // basically die and wait for WDT to reset me
       while (1);
     }
  }
}
