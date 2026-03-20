#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "mavlink/common/mavlink.h"

const char* AP_SSID     = "DroneMonitor";
const char* AP_PASS     = "12345678";
const uint16_t QGC_PORT = 14550;

#define sampleFreq 100.0f
#define twoKpDef   2.0f
#define twoKiDef   0.0f
float twoKp=twoKpDef,twoKi=twoKiDef;
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
float integralFBx=0.0f,integralFBy=0.0f,integralFBz=0.0f;

void MahonyUpdate(float gx,float gy,float gz,float ax,float ay,float az){
  float recipNorm,halfvx,halfvy,halfvz,halfex,halfey,halfez,qa,qb,qc;
  if(!((ax==0.0f)&&(ay==0.0f)&&(az==0.0f))){
    recipNorm=1.0f/sqrtf(ax*ax+ay*ay+az*az);
    ax*=recipNorm;ay*=recipNorm;az*=recipNorm;
    halfvx=q1*q3-q0*q2;halfvy=q0*q1+q2*q3;halfvz=q0*q0-0.5f+q3*q3;
    halfex=ay*halfvz-az*halfvy;halfey=az*halfvx-ax*halfvz;halfez=ax*halfvy-ay*halfvx;
    if(twoKi>0.0f){
      integralFBx+=twoKi*halfex*(1.0f/sampleFreq);
      integralFBy+=twoKi*halfey*(1.0f/sampleFreq);
      integralFBz+=twoKi*halfez*(1.0f/sampleFreq);
      gx+=integralFBx;gy+=integralFBy;gz+=integralFBz;
    } else {integralFBx=0.0f;integralFBy=0.0f;integralFBz=0.0f;}
    gx+=twoKp*halfex;gy+=twoKp*halfey;gz+=twoKp*halfez;
  }
  gx*=(0.5f/sampleFreq);gy*=(0.5f/sampleFreq);gz*=(0.5f/sampleFreq);
  qa=q0;qb=q1;qc=q2;
  q0+=(-qb*gx-qc*gy-q3*gz);q1+=(qa*gx+qc*gz-q3*gy);
  q2+=(qa*gy-qb*gz+q3*gx);q3+=(qa*gz+qb*gy-qc*gx);
  recipNorm=1.0f/sqrtf(q0*q0+q1*q1+q2*q2+q3*q3);
  q0*=recipNorm;q1*=recipNorm;q2*=recipNorm;q3*=recipNorm;
}

#define MPU_ADDR     0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define LOOP_HZ      100
#define LOOP_US      (1000000/LOOP_HZ)

void mpuWrite(uint8_t reg,uint8_t val){Wire.beginTransmission(MPU_ADDR);Wire.write(reg);Wire.write(val);Wire.endTransmission();}
void mpuRead(uint8_t reg,uint8_t* buf,uint8_t len){Wire.beginTransmission(MPU_ADDR);Wire.write(reg);Wire.endTransmission(false);Wire.requestFrom(MPU_ADDR,(uint8_t)len);for(uint8_t i=0;i<len;i++)buf[i]=Wire.read();}
int16_t toInt16(uint8_t hi,uint8_t lo){return(int16_t)((hi<<8)|lo);}

float offAx=0,offAy=0,offAz=0,offGx=0,offGy=0,offGz=0;
float g_roll=0,g_pitch=0,g_yaw=0;

void getRaw(float &ax,float &ay,float &az,float &gx,float &gy,float &gz){
  uint8_t raw[14];mpuRead(ACCEL_XOUT_H,raw,14);
  ax=toInt16(raw[0],raw[1])/16384.0f;ay=toInt16(raw[2],raw[3])/16384.0f;
  az=toInt16(raw[4],raw[5])/16384.0f;gx=toInt16(raw[8],raw[9])/131.0f;
  gy=toInt16(raw[10],raw[11])/131.0f;gz=toInt16(raw[12],raw[13])/131.0f;
}

void calibrate(int n=500){
  Serial.println("Calibrating...");
  double sax=0,say=0,saz=0,sgx=0,sgy=0,sgz=0;
  float ax,ay,az,gx,gy,gz;
  for(int i=0;i<n;i++){
    getRaw(ax,ay,az,gx,gy,gz);
    sax+=ax;say+=ay;saz+=az;sgx+=gx;sgy+=gy;sgz+=gz;delay(2);
  }
  offAx=sax/n;offAy=say/n;offAz=saz/n-1.0f;
  offGx=sgx/n;offGy=sgy/n;offGz=sgz/n;
  Serial.println("Done.");
}

WiFiUDP udp;
IPAddress qgcIP;
bool qgcKnown=false;
bool armed=false;

void sendMavlink(uint8_t* buf,uint16_t len){
  if(qgcKnown){udp.beginPacket(qgcIP,QGC_PORT);udp.write(buf,len);udp.endPacket();}
}

void sendHeartbeat(){
  mavlink_message_t msg;uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint8_t mode=armed?MAV_MODE_FLAG_SAFETY_ARMED|MAV_MODE_FLAG_MANUAL_INPUT_ENABLED:MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
  mavlink_msg_heartbeat_pack(1,MAV_COMP_ID_AUTOPILOT1,&msg,MAV_TYPE_QUADROTOR,MAV_AUTOPILOT_GENERIC,mode,0,armed?MAV_STATE_ACTIVE:MAV_STATE_STANDBY);
  sendMavlink(buf,mavlink_msg_to_send_buffer(buf,&msg));
}

void sendAttitude(float r,float p,float y){
  mavlink_message_t msg;uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_attitude_pack(1,MAV_COMP_ID_AUTOPILOT1,&msg,millis(),r,p,y,0,0,0);
  sendMavlink(buf,mavlink_msg_to_send_buffer(buf,&msg));
}

void sendSysStatus(){
  int raw=analogRead(34);
  float voltage=raw*3.3f/4095.0f;
  uint16_t vbat=(uint16_t)(voltage*1000*4);
  mavlink_message_t msg;uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_sys_status_pack(1,MAV_COMP_ID_AUTOPILOT1,&msg,
    0,0,0,500,vbat,100,50,0,0,0,0,0,0,0,0,0);
  sendMavlink(buf,mavlink_msg_to_send_buffer(buf,&msg));
}

void sendVfrHud(){
  mavlink_message_t msg;uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  float heading=g_yaw*RAD_TO_DEG;
  if(heading<0)heading+=360;
  mavlink_msg_vfr_hud_pack(1,MAV_COMP_ID_AUTOPILOT1,&msg,0,0,(uint16_t)heading,armed?50:0,0,0);
  sendMavlink(buf,mavlink_msg_to_send_buffer(buf,&msg));
}

void sendStatusText(const char* text,uint8_t severity=MAV_SEVERITY_INFO){
  mavlink_message_t msg;uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_statustext_pack(1,MAV_COMP_ID_AUTOPILOT1,&msg,severity,text,0,0);
  sendMavlink(buf,mavlink_msg_to_send_buffer(buf,&msg));
}

void sendRcChannels(){
  mavlink_message_t msg;uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t ch1=(uint16_t)(analogRead(34)*3.3f/4095.0f/3.3f*1000+1000);
  mavlink_msg_rc_channels_pack(1,MAV_COMP_ID_AUTOPILOT1,&msg,
    millis(),8,
    ch1,1500,1500,1500,1500,1500,1500,1500,
    1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,
    255);
  sendMavlink(buf,mavlink_msg_to_send_buffer(buf,&msg));
}

void handleIncoming(){
  int sz=udp.parsePacket();
  if(sz>0){
    qgcIP=udp.remoteIP();
    if(!qgcKnown){
      qgcKnown=true;
      Serial.printf("QGC found at: %s\n",qgcIP.toString().c_str());
      sendStatusText("ESP32 DroneMonitor connected");
    }
    uint8_t buf[512];
    int len=udp.read(buf,sizeof(buf));
    mavlink_message_t msg;mavlink_status_t status;
    for(int i=0;i<len;i++){
      if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&status)){
        if(msg.msgid==76){
          mavlink_command_long_t cmd;
          mavlink_msg_command_long_decode(&msg,&cmd);
          if(cmd.command==400){
            armed=(cmd.param1==1.0f);
            Serial.printf("%s\n",armed?"ARMED":"DISARMED");
            sendStatusText(armed?"Armed":"Disarmed");
          }
        }
      }
    }
  }
}

void setup(){
  Serial.begin(115200);
  delay(1000);
  Wire.begin(21,22);delay(500);
  mpuWrite(PWR_MGMT_1,0x00);delay(100);
  calibrate(500);
  WiFi.softAP(AP_SSID,AP_PASS);
  Serial.printf("AP: %s  IP: %s\n",AP_SSID,WiFi.softAPIP().toString().c_str());
  udp.begin(QGC_PORT);
  Serial.println("Waiting for QGC...");
}

uint32_t lastLoop=0,lastHB=0;

void loop(){
  while(micros()-lastLoop<LOOP_US);
  lastLoop=micros();
  handleIncoming();
  float ax,ay,az,gx,gy,gz;
  getRaw(ax,ay,az,gx,gy,gz);
  ax-=offAx;ay-=offAy;az-=offAz;
  gx-=offGx;gy-=offGy;gz-=offGz;
  MahonyUpdate(gx*DEG_TO_RAD,gy*DEG_TO_RAD,gz*DEG_TO_RAD,ax,ay,az);
  g_roll =atan2f(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
  g_pitch=asinf (2*(q0*q2-q3*q1));
  g_yaw  =atan2f(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
  sendAttitude(g_roll,g_pitch,g_yaw);
  if(millis()-lastHB>1000){
    sendHeartbeat();
    sendSysStatus();
    sendVfrHud();
    sendRcChannels();
    lastHB=millis();
    Serial.printf("HB | roll:%.1f pitch:%.1f yaw:%.1f armed:%s\n",
      g_roll*RAD_TO_DEG,g_pitch*RAD_TO_DEG,g_yaw*RAD_TO_DEG,
      armed?"YES":"NO");
  }
}