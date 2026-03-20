#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "mavlink/common/mavlink.h"

// ===== Config =====
const char* AP_SSID     = "DroneMonitor";
const char* AP_PASS     = "12345678";
const uint16_t QGC_PORT = 14550;
const uint8_t SERVO_PIN = 13;

// ===== Mahony Filter =====
#define sampleFreq  100.0f
#define twoKpDef    2.0f
#define twoKiDef    0.0f

float twoKp = twoKpDef, twoKi = twoKiDef;
float q0=1.0f, q1=0.0f, q2=0.0f, q3=0.0f;
float integralFBx=0.0f, integralFBy=0.0f, integralFBz=0.0f;

void MahonyUpdate(float gx,float gy,float gz,float ax,float ay,float az){
  float recipNorm,halfvx,halfvy,halfvz,halfex,halfey,halfez,qa,qb,qc;
  if(!((ax==0.0f)&&(ay==0.0f)&&(az==0.0f))){
    recipNorm=1.0f/sqrtf(ax*ax+ay*ay+az*az);
    ax*=recipNorm;ay*=recipNorm;az*=recipNorm;
    halfvx=q1*q3-q0*q2;
    halfvy=q0*q1+q2*q3;
    halfvz=q0*q0-0.5f+q3*q3;
    halfex=ay*halfvz-az*halfvy;
    halfey=az*halfvx-ax*halfvz;
    halfez=ax*halfvy-ay*halfvx;
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
  q0+=(-qb*gx-qc*gy-q3*gz);
  q1+=(qa*gx+qc*gz-q3*gy);
  q2+=(qa*gy-qb*gz+q3*gx);
  q3+=(qa*gz+qb*gy-qc*gx);
  recipNorm=1.0f/sqrtf(q0*q0+q1*q1+q2*q2+q3*q3);
  q0*=recipNorm;q1*=recipNorm;q2*=recipNorm;q3*=recipNorm;
}

// ===== MPU6050 =====
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
  for(int i=0;i<n;i++){getRaw(ax,ay,az,gx,gy,gz);sax+=ax;say+=ay;saz+=az;sgx+=gx;sgy+=gy;sgz+=gz;delay(2);}
  offAx=sax/n;offAy=say/n;offAz=saz/n-1.0f;
  offGx=sgx/n;offGy=sgy/n;offGz=sgz/n;
  Serial.println("Done.");
}

// ===== MAVLink =====
WiFiUDP udp;
IPAddress qgcIP;
bool qgcKnown=false;

void sendMavlink(uint8_t* buf,uint16_t len){
  if(qgcKnown){udp.beginPacket(qgcIP,QGC_PORT);udp.write(buf,len);udp.endPacket();}
}

void sendHeartbeat(){
  mavlink_message_t msg;uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_heartbeat_pack(1,MAV_COMP_ID_AUTOPILOT1,&msg,MAV_TYPE_QUADROTOR,MAV_AUTOPILOT_GENERIC,MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,0,MAV_STATE_ACTIVE);
  sendMavlink(buf,mavlink_msg_to_send_buffer(buf,&msg));
}

void sendAttitude(float r,float p,float y){
  mavlink_message_t msg;uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_attitude_pack(1,MAV_COMP_ID_AUTOPILOT1,&msg,millis(),r,p,y,0,0,0);
  sendMavlink(buf,mavlink_msg_to_send_buffer(buf,&msg));
}

// ===== GPIO + PWM =====
void setupGPIO(){
  pinMode(2,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,INPUT_PULLUP);
  ledcAttach(SERVO_PIN,50,16);
}

uint32_t angleToDuty(int angle){
  float ms=1.0f+(angle/180.0f)*1.0f;
  return(uint32_t)((ms/20.0f)*65535);
}

// ===== Web Server + WebSocket =====
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void setupRoutes(){
  // Serve dashboard
  server.on("/",HTTP_GET,[](AsyncWebServerRequest* req){
    req->send(200,"text/html",R"rawhtml(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Drone Monitor</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:sans-serif;background:#0d0d0d;color:#eee;padding:16px}
h1{color:#4af;font-size:18px;margin-bottom:16px}
h3{color:#888;font-size:13px;font-weight:500;margin-bottom:8px;text-transform:uppercase;letter-spacing:0.05em}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:12px}
.card{background:#1a1a1a;border-radius:8px;padding:14px;border:1px solid #222}
.card.full{grid-column:1/-1}
.val{font-size:28px;font-weight:600;color:#4af;margin:4px 0}
.label{font-size:11px;color:#555;margin-bottom:2px}
.row{display:flex;gap:8px;margin-bottom:8px;flex-wrap:wrap}
button{background:#4af;color:#000;border:none;padding:8px 16px;border-radius:6px;cursor:pointer;font-size:13px;font-weight:600}
button.off{background:#2a2a2a;color:#888;border:1px solid #333}
input[type=range]{width:100%;margin:8px 0}
.status{font-size:12px;color:#4af;margin-top:8px}
.dot{width:8px;height:8px;border-radius:50%;background:#4af;display:inline-block;margin-right:6px;animation:pulse 1s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:0.3}}
.pin-row{display:flex;align-items:center;justify-content:space-between;margin-bottom:6px}
.pin-label{font-size:12px;color:#888}
.pin-val{font-size:12px;color:#4af;font-weight:600}
</style></head><body>
<h1><span class="dot"></span>Drone Attitude Monitor</h1>

<div class="grid">
  <div class="card">
    <div class="label">Roll</div>
    <div class="val" id="roll">--</div>
  </div>
  <div class="card">
    <div class="label">Pitch</div>
    <div class="val" id="pitch">--</div>
  </div>
  <div class="card">
    <div class="label">Yaw</div>
    <div class="val" id="yaw">--</div>
  </div>
  <div class="card">
    <div class="label">ADC (GPIO34)</div>
    <div class="val" id="adc">--</div>
    <div class="label">Voltage</div>
  </div>
</div>

<div class="card full" style="margin-bottom:12px">
  <h3>Digital Output</h3>
  <div class="row">
    <button onclick="gpio(2,1)">GPIO2 ON</button>
    <button class="off" onclick="gpio(2,0)">GPIO2 OFF</button>
    <button onclick="gpio(4,1)">GPIO4 ON</button>
    <button class="off" onclick="gpio(4,0)">GPIO4 OFF</button>
  </div>
  <div class="pin-row">
    <span class="pin-label">GPIO2 state</span>
    <span class="pin-val" id="pin2">--</span>
  </div>
  <div class="pin-row">
    <span class="pin-label">GPIO5 button</span>
    <span class="pin-val" id="pin5">--</span>
  </div>
</div>

<div class="card full" style="margin-bottom:12px">
  <h3>Servo (GPIO13)</h3>
  <input type="range" min="0" max="180" value="90" id="servo"
    oninput="servoMove(this.value)">
  <div class="pin-row">
    <span class="pin-label">Angle</span>
    <span class="pin-val" id="sdeg">90°</span>
  </div>
</div>

<div class="status" id="status">Connecting...</div>

<script>
var ws=new WebSocket('ws://'+location.hostname+'/ws');
ws.onopen=function(){document.getElementById('status').innerText='Connected to ESP32';};
ws.onclose=function(){document.getElementById('status').innerText='Disconnected — reload page';};
ws.onmessage=function(e){
  var d=JSON.parse(e.data);
  document.getElementById('roll').innerText=d.roll.toFixed(1)+'°';
  document.getElementById('pitch').innerText=d.pitch.toFixed(1)+'°';
  document.getElementById('yaw').innerText=d.yaw.toFixed(1)+'°';
  if(d.adc&&d.adc['34']!==undefined)
    document.getElementById('adc').innerText=d.adc['34'].toFixed(2)+' V';
  if(d.pins){
    document.getElementById('pin2').innerText=d.pins['2']?'HIGH':'LOW';
    document.getElementById('pin5').innerText=d.pins['5']?'HIGH (open)':'LOW (pressed)';
  }
};
function gpio(pin,val){fetch('/gpio/set?pin='+pin+'&val='+val);}
function servoMove(v){
  document.getElementById('sdeg').innerText=v+'°';
  fetch('/gpio/pwm?pin=13&duty='+v);
}
</script>
</body></html>
)rawhtml");
  });

  // Digital output
  server.on("/gpio/set",HTTP_GET,[](AsyncWebServerRequest* req){
    if(req->hasParam("pin")&&req->hasParam("val")){
      int pin=req->getParam("pin")->value().toInt();
      int val=req->getParam("val")->value().toInt();
      pinMode(pin,OUTPUT);digitalWrite(pin,val);
      req->send(200,"application/json","{\"pin\":"+String(pin)+",\"val\":"+String(val)+"}");
    } else req->send(400,"text/plain","Missing params");
  });

  // PWM servo
  server.on("/gpio/pwm",HTTP_GET,[](AsyncWebServerRequest* req){
    if(req->hasParam("duty")){
      int angle=constrain(req->getParam("duty")->value().toInt(),0,180);
      ledcWrite(0,angleToDuty(angle));
      req->send(200,"application/json","{\"angle\":"+String(angle)+"}");
    } else req->send(400,"text/plain","Missing duty");
  });

  // Digital read
  server.on("/gpio/read",HTTP_GET,[](AsyncWebServerRequest* req){
    if(req->hasParam("pin")){
      int pin=req->getParam("pin")->value().toInt();
      req->send(200,"application/json","{\"pin\":"+String(pin)+",\"val\":"+String(digitalRead(pin))+"}");
    } else req->send(400,"text/plain","Missing pin");
  });

  // ADC read
  server.on("/adc",HTTP_GET,[](AsyncWebServerRequest* req){
    if(req->hasParam("pin")){
      int pin=req->getParam("pin")->value().toInt();
      int raw=analogRead(pin);
      float v=raw*3.3f/4095.0f;
      char buf[64];snprintf(buf,sizeof(buf),"{\"pin\":%d,\"raw\":%d,\"voltage\":%.3f}",pin,raw,v);
      req->send(200,"application/json",buf);
    } else req->send(400,"text/plain","Missing pin");
  });
}

// ===== WebSocket push =====
uint32_t lastWS=0;
void pushWS(){
  if(millis()-lastWS<50)return;
  lastWS=millis();
  float adc34=analogRead(34)*3.3f/4095.0f;
  char buf[200];
  snprintf(buf,sizeof(buf),
    "{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
    "\"pins\":{\"2\":%d,\"5\":%d},"
    "\"adc\":{\"34\":%.3f}}",
    g_roll*RAD_TO_DEG,g_pitch*RAD_TO_DEG,g_yaw*RAD_TO_DEG,
    digitalRead(2),digitalRead(5),adc34);
  ws.textAll(buf);
}

// ===== Setup =====
void setup(){
  Serial.begin(115200);
  delay(1000);
  Wire.begin(21,22);
  delay(500);
  mpuWrite(PWR_MGMT_1,0x00);
  delay(100);
  calibrate(500);
  setupGPIO();

  WiFi.softAP(AP_SSID,AP_PASS);
  Serial.printf("AP: %s  IP: %s\n",AP_SSID,WiFi.softAPIP().toString().c_str());

  udp.begin(QGC_PORT);

  ws.onEvent([](AsyncWebSocket*,AsyncWebSocketClient*,AwsEventType,void*,uint8_t*,size_t){});
  server.addHandler(&ws);
  setupRoutes();
  server.begin();

  Serial.println("Ready.");
  Serial.println("1. Connect WiFi to DroneMonitor");
  Serial.println("2. Open QGC -> UDP -> 14550");
  Serial.println("3. Open browser -> http://192.168.4.1");
}

// ===== Loop =====
uint32_t lastLoop=0,lastHB=0;

void loop(){
  while(micros()-lastLoop<LOOP_US);
  lastLoop=micros();

  int sz=udp.parsePacket();
  if(sz>0){qgcIP=udp.remoteIP();qgcKnown=true;}

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
    sendHeartbeat();lastHB=millis();
    Serial.printf("HB | roll:%.1f pitch:%.1f yaw:%.1f\n",
      g_roll*RAD_TO_DEG,g_pitch*RAD_TO_DEG,g_yaw*RAD_TO_DEG);
  }

  pushWS();
  ws.cleanupClients();
}
