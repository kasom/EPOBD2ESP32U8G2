const char info_text[] PROGMEM = R"rawliteral(
/*
  EP-OBD2-ESP32-U8G2
  
  A standalone tool to automatically turn the DRLs off when parked.
  Can retrieves some interesting information such as battery SoC/SoH 
  and display it on an optional LCD or on a web browser via WiFi. (Both
  station mode and softAP mode are supported.)

  https://github.com/kasom/EPOBD2ESP32U8G2

  Copyright 2022, Kasom
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
  

  Hardware:
  
     1. ESP32 dev board
     2. CAN bus transceiver module (3.3v) (Tested: SN65HVD230DR)
           ***Take the 120 Ohm resister (if available) out from the board.*** 
           Our tool is a stub, not the end of the CAN bus.
           The terminating resistors are already in the car.
     3. OBD2 male connector + cable
     4. 128x64 COG 3.3v SPI LCD module (ST7565R/12864) (optional, for the LCD)
     5. USB cables, soldering iron, solder wire, box, etc.
     6. PC or Mac to build/flash the code
     

  ***LCD_K*** to GND via 180 Ohm resistor
     
  | ESP32 Dev K. | ESP32 Lite | CAN Tran. | LCD Module        |Blt-in  LED|  SW  |
  |--------------|------------|-----------|-------------------|-----------|------|
  |     3V3      |      3V    |  3V3/VCC  |  LCD_VDD          |           |      |
  |     GND      |      G     |   G/GND   |LCD_VSS,***LCD_K***|           |   X  |
  |  *GPIO_2*    |  *GPIO_22* |           |                   |     X     |      |
  |   GPIO_16    |   GPIO_16  |  CAN_RX   |                   |           |      |
  |   GPIO_17    |   GPIO_17  |  CAN_TX   |                   |           |      |
  |   GPIO_5     |   GPIO_5   |           | LCD_CS            |           |      |
  |   GPIO_15    |   GPIO_15  |           | LCD_RSE           |           |      |
  |  *GPIO_22*   |  *GPIO_2*  |           | LCD_RS (DC)       |           |      |
  |   GPIO_18    |   GPIO_18  |           | LCD_SCL           |           |      |
  |   GPIO_23    |   GPIO_23  |           | LCD_(MO)SI        |           |      |
  |   GPIO_4     |   GPIO_4   |           | LCD_A             |           |      |

  Softwares and libraries:
     
     https://www.arduino.cc/
       Arduino IDE 1.8.19
       
     https://github.com/espressif/arduino-esp32
       Arduino core for ESP32
       
     https://github.com/me-no-dev/AsyncTCP
     https://github.com/me-no-dev/ESPAsyncWebServer
        Web Server        
        
     https://github.com/me-no-dev/arduino-esp32fs-plugin
        Arduino ESP32 filesystem uploader (for the web interface)
       
     https://github.com/olikraus/u8g2 
        U8g2 graphic library (optional, for the LCD)
  
  
                                [Web browser via PC/Tablet/Mobile]
                                                |
                                                :
                                                |
                                    ---------------------
                                    | WiFi Access Point |
                                    ---------------------
                                                   |
                                                   : (WiFi station mode)
                                                   |
     ------------    -------------------     ---------------     --------
     | CAR OBD2 | -- | CAN transceiver | --- | ESP32 board | --- | GLCD |
     ------------    -------------------     ---------------     --------
                                                   |
                                                   : (WiFi softAP mode)
                                                   |
                               [Web browser via PC/Tablet/Mobile]
  
  References/tutorials:

     https://play.google.com/store/apps/details?id=com.moonoi.mgep&hl=en_US&gl=US
     https://www.youtube.com/watch?v=busKOFT2vug 
     https://github.com/openvehicles/Open-Vehicle-Monitoring-System-3/tree/master/vehicle/OVMS.V3/components/vehicle_mgev/src
     https://docs.espressif.com/projects/esp-idf/en/release-v4.1/api-reference/peripherals/can.html
     https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html
     https://randomnerdtutorials.com/getting-started-with-esp32/
     https://summivox.wordpress.com/2019/05/31/obd-2-in-bullet-points-and-examples/
     https://en.wikipedia.org/wiki/ISO_15765-2
     https://www.mgevs.com/threads/where-is-the-obd2-port-on-an-mg5.1386/ (mg5_data_extended.txt)
     https://github.com/peternixon/MG-EV-OBD-PID/find/main
     
 */
)rawliteral";

#include "config.h"

/*****************************************************************************/

#if !defined( ESP32 )
  #error This code is intended to run on the ESP32 platform! Please check your Tools->Board setting.
#endif

String version_string=String("Firmware compiled : ") + String(__DATE__ " " __TIME__ "\n") + "Built for " 
#ifdef ESP32_LITE
                + "ESP32 Lite"
#elif defined ESP32_DEV_KIT_V1
                + "ESP32 Dev Kit v1"
#else
                + "unknown configuration"
#endif
                + "\n";


#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <driver/gpio.h>
#include <driver/can.h>
#include <Preferences.h>
#include <nvs_flash.h>
#include <SPIFFS.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#ifdef USE_OTA_WEB_UPDATER
# include <Update.h>
#endif
#include <Preferences.h>
#include "LCD.h"
#include "AutoPollingMessages.h"

WiFiUDP wifiUDP;
AsyncWebServer server(WEB_SERVER_PORT);
AsyncWebSocket ws("/ws");

const char *our_name=OUR_NAME;

const char *preference_name=our_name;

const char *default_soft_ap_ssid = DEFAULT_SOFT_AP_SSID;
const char *default_soft_ap_pass = DEFAULT_SOFT_AP_PASS;

const char *default_sta_ap_ssid = DEFAULT_STA_AP_SSID;
const char *default_sta_ap_pass = DEFAULT_STA_AP_PASS;

String local_ip="";

bool wifi_ap_on=false;
bool wifi_sta_on=false;

bool wifi_was_disconnected=false;

enum DRL_STATES { DRL_AUTO, DRL_OFF, DRL_ON };
enum DRL_CONTROLS { DRL_TURN_ON, DRL_TURN_OFF, DRL_DO_NOTHING };
enum LED_STATES { LED_STATE_DRL_AUTO, LED_STATE_DRL_OFF, LED_STATE_DRL_ON, LED_STATE_RESET_WARNING, LED_STATE_RESETTING };
uint32_t led_patterns[]={ 0x00000001, 0x00000011, 0x00000111, 0x55555555, 0xf5f5f5f5 };

DRL_STATES target_drl_state=DRL_AUTO;
DRL_CONTROLS drl_control=DRL_DO_NOTHING;

DRL_STATES previous_target_drl_state=DRL_AUTO;
DRL_CONTROLS previous_drl_control=DRL_DO_NOTHING;
char previous_gear_pos=' ';


unsigned long previousMillisWifiRetry = 0;
unsigned long previousMillisOBD2Polling = 0;

int pollingHead = 0;
volatile bool autoPolling = true;
volatile bool gwmAuthRequest = false;
unsigned long lastReceived = 0;
unsigned long lastPollStart=0;
unsigned long lastStartAuth=0;

SemaphoreHandle_t canBusSendSemaphore;
can_message_t canMessages[CAN_SEND_BUFFER_SIZE];
unsigned char canMessageHead=0;
unsigned char canMessageTail=0;

float hvb_SoH=0.0;
float hvb_SoC=0.0;
float hvb_coolant_temp=0.0;
float hvb_v=0.0;
float hvb_a=0.0;
char gear_pos=' ';
float dc_dc_temp=0.0;
float dc_dc_v=0.0;
float dc_dc_a=0.0;
float dc_dc_w=0.0;
float motor_coolant_temp=0.0;
float motor_temp=0.0;
int charging_status=0;
char charging_status_text[20]="";
unsigned long odometer=0;

char can2udp_ip[128]="";
uint16_t can2udp_port=0;
bool can2udp_enable=false;

void wiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_START:
        Serial.println("WiFi client started");
#ifdef USE_LCD
        lcd_wifi_status=LCD_WIFI_DISCONNECTED;
        break;
#endif
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        Serial.println("Connected to access point");
        wifi_was_disconnected=false;
#ifdef USE_LCD
        lcd_wifi_status=LCD_WIFI_CONNECTED_HIGH;
#endif
        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        if (wifi_sta_on) {
          if (!wifi_was_disconnected) {
            Serial.println("Disconnected from WiFi access point");
            wifi_was_disconnected=true;
#ifdef USE_LCD
            lcd_wifi_status=LCD_WIFI_DISCONNECTED;
#endif
            WiFi.disconnect();
          }
        }
        break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        Serial.print("Obtained IP address: ");
        Serial.println(WiFi.localIP());
        local_ip=WiFi.localIP().toString();
#ifdef USE_LCD
        strcpy(lcd_text,local_ip.c_str());
#endif
        break;
    case ARDUINO_EVENT_WIFI_AP_START:
        Serial.println("WiFi access point started");
#ifdef USE_LCD
        lcd_ap_status=LCD_AP_ON;
#endif
        break;
    default: break;
  }
}

void reset_all_data() {
  nvs_flash_erase();
  nvs_flash_init();
  //SPIFFS.format();
  ESP.restart();
}

void webMain(AsyncWebServerRequest *request) {
  bool has_web=SPIFFS.exists("/web/index.htm");
  request->redirect(has_web?"/web/index.htm":"/web-update");
}

void getInfo(AsyncWebServerRequest *request) {
  char buffer[2048];

  sprintf(buffer,"{ "
            "\"hvb_SoH\": %f, "
            "\"hvb_SoC\": %f, "
            "\"coolant_temp\": %f, "
            "\"hvb_v\": %f, "
            "\"hvb_a\": %f, "
            "\"gear_pos\": \"%c\", "
            "\"dc_dc_temp\": %f, "
            "\"dc_dc_v\": %f, "
            "\"dc_dc_a\": %f, "
            "\"dc_dc_w\": %f, "
            "\"motor_coolant_temp\": %f, "
            "\"motor_temp\": %f, "
            "\"charging_status\": %d, "
            "\"charging_status_text\": \"%s\" "
            " }",
            hvb_SoH,
            hvb_SoC,
            hvb_coolant_temp,
            hvb_v,
            hvb_a,
            gear_pos,
            dc_dc_temp,
            dc_dc_v,
            dc_dc_a,
            dc_dc_w,
            motor_coolant_temp,
            motor_temp,
            charging_status,
            charging_status_text
            );

  request->send(200, "application/json", buffer);
}

#ifdef USE_OTA_WEB_UPDATER
void firmwareUploadForm(AsyncWebServerRequest *request) {
  
  const char firmware_upload_form[] PROGMEM = R"rawliteral(

<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>

<p>Firmware Update (EPOBD2ESP32U8G2.<b>ino.esp32</b>.bin)</p>

<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>
  <input type='file' name='update'>
  <input type='submit' value='Update'>
</form>

<div id='prg'>progress: 0%</div>

<script>
  $('form').submit(function(e) {
    e.preventDefault();
    
    var form = $('#upload_form')[0];
    var data = new FormData(form);
    
    $.ajax({
      url: '/firmware-update',
      type: 'POST',
      data: data,
      contentType: false,
      processData: false,
      xhr: function() {
        var xhr = new window.XMLHttpRequest();
        xhr.upload.addEventListener('progress', function(evt) {
          if (evt.lengthComputable) {
            var per = evt.loaded / evt.total;
            $('#prg').html('progress: ' + Math.round(per*100) + '%');
          }
        }, false);
        
        return xhr;
        },
        success:function(d, s) {
          console.log('success!')
        },
        error: function (a, b, c) {
       }
    });
  });
</script>

)rawliteral";

  request->send(200, "text/html", firmware_upload_form);
}

void firmwareUpdateHandler(AsyncWebServerRequest *request) {
  request->send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
  ESP.restart();
}

void firmwareUploadingFileHandler(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)  {
  if (index==0) {
    // start
#ifdef USE_LCD
    strcpy(lcd_text,"UPDATING...");
    updateLCDScreen();
#endif
    Serial.printf("Update firmware: %s\r\n", filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
      Update.printError(Serial);
    }
  }
  
  if (len) {
    /* flashing firmware to ESP*/
    if (Update.write(data, len) != len) {
      Update.printError(Serial);
    }
  }
  if (final) {
    if (Update.end(true)) { //true to set the size to the current progress
      Serial.println("Update Success.");
      Serial.println("Rebooting...");
    } else {
      Update.printError(Serial);
    }
  }
}

bool webUpdateHasError=false;

void webUploadForm(AsyncWebServerRequest *request) {
  const char web_upload_form[] PROGMEM = R"rawliteral(

<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>

<p>Web Update (EPOBD2ESP32U8G2.<b>spiffs</b>.bin)</p>

<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>
  <input type='file' name='update'>
  <input type='submit' value='Update'>
</form>

<div id='prg'>progress: 0%</div>

<script>
  $('form').submit(function(e) {
    e.preventDefault();
    
    var form = $('#upload_form')[0];
    var data = new FormData(form);
    
    $.ajax({
      url: '/web-update',
      type: 'POST',
      data: data,
      contentType: false,
      processData: false,
      xhr: function() {
        var xhr = new window.XMLHttpRequest();
        xhr.upload.addEventListener('progress', function(evt) {
          if (evt.lengthComputable) {
            var per = evt.loaded / evt.total;
            $('#prg').html('progress: ' + Math.round(per*100) + '%');
          }
        }, false);
        
        return xhr;
        },
        success:function(d, s) {
          console.log('success!')
        },
        error: function (a, b, c) {
       }
    });
  });
</script>

)rawliteral";

  request->send(200, "text/html", web_upload_form);
}

void webUpdateHandler(AsyncWebServerRequest *request) {
  request->send(200, "text/plain", (webUpdateHasError) ? "FAIL" : "OK");
  ESP.restart();
}

void webUploadingFileHandler(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)  {
  if (index == 0) {
    // start
#ifdef USE_LCD
    strcpy(lcd_text,"WEB UPDATE...");
    updateLCDScreen();
#endif
    Serial.printf("Update web: %s\r\n", filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN,U_SPIFFS)) { //start with max available size
      Update.printError(Serial);
    }
  }
  if (len) {
    /* flashing  SPIFFS */
    if (Update.write(data, len) != len) {
      Update.printError(Serial);
    }
  }
  if (final) {
    if (Update.end(true)) { //true to set the size to the current progress
      Serial.println("Update Success.");
      Serial.println("Rebooting...");
    } else {
      Update.printError(Serial);
    }
  }
}

#endif

void listAllFiles(){
 
  File root = SPIFFS.open("/");
 
  File file = root.openNextFile();
 
  while(file){
 
      Serial.print("FILE: ");
      Serial.println(file.name());
 
      file = root.openNextFile();
  }
 
}

void getInfoText(AsyncWebServerRequest *request) {
  request->send(200, "text/plain", version_string+"Local ip: "+(local_ip==""?"not connected":local_ip)+"\n"+info_text);
}

void getWiFiConfig(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  DynamicJsonDocument json(1024);

  Preferences preferences;
  preferences.begin(preference_name, true); // open in read-only mode

  json["ap_mode_en"] = preferences.getBool("ap_mode_en",DEFAULT_AP_MODE_EN);
  json["soft_ap_ssid"] = preferences.getString("soft_ap_ssid",default_soft_ap_ssid);
  json["soft_ap_pass"] = preferences.getString("soft_ap_pass",default_soft_ap_pass);
  
  json["sta_mode_en"] = preferences.getBool("sta_mode_en",DEFAULT_STA_MODE_EN);  
  json["sta_ap_ssid"] = preferences.getString("sta_ap_ssid",default_sta_ap_ssid);
  json["sta_ap_pass"] = preferences.getString("sta_ap_pass",default_sta_ap_pass);

  preferences.end();

  serializeJson(json, *response);
  request->send(response);
}

void getCAN2UDPConfig(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  DynamicJsonDocument json(1024);

  Preferences preferences;
  preferences.begin(preference_name, true); // open in read-only mode

  json["can2udp_ip"] = preferences.getString("can2udp_ip",can2udp_ip);
  json["can2udp_port"] = preferences.getUShort("can2udp_port",can2udp_port);
  json["can2udp_enable"] = preferences.getBool("can2udp_enable",can2udp_enable);
  
  preferences.end();

  serializeJson(json, *response);
  request->send(response);
}


#ifdef USE_LCD
void lcdAdjust(AsyncWebServerRequest *request) {
  char buffer[100];

  Preferences preferences;

  preferences.begin(preference_name, false); // open in read-write mode

  int brightness=preferences.getInt("lcd_brightness",DEFAULT_LCD_BRIGHTNESS);
  int contrast=preferences.getInt("lcd_contrast",DEFAULT_LCD_CONTRAST);

  if (request->hasParam("contrast")) {
    contrast = request->getParam("contrast")->value().toInt();
    setLCDContrast(contrast);
    preferences.putInt("lcd_brightness",contrast);
    Serial.println("New contrast: " + contrast);
  }
  if (request->hasParam("brightness")) {
    brightness = request->getParam("brightness")->value().toInt();
    setLCDBrightness(brightness);
    preferences.putInt("lcd_brightness",brightness);
    Serial.println("New brightness: " + brightness);
  }

  preferences.end();

  sprintf(buffer,"{ \"brightness\" : %d, \"contrast\" : %d }",brightness,contrast);

  request->send(200, "application/json", buffer);
}
#endif

// https://github.com/me-no-dev/ESPAsyncWebServer/blob/master/examples/ESP_AsyncFSBrowser/ESP_AsyncFSBrowser.ino
void onWSEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if(type == WS_EVT_CONNECT){
    //client connected
    Serial.printf("ws[%s][%u] connect\r\n", server->url(), client->id());
    client->printf("Client %u\n%s\n", client->id(),version_string.c_str());
    //client->ping();
  } else if(type == WS_EVT_DISCONNECT){
    //client disconnected
    Serial.printf("ws[%s][%u] disconnect: %u\r\n", server->url(), client->id());
  } else if(type == WS_EVT_ERROR){
    //error was received from the other end
    Serial.printf("ws[%s][%u] error(%u): %s\r\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);
  } else if(type == WS_EVT_PONG){
    //pong message was received (in response to a ping request maybe)
    Serial.printf("ws[%s][%u] pong[%u]: %s\r\n", server->url(), client->id(), len, (len)?(char*)data:"");
  } else if(type == WS_EVT_DATA){
    //data packet
    AwsFrameInfo * info = (AwsFrameInfo*)arg;
    if(info->final && info->index == 0 && info->len == len){
      //the whole message is in a single frame and we got all of it's data
      Serial.printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT)?"text":"binary", info->len);
      if(info->opcode == WS_TEXT){
        data[len] = 0;
        Serial.printf("%s\r\n", (char*)data);

        char message_buffer[256];
        message_buffer[0]='*';
        message_buffer[1]=' ';

        if (xSemaphoreTake(canBusSendSemaphore,(TickType_t)10)) {
          if ((canMessageTail+1)%CAN_SEND_BUFFER_SIZE != canMessageHead) {
            // not full

            sscanf((char *)data,"%x %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx",
                  &canMessages[canMessageTail].identifier,
                  &canMessages[canMessageTail].flags,
                  &canMessages[canMessageTail].data_length_code,
                  &canMessages[canMessageTail].data[0],
                  &canMessages[canMessageTail].data[1],
                  &canMessages[canMessageTail].data[2],
                  &canMessages[canMessageTail].data[3],
                  &canMessages[canMessageTail].data[4],
                  &canMessages[canMessageTail].data[5],
                  &canMessages[canMessageTail].data[6],
                  &canMessages[canMessageTail].data[7]);

            canMessageTail++;
            
            if (canMessageTail==CAN_SEND_BUFFER_SIZE) {
              canMessageTail=0;
            }
          }
          
          xSemaphoreGive(canBusSendSemaphore);          
        } else {
          client->text("Buffer full.");
        }
      } else {
        for(size_t i=0; i < info->len; i++){
          Serial.printf("%02x ", data[i]);
        }
        Serial.printf("\r\n");
        client->binary("binary message is not supported");
      }        
    } else {
      //message is comprised of multiple frames or the frame is split into multiple packets
      if(info->index == 0){
        if(info->num == 0)
          Serial.printf("ws[%s][%u] %s-message start\r\n", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");
        Serial.printf("ws[%s][%u] frame[%u] start[%llu]\r\n", server->url(), client->id(), info->num, info->len);
      }

      Serial.printf("ws[%s][%u] frame[%u] %s[%llu - %llu]: ", server->url(), client->id(), info->num, (info->message_opcode == WS_TEXT)?"text":"binary", info->index, info->index + len);
      if(info->message_opcode == WS_TEXT){
        data[len] = 0;
        Serial.printf("%s\r\n", (char*)data);
      } else {
        for(size_t i=0; i < len; i++){
          Serial.printf("%02x ", data[i]);
        }
        Serial.printf("\r\n");
      }

      if((info->index + len) == info->len){
        Serial.printf("ws[%s][%u] frame[%u] end[%llu]\r\n", server->url(), client->id(), info->num, info->len);
        if(info->final){
          Serial.printf("ws[%s][%u] %s-message end\r\n", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");
          if(info->message_opcode == WS_TEXT)
            client->text("Multiple frames/packet is not supported.");
          else
            client->binary("binary message is not supported");
        }
      }
    }
  }
}

void setWiFiConfig(AsyncWebServerRequest *request) {
  Preferences preferences;
  preferences.begin(preference_name, false); // open in read-write mode

  bool ap_mode_en=request->getParam("ap_mode_en",true)->value()=="true";
  preferences.putBool("ap_mode_en",ap_mode_en);

  String soft_ap_pass=request->getParam("soft_ap_pass",true)->value();
  preferences.putString("soft_ap_pass",soft_ap_pass);
  
  String soft_ap_ssid=request->getParam("soft_ap_ssid",true)->value();
  preferences.putString("soft_ap_ssid",soft_ap_ssid);
  
  bool sta_mode_en=request->getParam("sta_mode_en",true)->value()=="true";
  Serial.println(preferences.putBool("sta_mode_en",sta_mode_en));

  
  String sta_ap_pass=request->getParam("sta_ap_pass",true)->value();
  preferences.putString("sta_ap_pass",sta_ap_pass);

  String sta_ap_ssid=request->getParam("sta_ap_ssid",true)->value();
  preferences.putString("sta_ap_ssid",sta_ap_ssid);

  preferences.end();

  request->send(200, "text/plain", "OK! Restarting...");
  delay(500);
  ESP.restart();
}

void setCAN2UDPConfig(AsyncWebServerRequest *request) {
  Preferences preferences;
  preferences.begin(preference_name, false); // open in read-write mode

  String ip=request->getParam("can2udp_ip",true)->value();
  strcpy(can2udp_ip,ip.c_str());
  preferences.putString("can2udp_ip",ip);

  can2udp_port=strtoul(request->getParam("can2udp_port",true)->value().c_str(),NULL,0);
  preferences.putUShort("can2udp_port",can2udp_port);
  
  can2udp_enable=request->getParam("can2udp_enable",true)->value()=="true";
  Serial.println(preferences.putBool("can2udp_enable",can2udp_enable));

  preferences.end();

  request->send(200, "text/plain", "ok");
}

void setAutoPolling(AsyncWebServerRequest *request) {
  autoPolling=request->getParam("mode",true)->value()=="on";

  if (autoPolling) {
    request->send(200, "text/plain", "Auto polling is on.");
  } else {
    request->send(200, "text/plain", "Auto polling is off.");
  }
}

void gwmAuthRequestTrig(AsyncWebServerRequest *request) {
  if (autoPolling) {
    request->send(200, "text/plain", "Auto polling is on. Not set.");
  } else {
    gwmAuthRequest=true;
    request->send(200, "text/plain", "ok.");
  }
}

void sendCANMessage(AsyncWebServerRequest *request) {
  if (xSemaphoreTake(canBusSendSemaphore,(TickType_t)10)) {
    if ((canMessageTail+1)%CAN_SEND_BUFFER_SIZE != canMessageHead) {
      // not full

      sscanf(request->getParam("message",true)->value().c_str(),"%x %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx",
            &canMessages[canMessageTail].identifier,
            &canMessages[canMessageTail].flags,
            &canMessages[canMessageTail].data_length_code,
            &canMessages[canMessageTail].data[0],
            &canMessages[canMessageTail].data[1],
            &canMessages[canMessageTail].data[2],
            &canMessages[canMessageTail].data[3],
            &canMessages[canMessageTail].data[4],
            &canMessages[canMessageTail].data[5],
            &canMessages[canMessageTail].data[6],
            &canMessages[canMessageTail].data[7]);

      canMessageTail++;
      
      if (canMessageTail==CAN_SEND_BUFFER_SIZE) {
        canMessageTail=0;
      }
    }
    
    xSemaphoreGive(canBusSendSemaphore);          
  }
  request->send(200, "text/plain", "ok");
}

void setupServer() {
  server.on("/", HTTP_GET, webMain);
#ifdef USE_OTA_WEB_UPDATER
  server.on("/firmware-update", HTTP_GET, firmwareUploadForm);
  server.on("/firmware-update", HTTP_POST, firmwareUpdateHandler, firmwareUploadingFileHandler);
  server.on("/web-update", HTTP_GET, webUploadForm);
  server.on("/web-update", HTTP_POST, webUpdateHandler, webUploadingFileHandler);
#endif
  server.on("/wifi", HTTP_GET, getWiFiConfig);
  server.on("/wifi", HTTP_POST, setWiFiConfig);

  server.on("/can2udp", HTTP_GET, getCAN2UDPConfig);
  server.on("/can2udp", HTTP_POST, setCAN2UDPConfig);
  
  server.on("/info", HTTP_GET, getInfo);
  server.on("/about", HTTP_GET, getInfoText);

#ifdef USE_LCD
  server.on("/lcd", HTTP_GET, lcdAdjust);
#endif

  server.on("/polling", HTTP_POST, setAutoPolling);

  server.on("/gwmAuth", HTTP_GET, gwmAuthRequestTrig);

  server.on("/can-send", HTTP_POST, sendCANMessage);

  server.serveStatic("/web", SPIFFS, "/web");

  server.onNotFound([](AsyncWebServerRequest *request) {
    if (request->method() == HTTP_OPTIONS) {
      request->send(200);
    } else {
      request->send(404);
    }
  });

  ws.onEvent(onWSEvent);

  server.addHandler(&ws);

  server.begin();
}

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t led_pattern=0x00000001;

void ARDUINO_ISR_ATTR onTimer(){
  static uint32_t current_led_pattern=0xffffffff;
  static uint32_t current_led_shifted_pattern=0xffffffff;
  static uint8_t current_led_pos=0;
  
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);

  if (led_pattern != current_led_pattern) {
    current_led_shifted_pattern=current_led_pattern=led_pattern;
    current_led_pos=0;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
  // It is safe to use digitalRead/Write here if you want to toggle an output
  if (current_led_pos==32) {
    current_led_pos=0;
    current_led_shifted_pattern=current_led_pattern;
  }
  digitalWrite(LED_PIN,(current_led_shifted_pattern & 0x00000001) ^ LED_INVERT);
  current_led_shifted_pattern>>=1;
  current_led_pos++;
}

void setupTimer() {
  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every 1/20 second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, 50000, true);

  // Start an alarm
  timerAlarmEnable(timer);
}

void setup() {
  pinMode(PUSH_BUTTON_PIN,INPUT);
  pinMode(LED_PIN,OUTPUT);

  setupTimer();
  
  Preferences preferences;
  preferences.begin(preference_name, true); // open in read-only mode

  Serial.begin(115200);

#ifdef USE_LCD
  initLCD(preferences.getInt("lcd_contrast",DEFAULT_LCD_CONTRAST),
        preferences.getInt("lcd_brightness",DEFAULT_LCD_BRIGHTNESS));
  strcpy(lcd_text,"STARTING...");
  updateLCDScreen();
#endif

  if(!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
#ifdef USE_LCD
    strcpy(lcd_text,"err: SPIFFS");
    updateLCDScreen();
#endif
    while (1);
  }

  canBusSendSemaphore=xSemaphoreCreateMutex();

  wifi_ap_on=preferences.getBool("ap_mode_en",DEFAULT_AP_MODE_EN);
  wifi_sta_on=preferences.getBool("sta_mode_en",DEFAULT_STA_MODE_EN);

#ifdef USE_LCD
  lcd_wifi_status=LCD_WIFI_DISABLED;
  lcd_ap_status=LCD_AP_OFF;
#endif

  if (wifi_ap_on || wifi_sta_on) {
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
    WiFi.setHostname(our_name);
    WiFi.onEvent(wiFiEvent);

    if (wifi_ap_on && wifi_sta_on) {
      WiFi.mode(WIFI_MODE_APSTA);
    } else if (wifi_ap_on) {
      WiFi.mode(WIFI_MODE_AP);
    } else {
      WiFi.mode(WIFI_MODE_STA);
    }
  
    if (wifi_ap_on) {
      char ap_ssid[128];
      char ap_pass[128];

      strcpy(ap_ssid,preferences.getString("soft_ap_ssid",default_soft_ap_ssid).c_str());
      strcpy(ap_pass,preferences.getString("soft_ap_pass",default_soft_ap_pass).c_str());
      //Serial.printf("ap_mode_en %s %s\r\n",ap_ssid,ap_pass);
      Serial.println("softAP mode enabled.");
      WiFi.softAP(ap_ssid, ap_pass);
    } else {
      Serial.println("ap_mode_disabled");
    }
    
    if (wifi_sta_on) {
      char ap_ssid[128];
      char ap_pass[128];

      strcpy(ap_ssid,preferences.getString("sta_ap_ssid",default_sta_ap_ssid).c_str());
      strcpy(ap_pass,preferences.getString("sta_ap_pass",default_sta_ap_pass).c_str());
      //Serial.printf("sta_mode_en %s %s\r\n",ap_ssid,ap_pass);
      Serial.println("STA mode enabled.");
      WiFi.begin(ap_ssid,ap_pass);
    } else {
      Serial.println("sta_mode_disabled");
    }
    if (!wifiUDP.begin(LOCAL_UDP_PORT)) {
      Serial.println("WiFiUDP: no sockets available");
    } else {
      Serial.println("WiFiUDP: started.");
    }

    if (!MDNS.begin(our_name)) {
      Serial.println("Error setting up MDNS responder!");
    }

    setupServer();

  } else {
#ifdef USE_LCD
    strcpy(lcd_text,"WiFi off");
#endif
  }

#ifdef USE_LCD
  updateLCDScreen();
#endif

  strcpy(can2udp_ip,preferences.getString("can2udp_ip",can2udp_ip).c_str());
  can2udp_port=preferences.getUShort("can2udp_port",can2udp_port);
  can2udp_enable=preferences.getBool("can2udp_enable",can2udp_enable);

  preferences.end();
  
  can_general_config_t g_config = {
    .mode = CAN_MODE_NORMAL,
    .tx_io = CAN_TX,
    .rx_io = CAN_RX,
    .clkout_io = ((gpio_num_t) - 1),
    .bus_off_io = ((gpio_num_t) - 1),
    .tx_queue_len = 5,
    .rx_queue_len = 5,
    .alerts_enabled = CAN_ALERT_NONE,
    .clkout_divider = 0
  };
  can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
  can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

  if (can_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("Failed to install driver");
    return;
  }

  if (can_start() != ESP_OK) {
    Serial.println("Failed to start driver");
    return;
  }

  listAllFiles();
}

 #ifdef USE_LCD

void int2String(char *output,int n,int decimal,int force_sign) {
  char tmp[10];
  char *p=tmp;
  int save_n=n;
  int negative=n<0;

  if (negative) {
    n=-n;
  }
  if (n==0) {
    for (int i=0;i<decimal;i++) {
      *(p++)='0';
    }
  } else while (n>0) {
    *(p++)='0'+n%10;
    n/=10;
  }

  while (p-tmp<=decimal) {
    *(p++)='0';
  }

  char *o=output;
  if (negative) {
    *(o++)='-';
  } else if (force_sign) {
    *(o++)='+';
  }
  
  p--;
  do {
    if (decimal>0 && p-tmp==decimal-1) {
      *(o++)='.';
    }
    *(o++)=*(p--);
  } while (p>=tmp);

  *o=0;
}

#endif

bool processButton() {
  static unsigned long buttonPressStart;
  static bool buttonPressed=false;
  bool buttonPressing=!digitalRead(PUSH_BUTTON_PIN);

  if (buttonPressing) {
    if (buttonPressed) {
      // holding
      unsigned long hold_for=millis()-buttonPressStart;
      if (hold_for>BUTTON_RESET_CONFIRM) {
        portENTER_CRITICAL(&timerMux);
        led_pattern=led_patterns[LED_STATE_RESETTING];
        portEXIT_CRITICAL(&timerMux);
        Serial.println("Resetting...");
#ifdef USE_LCD
        strcpy(lcd_text,"RESETTING...");
        updateLCDScreen();
#endif
        reset_all_data();
        return true;
      } else if (hold_for>BUTTON_RESET_WARNING) {
#ifdef USE_LCD
        portENTER_CRITICAL(&timerMux);
        led_pattern=led_patterns[LED_STATE_RESET_WARNING];
        portEXIT_CRITICAL(&timerMux);
        Serial.println("Going to reset soon...\n");
        strcpy(lcd_text,"?! RESET !?");
        updateLCDScreen();
        return true;
#endif
      }
    } else {
      // just pressing
      buttonPressed=true;
      buttonPressStart=millis();
    }
  } else {
    if (buttonPressed) {
      // releasing
      unsigned long hold_for=millis()-buttonPressStart;
      buttonPressed=false;
      if (hold_for>BUTTON_RESET_WARNING) {
        Serial.println("Reset canceled");

#ifdef USE_LCD
        if (WiFi.status()==WL_CONNECTED) {
          strcpy(lcd_text,WiFi.localIP().toString().c_str());
        }
#endif

      } else if (hold_for>BUTTON_LONG_PRESS_MILLI) {
        Serial.println("Long press");

        if (wifi_was_disconnected && wifi_sta_on) {
          WiFi.reconnect();
          wifi_was_disconnected=false;
        }

#ifdef USE_LCD
        if (WiFi.status()==WL_CONNECTED) {
          strcpy(lcd_text,WiFi.localIP().toString().c_str());
        }
#endif
      } else if (hold_for>BUTTON_SHORT_PRESS_MILLI) {
        Serial.println("Short press");
        switch (target_drl_state) {
          case DRL_AUTO:
            target_drl_state=DRL_OFF;
#ifdef USE_LCD
            strcpy(lcd_text,"DRL OFF");
#endif
          break;
          case DRL_OFF:
            target_drl_state=DRL_ON;
#ifdef USE_LCD
            strcpy(lcd_text,"DRL ON");
#endif
          break;
          case DRL_ON:
            target_drl_state=DRL_AUTO;
#ifdef USE_LCD
            strcpy(lcd_text,"DRL AUTO");
#endif
          break;
        }
      } else {
        // ignore
      }
    } else {
      // nothing special
    }
  }

 switch (target_drl_state) {
  case DRL_AUTO:
    led_pattern=led_patterns[LED_STATE_DRL_AUTO];
  break;
  case DRL_OFF:
    led_pattern=led_patterns[LED_STATE_DRL_OFF];
  break;
  case DRL_ON:
    led_pattern=led_patterns[LED_STATE_DRL_ON];
  break;
 }

 return false;
}


// taken from https://github.com/openvehicles/Open-Vehicle-Monitoring-System-3/blob/master/vehicle/OVMS.V3/components/vehicle_mgev/src/mg_auth.cpp
uint32_t umul_lsr45(uint32_t a, uint32_t b) {
    uint32_t i = a & 0xffffu;
    a >>= 16u;
    uint32_t j = b & 0xffffu;
    b >>= 16u;
    return (((((i * j) >> 16u) + (i * b + j * a)) >> 16u) + (a * b)) >> 13u;
}

uint32_t iterate(uint32_t seed, uint32_t count) {
    while (count--) {
        seed = (seed << 1u) | ((((((((seed >> 6u) ^ seed) >> 12u) ^ seed) >> 10u) ^ seed) >> 2u) & 1u);
    }
    return seed;
}

uint32_t GWMKey1(uint32_t seed) {
    uint32_t i = seed & 0xffffu;
    uint32_t i2 = 1u;
    uint32_t i3 = 0x12e5u;
    while (i3) {
        if (i3 & 1u)
        {
            uint32_t tmp = i2 * i;
            i2 = (tmp - (umul_lsr45(tmp, 0x82b87f05u) * 0x3eabu));
        }
        uint32_t tmp2 = i * i;
        i = (tmp2 - (umul_lsr45(tmp2, 0x82b87f05u) * 0x3eabu));
        i3 >>= 1u;
    }
    uint32_t i5 = ((i2 >> 8u) + i2) ^ 0x0fu;
    uint32_t i6 = (i2 ^ (i5 << 8u)) & 0xff00u;
    uint32_t i7 = ((i2 ^ i5) & 0xffu) | i6;
    return (i7 | i7 << 16u) ^ 0xad0779e2u;
}

uint32_t GWMKey2(uint32_t seed) {
    uint32_t count = 0x25u + (((seed >> 0x18u) & 0x1cu) ^ 0x08u);
    return iterate(seed, count) ^ 0xdc8fe1aeu;
}

uint32_t BCMKey(uint32_t seed) {
    uint32_t count = 0x2bu + (((seed >> 0x18u) & 0x17u) ^ 0x02u);
    return iterate(seed, count) ^ 0x594e348au;
}

void enqueueCANMessage(can_message_t *message) {
  if (xSemaphoreTake(canBusSendSemaphore,(TickType_t)10)) {
    if ((canMessageTail+1)%CAN_SEND_BUFFER_SIZE != canMessageHead) {
      // not full

      memcpy(&canMessages[canMessageTail],message,sizeof(can_message_t));

      canMessageTail++;
      
      if (canMessageTail==CAN_SEND_BUFFER_SIZE) {
        canMessageTail=0;
      }
    }
    
    xSemaphoreGive(canBusSendSemaphore);          
  }
}

void gwmAuth(can_message_t *message, unsigned long currentMillis) {
  char message_buffer[1024];
  unsigned long lastAuthAttempt=0;
  
  if (message==NULL) {
    // called by loop
    unsigned long timeout=OBD2_POLLING_DELAY_MS*(3+NUM_AUTO_POLLING_CAN_MESSAGES);
    can_message_t start_auth_message =   {.flags=0, .identifier=GWM_ID, .data_length_code=8, .data={0x02,0x10,0x01,0x00,0x00,0x00,0x00,0x00}};
    
    if (!autoPolling) {
      timeout=5*OBD2_POLLING_DELAY_MS;
      
      if (gwmAuthRequest) {
        ws.textAll("GWM auth: starting (web trigger)");
        enqueueCANMessage(&start_auth_message);
        gwmAuthRequest=false;
      }
    } else if (lastReceived!=0 && 
        (
          (lastReceived < lastPollStart && (lastReceived + timeout) > currentMillis) // New reply not received, but not timed out yet
          ||
          (lastReceived > lastPollStart) // New reply received
        )
       )  {
      // Imply that the auth is not needed now
    } else if ((gwmAuthRequest || autoPolling) && lastPollStart!=0 && currentMillis > (lastReceived+timeout)) {
      // no reply?
      if (lastStartAuth==0 || currentMillis-lastStartAuth > GWM_AUTH_TIMEOUT) {
        ws.textAll("GWM auth: starting (auto polling no response)");
        enqueueCANMessage(&start_auth_message);
        lastStartAuth=currentMillis;
        gwmAuthRequest=false;
      }
    }
  } else {
    // called by decodeCAN
    if (message->data[0]==0x06 && (message->data[1] & 0xbf) == 0x10 && message->data[2]==0x01) {
      ws.textAll("GWM auth: sending 1003");
      can_message_t start_gwm_auth_message = {.flags=0, .identifier=GWM_ID, .data_length_code=8, .data={0x02,0x10,0x03,0x00,0x00,0x00,0x00,0x00}};
      enqueueCANMessage(&start_gwm_auth_message);
    } else if (message->data[0]==0x06 && (message->data[1] & 0xbf) == 0x10 && message->data[2]==0x03) {
      ws.textAll("GWM auth: Request seed1");
      can_message_t request_seed1 = {.flags=0, .identifier=GWM_ID, .data_length_code=8, .data={0x06,0x27,0x41,0x3e,0xab,0x00,0x0d,0x00}};
      enqueueCANMessage(&request_seed1);
    } else if (message->data[0]==0x06 && (message->data[1] & 0xbf) == 0x27 && message->data[2]==0x41) {
      // Seed1 response
      uint32_t seed = (message->data[3] << 24) | (message->data[4] << 16) | (message->data[5] << 8) | message->data[6];
      uint32_t key = GWMKey1(seed);
      sprintf(message_buffer,"GWM auth: seed1 received %08x. Replying with key1 %08x", seed, key);
      ws.textAll(message_buffer);
      can_message_t seed1_response = {.flags=0, .identifier=GWM_ID, .data_length_code=8, .data={0x06,0x27,0x42,key >> 24u,key >> 16u,key >> 8u,key,0x00}};
      enqueueCANMessage(&seed1_response);
    } else if (message->data[0]==0x02 && (message->data[1] & 0xbf) == 0x27 && message->data[2]==0x42) {
      ws.textAll("GWM: key1 accepted, requesting seed2");
      can_message_t request_seed2 = {.flags=0, .identifier=GWM_ID, .data_length_code=8, .data={0x02,0x27,0x01,0x00,0x00,0x00,0x00,0x00}};
      enqueueCANMessage(&request_seed2);
    } else if (message->data[0]==0x06 && (message->data[1] & 0xbf) == 0x27 && message->data[2]==0x01) {
      uint32_t seed = (message->data[3] << 24) | (message->data[4] << 16) | (message->data[5] << 8) | message->data[6];
      uint32_t key = GWMKey2(seed);
      sprintf(message_buffer,"GWM auth: seed2 received %08x. Replying with key2 %08x", seed, key);
      ws.textAll(message_buffer);
      can_message_t request_seed2 = {.flags=0, .identifier=GWM_ID, .data_length_code=8, .data={0x06,0x27,0x02,key >> 24u,key >> 16u,key >> 8u,key,0x00}};
      enqueueCANMessage(&request_seed2);
    } else if (message->data[0]==0x02 && (message->data[1] & 0xbf) == 0x27 && message->data[2]==0x02) {
      ws.textAll("GWM auth: key2 accepted, starting routine");
      can_message_t request_seed2 = {.flags=0, .identifier=GWM_ID, .data_length_code=8, .data={0x05,0x31,0x01,0xaa,0xff,0x00,0x00,0x00}};
      enqueueCANMessage(&request_seed2);
    } else if (message->data[0]==0x04 && (message->data[1] & 0xbf) == 0x31 && message->data[2]==0x01) {
      ws.textAll("GWM auth: Routine started, request routine control");
      can_message_t request_seed2 = {.flags=0, .identifier=GWM_ID, .data_length_code=8, .data={0x04,0x31,0x03,0xaa,0xff,0x00,0x00,0x00}};
      enqueueCANMessage(&request_seed2);
    } else if (message->data[0]==0x05 && (message->data[1] & 0xbf) == 0x31 && message->data[2]==0x03) {
      ws.textAll("GWM auth: Requested routine control. Gateway authentication complete.");
    }
  }
}

void startDrlControl(unsigned long currentMillis, DRL_CONTROLS target) {
  ws.textAll("BCM drl: first frame");
  can_message_t drl_first_frame = {.flags=0, .identifier=BCM_ID, .data_length_code=8, .data={0x10,0x0c,0x2f,0xd1,0x17,0x03,target==DRL_TURN_ON?0x04:0x00,0x00}};
  enqueueCANMessage(&drl_first_frame);
}

void bcm(can_message_t *message, unsigned long currentMillis,DRL_CONTROLS drlCtrl) {
  char message_buffer[1024];
  static DRL_CONTROLS internalTarget=DRL_DO_NOTHING;
  static unsigned long lastSentTESTERPRESENT = 0;
  static unsigned long pendingFrame = 0;
  static unsigned long firstFrameSent = 0;
  static char drl_response[5];
  static bool sendTesterPresent=false;

  if (message==NULL) {
    // called from resolveDRLState() or loop()
    
    if (drlCtrl==DRL_DO_NOTHING) {
      // called from loop()
      if (pendingFrame!=0 && pendingFrame < currentMillis) {
        ws.textAll("BCM drl: next frame");
        pendingFrame=0;
        can_message_t next_frame = {.flags=0, .identifier=BCM_ID, .data_length_code=8, .data={0x21,0x00,0x00,0x04,0x00,0x00,0x00,0x00}};
        enqueueCANMessage(&next_frame);
      }
      
      if (sendTesterPresent && (currentMillis - lastSentTESTERPRESENT >= TESTERPRESENT_DELAY_MS)) {
        can_message_t tester_present = {.flags=0, .identifier=BCM_ID, .data_length_code=8, .data={0x02,0x3e,0x00,0x00,0x00,0x00,0x00,0x00}};
        enqueueCANMessage(&tester_present);
        lastSentTESTERPRESENT=currentMillis;
      }
    } else {
      // called from resolveDRLState()
      sprintf(message_buffer,"BCM auth: start. drlCtrl=%d",drlCtrl);
      ws.textAll(message_buffer);
      internalTarget=drlCtrl;
      can_message_t authStart = {.flags=0, .identifier=BCM_ID, .data_length_code=8, .data={0x02,0x10,0x03,0x00,0x00,0x00,0x00,0x00}};
      enqueueCANMessage(&authStart);
      sendTesterPresent=false; // will set back on later if DTRL_TURN_OFF
    }
  } else {
    // called from decodeCAN()
    if (message->data[0]==0x06 && (message->data[1] & 0xbf) == 0x10 && message->data[2]==0x03) {
      ws.textAll("BCM auth: TESTERPRESENT & requesting seed");
      
      can_message_t tester_present = {.flags=0, .identifier=BCM_ID, .data_length_code=8, .data={0x02,0x3e,0x00,0x00,0x00,0x00,0x00,0x00}};
      enqueueCANMessage(&tester_present);     
      can_message_t requesting_seed = {.flags=0, .identifier=BCM_ID, .data_length_code=8, .data={0x02,0x27,0x01,0x00,0x00,0x00,0x00,0x00}};
      enqueueCANMessage(&requesting_seed);
    } else if (message->data[0]==0x06 && (message->data[1] & 0xbf) == 0x27 && message->data[2]==0x01) {
      uint32_t seed = (message->data[3] << 24) | (message->data[4] << 16) | (message->data[5] << 8) | message->data[6];

      if (seed==0) {
        sprintf(message_buffer,"BCM auth: seed received %08x. BCM seems to already have been authenticated", seed);
        ws.textAll(message_buffer);
        startDrlControl(currentMillis,internalTarget);
        firstFrameSent=currentMillis; // just approximate it :p
      } else {
        uint32_t key = BCMKey(seed);
        sprintf(message_buffer,"BCM auth: seed received %08x. Replying with key %08x", seed, key);
        ws.textAll(message_buffer);
        can_message_t seed1_response = {.flags=0, .identifier=BCM_ID, .data_length_code=8, .data={0x06,0x27,0x02,key >> 24u,key >> 16u,key >> 8u,key,0x00}};
        enqueueCANMessage(&seed1_response);
      } 
    } else if (message->data[0]==0x02 && (message->data[1] & 0xbf) == 0x27 && message->data[2]==0x02) {
        ws.textAll("BCM auth: key accepted, authentication complete");
        startDrlControl(currentMillis,internalTarget);
        firstFrameSent=currentMillis; // just approximate it :p
    } else if (message->data[0]==0x30 && message->data[1] == 0x08) {
        uint8_t desired_separation_time = message->data[2];
        pendingFrame=firstFrameSent + desired_separation_time;
        sprintf(message_buffer,"BCM drl: scheduled next frame at %Lu (first frame %Lu, now %Lu)", pendingFrame, firstFrameSent, currentMillis);
        ws.textAll(message_buffer);
    } else if (message->data[0]==0x10 && message->data[1] == 0x08 && message->data[2] == 0x6f) {
        ws.textAll("BCM drl: Send flow control and save the first half of the response");
        drl_response[0]=message->data[4];
        drl_response[1]=message->data[5];
        drl_response[2]=message->data[6];
        can_message_t flow_control = {.flags=0, .identifier=BCM_ID, .data_length_code=8, .data={0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00}};
        enqueueCANMessage(&flow_control);
    } else if (message->data[0]==0x21) {
        drl_response[3]=message->data[1];
        drl_response[4]=message->data[2];

        sprintf(message_buffer,"BCM drl: second multi-frame response received. Total DRL response = %02x %02x %02x %02x %02x",
                  drl_response[0],drl_response[1],drl_response[2],drl_response[3],drl_response[4]);
        ws.textAll(message_buffer);

        // Only send TESTERPRESENT when turning DRL_TURN_OFF
        sendTesterPresent = (internalTarget == DRL_TURN_OFF);

        // make sure that nothing left?
        can_message_t flow_control = {.flags=0, .identifier=BCM_ID, .data_length_code=8, .data={0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00}};
        enqueueCANMessage(&flow_control);
    }
  }
}

void decodeCAN(can_message_t *message, unsigned long currentMillis) {
  if (message->data_length_code!=8) return;
  if (message->flags!=0) return;

  uint8_t *data=message->data;

  switch (message->identifier) {
    case 0x718:
      gwmAuth(message,currentMillis);
      break;

    case 0x748:
      bcm(message,currentMillis,DRL_DO_NOTHING);
      break;
      
    case 0x7ed: // BMS
      lastReceived=currentMillis;
      
      switch (data[0]<<24 | data[1]<<16 | data[2]<<8 | data[3]) {
        case 0x0562b061: // SoH
          hvb_SoH=0.01*(data[4]<<8|data[5]);
#ifdef USE_LCD
          int2String(lcd_hvb_SoH,data[4]<<8|data[5],2,0);
#endif
          break;
        case 0x0562b046: // SoC
          hvb_SoC=0.1*(data[4]<<8|data[5]);
#ifdef USE_LCD
          int2String(lcd_hvb_SoC,data[4]<<8|data[5],1,0);
#endif
          break;
        case 0x0462b048: // Charging status
          charging_status=data[4];
          switch (charging_status) {
            case  0: strcpy(charging_status_text,"connected unlocked"); break;
            case  1: strcpy(charging_status_text,"idle"); break;
            case  3: strcpy(charging_status_text,"running"); break;
            case  6: strcpy(charging_status_text,"charging"); break;
            case  7: strcpy(charging_status_text,"rapid charging"); break;
            case  8: strcpy(charging_status_text,"sleep"); break;
            case 10: strcpy(charging_status_text,"connected"); break;
            case 12: strcpy(charging_status_text,"charge starting"); break;
            default:
              strcpy(charging_status_text,"unknown");
          }
          break;
        case 0x0462b05c: // Batt temp
          hvb_coolant_temp=0.1*(data[4]*5-400);
#ifdef USE_LCD
          int2String(lcd_hvb_coolant_temp,data[4]*5-400,1,0);
#endif
          break;
        case 0x0562b042: // Batt voltage
          hvb_v=0.25*((data[4]<<8|data[5]));
#ifdef USE_LCD
          int2String(lcd_hvb_v,(data[4]<<8|data[5])*100/4,2,0);
          sprintf(lcd_hvb_w,"%.2f",hvb_v*hvb_a*0.001);
#endif
          break;
        case 0x0562b043: // Batt current
          hvb_a=0.025*((data[4]<<8|data[5])-40000);
#ifdef USE_LCD
          int2String(lcd_hvb_a,((data[4]<<8|data[5])-40000)*100/4,3,0);
          sprintf(lcd_hvb_w,"%.2f",hvb_v*hvb_a*0.001);
#endif
          break;
      };
      break;

    case 0x7eb: // VCU
      lastReceived=currentMillis;

      switch (data[0]<<24 | data[1]<<16 | data[2]<<8 | data[3]) {
        case 0x0462b900: // Gear
          switch (data[4]) {
            case 0x07: gear_pos='R'; break;
            case 0x06: gear_pos='N'; break;
            case 0x05: gear_pos='D'; break;
            case 0x08: gear_pos='P'; break;
            default: gear_pos=' ';
          }
#ifdef USE_LCD          
          lcd_gear_pos[0]=gear_pos;
          lcd_gear_pos[1]=0;
#endif
          break;

        case 0x0562b583: // DC-DC A
          dc_dc_a=0.1*(data[4]<<8|data[5]);
#ifdef USE_LCD          
          int2String(lcd_dc_dc_a,data[4]<<8|data[5],1,0);
          sprintf(lcd_dc_dc_w,"%.0f",dc_dc_v*dc_dc_a);
#endif
          break;

        case 0x0562b584: // DC-DC V
          dc_dc_v=0.1*(data[4]<<8|data[5]);
#ifdef USE_LCD          
          int2String(lcd_dc_dc_v,data[4]<<8|data[5],1,0);
          sprintf(lcd_dc_dc_w,"%.0f",dc_dc_v*dc_dc_a);
#endif
          break;

        case 0x0462b587: // DC-DC T
          dc_dc_temp=data[4]-40.0;
#ifdef USE_LCD          
          int2String(lcd_dc_dc_temp,data[4]-40,0,0);
#endif
          break;

        case 0x0462b309: // Motor coolant T
          motor_coolant_temp=data[4]-40.0;
#ifdef USE_LCD          
          sprintf(lcd_motor_temp,"%.0f",(motor_coolant_temp>motor_temp)?motor_coolant_temp:motor_temp);
#endif
          break;
          
        case 0x0462b405: // Motor T
          motor_temp=data[4]-40.0;
#ifdef USE_LCD          
          sprintf(lcd_motor_temp,"%.0f",(motor_coolant_temp>motor_temp)?motor_coolant_temp:motor_temp);
#endif
          break;

        case 0x0662e101: // Odometer
          odometer=data[4]<<16|data[5]<<8<<data[6]; 
          break;
      }
      break;

    default: return;
  }
}

void can2str(can_message_t *message,char *message_buffer) {
    sprintf(message_buffer,"%x\t%x %x ",message->identifier,message->flags,message->data_length_code);
  
    if (message->data_length_code) {
      char *p=message_buffer+strlen(message_buffer);

      for (int i=0;i<message->data_length_code;i++) {
        sprintf(p,"%02x ",message->data[i]);
        p+=3;
      }
      *p=0;
    }
}

void resolveDRLState(unsigned long currentMillis) {
  char message_buffer[256];

  if (previous_target_drl_state!=target_drl_state) {
    // User set new DRL state
    sprintf(message_buffer,"DBG: Target drl %d to %d",previous_target_drl_state,target_drl_state);
    ws.textAll(message_buffer);
    
    switch (target_drl_state) {
      case DRL_AUTO:
        if (gear_pos=='P') {
          drl_control=DRL_TURN_OFF;
          ws.textAll("DBG: DRL_TURN_OFF");
        } else {
          if (drl_control==DRL_TURN_OFF) {
            ws.textAll("DBG: DRL_TURN_ON");
            drl_control=DRL_TURN_ON;
          } else {
            ws.textAll("DBG: DRL_DO_NOTHING");
            drl_control=DRL_DO_NOTHING;
          }
        }
        break;
      case DRL_OFF:
        ws.textAll("DBG: DRL_TURN_OFF");
        drl_control=DRL_TURN_OFF;
        break;
      case DRL_ON:
        ws.textAll("DBG: DRL_TURN_ON");
        drl_control=DRL_TURN_ON;
        break;
    }
    previous_target_drl_state=target_drl_state;
  }

  if (previous_gear_pos!=gear_pos) {
    // Gear changed
    sprintf(message_buffer,"DBG: Gear change from %c to %c",previous_gear_pos,gear_pos);
    ws.textAll(message_buffer);
    
    if (target_drl_state==DRL_AUTO) {
      if (gear_pos=='P') {
        ws.textAll("DBG: DRL_TURN_OFF");
        drl_control=DRL_TURN_OFF;
      } else {
        ws.textAll("DBG: DRL_TURN_ON");
        drl_control=DRL_TURN_ON;
      }
    }
    previous_gear_pos=gear_pos;
  }

  if (drl_control!=previous_drl_control) {
    if (drl_control!=DRL_DO_NOTHING) {
      ws.textAll("DBG: Setting DRL");
      bcm(NULL,currentMillis,drl_control);
    }
    previous_drl_control=drl_control;
  }
}

void loop() {
  unsigned long currentMillis = millis();

  if (processButton()) return; // do nothing else if the user want to reset

  ws.cleanupClients();
  
  char message_buffer[256];

  gwmAuth(NULL,currentMillis);
  bcm(NULL,currentMillis,DRL_DO_NOTHING);
  
  can_message_t message;
  
  if (can_receive(&message, pdMS_TO_TICKS(50)) == ESP_OK) {
    can2str(&message,message_buffer);
    decodeCAN(&message,currentMillis);

    if (can2udp_enable && can2udp_ip[0]!=0 && can2udp_port!=0 && WiFi.status()==WL_CONNECTED) {
      wifiUDP.beginPacket(can2udp_ip,can2udp_port);
      wifiUDP.write((const uint8_t *)message_buffer,strlen(message_buffer));
      wifiUDP.endPacket();
    }

    Serial.printf("send %s\r\n",message_buffer);
    ws.textAll(message_buffer);
  }

  resolveDRLState(currentMillis);

  if (xSemaphoreTake(canBusSendSemaphore,(TickType_t)10)) {
    while (canMessageHead!=canMessageTail) {
        if (can_transmit(&canMessages[canMessageHead], pdMS_TO_TICKS(50)) == ESP_OK) {
          // Good
          can2str(&canMessages[canMessageHead],message_buffer);
          strcpy(message_buffer+strlen(message_buffer)," (S)");
          ws.textAll(message_buffer);
        } else {
          ws.textAll("DBG: Failed to queue message for transmission");
        }

        canMessageHead++;
        
        if (canMessageHead==CAN_SEND_BUFFER_SIZE) {
          canMessageHead=0;
        }
    }

    xSemaphoreGive(canBusSendSemaphore);
  }
      
#ifdef USE_LCD
  if (wifi_sta_on) {
    if (WiFi.status()==WL_CONNECTED) {
      if (WiFi.RSSI()>-60) {
        lcd_wifi_status=LCD_WIFI_CONNECTED_HIGH;
      } else {
        lcd_wifi_status=LCD_WIFI_CONNECTED_LOW;
      }
    } else {
      lcd_wifi_status=LCD_WIFI_DISCONNECTED;
    }
  }

  updateLCDScreen();
#endif

  // if WiFi is down, try reconnecting
  if (wifi_sta_on && (WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillisWifiRetry >=WIFI_AUTO_RECONNECT_INTERVAL_MS)) {
    Serial.println("Try reconecting WiFi.");
    wifi_was_disconnected=false;
    WiFi.reconnect();
    previousMillisWifiRetry = currentMillis;
  }

  if (autoPolling && (currentMillis - previousMillisOBD2Polling >= OBD2_POLLING_DELAY_MS)) {
    enqueueCANMessage(&auto_polling_messages[pollingHead]);
    if (pollingHead==0) {
      lastPollStart=currentMillis;
    }
    pollingHead++;

    if (pollingHead==NUM_AUTO_POLLING_CAN_MESSAGES) {
      pollingHead=0;
    }

    previousMillisOBD2Polling = currentMillis;
  }
  
  yield();
}
