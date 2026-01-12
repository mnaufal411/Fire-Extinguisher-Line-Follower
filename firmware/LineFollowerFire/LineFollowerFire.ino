#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

//  WIFI & MQTT 
const char* WIFI_SSID = "Pribadi"; // adjust with your WiFi SSID 
const char* WIFI_PASS = "12345677"; // adjust with your WiFi Password

const char* MQTT_BROKER = "172.20.10.2";
const int   MQTT_PORT   = 1884; // adjust with your MQTT Port
const char* MQTT_TOPIC  = "robot/fire"; // adjust with your MQTT Topic

WiFiClient espClient;
PubSubClient mqtt(espClient);

// FIRE STATUS
volatile bool fireExists = false;  // Fire status from MQTT

// MISSION FLAGS
enum MissionPhase {
  PHASE_IDLE,              // waiting for orders
  PHASE_CHECK_FIRE,        // Check the fire status at the beginning
  PHASE_GOING_TO_FIRE,     // Heading towards the fire (status: there is fire)
  PHASE_EXTINGUISHING,     // Extiguishing the fire (00000 detected)
  PHASE_WAIT_EXTINGUISH,   // Wait for confirmation that the fire is out
  PHASE_RETURNING,         // Return to base (status: no fire)
  PHASE_COMPLETED          // Mission completed, wait for the fire status to start again
};

MissionPhase missionPhase = PHASE_IDLE;
int lastKnownDirection = 0;
bool missionActive = false;

// LINE SENSOR 
const int sensorPin[5] = {34, 35, 32, 22, 23}; 

// MOTOR + TB6612 
#define PWMA 25
#define AIN1 26
#define AIN2 27
#define PWMB 33
#define BIN1 14
#define BIN2 12
#define STBY 13
#define FLAME_PIN 39

// FAN MODULE 
#define FAN_INA 4
#define FAN_INB 2
#define FAN_PWM 15

// SPEED & TUNING 
#define BASE_SPEED 110          
#define MAX_SPEED  255
#define TURN_SPEED 130          
#define HARD_TURN_SPEED 200     

#define PRE_TURN_TIME 80        
#define SHARP_CONFIRM_TIME 40   
#define LOOP_DELAY_MS 6

#define STEER_GAIN 18.0f        
#define EDGE_BOOST 1.3f         
#define POS_ALPHA  0.35f        
#define DEADZONE   0.12f        

// LINE FOLLOWING STATE 
enum LineState {
  LINE_NORMAL,
  LINE_PRE_TURN,
  LINE_ROTATING,
  LINE_RECOVERY
};

LineState lineState = LINE_NORMAL;
int turnDir = 0;
unsigned long stateTimer = 0;
bool sharpCandidate = false;
unsigned long sharpTimer = 0;

float posFiltered = 0.0f;
const float weights[5] = {-1.2f, -0.6f, 0.0f, 0.6f, 1.2f}; 

void moderateTurnLeft() { motor(1, BASE_SPEED/2, 1, BASE_SPEED); }
void moderateTurnRight() { motor(1, BASE_SPEED, 1, BASE_SPEED/2); }

// MOTOR DIRECTION
void motor(bool lDir, int lSpd, bool rDir, int rSpd) {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, lDir);
  digitalWrite(AIN2, !lDir);
  digitalWrite(BIN1, rDir);
  digitalWrite(BIN2, !rDir);
  analogWrite(PWMA, constrain(lSpd, 0, 255));
  analogWrite(PWMB, constrain(rSpd, 0, 255));
}

void stopBot() { motor(0, 0, 0, 0); }
void forward(int s) { motor(1, s, 1, s); }
void backward(int s) { motor(0, s, 0, s); }
void rotateLeft()  { motor(0, TURN_SPEED, 1, TURN_SPEED); }
void rotateRight() { motor(1, TURN_SPEED, 0, TURN_SPEED); }

void extremeTurnLeft() { motor(0, HARD_TURN_SPEED, 1, HARD_TURN_SPEED); }  
void extremeTurnRight() { motor(1, HARD_TURN_SPEED, 0, HARD_TURN_SPEED); } 

void retreatAndTurn180() {
  backward(BASE_SPEED); 
  delay(350);
  motor(1, TURN_SPEED, 0, TURN_SPEED);
  delay(520); 
  stopBot();
  delay(100);
}

// FAN CONDITION
void fanOn() {
  digitalWrite(FAN_INA, HIGH);
  digitalWrite(FAN_INB, LOW);
  analogWrite(FAN_PWM, 255);
}
void fanOff() {
  digitalWrite(FAN_INA, LOW);
  digitalWrite(FAN_INB, LOW);
  analogWrite(FAN_PWM, 0);
}

// SENSOR HELPER 
inline bool isLine(int v) { return v == 0; }

bool computePosition(int s[5], float &pos) {
  float sum = 0.0f;
  int cnt = 0;
  for (int i = 0; i < 5; i++) {
    if (isLine(s[i])) {
      sum += weights[i];
      cnt++;
    }
  }
  if (cnt == 0) return false;
  pos = sum / cnt;
  
  if (pos < -0.3) lastKnownDirection = -1;
  else if (pos > 0.3) lastKnownDirection = 1;
  else lastKnownDirection = 0;
  
  return true;
}

// STEERING
void applySteering(float pos, float boost = 1.0f) {
  if (abs(pos) < DEADZONE) {
    forward(BASE_SPEED);
    return;
  }
  float corr = pos * STEER_GAIN * boost;
  int L = BASE_SPEED + corr;
  int R = BASE_SPEED - corr;
  motor(1, constrain(L,0,255), 1, constrain(R,0,255));
}

// MQTT
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String data;
  for (uint8_t i = 0; i < length; i++) data += (char)payload[i];

  if (data == "ADA_API") {
    fireExists = true;
    
    // If it is IDLE or COMPLETED, start a new mission
    if (missionPhase == PHASE_IDLE || missionPhase == PHASE_COMPLETED) {
      missionPhase = PHASE_CHECK_FIRE;
      missionActive = true;
      lineState = LINE_NORMAL;
    }
  } 
  else if (data == "TIDAK_ADA_API") {
    fireExists = false;
  }
}

void mqttReconnect() {
  while (!mqtt.connected()) {
    if (mqtt.connect("ESP32_LINE_FIRE")) {
      mqtt.subscribe(MQTT_TOPIC);
    } else {
      Serial.print(" failed, rc=");
      Serial.println(mqtt.state());
      delay(1000);
    }
  }
}

// LINE FOLLOWING LOGIC
void doLineFollowing(int s[5], bool allBlack, bool allWhite) {
  bool edgeLeft  = (s[0]==1 && s[1]==0 && s[2]==0 && s[3]==0 && s[4]==0);
  bool edgeRight = (s[0]==0 && s[1]==0 && s[2]==0 && s[3]==0 && s[4]==1);

  bool leftMid    = (s[0]==1 && s[1]==0 && s[2]==1 && s[3]==1 && s[4]==1);
  bool leftOuter  = (s[0]==0 && s[1]==1 && s[2]==1 && s[3]==1 && s[4]==1);
  bool leftExtreme= (s[0]==0 && s[1]==0 && s[2]==1 && s[3]==1 && s[4]==1);

  bool rightMid    = (s[0]==1 && s[1]==1 && s[2]==1 && s[3]==0 && s[4]==1);
  bool rightOuter  = (s[0]==1 && s[1]==1 && s[2]==1 && s[3]==1 && s[4]==0);
  bool rightExtreme = (s[0]==1 && s[1]==1 && s[2]==0 && s[3]==0 && s[4]==0) ||
                      (s[0]==1 && s[1]==1 && s[2]==1 && s[3]==0 && s[4]==0);

  bool sharpLeft  = (s[0]==0 && s[1]==0);
  bool sharpRight = (s[3]==0 && s[4]==0);
  bool flameDetected =digitalRead(FLAME_PIN) == LOW;

  if (allWhite && !flameDetected && lineState != LINE_RECOVERY) {
    lineState = LINE_RECOVERY;
    stateTimer = millis();
    turnDir = lastKnownDirection != 0 ? lastKnownDirection : 1;
  }

  switch (lineState) {
    case LINE_NORMAL:
      if (allWhite) { 
        stopBot(); 
        break; 
      }

      if (leftExtreme) {
        extremeTurnLeft(); 
        break;
      }
      if (rightExtreme) {
        extremeTurnRight();
        break;
      }
      if (leftMid) {
        moderateTurnLeft();
        break;
      }
      if (rightMid) {
        moderateTurnRight();
        break;
      }

      if (sharpLeft || sharpRight) {
        if (!sharpCandidate) {
          sharpCandidate = true;
          sharpTimer = millis();
        }
        float p;
        if (computePosition(s, p)) {
          posFiltered = POS_ALPHA * p + (1 - POS_ALPHA) * posFiltered;
          applySteering(posFiltered, 0.6f);
        }

        if (millis() - sharpTimer > SHARP_CONFIRM_TIME) {
          turnDir = sharpLeft ? -1 : 1;
          lineState = LINE_PRE_TURN;
          stateTimer = millis();
          sharpCandidate = false;
        }
      } else {
        sharpCandidate = false;
        float p;
        if (!computePosition(s, p)) stopBot();
        else {
          posFiltered = POS_ALPHA * p + (1 - POS_ALPHA) * posFiltered;
          if (edgeLeft || edgeRight) applySteering(posFiltered, EDGE_BOOST);
          else applySteering(posFiltered, 1.0f);
        }
      }
      break;

    case LINE_PRE_TURN:
      if (millis() - stateTimer < PRE_TURN_TIME) forward(BASE_SPEED + 20);
      else lineState = LINE_ROTATING;
      break;

    case LINE_ROTATING:
      if (turnDir == -1) rotateLeft();
      else rotateRight();

      if (isLine(s[2]) || isLine(s[1]) || isLine(s[3])) {
        stopBot();
        delay(50); 
        lineState = LINE_NORMAL;
      }
      break;

    case LINE_RECOVERY:
      if (isLine(s[0]) || isLine(s[1]) || isLine(s[2]) || isLine(s[3]) || isLine(s[4])) {
        lineState = LINE_NORMAL;
        break;
      }

      unsigned long recoveryTime = millis() - stateTimer;

      if (recoveryTime < 400) {
        if (turnDir == -1) rotateLeft();
        else rotateRight();
      } 
      else if (recoveryTime < 1200) {
        if (turnDir == -1) rotateRight();
        else rotateLeft();
      }
      else {
        stopBot();
        delay(500);
        stateTimer = millis();
      }
      break;
  }
}

// SETUP
void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected!");
  
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  for (int i = 0; i < 5; i++) pinMode(sensorPin[i], INPUT);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(FAN_INA, OUTPUT); pinMode(FAN_INB, OUTPUT);
  pinMode(FAN_PWM, OUTPUT);
  pinMode(FLAME_PIN, INPUT);
  fanOff();
  stopBot();
  
}

// MAIN LOOP 
void loop() {
  if (!mqtt.connected() && WiFi.status() == WL_CONNECTED) mqttReconnect();
  mqtt.loop();

  int s[5];
  for (int i = 0; i < 5; i++) s[i] = digitalRead(sensorPin[i]);

  bool allBlack = (!s[0] && !s[1] && !s[2] && !s[3] && !s[4]);
  bool allWhite = (s[0] && s[1] && s[2] && s[3] && s[4]);

  // MISSION STATE MACHINE
  switch (missionPhase) {
    
    case PHASE_IDLE: {
      stopBot();
      fanOff();
      break;
    }

    case PHASE_CHECK_FIRE: {
      stopBot();
      delay(500);
      
      if (fireExists) {
        // There is a fire → Continue to the location
        missionPhase = PHASE_GOING_TO_FIRE;
        lineState = LINE_NORMAL;
      } else {
        // No fire → Direct rewind
        retreatAndTurn180();
        missionPhase = PHASE_COMPLETED;
        missionActive = false;
      }
      break;
    }

    case PHASE_GOING_TO_FIRE: {
      bool flameDetected = digitalRead(FLAME_PIN) == LOW;
      fanOff();
      
      if (allWhite && flameDetected) {
        // Arrived at the fire location (00000 sensor detected)
        stopBot();
        fanOn();
        missionPhase = PHASE_WAIT_EXTINGUISH;
        delay(100);
      } else {
        doLineFollowing(s, allBlack, allWhite);
      }
      break;
    }

    case PHASE_WAIT_EXTINGUISH: {
      stopBot();
      fanOn();
      // Wait for the fire status to change
      if (!fireExists) {
        // Fire out → Back up and turn around
        fanOff();
        retreatAndTurn180();
        missionPhase = PHASE_RETURNING;
        lineState = LINE_NORMAL;
      }
      break;
    }

    case PHASE_RETURNING: {
      fanOff();
      // Check if it has reached the base
      if (allBlack) {
        // To base (00000 allBlack) - ROTATE 180° first
        stopBot();
        delay(100);        
        // Rotate 180 degrees to the starting position
        motor(1, TURN_SPEED, 0, TURN_SPEED);
        delay(520); 
        stopBot();
        delay(100);
        
        // LOCK ke COMPLETED - no further path
        missionPhase = PHASE_COMPLETED;
        missionActive = false;
        
        // Turn everything off
        stopBot();
        fanOff();
        
        // Break and don't execute anything anymore
        break;
      }
      
      // Check fire status only if it has not reached base
      if (fireExists) {
        // Fire appears again! → Rewind to 00000
        stopBot();
        retreatAndTurn180();
        missionPhase = PHASE_GOING_TO_FIRE;
        lineState = LINE_NORMAL;
        break;
      }
      
      // Line following is normal if it has not reached the base and there is no fire
      doLineFollowing(s, allBlack, allWhite);
      break;
    }

    case PHASE_COMPLETED: {
      // HARD LOCK - Robot stops completely, no logic whatsoever
      stopBot();
      fanOff();
      
      // Don't execute anything, just return from switch.
      return;  // Exit loop() completely
    }
  }

  delay(LOOP_DELAY_MS);
}