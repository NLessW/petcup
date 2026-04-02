/**
 * PETMON Firmware Refactored
 * Version: 2.0 (Refactored from 1.2)
 * 
 * Target: Arduino Mega / Standard Arduino Environment
 * Description: Modular Object-Oriented Firmware for PetMon Machine
 */
/**
 * PETMON Firmware Refactored
 * Version: 2.0 (Refactored from 1.2)
 * 
 * Target: Arduino Mega / Standard Arduino Environment
 * Description: Modular Object-Oriented Firmware for PetMon Machine
 */

/*
 * Copyright (c) 2026 (주)리한 (ReHAN Co., LTD.)
 * All rights reserved.
 *
 * 이 소프트웨어와 관련 문서의 저작권은 (주)리한에 있으며,
 * 저작권자의 서면 동의 없이 무단으로 복제, 배포, 수정, 전송할 수 없습니다.
 * 
 * This software is the confidential and proprietary information of [ReHAN Co. LTD.].
 * You shall not disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered into with [ReHAN Co. LTD.].
 */

 #include <Arduino.h>
 #include <EEPROM.h>

 namespace Pin {
    // 12V DC Motor Pins
    constexpr int MOTOR_12V_ENB = 11;
    constexpr int MOTOR_12V_IN3 = 9;
    constexpr int MOTOR_12V_IN4 = 10;

    // RS-485 Control Pins
    constexpr int RS485_DE = 2;  // Driver Enable
    constexpr int RS485_RE = 3;  // Receiver Enable (Active Low)

    // SENSORS
    constexpr int SENSOR_DOOR_OPEN = 36;
    constexpr int SENSOR_DOOR_CLOSE = 37;
 }

 namespace EEPROM_Addr {
    constexpr int SPEED_DO = 0; // DOOR OPEN
    constexpr int SPEED_DC = 1; // DOOR CLOSE
 }

 namespace Defaults {
    constexpr int SPEED_DO = 200; // Default Speed for Door Open
    constexpr int SPEED_DC = 200; // Default Speed for Door Close
    constexpr int DEVICE_ID = 1;  // RS-485 Device ID
    constexpr long BAUD_RATE = 9600; // RS-485 Communication Speed
 }

 namespace SystemState {
    int speed_DO = Defaults::SPEED_DO; // Speed for Door Open
    int speed_DC = Defaults::SPEED_DC; // Speed for Door Close
 }

 // 모터드라이버 추상화 클래스
class MotorDriver {
private:
    int enPin;
    int inPin1;
    int inPin2;
public:
    // 생성자, 모터드라이버 핀 설정
    MotorDriver(int en, int in1, int in2) : enPin(en), inPin1(in1), inPin2(in2) {}
    // 모터 초기화, 핀 모드 설정
    void init() {
        pinMode(enPin, OUTPUT);
        pinMode(inPin1, OUTPUT);
        pinMode(inPin2, OUTPUT);
        stop(); // 초기 상태는 정지
    }
    // 모터 정지
    void stop() {
        digitalWrite(enPin, LOW);
        digitalWrite(inPin1, LOW);
        digitalWrite(inPin2, LOW);
    }
    // 모터 정방향 회전 (문 열기)
    void forward(int speed) {
        analogWrite(enPin, speed);
        digitalWrite(inPin1, HIGH);
        digitalWrite(inPin2, LOW);
    }
    // 모터 역방향 회전 (문 닫기)
    void backward(int speed) {
        analogWrite(enPin, speed);
        digitalWrite(inPin1, LOW);
        digitalWrite(inPin2, HIGH);
    }
};

// RS-485 통신 클래스
class RS485Communication {
private:
    int dePin;
    int rePin;
    int deviceID;
    
    void setTransmitMode() {
        digitalWrite(dePin, HIGH);
        digitalWrite(rePin, HIGH);
        delayMicroseconds(10);
    }
    
    void setReceiveMode() {
        digitalWrite(dePin, LOW);
        digitalWrite(rePin, LOW);
        delayMicroseconds(10);
    }

public:
    RS485Communication(int de, int re, int id) : dePin(de), rePin(re), deviceID(id) {}
    
    void init(long baudRate) {
        pinMode(dePin, OUTPUT);
        pinMode(rePin, OUTPUT);
        setReceiveMode(); // 기본은 수신 모드
        Serial1.begin(baudRate); // Arduino Mega Serial1 사용
    }
    
    void sendResponse(const char* msg) {
        setTransmitMode();
        Serial1.print(msg);
        Serial1.flush();
        setReceiveMode();
    }
    
    bool readCommand(char* buffer, int maxLen) {
        int idx = 0;
        while (Serial1.available() && idx < maxLen - 1) {
            char c = Serial1.read();
            if (c == '\n' || c == '\r') {
                buffer[idx] = '\0';
                return idx > 0;
            }
            buffer[idx++] = c;
        }
        buffer[idx] = '\0';
        return idx > 0;
    }
    
    int getDeviceID() { return deviceID; }
};

// 문 제어 클래스
class DoorController {
private:
    MotorDriver& motor;
    int sensorOpen;
    int sensorClose;
    int speedOpen;
    int speedClose;
    
    enum DoorState {
        DOOR_IDLE,
        DOOR_OPENING,
        DOOR_CLOSING,
        DOOR_OPEN,
        DOOR_CLOSED
    };
    
    DoorState state;

public:
    DoorController(MotorDriver& m, int sOpen, int sClose, int spOpen, int spClose) 
        : motor(m), sensorOpen(sOpen), sensorClose(sClose), 
          speedOpen(spOpen), speedClose(spClose), state(DOOR_IDLE) {}
    
    void init() {
        pinMode(sensorOpen, INPUT_PULLUP);
        pinMode(sensorClose, INPUT_PULLUP);
        motor.init();
    }
    
    bool isDoorOpen() {
        return digitalRead(sensorOpen) == LOW; // 센서 활성화 시 LOW
    }
    
    bool isDoorClosed() {
        return digitalRead(sensorClose) == LOW; // 센서 활성화 시 LOW
    }
    
    void openDoor() {
        if (isDoorOpen()) {
            state = DOOR_OPEN;
            return;
        }
        state = DOOR_OPENING;
        motor.forward(speedOpen);
    }
    
    void closeDoor() {
        if (isDoorClosed()) {
            state = DOOR_CLOSED;
            return;
        }
        state = DOOR_CLOSING;
        motor.backward(speedClose);
    }
    
    void stopDoor() {
        motor.stop();
        state = DOOR_IDLE;
    }
    
    void update() {
        switch (state) {
            case DOOR_OPENING:
                if (isDoorOpen()) {
                    motor.stop();
                    state = DOOR_OPEN;
                }
                break;
            case DOOR_CLOSING:
                if (isDoorClosed()) {
                    motor.stop();
                    state = DOOR_CLOSED;
                }
                break;
            default:
                break;
        }
    }
    
    const char* getStatus() {
        switch (state) {
            case DOOR_IDLE: return "IDLE";
            case DOOR_OPENING: return "OPENING";
            case DOOR_CLOSING: return "CLOSING";
            case DOOR_OPEN: return "OPEN";
            case DOOR_CLOSED: return "CLOSED";
            default: return "UNKNOWN";
        }
    }
    
    void setSpeed(int spOpen, int spClose) {
        speedOpen = spOpen;
        speedClose = spClose;
    }
};

// 전역 객체 생성
MotorDriver doorMotor(Pin::MOTOR_12V_ENB, Pin::MOTOR_12V_IN3, Pin::MOTOR_12V_IN4);
RS485Communication rs485(Pin::RS485_DE, Pin::RS485_RE, Defaults::DEVICE_ID);
DoorController door(doorMotor, Pin::SENSOR_DOOR_OPEN, Pin::SENSOR_DOOR_CLOSE, 
                    SystemState::speed_DO, SystemState::speed_DC);

// 명령 처리 함수
void processCommand(char* cmd) {
    // 명령 형식: <ID>:<CMD>:<PARAM>
    // 예: 1:OPEN, 1:CLOSE, 1:STOP, 1:STATUS, 1:SETSPEED:200:180
    
    char response[64];
    
    // ID 파싱
    char* idStr = strtok(cmd, ":");
    if (idStr == NULL) return;
    
    int cmdID = atoi(idStr);
    if (cmdID != rs485.getDeviceID()) return; // 다른 장치용 명령
    
    // 명령어 파싱
    char* command = strtok(NULL, ":");
    if (command == NULL) return;
    
    // 명령 처리
    if (strcmp(command, "OPEN") == 0) {
        door.openDoor();
        snprintf(response, sizeof(response), "%d:OK:OPENING\n", cmdID);
        rs485.sendResponse(response);
    }
    else if (strcmp(command, "CLOSE") == 0) {
        door.closeDoor();
        snprintf(response, sizeof(response), "%d:OK:CLOSING\n", cmdID);
        rs485.sendResponse(response);
    }
    else if (strcmp(command, "STOP") == 0) {
        door.stopDoor();
        snprintf(response, sizeof(response), "%d:OK:STOPPED\n", cmdID);
        rs485.sendResponse(response);
    }
    else if (strcmp(command, "STATUS") == 0) {
        snprintf(response, sizeof(response), "%d:STATUS:%s\n", cmdID, door.getStatus());
        rs485.sendResponse(response);
    }
    else if (strcmp(command, "SETSPEED") == 0) {
        char* openSpeed = strtok(NULL, ":");
        char* closeSpeed = strtok(NULL, ":");
        if (openSpeed && closeSpeed) {
            int spOpen = atoi(openSpeed);
            int spClose = atoi(closeSpeed);
            door.setSpeed(spOpen, spClose);
            SystemState::speed_DO = spOpen;
            SystemState::speed_DC = spClose;
            EEPROM.write(EEPROM_Addr::SPEED_DO, spOpen);
            EEPROM.write(EEPROM_Addr::SPEED_DC, spClose);
            snprintf(response, sizeof(response), "%d:OK:SPEED_SET\n", cmdID);
            rs485.sendResponse(response);
        }
    }
    else {
        snprintf(response, sizeof(response), "%d:ERROR:UNKNOWN_CMD\n", cmdID);
        rs485.sendResponse(response);
    }
}

void setup() {
    // EEPROM에서 속도 설정 로드
    int savedSpeedDO = EEPROM.read(EEPROM_Addr::SPEED_DO);
    int savedSpeedDC = EEPROM.read(EEPROM_Addr::SPEED_DC);
    
    if (savedSpeedDO > 0 && savedSpeedDO <= 255) {
        SystemState::speed_DO = savedSpeedDO;
    }
    if (savedSpeedDC > 0 && savedSpeedDC <= 255) {
        SystemState::speed_DC = savedSpeedDC;
    }
    
    // 시스템 초기화
    door.init();
    rs485.init(Defaults::BAUD_RATE);
    
    // 디버그용 시리얼 (선택사항)
    Serial.begin(9600);
    Serial.println("PETMON Door Control System - RS485");
    Serial.print("Device ID: ");
    Serial.println(Defaults::DEVICE_ID);
}

void loop() {
    // 문 상태 업데이트 (센서 확인)
    door.update();
    
    // RS-485 명령 수신 및 처리
    static char cmdBuffer[64];
    if (rs485.readCommand(cmdBuffer, sizeof(cmdBuffer))) {
        Serial.print("Received: "); // 디버그
        Serial.println(cmdBuffer);
        processCommand(cmdBuffer);
    }
    delay(10); // 짧은 딜레이
}