/**
 * PETMON Firmware - Modbus RTU
 * Version: 3.0 (Modbus RTU Protocol)
 * 
 * Target: Arduino Mega / Standard Arduino Environment
 * Description: Modbus RTU based firmware for PetMon Machine
 * Protocol: Modbus RTU (RS-485)
 * Supported Function Codes: 0x03 (Read), 0x06 (Write Single), 0x10 (Write Multiple)
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

 // ========================================
 // Modbus RTU Register Map
 // ========================================
 namespace ModbusReg {
    constexpr uint16_t DOOR_CMD = 0x0000;         // Write: 0=STOP, 1=OPEN, 2=CLOSE
    constexpr uint16_t DOOR_STATUS = 0x0001;       // Read: 0=IDLE, 1=OPENING, 2=CLOSING, 3=OPEN, 4=CLOSED
    constexpr uint16_t UV_CTRL = 0x0002;           // R/W: 0=OFF, 1=ON
    constexpr uint16_t PUMP_CTRL = 0x0003;         // R/W: 0=OFF, 1=ON
    constexpr uint16_t FAN1_CTRL = 0x0004;         // R/W: 0=OFF, 1=ON
    constexpr uint16_t FAN2_CTRL = 0x0005;         // R/W: 0=OFF, 1=ON
    constexpr uint16_t INVERTER_CTRL = 0x0006;     // R/W: 0=OFF, 1=ON
    constexpr uint16_t FWD_SIGNAL = 0x0007;        // R/W: 0=OFF, 1=ON
    constexpr uint16_t REV_SIGNAL = 0x0008;        // R/W: 0=OFF, 1=ON
    constexpr uint16_t DOOR_SPEED_OPEN = 0x0009;   // R/W: 0-255
    constexpr uint16_t DOOR_SPEED_CLOSE = 0x000A;  // R/W: 0-255
    constexpr uint16_t SENSOR_OPEN = 0x000B;       // Read: 0=OFF, 1=ON
    constexpr uint16_t SENSOR_CLOSE = 0x000C;      // Read: 0=OFF, 1=ON
    constexpr uint16_t MAX_REGISTER = 0x000D;      // Total 13 registers
 }

 // Modbus Function Codes
 namespace ModbusFunc {
    constexpr uint8_t READ_HOLDING_REGISTERS = 0x03;
    constexpr uint8_t WRITE_SINGLE_REGISTER = 0x06;
    constexpr uint8_t WRITE_MULTIPLE_REGISTERS = 0x10;
 }

 // Modbus Exception Codes
 namespace ModbusException {
    constexpr uint8_t ILLEGAL_FUNCTION = 0x01;
    constexpr uint8_t ILLEGAL_DATA_ADDRESS = 0x02;
    constexpr uint8_t ILLEGAL_DATA_VALUE = 0x03;
 }

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

    // Water pump
    constexpr int WATER_PUMP = 45;

    // UV Light
    constexpr int UV_LIGHT = 49;

    // Fan
    constexpr int FAN1 = 43;
    constexpr int FAN2 = 44;

    // Inverter (MC12B, FA-50)
    constexpr int INVERTER_ENABLE = 51;
    constexpr int INVERTER_FWD = 40;  // FA-50 Forward Signal
    constexpr int INVERTER_REV = 39;  // FA-50 Reverse Signal
 }

 namespace EEPROM_Addr {
    constexpr int SPEED_DO = 0; // DOOR OPEN
    constexpr int SPEED_DC = 1; // DOOR CLOSE
 }

 namespace Defaults {
    constexpr int SPEED_DO = 200; // Default Speed for Door Open
    constexpr int SPEED_DC = 200; // Default Speed for Door Close
    constexpr uint8_t MODBUS_SLAVE_ID = 1;  // Modbus Slave ID
    constexpr long BAUD_RATE = 9600; // RS-485 Communication Speed
 }

 // Modbus Holding Registers Storage (13 registers)
 uint16_t modbusRegisters[ModbusReg::MAX_REGISTER] = {0};

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

// ========================================
// Modbus RTU CRC-16 계산
// ========================================
uint16_t calculateModbusCRC(uint8_t* data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    
    for (uint8_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

// ========================================
// Modbus RTU 통신 클래스
// ========================================
class ModbusRTU {
private:
    int dePin;
    int rePin;
    uint8_t slaveID;
    uint8_t rxBuffer[256];
    uint8_t txBuffer[256];
    
    void setTransmitMode() {
        digitalWrite(dePin, HIGH);
        digitalWrite(rePin, HIGH);
        delayMicroseconds(100);
    }
    
    void setReceiveMode() {
        digitalWrite(dePin, LOW);
        digitalWrite(rePin, LOW);
        delayMicroseconds(100);
    }

public:
    ModbusRTU(int de, int re, uint8_t id) : dePin(de), rePin(re), slaveID(id) {}
    
    void init(long baudRate) {
        pinMode(dePin, OUTPUT);
        pinMode(rePin, OUTPUT);
        setReceiveMode();
        Serial3.begin(baudRate); // Arduino Mega Serial3 (Pin 14, 15)
    }
    
    // Modbus 응답 전송
    void sendResponse(uint8_t* data, uint8_t length) {
        // CRC 계산 및 추가
        uint16_t crc = calculateModbusCRC(data, length);
        data[length++] = crc & 0xFF;        // CRC Low
        data[length++] = (crc >> 8) & 0xFF;  // CRC High
        
        // 송신 모드로 전환
        setTransmitMode();
        delay(2);
        
        // 데이터 전송
        Serial3.write(data, length);
        Serial3.flush();
        
        // 수신 모드로 복귀
        delay(2);
        setReceiveMode();
    }
    
    // Modbus 예외 응답 전송
    void sendException(uint8_t functionCode, uint8_t exceptionCode) {
        uint8_t response[3];
        response[0] = slaveID;
        response[1] = functionCode | 0x80;  // 최상위 비트 설정
        response[2] = exceptionCode;
        sendResponse(response, 3);
    }
    
    // Modbus 요청 읽기 (non-blocking)
    bool readRequest(uint8_t* buffer, uint8_t* length) {
        static uint8_t idx = 0;
        static unsigned long lastByteTime = 0;
        const unsigned long FRAME_TIMEOUT = 10; // 3.5 character time (약 10ms @ 9600 baud)
        
        unsigned long currentTime = millis();
        
        // 프레임 타임아웃 체크 (3.5 character silence)
        if (idx > 0 && (currentTime - lastByteTime) > FRAME_TIMEOUT) {
            // 프레임 수신 완료
            *length = idx;
            memcpy(buffer, rxBuffer, idx);
            idx = 0;
            return true;
        }
        
        // 바이트 읽기
        while (Serial3.available()) {
            if (idx < 256) {
                rxBuffer[idx++] = Serial3.read();
                lastByteTime = currentTime;
            } else {
                // 버퍼 오버플로우
                idx = 0;
                break;
            }
        }
        
        return false;
    }
    
    uint8_t getSlaveID() { return slaveID; }
};

// 디버그 로그 헬퍼 함수
void debugLog(const char* msg) {
    Serial.println(msg);
}

// 인버터 제어 클래스 (MC12B, FA-50)
class InverterController {
public:
    static void init() {
        pinMode(Pin::INVERTER_ENABLE, OUTPUT);
        pinMode(Pin::INVERTER_FWD, OUTPUT);
        pinMode(Pin::INVERTER_REV, OUTPUT);
        digitalWrite(Pin::INVERTER_ENABLE, LOW);
        digitalWrite(Pin::INVERTER_FWD, LOW);
        digitalWrite(Pin::INVERTER_REV, LOW);
    }
    
    static void enable() {
        digitalWrite(Pin::INVERTER_ENABLE, HIGH);
    }
    
    static void disable() {
        digitalWrite(Pin::INVERTER_ENABLE, LOW);
    }
    
    static void on() {
        digitalWrite(Pin::INVERTER_ENABLE, HIGH);
    }
    
    static void off() {
        digitalWrite(Pin::INVERTER_ENABLE, LOW);
    }
    
    static void setFwd(bool state) {
        if (state) {
            digitalWrite(Pin::INVERTER_FWD, HIGH);
            debugLog("⚡ FA-50 FWD ON (Pin 40 = HIGH)");
        } else {
            digitalWrite(Pin::INVERTER_FWD, LOW);
            debugLog("⚡ FA-50 FWD OFF (Pin 40 = LOW)");
        }
    }
    
    static void setRev(bool state) {
        if (state) {
            digitalWrite(Pin::INVERTER_REV, HIGH);
            debugLog("🔄 FA-50 REV ON (Pin 39 = HIGH)");
        } else {
            digitalWrite(Pin::INVERTER_REV, LOW);
            debugLog("🔄 FA-50 REV OFF (Pin 39 = LOW)");
        }
    }
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
        DOOR_IDLE = 0,
        DOOR_OPENING = 1,
        DOOR_CLOSING = 2,
        DOOR_OPEN = 3,
        DOOR_CLOSED = 4
    };
    
    DoorState state;

public:
    DoorController(MotorDriver& m, int sOpen, int sClose, int spOpen, int spClose) 
        : motor(m), sensorOpen(sOpen), sensorClose(sClose), 
          speedOpen(spOpen), speedClose(spClose), state(DOOR_IDLE) {}
    
    void init() {
        pinMode(sensorOpen, INPUT);
        pinMode(sensorClose, INPUT);
        motor.init();
        // 초기 상태를 레지스터에 기록
        modbusRegisters[ModbusReg::DOOR_STATUS] = DOOR_IDLE;
    }
    
    bool isDoorOpen() {
        return digitalRead(sensorOpen) == HIGH;
    }
    
    bool isDoorClosed() {
        return digitalRead(sensorClose) == HIGH;
    }
    
    void openDoor() {
        state = DOOR_OPENING;
        modbusRegisters[ModbusReg::DOOR_STATUS] = state;
        motor.forward(speedOpen);
    }
    
    void closeDoor() {
        state = DOOR_CLOSING;
        modbusRegisters[ModbusReg::DOOR_STATUS] = state;
        motor.backward(speedClose);
    }
    
    void stopDoor() {
        motor.stop();
        state = DOOR_IDLE;
        modbusRegisters[ModbusReg::DOOR_STATUS] = state;
    }
    
    void update() {
        // 센서 상태를 레지스터에 업데이트
        modbusRegisters[ModbusReg::SENSOR_OPEN] = isDoorOpen() ? 1 : 0;
        modbusRegisters[ModbusReg::SENSOR_CLOSE] = isDoorClosed() ? 1 : 0;
        
        switch (state) {
            case DOOR_OPENING:
                if (isDoorOpen()) {
                    motor.stop();
                    state = DOOR_OPEN;
                    modbusRegisters[ModbusReg::DOOR_STATUS] = state;
                }
                break;
            case DOOR_CLOSING:
                if (isDoorClosed()) {
                    motor.stop();
                    state = DOOR_CLOSED;
                    modbusRegisters[ModbusReg::DOOR_STATUS] = state;
                    debugLog("✅ Door CLOSED (sensor detected)");
                    // 문이 닫히면 인버터(MC12B) 켜고 FWD 신호 전송
                    debugLog("🔌 MC12B ON (Pin 51 = HIGH)");
                    InverterController::enable();
                    modbusRegisters[ModbusReg::INVERTER_CTRL] = 1;
                    delay(50);
                    InverterController::setFwd(true);
                    modbusRegisters[ModbusReg::FWD_SIGNAL] = 1;
                }
                break;
            case DOOR_OPEN:
                if (!isDoorOpen()) {
                    state = DOOR_CLOSING;
                    modbusRegisters[ModbusReg::DOOR_STATUS] = state;
                    motor.backward(speedClose);
                }
                break;
            case DOOR_CLOSED:
                if (!isDoorClosed()) {
                    state = DOOR_OPENING;
                    modbusRegisters[ModbusReg::DOOR_STATUS] = state;
                    motor.forward(speedOpen);
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
        modbusRegisters[ModbusReg::DOOR_SPEED_OPEN] = spOpen;
        modbusRegisters[ModbusReg::DOOR_SPEED_CLOSE] = spClose;
    }
    
    uint16_t getState() { return (uint16_t)state; }
};

// 기타 제어 클래스
class DeviceController {
public:
    static void init() {
        pinMode(Pin::UV_LIGHT, OUTPUT);
        pinMode(Pin::WATER_PUMP, OUTPUT);
        pinMode(Pin::FAN1, OUTPUT);
        pinMode(Pin::FAN2, OUTPUT);
        
        // 초기 상태: 모두 OFF
        digitalWrite(Pin::UV_LIGHT, LOW);
        digitalWrite(Pin::WATER_PUMP, LOW);
        digitalWrite(Pin::FAN1, LOW);
        digitalWrite(Pin::FAN2, LOW);
    }
    
    static void setUV(bool on) {
        digitalWrite(Pin::UV_LIGHT, on ? HIGH : LOW);
    }
    
    static void setPump(bool on) {
        digitalWrite(Pin::WATER_PUMP, on ? HIGH : LOW);
    }
    
    static void setFan1(bool on) {
        digitalWrite(Pin::FAN1, on ? HIGH : LOW);
    }
    
    static void setFan2(bool on) {
        digitalWrite(Pin::FAN2, on ? HIGH : LOW);
    }
    
    static void setAllFans(bool on) {
        setFan1(on);
        setFan2(on);
    }
};

// 전역 객체 생성
MotorDriver doorMotor(Pin::MOTOR_12V_ENB, Pin::MOTOR_12V_IN3, Pin::MOTOR_12V_IN4);
ModbusRTU modbus(Pin::RS485_DE, Pin::RS485_RE, Defaults::MODBUS_SLAVE_ID);
DoorController door(doorMotor, Pin::SENSOR_DOOR_OPEN, Pin::SENSOR_DOOR_CLOSE, 
                    SystemState::speed_DO, SystemState::speed_DC);

// ========================================
// Modbus 요청 처리 함수
// ========================================
void processModbusRequest(uint8_t* request, uint8_t length) {
    // 최소 길이 체크 (SlaveID + Function + Data + CRC = 최소 5바이트)
    if (length < 5) return;
    
    uint8_t slaveID = request[0];
    uint8_t functionCode = request[1];
    
    // Slave ID 확인
    if (slaveID != modbus.getSlaveID()) return;
    
    // CRC 검증
    uint16_t receivedCRC = request[length - 2] | (request[length - 1] << 8);
    uint16_t calculatedCRC = calculateModbusCRC(request, length - 2);
    
    if (receivedCRC != calculatedCRC) {
        debugLog("⚠️ CRC Error");
        return;
    }
    
    // Function Code 처리
    switch (functionCode) {
        case ModbusFunc::READ_HOLDING_REGISTERS:
            handleReadHoldingRegisters(request, length);
            break;
            
        case ModbusFunc::WRITE_SINGLE_REGISTER:
            handleWriteSingleRegister(request, length);
            break;
            
        case ModbusFunc::WRITE_MULTIPLE_REGISTERS:
            handleWriteMultipleRegisters(request, length);
            break;
            
        default:
            modbus.sendException(functionCode, ModbusException::ILLEGAL_FUNCTION);
            break;
    }
}

// Function 0x03: Read Holding Registers
void handleReadHoldingRegisters(uint8_t* request, uint8_t length) {
    if (length != 8) {
        modbus.sendException(ModbusFunc::READ_HOLDING_REGISTERS, ModbusException::ILLEGAL_DATA_VALUE);
        return;
    }
    
    uint16_t startAddr = (request[2] << 8) | request[3];
    uint16_t numRegs = (request[4] << 8) | request[5];
    
    // 주소 범위 체크
    if (startAddr >= ModbusReg::MAX_REGISTER || (startAddr + numRegs) > ModbusReg::MAX_REGISTER || numRegs == 0 || numRegs > 125) {
        modbus.sendException(ModbusFunc::READ_HOLDING_REGISTERS, ModbusException::ILLEGAL_DATA_ADDRESS);
        return;
    }
    
    // 응답 생성
    uint8_t response[256];
    uint8_t idx = 0;
    response[idx++] = modbus.getSlaveID();
    response[idx++] = ModbusFunc::READ_HOLDING_REGISTERS;
    response[idx++] = numRegs * 2; // 바이트 수
    
    for (uint16_t i = 0; i < numRegs; i++) {
        uint16_t regValue = modbusRegisters[startAddr + i];
        response[idx++] = (regValue >> 8) & 0xFF; // High byte
        response[idx++] = regValue & 0xFF;         // Low byte
    }
    
    modbus.sendResponse(response, idx);
}

// Function 0x06: Write Single Register
void handleWriteSingleRegister(uint8_t* request, uint8_t length) {
    if (length != 8) {
        modbus.sendException(ModbusFunc::WRITE_SINGLE_REGISTER, ModbusException::ILLEGAL_DATA_VALUE);
        return;
    }
    
    uint16_t regAddr = (request[2] << 8) | request[3];
    uint16_t regValue = (request[4] << 8) | request[5];
    
    // 주소 범위 체크
    if (regAddr >= ModbusReg::MAX_REGISTER) {
        modbus.sendException(ModbusFunc::WRITE_SINGLE_REGISTER, ModbusException::ILLEGAL_DATA_ADDRESS);
        return;
    }
    
    // 레지스터 쓰기 및 동작 실행
    if (!writeRegisterAndExecute(regAddr, regValue)) {
        modbus.sendException(ModbusFunc::WRITE_SINGLE_REGISTER, ModbusException::ILLEGAL_DATA_VALUE);
        return;
    }
    
    // Echo 응답 (원본 요청의 앞 6바이트)
    uint8_t response[6];
    memcpy(response, request, 6);
    modbus.sendResponse(response, 6);
}

// Function 0x10: Write Multiple Registers
void handleWriteMultipleRegisters(uint8_t* request, uint8_t length) {
    if (length < 9) {
        modbus.sendException(ModbusFunc::WRITE_MULTIPLE_REGISTERS, ModbusException::ILLEGAL_DATA_VALUE);
        return;
    }
    
    uint16_t startAddr = (request[2] << 8) | request[3];
    uint16_t numRegs = (request[4] << 8) | request[5];
    uint8_t byteCount = request[6];
    
    // 데이터 길이 체크
    if (byteCount != numRegs * 2 || length != (9 + byteCount)) {
        modbus.sendException(ModbusFunc::WRITE_MULTIPLE_REGISTERS, ModbusException::ILLEGAL_DATA_VALUE);
        return;
    }
    
    // 주소 범위 체크
    if (startAddr >= ModbusReg::MAX_REGISTER || (startAddr + numRegs) > ModbusReg::MAX_REGISTER || numRegs == 0) {
        modbus.sendException(ModbusFunc::WRITE_MULTIPLE_REGISTERS, ModbusException::ILLEGAL_DATA_ADDRESS);
        return;
    }
    
    // 레지스터 쓰기
    for (uint16_t i = 0; i < numRegs; i++) {
        uint16_t regValue = (request[7 + i * 2] << 8) | request[8 + i * 2];
        if (!writeRegisterAndExecute(startAddr + i, regValue)) {
            modbus.sendException(ModbusFunc::WRITE_MULTIPLE_REGISTERS, ModbusException::ILLEGAL_DATA_VALUE);
            return;
        }
    }
    
    // 응답 생성
    uint8_t response[6];
    response[0] = modbus.getSlaveID();
    response[1] = ModbusFunc::WRITE_MULTIPLE_REGISTERS;
    response[2] = (startAddr >> 8) & 0xFF;
    response[3] = startAddr & 0xFF;
    response[4] = (numRegs >> 8) & 0xFF;
    response[5] = numRegs & 0xFF;
    modbus.sendResponse(response, 6);
}

// 레지스터 쓰기 및 실제 동작 실행
bool writeRegisterAndExecute(uint16_t addr, uint16_t value) {
    // 읽기 전용 레지스터 체크
    if (addr == ModbusReg::DOOR_STATUS || addr == ModbusReg::SENSOR_OPEN || addr == ModbusReg::SENSOR_CLOSE) {
        return false; // 읽기 전용
    }
    
    // 레지스터에 값 저장
    modbusRegisters[addr] = value;
    
    // 실제 동작 실행
    switch (addr) {
        case ModbusReg::DOOR_CMD:
            if (value == 0) {
                door.stopDoor();
            } else if (value == 1) {
                door.openDoor();
            } else if (value == 2) {
                door.closeDoor();
            } else {
                return false;
            }
            break;
            
        case ModbusReg::UV_CTRL:
            DeviceController::setUV(value != 0);
            break;
            
        case ModbusReg::PUMP_CTRL:
            DeviceController::setPump(value != 0);
            break;
            
        case ModbusReg::FAN1_CTRL:
            DeviceController::setFan1(value != 0);
            break;
            
        case ModbusReg::FAN2_CTRL:
            DeviceController::setFan2(value != 0);
            break;
            
        case ModbusReg::INVERTER_CTRL:
            if (value != 0) {
                InverterController::enable();
            } else {
                InverterController::disable();
            }
            break;
            
        case ModbusReg::FWD_SIGNAL:
            InverterController::setFwd(value != 0);
            break;
            
        case ModbusReg::REV_SIGNAL:
            InverterController::setRev(value != 0);
            break;
            
        case ModbusReg::DOOR_SPEED_OPEN:
            if (value > 255) return false;
            SystemState::speed_DO = value;
            door.setSpeed(SystemState::speed_DO, SystemState::speed_DC);
            EEPROM.write(EEPROM_Addr::SPEED_DO, value);
            break;
            
        case ModbusReg::DOOR_SPEED_CLOSE:
            if (value > 255) return false;
            SystemState::speed_DC = value;
            door.setSpeed(SystemState::speed_DO, SystemState::speed_DC);
            EEPROM.write(EEPROM_Addr::SPEED_DC, value);
            break;
            
        default:
            break;
    }
    
    return true;
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
    
    // Modbus 레지스터 초기화
    modbusRegisters[ModbusReg::DOOR_CMD] = 0;
    modbusRegisters[ModbusReg::DOOR_STATUS] = 0;
    modbusRegisters[ModbusReg::UV_CTRL] = 0;
    modbusRegisters[ModbusReg::PUMP_CTRL] = 0;
    modbusRegisters[ModbusReg::FAN1_CTRL] = 0;
    modbusRegisters[ModbusReg::FAN2_CTRL] = 0;
    modbusRegisters[ModbusReg::INVERTER_CTRL] = 0;
    modbusRegisters[ModbusReg::FWD_SIGNAL] = 0;
    modbusRegisters[ModbusReg::REV_SIGNAL] = 0;
    modbusRegisters[ModbusReg::DOOR_SPEED_OPEN] = SystemState::speed_DO;
    modbusRegisters[ModbusReg::DOOR_SPEED_CLOSE] = SystemState::speed_DC;
    modbusRegisters[ModbusReg::SENSOR_OPEN] = 0;
    modbusRegisters[ModbusReg::SENSOR_CLOSE] = 0;
    
    // 시스템 초기화
    door.init();
    modbus.init(Defaults::BAUD_RATE);
    InverterController::init();
    DeviceController::init();
    
    // USB 시리얼 초기화 (디버그용)
    Serial.begin(9600);
    delay(100);
    
    // 초기화 완료 메시지
    debugLog("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    debugLog("PETCUP System Initialized");
    debugLog("Protocol: Modbus RTU");
    debugLog("Slave ID: 1");
    debugLog("Baud Rate: 9600");
    debugLog("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

void loop() {
    // 문 상태 업데이트 (센서 확인 및 레지스터 업데이트)
    door.update();
    
    // Modbus RTU 요청 수신 및 처리
    static uint8_t modbusRequest[256];
    uint8_t requestLength = 0;
    
    if (modbus.readRequest(modbusRequest, &requestLength)) {
        // 디버깅: 받은 요청 출력 (16진수)
        char logMsg[128];
        snprintf(logMsg, sizeof(logMsg), "📥 Modbus RX [%d]: ", requestLength);
        Serial.print(logMsg);
        for (uint8_t i = 0; i < requestLength; i++) {
            char hexByte[4];
            snprintf(hexByte, sizeof(hexByte), "%02X ", modbusRequest[i]);
            Serial.print(hexByte);
        }
        Serial.println();
        
        // Modbus 요청 처리
        processModbusRequest(modbusRequest, requestLength);
    }
    
    delay(5); // 짧은 딜레이
}