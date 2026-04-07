# PETCUP Modbus RTU 프로토콜 가이드

## 📋 개요

PETCUP 시스템은 **Modbus RTU** 프로토콜을 사용하여 RS-485 통신을 수행합니다.

- **프로토콜**: Modbus RTU (바이너리)
- **통신 방식**: RS-485 (Half-Duplex)
- **전송 속도**: 9600 baud, 8N1
- **Slave ID**: 1
- **지원 Function Code**:
    - `0x03`: Read Holding Registers
    - `0x06`: Write Single Register
    - `0x10`: Write Multiple Registers

---

## 📊 Modbus 레지스터 맵

| 주소 (Hex) | 주소 (Dec) | 이름             | 타입 | 설명              | 값 범위                                        |
| ---------- | ---------- | ---------------- | ---- | ----------------- | ---------------------------------------------- |
| 0x0000     | 0          | DOOR_CMD         | W    | 문 제어 명령      | 0=STOP, 1=OPEN, 2=CLOSE                        |
| 0x0001     | 1          | DOOR_STATUS      | R    | 문 상태           | 0=IDLE, 1=OPENING, 2=CLOSING, 3=OPEN, 4=CLOSED |
| 0x0002     | 2          | UV_CTRL          | R/W  | UV 라이트 제어    | 0=OFF, 1=ON                                    |
| 0x0003     | 3          | PUMP_CTRL        | R/W  | 워터 펌프 제어    | 0=OFF, 1=ON                                    |
| 0x0004     | 4          | FAN1_CTRL        | R/W  | 팬1 제어          | 0=OFF, 1=ON                                    |
| 0x0005     | 5          | FAN2_CTRL        | R/W  | 팬2 제어          | 0=OFF, 1=ON                                    |
| 0x0006     | 6          | INVERTER_CTRL    | R/W  | MC12B 인버터      | 0=OFF, 1=ON                                    |
| 0x0007     | 7          | FWD_SIGNAL       | R/W  | FA-50 정방향 신호 | 0=OFF, 1=ON                                    |
| 0x0008     | 8          | REV_SIGNAL       | R/W  | FA-50 역방향 신호 | 0=OFF, 1=ON                                    |
| 0x0009     | 9          | DOOR_SPEED_OPEN  | R/W  | 문 열림 속도      | 0-255 (PWM)                                    |
| 0x000A     | 10         | DOOR_SPEED_CLOSE | R/W  | 문 닫힘 속도      | 0-255 (PWM)                                    |
| 0x000B     | 11         | SENSOR_OPEN      | R    | 문 열림 센서 상태 | 0=OFF, 1=ON                                    |
| 0x000C     | 12         | SENSOR_CLOSE     | R    | 문 닫힘 센서 상태 | 0=OFF, 1=ON                                    |

**범례**:

- **R**: 읽기 전용
- **W**: 쓰기 전용
- **R/W**: 읽기/쓰기 가능

---

## 🔧 Modbus Function Code 사용법

### 1️⃣ Function 0x03: Read Holding Registers

레지스터 값을 읽습니다.

**요청 패킷**:

```
[Slave ID][0x03][Start Addr Hi][Start Addr Lo][Num Regs Hi][Num Regs Lo][CRC Lo][CRC Hi]
```

**예시: 문 상태 읽기 (Addr 0x0001, 1개)**

```
01 03 00 01 00 01 D5 CA
```

- Slave ID: `01`
- Function: `03` (Read)
- Start Addr: `00 01` (0x0001)
- Num Regs: `00 01` (1개)
- CRC: `D5 CA`

**응답 패킷**:

```
[Slave ID][0x03][Byte Count][Data Hi][Data Lo][CRC Lo][CRC Hi]
```

**예시 응답: 문이 OPEN 상태 (값=3)**

```
01 03 02 00 03 39 84
```

- Byte Count: `02` (2바이트)
- Data: `00 03` (3 = DOOR_OPEN)
- CRC: `39 84`

---

### 2️⃣ Function 0x06: Write Single Register

단일 레지스터에 값을 씁니다.

**요청 패킷**:

```
[Slave ID][0x06][Reg Addr Hi][Reg Addr Lo][Value Hi][Value Lo][CRC Lo][CRC Hi]
```

**예시: 문 열기 명령 (Addr 0x0000, Value=1)**

```
01 06 00 00 00 01 48 0A
```

- Slave ID: `01`
- Function: `06` (Write Single)
- Reg Addr: `00 00` (0x0000 = DOOR_CMD)
- Value: `00 01` (1 = OPEN)
- CRC: `48 0A`

**응답 패킷** (에코):

```
01 06 00 00 00 01 48 0A
```

(요청과 동일)

---

### 3️⃣ Function 0x10: Write Multiple Registers

여러 레지스터에 동시에 값을 씁니다.

**요청 패킷**:

```
[Slave ID][0x10][Start Addr Hi][Start Addr Lo][Num Regs Hi][Num Regs Lo][Byte Count][Data...][CRC Lo][CRC Hi]
```

**예시: UV와 펌프 켜기 (Addr 0x0002~0x0003, 각각 1)**

```
01 10 00 02 00 02 04 00 01 00 01 XX XX
```

- Start Addr: `00 02` (0x0002 = UV_CTRL)
- Num Regs: `00 02` (2개)
- Byte Count: `04` (4바이트)
- Data: `00 01 00 01` (UV=1, PUMP=1)
- CRC: 계산 필요

**응답 패킷**:

```
01 10 00 02 00 02 XX XX
```

---

## 📡 명령어 예시

### 🚪 문 제어

**문 열기**:

```
01 06 00 00 00 01 48 0A
```

**문 닫기**:

```
01 06 00 00 00 02 09 CA
```

**문 정지**:

```
01 06 00 00 00 00 89 CB
```

---

### 💡 장치 제어

**UV 라이트 ON**:

```
01 06 00 02 00 01 E9 CA
```

**UV 라이트 OFF**:

```
01 06 00 02 00 00 28 0A
```

**워터 펌프 ON**:

```
01 06 00 03 00 01 B8 0A
```

**워터 펌프 OFF**:

```
01 06 00 03 00 00 79 CA
```

**팬1 ON**:

```
01 06 00 04 00 01 88 0B
```

**팬2 ON**:

```
01 06 00 05 00 01 D9 CB
```

---

### ⚙️ 인버터 제어

**MC12B 인버터 ON**:

```
01 06 00 06 00 01 29 CB
```

**FWD 신호 ON**:

```
01 06 00 07 00 01 78 0B
```

**REV 신호 ON**:

```
01 06 00 08 00 01 89 C8
```

---

### 📊 상태 읽기

**문 상태 확인**:

```
01 03 00 01 00 01 D5 CA
```

**센서 상태 확인 (열림/닫힘)**:

```
01 03 00 0B 00 02 35 CE
```

- Addr 0x000B (SENSOR_OPEN)
- Addr 0x000C (SENSOR_CLOSE)

**전체 상태 읽기 (13개 레지스터)**:

```
01 03 00 00 00 0D 05 C6
```

---

## 🧪 테스트 도구

### Python으로 Modbus 명령 전송

```python
import serial
import struct

def calculate_crc(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc

def write_register(ser, addr, value):
    packet = bytearray([0x01, 0x06])
    packet.extend(struct.pack('>HH', addr, value))
    crc = calculate_crc(packet)
    packet.extend(struct.pack('<H', crc))
    ser.write(packet)
    response = ser.read(8)
    print(f"TX: {packet.hex().upper()}")
    print(f"RX: {response.hex().upper()}")

# 포트 열기
ser = serial.Serial('COM3', 9600, timeout=1)

# 문 열기
write_register(ser, 0x0000, 1)

ser.close()
```

---

## 🔍 CRC-16 (Modbus) 계산

Modbus RTU는 **CRC-16-ANSI (CRC-16-IBM)** 알고리즘을 사용합니다.

**초기값**: `0xFFFF`  
**다항식**: `0xA001` (역순)

**C/C++ 예제**:

```c
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
    return crc; // Little-endian: [Low Byte, High Byte]
}
```

---

## ⚠️ 주의사항

1. **CRC 검증 실패**: CRC가 맞지 않으면 Arduino는 응답하지 않습니다.
2. **읽기 전용 레지스터**: `DOOR_STATUS`, `SENSOR_OPEN`, `SENSOR_CLOSE`에 쓰기 시도 시 오류 응답 (Exception 0x03)
3. **타임아웃**: 요청 후 50ms 내에 응답이 없으면 재전송 필요
4. **3.5 Character Silence**: Modbus RTU는 프레임 간 최소 3.5 character time (약 10ms @ 9600 baud) 필요
5. **Half-Duplex**: RS-485는 송신/수신 동시 불가. DE/RE 핀 제어 필요

---

## 📚 참고 자료

- [Modbus Protocol Specification](https://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf)
- [Modbus RTU Frame Format](https://www.simplymodbus.ca/RTU.htm)
- [CRC-16 Calculator](https://www.lammertbies.nl/comm/info/crc-calculation)

---

**문서 버전**: 1.0  
**최종 수정**: 2026-04-07  
**작성자**: PETCUP Development Team
