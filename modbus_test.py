#!/usr/bin/env python3
"""
PETCUP Modbus RTU Test Script
RS-485 통신을 통해 PETCUP 시스템을 제어하는 테스트 스크립트
"""

import serial
import struct
import time
import sys

# Modbus 설정
MODBUS_SLAVE_ID = 1
BAUD_RATE = 9600
TIMEOUT = 0.5  # 500ms

# 레지스터 맵
class ModbusReg:
    DOOR_CMD = 0x0000
    DOOR_STATUS = 0x0001
    UV_CTRL = 0x0002
    PUMP_CTRL = 0x0003
    FAN1_CTRL = 0x0004
    FAN2_CTRL = 0x0005
    INVERTER_CTRL = 0x0006
    FWD_SIGNAL = 0x0007
    REV_SIGNAL = 0x0008
    DOOR_SPEED_OPEN = 0x0009
    DOOR_SPEED_CLOSE = 0x000A
    SENSOR_OPEN = 0x000B
    SENSOR_CLOSE = 0x000C

# CRC-16 (Modbus) 계산
def calculate_crc(data):
    """Modbus RTU CRC-16 계산"""
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

# Modbus RTU 패킷 생성
def build_read_registers_packet(addr, count):
    """Function 0x03: Read Holding Registers"""
    packet = bytearray([MODBUS_SLAVE_ID, 0x03])
    packet.extend(struct.pack('>HH', addr, count))
    crc = calculate_crc(packet)
    packet.extend(struct.pack('<H', crc))
    return packet

def build_write_single_register_packet(addr, value):
    """Function 0x06: Write Single Register"""
    packet = bytearray([MODBUS_SLAVE_ID, 0x06])
    packet.extend(struct.pack('>HH', addr, value))
    crc = calculate_crc(packet)
    packet.extend(struct.pack('<H', crc))
    return packet

# Modbus 통신 함수
def send_modbus_command(ser, packet):
    """Modbus 명령 전송 및 응답 수신"""
    print(f"📤 TX: {packet.hex().upper()}")
    ser.write(packet)
    time.sleep(0.05)  # 50ms 대기
    
    response = ser.read(256)
    if len(response) > 0:
        print(f"📥 RX: {response.hex().upper()}")
        
        # CRC 검증
        if len(response) >= 5:
            received_crc = struct.unpack('<H', response[-2:])[0]
            calculated_crc = calculate_crc(response[:-2])
            if received_crc == calculated_crc:
                print("✅ CRC 검증 성공")
                return response
            else:
                print(f"❌ CRC 오류! (받음: {received_crc:04X}, 계산: {calculated_crc:04X})")
        return response
    else:
        print("⚠️ 응답 없음")
        return None

def write_register(ser, addr, value):
    """레지스터 쓰기"""
    packet = build_write_single_register_packet(addr, value)
    return send_modbus_command(ser, packet)

def read_registers(ser, addr, count):
    """레지스터 읽기"""
    packet = build_read_registers_packet(addr, count)
    response = send_modbus_command(ser, packet)
    
    if response and len(response) >= 5:
        byte_count = response[2]
        values = []
        for i in range(0, byte_count, 2):
            value = struct.unpack('>H', response[3+i:5+i])[0]
            values.append(value)
        print(f"📊 값: {values}")
        return values
    return None

# 테스트 함수들
def test_door_open(ser):
    """문 열기 테스트"""
    print("\n🚪 문 열기 테스트")
    print("="*50)
    write_register(ser, ModbusReg.DOOR_CMD, 1)
    time.sleep(1)
    read_registers(ser, ModbusReg.DOOR_STATUS, 1)

def test_door_close(ser):
    """문 닫기 테스트"""
    print("\n🚪 문 닫기 테스트")
    print("="*50)
    write_register(ser, ModbusReg.DOOR_CMD, 2)
    time.sleep(1)
    read_registers(ser, ModbusReg.DOOR_STATUS, 1)

def test_uv_control(ser):
    """UV 라이트 제어 테스트"""
    print("\n💡 UV 라이트 ON")
    print("="*50)
    write_register(ser, ModbusReg.UV_CTRL, 1)
    time.sleep(2)
    
    print("\n💡 UV 라이트 OFF")
    print("="*50)
    write_register(ser, ModbusReg.UV_CTRL, 0)

def test_pump_control(ser):
    """펌프 제어 테스트"""
    print("\n💧 워터 펌프 ON (3초)")
    print("="*50)
    write_register(ser, ModbusReg.PUMP_CTRL, 1)
    time.sleep(3)
    
    print("\n💧 워터 펌프 OFF")
    print("="*50)
    write_register(ser, ModbusReg.PUMP_CTRL, 0)

def test_fan_control(ser):
    """팬 제어 테스트"""
    print("\n🌀 팬 ON")
    print("="*50)
    write_register(ser, ModbusReg.FAN1_CTRL, 1)
    time.sleep(0.1)
    write_register(ser, ModbusReg.FAN2_CTRL, 1)
    time.sleep(2)
    
    print("\n🌀 팬 OFF")
    print("="*50)
    write_register(ser, ModbusReg.FAN1_CTRL, 0)
    time.sleep(0.1)
    write_register(ser, ModbusReg.FAN2_CTRL, 0)

def test_read_all_status(ser):
    """전체 상태 읽기"""
    print("\n📊 전체 상태 읽기 (13개 레지스터)")
    print("="*50)
    values = read_registers(ser, 0x0000, 13)
    
    if values:
        print("\n📋 상태 요약:")
        print(f"  문 명령:         {values[0]}")
        print(f"  문 상태:         {values[1]} (0=IDLE, 1=OPENING, 2=CLOSING, 3=OPEN, 4=CLOSED)")
        print(f"  UV 라이트:       {values[2]} (0=OFF, 1=ON)")
        print(f"  워터 펌프:       {values[3]} (0=OFF, 1=ON)")
        print(f"  팬1:             {values[4]} (0=OFF, 1=ON)")
        print(f"  팬2:             {values[5]} (0=OFF, 1=ON)")
        print(f"  인버터 MC12B:    {values[6]} (0=OFF, 1=ON)")
        print(f"  FWD 신호:        {values[7]} (0=OFF, 1=ON)")
        print(f"  REV 신호:        {values[8]} (0=OFF, 1=ON)")
        print(f"  문 열림 속도:    {values[9]} (0-255)")
        print(f"  문 닫힘 속도:    {values[10]} (0-255)")
        print(f"  열림 센서:       {values[11]} (0=OFF, 1=ON)")
        print(f"  닫힘 센서:       {values[12]} (0=OFF, 1=ON)")

def test_sensor_monitoring(ser):
    """센서 모니터링 (10초)"""
    print("\n👁️ 센서 모니터링 (10초)")
    print("="*50)
    
    for i in range(20):
        values = read_registers(ser, ModbusReg.SENSOR_OPEN, 2)
        if values:
            print(f"[{i*0.5:.1f}s] 열림센서={values[0]}, 닫힘센서={values[1]}")
        time.sleep(0.5)

# 메인 메뉴
def main():
    """메인 함수"""
    if len(sys.argv) < 2:
        print("사용법: python modbus_test.py <COM_PORT>")
        print("예시: python modbus_test.py COM3")
        sys.exit(1)
    
    port = sys.argv[1]
    
    try:
        print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        print("🔧 PETCUP Modbus RTU 테스트 스크립트")
        print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        print(f"포트: {port}")
        print(f"통신 속도: {BAUD_RATE} baud")
        print(f"Slave ID: {MODBUS_SLAVE_ID}")
        print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n")
        
        # 시리얼 포트 열기
        ser = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT)
        time.sleep(0.5)
        
        while True:
            print("\n📋 테스트 메뉴:")
            print("  1. 문 열기")
            print("  2. 문 닫기")
            print("  3. UV 라이트 제어")
            print("  4. 워터 펌프 제어 (3초)")
            print("  5. 팬 제어")
            print("  6. 전체 상태 읽기")
            print("  7. 센서 모니터링 (10초)")
            print("  0. 종료")
            
            choice = input("\n선택: ").strip()
            
            if choice == '1':
                test_door_open(ser)
            elif choice == '2':
                test_door_close(ser)
            elif choice == '3':
                test_uv_control(ser)
            elif choice == '4':
                test_pump_control(ser)
            elif choice == '5':
                test_fan_control(ser)
            elif choice == '6':
                test_read_all_status(ser)
            elif choice == '7':
                test_sensor_monitoring(ser)
            elif choice == '0':
                print("👋 종료합니다.")
                break
            else:
                print("❌ 잘못된 선택입니다.")
        
        ser.close()
        
    except serial.SerialException as e:
        print(f"❌ 시리얼 포트 오류: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\n⚠️ 사용자에 의해 중단되었습니다.")
        sys.exit(0)

if __name__ == "__main__":
    main()
