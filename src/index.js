// 페트컵 제어 포트 (COM8 RS-485)
let petcupPort = null;
let petcupReader = null;
let petcupWriter = null;

// 레이더 센서 포트 (COM9)
let radarPort = null;
let radarReader = null;
let radarWriter = null;

// XL430 서보 모터 포트 (COM5)
let servoPort = null;
let servoReader = null;
let servoWriter = null;

// ========================================
// Modbus RTU CRC16 계산 (레이더 센서용)
// ========================================

function calculateModbusCRC16(buffer) {
    let crc = 0xffff;
    for (let pos = 0; pos < buffer.length; pos++) {
        crc ^= buffer[pos];
        for (let i = 8; i !== 0; i--) {
            if ((crc & 0x0001) !== 0) {
                crc >>= 1;
                crc ^= 0xa001;
            } else {
                crc >>= 1;
            }
        }
    }
    // Swap bytes
    crc = ((crc & 0x00ff) << 8) | ((crc & 0xff00) >> 8);
    return crc;
}

// ========================================
// Dynamixel Protocol 2.0 구현
// ========================================

// Dynamixel ID는 HTML input에서 가져옴
function getDxlId() {
    return parseInt(document.getElementById('dxlId').value) || 1;
}

// Control Table 주소 (XL430-W250)
const ADDR_TORQUE_ENABLE = 64;
const ADDR_GOAL_POSITION = 116;
const ADDR_PRESENT_POSITION = 132;
const ADDR_MOVING = 122;

// 명령어
const INST_PING = 0x01;
const INST_READ = 0x02;
const INST_WRITE = 0x03;

// CRC 계산 함수
function updateCRC(crc_accum, data_blk) {
    const crc_table = [
        0x0000, 0x8005, 0x800f, 0x000a, 0x801b, 0x001e, 0x0014, 0x8011, 0x8033, 0x0036, 0x003c, 0x8039, 0x0028, 0x802d,
        0x8027, 0x0022, 0x8063, 0x0066, 0x006c, 0x8069, 0x0078, 0x807d, 0x8077, 0x0072, 0x0050, 0x8055, 0x805f, 0x005a,
        0x804b, 0x004e, 0x0044, 0x8041, 0x80c3, 0x00c6, 0x00cc, 0x80c9, 0x00d8, 0x80dd, 0x80d7, 0x00d2, 0x00f0, 0x80f5,
        0x80ff, 0x00fa, 0x80eb, 0x00ee, 0x00e4, 0x80e1, 0x00a0, 0x80a5, 0x80af, 0x00aa, 0x80bb, 0x00be, 0x00b4, 0x80b1,
        0x8093, 0x0096, 0x009c, 0x8099, 0x0088, 0x808d, 0x8087, 0x0082, 0x8183, 0x0186, 0x018c, 0x8189, 0x0198, 0x819d,
        0x8197, 0x0192, 0x01b0, 0x81b5, 0x81bf, 0x01ba, 0x81ab, 0x01ae, 0x01a4, 0x81a1, 0x01e0, 0x81e5, 0x81ef, 0x01ea,
        0x81fb, 0x01fe, 0x01f4, 0x81f1, 0x81d3, 0x01d6, 0x01dc, 0x81d9, 0x01c8, 0x81cd, 0x81c7, 0x01c2, 0x0140, 0x8145,
        0x814f, 0x014a, 0x815b, 0x015e, 0x0154, 0x8151, 0x8173, 0x0176, 0x017c, 0x8179, 0x0168, 0x816d, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012c, 0x8129, 0x0138, 0x813d, 0x8137, 0x0132, 0x0110, 0x8115, 0x811f, 0x011a, 0x810b, 0x010e,
        0x0104, 0x8101, 0x8303, 0x0306, 0x030c, 0x8309, 0x0318, 0x831d, 0x8317, 0x0312, 0x0330, 0x8335, 0x833f, 0x033a,
        0x832b, 0x032e, 0x0324, 0x8321, 0x0360, 0x8365, 0x836f, 0x036a, 0x837b, 0x037e, 0x0374, 0x8371, 0x8353, 0x0356,
        0x035c, 0x8359, 0x0348, 0x834d, 0x8347, 0x0342, 0x03c0, 0x83c5, 0x83cf, 0x03ca, 0x83db, 0x03de, 0x03d4, 0x83d1,
        0x83f3, 0x03f6, 0x03fc, 0x83f9, 0x03e8, 0x83ed, 0x83e7, 0x03e2, 0x83a3, 0x03a6, 0x03ac, 0x83a9, 0x03b8, 0x83bd,
        0x83b7, 0x03b2, 0x0390, 0x8395, 0x839f, 0x039a, 0x838b, 0x038e, 0x0384, 0x8381, 0x0280, 0x8285, 0x828f, 0x028a,
        0x829b, 0x029e, 0x0294, 0x8291, 0x82b3, 0x02b6, 0x02bc, 0x82b9, 0x02a8, 0x82ad, 0x82a7, 0x02a2, 0x82e3, 0x02e6,
        0x02ec, 0x82e9, 0x02f8, 0x82fd, 0x82f7, 0x02f2, 0x02d0, 0x82d5, 0x82df, 0x02da, 0x82cb, 0x02ce, 0x02c4, 0x82c1,
        0x8243, 0x0246, 0x024c, 0x8249, 0x0258, 0x825d, 0x8257, 0x0252, 0x0270, 0x8275, 0x827f, 0x027a, 0x826b, 0x026e,
        0x0264, 0x8261, 0x0220, 0x8225, 0x822f, 0x022a, 0x823b, 0x023e, 0x0234, 0x8231, 0x8213, 0x0216, 0x021c, 0x8219,
        0x0208, 0x820d, 0x8207, 0x0202,
    ];

    for (let j = 0; j < data_blk.length; j++) {
        let i = ((crc_accum >> 8) ^ data_blk[j]) & 0xff;
        crc_accum = ((crc_accum << 8) ^ crc_table[i]) & 0xffff;
    }
    return crc_accum;
}

// Dynamixel 패킷 생성
function makeDynamixelPacket(id, instruction, params) {
    const length = params.length + 3; // instruction(1) + params + CRC(2)

    let packet = [
        0xff,
        0xff,
        0xfd,
        0x00, // Header
        id, // ID
        length & 0xff, // Length Low
        (length >> 8) & 0xff, // Length High
        instruction, // Instruction
    ];

    // Parameters 추가
    packet = packet.concat(params);

    // CRC 계산
    const crc = updateCRC(0, packet);
    packet.push(crc & 0xff); // CRC Low
    packet.push((crc >> 8) & 0xff); // CRC High

    return new Uint8Array(packet);
}

// 토크 활성화/비활성화
function buildTorquePacket(id, enable) {
    return makeDynamixelPacket(id, INST_WRITE, [
        ADDR_TORQUE_ENABLE & 0xff,
        (ADDR_TORQUE_ENABLE >> 8) & 0xff,
        enable ? 1 : 0,
    ]);
}

// 위치 이동
function buildPositionPacket(id, position) {
    return makeDynamixelPacket(id, INST_WRITE, [
        ADDR_GOAL_POSITION & 0xff,
        (ADDR_GOAL_POSITION >> 8) & 0xff,
        position & 0xff,
        (position >> 8) & 0xff,
        (position >> 16) & 0xff,
        (position >> 24) & 0xff,
    ]);
}

// ========================================
// 페트컵 제어 (COM8 RS-485)
// ========================================

async function connectPetcup() {
    try {
        petcupPort = await navigator.serial.requestPort();
        await petcupPort.open({ baudRate: 9600 });

        const textDecoder = new TextDecoderStream();
        const readableStreamClosed = petcupPort.readable.pipeTo(textDecoder.writable);
        petcupReader = textDecoder.readable.getReader();
        petcupWriter = petcupPort.writable.getWriter();

        document.getElementById('connectPetcupBtn').disabled = true;
        document.getElementById('disconnectPetcupBtn').disabled = false;
        document.getElementById('petcupStatus').textContent = '연결됨';
        log('[페트컵] 포트 연결 성공');

        readPetcupData();
    } catch (error) {
        log('[페트컵] 연결 실패: ' + error.message);
    }
}

async function disconnectPetcup() {
    try {
        if (petcupReader) {
            await petcupReader.cancel();
            petcupReader = null;
        }
        if (petcupWriter) {
            await petcupWriter.close();
            petcupWriter = null;
        }
        if (petcupPort) {
            await petcupPort.close();
            petcupPort = null;
        }

        document.getElementById('connectPetcupBtn').disabled = false;
        document.getElementById('disconnectPetcupBtn').disabled = true;
        document.getElementById('petcupStatus').textContent = '연결 안 됨';
        log('[페트컵] 포트 연결 해제');
    } catch (error) {
        log('[페트컵] 연결 해제 실패: ' + error.message);
    }
}

async function readPetcupData() {
    let buffer = '';
    try {
        while (true) {
            const { value, done } = await petcupReader.read();
            if (done) break;

            buffer += value;
            let newlineIndex;
            while ((newlineIndex = buffer.indexOf('\n')) !== -1) {
                const line = buffer.substring(0, newlineIndex).trim();
                if (line.length > 0) {
                    log('[페트컵] ' + line);
                }
                buffer = buffer.substring(newlineIndex + 1);
            }
        }
    } catch (error) {
        log('[페트컵] 수신 오류: ' + error.message);
    }
}

async function writeToPetcup(data) {
    if (!petcupWriter) {
        log('[페트컵] 포트가 연결되지 않았습니다.');
        return;
    }
    try {
        const encoder = new TextEncoder();
        const encodedData = encoder.encode(data + '\n');
        await petcupWriter.write(encodedData);
        await new Promise((resolve) => setTimeout(resolve, 100));
    } catch (error) {
        log('[페트컵] 전송 오류: ' + error.message);
    }
}

// ========================================
// 레이더 센서 제어 (COM9 - Modbus RTU)
// ========================================

let radarMonitoring = false; // 연속 모니터링 플래그

async function connectRadar() {
    try {
        radarPort = await navigator.serial.requestPort();
        // 보드레이트를 9600으로 시도 (일부 레이더 센서는 9600 사용)
        await radarPort.open({ baudRate: 9600 });

        // 바이너리 통신을 위한 Raw 스트림 사용
        radarReader = radarPort.readable.getReader();
        radarWriter = radarPort.writable.getWriter();

        document.getElementById('connectRadarBtn').disabled = true;
        document.getElementById('disconnectRadarBtn').disabled = false;
        document.getElementById('radarStatus').textContent = '연결됨';
        log('[레이더] 포트 연결 성공 (9600 baud)');
        log('[레이더] 자동 수신 데이터 모니터링 시작...');

        // 백그라운드로 연속 데이터 수신 시작
        radarMonitoring = true;
        monitorRadarData();
    } catch (error) {
        log('[레이더] 연결 실패: ' + error.message);
    }
}

async function disconnectRadar() {
    try {
        radarMonitoring = false; // 모니터링 중지

        if (radarReader) {
            await radarReader.cancel();
            radarReader = null;
        }
        if (radarWriter) {
            await radarWriter.close();
            radarWriter = null;
        }
        if (radarPort) {
            await radarPort.close();
            radarPort = null;
        }

        document.getElementById('connectRadarBtn').disabled = false;
        document.getElementById('disconnectRadarBtn').disabled = true;
        document.getElementById('radarStatus').textContent = '연결 안 됨';
        log('[레이더] 포트 연결 해제');
    } catch (error) {
        log('[레이더] 연결 해제 실패: ' + error.message);
    }
}

// 레이더 센서 자동 수신 모니터링 (연속 모드)
async function monitorRadarData() {
    log('[레이더] 연속 모니터링 시작 - 센서가 보내는 모든 데이터 출력');

    try {
        while (radarMonitoring && radarReader) {
            const { value, done } = await Promise.race([
                radarReader.read(),
                new Promise((resolve) => setTimeout(() => resolve({ value: null, done: false, timeout: true }), 2000)),
            ]);

            if (!radarMonitoring) break;
            if (done) {
                log('[레이더] 연결 종료');
                break;
            }

            if (value && value.length > 0) {
                const hexString = Array.from(value)
                    .map((b) => '0x' + b.toString(16).padStart(2, '0'))
                    .join(' ');
                log(`[레이더 수신] ${value.length}바이트: ${hexString}`);

                // 간단한 자동 파싱 시도 (패킷이 충분하면)
                if (value.length >= 7 && value[0] === 0x01 && value[1] === 0x03 && value[2] === 0x02) {
                    const distance = value[3] * 256 + value[4];
                    log(`[레이더 자동파싱] 거리: ${distance}mm (${(distance / 10).toFixed(1)}cm)`);
                    document.getElementById('radarData').textContent =
                        `거리: ${distance}mm (${(distance / 10).toFixed(1)}cm)`;
                }
            }

            // CPU 부하 방지
            await new Promise((resolve) => setTimeout(resolve, 10));
        }
    } catch (error) {
        log('[레이더] 모니터링 오류: ' + error.message);
    }
}

async function readRadarDistance() {
    if (!radarReader || !radarWriter) {
        log('[레이더] 포트가 연결되지 않았습니다.');
        return null;
    }

    try {
        // Modbus RTU 명령: {0x01, 0x03, 0x01, 0x01, 0x00, 0x01, 0xd4, 0x36}
        const command = new Uint8Array([0x01, 0x03, 0x01, 0x01, 0x00, 0x01, 0xd4, 0x36]);

        // 아두이노 코드처럼: 명령 전송
        await radarWriter.write(command);
        log('[레이더] 거리 측정 명령 전송');

        // 명령 전송 후 대기 (아두이노: delay(10))
        await new Promise((resolve) => setTimeout(resolve, 20));

        // 응답 읽기 (아두이노의 readN 함수와 유사하게)
        const responseData = new Uint8Array(7); // [0x01, 0x03, 0x02, DATA_H, DATA_L, CRC_L, CRC_H]
        let offset = 0;
        const startTime = Date.now();
        const timeout = 500; // 아두이노와 동일하게 500ms

        while (offset < 7) {
            const remainingTime = timeout - (Date.now() - startTime);
            if (remainingTime <= 0) {
                log('[레이더] 응답 타임아웃 (500ms 초과)');
                break;
            }

            // Promise.race로 read()와 타임아웃을 경쟁시킴
            const result = await Promise.race([
                radarReader.read(),
                new Promise((resolve) =>
                    setTimeout(() => resolve({ value: null, done: false, timeout: true }), remainingTime),
                ),
            ]);

            if (result.timeout) {
                log('[레이더] 응답 없음 (타임아웃)');
                break;
            }

            if (result.done) {
                log('[레이더] 연결이 종료되었습니다.');
                break;
            }

            if (result.value && result.value.length > 0) {
                log(
                    `[레이더] 수신: ${result.value.length} 바이트 - ${Array.from(result.value)
                        .map((b) => '0x' + b.toString(16).padStart(2, '0'))
                        .join(' ')}`,
                );

                // 받은 데이터를 responseData에 복사
                const bytesToCopy = Math.min(result.value.length, 7 - offset);
                responseData.set(result.value.slice(0, bytesToCopy), offset);
                offset += bytesToCopy;

                // 7바이트 모두 받았으면 파싱
                if (offset >= 7) {
                    log(
                        `[레이더] 완전한 패킷 수신: ${Array.from(responseData)
                            .map((b) => '0x' + b.toString(16).padStart(2, '0'))
                            .join(' ')}`,
                    );

                    // 응답 패킷 구조 확인: [0x01, 0x03, 0x02, DATA_H, DATA_L, CRC_L, CRC_H]
                    if (responseData[0] === 0x01 && responseData[1] === 0x03 && responseData[2] === 0x02) {
                        // CRC 검증 (아두이노: Data[5] * 256 + Data[6])
                        const receivedCRC = responseData[5] * 256 + responseData[6];
                        const calculatedCRC = calculateModbusCRC16(responseData.slice(0, 5));

                        if (receivedCRC === calculatedCRC) {
                            // 거리 계산 (아두이노: Data[3] * 256 + Data[4])
                            const distance = responseData[3] * 256 + responseData[4];
                            log(`[레이더] 거리: ${distance}mm (${(distance / 10).toFixed(1)}cm)`);
                            document.getElementById('radarData').textContent =
                                `거리: ${distance}mm (${(distance / 10).toFixed(1)}cm)`;
                            return distance;
                        } else {
                            log(
                                `[레이더] CRC 오류: 수신=0x${receivedCRC.toString(16)}, 계산=0x${calculatedCRC.toString(16)}`,
                            );
                        }
                    } else {
                        log(
                            `[레이더] 잘못된 패킷 헤더: ${Array.from(responseData.slice(0, 3))
                                .map((b) => '0x' + b.toString(16).padStart(2, '0'))
                                .join(' ')}`,
                        );
                    }
                    break;
                }
            }
        }

        if (offset === 0) {
            log('[레이더] 응답 없음 (타임아웃) - 센서 연결 및 전원을 확인하세요');
        } else if (offset < 7) {
            log(`[레이더] 불완전한 응답: ${offset}/7 바이트 수신`);
        }

        return null;
    } catch (error) {
        log('[레이더] 수신 오류: ' + error.message);
        return null;
    }
}

// 레이더 센서는 이제 readRadarDistance() 함수로 직접 측정

// ========================================
// XL430 서보 모터 제어 (COM5)
// ========================================

async function connectServo() {
    try {
        servoPort = await navigator.serial.requestPort();
        await servoPort.open({ baudRate: 57600 }); // Dynamixel 기본 보드레이트

        // Dynamixel은 바이너리 통신이므로 Raw 스트림 사용
        servoReader = servoPort.readable.getReader();
        servoWriter = servoPort.writable.getWriter();

        document.getElementById('connectServoBtn').disabled = true;
        document.getElementById('disconnectServoBtn').disabled = false;
        document.getElementById('servoStatus').textContent = '연결됨';
        log('[서보] XL430 연결 성공 (Dynamixel Protocol 2.0, 57600 baud)');

        readServoData();
    } catch (error) {
        log('[서보] 연결 실패: ' + error.message);
    }
}

async function disconnectServo() {
    try {
        if (servoReader) {
            await servoReader.cancel();
            servoReader = null;
        }
        if (servoWriter) {
            await servoWriter.close();
            servoWriter = null;
        }
        if (servoPort) {
            await servoPort.close();
            servoPort = null;
        }

        document.getElementById('connectServoBtn').disabled = false;
        document.getElementById('disconnectServoBtn').disabled = true;
        document.getElementById('servoStatus').textContent = '연결 안 됨';
        log('[서보] 포트 연결 해제');
    } catch (error) {
        log('[서보] 연결 해제 실패: ' + error.message);
    }
}

async function readServoData() {
    try {
        while (true) {
            const { value, done } = await servoReader.read();
            if (done) break;

            // Dynamixel 응답 패킷 파싱 (간단한 버전)
            // 실제로는 헤더 확인, CRC 검증 등이 필요하지만 여기서는 생략
            if (value && value.length >= 11) {
                // Status Packet: [0xFF 0xFF 0xFD 0x00 ID LEN_L LEN_H INST ERR PARAM... CRC_L CRC_H]
                const header = `${value[0].toString(16)} ${value[1].toString(16)} ${value[2].toString(16)}`;
                const id = value[4];
                const error = value[8];

                if (error === 0) {
                    log(`[서보] 응답: ID=${id}, 성공`);
                    document.getElementById('servoData').textContent = `서보 ID ${id}: 명령 성공`;
                } else {
                    log(`[서보] 응답: ID=${id}, 에러=${error}`);
                    document.getElementById('servoData').textContent = `서보 ID ${id}: 에러 ${error}`;
                }
            }
        }
    } catch (error) {
        log('[서보] 수신 오류: ' + error.message);
    }
}

async function writeToServo(packetArray) {
    if (!servoWriter) {
        log('[서보] 포트가 연결되지 않았습니다.');
        return;
    }
    try {
        await servoWriter.write(packetArray);
        await new Promise((resolve) => setTimeout(resolve, 50));
    } catch (error) {
        log('[서보] 전송 오류: ' + error.message);
    }
}

// ========================================
// 명령 함수
// ========================================

function sendCommand(cmd) {
    const deviceId = document.getElementById('deviceId').value;
    const command = `${deviceId}:${cmd}`;
    displayCommand(command);
    log(`[전송] ${command}`);
    writeToPetcup(command);
}

function setSpeed() {
    const deviceId = document.getElementById('deviceId').value;
    const speedOpen = document.getElementById('speedOpen').value;
    const speedClose = document.getElementById('speedClose').value;
    const command = `${deviceId}:SETSPEED:${speedOpen}:${speedClose}`;
    displayCommand(command);
    log(`[전송] ${command}`);
    writeToPetcup(command);
}

// 레이더 명령
async function requestRadarData() {
    // 연속 모니터링 중에는 명령 모드로 전환
    if (radarMonitoring) {
        log('[레이더] 연속 모니터링 중... 명령 전송 모드는 연결 해제 후 재연결 필요');
        return;
    }
    await readRadarDistance();
}

// XL430 서보 명령 (ID 2)
function moveForward() {
    const id = 2;
    const angle = 268.3;
    // 각도를 위치값으로 변환 (0-360° → 0-4095)
    const position = Math.round((angle / 360) * 4095);
    const packet = buildPositionPacket(id, position);
    log(`[서보] ID ${id} 앞으로 이동: ${angle}° → 위치 ${position}`);
    writeToServo(packet);
    document.getElementById('servoData').textContent = `서보: 앞으로 (${angle}°)`;
}

function moveBackward() {
    const id = 2;
    const angle = 323;
    // 각도를 위치값으로 변환 (0-360° → 0-4095)
    const position = Math.round((angle / 360) * 4095);
    const packet = buildPositionPacket(id, position);
    log(`[서보] ID ${id} 뒤로 이동: ${angle}° → 위치 ${position}`);
    writeToServo(packet);
    document.getElementById('servoData').textContent = `서보: 뒤로 (${angle}°)`;
}

// 그리퍼 명령 (ID 1)
function openGripper() {
    const id = 1;
    const angle = 160;
    const position = Math.round((angle / 360) * 4095);
    const packet = buildPositionPacket(id, position);
    log(`[그리퍼] ID ${id} 열기: ${angle}° → 위치 ${position}`);
    writeToServo(packet);
    document.getElementById('gripperData').textContent = `그리퍼: 열기 (${angle}°)`;
}

function closeGripper() {
    const id = 1;
    const angle = 184;
    const position = Math.round((angle / 360) * 4095);
    const packet = buildPositionPacket(id, position);
    log(`[그리퍼] ID ${id} 닫기: ${angle}° → 위치 ${position}`);
    writeToServo(packet);
    document.getElementById('gripperData').textContent = `그리퍼: 닫기 (${angle}°) - 물체 감지 시 자동 정지`;
}

function enableGripper() {
    const id = 1;
    const packet = buildTorquePacket(id, true);
    log(`[그리퍼] ID ${id} 토크 활성화`);
    writeToServo(packet);
    document.getElementById('gripperData').textContent = `그리퍼 ID ${id}: 토크 ON`;
}

function disableGripper() {
    const id = 1;
    const packet = buildTorquePacket(id, false);
    log(`[그리퍼] ID ${id} 토크 비활성화`);
    writeToServo(packet);
    document.getElementById('gripperData').textContent = `그리퍼 ID ${id}: 토크 OFF`;
}

function enableServo() {
    const id = 2;
    const packet = buildTorquePacket(id, true);
    log(`[서보] ID ${id} 토크 활성화`);
    writeToServo(packet);
    document.getElementById('servoData').textContent = `서보 ID ${id}: 토크 ON`;
}

function disableServo() {
    const id = 2;
    const packet = buildTorquePacket(id, false);
    log(`[서보] ID ${id} 토크 비활성화`);
    writeToServo(packet);
    document.getElementById('servoData').textContent = `서보 ID ${id}: 토크 OFF`;
}

function displayCommand(cmd) {
    document.getElementById('commandOutput').textContent = cmd;
}

function log(message) {
    const logArea = document.getElementById('log');
    const timestamp = new Date().toLocaleTimeString();
    logArea.value += `[${timestamp}] ${message}\n`;
    logArea.scrollTop = logArea.scrollHeight;
}

function clearLog() {
    document.getElementById('log').value = '';
}

// 초기 메시지
log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
log('페트컵 통합 제어 시스템 v2.0');
log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
log('');
log('📡 포트 연결 정보:');
log('  - 페트컵 제어: COM8 (RS-485, 9600 baud)');
log('  - 레이더 센서: COM9 (자동 감지, 9600 baud 시도)');
log('  - XL430 서보: COM5 (Dynamixel Protocol 2.0, 57600 baud)');
log('');
log('💡 사용 방법:');
log('  1. 각 포트를 순서대로 연결하세요');
log('  2. 레이더 센서: 연결 시 자동으로 수신 데이터 모니터링');
log('  3. XL430 사용 전 먼저 "토크 ON" 버튼 클릭');
log('  4. 서보 ID가 1이 아닌 경우 Dynamixel ID 변경');
log('');
log('🔧 명령 형식 (페트컵): <ID>:<CMD>:<PARAM>');
log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');

// Web Serial API 지원 확인
if (!('serial' in navigator)) {
    log('⚠️ 경고: 이 브라우저는 Web Serial API를 지원하지 않습니다.');
    log('Chrome 또는 Edge 브라우저를 사용하세요.');
}
