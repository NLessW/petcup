// ========================================
// PETMON 자동 분류 시스템
// ========================================

// 메인 컨트롤러 포트 (RS-485)
// VID_0403+PID_6001+A5069RR4A\0000
let mainPort = null;
let mainReader = null;
let mainWriter = null;

// 서보 모터 포트 (Dynamixel)
// VID_0403+PID_6001+AL01QFACA\0000
let servoPort = null;
let servoReader = null;
let servoWriter = null;

// 시스템 상태
let isProcessing = false;
let processStep = 0;
let waitingForConfirmation = false;
let totalSteps = 10; // 기본 10단계 (손 감지 시 +1)

// ========================================
// Dynamixel Protocol 2.0 구현
// ========================================

// Control Table 주소 (XL430-W250)
const ADDR_TORQUE_ENABLE = 64;
const ADDR_GOAL_POSITION = 116;

// 명령어
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
// 포트 연결 함수
// ========================================

async function connectMainController() {
    try {
        // VID_0403+PID_6001+A5069RR4A\0000로 필터링
        const ports = await navigator.serial.getPorts();
        let targetPort = null;

        for (const port of ports) {
            const info = port.getInfo();
            if (info.usbVendorId === 0x0403 && info.usbProductId === 0x6001) {
                // Serial Number 확인하려면 추가 로직 필요
                // 일단 첫 번째 매칭되는 포트 사용
                targetPort = port;
                break;
            }
        }

        if (!targetPort) {
            log('[메인] 포트를 찾을 수 없습니다. 포트를 선택해주세요...');
            // 포트가 없으면 사용자에게 선택 요청
            targetPort = await navigator.serial.requestPort({
                filters: [{ usbVendorId: 0x0403, usbProductId: 0x6001 }],
            });
        }

        mainPort = targetPort;
        await mainPort.open({ baudRate: 9600 });

        const textDecoder = new TextDecoderStream();
        mainPort.readable.pipeTo(textDecoder.writable);
        mainReader = textDecoder.readable.getReader();
        mainWriter = mainPort.writable.getWriter();

        log('[메인] RS-485 컨트롤러 연결 성공 (9600 baud)');

        readMainData();
        return true;
    } catch (error) {
        log('[메인] 연결 실패: ' + error.message);
        return false;
    }
}

async function connectServoController() {
    try {
        // VID_0403+PID_6001+AL01QFACA\0000로 필터링
        const ports = await navigator.serial.getPorts();
        let targetPort = null;

        for (const port of ports) {
            const info = port.getInfo();
            if (info.usbVendorId === 0x0403 && info.usbProductId === 0x6001) {
                // Main과 다른 포트 찾기
                if (port !== mainPort) {
                    targetPort = port;
                    break;
                }
            }
        }

        if (!targetPort) {
            log('[서보] 포트를 찾을 수 없습니다. 포트를 선택해주세요...');
            targetPort = await navigator.serial.requestPort({
                filters: [{ usbVendorId: 0x0403, usbProductId: 0x6001 }],
            });
        }

        servoPort = targetPort;
        await servoPort.open({ baudRate: 57600 });

        servoReader = servoPort.readable.getReader();
        servoWriter = servoPort.writable.getWriter();

        log('[서보] Dynamixel 컨트롤러 연결 성공 (57600 baud)');

        readServoData();
        return true;
    } catch (error) {
        log('[서보] 연결 실패: ' + error.message);
        return false;
    }
}

async function readMainData() {
    let buffer = '';
    try {
        while (true) {
            const { value, done } = await mainReader.read();
            if (done) break;

            buffer += value;
            let newlineIndex;
            while ((newlineIndex = buffer.indexOf('\n')) !== -1) {
                const line = buffer.substring(0, newlineIndex).trim();
                if (line.length > 0) {
                    log('[메인] ' + line);
                    // HAND 명령 응답 파싱
                    if (line.includes('HAND:DETECTED')) {
                        lastHandResponse = 'DETECTED';
                    } else if (line.includes('HAND:CLEAR')) {
                        lastHandResponse = 'CLEAR';
                    }
                }
                buffer = buffer.substring(newlineIndex + 1);
            }
        }
    } catch (error) {
        log('[메인] 수신 오류: ' + error.message);
    }
}

async function readServoData() {
    try {
        while (true) {
            const { value, done } = await servoReader.read();
            if (done) break;
            // 서보 응답 처리 (필요시)
        }
    } catch (error) {
        log('[서보] 수신 오류: ' + error.message);
    }
}

// ========================================
// 명령 전송 함수
// ========================================

async function sendMainCommand(cmd) {
    if (!mainWriter) {
        log('[메인] 포트가 연결되지 않았습니다.');
        return false;
    }
    try {
        const encoder = new TextEncoder();
        const command = `1:${cmd}`;
        const encodedData = encoder.encode(command + '\n');
        await mainWriter.write(encodedData);
        log(`[전송] ${command}`);
        await delay(100);
        return true;
    } catch (error) {
        log('[메인] 전송 오류: ' + error.message);
        return false;
    }
}

// 손 감지 확인 함수
let lastHandResponse = 'CLEAR';

async function checkHandDetection() {
    if (!mainWriter) {
        return false;
    }
    try {
        lastHandResponse = 'CLEAR'; // 초기화
        const encoder = new TextEncoder();
        const command = '1:HAND';
        const encodedData = encoder.encode(command + '\n');
        await mainWriter.write(encodedData);
        log('[전송] 1:HAND');
        await delay(300); // 응답 대기
        // lastHandResponse가 readMainData에서 업데이트됨
        return lastHandResponse === 'DETECTED';
    } catch (error) {
        log('[손감지] 확인 오류: ' + error.message);
        return false;
    }
}

async function sendServoCommand(packetArray) {
    if (!servoWriter) {
        log('[서보] 포트가 연결되지 않았습니다.');
        return false;
    }
    try {
        await servoWriter.write(packetArray);
        await delay(100);
        return true;
    } catch (error) {
        log('[서보] 전송 오류: ' + error.message);
        return false;
    }
}

// ========================================
// 서보 모터 제어 함수
// ========================================

async function enableTorque(id) {
    const packet = buildTorquePacket(id, true);
    log(`[서보] ID ${id} 토크 활성화`);
    return await sendServoCommand(packet);
}

async function moveGripper(open) {
    const id = 1;
    const angle = open ? 160 : 184;
    const position = Math.round((angle / 360) * 4095);
    const packet = buildPositionPacket(id, position);
    log(`[그리퍼] ${open ? '열기' : '닫기'}: ${angle}° → 위치 ${position}`);
    return await sendServoCommand(packet);
}

async function moveServo(forward) {
    const id = 2;
    const angle = forward ? 268.3 : 323;
    const position = Math.round((angle / 360) * 4095);
    const packet = buildPositionPacket(id, position);
    log(`[서보] ${forward ? '앞으로' : '뒤로'} 이동: ${angle}° → 위치 ${position}`);
    return await sendServoCommand(packet);
}

// ========================================
// UI 업데이트 함수
// ========================================

function updateDateTime() {
    const now = new Date();
    const dateStr = now.toLocaleDateString('ko-KR', {
        year: 'numeric',
        month: '2-digit',
        day: '2-digit',
        weekday: 'long',
    });
    const timeStr = now.toLocaleTimeString('ko-KR');
    document.getElementById('datetime').textContent = `${dateStr} ${timeStr}`;
}

setInterval(updateDateTime, 1000);
updateDateTime();

function showProcessScreen() {
    document.getElementById('mainScreen').style.display = 'none';
    document.getElementById('processScreen').classList.add('active');
    document.getElementById('emergencyBtn').style.display = 'block';
}

function hideProcessScreen() {
    document.getElementById('mainScreen').style.display = 'block';
    document.getElementById('processScreen').classList.remove('active');
    document.getElementById('emergencyBtn').style.display = 'none';
}

function updateProcessStep(step, icon, title, description) {
    document.getElementById('processIcon').textContent = icon;
    document.getElementById('processTitle').textContent = title;
    document.getElementById('processDescription').textContent = description;
    document.getElementById('processProgress').textContent = `${step} / ${totalSteps}`;
}

function showConfirmButton() {
    document.getElementById('confirmButton').style.display = 'block';
}

function hideConfirmButton() {
    document.getElementById('confirmButton').style.display = 'none';
}

function toggleLog() {
    const logPopup = document.getElementById('logPopup');
    logPopup.classList.toggle('active');
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

function delay(ms) {
    return new Promise((resolve) => setTimeout(resolve, ms));
}

// ========================================
// 시스템 초기화
// ========================================

async function initializeSystem() {
    log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
    log('PETMON 시스템 초기화 시작');
    log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');

    const systemBox = document.getElementById('systemBox');
    const systemStatusText = document.getElementById('systemStatusText');
    systemStatusText.textContent = '초기화 중...';
    systemBox.classList.add('disabled');

    // 메인 컨트롤러 연결
    log('📦 메인 컨트롤러 연결 중...');
    const mainConnected = await connectMainController();
    if (!mainConnected) {
        log('❌ 메인 컨트롤러 연결 실패');
        systemStatusText.textContent = '연결 실패';
        return;
    }

    await delay(500);

    // 서보 컨트롤러 연결
    log('🤖 서보 컨트롤러 연결 중...');
    const servoConnected = await connectServoController();
    if (!servoConnected) {
        log('❌ 서보 컨트롤러 연결 실패');
        systemStatusText.textContent = '연결 실패';
        return;
    }

    await delay(500);

    // 서보 모터 토크 활성화
    log('⚙️ 서보 모터 초기화 중...');
    await enableTorque(1); // 그리퍼
    await delay(300);
    await enableTorque(2); // 메인 서보
    await delay(300);

    log('✅ 시스템 초기화 완료');
    log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');

    systemStatusText.textContent = '준비 완료';
    systemBox.classList.remove('disabled');
    document.getElementById('startButton').disabled = false;

    return true;
}

// ========================================
// 자동 프로세스
// ========================================

async function startProcess() {
    if (isProcessing) {
        log('⚠️ 프로세스가 이미 실행 중입니다.');
        return;
    }

    // 시스템 연결 상태 확인 및 자동 연결
    if (!mainWriter || !servoWriter) {
        log('🔌 시스템이 연결되지 않았습니다. 자동 연결을 시작합니다...');
        const initialized = await initializeSystem();
        if (!initialized && (!mainWriter || !servoWriter)) {
            log('❌ 시스템 연결에 실패했습니다. 프로세스를 시작할 수 없습니다.');
            alert('시스템 연결에 실패했습니다. 하드웨어 연결을 확인하세요.');
            return;
        }
        await delay(1000);
    }

    isProcessing = true;
    processStep = 0;

    showProcessScreen();

    log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
    log('🚀 자동 프로세스 시작');
    log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');

    try {
        // UV와 FAN 켜기
        log('💡 UV 라이트 및 팬 가동 중...');
        await sendMainCommand('UV:ON');
        await sendMainCommand('FAN:ON');
        await delay(500);

        // 1단계: 투입구 열림
        processStep = 1;
        updateProcessStep(processStep, '🚪', '투입구 열기', '문이 열리고 있습니다...');
        log(`[${processStep}/${totalSteps}] 투입구 열기...`);
        await sendMainCommand('OPEN');
        await delay(3000);

        // 2단계: 그리퍼 열림
        processStep = 2;
        updateProcessStep(processStep, '🤏', '그리퍼 준비', '그리퍼가 펼쳐지고 있습니다...');
        log(`[${processStep}/${totalSteps}] 그리퍼 열기...`);
        await moveGripper(true);
        await delay(1500);

        // 3단계: 투입 완료 대기
        processStep = 3;
        updateProcessStep(processStep, '📦', 'PET병 투입', 'PET병을 투입구에 넣어주세요');
        log(`[${processStep}/${totalSteps}] 투입 완료 대기 중...`);
        showConfirmButton();
        waitingForConfirmation = true;
    } catch (error) {
        log('❌ 프로세스 실행 중 오류: ' + error.message);
        stopProcess();
    }
}

async function confirmInsertion() {
    if (!waitingForConfirmation) return;

    waitingForConfirmation = false;
    hideConfirmButton();

    try {
        // 4단계: 그리퍼 닫기
        processStep = 4;
        updateProcessStep(processStep, '✊', '컵 잡기', '컵을 잡고 있습니다...');
        log(`[${processStep}/${totalSteps}] 그리퍼 닫기...`);
        await moveGripper(false);
        await delay(1500);

        // 손 감지 확인 (petmon.ino 방식)
        log('손 감지 센서 확인 중...');
        const isHandDetected = await checkHandDetection();

        if (isHandDetected) {
            // 손이 감지되면 → 경고 단계 추가
            processStep = 5;
            totalSteps = 11; // 손 빼기 단계 포함
            updateProcessStep(processStep, '✋', '손 감지!', '투입구에서 손을 빼주세요!');
            log(`[${processStep}/${totalSteps}] *** 손 감지! 손을 빼주세요 ***`);
            await delay(3000); // 손을 뺄 시간 제공
            processStep = 6;
        } else {
            // 손이 감지되지 않으면 → 바로 문 닫기
            log('손 감지 안됨 - 안전 확인 완료');
            processStep = 5; // 손 빼기 단계 없이 진행
            totalSteps = 10; // 기본 10단계
        }

        // 문 닫기 단계
        processStep = processStep;
        updateProcessStep(processStep, '🚪', '투입구 닫기', '투입구를 닫고 있습니다...');
        log(`[${processStep}/${totalSteps}] 투입구 닫기...`);
        await sendMainCommand('CLOSE');
        await delay(3000);

        // 물 분사 단계
        processStep = processStep + 1;
        updateProcessStep(processStep, '💧', '세척 중', '깨끗하게 세척하고 있습니다...');
        log(`[${processStep}/${totalSteps}] 물 분사 시작...`);
        await sendMainCommand('PUMP:ON');
        await delay(3000);
        await sendMainCommand('PUMP:OFF');
        log('물 분사 완료');

        // 서보 모터 뒤로 이동
        processStep = processStep + 1;
        updateProcessStep(processStep, '🔄', '이동 중', '배출 위치로 이동하고 있습니다...');
        log(`[${processStep}/${totalSteps}] 서보 모터 뒤로 이동...`);
        await moveServo(false);
        await delay(2000);

        // 그리퍼 열고 배출
        processStep = processStep + 1;
        updateProcessStep(processStep, '📤', '배출 중', '컵을 배출하고 있습니다...');
        log(`[${processStep}/${totalSteps}] 그리퍼 열기 (배출)...`);
        await moveGripper(true);
        await delay(2000);

        // 그리퍼 닫기
        processStep = processStep + 1;
        updateProcessStep(processStep, '🔄', '정리 중', '투입하신 컵을 정리중입니다...');
        log(`[${processStep}/${totalSteps}] 그리퍼 닫기...`);
        await moveGripper(false);
        await delay(1500);

        // 서보 모터 앞으로 (초기 위치)
        processStep = processStep + 1;
        updateProcessStep(processStep, '🏠', '복귀 중', '초기 위치로 돌아가고 있습니다...');
        log(`[${processStep}/${totalSteps}] 서보 모터 앞으로 이동...`);
        await moveServo(true);
        await delay(2000);

        // 프로세스 완료
        updateProcessStep(totalSteps, '✅', '완료!', '감사합니다. 포인트가 적립되었습니다.');
        log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
        log('✅ 프로세스 완료!');
        log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');

        // UV, FAN, 인버터 끄기 (딜레이를 두고 순차적으로 전송)
        // MC12B(인버터 전원)는 계속 켜두고 FWD 신호만 끔
        log('💡 UV 라이트, 팬 정지 및 FWD 신호 OFF...');
        await sendMainCommand('UV:OFF');
        await delay(200);
        await sendMainCommand('FAN:OFF');
        await delay(200);
        await sendMainCommand('FWD:OFF');

        await delay(3000);
        isProcessing = false;
        hideProcessScreen();
        document.getElementById('startButton').disabled = false;
    } catch (error) {
        log('❌ 프로세스 실행 중 오류: ' + error.message);
        stopProcess();
    }
}

async function emergencyStop() {
    if (!confirm('긴급 정지하시겠습니까?')) {
        return;
    }

    log('⚠️ 긴급 정지!');
    isProcessing = false;
    waitingForConfirmation = false;
    processStep = 0;

    hideProcessScreen();
    hideConfirmButton();
    document.getElementById('startButton').disabled = false;

    // 긴급 정지: 모든 모터 및 장치 정지 (딜레이를 두고 순차적으로 전송)
    // MC12B(인버터 전원)는 끄지 않음
    if (mainWriter) {
        await sendMainCommand('STOP');
        await delay(200);
        await sendMainCommand('PUMP:OFF');
        await delay(200);
        await sendMainCommand('UV:OFF');
        await delay(200);
        await sendMainCommand('FAN:OFF');
        await delay(200);
        await sendMainCommand('FWD:OFF');
    }
}

// ========================================
// 초기 로그
// ========================================

log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
log('PETMON 자동 분류 시스템 v3.0');
log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
log('💡 시스템 준비 완료');
log('🚀 "시작하기" 버튼을 눌러 프로세스를 시작하세요');
log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');

// Web Serial API 지원 확인
if (!('serial' in navigator)) {
    log('⚠️ 경고: 이 브라우저는 Web Serial API를 지원하지 않습니다.');
    log('Chrome 또는 Edge 브라우저를 사용하세요.');
    document.getElementById('operationStatus').textContent = '지원 안됨';
    document.getElementById('operationStatus').style.color = '#e74c3c';
}
