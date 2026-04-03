// 페트컵 제어 포트
let petcupPort = null;
let petcupReader = null;
let petcupWriter = null;

// 레이더/서보 포트
let radarPort = null;
let radarReader = null;
let radarWriter = null;

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
        await new Promise(resolve => setTimeout(resolve, 100));
    } catch (error) {
        log('[페트컵] 전송 오류: ' + error.message);
    }
}

// ========================================
// 레이더/서보 제어 (COM9)
// ========================================

async function connectRadar() {
    try {
        radarPort = await navigator.serial.requestPort();
        await radarPort.open({ baudRate: 9600 });

        const textDecoder = new TextDecoderStream();
        const readableStreamClosed = radarPort.readable.pipeTo(textDecoder.writable);
        radarReader = textDecoder.readable.getReader();
        radarWriter = radarPort.writable.getWriter();

        document.getElementById('connectRadarBtn').disabled = true;
        document.getElementById('disconnectRadarBtn').disabled = false;
        document.getElementById('radarStatus').textContent = '연결됨';
        log('[레이더] 포트 연결 성공');

        readRadarData();
    } catch (error) {
        log('[레이더] 연결 실패: ' + error.message);
    }
}

async function disconnectRadar() {
    try {
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

async function readRadarData() {
    let buffer = '';
    try {
        while (true) {
            const { value, done } = await radarReader.read();
            if (done) break;

            buffer += value;
            let newlineIndex;
            while ((newlineIndex = buffer.indexOf('\n')) !== -1) {
                const line = buffer.substring(0, newlineIndex).trim();
                if (line.length > 0) {
                    log('[레이더] ' + line);
                    // 레이더 데이터 표시
                    if (line.startsWith('RADAR:')) {
                        document.getElementById('radarData').textContent = '레이더: ' + line.substring(6);
                    }
                }
                buffer = buffer.substring(newlineIndex + 1);
            }
        }
    } catch (error) {
        log('[레이더] 수신 오류: ' + error.message);
    }
}

async function writeToRadar(data) {
    if (!radarWriter) {
        log('[레이더] 포트가 연결되지 않았습니다.');
        return;
    }
    try {
        const encoder = new TextEncoder();
        const encodedData = encoder.encode(data + '\n');
        await radarWriter.write(encodedData);
        await new Promise(resolve => setTimeout(resolve, 50));
    } catch (error) {
        log('[레이더] 전송 오류: ' + error.message);
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

function sendServoCommand() {
    const angle = document.getElementById('servoAngle').value;
    const command = `SERVO:${angle}`;
    log(`[전송 서보] ${command}`);
    writeToRadar(command);
}

function requestRadarData() {
    const command = 'RADAR:READ';
    log(`[전송 레이더] ${command}`);
    writeToRadar(command);
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
log('페트컵 통합 제어 시스템 시작');
log('명령 형식: <ID>:<CMD>:<PARAM>');
log('포트 연결 필요: 페트컵(COM8), 레이더/서보(COM9)');

// Web Serial API 지원 확인
if (!('serial' in navigator)) {
    log('경고: 이 브라우저는 Web Serial API를 지원하지 않습니다.');
    log('Chrome/Edge 브라우저를 사용하세요.');
}
