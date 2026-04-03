let port = null;
let reader = null;
let writer = null;

async function connectSerial() {
    try {
        // Web Serial API로 포트 선택
        port = await navigator.serial.requestPort();
        await port.open({ baudRate: 9600 });

        const textDecoder = new TextDecoderStream();
        const readableStreamClosed = port.readable.pipeTo(textDecoder.writable);
        reader = textDecoder.readable.getReader();

        writer = port.writable.getWriter();

        document.getElementById('connectBtn').disabled = true;
        document.getElementById('disconnectBtn').disabled = false;
        document.getElementById('connectionStatus').textContent = '연결됨';
        log('시리얼 포트 연결 성공');

        // 수신 데이터 읽기
        readSerialData();
    } catch (error) {
        log('연결 실패: ' + error.message);
    }
}

async function disconnectSerial() {
    try {
        if (reader) {
            await reader.cancel();
            reader = null;
        }
        if (writer) {
            await writer.close();
            writer = null;
        }
        if (port) {
            await port.close();
            port = null;
        }

        document.getElementById('connectBtn').disabled = false;
        document.getElementById('disconnectBtn').disabled = true;
        document.getElementById('connectionStatus').textContent = '연결 안 됨';
        log('시리얼 포트 연결 해제');
    } catch (error) {
        log('연결 해제 실패: ' + error.message);
    }
}

async function readSerialData() {
    let buffer = '';
    try {
        while (true) {
            const { value, done } = await reader.read();
            if (done) break;

            // 받은 문자를 버퍼에 추가
            buffer += value;

            // 줄바꿈 문자가 있으면 한 줄씩 처리
            let newlineIndex;
            while ((newlineIndex = buffer.indexOf('\n')) !== -1) {
                const line = buffer.substring(0, newlineIndex).trim();
                if (line.length > 0) {
                    log('수신: ' + line);
                }
                buffer = buffer.substring(newlineIndex + 1);
            }
        }
    } catch (error) {
        log('수신 오류: ' + error.message);
    }
}

async function writeToSerial(data) {
    if (!writer) {
        log('포트가 연결되지 않았습니다.');
        return;
    }
    try {
        const encoder = new TextEncoder();
        const encodedData = encoder.encode(data + '\n');
        await writer.write(encodedData);
        // 전송 후 충분한 대기 (RS-485 안정화)
        await new Promise((resolve) => setTimeout(resolve, 100));
    } catch (error) {
        log('전송 오류: ' + error.message);
    }
}

function sendCommand(cmd) {
    const deviceId = document.getElementById('deviceId').value;
    const command = `${deviceId}:${cmd}`;
    displayCommand(command);
    log(`전송: ${command}`);
    writeToSerial(command);
}

function setSpeed() {
    const deviceId = document.getElementById('deviceId').value;
    const speedOpen = document.getElementById('speedOpen').value;
    const speedClose = document.getElementById('speedClose').value;
    const command = `${deviceId}:SETSPEED:${speedOpen}:${speedClose}`;
    displayCommand(command);
    log(`전송: ${command}`);
    writeToSerial(command);
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
log('페트컵 테스트 페이지 시작');
log('명령 형식: <ID>:<CMD>:<PARAM>');

// Web Serial API 지원 확인
if (!('serial' in navigator)) {
    log('경고: 이 브라우저는 Web Serial API를 지원하지 않습니다.');
    log('Chrome/Edge 브라우저를 사용하세요.');
}
