// 핀 설정
const int RPWM = 9;
const int LPWM = 10;

// 핀 설정
const int R_IS = A6;
const int L_IS = A4;

void setup() {
  Serial.begin(9600);
  
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_IS, INPUT);
  pinMode(L_IS, INPUT);
}

void loop() {
  // 1. 정방향 주행 및 전류 체크
  Serial.println("정방향 회전 시작");
  motorDrive(150, 0); // 속도 150 (0~255)으로 정방향
  delay(2000);
  checkCurrent();     // 전류 값 출력
  
  // 2. 정지
  motorDrive(0, 0);
  delay(1000);

  // 3. 역방향 주행 및 전류 체크
  Serial.println("역방향 회전 시작");
  motorDrive(0, 150);
  delay(2000);
  checkCurrent();     
  
  // 4. 정지
  motorDrive(0, 0);
  delay(1000);
}

// 모터 구동 함수
void motorDrive(int r_speed, int l_speed) {
  analogWrite(RPWM, r_speed);
  analogWrite(LPWM, l_speed);
}

// 전류 측정 및 시리얼 모니터 출력 함수
void checkCurrent() {
  int r_sense = analogRead(R_IS);
  int l_sense = analogRead(L_IS);

  // 전압 변환 (5V 기준)
  float r_voltage = r_sense * (5.0 / 1023.0);
  float l_voltage = l_sense * (5.0 / 1023.0);
  
  Serial.print("R_IS: ");
  Serial.print(r_sense);
  Serial.print(" \t볼트 : ");
  Serial.println(r_voltage);

  Serial.print("L_IS: ");
  Serial.print(l_sense);
  Serial.print(" \t볼트 : ");
  Serial.println(l_voltage);
  Serial.println("---------------------------------");
}
