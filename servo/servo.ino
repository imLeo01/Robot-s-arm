#include <AccelStepper.h>
#include <Servo.h>

// Khai báo chân điều khiển stepper
#define STEP_PIN_1 2  // stepX
#define DIR_PIN_1 5   // dirX
#define STEP_PIN_2 3  // stepY
#define DIR_PIN_2 6   // dirY
#define ENABLE_PIN 8  // enPin

// Servo
#define SERVO_PIN 12          // Servo pin
#define PEN_UP_ANGLE 90        // Góc nhấc bút
#define PEN_DOWN_ANGLE 0     // Góc hạ bút

// Tạo đối tượng stepper
AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);

// Servo
Servo penServo;

// Vị trí mục tiêu
long target1 = 0;
long target2 = 0;

void setup() {
  Serial.begin(115200);
  
  // Thiết lập chân ENABLE
  pinMode(ENABLE_PIN, OUTPUT);
  enableMotors();  // Kích hoạt động cơ
  
  // Thiết lập tốc độ và gia tốc
  configureSteppers();

  // Gắn servo và nâng bút ban đầu
  penServo.attach(SERVO_PIN);
  pen_up();
  
  Serial.println("READY");
}

void loop() {
  // Chạy động cơ stepper để đạt đến vị trí mục tiêu
  stepper1.run();
  stepper2.run();
  
  // Kiểm tra dữ liệu từ Serial
  handleSerialInput();
}

// Cấu hình tốc độ và gia tốc của động cơ stepper
void configureSteppers() {
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);
}

// Xử lý lệnh từ Serial
void handleSerialInput() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();  // Xóa khoảng trắng

    if (command.startsWith("GOTO")) {
      handleGotoCommand(command);
    } else if (command == "PU") {
      pen_up();
      Serial.println("PEN UP");
    } else if (command == "PD") {
      pen_down();
      Serial.println("PEN DOWN");
    } else if (command == "HOME") {
      moveToPosition(0, 0);
      Serial.println("HOME OK");
    } else if (command == "STOP") {
      stopMotors();
    } else if (command == "DISABLE") {
      disableMotors();
    } else if (command == "ENABLE") {
      enableMotors();
    } else if (command == "STATUS") {
      reportStatus();
    } else if (command == "TEST") {
      testMotors();
    } else {
      Serial.println("UNKNOWN CMD: " + command);
    }
  }
}

// Xử lý lệnh di chuyển đến vị trí GOTO
void handleGotoCommand(String command) {
  int space1 = command.indexOf(' ');
  int space2 = command.indexOf(' ', space1 + 1);
  
  if (space1 > 0 && space2 > space1) {
    long s1 = command.substring(space1 + 1, space2).toInt();
    long s2 = command.substring(space2 + 1).toInt();
    moveToPosition(s1, s2);
    Serial.println("OK");
  } else {
    Serial.println("ERR GOTO SYNTAX");
  }
}

// Di chuyển đến vị trí mục tiêu
void moveToPosition(long s1, long s2) {
  target1 = s1;
  target2 = s2;
  stepper1.moveTo(target1);
  stepper2.moveTo(target2);
}

// Nhấc bút
void pen_up() {
  penServo.write(PEN_UP_ANGLE);
  delay(300);  // Chờ servo di chuyển
}

// Hạ bút
void pen_down() {
  penServo.write(PEN_DOWN_ANGLE);
  delay(300);  // Chờ servo di chuyển
}

// Vô hiệu hóa động cơ
void disableMotors() {
  digitalWrite(ENABLE_PIN, HIGH);  // Vô hiệu hóa động cơ
  Serial.println("MOTORS DISABLED");
}

// Kích hoạt động cơ
void enableMotors() {
  digitalWrite(ENABLE_PIN, LOW);  // Kích hoạt động cơ
  Serial.println("MOTORS ENABLED");
}

// Dừng động cơ
void stopMotors() {
  stepper1.stop();
  stepper2.stop();
  Serial.println("STOPPED");
}

// Báo cáo trạng thái hiện tại
void reportStatus() {
  Serial.print("Position: X=");
  Serial.print(stepper1.currentPosition());
  Serial.print(", Y=");
  Serial.println(stepper2.currentPosition());
}

// Thử nghiệm động cơ
void testMotors() {
  Serial.println("Testing motors...");
  
  // Test động cơ 1
  stepper1.moveTo(100);
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
  }
  delay(500);
  stepper1.moveTo(0);
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
  }
  
  // Test động cơ 2
  stepper2.moveTo(100);
  while (stepper2.distanceToGo() != 0) {
    stepper2.run();
  }
  delay(500);
  stepper2.moveTo(0);
  while (stepper2.distanceToGo() != 0) {
    stepper2.run();
  }
  
  Serial.println("Motor test completed");
}