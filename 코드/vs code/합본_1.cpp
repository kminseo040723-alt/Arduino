#include <Adafruit_VL53L0X.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// --- 시스템 공통 상수 ---
#define SERVO_MIN_PULSE_WIDTH 650
#define SERVO_MAX_PULSE_WIDTH 2350
#define PCA_FREQUENCY 50

enum SystemMode { 
    MODE_IDLE, 
    MODE_RUNNING, 
    MODE_DONE, 
    MODE_FAIL };

enum Signal { 
    KEEP, 
    NEXT, 
    RETRY, 
    FAIL, 
    RECLIMB};

// 스텝 모터 기반 그리퍼 제어 클래스
class Gripper {
private:
    int STR;
    int DIR;
    const int Step_Delay_Open = 100; // 마이크로초 단위
    const int Step_Delay_Close = 100; // 마이크로초 단위
public:
    Gripper(int STR, int DIR)  : STR(STR), DIR(DIR) {}

    void Init () {
        pinMode(DIR, OUTPUT);
        pinMode(STR, OUTPUT);
    }

    
    void Set_Direction(bool Gripper_Direction) {
        digitalWrite(DIR, Gripper_Direction ? HIGH : LOW);
    }
    
    void Pulse() {
        digitalWrite(STR, HIGH);
        delayMicroseconds(Step_Delay_Open); 
        digitalWrite(STR, LOW);
    }
    
    void stop() {
        digitalWrite(STR, LOW);
    }
};

// BTS7960 모터 드라이버 제어 클래스 (수평 및 등반 모터)
class Motor_Driver {
private:
    int R_PWM, L_PWM;
    int currentSpeed;

    const int Total_Motor_Num = 3;
public:
    Motor_Driver(int R, int L) : R_PWM(R), L_PWM(L), currentSpeed(0) {}
    
    void init() {
        pinMode(R_PWM, OUTPUT);
        pinMode(L_PWM, OUTPUT);
        stop();
    }
    
    void setSpeed(int speed, int minSpeed = 0, int maxSpeed = 255) {
        currentSpeed = constrain(speed, minSpeed, maxSpeed);
        analogWrite(R_PWM, currentSpeed);
        analogWrite(L_PWM, 0);
    }
    
    int getSpeed() const { return currentSpeed; }
    
    void stop() {
        currentSpeed = 0;
        analogWrite(R_PWM, 0);
        analogWrite(L_PWM, 0);
    }
};

// VL53L0X 거리 센서 관리 클래스 (필터링 및 이동평균 내장)
class DistanceSensor {
private:
    Adafruit_VL53L0X lox;
    unsigned long scanTime;
    unsigned long avgStartTime;
    unsigned long distanceSum;
    int distanceCount;
    const unsigned long scanInterval = 20;
    const unsigned long avgDuration = 500;

public:
    enum ScanState { DISTANCE_WAITING, DISTANCE_READY, DISTANCE_FAILED };

    bool init() {
        return lox.begin();
    }

    bool readRaw(int& outDistance) {
        VL53L0X_RangingMeasurementData_t measure;
        lox.rangingTest(&measure, false);
        if (measure.RangeStatus != 4) {
            outDistance = measure.RangeMilliMeter;
            return true;
        }
        return false;
    }

    void resetAverage() {
        scanTime = millis() - scanInterval;
        avgStartTime = millis();
        distanceSum = 0;
        distanceCount = 0;
    }

    ScanState getAveragedDistance(int& outAverage) {
        if (millis() - scanTime < scanInterval) return DISTANCE_WAITING;
        scanTime = millis();

        int distance = 0;
        if (readRaw(distance)) {
            distanceSum += distance;
            distanceCount++;
        }

        if (millis() - avgStartTime < avgDuration) return DISTANCE_WAITING;

        if (distanceCount == 0) {
            resetAverage();
            return DISTANCE_FAILED;
        }

        outAverage = distanceSum / distanceCount;
        resetAverage();
        return DISTANCE_READY;
    }
};

// PCA9685 서보 모터 관리 클래스
class ServoCluster {
private:
    Adafruit_PWMServoDriver pwm;
    const int channels[4] = {4, 5, 6, 7};
public:
    void init() {
        pwm.begin();
        pwm.setPWMFreq(PCA_FREQUENCY);
    }
    
    void writeAngle(int channel, int angle) {
        int pulseLength = map(angle, 0, 180, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
        int pwmValue = pulseLength * PCA_FREQUENCY / 1000000.0 * 4096;
        pwm.setPWM(channel, 0, pwmValue);
    }
    
    void writeAll(int angle) {
        for (int i = 0; i < 4; i++) {
            writeAngle(channels[i], angle);
        }
    }
    
    void releaseAll() {
        for (int i = 0; i < 4; i++) {
            pwm.setPWM(channels[i], 0, 0);
        }
    }
};

class RobotState {
public:
    virtual void onEnter() = 0;
    virtual Signal onUpdate() = 0;
    virtual void onExit() = 0;
    virtual ~RobotState() {}
};

// 상태 1 & 3: 그리퍼 구동 상태 (오픈/클로즈 공용 클래스화)
class GripperState : public RobotState {
private:
    Gripper& gripper;
    bool openMode; // true: 오픈, false: 클로즈
    int currentStep;
    const int maxSteps = 14400;
    const int stepDelay = 1000;

public:
    GripperState(Gripper& g, bool open) : gripper(g), openMode(open) {}

    void onEnter() override {
        currentStep = 0;
        gripper.setDirection(openMode ? HIGH : LOW);
    }

    Signal onUpdate() override {
        // [비동기 if 구조] loop가 돌 때마다 딱 1스텝씩 튕겨줌! 아두이노가 안 멈춤!
        if (currentStep < maxSteps) {
            gripper.pulse();
            delayMicroseconds(stepDelay * 2); 
            currentStep++;
            return KEEP;
        }
        return NEXT;
    }

    void onExit() override {
        gripper.stop();
    }
};

// 상태 2: 수평 이동 상태
class HorizontalMoveState : public RobotState {
private:
    Motor_Driver& horizMotor;
    unsigned long startTime;
    unsigned long lastSpeedChangeTime;
    int speed;

    const unsigned long duration = 3000;
    const unsigned long decelDuration = 4000;
    const unsigned long accelInterval = 200;
    const unsigned long decelInterval = 200;
    const int startSpeed = 50;
    const int speedIncrement = 5;
    const int targetSpeed = 180;
    const int safeStopSpeed = 50; // 안 자빠지는 안전 감속 속도 기준

public:
    HorizontalMoveState(Motor_Driver& hm) : horizMotor(hm) {}

    void onEnter() override {
        startTime = millis();
        lastSpeedChangeTime = millis();
        speed = 0;
    }

    Signal onUpdate() override {
        unsigned long elapsedTime = millis() - startTime;

        // 1. 가속 및 주행 구간
        if (elapsedTime <= duration) {
            if (millis() - lastSpeedChangeTime >= accelInterval) {
                lastSpeedChangeTime = millis();
                speed = (speed == 0) ? startSpeed : min(speed + speedIncrement, targetSpeed);
            }
            horizMotor.setSpeed(speed);
            return KEEP;
        }

        // 2. 감속 및 제어권 이양 구간
        if (elapsedTime <= duration + decelDuration) {
            if (millis() - lastSpeedChangeTime >= decelInterval) {
                lastSpeedChangeTime = millis();
                speed = max(0, speed - speedIncrement);
            }
            horizMotor.setSpeed(speed);

            // 사용자가 의도한 "안전 속도(50)" 이하로 떨어지면 미련 없이 즉시 NEXT!
            if (speed <= safeStopSpeed) {
                return NEXT;
            }
            return KEEP;
        }
        return NEXT;
    }

    void onExit() override {
        horizMotor.stop();
    }
};

class ClimbState : public RobotState {
private:
    Motor_Driver& motorL;
    Motor_Driver& motorR;
    Gripper& gripper;
    DistanceSensor& sensor;

    enum Trend { TREND_NORMAL, TREND_DECREASING, TREND_HOLDING };
    enum Phase { PHASE_NORMAL, PHASE_TIGHTEN, PHASE_RE_ACCEL };
    
    Trend climbTrend;
    Phase currentPhase;

    int speed;
    int climbDistance;
    int lastDistance;
    int stallBaseDistance;
    bool initialDistanceRead;
    
    unsigned long decelTime;
    unsigned long stallStartTime;
    unsigned long phaseStartTime;
    unsigned long lastGripStepTime;
    unsigned long accelTime;

    const int targetDistance = 800;
    const int intermediaryDistance = 500;
    const int tolerance = 5;

public:
    ClimbState(Motor_Driver& l, Motor_Driver& r, Gripper& g, DistanceSensor& s) 
        : motorL(l), motorR(r), gripper(g), sensor(s) {}

    void onEnter() override {
        speed = 50;
        initialDistanceRead = false;
        climbTrend = TREND_NORMAL;
        currentPhase = PHASE_NORMAL;
        decelTime = millis();
        sensor.resetAverage();
        setMotors(speed);
    }

    Signal onUpdate() override {
        int tempDistance = 0;
        DistanceSensor::ScanState res = sensor.getAveragedDistance(tempDistance);

        if (res == DISTANCE_WAITING) return KEEP;
        if (res == DISTANCE_FAILED) return FAIL; // 완벽한 구조를 위해 에러 시 FAIL 신호

        climbDistance = tempDistance;

        if (climbDistance >= targetDistance) return NEXT;

        // [핵심] 미끄러짐/정체 시퀀스 핸들러 실행
        if (handleSequenceChain(climbDistance)) {
            return KEEP; // 조임이나 재가속 단계 진행 중이면 하단 일반 주행 로직 패스!
        }

        // 일반 주행 상태 로직
        if (climbDistance < intermediaryDistance) {
            speed = 180; // 정속 등반
            setMotors(speed);
        } else if (climbDistance < targetDistance) {
            if (millis() - decelTime >= 200) { // 감속 인터벌
                decelTime = millis();
                speed = max(0, speed - 5);
                setMotors(speed);
            }
        }
        return KEEP;
    }

    void onExit() override {
        motorL.stop();
        motorR.stop();
    }

private:
    void setMotors(int targetSpeed) {
        motorL.setSpeed(targetSpeed);
        motorR.setSpeed(targetSpeed);
    }

    // 등반 중 꼬임 방지 핵심 체인 (4스텝 조임 -> 2초 가속 제어)
    bool handleSequenceChain(int dist) {
        if (!initialDistanceRead) {
            initialDistanceRead = true;
            lastDistance = dist;
            stallBaseDistance = dist;
            stallStartTime = millis();
            return false;
        }

        // 1단계: 1초 동안 그리퍼 4스텝 조이기 단계 진행 중일 때
        if (currentPhase == PHASE_TIGHTEN) {
            static int tightenCount = 0;
            if (tightenCount < 4) {
                if (millis() - lastGripStepTime >= 250) { // 1초에 4번 쪼개서 펄스 생성
                    lastGripStepTime = millis();
                    gripper.setDirection(LOW); // 조이는 방향
                    gripper.pulse();
                    tightenCount++;
                }
            }
            if (millis() - phaseStartTime >= 1000) { // 1초 경과 시 재가속으로 토스
                currentPhase = PHASE_RE_ACCEL;
                phaseStartTime = millis();
                accelTime = millis();
                tightenCount = 0;
            }
            return true;
        }

        // 2단계: 2초 동안 가속 모드로 주행 중일 때
        if (currentPhase == PHASE_RE_ACCEL) {
            if (millis() - phaseStartTime < 2000) {
                if (millis() - accelTime >= 200) {
                    accelTime = millis();
                    speed = min(speed + 5, 255);
                    setMotors(speed);
                }
            } else { // 2초 가속 종료 시 원래 일반 트랙 복귀
                currentPhase = PHASE_NORMAL;
                stallStartTime = millis();
                stallBaseDistance = dist;
                climbTrend = TREND_NORMAL;
            }
            return true;
        }

        // 미끄러짐 및 정체 모니터링 트리거 파트
        bool isDecreasing = dist <= lastDistance - tolerance;
        bool isHolding = abs(dist - stallBaseDistance) <= tolerance;

        if (isDecreasing) {
            if (climbTrend != TREND_DECREASING) {
                climbTrend = TREND_DECREASING;
                stallStartTime = millis();
            }
            if (millis() - stallStartTime >= 2000) { // 2초 정체 시 트리거 발동!
                triggerSequence();
            }
            lastDistance = dist;
            return true;
        }

        if (isHolding) {
            if (climbTrend != TREND_HOLDING) {
                climbTrend = TREND_HOLDING;
                stallStartTime = millis();
                stallBaseDistance = dist;
            }
            if (millis() - stallStartTime >= 3000) { // 3초 동일 정체 시 트리거 발동!
                triggerSequence();
            }
            lastDistance = dist;
            return true;
        }

        climbTrend = TREND_NORMAL;
        lastDistance = dist;
        stallBaseDistance = dist;
        stallStartTime = millis();
        return false;
    }

    void triggerSequence() {
        motorL.stop(); // 즉시 하중 정지
        motorR.stop();
        currentPhase = PHASE_TIGHTEN;
        phaseStartTime = millis();
        lastGripStepTime = millis();
    }
};

// 상태 5: 수확 상태
class HarvestState : public RobotState {
private:
    MotorDriver& motorL;
    MotorDriver& motorR;
    ServoCluster& servos;
    DistanceSensor& sensor;

    unsigned long pauseStartTime;
    unsigned long servoTime;
    int angle;
    int direction;
    int rotations;
    int climbDistance;

    const int maxAngle = 170;
    const int startAngle = 0;
    const int targetDistance = 800;
    const int distanceDrop = 30;

public:
    HarvestState(MotorDriver& l, MotorDriver& r, ServoCluster& sv, DistanceSensor& sn) 
        : motorL(l), motorR(r), servos(sv), sensor(sn) {}

    void onEnter() override {
        rotations = 0;
        direction = 1;
        angle = startAngle;
        pauseStartTime = millis();
        servoTime = millis();
        sensor.resetAverage();
        servos.writeAll(angle);
        motorL.setSpeed(40); // 느린 속도로 지속 등반
        motorR.setSpeed(40);
    }

    Signal onUpdate() override {
        if (rotations >= 3) return NEXT;

        // 수확 중에도 미끄러지면 강제 RECLIMB 신호 리턴
        int tempDist = 0;
        if (sensor.getAveragedDistance(tempDist) == DistanceSensor::DISTANCE_READY) {
            climbDistance = tempDist;
            if (climbDistance <= targetDistance - distanceDrop) {
                return RECLIMB;
            }
        }

        if (millis() - pauseStartTime < 1000) return KEEP; // 1초 대기구간

        // 서보 스윙 제어
        if (millis() - servoTime >= 1000) {
            servoTime = millis();
            if (direction == 1) {
                angle += 10;
                if (angle >= maxAngle) { angle = maxAngle; direction = -1; }
            } else {
                angle -= 10;
                if (angle <= startAngle) { angle = startAngle; direction = 1; rotations++; }
            }
            servos.writeAll(angle);
        }
        return KEEP;
    }

    void onExit() override {
        servos.releaseAll();
        motorL.stop();
        motorR.stop();
    }
};

// =========================================================================
// [3] 상태 총괄 제어기 클래스 (FSM 중앙 집중 관리 매니저)
// =========================================================================

class RobotController {
private:
    Gripper gripper;
    MotorDriver climbMotorL;
    MotorDriver climbMotorR;
    MotorDriver horizMotor;
    DistanceSensor sensor;
    ServoCluster servos;

    RobotState* states[5];
    int currentStateIndex;
    bool isNewState;
    int failCount;
    unsigned long idleStartTime;
    
    const int climbStateIndex = 3;

public:
    RobotController() : 
        gripper(4, 7), 
        climbMotorL(3, 5), climbMotorR(6, 9), horizMotor(10, 11),
        currentStateIndex(0), isNewState(true), failCount(0) {}

    void begin() {
        Serial.begin(115200);
        Wire.begin();
        
        gripper.init();
        climbMotorL.init();
        climbMotorR.init();
        horizMotor.init();
        servos.init();

        if (!sensor.init()) {
            Serial.println("VL53L0X Error");
            while (1);
        }

        // 런타임에 다형성 매핑 객체 생성
        states[0] = new GripperState(gripper, true);         // Gripper_Open
        states[1] = new HorizontalMoveState(horizMotor);    // Horizontal_Move
        states[2] = new GripperState(gripper, false);        // Gripper_Close
        states[3] = new ClimbState(climbMotorL, climbMotorR, gripper, sensor); // Climb
        states[4] = new HarvestState(climbMotorL, climbMotorR, servos, sensor); // Harvest
    }

    void handleIdle() {
        if (isNewState) {
            stopAll();
            idleStartTime = millis();
            isNewState = false;
        }
        if (millis() - idleStartTime >= 2000) { // 2초 대기 후 가동
            currentStateIndex = 0;
            isNewState = true;
            CurrentMode = MODE_RUNNING;
        }
    }

    void handleRunning() {
        if (isNewState) {
            states[currentStateIndex]->onEnter();
            isNewState = false;
            return;
        }

        Signal sig = states[currentStateIndex]->onUpdate();

        // 🌟 모든 상태 탈출 조건 통합: KEEP이 아니면 무조건 해당 상태의 onExit 실행!
        if (sig != KEEP) {
            states[currentStateIndex]->onExit();

            if (sig == NEXT) {
                currentStateIndex++;
                if (currentStateIndex >= 5) {
                    failCount = 0;
                    isNewState = true;
                    CurrentMode = MODE_DONE;
                } else {
                    isNewState = true;
                }
            } 
            else if (sig == RECLIMB) {
                currentStateIndex = climbStateIndex;
                isNewState = true;
            } 
            else if (sig == RETRY) {
                isNewState = true;
            } 
            else if (sig == FAIL) {
                failCount++;
                isNewState = true;
                CurrentMode = (failCount < 2) ? MODE_IDLE : MODE_FAIL;
            }
        }
    }

    void handleDone() {
        if (isNewState) { stopAll(); isNewState = false; }
    }

    void handleFail() {
        if (isNewState) {
            stopAll();
            Serial.println("System failed permanently");
            isNewState = false;
        }
    }

    void stopAll() {
        gripper.stop();
        climbMotorL.stop();
        climbMotorR.stop();
        horizMotor.stop();
        servos.releaseAll();
    }
};
RobotController robot;

void setup() {
    robot.begin();
}

void loop() {
    switch (CurrentMode) {
        case MODE_IDLE:    robot.handleIdle();    break;
        case MODE_RUNNING: robot.handleRunning(); break;
        case MODE_DONE:    robot.handleDone();    break;
        case MODE_FAIL:    robot.handleFail();    break;
    }
}