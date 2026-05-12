#include <Arduino.h>
#include <bq769x0.h>
#include "driver/ledc.h"
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MLX90614.h> 

Adafruit_ADS1115 ads;
Adafruit_MLX90614 mlx = Adafruit_MLX90614(); 

// ==========================================
// --- BQ76920 BMS CONFIGURATION ---
// ==========================================
#define BMS_ALERT_PIN 34     
#define BMS_BOOT_PIN 25      
#define BMS_I2C_ADDRESS 0x08

bq769x0 BMS(bq76920, BMS_I2C_ADDRESS);

// --- DUAL-LOOP TIMERS ---
const long safetyInterval = 300;       // Fast loop for OCC/OCD (Throttled to 2Hz for clean I2C)
unsigned long lastSafetyMillis = 0;

const long balancerInterval = 10000;   // Slow loop for Efficiency (Every 10s)
unsigned long lastBalancerMillis = 0;

bool is_bms_settling = false;
unsigned long bms_settle_timer = 0;

// ==========================================
// --- BALANCER HARDWARE & VARIABLES ---
// ==========================================
float vZeroIn = 1.660; 
float vZeroOut = 1.660;

long bq_current_offset_mA = 0;

const float L_uH = 47.0;
const float V_CELL_EST = 3.3; 
const float MAX_PULSE_TIME_US = 50.0;

const int activePinA = 26; 
const int activePinB = 27;
const int inputMatrix[]  = {14, 13, 12, 5, 4};  
const int outputMatrix[] = {19, 18, 17, 16, 15};

float pulse_time_us = 10.0; 
float discharge_ratio = 0.7; 
float dead_time_us = 0.21;

// --- HYSTERESIS & STICKY STATE VARIABLES ---
const int BAL_START_MV = 10; 
const int BAL_STOP_MV = 2;   
int currentSourceCell = -1;
int currentTargetCell = -1;
bool auto_balance_enabled = false; 
bool is_running = false;           

unsigned long lastDisplayTime = 0;
const int displayInterval = 1000; 

unsigned long lastAutoCalMillis = 0;
const long autoCalInterval = 15000; // 15 seconds

// ==========================================
// --- SETUP ---
// ==========================================
void setup() {
  Serial.begin(115200); 
  
  for (int i = 0; i < 5; i++) {
    pinMode(inputMatrix[i], OUTPUT);
    pinMode(outputMatrix[i], OUTPUT);
    digitalWrite(inputMatrix[i], LOW);
    digitalWrite(outputMatrix[i], LOW);
  }

  ledc_timer_config_t timer_conf = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_10_BIT,
      .timer_num = LEDC_TIMER_0,
      .freq_hz = 1000, 
      .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_conf);

  setupLEDCChannel(activePinA, LEDC_CHANNEL_0);
  setupLEDCChannel(activePinB, LEDC_CHANNEL_1);

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS1115!");
    while (1);
  }
  ads.setGain(GAIN_ONE); 
  ads.setDataRate(RATE_ADS1115_860SPS);

  if (!mlx.begin()) {
    Serial.println("Failed to initialize MLX90614 Sensor! Check wiring.");
  }

  pinMode(BMS_ALERT_PIN, INPUT_PULLDOWN);
  pinMode(BMS_BOOT_PIN, OUTPUT);
  digitalWrite(BMS_BOOT_PIN, HIGH);
  delay(5);
  pinMode(BMS_BOOT_PIN, INPUT);
  delay(20);

  int err = BMS.begin(BMS_ALERT_PIN, BMS_BOOT_PIN);
  if (err != 0) {
    Serial.println("!!! ERROR: BMS failed to initialize!");
    while (1); 
  }
  
  BMS.enableDischarging();
  BMS.enableCharging();
  BMS.setTemperatureLimits(-20, 45, 0, 45);
  BMS.setShuntResistorValue(5);
  
  // --- TIER 2 HARDWARE FAILSAFES ---
  BMS.setShortCircuitProtection(8800, 100);         
  BMS.setOvercurrentDischargeProtection(4000, 320); 
  
  BMS.setCellUndervoltageProtection(2700, 2); 
  BMS.setCellOvervoltageProtection(3650, 2);  
  BMS.setBalancingThresholds(0, 4000, 20);  
  BMS.setIdleCurrentThreshold(30);

  calibrateSensors();
  printMenu();
}

// ==========================================
// --- MAIN LOOP ---
// ==========================================
void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input == "1") { 
      auto_balance_enabled = true; 
      Serial.println("\n>>> AUTO-BALANCE ENABLED <<<");
    } 
    else if (input == "0") { 
      auto_balance_enabled = false; 
      is_running = false; 
      currentSourceCell = -1;
      currentTargetCell = -1;
      stopPWM(true); // Full wipe
      Serial.println("\n>>> AUTO-BALANCE DISABLED <<<");
    }
    else if (input == "m") { 
      if (currentSourceCell != -1 && currentTargetCell != -1) {
        auto_balance_enabled = false; 
        is_running = true; 
        // Explicitly re-assert the matrix before turning on PWM
        selectSource(currentSourceCell); 
        selectTarget(currentTargetCell);
        update_pwm_duty(); 
        Serial.println("\n>>> MANUAL BALANCE STARTED <<<");
      } else {
        Serial.println("\n!!! ERROR: Assign Source (s#) and Target (t#) first! !!!");
      }
    }
    else if (input == "z") { calibrateSensors(); } 
    else if (input == "c") { 
      BMS.disableCharging(); 
      Serial.println("\n>>> MAIN PACK: CHARGING FET DISABLED <<<"); 
    }
    else if (input == "d") { 
      BMS.disableDischarging(); 
      Serial.println("\n>>> MAIN PACK: DISCHARGING FET DISABLED <<<"); 
    }
    else if (input == "o") { 
      // 1. Wipe the hardware fault latches in the BQ memory FIRST
      BMS.writeRegister(0x00, 0xFF); 
      delay(10);
      
      // 2. Re-enable the MOSFETs
      BMS.enableCharging(); 
      BMS.enableDischarging(); 
      Serial.println("\n>>> MAIN PACK: FAULTS CLEARED & ALL FETS RE-ENABLED (ON) <<<"); 
    }
    else if (input.startsWith("s")) { 
      is_running = false; 
      stopPWM(false); // Stop switching, but DO NOT wipe the matrix
      int cell = input.substring(1).toInt();
      selectSource(cell); 
      auto_balance_enabled = false; 
      Serial.printf("\n>>> MANUAL: Source Cell set to %d <<<\n", cell);
    }
    else if (input.startsWith("t")) { 
      is_running = false; 
      stopPWM(false); // Stop switching, but DO NOT wipe the matrix
      int cell = input.substring(1).toInt();
      selectTarget(cell); 
      auto_balance_enabled = false; 
      Serial.printf("\n>>> MANUAL: Target Cell set to %d <<<\n", cell);
    }
    else if (input == "q") { pulse_time_us = min(pulse_time_us + 1.0f, MAX_PULSE_TIME_US); update_pwm_duty(); }
    else if (input == "w") { pulse_time_us = max(pulse_time_us - 1.0f, 1.0f); update_pwm_duty(); }
    else if (input == "e") { discharge_ratio = min(discharge_ratio + 0.05f, 1.5f); update_pwm_duty(); }
    else if (input == "r") { discharge_ratio = max(discharge_ratio - 0.05f, 0.1f); update_pwm_duty(); }
  }

  if (millis() - lastDisplayTime >= displayInterval) {
    displaySensors();
    lastDisplayTime = millis();
  }

  // ==================================================
  // TASK 1: THE SAFETY LOOP (Runs every 500ms)
  // ==================================================
  if (millis() - lastSafetyMillis >= safetyInterval) {
    lastSafetyMillis = millis();
    runSafetyChecks(); 
  }

  // ==================================================
  // TASK 2: THE BALANCER TRIGGER (Runs every 10s)
  // ==================================================
  if ((millis() - lastBalancerMillis >= balancerInterval) && !is_bms_settling) {
    if (is_running) {
      stopPWM(true); // WE WANT to wipe the matrix during the 3s pause to protect it
      is_bms_settling = true;
      bms_settle_timer = millis();
      Serial.println("\n>>> [10s TICK] Pausing Balancer for 3s to get Resting Voltages...");
    } else {
      lastBalancerMillis = millis();
      runBalancerLogic(); 
    }
  }

  // ==================================================
  // TASK 3: THE BALANCER EXECUTION (After 3s pause)
  // ==================================================
  if (is_bms_settling && (millis() - bms_settle_timer >= 3000)) {
    is_bms_settling = false;
    lastBalancerMillis = millis();
    runBalancerLogic(); 
  }

  // === IDLE AUTO-CALIBRATION ===
  if (millis() - lastAutoCalMillis >= autoCalInterval) {
    lastAutoCalMillis = millis();
    if (!is_running && currentSourceCell == -1 && currentTargetCell == -1) {
      long sumIn = 0, sumOut = 0;
      for(int i = 0; i < 10; i++) {
        sumIn += ads.readADC_SingleEnded(2); 
        sumOut += ads.readADC_SingleEnded(0); 
        delay(2); 
      }
      vZeroIn = (sumIn / 10.0) * 0.000125;
      vZeroOut = (sumOut / 10.0) * 0.000125;
      Serial.printf("\n[IDLE AUTO-CAL] TMCS Sensors re-zeroed. In: %.3fV | Out: %.3fV\n", vZeroIn, vZeroOut);
    }
  }
}

// ==========================================
// --- CORE FUNCTIONS ---
// ==========================================

void runSafetyChecks() {
  // --- EDGE-TRIGGER MEMORY FLAGS ---
  static int last_sys_stat = 0;
  static bool ot_triggered = false;
  static bool ut_triggered = false;
  static bool sw_ocd_triggered = false;
  static bool sw_occ_triggered = false;

  bool fault_active = false; // NEW: Master switch for emergency stop

  // === 1. HARDWARE FAULTS (Tier 2) ===
  int sys_stat = BMS.readRegister(0x00); 
  
  if (sys_stat & 0x0F) fault_active = true; // If any bottom 4 bits (OV, UV, SCD, OCD) are 1

  // Only print if the fault is NEW (not in the last_sys_stat memory)
  if ((sys_stat & 0x01) && !(last_sys_stat & 0x01)) Serial.println("\n!!! BQ FAULT: OVERCURRENT DISCHARGE (OCD) !!!");
  if ((sys_stat & 0x02) && !(last_sys_stat & 0x02)) Serial.println("\n!!! BQ FAULT: SHORT CIRCUIT DISCHARGE (SCD) !!!");
  if ((sys_stat & 0x04) && !(last_sys_stat & 0x04)) Serial.println("\n!!! BQ FAULT: OVERVOLTAGE (OV) !!!");
  if ((sys_stat & 0x08) && !(last_sys_stat & 0x08)) Serial.println("\n!!! BQ FAULT: UNDERVOLTAGE (UV) !!!");
  
  last_sys_stat = sys_stat; 

  // === 2. UPDATE CORE LIBRARY ===
  BMS.update(); 
  
  // === 3. SOFTWARE THERMAL PROTECTION (MLX) ===
  float targetTemp = mlx.readObjectTempC();
  
  if (targetTemp >= 50.0) {
    fault_active = true;
    if (!ot_triggered) { 
      Serial.println("\n!!! SOFTWARE FAULT: OVERTEMPERATURE (OT) !!!");
      BMS.disableCharging(); 
      BMS.disableDischarging();
      ot_triggered = true;
    }
  } else {
    ot_triggered = false; 
  }

  if (targetTemp <= 0.0) {
    fault_active = true;
    if (!ut_triggered) {
      Serial.println("\n!!! SOFTWARE FAULT: UNDERTEMPERATURE (UT) !!!");
      BMS.disableCharging(); 
      BMS.disableDischarging();
      ut_triggered = true;
    }
  } else {
    ut_triggered = false;
  }

  // === 4. SOFTWARE CURRENT PROTECTION (Tier 1) ===
  long packCurrent_mA = BMS.getBatteryCurrent() - bq_current_offset_mA; 
  
  if (packCurrent_mA < -1800) {
    fault_active = true;
    if (!sw_ocd_triggered) {
      Serial.println("\n!!! SOFTWARE FAULT: OVERCURRENT DISCHARGE (OCD) 1.8A !!!");
      BMS.disableDischarging();
      sw_ocd_triggered = true;
    }
  } else {
    sw_ocd_triggered = false;
  }

  if (packCurrent_mA > 1800) {
    fault_active = true;
    if (!sw_occ_triggered) {
      Serial.println("\n!!! SOFTWARE FAULT: OVERCURRENT CHARGE (OCC) 1.8A !!!");
      BMS.disableCharging();
      sw_occ_triggered = true;
    }
  } else {
    sw_occ_triggered = false;
  }

  // === 5. EMERGENCY BALANCER SHUTDOWN ===
  // If any fault is active, and the balancer is currently trying to run, kill it instantly.
  if (fault_active && (is_running || auto_balance_enabled)) {
    Serial.println("\n!!! EMERGENCY STOP: HALTING ACTIVE BALANCER !!!");
    is_running = false;
    auto_balance_enabled = false;
    currentSourceCell = -1;
    currentTargetCell = -1;
    stopPWM(true); // Full wipe of the matrix and high-frequency switching
  }
}

void runBalancerLogic() {
  int maxV = -1, minV = 10000;
  int maxIndex = -1, minIndex = -1;
  long sumV = 0;

  Serial.println("\n--- BQ76920 RESTING PACK DATA ---");
  for (int i = 1; i <= 5; i++) {
    int v = BMS.getCellVoltage(i);
    sumV += v;
    Serial.printf("  Cell %d: %d mV\n", i, v);
    if (v > maxV) { maxV = v; maxIndex = i; }
    if (v < minV) { minV = v; minIndex = i; }
  }
  
  int packAvg = sumV / 5;
  int totalSpread = maxV - minV;
  long packCurrent_mA = BMS.getBatteryCurrent() - bq_current_offset_mA; 
  float ambientTemp = mlx.readAmbientTempC();
  float targetTemp = mlx.readObjectTempC();

  // === CLEAN CSV PRINT ===
  Serial.printf("CSV,%lu,%d,%d,%d,%d,%d,%ld,%.2f,%.2f\n", 
                millis(), 
                BMS.getCellVoltage(1), BMS.getCellVoltage(2), BMS.getCellVoltage(3), 
                BMS.getCellVoltage(4), BMS.getCellVoltage(5),
                packCurrent_mA, ambientTemp, targetTemp);

  Serial.printf("  Pack Spread: %d mV | Pack Average: %d mV\n", totalSpread, packAvg);

  // === 1. UNIVERSAL TARGET TRACKING ===
  if (is_running && currentSourceCell != -1 && currentTargetCell != -1) {
    int sourceV = BMS.getCellVoltage(currentSourceCell);
    int targetV = BMS.getCellVoltage(currentTargetCell);
    int lockedPairSpread = sourceV - targetV;
      
    if (lockedPairSpread <= BAL_STOP_MV || sourceV <= (packAvg + 1) || targetV >= (packAvg - 1)) {
      Serial.println(">>> Target achieved or Average crossed. Releasing lock.");
      is_running = false;
      currentSourceCell = -1;
      currentTargetCell = -1;
    } 
  } 
  
  // === 2. AUTO-BALANCE PAIR HUNTING ===
  if (auto_balance_enabled && !is_running) {
    if (minV < 2750) {
      Serial.printf(">>> EMERGENCY: C%d is low. Forcing charge from C%d.\n", minIndex, maxIndex);
      selectSource(maxIndex); selectTarget(minIndex); delay(20); is_running = true;
    } else if (totalSpread >= BAL_START_MV) {
      Serial.printf(">>> Auto: Locking C%d (Source) and C%d (Target).\n", maxIndex, minIndex);
      selectSource(maxIndex); selectTarget(minIndex); delay(20); is_running = true;
    }
  }

  // === 3. HARDWARE EXECUTION ===
  if (is_running) {
    selectSource(currentSourceCell); 
    selectTarget(currentTargetCell);
    update_pwm_duty(); 
  } else {
    // Only wipe the physical matrix if we are in full standby (no manual cells picked)
    if (currentSourceCell == -1 && currentTargetCell == -1) {
      stopPWM(true);
    } else {
      stopPWM(false); // Just stop PWM, keep the user's manual selection alive!
    }
  }
}

void setupLEDCChannel(int pin, ledc_channel_t channel) {
  ledc_channel_config_t ch_conf = {
      .gpio_num = pin,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = channel,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_0,
      .duty = 0,
      .hpoint = 0
  };
  ledc_channel_config(&ch_conf);
}

void selectSource(int cell) {
  for (int i = 0; i < 5; i++) digitalWrite(inputMatrix[i], LOW);
  delay(10); 
  if (cell >= 1 && cell <= 5) {
    digitalWrite(inputMatrix[cell - 1], HIGH);
    currentSourceCell = cell;
  } else {
    currentSourceCell = -1;
  }
}

void selectTarget(int cell) {
  for (int i = 0; i < 5; i++) digitalWrite(outputMatrix[i], LOW);
  delay(10); 
  if (cell >= 1 && cell <= 5) {
    digitalWrite(outputMatrix[cell - 1], HIGH);
    currentTargetCell = cell;
  } else {
    currentTargetCell = -1;
  }
}

void update_pwm_duty() {
  float discharge_time_us = pulse_time_us * discharge_ratio;
  float PWM_PERIOD_US = pulse_time_us + discharge_time_us + (2 * dead_time_us); 
  
  uint32_t freq_hz = (uint32_t)(1000000.0 / PWM_PERIOD_US);
  float ticks_per_us = 1024.0 / PWM_PERIOD_US;
  
  uint32_t duty_ticks_A = (uint32_t)(pulse_time_us * ticks_per_us);
  uint32_t duty_ticks_B = (uint32_t)(discharge_time_us * ticks_per_us);
  uint32_t dead_time_ticks = (uint32_t)(dead_time_us * ticks_per_us);

  if (is_running && currentSourceCell != -1 && currentTargetCell != -1) {
    ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, freq_hz);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_ticks_A);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty_with_hpoint(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty_ticks_B, duty_ticks_A + dead_time_ticks);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
  } else {
    stopPWM(false);
  }

  float i_peak = (V_CELL_EST * pulse_time_us) / L_uH;
  float i_in_avg_exp = (0.5 * i_peak * pulse_time_us) / PWM_PERIOD_US;
  float i_out_avg_exp = (0.5 * i_peak * discharge_time_us) / PWM_PERIOD_US;

  Serial.println("\n--- PWM TIMING UPDATED ---");
  if (!is_running || currentSourceCell == -1 || currentTargetCell == -1) {
    Serial.println("  >>> STANDBY: Parameters Saved. Waiting for Start Command. <<<");
  } else {
    Serial.println("  >>> ACTIVE: Timings applied to Hardware. <<<");
  }
  
  Serial.printf("FREQ: %d Hz | PULSE: %.1fus | RATIO: %.2f\n", freq_hz, pulse_time_us, discharge_ratio);
  Serial.printf("EXPECTED IN: %.0fmA | EXPECTED OUT: %.0fmA\n", i_in_avg_exp * 1000, i_out_avg_exp * 1000);
}

void displaySensors() {
  long sumRawIn = 0;
  long sumRawOut = 0;
  const int numSamples = 20; 

  for (int i = 0; i < numSamples; i++) {
    sumRawIn += ads.readADC_SingleEnded(2);
    sumRawOut += ads.readADC_SingleEnded(0);
  }

  int16_t rawIn = sumRawIn / numSamples;
  int16_t rawOut = sumRawOut / numSamples;

  float vIn = rawIn * 0.000125;
  float vOut = rawOut * 0.000125;
  
  float iInAmps = (vIn - vZeroIn) / 0.400;
  float iOutAmps = (vOut - vZeroOut) / 0.400;
  
  float eff = 0.0;
  float pInWatts = 0.0;
  float pOutWatts = 0.0;

  if (abs(iInAmps) > 0.02 && currentSourceCell != -1 && currentTargetCell != -1) {
    float vSource = BMS.getCellVoltage(currentSourceCell) / 1000.0;
    float vTarget = BMS.getCellVoltage(currentTargetCell) / 1000.0;
    
    pInWatts = abs(iInAmps) * vSource;
    pOutWatts = abs(iOutAmps) * vTarget;
    
    if (pInWatts > 0) {
      eff = (pOutWatts / pInWatts) * 100.0;
    }
  } else if (abs(iInAmps) > 0.02) {
    eff = (abs(iOutAmps) / abs(iInAmps)) * 100.0;
  }

  Serial.printf("[RAW ADS] AIN2(IN): %5d (%.3fV) | AIN0(OUT): %5d (%.3fV) || ZERO In: %.3fV | Out: %.3fV\n", 
                rawIn, vIn, rawOut, vOut, vZeroIn, vZeroOut);
  
  Serial.printf("[REALTIME] Iin: %4.0fmA | Iout: %4.0fmA | Pin: %.2fW | Pout: %.2fW | Eff: %.1f%%\n", 
                iInAmps * 1000, iOutAmps * 1000, pInWatts, pOutWatts, eff);
}

void calibrateSensors() {
  Serial.println("\n>>> Calibrating Sensors (Forcing PWM OFF)...");
  stopPWM(true); 
  delay(100); 

  long sumIn = 0, sumOut = 0;
  for(int i=0; i<10; i++) {
    sumIn += ads.readADC_SingleEnded(2); 
    sumOut += ads.readADC_SingleEnded(0); 
    delay(5);
  }
  vZeroIn = (sumIn / 10.0) * 0.000125;
  vZeroOut = (sumOut / 10.0) * 0.000125;
  Serial.printf(">>> ADS Calibrated! Zero In: %.3fV | Zero Out: %.3fV\n", vZeroIn, vZeroOut);

  long bq_sum = 0;
  for(int i=0; i<5; i++) {
    BMS.update();
    bq_sum += BMS.getBatteryCurrent();
    delay(50); 
  }
  bq_current_offset_mA = bq_sum / 5;
  Serial.printf(">>> BQ76920 Calibrated! Idle Current Offset: %ld mA\n", bq_current_offset_mA);
}

// Updated with a bool argument to selectively wipe the matrix
void stopPWM(bool wipeMatrix) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
  
  if (wipeMatrix) {
    for (int i = 0; i < 5; i++) {
      digitalWrite(inputMatrix[i], LOW);
      digitalWrite(outputMatrix[i], LOW);
    }
  }
}

void printMenu() {
  Serial.println("\n============= COMMANDS =============");
  Serial.println(" 1 / 0 : Start / Stop AUTO-BALANCE");
  Serial.println(" c     : Recalibrate Current Sensors");
  Serial.println(" q / w : Pulse Time +1us / -1us");
  Serial.println(" e / r : Disch. Ratio +0.05 / -0.05");
  Serial.println(" s[#]  : Manual Source (e.g. s5)");
  Serial.println(" t[#]  : Manual Target (e.g. t1)");
  Serial.println("====================================");
}