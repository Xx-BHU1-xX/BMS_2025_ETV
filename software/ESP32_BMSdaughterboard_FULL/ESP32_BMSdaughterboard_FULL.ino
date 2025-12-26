#define NUM_SEG 6
#define NUM_MOD_SEG 12
#define I_SOC_TRESHOLD 5
#define I_DCIR_TRESHOLD 5
#define MOD_CAPACITY 25
#define DCIR_MIN_LIMIT 2500
#define DCIR_MAX_LIMIT 5000
#define DCIR_JUMP_LIMIT 1000
#define BETA 0.1
#define SOC_MIN_LIMIT 0
#define SOC_MAX_LIMIT 100
#define SOC_MAX_FAULT 100
#define SOC_MIN_FAULT 0
#define ADS1115_CONFIG_REWRITE_CYCLES 100
#define RF24_CE_PIN 5
#define RF24_CSN_PIN 4
#define RF24_IRQ_PIN 13
#define BALANCE_TRESHOLD 7
#define BALANCE_RES_VALUE 33
#define HBO300_ADS1115_CURRENT_MULTIPLY_FACTOR 0.01
#define ADS1115_RDY_PIN 14
#define SDC_CTRL_PIN 27
#define PWR_LED_PIN 32
#define CURRENT_DEV_FAULT 5
#define CELLV_CHANGE_CURRENT 5
#define VAT4300_RESET_CONFIG_RETRIES 20
#define CAN_TX 26
#define CAN_RX 25
#define OTA_RETRY_DELAY_MS 3000

#include <Wire.h>
TwoWire I2CADS1115 = TwoWire(0);

#include <SPI.h>
#include "RF24.h"
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "OTA.h"
#include <ESP32-TWAI-CAN.hpp>

float VAT4300_bias = 0;
float ADS1115_bias = 0;

void TaskLinduinoCVComms(void *pvParameters);
void TaskUpdateSOC(void *pvParameters);
void TaskUpdateDCIR(void *pvParameters);
void TaskADS1115(void *pvParameters);
void TaskVAT4300(void *pvParameters);
void TaskBalancing(void *pvParameters);
void TaskUpdateTSCurrent(void *pvParameters);
void TaskSDCFault(void *pvParameters);
void TaskTelnetStreamPrintDebug(void *pvParameters);
void TaskOTAHandleAlive(void *pvParameters);
void TaskCANComms(void *pvParameters);

TaskHandle_t Linduino_CV_Comms_Task_Handle;
TaskHandle_t Update_SOC_Task_Handle;
TaskHandle_t Update_DCIR_Task_Handle;
TaskHandle_t ADS1115_Task_Handle;
TaskHandle_t VAT4300_Task_Handle;
TaskHandle_t Balancing_Task_Handle;
TaskHandle_t Update_TS_Current_Task_Handle;
TaskHandle_t SDC_Fault_Task_Handle;
TaskHandle_t TelnetStream_Print_Debug_Task_Handle;
TaskHandle_t OTA_Handle_Alive_Task_Handle;
TaskHandle_t CAN_Comms_Task_Handle;

SemaphoreHandle_t linduino_serial_mutex = NULL;
SemaphoreHandle_t i2c_mutex = NULL;
SemaphoreHandle_t rf24_mutex = NULL;
SemaphoreHandle_t telnet_mutex = NULL;
SemaphoreHandle_t twai_mutex = NULL;

QueueHandle_t cellV[NUM_SEG][NUM_MOD_SEG];  //in 100uV steps
QueueHandle_t SOC1[NUM_SEG][NUM_MOD_SEG];   //in uint percent like 95
QueueHandle_t SOC2[NUM_SEG][NUM_MOD_SEG];   //in uint percent like 95
QueueHandle_t DCIR[NUM_SEG][NUM_MOD_SEG];   //in micro ohms
QueueHandle_t SOC[NUM_SEG][NUM_MOD_SEG];    //in uint percent like 95
QueueHandle_t BAL[NUM_SEG];
QueueHandle_t TSCurrent;
QueueHandle_t SOCCoulombCountingTime;
QueueHandle_t ADS1115Current;
QueueHandle_t VAT4300Current;
QueueHandle_t TSVoltage;
QueueHandle_t TSPower;
QueueHandle_t VAT4300OnTimeSeconds;

float soc_to_ocv_above_95[4] = { -18472846.4, 574008.04, -5933.23, 20.446 };
float soc_to_ocv_below_20[4] = { 20660.0, 1931.77, -94.9, 1.635 };
float soc_to_ocv_middle[4] = { 34288.97, -8.631, 1.49, -0.00723 };
float ocv_to_soc_above_4_05[4] = { -313728.7, 226647.04, -54564.92, 4379.07 };
float ocv_to_soc_below_3_45[4] = { -5184.07, 5038.43, -1632.68, 176.67 };
float ocv_to_soc_middle[4] = { -9897.09, 7584.83, -1950.01, 169.44 };


void IRAM_ATTR ads1115ISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(ADS1115_Task_Handle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IRAM_ATTR rf24ISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(VAT4300_Task_Handle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void setup() {
  pinMode(SDC_CTRL_PIN, OUTPUT);
  pinMode(PWR_LED_PIN, OUTPUT);
  if ((esp_reset_reason() == ESP_RST_TASK_WDT) || (esp_reset_reason() == ESP_RST_WDT)) {
    while (1) {
      digitalWrite(SDC_CTRL_PIN, LOW);
      digitalWrite(PWR_LED_PIN, HIGH);
    }
  }

  Serial2.begin(19200, SERIAL_8N1, 16, 17);
  Serial.begin(115200);
  twai_general_config_t noack_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)(CAN_TX), (gpio_num_t)(CAN_RX), TWAI_MODE_NO_ACK);
  ESP32Can.begin(ESP32Can.convertSpeed(500), CAN_TX, CAN_RX, 10, 10, nullptr, &noack_config);

  uint8_t j = 0;
  uint8_t i = 0;
  for (i = 0; i < NUM_SEG; i++) {
    for (j = 0; j < NUM_MOD_SEG; j++) {
      cellV[i][j] = xQueueCreate(1, sizeof(uint16_t));
      SOC1[i][j] = xQueueCreate(1, sizeof(uint8_t));
      SOC2[i][j] = xQueueCreate(1, sizeof(uint8_t));
      DCIR[i][j] = xQueueCreate(1, sizeof(uint16_t));
      SOC[i][j] = xQueueCreate(1, sizeof(uint8_t));
    }
    BAL[i] = xQueueCreate(1, sizeof(uint16_t));
  }
  TSCurrent = xQueueCreate(1, sizeof(float));
  SOCCoulombCountingTime = xQueueCreate(1, sizeof(unsigned long));
  ADS1115Current = xQueueCreate(1, sizeof(int16_t));
  VAT4300Current = xQueueCreate(1, sizeof(float));
  TSVoltage = xQueueCreate(1, sizeof(float));
  TSPower = xQueueCreate(1, sizeof(float));
  VAT4300OnTimeSeconds = xQueueCreate(1, sizeof(int32_t));

  uint8_t soc_def = 0;
  uint16_t dcir_def = 3500;
  uint16_t cellV_def = 0;
  uint16_t bal_def = 0;
  float ts_current_def = 0.0;
  unsigned long time_def = 0;
  int16_t ads1115_current_def = 0;
  int32_t vat4300_time_def = 0;

  for (i = 0; i < NUM_SEG; i++) {
    for (j = 0; j < NUM_MOD_SEG; j++) {
      xQueueOverwrite(SOC1[i][j], &soc_def);
      xQueueOverwrite(SOC2[i][j], &soc_def);
      xQueueOverwrite(DCIR[i][j], &dcir_def);
      xQueueOverwrite(SOC[i][j], &soc_def);
      xQueueOverwrite(cellV[i][j], &cellV_def);
    }
    xQueueOverwrite(BAL[i], &bal_def);
  }
  xQueueOverwrite(TSCurrent, &ts_current_def);
  xQueueOverwrite(SOCCoulombCountingTime, &time_def);
  xQueueOverwrite(ADS1115Current, &ads1115_current_def);
  xQueueOverwrite(VAT4300Current, &ts_current_def);
  xQueueOverwrite(TSVoltage, &ts_current_def);
  xQueueOverwrite(TSPower, &ts_current_def);
  xQueueOverwrite(VAT4300OnTimeSeconds, &vat4300_time_def);

  linduino_serial_mutex = xSemaphoreCreateMutex();
  i2c_mutex = xSemaphoreCreateMutex();
  rf24_mutex = xSemaphoreCreateMutex();
  telnet_mutex = xSemaphoreCreateMutex();
  twai_mutex = xSemaphoreCreateMutex();

  I2CADS1115.begin(21, 22, 100000);
  pinMode(ADS1115_RDY_PIN, INPUT_PULLUP);
  pinMode(RF24_IRQ_PIN, INPUT_PULLUP);
  attachInterrupt(ADS1115_RDY_PIN, ads1115ISR, FALLING);
  attachInterrupt(RF24_IRQ_PIN, rf24ISR, FALLING);
  digitalWrite(SDC_CTRL_PIN, LOW);
  digitalWrite(PWR_LED_PIN, HIGH);

  xTaskCreatePinnedToCore(TaskLinduinoCVComms, "Linduino CV Comms", 4096, NULL, 1, &Linduino_CV_Comms_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskUpdateSOC, "SOC Update", 2048, NULL, 1, &Update_SOC_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskUpdateDCIR, "DCIR Update", 2048, NULL, 1, &Update_DCIR_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskADS1115, "ADS1115 Comms", 4096, NULL, 1, &ADS1115_Task_Handle, 0);
  xTaskCreatePinnedToCore(TaskVAT4300, "VAT4300 RF24 Comms", 8192, NULL, 1, &VAT4300_Task_Handle, 0);
  xTaskCreatePinnedToCore(TaskBalancing, "Module Balancing", 2048, NULL, 1, &Balancing_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskUpdateTSCurrent, "TS Current Update w ADS115 and VAT4300", 2048, NULL, 1, &Update_TS_Current_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskSDCFault, "SDC Fault monitor", 2048, NULL, 1, &SDC_Fault_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskTelnetStreamPrintDebug, "Print Debug", 4096, NULL, 1, &TelnetStream_Print_Debug_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskOTAHandleAlive, "OTA handle", 16384, NULL, 1, &OTA_Handle_Alive_Task_Handle, 1);
  xTaskCreatePinnedToCore(TaskCANComms, "CAN comms", 8192, NULL, 1, &CAN_Comms_Task_Handle, 1);
}

void TaskLinduinoCVComms(void *pvParameters) {
  uint8_t initCodes[NUM_SEG] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06 };
  uint8_t cellCodesL = NUM_MOD_SEG + 1;
  uint16_t cellCodes[cellCodesL];
  uint8_t x = 0x00;
  uint8_t crc = 0x00;
  uint16_t gen = 0x8380;
  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t temp_byte = 0;
  uint8_t first_complete = 0;
  uint16_t balance_config = 0;
  for (j = 0; j < NUM_MOD_SEG; j++) {
    cellCodes[j] = 0x0000;
  }
  while (1) {
    if (xSemaphoreTake(linduino_serial_mutex, portMAX_DELAY) == pdTRUE) {
      xQueuePeek(BAL[i], &balance_config, portMAX_DELAY);
      for (j = 0; j < 4; j++) {
        temp_byte = temp_byte | (initCodes[i] << 5);
        temp_byte = temp_byte | (j << 3);
        temp_byte = temp_byte | (uint8_t)((balance_config >> (j * 3)) & 0x0007);
        Serial2.write(temp_byte);
        temp_byte = 0;
        vTaskDelay(1 / portTICK_PERIOD_MS);
      }
      xSemaphoreGive(linduino_serial_mutex);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
    if (Serial2.available()) {
      x = Serial2.read();
      if (x != initCodes[i]) {
        while (Serial2.available()) {
          x = Serial2.read();
        }
        continue;
      } else {
        for (j = 0; j < NUM_MOD_SEG; j++) {
          x = Serial2.read();
          cellCodes[j] = (uint16_t)(x) << 8;
          x = Serial2.read();
          cellCodes[j] = cellCodes[j] | ((uint16_t)(x));
        }
        crc = Serial2.read();
        cellCodes[NUM_MOD_SEG] = 0x0000;
        uint8_t rsc = 0;
        uint8_t a = 0;
        int8_t b = 9;
        uint16_t tmp = 0x0000;
        uint16_t gen = 0x8380;
        uint16_t calc_crc = 0x0000;
        if (((cellCodes[0] & 0x8000) >> 15) == 0x0001) {
          calc_crc = (cellCodes[0] ^ gen);
        } else {
          calc_crc = cellCodes[0];
        }
        calc_crc = calc_crc & 0xFF80;
        while (rsc < 191) {
          tmp = uint16_t((calc_crc << 1) | ((cellCodes[a] << b) & 0x8000) >> 8);
          if (((tmp & 0x8000) >> 15) == 0x0001) {
            calc_crc = tmp ^ gen;
          } else {
            calc_crc = tmp;
          }
          calc_crc = calc_crc & 0xFF80;
          rsc++;
          if (b == 15) {
            b = -1;
            a++;
          }
          b++;
        }
        calc_crc = calc_crc >> 7;
        if (uint8_t(calc_crc) == crc) {
          for (j = 0; j < NUM_MOD_SEG; j++) {
            xQueueOverwrite(cellV[i][j], &cellCodes[j]);
          }
          i++;
          if (i >= NUM_SEG) {
            i = 0;
            if (first_complete == 0) {
              xTaskNotifyGive(Update_SOC_Task_Handle);
              first_complete = 1;
            }
          }
        }
      }
    }
  }
}

void TaskUpdateSOC(void *pvParameters) {
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  float disI = 0.0;
  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t a = 0;
  uint16_t dcir_temp = 0;
  float ocv_temp = 0;
  float soc1_temp = 0.0;
  float soc2_temp = 0.0;
  uint8_t soc1 = 0;
  uint8_t soc2 = 0;
  uint8_t soc2_old = 0;
  uint16_t cellV_temp;
  uint8_t first = 1;
  float alpha = 0.0;
  float soc_final_temp = 0.0;
  uint8_t soc_final = 0;
  uint8_t soc1_valid = 0;
  uint8_t soc2_valid = 0;
  unsigned long old_time = 0;
  unsigned long new_time = 0;
  uint16_t bal_config = 0;
  uint8_t bal_flag = 0;
  uint8_t soc_minl = SOC_MIN_LIMIT;
  uint8_t soc_maxl = SOC_MAX_LIMIT;
  while (1) {
    xQueuePeek(TSCurrent, &disI, portMAX_DELAY);
    alpha = max(0.0F, (1 - (abs(disI) / I_SOC_TRESHOLD)));
    for (i = 0; i < NUM_SEG; i++) {
      xQueuePeek(BAL[i], &bal_config, portMAX_DELAY);
      for (j = 0; j < NUM_MOD_SEG; j++) {
        xQueuePeek(DCIR[i][j], &dcir_temp, portMAX_DELAY);
        xQueuePeek(cellV[i][j], &cellV_temp, portMAX_DELAY);
        bal_flag = (uint8_t)((bal_config >> j) & 0x0001);
        ocv_temp = (cellV_temp / 10000.0) + (dcir_temp / 1000000.0) * (disI + bal_flag * (cellV_temp / (BALANCE_RES_VALUE * 10000.0)));
        if (ocv_temp > 4.05) {
          for (a = 0; a < 4; a++) {
            soc1_temp += ocv_to_soc_above_4_05[a] * pow(ocv_temp, a);
          }
        } else if (ocv_temp < 3.45) {
          for (a = 0; a < 4; a++) {
            soc1_temp += ocv_to_soc_below_3_45[a] * pow(ocv_temp, a);
          }
        } else {
          for (a = 0; a < 4; a++) {
            soc1_temp += ocv_to_soc_middle[a] * pow(ocv_temp, a);
          }
        }
        soc1 = uint8_t(soc1_temp);
        soc1 = constrain(soc1, (uint8_t)(SOC_MIN_LIMIT), (uint8_t)(SOC_MAX_LIMIT));
        soc1_temp = 0.0;
        if ((soc1 >= SOC_MIN_LIMIT) && (soc1 <= SOC_MAX_LIMIT)) {
          soc1_valid = 1;
          xQueueOverwrite(SOC1[i][j], &soc1);
          if (first == 1) {
            xQueueOverwrite(SOC2[i][j], &soc1);
            soc2 = soc1;
            soc2_valid = 1;
          }
        } 
        if (first != 1) {
          xQueuePeek(SOC2[i][j], &soc2_old, portMAX_DELAY);
          xQueuePeek(SOCCoulombCountingTime, &old_time, portMAX_DELAY);
          new_time = millis();
          soc2_temp = (((soc2_old / 100.0) * (MOD_CAPACITY) - (disI * ((new_time - old_time) / (1000 * 3600)))) / MOD_CAPACITY) * 100.0;
          xQueueOverwrite(SOCCoulombCountingTime, &new_time);
          soc2 = uint8_t(soc2_temp);
          soc2 = constrain(soc2, (uint8_t)(SOC_MIN_LIMIT), (uint8_t)(SOC_MAX_LIMIT));
          if ((soc2 >= SOC_MIN_LIMIT) && (soc2 <= SOC_MAX_LIMIT)) {
            soc2_valid = 1;
            xQueueOverwrite(SOC2[i][j], &soc2);
          }
        }
        soc_final_temp = alpha * soc1 + (1.0 - alpha) * soc2;
        soc_final = uint8_t(soc_final_temp);
        if ((soc1_valid == 1) && (soc2_valid == 1)) {
          xQueueOverwrite(SOC[i][j], &soc_final);
        }
        soc1_valid = 0;
        soc2_valid = 0;
      }
    }
    if (first == 1) {
      first = 0;
      xTaskNotifyGive(Update_DCIR_Task_Handle);
      xTaskNotifyGive(Balancing_Task_Handle);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void TaskUpdateDCIR(void *pvParameters) {
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  float disI = 0.0;
  uint8_t soc = 0;
  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t a = 0;
  float ocv_temp = 0.0;
  float dcir_temp = 0.0;
  uint16_t dcir_new = 0;
  uint16_t dcir_old = 0;
  uint16_t dcir_final = 0;
  uint16_t cellV_temp = 0;
  uint16_t bal_config = 0;
  uint8_t bal_flag = 0;

  while (1) {
    xQueuePeek(TSCurrent, &disI, portMAX_DELAY);
    if (disI > I_DCIR_TRESHOLD) {
      for (i = 0; i < NUM_SEG; i++) {
        xQueuePeek(BAL[i], &bal_config, portMAX_DELAY);
        for (j = 0; j < NUM_MOD_SEG; j++) {
          xQueuePeek(SOC[i][j], &soc, portMAX_DELAY);
          if (soc > 95) {
            for (a = 0; a < 4; a++) {
              ocv_temp += soc_to_ocv_above_95[a] * pow(soc, a);
            }
          } else if (soc < 20) {
            for (a = 0; a < 4; a++) {
              ocv_temp += soc_to_ocv_below_20[a] * pow(soc, a);
            }
          } else {
            for (a = 0; a < 4; a++) {
              ocv_temp += soc_to_ocv_middle[a] * pow(soc, a);
            }
          }
          xQueuePeek(cellV[i][j], &cellV_temp, portMAX_DELAY);
          bal_flag = (uint8_t)((bal_config >> j) & 0x0001);
          dcir_temp = ((ocv_temp - cellV_temp) / (disI + bal_flag * (cellV_temp / (10000.0 * BALANCE_RES_VALUE)))) * 100.0;
          ocv_temp = 0.0;
          if (dcir_temp > 0.0) {
            dcir_new = uint16_t(dcir_temp);
            xQueuePeek(DCIR[i][j], &dcir_old, portMAX_DELAY);
            if ((dcir_new > DCIR_MIN_LIMIT) && (dcir_new < DCIR_MAX_LIMIT) && (abs(dcir_new - dcir_old) < DCIR_JUMP_LIMIT)) {
              dcir_final = uint16_t((1.0 - BETA) * dcir_old + BETA * dcir_new);
              xQueueOverwrite(DCIR[i][j], &dcir_final);
            }
          }
        }
      }
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void TaskADS1115(void *pvParameters) {
  uint8_t i = 0;
  uint8_t f = 1;
  uint8_t r = 0;
  uint8_t curReads = 0;
  int16_t cur = 0;
  int16_t first_sum = 0;
  uint8_t loop_count = 0;

  while (1) {
    switch (i) {
      case 0:
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
          I2CADS1115.beginTransmission(0x48);
          I2CADS1115.write(0x01);
          I2CADS1115.write(0x03);
          I2CADS1115.write(0x80);
          f = I2CADS1115.endTransmission();
          xSemaphoreGive(i2c_mutex);
        }
        if (f == 0) {
          i = 1;
          f = 1;
        }
        break;
      case 1:
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
          I2CADS1115.beginTransmission(0x48);
          I2CADS1115.write(0x02);
          I2CADS1115.write(0x00);
          I2CADS1115.write(0x00);
          f = I2CADS1115.endTransmission();
          xSemaphoreGive(i2c_mutex);
        }
        if (f == 0) {
          i = 2;
          f = 1;
        }
        break;
      case 2:
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
          I2CADS1115.beginTransmission(0x48);
          I2CADS1115.write(0x03);
          I2CADS1115.write(0x80);
          I2CADS1115.write(0x00);
          f = I2CADS1115.endTransmission();
          xSemaphoreGive(i2c_mutex);
        }
        if (f == 0) {
          i = 3;
          f = 1;
        }
        break;
      case 3:
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
          I2CADS1115.beginTransmission(0x48);
          I2CADS1115.write(0x01);
          I2CADS1115.write(0x83);
          I2CADS1115.write(0x80);
          f = I2CADS1115.endTransmission();
          xSemaphoreGive(i2c_mutex);
        }
        if (f == 0) {
          f = 1;
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
            I2CADS1115.beginTransmission(0x48);
            I2CADS1115.write(0x00);
            I2CADS1115.endTransmission();
            r = I2CADS1115.requestFrom(0x48, 2);
            xSemaphoreGive(i2c_mutex);
          }
          if (r == 2) {
            i = 4;
            r = 0;
          }
        }
        break;
      case 4:
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
          if (I2CADS1115.available() >= 2) {
            cur = I2CADS1115.read();
            cur = (cur << 8) | (I2CADS1115.read());
          }
          xSemaphoreGive(i2c_mutex);
        }
        if (loop_count < 10) {
          first_sum += cur;
          loop_count++;
        } else if (loop_count == 10) {
          ADS1115_bias = first_sum / 10.0;
          loop_count++;
        } else {
          if (ADS1115Current != NULL) {
            xQueueOverwrite(ADS1115Current, &cur);
          }
        }
        if (curReads < ADS1115_CONFIG_REWRITE_CYCLES) {
          i = 3;
          curReads++;
        } else {
          i = 0;
          curReads = 0;
        }
        break;
      default:
        i = 0;
        break;
    }
    vTaskDelay(2 / portTICK_PERIOD_MS);
  }
}

void TaskVAT4300(void *pvParameters) {
  uint8_t i = 0;
  uint64_t pipes[2] = { 0x8967452300LL, 0x8967452300LL };
  const int VAT4300_address = 5;
  const char VAT4300_frequency = 'A';
  uint8_t aTX_PAYLOAD[32] = { 0xAA, 0x04, 0x01, VAT4300_address, 0x00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00 };
  uint8_t aRX_PAYLOAD[32] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00 };
  uint8_t a = 0;
  uint32_t notified = 0;
  float voltage = 0.0;
  float amperage = 0.0;
  float wattage = 0.0;
  float ampHours = 0.0;
  float wattHours = 0.0;
  int32_t seconds = 0;
  float first_sum = 0.0;
  uint8_t loop_count = 0;
  uint8_t tx_rt = 0;
  uint8_t first_tx = 1;

  RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);

  while (1) {
    switch (i) {
      case 0:
        if (!radio.begin()) {
          i = 0;
        } else {
          if (xSemaphoreTake(rf24_mutex, portMAX_DELAY) == pdTRUE) {
            radio.setAutoAck(false);
            radio.setDataRate(RF24_2MBPS);
            for (a = 0; a < 6; a++) {
              radio.closeReadingPipe(i);
            }
            pipes[0] |= (uint64_t)VAT4300_address;
            pipes[1] |= (uint64_t)VAT4300_address;
            radio.openWritingPipe(pipes[0]);
            radio.openReadingPipe(0, pipes[1]);
            radio.setAutoAck(0, true);
            radio.setChannel((VAT4300_frequency - 'A') * 4);
            radio.setPALevel(RF24_PA_MAX);
            if (!radio.failureDetected) {
              i = 1;
            }
            xSemaphoreGive(rf24_mutex);
          }
        }
        break;
      case 1:
        if (xSemaphoreTake(rf24_mutex, portMAX_DELAY) == pdTRUE) {
          if (radio.getDataRate() != RF24_2MBPS) {
            i = 0;
          } else {
            radio.stopListening();
            if (first_tx == 1) {
              aTX_PAYLOAD[2] = 0xFF;
              aTX_PAYLOAD[4] = VAT4300_address + 0xAD;
            } else {
              aTX_PAYLOAD[2] = 0x01;
              aTX_PAYLOAD[4] = VAT4300_address + 0xAF;
            }
            if (!radio.write(&aTX_PAYLOAD, sizeof(aTX_PAYLOAD))) {
              i = 0;
              xTaskNotifyStateClear(VAT4300_Task_Handle);
            } else {
              xTaskNotifyStateClear(VAT4300_Task_Handle);
              radio.startListening();
              notified = ulTaskNotifyTake(pdTRUE, 1000 / portTICK_PERIOD_MS);
              if (notified > 0) {
                tx_rt = 0;
                i = 2;
              } else {
                tx_rt++;
                if (tx_rt > VAT4300_RESET_CONFIG_RETRIES) {
                  i = 0;
                  tx_rt = 0;
                } else {
                  i = 1;
                }
              }
              notified = 0;
            }
          }
          xSemaphoreGive(rf24_mutex);
        }
        break;
      case 2:
        if (xSemaphoreTake(rf24_mutex, portMAX_DELAY) == pdTRUE) {
          while (radio.available()) {
            radio.read(&aRX_PAYLOAD, sizeof(aRX_PAYLOAD));
          }
          xSemaphoreGive(rf24_mutex);
          if (first_tx == 1) {
            first_tx = 0;
            i = 1;
          } else {
            i = 3;
          }
        }
        break;
      case 3:
        voltage = (int16_t)(aRX_PAYLOAD[4] << 8 | aRX_PAYLOAD[5]) / 100.0;
        amperage = (int16_t)(aRX_PAYLOAD[6] << 8 | aRX_PAYLOAD[7]) / 10.0;
        wattage = (int32_t)(aRX_PAYLOAD[8] << 24 | aRX_PAYLOAD[9] << 16 | aRX_PAYLOAD[10] << 8 | aRX_PAYLOAD[11]) / 1000.0;
        ampHours = (int32_t)(aRX_PAYLOAD[12] << 24 | aRX_PAYLOAD[13] << 16 | aRX_PAYLOAD[14] << 8 | aRX_PAYLOAD[15]) / 1000.0;
        wattHours = (int32_t)(aRX_PAYLOAD[16] << 24 | aRX_PAYLOAD[17] << 16 | aRX_PAYLOAD[18] << 8 | aRX_PAYLOAD[19]) / 1000.0;
        seconds = (int32_t)(aRX_PAYLOAD[20] << 24 | aRX_PAYLOAD[21] << 16 | aRX_PAYLOAD[22] << 8 | aRX_PAYLOAD[23]);
        if (loop_count < 10) {
          first_sum += amperage;
          loop_count++;
        } else if (loop_count == 10) {
          VAT4300_bias = first_sum / 10.0;
          loop_count++;
        } else {
          xQueueOverwrite(TSVoltage, &voltage);
          xQueueOverwrite(VAT4300Current, &amperage);
          xQueueOverwrite(TSPower, &wattage);
          xQueueOverwrite(VAT4300OnTimeSeconds, &seconds);
        }
        i = 1;
        break;
      default:
        i = 0;
        break;
    }
    vTaskDelay(25 / portTICK_PERIOD_MS);
  }
}

void TaskBalancing(void *pvParameters) {
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t min_soc[NUM_SEG];
  uint8_t soc_temp = 0;
  uint16_t bal_temp = 0;

  for (i = 0; i < NUM_SEG; i++) {
    min_soc[i] = 100;
  }

  while (1) {
    for (i = 0; i < NUM_SEG; i++) {
      for (j = 0; j < NUM_MOD_SEG; j++) {
        xQueuePeek(SOC[i][j], &soc_temp, portMAX_DELAY);
        if (soc_temp < min_soc[i]) {
          min_soc[i] = soc_temp;
        }
      }
    }
    for (i = 0; i < NUM_SEG; i++) {
      for (j = 0; j < NUM_MOD_SEG; j++) {
        xQueuePeek(SOC[i][j], &soc_temp, portMAX_DELAY);
        if ((soc_temp - min_soc[i]) > BALANCE_TRESHOLD) {
          bal_temp = bal_temp | (1 << j);
        }
      }
      xQueueOverwrite(BAL[i], &bal_temp);
      bal_temp = 0;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void TaskUpdateTSCurrent(void *pvParameters) {
  float ADS1115_I = 0.0;
  float VAT4300_I = 0.0;
  int16_t ADS1115_raw = 0;
  float VAT4300_I_old = 0.0;

  while (1) {
    xQueuePeek(ADS1115Current, &ADS1115_raw, portMAX_DELAY);
    xQueuePeek(VAT4300Current, &VAT4300_I, portMAX_DELAY);
    if (VAT4300_I != VAT4300_I_old) {
      VAT4300_I = VAT4300_I - VAT4300_bias;
      xQueueOverwrite(TSCurrent, &VAT4300_I);
      VAT4300_I_old = VAT4300_I;
    } else {
      ADS1115_I = (((float)(ADS1115_raw)) - ADS1115_bias) * HBO300_ADS1115_CURRENT_MULTIPLY_FACTOR;
      xQueueOverwrite(TSCurrent, &ADS1115_I);
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void TaskSDCFault(void *pvParameters) {
  esp_task_wdt_add(NULL);
  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t soc_flag = 0;
  uint8_t soc_temp = 0;
  uint8_t I_mismatch_flag = 0;
  float VAT4300_I_old = 0.0;
  float VAT4300_I = 0.0;
  float ADS1115_I = 0.0;
  int16_t ADS1115_raw = 0;
  uint16_t cellV_temp = 0;
  uint16_t cellV_XOR_old[NUM_SEG];
  uint16_t cellV_XOR[NUM_SEG];
  float TSCurrent_temp = 0.0;
  uint8_t cellV_flag = 0;
  for (i = 0; i < NUM_SEG; i++) {
    cellV_XOR_old[i] = 0;
  }
  uint8_t flag[3] = { 0, 0, 0 };
  uint8_t first_loop = 1;

  while (1) {
    soc_flag = 0;
    for (i = 0; i < NUM_SEG; i++) {
      for (j = 0; j < NUM_MOD_SEG; j++) {
        xQueuePeek(SOC[i][j], &soc_temp, portMAX_DELAY);
        if ((soc_temp > SOC_MAX_FAULT || soc_temp < SOC_MIN_FAULT) && soc_temp != 0) {
          soc_flag = 1;
        }
      }
    }
    I_mismatch_flag = 0;
    xQueuePeek(VAT4300Current, &VAT4300_I, portMAX_DELAY);
    VAT4300_I = VAT4300_I - VAT4300_bias;
    if (VAT4300_I != VAT4300_I_old) {
      VAT4300_I_old = VAT4300_I;
      xQueuePeek(ADS1115Current, &ADS1115_raw, portMAX_DELAY);
      ADS1115_I = float(ADS1115_raw - ADS1115_bias) * HBO300_ADS1115_CURRENT_MULTIPLY_FACTOR;
      if (abs(ADS1115_I - VAT4300_I) > CURRENT_DEV_FAULT) {
        I_mismatch_flag = 1;
      }
    }
    for (i = 0; i < NUM_SEG; i++) {
      for (j = 0; j < NUM_MOD_SEG; j++) {
        xQueuePeek(cellV[i][j], &cellV_temp, portMAX_DELAY);
        cellV_XOR[i] ^= cellV_temp;
      }
    }
    xQueuePeek(TSCurrent, &TSCurrent_temp, portMAX_DELAY);
    cellV_flag = 0;
    for (i = 0; i < NUM_SEG; i++) {
      if ((cellV_XOR[i] == cellV_XOR_old[i]) && (TSCurrent_temp > CELLV_CHANGE_CURRENT)) {
        cellV_flag = 1;
      }
      cellV_XOR_old[i] = cellV_XOR[i];
    }
    if (soc_flag == 1) {
      flag[0]++;
    } else {
      flag[0] = 0;
    }
    if (I_mismatch_flag == 1) {
      flag[1]++;
    } else {
      flag[1] = 1;
    }
    if (cellV_flag == 1) {
      flag[2]++;
    } else {
      flag[2] = 0;
    }

    if (flag[0] > 10 || flag[1] > 10 || flag[2] > 10) {
      digitalWrite(SDC_CTRL_PIN, LOW);
      digitalWrite(PWR_LED_PIN, HIGH);
      vTaskSuspend(Linduino_CV_Comms_Task_Handle);
      vTaskSuspend(Update_SOC_Task_Handle);
      vTaskSuspend(Update_DCIR_Task_Handle);
      vTaskSuspend(Balancing_Task_Handle);
      if (xSemaphoreTake(telnet_mutex, portMAX_DELAY) == pdTRUE) {
        TelnetStream.print("\r\nFAULT!!!\r\n");
        TelnetStream.print(flag[0]);
        TelnetStream.print(" : ");
        TelnetStream.print(flag[1]);
        TelnetStream.print(" : ");
        TelnetStream.print(flag[2]);
        TelnetStream.print("\r\n");
        xSemaphoreGive(telnet_mutex);
      }
      while (1) {
        vTaskDelay(portMAX_DELAY);
      }
    }

    digitalWrite(SDC_CTRL_PIN, HIGH);
    digitalWrite(PWR_LED_PIN, !digitalRead(PWR_LED_PIN));
    esp_task_wdt_reset();
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void TaskTelnetStreamPrintDebug(void *pvParameters) {
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  uint8_t user_input = 0;
  uint8_t serial_char = 0;
  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t soc_temp = 0;
  uint16_t dcir_temp = 0;
  uint16_t cellV_temp = 0;
  float VAT4300_I = 0.0;
  int16_t ADS1115_raw = 0;
  float TSCurrent_temp = 0.0;
  unsigned long VAT4300_time_temp = 0;
  uint8_t inst = 0;
  while (1) {
    if (TelnetStream.available()) {
      serial_char = TelnetStream.read();
      if (((int)(serial_char) >= 48) && ((int)(serial_char) <= 57)) {
        user_input = serial_char - 48;
      } else {
        continue;
      }
    }
    switch (user_input) {
      case 0:
        if (inst == 0) {
          if (xSemaphoreTake(telnet_mutex, portMAX_DELAY) == pdTRUE) {
            TelnetStream.print("Instructions to print debug info\r\n");
            TelnetStream.print("Enter 1x to print SOC of x (0...) segment\r\n");
            TelnetStream.print("Enter 2x to print DCIR of x (0...) segment\r\n");
            TelnetStream.print("Enter 3x to print CellV of x (0...) segment\r\n");
            TelnetStream.print("Enter 4 to print VAT4300 current value\r\n");
            TelnetStream.print("Enter 5 to print ADS1115 raw values\r\n");
            TelnetStream.print("Enter 6 to print TS current value\r\n");
            TelnetStream.print("Enter 7 for current bias\r\n");
            TelnetStream.print("Enter 8 for VAT4300 on time\r\n");
            TelnetStream.print("Enter 0 for instructions\r\n");
            xSemaphoreGive(telnet_mutex);
            inst = 1;
          }
        }
        while (!TelnetStream.available()) {
          inst = 0;
        }
        break;
      case 1:
        inst = 0;
        if (TelnetStream.available()) {
          i = TelnetStream.read();
          i = i - 48;
        }
        for (j = 0; j < NUM_MOD_SEG; j++) {
          xQueuePeek(SOC[i][j], &soc_temp, portMAX_DELAY);
          if (xSemaphoreTake(telnet_mutex, portMAX_DELAY) == pdTRUE) {
            TelnetStream.print(j);
            TelnetStream.print(":");
            TelnetStream.print(soc_temp);
            TelnetStream.print(" ");
            xSemaphoreGive(telnet_mutex);
          }
        }
        TelnetStream.print("\r\n");
        break;
      case 2:
        inst = 0;
        if (TelnetStream.available()) {
          i = TelnetStream.read();
          i = i - 48;
        }
        for (j = 0; j < NUM_MOD_SEG; j++) {
          xQueuePeek(DCIR[i][j], &dcir_temp, portMAX_DELAY);
          if (xSemaphoreTake(telnet_mutex, portMAX_DELAY) == pdTRUE) {
            TelnetStream.print(j);
            TelnetStream.print(":");
            TelnetStream.print(dcir_temp);
            TelnetStream.print(" ");
            xSemaphoreGive(telnet_mutex);
          }
        }
        TelnetStream.print("\r\n");
        break;
      case 3:
        inst = 0;
        if (TelnetStream.available()) {
          i = TelnetStream.read();
          i = i - 48;
        }
        for (j = 0; j < NUM_MOD_SEG; j++) {
          xQueuePeek(cellV[i][j], &cellV_temp, portMAX_DELAY);
          if (xSemaphoreTake(telnet_mutex, portMAX_DELAY) == pdTRUE) {
            TelnetStream.print(j);
            TelnetStream.print(":");
            TelnetStream.print(cellV_temp);
            TelnetStream.print(" ");
            xSemaphoreGive(telnet_mutex);
          }
        }
        TelnetStream.print("\r\n");
        break;
      case 4:
        inst = 0;
        xQueuePeek(VAT4300Current, &VAT4300_I, portMAX_DELAY);
        if (xSemaphoreTake(telnet_mutex, portMAX_DELAY) == pdTRUE) {
          TelnetStream.print(VAT4300_I);
          TelnetStream.print("\r\n");
          xSemaphoreGive(telnet_mutex);
        }
        break;
      case 5:
        inst = 0;
        xQueuePeek(ADS1115Current, &ADS1115_raw, portMAX_DELAY);
        if (xSemaphoreTake(telnet_mutex, portMAX_DELAY) == pdTRUE) {
          TelnetStream.print(ADS1115_raw);
          TelnetStream.print("\r\n");
          xSemaphoreGive(telnet_mutex);
        }
        break;
      case 6:
        inst = 0;
        xQueuePeek(TSCurrent, &TSCurrent_temp, portMAX_DELAY);
        if (xSemaphoreTake(telnet_mutex, portMAX_DELAY) == pdTRUE) {
          TelnetStream.print(TSCurrent_temp);
          TelnetStream.print("\r\n");
          xSemaphoreGive(telnet_mutex);
        }
        break;
      case 7:
        inst = 0;
        if (xSemaphoreTake(telnet_mutex, portMAX_DELAY) == pdTRUE) {
          TelnetStream.print("VAT4300 current bias: ");
          TelnetStream.print(VAT4300_bias);
          TelnetStream.print(" ADS1115 raw bias: ");
          TelnetStream.print(ADS1115_bias);
          TelnetStream.print("\r\n");
          xSemaphoreGive(telnet_mutex);
        }
        break;
      case 8:
        inst = 0;
        xQueuePeek(VAT4300OnTimeSeconds, &VAT4300_time_temp, portMAX_DELAY);
        if (xSemaphoreTake(telnet_mutex, portMAX_DELAY) == pdTRUE) {
          TelnetStream.print(VAT4300_time_temp);
          TelnetStream.print("\r\n");
          xSemaphoreGive(telnet_mutex);
        }
        break;
      default:
        user_input = 0;
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void TaskOTAHandleAlive(void *pvParameters) {
  bool setup_ret = false;
  ArduinoOTA.setHostname("ESP32_BMS");
  while (setup_ret != true) {
    setup_ret = setupOTA("", my_ssid, my_password);
    if (setup_ret != true) {
      vTaskDelay(OTA_RETRY_DELAY_MS / portTICK_PERIOD_MS);
    }
  }
  xTaskNotifyGive(TelnetStream_Print_Debug_Task_Handle);
  while (1) {
    ArduinoOTA.handle();
    vTaskDelay(37 / portTICK_PERIOD_MS);
  }
}

void TaskCANComms(void *pvParameters) {
  CanFrame TxFrame = { 0 };
  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t a = 0;
  uint8_t b = 0;
  float TSCurrent_temp = 0.0;
  float TSVoltage_temp = 0.0;
  float TSPower_temp = 0.0;
  uint32_t tempx = 0x00000000;
  uint32_t tempz = 0x00000000;
  uint8_t tempy = 0x00;
  int16_t ads1115_temp = 0;
  float VAT4300_temp = 0.0;
  float ads1115_cur_temp = 0.0;
  uint16_t soc_total = 0;
  uint8_t soc_avg = 0;
  uint16_t cellV_temp = 0;
  uint8_t soc_temp = 0;
  float cellV_total = 0.0;
  uint16_t dcir_temp = 0;
  uint16_t dcir_total = 0;
  while (1) {
    switch (i) {
      case 0:
        xQueuePeek(TSCurrent, &TSCurrent_temp, portMAX_DELAY);
        xQueuePeek(TSVoltage, &TSVoltage_temp, portMAX_DELAY);
        TxFrame.identifier = 0x5AA;
        TxFrame.extd = 0;
        TxFrame.data_length_code = 8;
        memcpy(&tempz, &TSCurrent_temp, sizeof(float));
        for (j = 0; j < 4; j++) {
          tempx = (tempz >> (j * 8)) & 0x000000FF;
          tempy = (uint8_t)(tempx);
          TxFrame.data[j] = tempy;
        }
        memcpy(&tempz, &TSVoltage_temp, sizeof(float));
        for (j = 0; j < 4; j++) {
          tempx = (tempz >> (j * 8)) & 0x000000FF;
          tempy = (uint8_t)(tempx);
          TxFrame.data[j + 4] = tempy;
        }
        if (xSemaphoreTake(twai_mutex, portMAX_DELAY) == pdTRUE) {
          ESP32Can.writeFrame(TxFrame, 0);
          xSemaphoreGive(twai_mutex);
        }
        i = 1;
        break;
      case 1:
        xQueuePeek(TSPower, &TSPower_temp, portMAX_DELAY);
        TxFrame.identifier = 0x5AB;
        TxFrame.extd = 0;
        TxFrame.data_length_code = 4;
        memcpy(&tempz, &TSPower_temp, sizeof(float));
        for (j = 0; j < 4; j++) {
          tempx = (tempz >> (j * 8)) & 0x000000FF;
          tempy = (uint8_t)(tempx);
          TxFrame.data[j] = tempy;
        }
        if (xSemaphoreTake(twai_mutex, portMAX_DELAY) == pdTRUE) {
          ESP32Can.writeFrame(TxFrame, 0);
          xSemaphoreGive(twai_mutex);
        }
        i = 2;
        break;
      case 2:
        xQueuePeek(ADS1115Current, &ads1115_temp, portMAX_DELAY);
        xQueuePeek(VAT4300Current, &VAT4300_temp, portMAX_DELAY);
        TxFrame.identifier = 0x5AC;
        TxFrame.extd = 0;
        TxFrame.data_length_code = 8;
        ads1115_cur_temp = (ads1115_temp - ADS1115_bias) * HBO300_ADS1115_CURRENT_MULTIPLY_FACTOR;
        memcpy(&tempz, &ads1115_cur_temp, sizeof(float));
        for (j = 0; j < 4; j++) {
          tempx = (tempz >> (j * 8)) & 0x000000FF;
          tempy = (uint8_t)(tempx);
          TxFrame.data[j] = tempy;
        }
        VAT4300_temp = VAT4300_temp - VAT4300_bias;
        memcpy(&tempz, &VAT4300_temp, sizeof(float));
        for (j = 0; j < 4; j++) {
          tempx = (tempz >> (j * 8)) & 0x000000FF;
          tempy = (uint8_t)(tempx);
          TxFrame.data[j + 4] = tempy;
        }
        if (xSemaphoreTake(twai_mutex, portMAX_DELAY) == pdTRUE) {
          ESP32Can.writeFrame(TxFrame, 0);
          xSemaphoreGive(twai_mutex);
        }
        i = 3;
        break;
      case 3:
        soc_total = 0;
        soc_avg = 0;
        cellV_total = 0.0;
        dcir_total = 0;
        for (a = 0; a < NUM_SEG; a++) {
          for (b = 0; b < NUM_MOD_SEG; b++) {
            xQueuePeek(SOC[a][b], &soc_temp, portMAX_DELAY);
            xQueuePeek(cellV[a][b], &cellV_temp, portMAX_DELAY);
            xQueuePeek(DCIR[a][b], &dcir_temp, portMAX_DELAY);
            soc_total += soc_temp;
            cellV_total += (cellV_temp / 10000.0);
            dcir_total += (dcir_temp / 1000);
          }
        }
        soc_avg = soc_total / (NUM_SEG * NUM_MOD_SEG);
        TxFrame.identifier = 0x5AD;
        TxFrame.extd = 0;
        TxFrame.data_length_code = 7;
        TxFrame.data[0] = soc_avg;
        memcpy(&tempz, &cellV_total, sizeof(float));
        for (j = 0; j < 4; j++) {
          tempx = (tempz >> (j * 8)) & 0x000000FF;
          tempy = (uint8_t)(tempx);
          TxFrame.data[j + 1] = tempy;
        }
        tempy = (uint8_t)(dcir_total & 0x00FF);
        TxFrame.data[5] = tempy;
        tempy = (uint8_t)((dcir_total >> 8) & 0x00FF);
        TxFrame.data[6] = tempy;
        if (xSemaphoreTake(twai_mutex, portMAX_DELAY) == pdTRUE) {
          ESP32Can.writeFrame(TxFrame, 0);
          xSemaphoreGive(twai_mutex);
        }
        i = 0;
        break;
      default:
        i = 0;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
void loop() {
}