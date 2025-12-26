#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"  //built in header files
#include "LT_SPI.h"
#include "LTC68041.h"
#include <SPI.h>

#define TOTAL_IC 6

uint16_t cell_codes[TOTAL_IC][12];
uint8_t tx_cfg[TOTAL_IC][6];
 uint32_t bal_reset = 0;
 
void init_cfg();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);
  LTC6804_initialize();
  init_cfg();
  wakeup_sleep();
  LTC6804_wrcfg(TOTAL_IC, tx_cfg);
  delay(10);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t c1 = 0x00;
  uint8_t c2 = 0x00;
  uint8_t c3 = 0x00;
  uint8_t c4 = 0x00;
  uint8_t seg1 = 0x00;
  uint8_t seg2 = 0x00;
  uint8_t seg3 = 0x00;
  uint8_t seg4 = 0x00;
  uint8_t j1 = 0x00;
  uint8_t j2 = 0x00;
  uint8_t j3 = 0x00;
  uint8_t j4 = 0x00;
  uint8_t i = 0;
  uint8_t j = 0;
  int error = -1;
  uint8_t rsc = 0;
  uint8_t a = 0;
  int8_t b = 9;
  uint16_t tmp = 0x0000;
  uint16_t gen = 0x8380;
  uint16_t calc_crc = 0x0000;
  uint16_t bal_cfg = 0x0000;
  uint8_t cellVsend[26];
  uint16_t cell_codes_temp[13];
 
  for (i = 0; i < 26; i++) {
    cellVsend[i] = 0;
  }
  if (Serial.available() >= 4) {
    c1 = Serial.read();
    c2 = Serial.read();
    c3 = Serial.read();
    c4 = Serial.read();
    seg1 = c1 >> 5;
    seg2 = c2 >> 5;
    seg3 = c3 >> 5;
    seg4 = c4 >> 5;
    if ((seg1 == seg2) && (seg2 == seg3) && (seg3 == seg4)) {
      wakeup_idle();
      LTC6804_adcv();
      delay(10);
      wakeup_idle();
      error = LTC6804_rdcv(0, TOTAL_IC, cell_codes);
      if (error != -1) {
        cellVsend[0] = seg1;
        for (i = 0; i < 12; i++) {
          cellVsend[2 * i + 1] = (uint8_t)((cell_codes[seg1 - 1][i] >> 8) & 0x00FF);
          cellVsend[2 * i + 2] = (uint8_t)(cell_codes[seg1 - 1][i] & 0x00FF);
        }
        rsc = 0;
        a = 0;
        b = 9;
        tmp = 0x0000;
        gen = 0x8380;
        calc_crc = 0x0000;
        for (i = 0; i < 12; i++) {
          cell_codes_temp[i] = cell_codes[seg1 - 1][i];
        }
        cell_codes_temp[12] = 0x0000;
        if (((cell_codes_temp[0] & 0x8000) >> 15) == 0x0001) {
          calc_crc = (cell_codes_temp[0] ^ gen);
        } else {
          calc_crc = cell_codes_temp[0];
        }
        calc_crc = calc_crc & 0xFF80;
        while (rsc < 191) {
          tmp = uint16_t((calc_crc << 1) | ((cell_codes_temp[a] << b) & 0x8000) >> 8);
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
        cellVsend[25] = (uint8_t)(calc_crc);
        for (i = 0; i < 26; i++) {
          Serial.write(cellVsend[i]);
          delay(1);
        }
      }
      tx_cfg[seg1 - 1][4] = 0x00;
      tx_cfg[seg1 - 1][5] = 0x00;
      j1 = (c1 >> 3) & 0x03;
      j2 = (c2 >> 3) & 0x03;
      j3 = (c3 >> 3) & 0x03;
      j4 = (c4 >> 3) & 0x03;
      bal_cfg = bal_cfg | (((uint16_t)(c1 & 0x07)) << (j1 * 3));
      bal_cfg = bal_cfg | (((uint16_t)(c2 & 0x07)) << (j2 * 3));
      bal_cfg = bal_cfg | (((uint16_t)(c3 & 0x07)) << (j3 * 3));
      bal_cfg = bal_cfg | (((uint16_t)(c4 & 0x07)) << (j4 * 3));
      tx_cfg[seg1 - 1][4] = (uint8_t)(bal_cfg & 0x00FF);
      tx_cfg[seg1 - 1][5] = (uint8_t)((bal_cfg >> 8) & 0x000F);
      wakeup_sleep();
      LTC6804_wrcfg(TOTAL_IC, tx_cfg);
      delay(10);
    }
  }
  if ((millis() - bal_reset) > 120000) {
    bal_reset = millis();
    init_cfg();
    wakeup_sleep();
    LTC6804_wrcfg(TOTAL_IC, tx_cfg);
    delay(10);
  }
}

void init_cfg() {
  for (short i = 0; i < TOTAL_IC; i++) {
    tx_cfg[i][0] = 0xFE;
    tx_cfg[i][1] = 0x00;
    tx_cfg[i][2] = 0x00;
    tx_cfg[i][3] = 0x00;
    tx_cfg[i][4] = 0x00;
    tx_cfg[i][5] = 0x00;
  }
}
