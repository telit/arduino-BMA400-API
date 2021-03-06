/*Copyright (C) 2021 Telit Communications S.p.A. Italy - All Rights Reserved.*/
/*    See LICENSE file in the root folder for full license information.     */

/*!
 * @ingroup bma400Examples
 * @defgroup bma400ExamplesTapDetection Tap detection
 * @brief To showcase tap detection feature
 * \include tap_detection.ino
 */

#include <telit_bma400.h>

void setup() {

  struct bma400_dev bma;

  int8_t rslt = 0;
  uint16_t int_status = 0;
  int8_t count = 0;

  struct bma400_sensor_conf conf[2];
  struct bma400_int_enable int_en[2];
  
  memset(conf, 0, sizeof(struct bma400_sensor_conf));
  Serial.begin(115200);
  delay(1000);


  rslt = bma400_interface_init(&bma, BMA400_I2C_INTF);
  bma400_check_rslt("bma400_interface_init", rslt);

  rslt = bma400_soft_reset(&bma);
  bma400_check_rslt("bma400_soft_reset", rslt);

  rslt = bma400_init(&bma);
  bma400_check_rslt("bma400_init", rslt);

  conf[0].type = BMA400_ACCEL;
  conf[1].type = BMA400_TAP_INT;

  rslt = bma400_get_sensor_conf(conf, 2, &bma);
  bma400_check_rslt("bma400_get_sensor_conf", rslt);

  conf[0].param.accel.odr = BMA400_ODR_200HZ;
  conf[0].param.accel.range = BMA400_RANGE_16G;
  conf[0].param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;
  conf[0].param.accel.filt1_bw = BMA400_ACCEL_FILT1_BW_1;

  conf[1].param.tap.int_chan = BMA400_INT_CHANNEL_1;
  conf[1].param.tap.axes_sel = BMA400_TAP_X_AXIS_EN | BMA400_TAP_Y_AXIS_EN | BMA400_TAP_Z_AXIS_EN;
  conf[1].param.tap.sensitivity = BMA400_TAP_SENSITIVITY_0;
  conf[1].param.tap.tics_th = BMA400_TICS_TH_6_DATA_SAMPLES;
  conf[1].param.tap.quiet = BMA400_QUIET_60_DATA_SAMPLES;
  conf[1].param.tap.quiet_dt = BMA400_QUIET_DT_4_DATA_SAMPLES;

  rslt = bma400_set_sensor_conf(conf, 2, &bma);
  bma400_check_rslt("bma400_set_sensor_conf", rslt);

  rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, &bma);
  bma400_check_rslt("bma400_set_power_mode", rslt);

  int_en[0].type = BMA400_SINGLE_TAP_INT_EN;
  int_en[0].conf = BMA400_ENABLE;

  int_en[1].type = BMA400_DOUBLE_TAP_INT_EN;
  int_en[1].conf = BMA400_ENABLE;

  rslt = bma400_enable_interrupt(int_en, 2, &bma);
  bma400_check_rslt("bma400_enable_interrupt", rslt);

  Serial.println("Single and Double Tap interrupts enabled");
  Serial.println("Perform single or double tap on the board");


  while (1)
  {
    rslt = bma400_get_interrupt_status(&int_status, &bma);
    bma400_check_rslt("bma400_get_interrupt_status", rslt);

    if (rslt == BMA400_OK)
    {
      if (int_status & BMA400_ASSERTED_S_TAP_INT)
      {
        Serial.println("Single tap detected");
        count++;
      }

      if (int_status & BMA400_ASSERTED_D_TAP_INT)
      {
        Serial.println("Double tap detected");
        count++;
      }

      if (count == 5)
      {
        Serial.println("Tap interrupt testing done. Exiting !");
        break;
      }
    }
    delay(10);
  }

}

void loop() {
}

