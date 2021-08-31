/*Copyright (C) 2021 Telit Communications S.p.A. Italy - All Rights Reserved.*/
/*    See LICENSE file in the root folder for full license information.     */

/*!
 * @ingroup bma400Examples
 * @defgroup bma400ExamplesAccelerometer Accelerometer read
 * @brief To read accelerometer xyz data for defined range and ODR
 * \include activity_change.ino
 */

#include <telit_bma400.h>

/* Macro to determine count of activity change for each axis */
#define BMA400_INT_COUNTER  UINT8_C(5)

/* struct to act as counter to test activity change interrupt - axis wise */
struct test_axes_wise_counter
{
    uint8_t x_counter;
    uint8_t y_counter;
    uint8_t z_counter;
};


void setup() {
  
  struct bma400_dev bma;

  int8_t rslt = 0;
  uint16_t int_status;

  struct test_axes_wise_counter act_ch_cnt = { 0 };
  struct bma400_sensor_data accel;
  struct bma400_sensor_conf accel_setting[2];;
  struct bma400_int_enable int_en;
  
  memset(accel_setting, 0, sizeof(struct bma400_sensor_conf));
  Serial.begin(115200);
  delay(1000);

  Serial.println("Functionality test for Activity change interrupt");

  rslt = bma400_interface_init(&bma, BMA400_I2C_INTF);
  bma400_check_rslt("bma400_interface_init", rslt);

  rslt = bma400_soft_reset(&bma);
  bma400_check_rslt("bma400_soft_reset", rslt);

  rslt = bma400_init(&bma);
  bma400_check_rslt("bma400_init", rslt);

  accel_setting[0].type = BMA400_ACTIVITY_CHANGE_INT;
  accel_setting[1].type = BMA400_ACCEL;

  rslt = bma400_get_sensor_conf(accel_setting, 2, &bma);
  bma400_check_rslt("bma400_get_sensor_conf", rslt);

  accel_setting[0].param.act_ch.int_chan = BMA400_INT_CHANNEL_1;
  accel_setting[0].param.act_ch.axes_sel = BMA400_AXIS_XYZ_EN;
  accel_setting[0].param.act_ch.act_ch_ntps = BMA400_ACT_CH_SAMPLE_CNT_64;
  accel_setting[0].param.act_ch.data_source = BMA400_DATA_SRC_ACC_FILT1;
  accel_setting[0].param.act_ch.act_ch_thres = 10;

  accel_setting[1].param.accel.odr = BMA400_ODR_100HZ;
  accel_setting[1].param.accel.range = BMA400_RANGE_2G;
  accel_setting[1].param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;

  /* Set the desired configurations to the sensor */
  rslt = bma400_set_sensor_conf(accel_setting, 2, &bma);
  bma400_check_rslt("bma400_set_sensor_conf", rslt);

  rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, &bma);
  bma400_check_rslt("bma400_set_power_mode", rslt);

  int_en.type = BMA400_ACTIVITY_CHANGE_INT_EN;
  int_en.conf = BMA400_ENABLE;

  rslt = bma400_enable_interrupt(&int_en, 1, &bma);
  bma400_check_rslt("bma400_enable_interrupt", rslt);

  Serial.println("\nShow activity on x y z axes of the board");
  
  while(1)
  {
    rslt = bma400_get_sensor_conf(accel_setting, 2, &bma);
    bma400_check_rslt("bma400_get_sensor_conf", rslt);

    rslt = bma400_get_interrupt_status(&int_status, &bma);
    bma400_check_rslt("bma400_get_interrupt_status", rslt);

    if (int_status & BMA400_ASSERTED_ACT_CH_X)
    {
        Serial.println("\nActivity change interrupt asserted on X axis");
        act_ch_cnt.x_counter++;

        rslt = bma400_get_accel_data(BMA400_DATA_SENSOR_TIME, &accel, &bma);
        bma400_check_rslt("bma400_get_accel_data_X", rslt);

        if (rslt == BMA400_OK)
        {
          Serial.print("Accel Data :  X : ");
          Serial.print(accel.x);
          Serial.print("    Y : ");
          Serial.print(accel.y);
          Serial.print("    Z : ");
          Serial.print(accel.z);
          Serial.print("    SENSOR_TIME : ");
          Serial.println(accel.sensortime);
        }
    }

    if (int_status & BMA400_ASSERTED_ACT_CH_Y)
    {
      Serial.println("\nActivity change interrupt asserted on Y axis");
      act_ch_cnt.y_counter++;

      rslt = bma400_get_accel_data(BMA400_DATA_SENSOR_TIME, &accel, &bma);
      bma400_check_rslt("bma400_get_accel_data_Y", rslt);

      if (rslt == BMA400_OK)
      {
        Serial.print("Accel Data :  X : ");
        Serial.print(accel.x);
        Serial.print("    Y : ");
        Serial.print(accel.y);
        Serial.print("    Z : ");
        Serial.print(accel.z);
        Serial.print("    SENSOR_TIME : ");
        Serial.println(accel.sensortime);
        delay(100);
      }
    }

    if (int_status & BMA400_ASSERTED_ACT_CH_Z)
    {
      Serial.println("\nActivity change interrupt asserted on Z axis");
      act_ch_cnt.z_counter++;

      rslt = bma400_get_accel_data(BMA400_DATA_SENSOR_TIME, &accel, &bma);
      bma400_check_rslt("bma400_get_accel_data_Z", rslt);

      if (rslt == BMA400_OK)
      {
        Serial.print("Accel Data :  X : ");
        Serial.print(accel.x);
        Serial.print("    Y : ");
        Serial.print(accel.y);
        Serial.print("    Z : ");
        Serial.print(accel.z);
        Serial.print("    SENSOR_TIME : ");
        Serial.println(accel.sensortime);
        delay(100);
      }
    }

    if ((act_ch_cnt.x_counter >= BMA400_INT_COUNTER) && (act_ch_cnt.y_counter >= BMA400_INT_COUNTER) &&
        (act_ch_cnt.z_counter >= BMA400_INT_COUNTER))
    {
        Serial.println("Activity change interrupt test done !");
        break;
    }
    delay(100);
  }
}

void loop() {
}

