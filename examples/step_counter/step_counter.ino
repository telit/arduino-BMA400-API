/*Copyright (C) 2021 Telit Communications S.p.A. Italy - All Rights Reserved.*/
/*    See LICENSE file in the root folder for full license information.     */

/*!
 * @ingroup bma400Examples
 * @defgroup bma400ExamplesStepCounter Step counter
 * @brief To showcase step counter feature
 * \include step_counter.ino
 */

#include <telit_bma400.h>


void setup() {

  int8_t rslt = 0;

  struct bma400_dev bma;
  struct bma400_sensor_conf accel_setting[2];
  struct bma400_int_enable int_en[2];

  uint16_t int_status;

  uint32_t step_count = 0;
  uint8_t act_int;

  uint32_t count = 10000;
    
  memset(accel_setting, 0, sizeof(struct bma400_sensor_conf));
  
  Serial.begin(115200);
  delay(1000);


  rslt = bma400_interface_init(&bma, BMA400_I2C_INTF);
  bma400_check_rslt("bma400_interface_init", rslt);

  rslt = bma400_soft_reset(&bma);
  bma400_check_rslt("bma400_soft_reset", rslt);

  rslt = bma400_init(&bma);
  bma400_check_rslt("bma400_init", rslt);

  accel_setting[0].type = BMA400_STEP_COUNTER_INT;
  accel_setting[1].type = BMA400_ACCEL;

  rslt = bma400_get_sensor_conf(accel_setting, 2, &bma);
  bma400_check_rslt("bma400_get_sensor_conf", rslt);

  accel_setting[0].param.step_cnt.int_chan = BMA400_INT_CHANNEL_1;

  accel_setting[1].param.accel.odr = BMA400_ODR_100HZ;
  accel_setting[1].param.accel.range = BMA400_RANGE_2G;
  accel_setting[1].param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;

  /* Set the desired configurations to the sensor */
  rslt = bma400_set_sensor_conf(accel_setting, 2, &bma);
  bma400_check_rslt("bma400_set_sensor_conf", rslt);

  rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, &bma);
  bma400_check_rslt("bma400_set_power_mode", rslt);

  int_en[0].type = BMA400_STEP_COUNTER_INT_EN;
  int_en[0].conf = BMA400_ENABLE;

  int_en[1].type = BMA400_LATCH_INT_EN;
  int_en[1].conf = BMA400_ENABLE;

  rslt = bma400_enable_interrupt(int_en, 2, &bma);
  bma400_check_rslt("bma400_enable_interrupt", rslt);

  Serial.println("Shake the board in terms of steps");


  while (1)
  {
    rslt = bma400_get_interrupt_status(&int_status, &bma);
    bma400_check_rslt("bma400_get_interrupt_status", rslt);

    if (int_status & BMA400_ASSERTED_STEP_INT)
    {
      rslt = bma400_get_steps_counted(&step_count, &act_int, &bma);
      bma400_check_rslt("bma400_get_steps_counted", rslt);

      switch (act_int)
      {
        case BMA400_STILL_ACT:
            Serial.println("Step counter interrupt received");
            Serial.print("Steps counted : ");
            Serial.println(step_count);
            
            Serial.println("Still Activity detected");
            count--;
            break;
        case BMA400_WALK_ACT:
            Serial.println("Step counter interrupt received");
            Serial.print("Steps counted : ");
            Serial.println(step_count);
            Serial.println("Walking Activity detected");
            count--;
            break;
        case BMA400_RUN_ACT:
            Serial.println("Step counter interrupt received");
            Serial.print("Steps counted : ");
            Serial.println(step_count);
            Serial.println("Running Activity detected");
            count--;
            break;
      }
      if (count == 0)
      {
        Serial.println("break loop");
        break;
      }
    }
    delay(10);
  }

}

void loop() {
}

