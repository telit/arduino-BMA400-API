/*Copyright (C) 2021 Telit Communications S.p.A. Italy - All Rights Reserved.*/
/*    See LICENSE file in the root folder for full license information.     */

/*!
 * @ingroup bma400Examples
 * @defgroup bma400ExamplesAccelerometer Accelerometer read
 * @brief To read accelerometer xyz data for defined range and ODR
 * \include accelerometer.ino
 */

#include <telit_bma400.h>

/* Earth's gravity in m/s^2 */
#define GRAVITY_EARTH     (9.80665f)

/* 39.0625us per tick */
#define SENSOR_TICK_TO_S  (0.0000390625f)

static float lsb_to_ms2(int16_t accel_data, uint8_t g_range, uint8_t bit_width);

struct bma400_dev bma;
struct bma400_sensor_conf conf;
struct bma400_sensor_data data;
struct bma400_int_enable int_en;
int8_t rslt;

float t, x, y, z;
uint16_t int_status = 0;


static float lsb_to_ms2(int16_t accel_data, uint8_t g_range, uint8_t bit_width)
{
    float accel_ms2;
    int16_t half_scale;

    half_scale = 1 << (bit_width - 1);
    accel_ms2 = (GRAVITY_EARTH * accel_data * g_range) / half_scale;

    return accel_ms2;

}



void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting...");

  /* Interface reference is given as a parameter
   *         For I2C : BMA400_I2C_INTF
   */
  rslt = bma400_interface_init(&bma, BMA400_I2C_INTF);
  bma400_check_rslt("bma400_interface_init", rslt);

  rslt = bma400_soft_reset(&bma);
  bma400_check_rslt("bma400_soft_reset", rslt);

  rslt = bma400_init(&bma);
  bma400_check_rslt("bma400_init", rslt);

  /* Select the type of configuration to be modified */
  conf.type = BMA400_ACCEL;

  /* Get the accelerometer configurations which are set in the sensor */
  rslt = bma400_get_sensor_conf(&conf, 1, &bma);
  bma400_check_rslt("bma400_get_sensor_conf", rslt);

  /* Modify the desired configurations as per macros
   * available in bma400_defs.h file */
  conf.param.accel.odr = BMA400_ODR_100HZ;
  conf.param.accel.range = BMA400_RANGE_2G;
  conf.param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;

  /* Set the desired configurations to the sensor */
  rslt = bma400_set_sensor_conf(&conf, 1, &bma);
  bma400_check_rslt("bma400_set_sensor_conf", rslt);

  rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, &bma);
  bma400_check_rslt("bma400_set_power_mode", rslt);

  int_en.type = BMA400_DRDY_INT_EN;
  int_en.conf = BMA400_ENABLE;

  rslt = bma400_enable_interrupt(&int_en, 1, &bma);
  bma400_check_rslt("bma400_enable_interrupt", rslt);

}

void loop() {
  uint8_t n_samples = 10;
  Serial.print("\nGet accel data - BMA400_DATA_SENSOR_TIME\n");

  Serial.print("Accel Gravity data in m/s^2\n");

  while (n_samples && (rslt == BMA400_OK))
  {
    rslt = bma400_get_interrupt_status(&int_status, &bma);
    bma400_check_rslt("bma400_get_interrupt_status", rslt);

    if (int_status & BMA400_ASSERTED_DRDY_INT)
    {
      rslt = bma400_get_accel_data(BMA400_DATA_SENSOR_TIME, &data, &bma);
      bma400_check_rslt("bma400_get_accel_data", rslt);

      /* 12-bit accelerometer at range 2G */
      x = lsb_to_ms2(data.x, 2, 12);
      y = lsb_to_ms2(data.y, 2, 12);
      z = lsb_to_ms2(data.z, 2, 12);
      t = (float)data.sensortime * SENSOR_TICK_TO_S;

      Serial.print("Gravity-x : ");
      Serial.print(x);
      Serial.print(", Gravity-y : ");
      Serial.print(y);
      Serial.print(", Gravity-z : ");
      Serial.print(z);
      Serial.print(", t(s) : ");
      Serial.println(t);
      n_samples--;
      delay(1000);
    }

  }

  n_samples = 10;

  Serial.print("\nGet accel data - BMA400_DATA_ONLY\n");

  Serial.print("Accel Gravity data in m/s^2\n");

  while (n_samples && (rslt == BMA400_OK))
  {
    rslt = bma400_get_interrupt_status(&int_status, &bma);
    bma400_check_rslt("bma400_get_interrupt_status", rslt);

    if (int_status & BMA400_ASSERTED_DRDY_INT)
    {
        rslt = bma400_get_accel_data(BMA400_DATA_ONLY, &data, &bma);
        bma400_check_rslt("bma400_get_accel_data", rslt);

        /* 12-bit accelerometer at range 2G */
        x = lsb_to_ms2(data.x, 2, 12);
        y = lsb_to_ms2(data.y, 2, 12);
        z = lsb_to_ms2(data.z, 2, 12);

        Serial.print("Gravity-x : ");
        Serial.print(x);
        Serial.print(", Gravity-y : ");
        Serial.print(y);
        Serial.print(", Gravity-z : ");
        Serial.println(z);
        n_samples--;
        delay(1000);
    }
  }
  delay(2000);
}

