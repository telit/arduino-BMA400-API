/*Copyright (C) 2021 Telit Communications S.p.A. Italy - All Rights Reserved.*/
/*    See LICENSE file in the root folder for full license information.     */

/*!
 * @ingroup bma400Examples
 * @defgroup bma400ExamplesFreeFall
 * @brief Perform free fall interrupt demo
 * based on How to generate freefall interrupt using BMA400.pdf
 * \include freefall.ino
 */

#include <telit_bma400.h>

const byte INTERRUPT = ACC_INT_1; //configure the Arduino GPIO pin connected to BMA400 INT1 pin
bool isr = false;

struct bma400_dev bma;

void setup() {
  Serial.begin(115200);

  delay(4000);


  Serial.println("Starting... freefall configuration on Sensor");


  struct bma400_sensor_conf sensor_settings[2];
  struct bma400_device_conf dev_settings[2];
  struct bma400_int_enable int_en[2];
  uint8_t data = 0;

  int8_t rslt;

  rslt = bma400_interface_init(&bma, BMA400_I2C_INTF);
  bma400_check_rslt("bma400_interface_init", rslt);

  rslt = bma400_init(&bma);
  bma400_check_rslt("bma400_init", rslt);

  rslt = bma400_soft_reset(&bma);
  bma400_check_rslt("bma400_soft_reset", rslt);

  rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, &bma);
  bma400_check_rslt("bma400_set_power_mode", rslt);


  rslt = bma400_get_sensor_conf(sensor_settings, 2, &bma);
  bma400_check_rslt("bma400_get_sensor_conf", rslt);


  sensor_settings[0].type = BMA400_ACCEL;

  sensor_settings[0].param.accel.odr = BMA400_ODR_200HZ;
  sensor_settings[0].param.accel.range = BMA400_RANGE_4G;
  sensor_settings[0].param.accel.osr = BMA400_ACCEL_OSR_SETTING_0;
  sensor_settings[0].param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;


  sensor_settings[1].type = BMA400_GEN1_INT;

  //set inactivity threshold to 0x3F = 63LSBs = 63LSBs * 8mg/LSB = 504mg (can be fine-tuned). So the threshold zone is    +/-504mg
  sensor_settings[1].param.gen_int.gen_int_thres = 0x3F;

  //set MSB of Gen1 to 0x00
  //set LSB of Gen1 interrupt duration to 0x0C = 12LSBs = 12 * 10ms =120ms (can be fine-tuned).
  //This corresponds to about 7cm height freefall. H=0.5 * g * t^2 = 0.5 * 9.81m/s^2 *(0.12s)^2
  sensor_settings[1].param.gen_int.gen_int_dur = 0x000C;

  //enable X/Y/Z axis for interrupt evaluation.
  sensor_settings[1].param.gen_int.axes_sel = BMA400_AXIS_XYZ_EN;
  //Gen1 interrupt engine data source is acc_filt2 which is fixed 100Hz ODR or 10ms   time interval.
  sensor_settings[1].param.gen_int.data_src = BMA400_DATA_SRC_ACCEL_FILT_2;

  //select inactivity detection which means the interrupt will be generated with in the positive and negative threshold zone
  sensor_settings[1].param.gen_int.criterion_sel = BMA400_INACTIVITY_INT;
  //Select AND logic meaning that when all enabled axes enter the threshold zone simultaneously an interrupt will be generated
  sensor_settings[1].param.gen_int.evaluate_axes = BMA400_ALL_AXES_INT;

  //Manual update and hysteresis 0mg
  sensor_settings[1].param.gen_int.ref_update = BMA400_UPDATE_MANUAL;
  sensor_settings[1].param.gen_int.hysteresis = BMA400_HYST_0_MG;

  sensor_settings[1].param.gen_int.int_thres_ref_x = 0; //  set X axis reference to 0mg
  sensor_settings[1].param.gen_int.int_thres_ref_y = 0; //  set Y axis reference to 0mg
  sensor_settings[1].param.gen_int.int_thres_ref_z = 0; //  set Z axis reference to 0mg

  sensor_settings[1].param.gen_int.int_chan = BMA400_INT_CHANNEL_1;

  /* Set the desired configurations to the sensor */
  rslt = bma400_set_sensor_conf(sensor_settings, 2, &bma);
  bma400_check_rslt("bma400_set_sensor_conf", rslt);

  int_en[0].type = BMA400_GEN1_INT_EN;
  int_en[0].conf = BMA400_ENABLE;


  rslt = bma400_enable_interrupt(int_en, 1, &bma);
  bma400_check_rslt("bma400_enable_interrupt", rslt);


  dev_settings[0].type = BMA400_INT_PIN_CONF;
  dev_settings[0].param.int_conf.int_chan = BMA400_MAP_BOTH_INT_PINS;
  dev_settings[0].param.int_conf.pin_conf = BMA400_INT_PUSH_PULL_ACTIVE_1;
  /* Set the desired configurations to the sensor */
  rslt = bma400_set_device_conf(dev_settings, 1, &bma);
  bma400_check_rslt("bma400_set_device_conf", rslt);


  pinMode (INTERRUPT, INPUT);
  digitalWrite (INTERRUPT, LOW);  // internal pull-down resistor
  attachInterrupt(digitalPinToInterrupt(INTERRUPT), freefall, RISING);
}

void freefall() {
  Serial.print("\n*****************\nFreefall Occurred\n*****************\n");
  isr = true;
}

void loop() {

  int8_t rslt;
  uint16_t int_status;
  if (isr == true)
  {
    isr = false;

    bma400_get_interrupt_status( &int_status, &bma);
    bma400_check_rslt("bma400_get_interrupt_status", rslt);

    Serial.print("int status: ");
    Serial.println(int_status);

  }
  else {
    Serial.println("Freefall did not occur");
  }
  delay(2000);
}
