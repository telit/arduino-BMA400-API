/*Copyright (C) 2021 Telit Communications S.p.A. Italy - All Rights Reserved.*/
/*    See LICENSE file in the root folder for full license information.     */

/*!
 * @ingroup bma400Examples
 * @defgroup bma400ExamplesSelfTest Self test
 * @brief Perform accelerometer self test
 * \include selftest.ino
 */

#include <telit_bma400.h>


void setup() {
  Serial.begin(115200);
  delay(1000);

  struct bma400_dev bma;
  int8_t rslt;

  rslt = bma400_interface_init(&bma, BMA400_I2C_INTF);
  bma400_check_rslt("bma400_interface_init", rslt);

  rslt = bma400_init(&bma);
  bma400_check_rslt("bma400_init", rslt);

  rslt = bma400_soft_reset(&bma);
  bma400_check_rslt("bma400_soft_reset", rslt);

  rslt = bma400_perform_self_test(&bma);
  bma400_check_rslt("bma400_perform_self_test", rslt);

  if (rslt == BMA400_OK)
  {
    Serial.println("\nSelf-test passed.");
  }

  delay(2000);
}

void loop() {
}
