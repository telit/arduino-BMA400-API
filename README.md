# Arduino Library wrapper of BMA400 Sensor API

> Bosch Sensortec's BMA400 accelerometer sensor API

This is a wrapper Arduino library of [original Bosch Sensortec C library for BMA400 sensor](https://github.com/BoschSensortec/BMA400-API). It provides a common I2C interface to communicate with the sensor. **SPI interface is not available**.


For more information on Bosch Sensortec source code and material, please refer to:

- [BMA400 product page](https://www.bosch-sensortec.com/bst/products/all_products/bma400_1)
- [BMA400 github page](https://github.com/BoschSensortec/BMA400-API)
- [BMA400 accelerometer design guide](https://community.bosch-sensortec.com/t5/Knowledge-base/BMA400-accelerometer-design-guide/ta-p/7397)
- [Knowledge base page](https://community.bosch-sensortec.com/t5/Knowledge-base/tkb-p/bst_community-mems-tkb)
- [Community support page](https://community.bosch-sensortec.com)

------------

## Arduino Library usage


To use the library in a sketch, include the `telit_bma400.h` header file

```C
#include <telit_bma400.h>
```

Then initialize the interface, the sensor and the library

```C
  struct bma400_dev bma;
  int8_t rslt;
  
  rslt = bma400_interface_init(&bma, BMA400_I2C_INTF);
  bma400_check_rslt("bma400_interface_init", rslt);
  
  rslt = bma400_soft_reset(&bma);
  bma400_check_rslt("bma400_soft_reset", rslt);

  rslt = bma400_init(&bma);
  bma400_check_rslt("bma400_init", rslt);
  
```

The sensor can then be configured with the required options depending on the application scenario, for example:

```C
  struct bma400_sensor_conf conf;
  conf.type = BMA400_ACCEL;
    
  /* Get the accelerometer configurations which are set in the sensor */
  rslt = bma400_get_sensor_conf(&conf, 1, &bma);
  
  /* Modify the desired configurations as per macros
  * available in bma400_defs.h file */
  conf.param.accel.odr = BMA400_ODR_100HZ;
  conf.param.accel.range = BMA400_RANGE_2G;
  conf.param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;

  /* Set the desired configurations to the sensor */
  rslt = bma400_set_sensor_conf(&conf, 1, &bma);
  bma400_check_rslt("bma400_set_sensor_conf", rslt);
```

The sensor operating mode can then be set with

```C
  rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, &bma);
  bma400_check_rslt("bma400_set_power_mode", rslt);
```


Please refer to the provided [examples](examples) for additional details.

