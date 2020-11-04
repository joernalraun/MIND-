/*
 * BMI160.c
 *
 *  Created on: Nov 3, 2020
 *      Author: Administrator
 */
#include "BMI160.h"
uint8_t I2C_addr=0;

uint8_t onlyAccel=1;
uint8_t onlyGyro=2;
uint8_t bothAccelGyro=3;
uint8_t stepNormalPowerMode=0;
uint8_t stepLowPowerMode=1;

const uint8_t int_mask_lookup_table[13] = {
	BMI160_INT1_SLOPE_MASK,
	BMI160_INT1_SLOPE_MASK,
	BMI160_INT2_LOW_STEP_DETECT_MASK,
	BMI160_INT1_DOUBLE_TAP_MASK,
	BMI160_INT1_SINGLE_TAP_MASK,
	BMI160_INT1_ORIENT_MASK,
	BMI160_INT1_FLAT_MASK,
	BMI160_INT1_HIGH_G_MASK,
	BMI160_INT1_LOW_G_MASK,
	BMI160_INT1_NO_MOTION_MASK,
	BMI160_INT2_DATA_READY_MASK,
	BMI160_INT2_FIFO_FULL_MASK,
	BMI160_INT2_FIFO_WM_MASK
};

//上层应用
void begin(TYPE v, uint8_t addr){
    Obmi160=(struct bmi160Dev *)malloc(sizeof(struct bmi160Dev));
    Oaccel= (struct bmi160SensorData*)malloc(sizeof(struct bmi160SensorData));
    Ogyro = (struct bmi160SensorData*)malloc(sizeof(struct bmi160SensorData));
	Obmi160->id = addr;
	Obmi160->interface = BMI160_I2C_INTF;
	I2cInit(Obmi160);
	if(v == eStep){
        setInt(2);
        setStepCounter(1,Obmi160);
        setStepPowerMode(stepNormalPowerMode,Obmi160);
	}
}

int8_t I2cInit(struct bmi160Dev *dev){
  int8_t rslt=BMI160_OK;
  uint8_t chip_id=0;
  uint8_t data=0;
  if (dev==NULL){
    return BMI160_E_NULL_PTR;
  }
  if (dev->interface == BMI160_SPI_INTF){
    rslt = getRegs(BMI160_SPI_COMM_TEST_ADDR, &data, 1, dev);
  }
  if (rslt == BMI160_OK){
    rslt = getRegs(BMI160_CHIP_ID_ADDR, &chip_id, 1, dev);
    if ((rslt == BMI160_OK)&&(chip_id==BMI160_CHIP_ID)){
      dev->any_sig_sel = eBmi160BothAnySigMotionDisabled;
      dev->chipId = chip_id;
      rslt = softReset(dev);
      if (rslt==BMI160_OK){
        rslt = setSensConf(dev);
      }
    }else{
      rslt = BMI160_E_DEV_NOT_FOUND;
    }
  }
  return rslt;
}

int8_t setSensConf(struct bmi160Dev *dev){
  int8_t rslt = BMI160_OK;
  dev->accelCfg.odr = BMI160_ACCEL_ODR_1600HZ;
  dev->accelCfg.range = BMI160_ACCEL_RANGE_2G;
  dev->accelCfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

  dev->accelCfg.power = BMI160_ACCEL_NORMAL_MODE;

  dev->gyroCfg.odr = BMI160_GYRO_ODR_3200HZ;
  dev->gyroCfg.range = BMI160_GYRO_RANGE_2000_DPS;
  dev->gyroCfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

  dev->gyroCfg.power = BMI160_GYRO_NORMAL_MODE;


  rslt = setAccelConf(dev);
  if (rslt == BMI160_OK) {
    rslt = setGyroConf(dev);
    if (rslt == BMI160_OK) {
      /* write power mode for accel and gyro */
      rslt = setPowerMode(dev);
      if (rslt == BMI160_OK)
        rslt = checkInvalidSettg(dev);
    }
  }

  return rslt;
}

int8_t softReset(struct bmi160Dev *dev){
  int8_t rslt=BMI160_OK;
  uint8_t data = BMI160_SOFT_RESET_CMD;
  if (dev==NULL){
    rslt = BMI160_E_NULL_PTR;
  }
  rslt = setRegs(BMI160_COMMAND_REG_ADDR, &data, 1, dev);
  //delay(BMI160_SOFT_RESET_DELAY_MS);
  HAL_Delay(BMI160_SOFT_RESET_DELAY_MS);
  if (rslt == BMI160_OK){
    defaultParamSettg(dev);
  }
  return rslt;
}

void defaultParamSettg(struct bmi160Dev *dev){
  /* Initializing accel and gyro params with
  * default values */
  dev->accelCfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
  dev->accelCfg.odr = BMI160_ACCEL_ODR_100HZ;
  dev->accelCfg.power = BMI160_ACCEL_SUSPEND_MODE;
  dev->accelCfg.range = BMI160_ACCEL_RANGE_2G;
  dev->gyroCfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
  dev->gyroCfg.odr = BMI160_GYRO_ODR_100HZ;
  dev->gyroCfg.power = BMI160_GYRO_SUSPEND_MODE;
  dev->gyroCfg.range = BMI160_GYRO_RANGE_2000_DPS;

  /* To maintain the previous state of accel configuration */
  dev->prevAccelCfg = dev->accelCfg;
  /* To maintain the previous state of gyro configuration */
  dev->prevGyroCfg = dev->gyroCfg;
}

int8_t setInt(int intNum){
  int8_t rslt=BMI160_OK;
  struct bmi160IntSettg intConfig;
  if (intNum == 1){
    intConfig.intChannel = BMI160_INT_CHANNEL_1;
  }else if(intNum == 2){
    intConfig.intChannel = BMI160_INT_CHANNEL_2;
  }else{
    return BMI160_E_NULL_PTR;
  }
  /* Select the Interrupt type */
  intConfig.intType = BMI160_STEP_DETECT_INT;// Choosing Step Detector interrupt
  /* Select the interrupt channel/pin settings */
  intConfig.intPinSettg.outputEn = BMI160_ENABLE;// Enabling interrupt pins to act as output pin
  intConfig.intPinSettg.outputMode = BMI160_DISABLE;// Choosing push-pull mode for interrupt pin
  intConfig.intPinSettg.outputType = BMI160_ENABLE;// Choosing active High output
  intConfig.intPinSettg.edgeCtrl = BMI160_ENABLE;// Choosing edge triggered output
  intConfig.intPinSettg.inputEn = BMI160_DISABLE;// Disabling interrupt pin to act as input
  intConfig.intPinSettg.latchDur =BMI160_LATCH_DUR_NONE;// non-latched output

  /* Select the Step Detector interrupt parameters, Kindly use the recommended settings for step detector */
  intConfig.intTypeCfg.accStepDetectInt.stepDetectorMode = BMI160_STEP_DETECT_NORMAL;
  intConfig.intTypeCfg.accStepDetectInt.stepDetectorEn = BMI160_ENABLE;// 1-enable, 0-disable the step detector
  rslt =setIntConfig(&intConfig, Obmi160);
  return rslt;
}

int8_t setIntConfig(struct bmi160IntSettg *intConfig, struct bmi160Dev *dev)
{
  int8_t rslt = BMI160_OK;
  switch (intConfig->intType) {
  case BMI160_ACC_ANY_MOTION_INT:
    /*Any-motion  interrupt*/
    //rslt = set_accel_any_motion_int(intConfig, dev);
    break;
  case BMI160_ACC_SIG_MOTION_INT:
    /* Significant motion interrupt */
    //rslt = set_accel_sig_motion_int(intConfig, dev);
    break;
  case BMI160_ACC_SLOW_NO_MOTION_INT:
    /* Slow or no motion interrupt */
    //rslt = set_accel_no_motion_int(intConfig, dev);
    break;
  case BMI160_ACC_DOUBLE_TAP_INT:
  case BMI160_ACC_SINGLE_TAP_INT:
     /* Double tap and single tap Interrupt */
    //rslt = set_accel_tap_int(intConfig, dev);
    break;
  case BMI160_STEP_DETECT_INT:
    /* Step detector interrupt */
    rslt = setAccelStepDetectInt(intConfig, dev);
    break;
  case BMI160_ACC_ORIENT_INT:
    /* Orientation interrupt */
    //rslt = set_accel_orientation_int(intConfig, dev);
    break;
  case BMI160_ACC_FLAT_INT:
    /* Flat detection interrupt */
    //rslt = set_accel_flat_detect_int(intConfig, dev);
    break;
  case BMI160_ACC_LOW_G_INT:
     /* Low-g interrupt */
    //rslt = set_accel_low_g_int(intConfig, dev);
    break;
  case BMI160_ACC_HIGH_G_INT:
    /* High-g interrupt */
    //rslt = set_accel_high_g_int(intConfig, dev);
    break;
  case BMI160_ACC_GYRO_DATA_RDY_INT:
    /* Data ready interrupt */
    //rslt = set_accel_gyro_data_ready_int(intConfig, dev);
    break;
  case BMI160_ACC_GYRO_FIFO_FULL_INT:
     /* Fifo full interrupt */
    //rslt = set_fifo_full_int(intConfig, dev);
    break;
  case BMI160_ACC_GYRO_FIFO_WATERMARK_INT:
     /* Fifo water-mark interrupt */
    //rslt = set_fifo_watermark_int(intConfig, dev);
    break;
  default:
    break;
  }
  return rslt;
}

int8_t setAccelStepDetectInt(struct bmi160IntSettg *intConfig, struct bmi160Dev *dev)
{
  int8_t rslt = BMI160_OK;
  /* Null-pointer check */
  if (dev == NULL){
    rslt = BMI160_E_NULL_PTR;
  }
  if ((rslt != BMI160_OK) || (intConfig == NULL)) {
    rslt = BMI160_E_NULL_PTR;
  } else {
    /* updating the interrupt structure to local structure */
    struct bmi160AccStepDetectIntCfg *stepDetectIntCfg =
                &(intConfig->intTypeCfg.accStepDetectInt);
    rslt = enableStepDetectInt(stepDetectIntCfg, dev);
    if (rslt == BMI160_OK) {
      /* Configure Interrupt pins */
      rslt = setIntrPinConfig(intConfig, dev);
      if (rslt == BMI160_OK) {
        rslt = mapFeatureInterrupt(intConfig, dev);
        if (rslt == BMI160_OK){
          rslt = configStepDetect(stepDetectIntCfg, dev);
        }
      }
    }
  }
  return rslt;
}

int8_t mapFeatureInterrupt(struct bmi160IntSettg *intConfig, struct bmi160Dev *dev){
  int8_t rslt;
  uint8_t data[3] = {0, 0, 0};
  uint8_t temp[3] = {0, 0, 0};
  rslt = getRegs(BMI160_INT_MAP_0_ADDR, data, 3, dev);
  if (rslt == BMI160_OK) {
    temp[0] = data[0] & ~int_mask_lookup_table[intConfig->intType];
    temp[2] = data[2] & ~int_mask_lookup_table[intConfig->intType];
    switch (intConfig->intChannel) {
    case BMI160_INT_CHANNEL_NONE:
      data[0] = temp[0];
      data[2] = temp[2];
      break;
    case BMI160_INT_CHANNEL_1:
      data[0] = temp[0] | int_mask_lookup_table[intConfig->intType];
      data[2] = temp[2];
      break;
    case BMI160_INT_CHANNEL_2:
      data[2] = temp[2] | int_mask_lookup_table[intConfig->intType];
      data[0] = temp[0];
      break;
    case BMI160_INT_CHANNEL_BOTH:
      data[0] = temp[0] | int_mask_lookup_table[intConfig->intType];
      data[2] = temp[2] | int_mask_lookup_table[intConfig->intType];
      break;
    default:
      rslt = BMI160_E_OUT_OF_RANGE;
    }
    if (rslt == BMI160_OK)
      rslt = setRegs(BMI160_INT_MAP_0_ADDR, data, 3, dev);
  }
  return rslt;
}

int8_t configStepDetect(struct bmi160AccStepDetectIntCfg *stepDetectIntCfg, struct bmi160Dev *dev){
  int8_t rslt;
  uint8_t temp = 0;
  uint8_t data_array[2] = {0};


  if (stepDetectIntCfg->stepDetectorMode == BMI160_STEP_DETECT_NORMAL) {
    /* Normal mode setting */
    data_array[0] = 0x15;
    data_array[1] = 0x03;
  } else if (stepDetectIntCfg->stepDetectorMode == BMI160_STEP_DETECT_SENSITIVE) {
    /* Sensitive mode setting */
    data_array[0] = 0x2D;
    data_array[1] = 0x00;
  } else if (stepDetectIntCfg->stepDetectorMode == BMI160_STEP_DETECT_ROBUST) {
    /* Robust mode setting */
    data_array[0] = 0x1D;
    data_array[1] = 0x07;
  } else if (stepDetectIntCfg->stepDetectorMode == BMI160_STEP_DETECT_USER_DEFINE) {
    /* Non recommended User defined setting */
    /* Configuring STEP_CONFIG register */
    rslt = getRegs(BMI160_INT_STEP_CONFIG_0_ADDR, &data_array[0], 2, dev);

    if (rslt == BMI160_OK) {
      temp = data_array[0] & ~BMI160_STEP_DETECT_MIN_THRES_MASK;
      /* Adding minThreshold */
      data_array[0] = temp | ((stepDetectIntCfg->minThreshold << 3)
          & BMI160_STEP_DETECT_MIN_THRES_MASK);

      temp = data_array[0] & ~BMI160_STEP_DETECT_STEPTIME_MIN_MASK;
      /* Adding steptimeMin */
      data_array[0] = temp | ((stepDetectIntCfg->steptimeMin)
          & BMI160_STEP_DETECT_STEPTIME_MIN_MASK);

      temp = data_array[1] & ~BMI160_STEP_MIN_BUF_MASK;
      /* Adding steptimeMin */
      data_array[1] = temp | ((stepDetectIntCfg->stepMinBuf) & BMI160_STEP_MIN_BUF_MASK);

    }
  }

  /* Write data to STEP_CONFIG register */
  rslt = setRegs(BMI160_INT_STEP_CONFIG_0_ADDR, data_array, 2, dev);

  return rslt;
}

int8_t setIntrPinConfig(struct bmi160IntSettg *intConfig, struct bmi160Dev *dev){
  int8_t rslt;
  /* configure the behavioural settings of interrupt pin */
  rslt = configIntOutCtrl(intConfig, dev);
  if (rslt == BMI160_OK)
    rslt = configIntLatch(intConfig, dev);
  return rslt;
}

int8_t configIntLatch(struct bmi160IntSettg *intConfig, struct bmi160Dev *dev){
  int8_t rslt;
  uint8_t temp = 0;
  uint8_t data = 0;
  /* Configuration of latch on pins INT1 and INT2 are done in
   * BMI160_INT_LATCH_ADDR register*/
  rslt = getRegs(BMI160_INT_LATCH_ADDR, &data, 1, dev);
  if (rslt == BMI160_OK) {
    /* updating the interrupt pin structure to local structure */
    const struct bmi160IntPinSettg *intr_pin_sett = &(intConfig->intPinSettg);
    if (intConfig->intChannel == BMI160_INT_CHANNEL_1) {
      /* Configuring channel 1 */
      /* Input enable */
      temp = data & ~BMI160_INT1_INPUT_EN_MASK;
      data = temp | ((intr_pin_sett->inputEn << 4) & BMI160_INT1_INPUT_EN_MASK);
    } else {
      /* Configuring channel 2 */
      /* Input enable */
      temp = data & ~BMI160_INT2_INPUT_EN_MASK;
      data = temp | ((intr_pin_sett->inputEn << 5) & BMI160_INT2_INPUT_EN_MASK);
    }
    /* In case of latch interrupt,update the latch duration */
    /* Latching holds the interrupt for the amount of latch
     * duration time */
    temp = data & ~BMI160_INT_LATCH_MASK;
    data = temp | (intr_pin_sett->latchDur & BMI160_INT_LATCH_MASK);
    /* OUT_CTRL_INT and LATCH_INT address lie consecutively,
     * hence writing data to respective registers at one go */
    rslt = setRegs(BMI160_INT_LATCH_ADDR, &data, 1, dev);
  }
  return rslt;
}

int8_t configIntOutCtrl(struct bmi160IntSettg *intConfig, struct bmi160Dev *dev){
  int8_t rslt;
  uint8_t temp = 0;
  uint8_t data = 0;

  /* Configuration of output interrupt signals on pins INT1 and INT2 are
   * done in BMI160_INT_OUT_CTRL_ADDR register*/
  rslt = getRegs(BMI160_INT_OUT_CTRL_ADDR, &data, 1, dev);

  if (rslt == BMI160_OK) {
    /* updating the interrupt pin structure to local structure */
    const struct bmi160IntPinSettg *intr_pin_sett = &(intConfig->intPinSettg);

    /* Configuring channel 1 */
    if (intConfig->intChannel == BMI160_INT_CHANNEL_1) {

      /* Output enable */
      temp = data & ~BMI160_INT1_OUTPUT_EN_MASK;
      data = temp | ((intr_pin_sett->outputEn << 3) & BMI160_INT1_OUTPUT_EN_MASK);

      /* Output mode */
      temp = data & ~BMI160_INT1_OUTPUT_MODE_MASK;
      data = temp | ((intr_pin_sett->outputMode << 2) & BMI160_INT1_OUTPUT_MODE_MASK);

      /* Output type */
      temp = data & ~BMI160_INT1_OUTPUT_TYPE_MASK;
      data = temp | ((intr_pin_sett->outputType << 1) & BMI160_INT1_OUTPUT_TYPE_MASK);

      /* edge control */
      temp = data & ~BMI160_INT1_EDGE_CTRL_MASK;
      data = temp | ((intr_pin_sett->edgeCtrl) & BMI160_INT1_EDGE_CTRL_MASK);

    } else {
      /* Configuring channel 2 */
      /* Output enable */
      temp = data & ~BMI160_INT2_OUTPUT_EN_MASK;
      data = temp | ((intr_pin_sett->outputEn << 7) & BMI160_INT2_OUTPUT_EN_MASK);

      /* Output mode */
      temp = data & ~BMI160_INT2_OUTPUT_MODE_MASK;
      data = temp | ((intr_pin_sett->outputMode << 6) & BMI160_INT2_OUTPUT_MODE_MASK);

      /* Output type */
      temp = data & ~BMI160_INT2_OUTPUT_TYPE_MASK;
      data = temp | ((intr_pin_sett->outputType << 5) & BMI160_INT2_OUTPUT_TYPE_MASK);

      /* edge control */
      temp = data & ~BMI160_INT2_EDGE_CTRL_MASK;
      data = temp | ((intr_pin_sett->edgeCtrl << 4) & BMI160_INT2_EDGE_CTRL_MASK);
    }

    rslt = setRegs(BMI160_INT_OUT_CTRL_ADDR, &data, 1, dev);
  }

  return rslt;
}

int8_t enableStepDetectInt(struct bmi160AccStepDetectIntCfg *stepDetectIntCfg, struct bmi160Dev *dev){
  int8_t rslt;
  uint8_t data = 0;
  uint8_t temp = 0;
  /* Enable data ready interrupt in Int Enable 2 register */
  rslt = getRegs(BMI160_INT_ENABLE_2_ADDR, &data, 1, dev);
  if (rslt == BMI160_OK) {
    temp = data & ~BMI160_STEP_DETECT_INT_EN_MASK;
    data = temp | ((stepDetectIntCfg->stepDetectorEn << 3) & BMI160_STEP_DETECT_INT_EN_MASK);
    /* Writing data to INT ENABLE 2 Address */
    rslt = setRegs(BMI160_INT_ENABLE_2_ADDR, &data, 1, dev);
  }
  return rslt;
}

int8_t setStepCounter(uint8_t step_cnt_enable, struct bmi160Dev *dev)
{
  int8_t rslt = BMI160_OK;
  uint8_t data = 0;
  /* Null-pointer check */
  if (dev == NULL){
    rslt = BMI160_E_NULL_PTR;
  }
  if (rslt != BMI160_OK) {
    rslt = BMI160_E_NULL_PTR;
  } else {
    rslt = getRegs(BMI160_INT_STEP_CONFIG_1_ADDR, &data, 1, dev);
    if (rslt == BMI160_OK) {
      if (step_cnt_enable == BMI160_ENABLE){
        data |= (uint8_t)(step_cnt_enable << 3);
      }else{
        data &= ~BMI160_STEP_COUNT_EN_BIT_MASK;
      }
      rslt = setRegs(BMI160_INT_STEP_CONFIG_1_ADDR, &data, 1, dev);
    }
  }
  return rslt;
}

int8_t setStepPowerMode(uint8_t model,struct bmi160Dev *dev)
{
  int8_t rslt = BMI160_OK;
  if (model == stepNormalPowerMode){
    dev->accelCfg.odr=BMI160_ACCEL_ODR_1600HZ;
    dev->accelCfg.power = BMI160_ACCEL_NORMAL_MODE;
    dev->gyroCfg.odr = BMI160_GYRO_ODR_3200HZ;
    dev->gyroCfg.power = BMI160_GYRO_NORMAL_MODE;
  }else if(model == stepLowPowerMode){
    dev->accelCfg.odr=BMI160_ACCEL_ODR_50HZ;
    dev->accelCfg.power = BMI160_ACCEL_LOWPOWER_MODE;
    dev->gyroCfg.odr = BMI160_GYRO_ODR_50HZ;
    dev->gyroCfg.power = BMI160_GYRO_SUSPEND_MODE;
  }else{
    dev->accelCfg.odr=BMI160_ACCEL_ODR_1600HZ;
    dev->accelCfg.power = BMI160_ACCEL_NORMAL_MODE;
    dev->gyroCfg.odr = BMI160_GYRO_ODR_3200HZ;
    dev->gyroCfg.power = BMI160_GYRO_NORMAL_MODE;
  }
  dev->accelCfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
  dev->accelCfg.range = BMI160_ACCEL_RANGE_2G;
  dev->gyroCfg.range = BMI160_GYRO_RANGE_2000_DPS;
  dev->gyroCfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
  rslt = setAccelConf(dev);
  if (rslt == BMI160_OK) {
    rslt = setGyroConf(dev);
    if (rslt == BMI160_OK) {
      /* write power mode for accel and gyro */
      rslt = setPowerMode(dev);
      if (rslt == BMI160_OK)
        rslt = checkInvalidSettg(dev);
    }
  }

  return rslt;
}

int8_t setGyroConf(struct bmi160Dev *dev){
  int8_t rslt;
  uint8_t data[2]={0};
  rslt = checkGyroConfig(data, dev);
  if (rslt == BMI160_OK) {
    // Write output data rate and bandwidth
    rslt = setRegs(BMI160_GYRO_CONFIG_ADDR, &data[0], 1, dev);
    if (rslt == BMI160_OK) {
      dev->prevGyroCfg.odr = dev->gyroCfg.odr;
      dev->prevGyroCfg.bw = dev->gyroCfg.bw;
      //delay(BMI160_ONE_MS_DELAY);
      HAL_Delay(BMI160_ONE_MS_DELAY);
      // Write gyro range
      rslt = setRegs(BMI160_GYRO_RANGE_ADDR, &data[1], 1, dev);
      if (rslt == BMI160_OK)
        dev->prevGyroCfg.range = dev->gyroCfg.range;
    }
  }
  return rslt;
}

int8_t checkGyroConfig(uint8_t *data, struct bmi160Dev *dev){
  int8_t rslt;
  /* read gyro Output data rate and bandwidth */
  rslt = getRegs(BMI160_GYRO_CONFIG_ADDR, data, 2, dev);
  if (rslt == BMI160_OK) {
    rslt = processGyroOdr(&data[0], dev);
    if (rslt == BMI160_OK) {
      rslt = processGyroBw(&data[0], dev);
      if (rslt == BMI160_OK)
        rslt = processGyroRange(&data[1], dev);
    }
  }
  return rslt;
}

int8_t processGyroOdr(uint8_t *data, struct bmi160Dev *dev){
  int8_t rslt = 0;
  uint8_t temp = 0;
  uint8_t odr = 0;
  if (dev->gyroCfg.odr <= BMI160_GYRO_ODR_MAX) {
    if (dev->gyroCfg.odr != dev->prevGyroCfg.odr) {
      odr = (uint8_t)dev->gyroCfg.odr;
      temp = (*data & ~BMI160_GYRO_ODR_MASK);
      /* Adding output data rate */
      *data = temp | (odr & BMI160_GYRO_ODR_MASK);
    }
  } else {
    rslt = BMI160_E_OUT_OF_RANGE;
  }

  return rslt;
}

int8_t processGyroRange(uint8_t *data, struct bmi160Dev *dev){
  int8_t rslt = 0;
  uint8_t temp = 0;
  uint8_t range = 0;
  if (dev->gyroCfg.range <= BMI160_GYRO_RANGE_MAX) {
    if (dev->gyroCfg.range != dev->prevGyroCfg.range) {
      range = (uint8_t)dev->gyroCfg.range;
      temp = *data & ~BMI160_GYRO_RANGE_MSK;
      /* Adding range */
      *data = temp | (range & BMI160_GYRO_RANGE_MSK);
    }
  } else {
    rslt = BMI160_E_OUT_OF_RANGE;
  }

  return rslt;
}

int8_t processGyroBw(uint8_t *data, struct bmi160Dev *dev){
  int8_t rslt = 0;
  uint8_t temp = 0;
  uint8_t bw = 0;
  if (dev->gyroCfg.bw <= BMI160_GYRO_BW_MAX) {
    bw = (uint8_t)dev->gyroCfg.bw;
    temp = *data & ~BMI160_GYRO_BW_MASK;
    /* Adding bandwidth */
    *data = temp | ((bw << 4) & BMI160_GYRO_BW_MASK);
  } else {
    rslt = BMI160_E_OUT_OF_RANGE;
  }
  return rslt;
}

int8_t checkInvalidSettg( struct bmi160Dev *dev){
  int8_t rslt;
  uint8_t data = 0;
  // read the error reg
  rslt = getRegs(BMI160_ERROR_REG_ADDR, &data, 1, dev);
  data = data >> 1;
  data = data & BMI160_ERR_REG_MASK;
  if (data == 1)
    rslt = BMI160_E_ACCEL_ODR_BW_INVALID;
  else if (data == 2)
    rslt = BMI160_E_GYRO_ODR_BW_INVALID;
  else if (data == 3)
    rslt = BMI160_E_LWP_PRE_FLTR_INT_INVALID;
  else if (data == 7)
    rslt = BMI160_E_LWP_PRE_FLTR_INVALID;

  return rslt;
}

int8_t setAccelConf(struct bmi160Dev *dev){
  int8_t rslt = BMI160_OK;
  uint8_t data[2] = {0};
  rslt = checkAccelConfig(data, dev);
  if (rslt == BMI160_OK) {
    rslt = setRegs(BMI160_ACCEL_CONFIG_ADDR, &data[0], 1, dev);
    if (rslt == BMI160_OK) {
      dev->prevAccelCfg.odr = dev->accelCfg.odr;
      dev->prevAccelCfg.bw = dev->accelCfg.bw;
      //delay(BMI160_ONE_MS_DELAY);
      HAL_Delay(BMI160_ONE_MS_DELAY);
      rslt = setRegs(BMI160_ACCEL_RANGE_ADDR, &data[1], 1, dev);
      if (rslt == BMI160_OK){
        dev->prevAccelCfg.range = dev->accelCfg.range;
      }
    }
  }
  return rslt;
}

int8_t checkAccelConfig(uint8_t *data, struct bmi160Dev *dev){
  int8_t rslt;
  /* read accel Output data rate and bandwidth */
  rslt = getRegs(BMI160_ACCEL_CONFIG_ADDR, data, 2, dev);
  if (rslt == BMI160_OK) {
    rslt = processAccelOdr(&data[0], dev);
    if (rslt == BMI160_OK) {
      rslt = processAccelBw(&data[0], dev);
      if (rslt == BMI160_OK)
        rslt = processAccelRange(&data[1], dev);
    }
  }
  return rslt;
}

int8_t processAccelOdr(uint8_t *data,  struct bmi160Dev *dev){
  int8_t rslt = 0;
  uint8_t temp = 0;
  uint8_t odr = 0;

  if (dev->accelCfg.odr <= BMI160_ACCEL_ODR_MAX) {
    if (dev->accelCfg.odr != dev->prevAccelCfg.odr) {
      odr = (uint8_t)dev->accelCfg.odr;
      temp = *data & ~BMI160_ACCEL_ODR_MASK;
      /* Adding output data rate */
      *data = temp | (odr & BMI160_ACCEL_ODR_MASK);
    }
  } else {
    rslt = BMI160_E_OUT_OF_RANGE;
  }

  return rslt;
}

int8_t processAccelBw(uint8_t *data, struct bmi160Dev *dev){
  int8_t rslt = 0;
  uint8_t temp = 0;
  uint8_t bw = 0;

  if (dev->accelCfg.bw <= BMI160_ACCEL_BW_MAX) {
    if (dev->accelCfg.bw != dev->prevAccelCfg.bw) {
      bw = (uint8_t)dev->accelCfg.bw;
      temp = *data & ~BMI160_ACCEL_BW_MASK;
      /* Adding bandwidth */
      *data = temp | ((bw << 4) & BMI160_ACCEL_ODR_MASK);
    }
  } else {
    rslt = BMI160_E_OUT_OF_RANGE;
  }

  return rslt;
}

int8_t processAccelRange(uint8_t *data, struct bmi160Dev *dev){
  int8_t rslt = 0;
  uint8_t temp = 0;
  uint8_t range = 0;

  if (dev->accelCfg.range <= BMI160_ACCEL_RANGE_MAX) {
    if (dev->accelCfg.range != dev->prevAccelCfg.range) {
      range = (uint8_t)dev->accelCfg.range;
      temp = *data & ~BMI160_ACCEL_RANGE_MASK;
      /* Adding range */
      *data = temp | (range & BMI160_ACCEL_RANGE_MASK);
    }
  } else {
    rslt = BMI160_E_OUT_OF_RANGE;
  }

  return rslt;
}

int8_t setPowerMode(struct bmi160Dev *dev){
  int8_t rslt = 0;
  rslt = setAccelPwr(dev);
  if (rslt == BMI160_OK){
    rslt = setGyroPwr(dev);
  }
  return rslt;
}

int8_t setAccelPwr(struct bmi160Dev *dev){
  int8_t rslt = 0;
  uint8_t data = 0;
  if ((dev->accelCfg.power >= BMI160_ACCEL_SUSPEND_MODE) &&
    (dev->accelCfg.power <= BMI160_ACCEL_LOWPOWER_MODE)) {
    if (dev->accelCfg.power != dev->prevAccelCfg.power) {
      rslt = processUnderSampling(&data, dev);
      if (rslt == BMI160_OK) {
        /* Write accel power */
        rslt = setRegs(BMI160_COMMAND_REG_ADDR, &dev->accelCfg.power, 1, dev);
        /* Add delay of 5 ms */
        if (dev->prevAccelCfg.power == BMI160_ACCEL_SUSPEND_MODE){
          //delay(BMI160_ACCEL_DELAY_MS);
        	HAL_Delay(BMI160_ACCEL_DELAY_MS);
        }
        dev->prevAccelCfg.power = dev->accelCfg.power;
      }
    }
  } else {
    rslt = BMI160_E_OUT_OF_RANGE;
  }
  return rslt;
}

int8_t processUnderSampling(uint8_t *data, struct bmi160Dev *dev){
  int8_t rslt;
  uint8_t temp = 0;
  uint8_t pre_filter = 0;
  rslt = getRegs(BMI160_ACCEL_CONFIG_ADDR, data, 1, dev);
  if (rslt == BMI160_OK) {
    if (dev->accelCfg.power == BMI160_ACCEL_LOWPOWER_MODE) {
      temp = *data & ~BMI160_ACCEL_UNDERSAMPLING_MASK;
      /* Set under-sampling parameter */
      *data = temp | ((1 << 7) & BMI160_ACCEL_UNDERSAMPLING_MASK);
      /* Write data */
      rslt = setRegs(BMI160_ACCEL_CONFIG_ADDR, data, 1, dev);
      /* disable the pre-filter data in
       * low power mode */
      if (rslt == BMI160_OK)
        /* Disable the Pre-filter data*/
        rslt = setRegs(BMI160_INT_DATA_0_ADDR, &pre_filter, 2, dev);
    } else {
      if (*data & BMI160_ACCEL_UNDERSAMPLING_MASK) {

        temp = *data & ~BMI160_ACCEL_UNDERSAMPLING_MASK;
        /* disable under-sampling parameter
        if already enabled */
        *data = temp;
        /* Write data */
        rslt =setRegs(BMI160_ACCEL_CONFIG_ADDR, data, 1, dev);
      }
    }
  }

  return rslt;
}

int8_t setGyroPwr(struct bmi160Dev *dev){
  int8_t rslt = 0;
  if ((dev->gyroCfg.power == BMI160_GYRO_SUSPEND_MODE) || (dev->gyroCfg.power == BMI160_GYRO_NORMAL_MODE)
    || (dev->gyroCfg.power == BMI160_GYRO_FASTSTARTUP_MODE)) {
    if (dev->gyroCfg.power != dev->prevGyroCfg.power) {
      /* Write gyro power */
      rslt = setRegs(BMI160_COMMAND_REG_ADDR, &dev->gyroCfg.power, 1, dev);
      if (dev->prevGyroCfg.power == BMI160_GYRO_SUSPEND_MODE) {
        /* Delay of 81 ms */
        //delay(BMI160_GYRO_DELAY_MS);
        HAL_Delay(BMI160_GYRO_DELAY_MS);
      } else if ((dev->prevGyroCfg.power == BMI160_GYRO_FASTSTARTUP_MODE)
        && (dev->gyroCfg.power == BMI160_GYRO_NORMAL_MODE)) {
        /* This delay is required for transition from
        fast-startup mode to normal mode */
        //delay(10);
        HAL_Delay(10);
      } else {
        /* do nothing */
      }
      dev->prevGyroCfg.power = dev->gyroCfg.power;
    }
  } else {
    rslt = BMI160_E_OUT_OF_RANGE;
  }

  return rslt;
}

float getAccX(){
	int rslt;
	float k;
	int16_t accelGyro[6]={0};
	rslt =getAccelGyroData(accelGyro);
	k=accelGyro[3]/16384.0;
	return k;
}

float getAccY(){
	int rslt;
	float k;
	int16_t accelGyro[6]={0};
	rslt =getAccelGyroData(accelGyro);
	k=accelGyro[4]/16384.0;
	return k;
}
float getAccZ(){
	int rslt;
	float k;
	int16_t accelGyro[6]={0};
	rslt =getAccelGyroData(accelGyro);
	k=accelGyro[5]/16384.0;
	return -k;
}

int getstep(void){
	uint16_t k=0;
    uint16_t stepCounter = 0;
    if (readStepCounter(&stepCounter,Obmi160)==BMI160_OK){
	  k=stepCounter;
    }
	return k;
}

int8_t readStepCounter(uint16_t *stepVal, struct bmi160Dev *dev){
  int8_t rslt = BMI160_OK;
  uint8_t data[2] = {0, 0};
  uint16_t msb = 0;
  uint8_t lsb = 0;
  if (dev == NULL){
    rslt = BMI160_E_NULL_PTR;
  }
  if (rslt != BMI160_OK) {
    rslt = BMI160_E_NULL_PTR;
  } else {
    rslt = getRegs(BMI160_INT_STEP_CNT_0_ADDR, data, 2, dev);
    if (rslt == BMI160_OK) {
      lsb = data[0];
      msb = data[1] << 8;
      *stepVal = msb | lsb;
    }
  }
  return rslt;
}

int8_t getAccelGyroData( int16_t* data)
{
  int8_t rslt = BMI160_OK;
  rslt = getSensorData((BMI160_ACCEL_SEL | BMI160_GYRO_SEL),Oaccel, Ogyro, Obmi160);
  if(rslt == BMI160_OK){
    data[0]=Ogyro->x;
    data[1]=Ogyro->y;
    data[2]=Ogyro->z;
    data[3]=Oaccel->x;
    data[4]=Oaccel->y;
    data[5]=Oaccel->z;
  }
  return rslt;
}

int8_t getSensorData(uint8_t select_sensor, struct bmi160SensorData *accel, struct bmi160SensorData *gyro,struct bmi160Dev *dev){
  int8_t rslt = BMI160_OK;
  uint8_t time_sel;
  uint8_t sen_sel;
  uint8_t len = 0;
  /*Extract the sensor  and time select information*/
  sen_sel = select_sensor & BMI160_SEN_SEL_MASK;
  time_sel = ((sen_sel & BMI160_TIME_SEL) >> 2);
  sen_sel = sen_sel & (BMI160_ACCEL_SEL | BMI160_GYRO_SEL);
  if (time_sel == 1)
    len = 3;
  /* Null-pointer check */
  if (dev != NULL) {
    switch (sen_sel) {
    case eBmi160AccelOnly:
      /* Null-pointer check */
      if (accel == NULL)
        rslt = BMI160_E_NULL_PTR;
      else
        rslt =getAccelData(len, accel, dev);
      break;
    case eBmi160GyroOnly:
      /* Null-pointer check */
      if (gyro == NULL)
        rslt = BMI160_E_NULL_PTR;
      else
        rslt = getGyroData1(len, gyro, dev);
      break;
    case eBmi160BothAccelAndGyro:
      /* Null-pointer check */
      if ((gyro == NULL) || (accel == NULL))
        rslt = BMI160_E_NULL_PTR;
      else
        rslt = getAccelGyroData1(len, accel, gyro, dev);
      break;
    default:
      rslt = BMI160_E_INVALID_INPUT;
      break;
    }
  } else {
    rslt = BMI160_E_NULL_PTR;
  }

  return rslt;
}

int8_t getGyroData1(uint8_t len, struct bmi160SensorData *gyro, struct bmi160Dev *dev){
  int8_t rslt;
  uint8_t idx = 0;
  uint8_t data_array[15] = {0};
  uint8_t time_0 = 0;
  uint16_t time_1 = 0;
  uint32_t time_2 = 0;
  uint8_t lsb;
  uint8_t msb;
  float msblsb;
  if (len == 0) {
    /* read gyro data only */
    rslt = getRegs(BMI160_GYRO_DATA_ADDR, data_array, 6, dev);
    if (rslt == BMI160_OK) {
      /* Gyro Data */
      lsb = data_array[idx++];
      msb = data_array[idx++];
      msblsb = (int16_t)((msb << 8) | lsb);
      gyro->x = msblsb; /* Data in X axis */

      lsb = data_array[idx++];
      msb = data_array[idx++];
      msblsb = (int16_t)((msb << 8) | lsb);
      gyro->y = msblsb; /* Data in Y axis */

      lsb = data_array[idx++];
      msb = data_array[idx++];
      msblsb = (int16_t)((msb << 8) | lsb);
      gyro->z = msblsb; /* Data in Z axis */
    } else {
      rslt = BMI160_E_COM_FAIL;
    }
  } else {
    /* read gyro sensor data along with time */
    rslt = getRegs(BMI160_GYRO_DATA_ADDR, data_array, 12 + len, dev);
    if (rslt == BMI160_OK) {
      /* Gyro Data */
      lsb = data_array[idx++];
      msb = data_array[idx++];
      msblsb = (int16_t)((msb << 8) | lsb);
      gyro->x = msblsb; /* gyro X axis data */

      lsb = data_array[idx++];
      msb = data_array[idx++];
      msblsb = (int16_t)((msb << 8) | lsb);
      gyro->y = msblsb; /* gyro Y axis data */

      lsb = data_array[idx++];
      msb = data_array[idx++];
      msblsb = (int16_t)((msb << 8) | lsb);
      gyro->z = msblsb; /* gyro Z axis data */

      idx = idx + 6;
      time_0 = data_array[idx++];
      time_1 = (uint16_t)(data_array[idx++] << 8);
      time_2 = (uint32_t)(data_array[idx++] << 16);
      //gyro->sensortime = (uint32_t)(time_2 | time_1 | time_0);

    } else {
      rslt = BMI160_E_COM_FAIL;
    }
  }

  return rslt;
}

int8_t getGyroData( int16_t* data){
  int rslt = BMI160_OK;
  rslt = getSensorData(BMI160_GYRO_SEL, NULL, Ogyro, Obmi160);
  if(rslt == BMI160_OK){
    data[0]=Oaccel->x;
    data[1]=Oaccel->y;
    data[2]=Oaccel->z;
  }
  return rslt;
}

int8_t getAccelData(uint8_t len, struct bmi160SensorData *accel, struct bmi160Dev *dev){
  int8_t rslt;
  uint8_t idx = 0;
  uint8_t data_array[9] = {0};
  uint8_t time_0 = 0;
  uint16_t time_1 = 0;
  uint32_t time_2 = 0;
  uint8_t lsb;
  uint8_t msb;
  float msblsb;
  rslt = getRegs(BMI160_ACCEL_DATA_ADDR, data_array, 6 + len, dev);
  if (rslt == BMI160_OK){
    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    accel->x = msblsb; /* Data in X axis */
    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    accel->y = msblsb; /* Data in X axis */
    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    accel->z = msblsb; /* Data in X axis */
  }else{
    rslt = BMI160_E_COM_FAIL;
  }
  return rslt;
}

int8_t getAccelGyroData1(uint8_t len, struct bmi160SensorData *accel, struct bmi160SensorData *gyro, struct bmi160Dev *dev){
  int8_t rslt;
  uint8_t idx = 0;
  uint8_t data_array[15] = {0};
  uint8_t time_0 = 0;
  uint16_t time_1 = 0;
  uint32_t time_2 = 0;
  uint8_t lsb;
  uint8_t msb;
  int16_t msblsb;

  /* read both accel and gyro sensor data
   * along with time if requested */
  rslt = getRegs(BMI160_GYRO_DATA_ADDR, data_array, 12 + len, dev);
  if (rslt == BMI160_OK) {
    /* Gyro Data */
    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    gyro->x = msblsb; /* gyro X axis data */

    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    gyro->y = msblsb; /* gyro Y axis data */

    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    gyro->z = msblsb; /* gyro Z axis data */

    /* Accel Data */
    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    accel->x = (int16_t)msblsb; /* accel X axis data */

    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    accel->y = (int16_t)msblsb; /* accel Y axis data */

    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    accel->z = (int16_t)msblsb; /* accel Z axis data */

    if (len == 3) {
      time_0 = data_array[idx++];
      time_1 = (uint16_t)(data_array[idx++] << 8);
      time_2 = (uint32_t)(data_array[idx++] << 16);
      //accel->sensortime = (uint32_t)(time_2 | time_1 | time_0);
      //gyro->sensortime = (uint32_t)(time_2 | time_1 | time_0);
    } else {
      //accel->sensortime = 0;
      //gyro->sensortime = 0;
      ;
    }
  } else {
    rslt = BMI160_E_COM_FAIL;
  }

  return rslt;
}

//底层通信层
int8_t getRegs(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi160Dev *dev){
  int8_t rslt = BMI160_OK;
  if (dev == NULL) {
    rslt = BMI160_E_NULL_PTR;
  } else {
    rslt = I2cGetRegs(dev, reg_addr, data, len);
    if (rslt != BMI160_OK){
      rslt = BMI160_E_COM_FAIL;
    }
  }
  return rslt;
}

int8_t I2cGetRegs(struct bmi160Dev *dev, uint8_t reg_addr, uint8_t *data, uint16_t len){
  HAL_I2C_Master_Transmit(&hi2c2,dev->id<<1,&reg_addr,1,1000);
  HAL_Delay(10);
  HAL_I2C_Master_Receive(&hi2c2,dev->id<<1,data,len,1000);
  return BMI160_OK;
}

int8_t setRegs(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi160Dev *dev){
  int8_t rslt = BMI160_OK;
  uint8_t count = 0;
  if(dev == NULL){
    rslt = BMI160_E_NULL_PTR;
  }else{
    rslt =I2cSetRegs(dev,reg_addr,data,len);
    HAL_Delay(1);
    if(rslt != BMI160_OK)
      rslt = BMI160_E_COM_FAIL;
  }
  return rslt;
}

int8_t I2cSetRegs(struct bmi160Dev *dev, uint8_t reg_addr, uint8_t *data, uint16_t len){
  uint8_t buf[255]={0};
  buf[0]=reg_addr;
  for(int i=1;i<len+1;i++)
	  buf[i]=data[i-1];
  HAL_I2C_Master_Transmit(&hi2c2,dev->id<<1,buf,len+1,1000);
  return BMI160_OK;
}
