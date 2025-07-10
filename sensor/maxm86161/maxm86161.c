/***************************************************************************//**
* @file maxm86161.c
* @brief Platform independent driver for maxm86161 biometric sensor
* @version 1.0
*******************************************************************************
* # License
* <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
*******************************************************************************
*
* SPDX-License-Identifier: Zlib
*
* The licensor of this software is Silicon Laboratories Inc.
*
* This software is provided \'as-is\', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented; you must not
*    claim that you wrote the original software. If you use this software
*    in a product, an acknowledgment in the product documentation would be
*    appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
*    misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*
*******************************************************************************
*
* EVALUATION QUALITY
* This code has been minimally tested to ensure that it builds
* with the specified dependency versions and is suitable as
* a demonstration for evaluation purposes only.
* This code will be maintained at the sole discretion of Silicon Labs.
*
******************************************************************************/

#define DT_DRV_COMPAT maxim_maxm86161

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include<errno.h>

#include "maxm86161.h"

// #include <maxm86161.h>
// #include <maxm86161_config.h>
// #include <sl_udelay.h>


static const maxm86161_device_config_t maxm86161_basic_cfg = {
    .ppg_cfg = {
        .alc = 0,            // Ambient light cancellation off
        .offset = 0,         // Offset cancellation off
        .ppg_tint = MAXM86161_PPG_CFG_TINT_58p7_US, // Integration time (choose as needed)
        .adc_range = MAXM86161_PPG_CFG_LED_RANGE_16k, // ADC range (choose as needed)
        .smp_rate = MAXM86161_PPG_CFG_SMP_RATE_P1_99sps, // 100 samples/sec
        .smp_freq = MAXM86161_PPG_CFG_SMP_AVG_1,     // No averaging
    },
    .ledsq_cfg = {
        .ledsq1 = MAXM86161_LEDSQ_GREEN, // Slot 1: Green LED
        .ledsq2 = MAXM86161_LEDSQ_IR,    // Slot 2: IR LED
        .ledsq3 = MAXM86161_LEDSQ_RED,   // Slot 3: Red LED
        .ledsq4 = MAXM86161_LEDSQ_OFF,   // Remaining slots off
        .ledsq5 = MAXM86161_LEDSQ_OFF,
        .ledsq6 = MAXM86161_LEDSQ_OFF,
    },
    .int_cfg = {
        .sha = 0,
        .proxy = 0,
        .led_compliant = 0,
        .full_fifo = 0,
        .data_rdy = 0,
        .alc_ovf = 0,
        .die_temp = 0,
    },
    .ledpa_cfg = {
        .green = 0x1F, // Example: 5mA (set as needed)
        .ir    = 0x1F,
        .red   = 0x1F,
    },
    .int_level = 1, // FIFO interrupt threshold (not used, but must be set)
};



// --------------------- PRIVATE FUNCTION DECLARATIONS -----------------------

//Function for checking data if it's 1 or 0
static int maxm86161_bool_check ( uint8_t value );

//Function for tint data validation check
static int maxm86161_ppg_tint_check ( uint8_t value );

//Function for led range data validation check
static int maxm86161_led_range_check ( uint8_t value );

//Function for sample rate data validation check
static int maxm86161_smp_rate_check ( uint8_t value );

//Function for smp avg freq data validation check
static int maxm86161_smo_freq_check ( uint8_t value );

//Function for led range current data validation check
static int maxm86161_led_range_curr_check ( uint8_t value );

//Function for squence data validation check
static int maxm86161_sequence_check ( uint8_t value );

//Function for delay after reset bit is set to 1
static void maxm86161_soft_reset_delay( void );

static int maxm86161_sample_fetch(const struct device *dev,
                                  enum sensor_channel chan);
static int maxm86161_channel_get(const struct device *dev,
                                 enum sensor_channel chan,
                                 struct sensor_value *val);


//Function for letting I2C wait after status check
//static void maxm86161_dev_i2c_delays(void);

maxm86161_ppg_sample_t latest_sample;

static int maxm86161_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct maxm86161_data *data = dev->data;
    

    // Read samples from FIFO into the sample structure
    if (!maxm86161_read_samples_in_fifo(&latest_sample)) {
        return -ENODATA;
    }

    return 0;
}

static int maxm86161_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    struct maxm86161_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_LIGHT: // Or use custom channels if you define them
        val[0].val1 = latest_sample.ppg1;
        val[1].val1 = latest_sample.ppg2;
        val[2].val1 = latest_sample.ppg3;
        return 0;
    default:
        return -ENOTSUP;
    }
}


// --------------------- Device configuration functions -----------------------

/***************************************************************************//**
 * @brief
 *    Initialize the Maxim86161 with the device configuration
 *
 * @param[in] global_cfg
 * device configuration structure
 *
 * @return
 *    sl_status_t error code
 ******************************************************************************/
int maxm86161_init_device(const maxm86161_device_config_t *global_cfg)
{
  int ret = 0;
  maxm86161_software_reset();
  ret |= maxm86161_ppg_config(&global_cfg->ppg_cfg);
  ret |= maxm86161_led_sequence_config(&global_cfg->ledsq_cfg);
  ret |= maxm86161_interrupt_control(&global_cfg->int_cfg);
  maxm86161_led_pa_config(&global_cfg->ledpa_cfg);
  // interrupt level setup should be happened after above configuration
  maxm86161_set_int_level(global_cfg->int_level);
  maxm86161_i2c_write_to_register(MAXM86161_REG_FIFO_CONFIG2, MAXM86161_FIFO_CFG_2_FULL_TYPE_RPT | MAXM86161_FIFO_CFG_2_FIFO_READ_DATA_CLR);
  // clear FIFO, don't know if it is necessary in this case
  maxm86161_flush_fifo();
#ifdef PROXIMITY
  maxm86161_i2c_write_to_register(MAXM86161_REG_LED_PILOT_PA, 0x05);
  maxm86161_i2c_write_to_register(MAXM86161_REG_PROX_INT_THRESHOLD, 0x01);  //threshold = 1*2048
#endif
  return ret;
}

/***************************************************************************//**
 * @brief
 *    Turn on/off device mode
 *
 *    All interrupts are cleared.
 *    In this mode, the oscillator is shutdown and the part draws minimum current
 *    If this bit is asserted during an active conversion,
 *    then the conversion is aborted.
 *
 * @param[in] turn_off
 * bool value for turn on/off option
 *
 * @return
 *    None
 ******************************************************************************/
void maxm86161_shutdown_device(bool turn_off)
{
  uint8_t value = 0;
  value = maxm86161_i2c_read_from_register(MAXM86161_REG_SYSTEM_CONTROL);
  if(turn_off)
    value |= MAXM86161_SYS_CTRL_SHUT_DOWN;
  else
    value &= ~MAXM86161_SYS_CTRL_SHUT_DOWN;
  maxm86161_i2c_write_to_register(MAXM86161_REG_SYSTEM_CONTROL, value);
}


/***************************************************************************//**
 * @brief
 *
 *  All configuration, threshold and data registers including distributed
 *  registers are reset to their power-on-state
 *
 * @return
 *    None
 ******************************************************************************/
void maxm86161_software_reset()
{
  uint8_t value = 0;
  value = maxm86161_i2c_read_from_register(MAXM86161_REG_SYSTEM_CONTROL);
  value |= MAXM86161_SYS_CTRL_SW_RESET;
  maxm86161_i2c_write_to_register(MAXM86161_REG_SYSTEM_CONTROL, value);
  maxm86161_soft_reset_delay();
}

/***************************************************************************//**
 * @brief
 *
 *  the FIFO gets flushed, FIFO_DATA_COUNT becomes 0.
 *  The contents of the FIFO are lost.
 *
 * @return
 *    None
 ******************************************************************************/
void maxm86161_flush_fifo()
{
  uint8_t value = 0;
  value = maxm86161_i2c_read_from_register(MAXM86161_REG_FIFO_CONFIG2);
  value |= MAXM86161_FIFO_CFG_2_FLUSH_FIFO;
  maxm86161_i2c_write_to_register(MAXM86161_REG_FIFO_CONFIG2, value);
}

/***************************************************************************//**
 * @brief
 *    Set the number of sample the fifo for maxim to fire an FULL interrupt
 *
 * @param[in] level
 * number of sample
 *
 * @return
 *    None
 ******************************************************************************/
void maxm86161_set_int_level(uint8_t level)
{
  uint8_t value = 0;
  value = 128 - level;
  maxm86161_i2c_write_to_register(MAXM86161_REG_FIFO_CONFIG1, value);
}

/***************************************************************************//**
 * @brief
 *    Configure PPG (such as adc range, sample rate, ...)
 *
 * @param[in] *ppg_cfg
 * pointer to the ppg configuration struct
 *
 * @return
 *    sl_status_t error code
 ******************************************************************************/
int maxm86161_ppg_config(const maxm86161_ppg_cfg_t *ppg_cfg)
{
    /* validate each field; return -EINVAL on any failure */
  if (maxm86161_bool_check(ppg_cfg->alc) < 0 ||
      maxm86161_bool_check(ppg_cfg->offset) < 0 ||
      maxm86161_ppg_tint_check(ppg_cfg->ppg_tint) < 0 ||
      maxm86161_led_range_check(ppg_cfg->adc_range) < 0 ||
      maxm86161_smp_rate_check(ppg_cfg->smp_rate) < 0 ||
      maxm86161_smo_freq_check(ppg_cfg->smp_freq) < 0) {
    return -EINVAL;
      }

  maxm86161_i2c_write_to_register( MAXM86161_REG_PPG_CONFIG1, (
                          ( ppg_cfg->alc << MAXM86161_PPG_CFG_ALC ) |
                          ( ppg_cfg->offset << MAXM86161_PPG_CFG_OFFSET ) |
                          ( ppg_cfg->adc_range << MAXM86161_PPG_CFG_ADC_RANGE ) |
                          ( ppg_cfg->ppg_tint << MAXM86161_PPG_CFG_TINT ) ) );

  maxm86161_i2c_write_to_register( MAXM86161_REG_PPG_CONFIG2, (
                          ( ppg_cfg->smp_rate << MAXM86161_PPG_CFG_SMP_RATE ) |
                          ( ppg_cfg->smp_freq << MAXM86161_PPG_CFG_SMP_AVG ) ) );
  return 0;
}


/***************************************************************************//**
 * @brief
 *    Configure LED current for a specific LED
 *
 * @param[in] ledx
 * no of led that need to change current
 *
 * @param[in] value
 * current of the LED
 *
 * @return
 *    sl_status_t error code
 ******************************************************************************/
int maxm86161_led_pa_config_specific(uint8_t ledx, uint8_t value)
{

  switch (ledx){
          case HEARTRATE2_LED_1:
            maxm86161_i2c_write_to_register(MAXM86161_REG_LED1_PA, value);
            break;
          case HEARTRATE2_LED_2:
            maxm86161_i2c_write_to_register(MAXM86161_REG_LED2_PA, value);
            break;
          case HEARTRATE2_LED_3:
            maxm86161_i2c_write_to_register(MAXM86161_REG_LED3_PA, value);
            break;
          default :
            return -EINVAL;
  }
  return 0;
}

/***************************************************************************//**
 * @brief
 *    Configure LED current for all the LED at the initial stage
 *
 * @param[in] *ledpa
 *  pointer to the ledpa struct
 *
 * @return
 *    None
 ******************************************************************************/
void maxm86161_led_pa_config (const maxm86161_ledpa_t *ledpa )
{
  maxm86161_i2c_write_to_register( MAXM86161_REG_LED1_PA, ledpa->green );
  maxm86161_i2c_write_to_register( MAXM86161_REG_LED2_PA, ledpa->ir );
  maxm86161_i2c_write_to_register( MAXM86161_REG_LED3_PA, ledpa->red );
}

/***************************************************************************//**
 * @brief
 *    Configure range current for all the LED at the initial stage
 *
 * @param[in] *led_range
 * pointer to the led_range struct
 *
 * @return
 *    sl_status_t error code
 *
 ******************************************************************************/
int maxm86161_led_range_config(const maxm86161_led_range_curr_t *led_range)
{
    int ret;

    /* Validate each field, return –EINVAL on any bad value */
    ret = maxm86161_led_range_curr_check(led_range->green);
    if (ret < 0) {
        return -EINVAL;
    }
    ret = maxm86161_led_range_curr_check(led_range->ir);
    if (ret < 0) {
        return -EINVAL;
    }
    ret = maxm86161_led_range_curr_check(led_range->red);
    if (ret < 0) {
        return -EINVAL;
    }

    /* Build the packed register value and write it */
    ret = maxm86161_i2c_write_to_register(
        MAXM86161_REG_LED_RANGE1,
        (led_range->green << MAXM86161_LED_RANGE_SHIFT_GREEN) |
        (led_range->ir    << MAXM86161_LED_RANGE_SHIFT_IR)    |
        (led_range->red   << MAXM86161_LED_RANGE_SHIFT_RED)
    );
    return ret;
}

/***************************************************************************//**
 * @brief
 *    Configure led sequence
 *    The data format in the FIFO as well as the sequencing of exposures are controlled by the LED Sequence
 *    Registers using LEDC1 through LEDC6
 *
 * @param[in] *ledsq
 * pointer to the led_range struct
 *
 * @return
 *    sl_status_t error code
 ******************************************************************************/
int maxm86161_led_sequence_config(const maxm86161_ledsq_cfg_t *ledsq)
{
    int ret;

    /* Validate each sequence entry */
    ret = maxm86161_sequence_check(ledsq->ledsq1);
    if (ret < 0) {
        return -EINVAL;
    }
    ret = maxm86161_sequence_check(ledsq->ledsq2);
    if (ret < 0) {
        return -EINVAL;
    }
    ret = maxm86161_sequence_check(ledsq->ledsq3);
    if (ret < 0) {
        return -EINVAL;
    }
    ret = maxm86161_sequence_check(ledsq->ledsq4);
    if (ret < 0) {
        return -EINVAL;
    }
    ret = maxm86161_sequence_check(ledsq->ledsq5);
    if (ret < 0) {
        return -EINVAL;
    }
    ret = maxm86161_sequence_check(ledsq->ledsq6);
    if (ret < 0) {
        return -EINVAL;
    }

    /* Pack and write each pair of sequence registers */
    ret = maxm86161_i2c_write_to_register(
        MAXM86161_REG_LED_SEQ1,
        (ledsq->ledsq2 << MAXM86161_LEDSQ_SHIFT) |
        (ledsq->ledsq1)
    );
    if (ret < 0) {
        return ret;
    }

    ret = maxm86161_i2c_write_to_register(
        MAXM86161_REG_LED_SEQ2,
        (ledsq->ledsq4 << MAXM86161_LEDSQ_SHIFT) |
        (ledsq->ledsq3)
    );
    if (ret < 0) {
        return ret;
    }

    return maxm86161_i2c_write_to_register(
        MAXM86161_REG_LED_SEQ3,
        (ledsq->ledsq6 << MAXM86161_LEDSQ_SHIFT) |
        (ledsq->ledsq5)
    );
}

/***************************************************************************//**
 * @brief
 *    Configure interrupt at the initial stage
 *
 * @param[in] *int_ctrl
 * pointer to the interrupt control struct
 *
 * @return
 *    sl_status_t error code
 *
 ******************************************************************************/
int maxm86161_interrupt_control(const maxm86161_int_t *int_ctrl)
{
    int ret;

    /* Validate each interrupt flag */
    ret = maxm86161_bool_check(int_ctrl->sha);
    if (ret < 0) {
        return -EINVAL;
    }
    ret = maxm86161_bool_check(int_ctrl->proxy);
    if (ret < 0) {
        return -EINVAL;
    }
    ret = maxm86161_bool_check(int_ctrl->led_compliant);
    if (ret < 0) {
        return -EINVAL;
    }
    ret = maxm86161_bool_check(int_ctrl->full_fifo);
    if (ret < 0) {
        return -EINVAL;
    }
    ret = maxm86161_bool_check(int_ctrl->data_rdy);
    if (ret < 0) {
        return -EINVAL;
    }
    ret = maxm86161_bool_check(int_ctrl->alc_ovf);
    if (ret < 0) {
        return -EINVAL;
    }
    ret = maxm86161_bool_check(int_ctrl->die_temp);
    if (ret < 0) {
        return -EINVAL;
    }

    /* Build and write IRQ_ENABLE1 */
    ret = maxm86161_i2c_write_to_register(
        MAXM86161_REG_IRQ_ENABLE1,
        (int_ctrl->full_fifo     << MAXM86161_INT_SHIFT_FULL)         |
        (int_ctrl->data_rdy      << MAXM86161_INT_SHIFT_DATA_RDY)    |
        (int_ctrl->alc_ovf       << MAXM86161_INT_SHIFT_ALC_OVF)     |
        (int_ctrl->proxy         << MAXM86161_INT_SHIFT_PROXY)       |
        (int_ctrl->led_compliant << MAXM86161_INT_SHIFT_LED_COMPLIANT)|
        (int_ctrl->die_temp      << MAXM86161_INT_SHIFT_DIE_TEMEP)
    );
    if (ret < 0) {
        return ret;
    }

    /* Write IRQ_ENABLE2 (SHA interrupt only) */
    return maxm86161_i2c_write_to_register(
        MAXM86161_REG_IRQ_ENABLE2,
        (int_ctrl->sha << MAXM86161_INT_SHIFT_SHA)
    );
}

/***************************************************************************//**
 * @brief
 *    Get status of all Maxim's interrupt
 *
 * @param[in] *int_status
 * pointer to queue where PPG sample is put
 *
 * @return
 *    None
 ******************************************************************************/
void maxm86161_get_irq_status(maxm86161_int_t *int_status)
{
  uint8_t int_status_value;

  int_status_value = maxm86161_i2c_read_from_register(MAXM86161_REG_IRQ_STATUS1);
  int_status->pwr_rdy = ((int_status_value >> MAXM86161_INT_SHIFT_PWR_RDY) &
                                                MAXM86161_INT_MASK );
  int_status->die_temp = ((int_status_value >> MAXM86161_INT_SHIFT_DIE_TEMEP) &
                                                 MAXM86161_INT_MASK );
  int_status->led_compliant = ((int_status_value >> MAXM86161_INT_SHIFT_LED_COMPLIANT) &
                                                      MAXM86161_INT_MASK );
  int_status->proxy = ((int_status_value >> MAXM86161_INT_SHIFT_PROXY) &
                                              MAXM86161_INT_MASK );
  int_status->alc_ovf = ((int_status_value >> MAXM86161_INT_SHIFT_ALC_OVF ) &
                                                MAXM86161_INT_MASK );
  int_status->data_rdy = ((int_status_value >> MAXM86161_INT_SHIFT_DATA_RDY) &
                                                 MAXM86161_INT_MASK );
  int_status->full_fifo = ( ( int_status_value >> MAXM86161_INT_SHIFT_FULL) &
                                                  MAXM86161_INT_MASK);

  int_status_value = maxm86161_i2c_read_from_register(MAXM86161_REG_IRQ_STATUS2);
  int_status->sha = ((int_status_value >> MAXM86161_INT_SHIFT_SHA) &
                                            MAXM86161_INT_MASK);
}

/***************************************************************************//**
 * @brief
 *    Process FULL interrupt to get PPG sample and put it into the queue
 *
 * @param[in] *sample
 * pointer to queue where PPG sample is put
 *
 * @return
 *    true: perfect sample (means PPG1, PPG2, PPG3)
 *    false: error
 ******************************************************************************/
bool maxm86161_read_samples_in_fifo(maxm86161_ppg_sample_t *sample)
{
  uint32_t temp_data;
  uint8_t sample_cnt;
  uint8_t block_buf[3*128];
  int i = 0;
  maxm86161_fifo_data_t fifo;
  bool task_started = false; // we only start to tream to uart incase we meet perfect sample (means PPG1, PPG2, PPG3)
  bool task_completed = false;

  sample_cnt = maxm86161_i2c_read_from_register(0x07);

  // reading one time for all the sample in buffer to prevent the case pushing speed to FIFO > reading speed
  maxm86161_i2c_block_read(MAXM86161_REG_FIFO_DATA, 3*sample_cnt, block_buf);
  for (i = 0; i < sample_cnt; i++)
  {
    temp_data = 0x000000;
    temp_data = (block_buf[i*3 + 0] << 16 | block_buf[i*3+1] << 8 | block_buf[i*3+2] );
    fifo.data_val = temp_data & MAXM86161_REG_FIFO_DATA_MASK;
    fifo.tag = (temp_data >> MAXM86161_REG_FIFO_RES) & MAXM86161_REG_FIFO_TAG_MASK;

#if (PROX_SELECTION & PROX_USE_IR)
    if( fifo.tag == 1){
      task_started = true;
      sample->ppg2 = fifo.data_val;
    } else if(fifo.tag == 2){
        sample->ppg1 = fifo.data_val;
    } else if(fifo.tag == 3){
        sample->ppg3 = fifo.data_val;
        if(task_started)
          task_completed = true;
    }
#elif (PROX_SELECTION & PROX_USE_RED)
    if( fifo.tag == 1){
      task_started = true;
      sample->ppg3 = fifo.data_val;
    } else if(fifo.tag == 2){
        sample->ppg2 = fifo.data_val;
    } else if(fifo.tag == 3){
        sample->ppg1 = fifo.data_val;
        if(task_started)
          task_completed = true;
    }
#else // default use green led for proximity
    if( fifo.tag == 1){
      task_started = true;
      sample->ppg1 = fifo.data_val;
    } else if(fifo.tag == 2){
        sample->ppg2 = fifo.data_val;
    } else if(fifo.tag == 3){
        sample->ppg3 = fifo.data_val;
        if(task_started)
          task_completed = true;
    }
#endif
    if(task_completed && task_started)
    {
      task_completed = false;
      task_started = false;
      return true;
    }
  }
  return false;
}

/* --------------------- PRIVATE FUNCTION DEFINITONS -----------------------*/

/* Need to delay to wait device ready after reset */
static void maxm86161_soft_reset_delay(void)
{
  k_busy_wait(1000);//microsecond
}

static int maxm86161_bool_check(uint8_t value)
{
    switch (value) {
    case 0x00:
    case 0x01:
        return 0;
    default:
        return -EINVAL;
    }
}

/** @brief Validate PPG integration time. */
static int maxm86161_ppg_tint_check(uint8_t value)
{
    switch (value) {
    case MAXM86161_PPG_CFG_TINT_14p8_US:
    case MAXM86161_PPG_CFG_TINT_29p4_US:
    case MAXM86161_PPG_CFG_TINT_58p7_US:
    case MAXM86161_PPG_CFG_TINT_117p3_US:
        return 0;
    default:
        return -EINVAL;
    }
}

/** @brief Validate LED ADC range. */
static int maxm86161_led_range_check(uint8_t value)
{
    switch (value) {
    case MAXM86161_PPG_CFG_LED_RANGE_4k:
    case MAXM86161_PPG_CFG_LED_RANGE_8k:
    case MAXM86161_PPG_CFG_LED_RANGE_16k:
    case MAXM86161_PPG_CFG_LED_RANGE_32k:
        return 0;
    default:
        return -EINVAL;
    }
}

/** @brief Validate sample rate setting. */
static int maxm86161_smp_rate_check(uint8_t value)
{
    switch (value) {
    case MAXM86161_PPG_CFG_SMP_RATE_P1_24sps:
    case MAXM86161_PPG_CFG_SMP_RATE_P1_50sps:
    case MAXM86161_PPG_CFG_SMP_RATE_P1_84sps:
    case MAXM86161_PPG_CFG_SMP_RATE_P1_99sps:
    case MAXM86161_PPG_CFG_SMP_RATE_P1_199sps:
    case MAXM86161_PPG_CFG_SMP_RATE_P1_399sps:
    case MAXM86161_PPG_CFG_SMP_RATE_P2_24sps:
    case MAXM86161_PPG_CFG_SMP_RATE_P2_50sps:
    case MAXM86161_PPG_CFG_SMP_RATE_P2_84sps:
    case MAXM86161_PPG_CFG_SMP_RATE_P2_99sps:
    case MAXM86161_PPG_CFG_SMP_RATE_P1_8sps:
    case MAXM86161_PPG_CFG_SMP_RATE_P1_16sps:
    case MAXM86161_PPG_CFG_SMP_RATE_P1_32sps:
    case MAXM86161_PPG_CFG_SMP_RATE_P1_64sps:
    case MAXM86161_PPG_CFG_SMP_RATE_P1_128sps:
    case MAXM86161_PPG_CFG_SMP_RATE_P1_256sps:
    case MAXM86161_PPG_CFG_SMP_RATE_P1_512sps:
    case MAXM86161_PPG_CFG_SMP_RATE_P1_1024sps:
    case MAXM86161_PPG_CFG_SMP_RATE_P1_2048sps:
    case MAXM86161_PPG_CFG_SMP_RATE_P1_4096sps:
        return 0;
    default:
        return -EINVAL;
    }
}

/** @brief Validate sample averaging (smooth filter) frequency. */
static int maxm86161_smo_freq_check(uint8_t value)
{
    switch (value) {
    case MAXM86161_PPG_CFG_SMP_AVG_1:
    case MAXM86161_PPG_CFG_SMP_AVG_2:
    case MAXM86161_PPG_CFG_SMP_AVG_4:
    case MAXM86161_PPG_CFG_SMP_AVG_8:
    case MAXM86161_PPG_CFG_SMP_AVG_16:
    case MAXM86161_PPG_CFG_SMP_AVG_32:
    case MAXM86161_PPG_CFG_SMP_AVG_64:
    case MAXM86161_PPG_CFG_SMP_AVG_128:
        return 0;
    default:
        return -EINVAL;
    }
}

/** @brief Validate LED current-range setting. */
static int maxm86161_led_range_curr_check(uint8_t value)
{
    switch (value) {
    case MAXM86161_LED_RANGE_CURRENT_31_MA:
    case MAXM86161_LED_RANGE_CURRENT_62_MA:
    case MAXM86161_LED_RANGE_CURRENT_93_MA:
    case MAXM86161_LED_RANGE_CURRENT_124_MA:
        return 0;
    default:
        return -EINVAL;
    }
}

/** @brief Validate LED-sequence code. */
static int maxm86161_sequence_check(uint8_t value)
{
    switch (value) {
    case MAXM86161_LEDSQ_GREEN:
    case MAXM86161_LEDSQ_IR:
    case MAXM86161_LEDSQ_RED:
    case MAXM86161_LEDSQ_PILOT_LED1:
    case MAXM86161_LEDSQ_DIRECT_AMBIENT:
    case MAXM86161_LEDSQ_OFF:
        return 0;
    default:
        return -EINVAL;
    }
}
void maxm86161_set_device(const struct device *dev);

int maxm86161_driver_init(const struct device *dev)
{
    const struct maxm86161_config *cfg = dev->config;
    struct maxm86161_data *data = dev->data;
    int ret;

    
    /* Set the device reference for I2C operations */
    maxm86161_set_device(dev);
    
    /* Check if I2C bus is ready */
    if (!device_is_ready(cfg->i2c.bus)) {
        return -ENODEV;
    }
    
    /* Initialize driver data */
    data->initialized = false;
    data->last_sample_time = 0;
    
    /* TODO: Add actual hardware initialization here */
    // int val = maxm86161_i2c_read_from_register(0xFF); // Replace 0x00 with a valid register
    // printk("MAXM86161: I2C read at boot returned %d\n", val);

    ret = maxm86161_i2c_read_from_register(MAXM86161_REG_PART_ID);
    if (ret < 0) {
        printk("MAXM86161: Failed to read part ID, error %d\n", ret);
        return ret;
    }

    printk("MAXM86161: Part ID read as %d\n", ret);

    // if (ret != MAXM86161_REG_PART_ID) {
    //     printk("MAXM86161: Part ID mismatch, expected %d, got %d\n", MAXM86161_REG_PART_ID, ret);
    //     return -ENODEV;
    // }
    // Replaced the code as the expected part ID is 0x36
    // Code above is BS as it compares the part ID with the register address
    // which is not correct. The expected part ID is 0x36.
    if (ret != 0x36) {
        printk("MAXM86161: Part ID mismatch, expected %d, got %d\n", 0x36, ret);
        return -ENODEV;
    }

    ret = maxm86161_init_device(&maxm86161_basic_cfg);
    if (ret) {
        printk("MAXM86161: Device config failed: %d\n", ret);
        return ret;
    }
    
    data->initialized = true;
    return 0;
}
static const struct sensor_driver_api maxm86161_api = 
{
    .sample_fetch = maxm86161_sample_fetch,
    .channel_get = maxm86161_channel_get,
};

#define MAXM86161_DEFINE(inst)                                              \
        static struct maxm86161_data maxm86161_data##inst;                  \
                                                                            \
        static const struct maxm86161_config maxm86161_config##inst = {     \
            .i2c = I2C_DT_SPEC_INST_GET(inst),                              \
        };                                                                  \
        DEVICE_DT_INST_DEFINE(inst,                                         \
                              maxm86161_driver_init,                        \
                              NULL,                                         \
                              &maxm86161_data##inst,                        \
                              &maxm86161_config##inst,                      \
                              POST_KERNEL,                                  \
                              CONFIG_SENSOR_INIT_PRIORITY,                  \
                              &maxm86161_api);

DT_INST_FOREACH_STATUS_OKAY(MAXM86161_DEFINE)