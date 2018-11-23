#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrfx_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_spi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log_backend_rtt.h"
#include "nrf_log_instance.h"
#include "nrf_log_backend_interface.h"
#include "nrf_log_backend_flash.h"
#include "nrf_log_instance.h"
#include "nrf_log_types.h"
#include "nrf_log_str_formatter.h"
#include <SEGGER_RTT_Conf.h>
#include <SEGGER_RTT.h>
#include "buckler.h"
#include "kobukiActuator.h"
#include "kobukiSensorTypes.h"
#include "kobukiSensorPoll.h"
#include "kobukiUtilities.h"
#include "display.h"
#include "mpu9250.h"

#define LOG_USES_RTT 1
#define NRF_CLI_RTT_TERMINAL_ID 0


NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);


typedef enum {
  DRIVE_INF,
  STOP,
  DRIVE_DIST,
  TURN_RIGHT_INF,
  TURN_LEFT_INF,
  TURN_RIGHT_ANG,
  TURN_LEFT_ANG,
  REV_INF,
  REV_DIST
} robot_state_t;

int speed;
int turnSpeed;

static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder){
  const float CONVERSION =  0.00008529;
  float ret = (current_encoder - previous_encoder) * CONVERSION;
  if (current_encoder < previous_encoder){
    ret = CONVERSION*(0xFFFF - previous_encoder + current_encoder);
  }
  if (ret > .6){
    return 0;
  }
  return ret;
}

static float measure_distance_rev(uint16_t current_encoder, uint16_t previous_encoder){
  const float CONVERSION =  0.00008529;
  float ret = CONVERSION *(previous_encoder - current_encoder);
  if (previous_encoder < current_encoder) {
    ret = CONVERSION*(0xFFFF - previous_encoder + current_encoder);
  }
  if (ret > .5){
    return 0;
  }
  return ret;
}

int main(void)
{

  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();

  // Initialize speed
  speed = 100;
  turnSpeed = 100;

   // initialize LEDs
  nrf_gpio_pin_dir_set(23, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(25, NRF_GPIO_PIN_DIR_OUTPUT);

  // initialize display
  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
    .sck_pin = BUCKLER_LCD_SCLK,
    .mosi_pin = BUCKLER_LCD_MOSI,
    .miso_pin = BUCKLER_LCD_MISO,
    .ss_pin = BUCKLER_LCD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
  display_write("Hello, Human!", DISPLAY_LINE_0);
  printf("Display initialized!\n");

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  mpu9250_init(&twi_mngr_instance);
  printf("IMU initialized!\n");

  // initialize Kobuki
  kobukiInit();
  printf("Kobuki initialized!\n");

  // configure initial state
  KobukiSensors_t sensors = {0};
  uint16_t enc_start;
  robot_state_t state = STOP;

  // Initialize distances
  float dist;
  float angle;

  // loop forever
  char p_data[9];
  while (1)
  {
    kobukiSensorPoll(&sensors);
    nrf_delay_ms(100);
    
    memset(p_data, 0, 9);


    size_t rcnt = SEGGER_RTT_Read(NRF_CLI_RTT_TERMINAL_ID, p_data, 8);
    if (rcnt > 0)
    {
      
      display_write(p_data, DISPLAY_LINE_0);
      if (strcmp(p_data, "setSpeed") == 0)
      {
        memset(p_data, 0, 9);
        SEGGER_RTT_Read(NRF_CLI_RTT_TERMINAL_ID, p_data, 3);
        sscanf(p_data, "%d", &speed);
      }
      else if(strcmp(p_data, "turn spd") == 0)
      {
        memset(p_data, 0, 9);
        SEGGER_RTT_Read(NRF_CLI_RTT_TERMINAL_ID, p_data, 3);
        sscanf(p_data, "%d", &turnSpeed);
      }
      else if (strcmp(p_data, "driveInf") == 0)
      {
        state = DRIVE_INF;
      }
      else if (strcmp(p_data, "stop") == 0)
      {
        state = STOP;
      }
      else if (strcmp(p_data, "driveDis") == 0)
      {
        memset(p_data, 0, 9);
        SEGGER_RTT_Read(NRF_CLI_RTT_TERMINAL_ID, p_data, 3);
        sscanf(p_data, "%f", &dist);
        state = DRIVE_DIST;
        enc_start = sensors.leftWheelEncoder;
      }
      else if (strcmp(p_data, "rightInf") == 0)
      {
        state = TURN_RIGHT_INF;
      }
      else if (strcmp(p_data, "left Inf") == 0)
      {
        state = TURN_LEFT_INF;
      }
      else if (strcmp(p_data, "rightAng") == 0)
      {
        memset(p_data, 0, 9);
        SEGGER_RTT_Read(NRF_CLI_RTT_TERMINAL_ID, p_data, 4);
        sscanf(p_data, "%f", &angle);
        state = TURN_RIGHT_ANG;
        mpu9250_start_gyro_integration();
      }
      else if (strcmp(p_data, "leftAngl") == 0)
      {
        memset(p_data, 0, 9);
        SEGGER_RTT_Read(NRF_CLI_RTT_TERMINAL_ID, p_data, 4);
        sscanf(p_data, "%f", &angle);
        state = TURN_LEFT_ANG;
        mpu9250_start_gyro_integration();
      }
      else if (strcmp(p_data, "reverInf") == 0)
      {
        state = REV_INF;
      }
      else if (strcmp(p_data, "reverDis") == 0)
      {
        memset(p_data, 0, 9);
        SEGGER_RTT_Read(NRF_CLI_RTT_TERMINAL_ID, p_data, 3);
        sscanf(p_data, "%f", &dist);
        state = REV_DIST;
        enc_start = sensors.leftWheelEncoder;
      }
    }
    switch (state)
    {
      case DRIVE_INF: {
        char buf[16];
        snprintf(buf, 16, "DRIVE_INF");
        display_write(buf, DISPLAY_LINE_0);
        kobukiDriveDirect(speed, speed);
        break; // Each case needs to end with a break
      }
      case STOP: {
        char buf[16];
        snprintf(buf, 16, "STOP");
        display_write(buf, DISPLAY_LINE_0);
        kobukiDriveDirect(0,0);
        break;
      }
      case DRIVE_DIST: {
        char buf[16];
        snprintf(buf, 16, "DRIVE_DIST");
        display_write(buf, DISPLAY_LINE_0);
        float travel_dist = measure_distance(sensors.leftWheelEncoder, enc_start);
        char buf2[16];
        snprintf(buf2, 16, "%f", travel_dist);
        display_write(buf2, DISPLAY_LINE_1);
        if( travel_dist < dist)
        {
          kobukiDriveDirect(speed, speed);
        }
        else
        {
          state = STOP;
        }
        break;
      }
      case TURN_RIGHT_INF: {
        char buf[16];
        snprintf(buf, 16, "TURN_RIGHT_INF");
        display_write(buf, DISPLAY_LINE_0);
        kobukiDriveDirect(turnSpeed, -turnSpeed);
        break;
      }
      case TURN_LEFT_INF: {
        char buf[16];
        snprintf(buf, 16, "TURN_LEFT_INF");
        display_write(buf, DISPLAY_LINE_0);
        kobukiDriveDirect(-turnSpeed, turnSpeed);
        break;
      }
      case TURN_RIGHT_ANG: {
        char buf[16];
        snprintf(buf, 16, "TURN_RIGHT_ANG");
        display_write(buf, DISPLAY_LINE_0);
        mpu9250_measurement_t angle_struct = mpu9250_read_gyro_integration();
        char buf2[16];
        snprintf(buf2, 16, "%f", fabs(angle_struct.z_axis));
        display_write(buf2, DISPLAY_LINE_1);
        if(fabs(angle_struct.z_axis) <= angle)
        {
          kobukiDriveDirect(turnSpeed, -turnSpeed);
        }
        else
        {
          mpu9250_stop_gyro_integration();
          state = STOP;
        }
        break;
      }
      case TURN_LEFT_ANG: {
        char buf[16];
        snprintf(buf, 16, "TURN_LEFT_ANG");
        display_write(buf, DISPLAY_LINE_0);
        mpu9250_measurement_t angle_struct = mpu9250_read_gyro_integration();
        char buf2[16];
        snprintf(buf2, 16, "%f", fabs(angle_struct.z_axis));
        display_write(buf2, DISPLAY_LINE_1);
        if(fabs(angle_struct.z_axis) <= angle)
        {
          kobukiDriveDirect(-turnSpeed, turnSpeed);
        }
        else
        {
          mpu9250_stop_gyro_integration();
          state = STOP;
        }
        break; 
      }
      case REV_INF: {
        char buf[16];
        snprintf(buf, 16, "REV_INF");
        display_write(buf, DISPLAY_LINE_0);
        kobukiDriveDirect(-speed, -speed);
        break;
      }
      case REV_DIST: {
        char buf[16];
        snprintf(buf, 16, "REV_DIST");
        display_write(buf, DISPLAY_LINE_0);
        float travel_dist = measure_distance_rev(sensors.leftWheelEncoder, enc_start);
        char buf2[16];
        snprintf(buf2, 16, "%f", travel_dist);
        display_write(buf2, DISPLAY_LINE_1);
        if(travel_dist < dist)
        {
          kobukiDriveDirect(-speed, -speed);
        }
        else
        {
          state = STOP;
        }
        break; 
      }
    }
  }
}
