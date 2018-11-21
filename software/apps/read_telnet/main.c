#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrfx_gpiote.h"
#include "nrf_gpio.h"
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
#define  LOG_USES_RTT 1
#define NRF_CLI_RTT_TERMINAL_ID 0

int main(void)
{

    ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // loop forever
  char p_data[50];
  memset(p_data, 0, 50);
  while (1) {
     
     size_t rcnt = SEGGER_RTT_Read(NRF_CLI_RTT_TERMINAL_ID, p_data, 40);
     if (rcnt > 0)
     {
      printf("data received! %s", p_data);
     }
  }
}

