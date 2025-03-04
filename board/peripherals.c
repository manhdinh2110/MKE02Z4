/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v15.0
processor: MKE02Z64xxx4
package_id: MKE02Z64VLD4
mcu_data: ksdk2_0
processor_version: 24.12.10
functionalGroups:
- name: BOARD_InitPeripherals
  UUID: defe82f7-5a4b-483b-b6f5-6220c6177bf6
  called_from_default_init: true
  selectedCore: core0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system'
- global_system_definitions:
  - user_definitions: ''
  - user_includes: ''
  - global_init: ''
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'uart_cmsis_common'
- type_id: 'uart_cmsis_common'
- global_USART_CMSIS_common:
  - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'gpio_adapter_common'
- type_id: 'gpio_adapter_common'
- global_gpio_adapter_common:
  - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"

/***********************************************************************************************************************
 * BOARD_InitPeripherals functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * KBI0 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'KBI0'
- type: 'kbi'
- mode: 'general'
- custom_name_enabled: 'false'
- type_id: 'kbi_2.0.1'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'KBI0'
- config_sets:
  - general:
    - detectMode: 'kKBI_EdgesDetect'
    - pinConfig:
      - config:
        - 0:
          - pinEdge: '0'
          - pinEnable: 'false'
        - 1:
          - pinEdge: '0'
          - pinEnable: 'false'
        - 2:
          - pinEdge: '0'
          - pinEnable: 'false'
        - 3:
          - pinEdge: '1'
          - pinEnable: 'true'
        - 4:
          - pinEdge: '0'
          - pinEnable: 'false'
        - 5:
          - pinEdge: '0'
          - pinEnable: 'false'
        - 6:
          - pinEdge: '0'
          - pinEnable: 'false'
        - 7:
          - pinEdge: '0'
          - pinEnable: 'false'
    - interrupt:
      - IRQn: 'KBI0_IRQn'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
kbi_config_t KBI0_Config = {
  .pinsEnabled = 0x8U,
  .pinsEdge = 0x8U,
  .mode = kKBI_EdgesDetect
};

static void KBI0_init(void) {
  KBI_Init(KBI0_PERIPHERAL, &KBI0_Config);
}

/***********************************************************************************************************************
 * FTM2 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'FTM2'
- type: 'ftm'
- mode: 'EdgeAligned'
- custom_name_enabled: 'false'
- type_id: 'ftm_2.6.0'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'FTM2'
- config_sets:
  - ftm_main_config:
    - ftm_config:
      - clockSource: 'kFTM_SystemClock'
      - clockSourceFreq: 'GetFreq'
      - timerPrescaler: '1'
      - timerOutputFrequency: '10 kHz'
      - systemClockSource: 'BusInterfaceClock'
      - systemClockSourceFreq: 'mirrored_value'
      - faultMode: 'kFTM_Fault_Disable'
      - inputFilterPeriod: '1'
      - faultInputs:
        - 0:
          - enableFaultInput: 'false'
          - faultLevelVal: 'low'
          - useFaultFilter: 'false'
        - 1:
          - enableFaultInput: 'false'
          - faultLevelVal: 'low'
          - useFaultFilter: 'false'
        - 2:
          - enableFaultInput: 'false'
          - faultLevelVal: 'low'
          - useFaultFilter: 'false'
        - 3:
          - enableFaultInput: 'false'
          - faultLevelVal: 'low'
          - useFaultFilter: 'false'
      - deadTimePrescale: 'kFTM_Deadtime_Prescale_1'
      - deadTimePeriod: '0'
      - pwmSyncMode: 'kFTM_SoftwareTrigger'
      - swTriggerResetCount: 'true'
      - hwTriggerResetCount: 'false'
      - reloadPoints: ''
      - extTriggers: ''
      - chnlInitState: ''
      - chnlPolarity: ''
      - bdmMode: 'kFTM_BdmMode_0'
      - useGlobalTimeBase: 'false'
    - timer_interrupts: ''
    - enable_irq: 'false'
    - ftm_interrupt:
      - IRQn: 'FTM0_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - EnableTimerInInit: 'true'
  - ftm_edge_aligned_mode:
    - ftm_edge_aligned_channels_config: []
    - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const ftm_config_t FTM2_config = {
  .prescale = kFTM_Prescale_Divide_1,
  .faultMode = kFTM_Fault_Disable,
  .faultFilterValue = 0,
  .deadTimePrescale = kFTM_Deadtime_Prescale_1,
  .deadTimeValue = 0,
  .pwmSyncMode = kFTM_SoftwareTrigger,
  .swTriggerResetCount = true,
  .hwTriggerResetCount = false,
  .reloadPoints = 0,
  .extTriggers = 0,
  .chnlInitState = 0,
  .chnlPolarity = 0,
  .bdmMode = kFTM_BdmMode_0,
  .useGlobalTimeBase = false
};

static void FTM2_init(void) {
  FTM_Init(FTM2_PERIPHERAL, &FTM2_config);
  FTM_SetTimerPeriod(FTM2_PERIPHERAL, FTM2_TIMER_MODULO_VALUE);
  FTM_StartTimer(FTM2_PERIPHERAL, kFTM_SystemClock);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void)
{
  /* Initialize components */
  KBI0_init();
  FTM2_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
  BOARD_InitPeripherals();
}
