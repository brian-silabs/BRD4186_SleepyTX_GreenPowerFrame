# BRD4186_SleepyTX_GreenPowerFrame

This repository holds an application that runs on EFR32MG24B010F1536IM48

# Build and Execution Context
* Gecko SDK 4.2.2
* Based on an Empty_C sample application from Simplicity Studio
* Built using IAR 9.20.4 within Simplicity Studio
* No support of MX25 Flash Shutdown (in case ran on an actual BRD4182)
* Uses internal DCDC to supply 1.8V to PAVDD and RFVDD
* Power Supply must be above 2.0V for DCDC Init to pass

Some of the above can be changed via the .slcp file :
* Any of the Generators can be added (GCC, vscode, IAR EW)
* Part number can be changed within the same family

*IF ANY OF THE ABOVE ARE CHANGED, NOTE THAT FILES UNDER <project root>/autogen WILL BE UPDATED. DO NOT ADD THOSE TO ANY COMMIT*

# Project description
This project periodically sends out 1 Fixed Green Power Frames (22bytes payload + 2 bytes CRC, no cryptographic operations involved) over 802.15.4 Channel 18

This TX is followed by a 5ms receive window set 20ms afterwards

The period is set to be as close as 60 seconds 

TODO : change Payload to 42Bytes (40 + 2)

It does not take care of on the fly radio calibrations (TODO if requested)

It will also by default :
* Write 4 bytes to a flash page
* Perform CCA before every TX
* Go down to EM2 (Deep Sleep) mode in between RX and TX if possible
* Increment a reset counter into retained RAM 
* Set a 60 second timer and go to EM4 (Hibernate)

It can be fine tuned to include more or less tests based on BELOW macros :

| Macro                 | Sub-test Macro        | Description                                                               | Default   |
| --------------------- | --------------------- | ------------------------------------------------------------------------- | --------- |
| TEST_EMULATED_NVM     |                       | Will perform an MSC_Write to the first free word of the last flash page   | 1         |
|                       | PERFRORM_PAGE_ERASE   | Will force a full page erase of the last flash page                       | 0         |
| TEST_RADIO            |                       | Queries an entry for the given Node EUI64                                 | 1         |
|                       | CCA_ENABLE            | Will Enable CCA before each Transmit                                      | 1         |
|                       | CCA_THRESHOLD_DBM     | Sets the CCA threshold value in dBm                                       | -75       |
|                       | CCA_USE_PEAK          | Sets CCA to measure RSSI peak instead of default average (not tested)     | 0         |
|                       | PTI_ENABLE            | Enable PTI traces to enable Network Analyzer debug access (not tested)    | 0         |
|                       | RADIO_WAKEUP_TIME_US  | Estimated time in us required for Radio operations after EM2 wake up      | 800       |
|                       | RADIO_CHANNEL         | 802.15.4 channel number on which to send the GP frame                     | 18        |
|                       | RX_TO_TX_US           | Delay between TX and RX in Microseconds                                   | 20000     |
|                       | RX_WINDOW_US          | RX window duration in Microseconds                                        | 5000      |
| TEST_RETENTION        |                       | Allows initialization and write into retained RAM at startup              | 1         |
| BURTC_IRQ_PERIOD_MS   |                       | Wake up period of the application in milliseconds                         | 60000     |

# Specific optimizations and dependencies
In order to achieve the lowest power, some code initializations have been manually modified
Some other inits have been included as well
The detailed list is as follows :

* This application DOES NOT use the onboard LFXO
* This application DOES NOT perform HFXO Current BIAS optimizations, saving about 10nWh at each startup
    Instead it uses a precomputed value (To be checked by MCU apps team)

    The changes are made to :
    * <project-root>/gecko_sdk_4.2.2/platform/emlib/src/em_cmu.c
        Added definition of `CMU_HFXOInitAlt` that supresses optimizations
        Please note that the `CMU_HFXOInit_TypeDef.coreBiasAna` field must be provided
    * <project-root>/gecko_sdk_4.2.2/platform/emlib/inc/em_cmu.h
        Added declaration of `CMU_HFXOInitAlt`
    * <project-root>/gecko_sdk_4.2.2/platform/service/device_init/src/sl_device_init_hfxo_s2.c
    ``` c
    #if 1
    hfxoInit.coreBiasAna = 0x8;
    CMU_HFXOInitAlt(&hfxoInit);//10ms
    #else
    CMU_HFXOInit(&hfxoInit);
    #endif
    ```

# Other potential optimizations to be investigated

* HFRCO DPLL as SYSCLK could help increase CPU speed and reducing up time
    Not tested as this crushes the RF signal

* `BURTC_Init` call takes about 12ms to wait on registers sync
    This happens during startup at EM0 power consumption level
    This could save extra energy if it can be accelerated
