/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "rail.h"
#include "rail_types.h"
#include "rail_ieee802154.h"

#include "pa_conversions_efr32.h"

#include "em_msc.h"
#include "em_rmu.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_burtc.h"


// Radio related tests
// Radio test master switch
#define TEST_RADIO 1

#if TEST_RADIO
// Disable / Enable CCA assesment upon TX
#define CCA_ENABLE            1
#define CCA_THRESHOLD_DBM     (-75)
#define CCA_USE_PEAK          0
#if CCA_USE_PEAK
#define TX_OPTIONS  (RAIL_TX_OPTIONS_DEFAULT | RAIL_TX_OPTION_CCA_PEAK_RSSI)
#else
#define TX_OPTIONS  RAIL_TX_OPTIONS_DEFAULT
#endif

#define RX_AFTER_TX           1

// Disable / Enable PTI for Radio debugging
#define PTI_ENABLE            1

#define RADIO_WAKEUP_TIME_US  800 //Estimated time in us required for Radio operations after EM2 wake up

#define TX_BUFFER_SIZE        512
#define TX_MAX_PACKET_LENGTH  128

#define RADIO_CHANNEL         (18)

#define RX_TO_TX_US           20000//20ms
#define RX_WINDOW_US          5000  //5ms

#endif

// NVM / Simulated EEPROM test settings
// NVM test master switch
#define TEST_EMULATED_NVM     0

#if TEST_EMULATED_NVM
// Using the last flash page as storage slot for FC
#define LAST_FLASH_PAGE       ((uint32_t*)(FLASH_SIZE - FLASH_PAGE_SIZE))
// Should this test include a page erase upon startup
#define PERFRORM_PAGE_ERASE   0
#endif

// Data retention test during Hibernation master switch
#define TEST_RETENTION        0


// Number of 1 KHz ULFRCO clocks between BURTC interrupts
// Must fit into uint32_t
#define BURTC_IRQ_PERIOD_MS  5000// Will be 60s


// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------

#if TEST_RADIO
static void radioEventHandler(RAIL_Handle_t railHandle, RAIL_Events_t events);
void on_channel_config_change(RAIL_Handle_t rail_handle,
                                           const RAIL_ChannelConfigEntry_t *entry);
#endif //TEST_RADIO

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
#if TEST_RADIO
// GLobal RAIL Handle
RAIL_Handle_t greenPowerHandle_g;

// Global RAIL Init Config structure
RAIL_Config_t railCfg = { // Must never be const
 .eventsCallback = &radioEventHandler
};

// Global RAIL TX Power Configurations
// TX Power dBm
RAIL_TxPower_t txPower = 00; // Default to 0 dBm
// Power Configuration
const RAIL_TxPowerConfig_t railTxPowerConfig = { // May be const
    .mode = RAIL_TX_POWER_MODE_2P4GIG_LP,// Use Low Power PA (up to 0dBm)
    .rampTime = 2,
    .voltage = 1800 //Apply DCDC voltage 1.8V
};

#if CCA_ENABLE
RAIL_CsmaConfig_t csmaConfig = RAIL_CSMA_CONFIG_802_15_4_2003_2p4_GHz_OQPSK_CSMA;
#endif

// Global TX Packet configuration
// Green Power Toggle Frame 22 Bytes Encrypted Payload + CRC
uint8_t txData[TX_MAX_PACKET_LENGTH] = {
  0x18,//Number of bytes that follow

  0x01, 0x08,//Frame Control
  0x42, //Sequence
  0xFF, 0xFF, //Destination PanID
  0xFF, 0xFF, //Destination Address
  0x8C,//NWK Frame Control
  0x18,//Extended NWK Frame Control
  0x78, 0x56, 0x34, 0x12, //GPD Src ID
  0x42, 0x00, 0x00, 0x00, //Security FC
  0x68,//GPD Command ID
  0xAC, 0x56, 0x00, 0x85,//MIC
  0x45, 0x92//CRC
};

//Data length, must be the same as txData[0]
uint16_t txDataLen = 0x18;//Data Length includes both CRC bytes, payload is actually 22Bytes

// rx Schedule configuration
static RAIL_ScheduleRxConfig_t scheduleRxConfig =
    {
        .startMode = RAIL_TIME_DELAY,
        .start = RX_TO_TX_US,
        .endMode = RAIL_TIME_DELAY,
        .end = RX_WINDOW_US,
        .hardWindowEnd = true,
        .rxTransitionEndSchedule = 0
    };

// RAIL Packet mode configuration structure
// Not mandatory
RAIL_DataConfig_t data_config = {
  .txSource = TX_PACKET_DATA,
  .rxSource = RX_PACKET_DATA,
  .txMethod = PACKET_MODE,
  .rxMethod = PACKET_MODE,
};

// RAIL Automatic State transitions
// Set automatic transitions to always go to idle upon TX/Rx
// Might be overwritten by 802.15.4 Init if called before it
RAIL_StateTransitions_t railStateTransitions = {
.success = RAIL_RF_STATE_IDLE,
.error = RAIL_RF_STATE_IDLE,
};


// RAIL 802.15.4 MAC Layer configuration
static const RAIL_IEEE802154_Config_t rail154Config = {
   .addresses = NULL,
   .ackConfig = {
     .enable = true,     // Turn on auto ACK for IEEE 802.15.4.
     .ackTimeout = 672,  // See note above: 54-12 sym * 16 us/sym = 672 us.
     .rxTransitions = {
       .success = RAIL_RF_STATE_IDLE,  // Return to RX after ACK processing
       .error = RAIL_RF_STATE_IDLE,    // Ignored
     },
     .txTransitions = {
       .success = RAIL_RF_STATE_IDLE,  // Return to RX after ACK processing
       .error = RAIL_RF_STATE_IDLE,    // Ignored
     },
   },
   .timings = {
     .idleToRx = 100,
     .idleToTx = 100,
     .rxToTx = 192,    // 12 symbols * 16 us/symbol = 192 us
     .txToRx = 192,    // 12 symbols * 16 us/symbol = 192 us
     .rxSearchTimeout = 0, // Not used
     .txToRxSearchTimeout = 0, // Not used
   },
   .framesMask = RAIL_IEEE802154_ACCEPT_STANDARD_FRAMES,
   .promiscuousMode = false,  // Enable format and address filtering.
   .isPanCoordinator = false,
   .defaultFramePendingInOutgoingAcks = false,
 };

#if PTI_ENABLE
//If used, RAIL PTI signals routing so we can debug radio over wireshark/SSv5
RAIL_PtiConfig_t railPtiConfig = {
  .mode = RAIL_PTI_MODE_UART,
  .baud = 1600000,

  .doutPort = (uint8_t)gpioPortD,//DATA
  .doutPin = 4,

  .dframePort = (uint8_t)gpioPortD,//SYNC
  .dframePin = 5,
};
#endif

#endif //#if TEST_RADIO

// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------

#if TEST_RADIO
// Static variables used for application purposes
static bool txSequenceINitiated = 0;
static bool rxAfterTx = RX_AFTER_TX;

static bool calibrateRadio = false;

static uint32_t wakeupTime = RADIO_WAKEUP_TIME_US;
static bool shouldSleep = true;//From the app perspective, always go to sleep
static bool sleepAllowed = false;//Holding value for RAIL decision result

// RAIL TX Buffer
// Must be 4 bytes aligned on series 2
static union {
  // Used to align this buffer as needed
  RAIL_FIFO_ALIGNMENT_TYPE align[TX_BUFFER_SIZE / RAIL_FIFO_ALIGNMENT];
  uint8_t fifo[TX_BUFFER_SIZE];
} txFifo;

#endif //#if TEST_RADIO

#if TEST_EMULATED_NVM
static uint32_t frameCounterTestValue = 0xDEADCAFE;
#endif //TEST_EMULATED_NVM

static bool shouldHibernate = true;//From the app perspective, always go to EM4
static bool saveLastFC = false;

// -----------------------------------------------------------------------------
//                          Function Definitions
// -----------------------------------------------------------------------------
//                             Init Functions                                 //

#if TEST_EMULATED_NVM
  static void nvmInit(void){
    // Enable MSC Clock
    CMU_ClockEnable(cmuClock_MSC, true);

#if PERFRORM_PAGE_ERASE
    // Clear the Userdata page of any previous data stored
    MSC_ErasePage(LAST_FLASH_PAGE);
#endif

#if(!TEST_RADIO)
    saveLastFC = true;// If we test NVM without radio, force FC write to NVM
#endif
  }
#endif//#if TEST_EMULATED_NVM

#if TEST_RADIO
void radioInitialize(void)
{
  RAIL_Status_t initStatus;
  uint16_t fifoSize;

  // Initialize the RAIL library and any internal state it requires
  greenPowerHandle_g = RAIL_Init(&railCfg, NULL);
  if(greenPowerHandle_g == NULL)
  {
      while(1);
  }

  initStatus = RAIL_ConfigData(greenPowerHandle_g, &data_config);
  if(initStatus != RAIL_STATUS_NO_ERROR)
  {
      while(1);
  }

  initStatus = RAIL_ConfigChannels(greenPowerHandle_g, NULL, on_channel_config_change);
  if(initStatus != RAIL_STATUS_NO_ERROR)
  {
       while(1);
  }

  initStatus = RAIL_IEEE802154_Config2p4GHzRadio(greenPowerHandle_g);
  if(initStatus != RAIL_STATUS_NO_ERROR)
  {
       while(1);
  }

  initStatus = RAIL_IEEE802154_Init(greenPowerHandle_g, &rail154Config);
  if(initStatus != RAIL_STATUS_NO_ERROR)
  {
       while(1);
  }

// Configure calibration settings
// initStatus = RAIL_ConfigCal(greenPowerHandle_g, RAIL_CAL_ALL);
// if(initStatus != RAIL_STATUS_NO_ERROR)
// {
//     while(1);
// }

 // Configure the most useful callbacks plus catch a few errors
 initStatus = RAIL_ConfigEvents(greenPowerHandle_g,
                                  RAIL_EVENTS_ALL,
                                  RAIL_EVENTS_ALL);
 if(initStatus != RAIL_STATUS_NO_ERROR)
  {
     while(1);
  }

 initStatus = RAIL_SetRxTransitions(greenPowerHandle_g, &railStateTransitions);
 if(initStatus != RAIL_STATUS_NO_ERROR)
  {
      while(1);
  }

 initStatus = RAIL_SetTxTransitions(greenPowerHandle_g, &railStateTransitions);
 if(initStatus != RAIL_STATUS_NO_ERROR)
  {
      while(1);
  }

 // Setup the transmit buffer
 fifoSize = RAIL_SetTxFifo(greenPowerHandle_g, txFifo.fifo, 0, TX_BUFFER_SIZE);
 if(fifoSize == 0)
  {
    while(1);
  }

 // Configure PA
 initStatus = RAIL_ConfigTxPower(greenPowerHandle_g, &railTxPowerConfig);
 if(initStatus != RAIL_STATUS_NO_ERROR)
  {
    while(1);
  }

 // Initialize PA power curves
 // They come with the RAIL Utility PA component
 RAIL_InitTxPowerCurvesAlt(&RAIL_TxPowerCurvesDcdc);
 RAIL_EnablePaCal(true);

 //Set PA Tx Power
 initStatus = RAIL_SetTxPowerDbm(greenPowerHandle_g, txPower);
 if(initStatus != RAIL_STATUS_NO_ERROR)
  {
      while(1);
  }

  //CMU_ClockEnable(cmuClock_PRS, true);
  // Configure sleep for timer synchronization
  // On the EFR32xG1 and EFR32xG12 this uses the RTCC's clock which is the LFE clock with a possible additional prescalar
  // The slower the clock source, the longer it will take to synchronize because the process requires several ticks of the RTCC clock.
  // EFR32xG13 and EFR32xG14 have a separate timer to help with the synchronization, but either the LFXO or LFRCO are still used as a clock.
  // Note that to use this internal timer the cmuClock_HFLE clock must always be enabled.
  // When calling RAIL_ConfigSleep(), the code will first try to set up the timer with the LFXO if it's started, then fall back to the LFRCO, and finally assert if no LF clocks are running.

  //Enable SleepTimer / PROTIMER first or it hangs here
  //Sleeptimer component is mandatory here (PROTIMER INIT MISSING FROM RAIL)
  initStatus = RAIL_ConfigSleep(greenPowerHandle_g, RAIL_SLEEP_CONFIG_TIMERSYNC_ENABLED);
  if(initStatus != RAIL_STATUS_NO_ERROR)
  {
    while(1);
  }

#if CCA_ENABLE
  csmaConfig.ccaThreshold = CCA_THRESHOLD_DBM;

#if RAIL_IEEE802154_SUPPORTS_SIGNAL_IDENTIFIER
  if(RAIL_IEEE802154_SupportsSignalIdentifier(greenPowerHandle_g))
  {
      initStatus = RAIL_IEEE802154_ConfigCcaMode(greenPowerHandle_g, RAIL_IEEE802154_CCA_MODE_RSSI);
      if(initStatus != RAIL_STATUS_NO_ERROR)
      {
        while(1);
      }
  }
#endif

#endif //#if CCA_ENABLE

#if PTI_ENABLE
  initStatus = RAIL_ConfigPti(greenPowerHandle_g, &railPtiConfig);
  if(initStatus != RAIL_STATUS_NO_ERROR)
   {
       while(1);
   }

  RAIL_SetPtiProtocol(greenPowerHandle_g, RAIL_PTI_PROTOCOL_ZIGBEE);
  if(initStatus != RAIL_STATUS_NO_ERROR)
    {
        while(1);
    }
#endif //PTI_ENABLE

}
#endif //#if TEST_RADIO

#if TEST_RETENTION
void initBURAM(void)
{
  CMU_ClockSelectSet(cmuClock_EM4GRPACLK, cmuSelect_ULFRCO);
  CMU_ClockEnable(cmuClock_BURAM, true);
}
#endif //#if TEST_RETENTION

void initBURTC(void)
{
  CMU_ClockSelectSet(cmuClock_EM4GRPACLK, cmuSelect_ULFRCO);
  CMU_ClockEnable(cmuClock_BURTC, true);

  BURTC_Init_TypeDef burtcInit = BURTC_INIT_DEFAULT;
  burtcInit.compare0Top = true; // reset counter when counter reaches compare value
  burtcInit.em4comp = true;     // BURTC compare interrupt wakes from EM4 (causes reset)

  //burtcInit.start = false;
  BURTC_Init(&burtcInit);// Adds 15ms ? @EM1

  BURTC_CounterReset();
  BURTC_CompareSet(0, BURTC_IRQ_PERIOD_MS);

  BURTC_IntEnable(BURTC_IEN_COMP);    // compare match
  NVIC_EnableIRQ(BURTC_IRQn);
}

static void lowPowerInit(void)
{
  EMU_EM4Init_TypeDef em4Init = EMU_EM4INIT_DEFAULT;
  EMU_EM23Init_TypeDef em23Init = EMU_EM23INIT_DEFAULT;

  em4Init.retainUlfrco = true;
  em23Init.vScaleEM23Voltage = emuVScaleEM23_LowPower;

  // Initialize EM23 energy modes
  EMU_EM23Init(&em23Init);
  EMU_EM4Init(&em4Init);
}


void checkResetCause (void)
{
  uint32_t cause = RMU_ResetCauseGet();
  RMU_ResetCauseClear();

  // Print reset cause
  if (    (cause & EMU_RSTCAUSE_PIN)
      ||  (cause & EMU_RSTCAUSE_POR)
      ||  (cause & EMU_RSTCAUSE_SYSREQ))
  {
#if TEST_RETENTION
    BURAM->RET[0].REG = 0; // reset EM4 wakeup counter
#endif
  }
  else if (cause & EMU_RSTCAUSE_EM4)
  {
#if TEST_RETENTION
    BURAM->RET[0].REG += 1; // increment EM4 wakeup counter
#endif
  }
}


//                            Radio Functions                                 //
#if TEST_RADIO
void on_channel_config_change(RAIL_Handle_t rail_handle,
                                           const RAIL_ChannelConfigEntry_t *entry)
{

}

static void radioEventHandler(RAIL_Handle_t railHandle,
 RAIL_Events_t events)
{
  RAIL_Status_t rxStatus;

 // ... handle RAIL events, e.g. receive and transmit completion
  if(events & RAIL_EVENTS_TX_COMPLETION)
  {
#if TEST_EMULATED_NVM
    //Save FC in NVM
    saveLastFC = true;
#endif  //#if TEST_EMULATED_NVM

    if(rxAfterTx)
      {
        //Schedule RX
        rxStatus = RAIL_ScheduleRx(greenPowerHandle_g, RADIO_CHANNEL,&scheduleRxConfig, NULL);
        if(rxStatus != RAIL_STATUS_NO_ERROR)
          {
            while(1);
          }
      } else {
          shouldHibernate = true;
      }
  }

  if (events & RAIL_EVENTS_RX_COMPLETION) {
      shouldHibernate = true;
  }

  if (events & RAIL_EVENT_RX_SCHEDULED_RX_END) {
      shouldHibernate = true;
  }

  if (events & RAIL_EVENT_CAL_NEEDED) {
    calibrateRadio = true;
  }
}
#endif //#if TEST_RADIO


//                            ISR Functions                                   //
void BURTC_IRQHandler(void)
{
  BURTC_IntClear(BURTC_IF_COMP); // compare match
  txSequenceINitiated = false;//For EM2 purposes only
}

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void)
{
  //EM2 Trap to avoid bricking the device (Series 2 only)
  //Not useful for final code - only to allow DCI access
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(gpioPortB, 1, gpioModeInputPullFilter, 1);
  if(0 == GPIO_PinInGet(gpioPortB, 1))
  {
    while(1);
  } else {
    GPIO_PinModeSet(gpioPortB, 1, gpioModeDisabled, 0);
    CMU_ClockEnable(cmuClock_GPIO, false);
  }

  initBURTC();
#if TEST_RETENTION
  initBURAM();
#endif //#if TEST_RETENTION
  checkResetCause();
  lowPowerInit();

#if TEST_RADIO
  radioInitialize();
#endif

#if TEST_EMULATED_NVM
  nvmInit();
#endif
}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void)
{

#if TEST_RADIO
  volatile uint16_t bytesWritten;
  volatile RAIL_Status_t txstatus;
#endif //#if TEST_RADIO


#if TEST_EMULATED_NVM
  if(saveLastFC)
  {
    static uint16_t i = 0;
    //Test purposes, we look for an empty slot
    for (i = 0; i < (FLASH_PAGE_SIZE/4); ++i) {
        if(LAST_FLASH_PAGE[i] != 0xFFFFFFFF)
        {
            continue;
        } else {
            break;
        }
    }

    // Read the initial value in the cleared page
    uint32_t cleared_value = LAST_FLASH_PAGE[i];
    if(cleared_value != 0xFFFFFFFF)
    {
      while(1);
    }

    // Write the value into the 4th word of the Userdata portion of the flash
    MSC_Init();
    MSC_WriteWord((&LAST_FLASH_PAGE[i]), &frameCounterTestValue, 4);
    MSC_Deinit();

    // Read the written data from the flash location it was stored in
    uint32_t set_value = LAST_FLASH_PAGE[i];
    if(set_value != frameCounterTestValue)
      {
        while(1);
      }
    saveLastFC = false;
  }
#endif



#if TEST_RADIO

  if(calibrateRadio){
  //  RAIL_CalMask_t pendingCals = RAIL_GetPendingCal(greenPowerHandle_g);
  //
    calibrateRadio = false;
  //
  //  // Perform the necessary calibrations and don't save the results
  //  if (pendingCals & RAIL_CAL_TEMP_VCO) {
  //    RAIL_CalibrateTemp(greenPowerHandle_g);
  //  }
  //
  //  if (pendingCals & RAIL_CAL_TEMP_HFXO) {
  //    // Compensation step 1: wait for thermistor measurement
  //    RAIL_StartThermistorMeasurement(greenPowerHandle_g);
  //  }
  //
  //  if (pendingCals & RAIL_CAL_COMPENSATE_HFXO) {
  //    // Compensation step 2: compute and apply ppm correction
  //    RAIL_CalibrateHFXO(greenPowerHandle_g, NULL);
  //  }
  //
  //  if (pendingCals & RAIL_CAL_ONETIME_IRCAL) {
  //    RAIL_AntennaSel_t rfPath = RAIL_ANTENNA_AUTO;
  //    RAIL_Status_t retVal = RAIL_GetRfPath(greenPowerHandle_g, &rfPath);
  //
  //    if (retVal == RAIL_STATUS_NO_ERROR) {
  //      RAIL_CalibrateIrAlt(greenPowerHandle_g, NULL, rfPath);
  //    }
  //  }
  }

  if(!txSequenceINitiated && !calibrateRadio){

      bytesWritten = RAIL_WriteTxFifo(greenPowerHandle_g, txData, txDataLen, true);
      if(bytesWritten == 0)
        while(1);

#if CCA_ENABLE
      txstatus = RAIL_StartCcaCsmaTx(greenPowerHandle_g, RADIO_CHANNEL, TX_OPTIONS, &csmaConfig, NULL);
#else
      txstatus = RAIL_StartTx(greenPowerHandle_g, RADIO_CHANNEL, TX_OPTIONS, NULL);
#endif //#if CCA_ENABLE

      if(txstatus != 0)
        while(1);

      shouldHibernate = false;
      txSequenceINitiated = true;
  }

  CORE_DECLARE_IRQ_STATE;
  // Go critical to assess sleep decisions
  CORE_ENTER_CRITICAL();

  if(shouldSleep && (!saveLastFC)){
    txstatus = RAIL_Sleep(wakeupTime, &sleepAllowed);
    if (txstatus != RAIL_STATUS_NO_ERROR) {
      CORE_EXIT_CRITICAL();
      return;
    }
    if (sleepAllowed) {
        if(shouldHibernate)//Due to EM2 implementation, which has no reset
        {
            BURTC_Enable(true);//Enable next TX
            shouldHibernate = false;//Disable re-enabling the radio TX in case we were woken up in between
        }
      // Go to sleep
      EMU_EnterEM2(true);
    }
    // Wakeup and sync the RAIL timebase back up
    RAIL_Wake(0);
  }

  CORE_EXIT_CRITICAL();

#else// !#if TEST_RADIO
  if(shouldHibernate)
    {
      BURTC_Enable(true);//Set Period timer
      EMU_EnterEM4();
    }
  else
  {
      while(1);
  }
#endif//#if TEST_RADIO
}

void RAILCb_AssertFailed(RAIL_Handle_t railHandle, uint32_t errorCode)
{
  static const char* railErrorMessages[] = RAIL_ASSERT_ERROR_MESSAGES;
  const char *errorMessage = "Unknown";
  // If this error code is within the range of known error messages then use
  // the appropriate error message.
  if (errorCode < (sizeof(railErrorMessages) / sizeof(char*))) {
    errorMessage = railErrorMessages[errorCode];
  }

  while(1);
  //printf(errorMessage);
  // Reset the chip since an assert is a fatal error
  //NVIC_SystemReset();
}
