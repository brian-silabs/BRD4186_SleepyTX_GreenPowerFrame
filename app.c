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

#include "sl_device_init_dpll.h"

#include "pa_conversions_efr32.h"

#include "em_msc.h"
#include "em_rmu.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_burtc.h"
#include "em_iadc.h"
#include "em_ldma.h"
#include "em_prs.h"
#include "em_letimer.h"

// Radio related tests
// Radio test master switch
#define TEST_RADIO                        1

#if TEST_RADIO
// Disable / Enable CCA assesment upon TX
#define CCA_ENABLE                        1
#define CCA_THRESHOLD_DBM                 (-75)
#define CCA_USE_PEAK                      0
#if CCA_USE_PEAK
#define TX_OPTIONS                        (RAIL_TX_OPTIONS_DEFAULT | RAIL_TX_OPTION_CCA_PEAK_RSSI)
#else
#define TX_OPTIONS                        RAIL_TX_OPTIONS_DEFAULT
#endif

#define RX_AFTER_TX                       1

// Disable / Enable PTI for Radio debugging
#define PTI_ENABLE                        1

#define RADIO_WAKEUP_TIME_US              800 //Estimated time in us required for Radio operations after EM2 wake up

#define TX_BUFFER_SIZE                    512
#define TX_MAX_PACKET_LENGTH              128

#define RADIO_CHANNEL                     (18)

#define RX_TO_TX_US                       20000//20ms
#define RX_WINDOW_US                      5000  //5ms

#endif//#if TEST_RADIO

// NVM / Simulated EEPROM test settings
// NVM test master switch
#define TEST_EMULATED_NVM                 0

#if TEST_EMULATED_NVM
// Using the last flash page as storage slot for FC
#define LAST_FLASH_PAGE                   ((uint32_t*)(FLASH_SIZE - FLASH_PAGE_SIZE))
// Should this test include a page erase upon startup
#define PERFRORM_PAGE_ERASE               0
#endif//#define TEST_EMULATED_NVM

// Data retention test during Hibernation master switch
#define TEST_RETENTION                    0

#define TEST_IADC                         1

#if TEST_IADC
#define USE_LETIMER_AS_SAMPLING_TRIGGER   1

// How many samples to capture
#define NUM_SAMPLES                       64

// Set CLK_ADC to 10MHz
#define CLK_SRC_ADC_FREQ                  20000000 // CLK_SRC_ADC

// Takes Errata IADC_E306 into account
#define CLK_ADC_FREQ_GAIN_4X              2500000 // CLK_ADC - 2.5MHz max in gain 4x
#define CLK_ADC_FREQ_GAIN_0P5X            10000000 // CLK_ADC - 10MHz max in 0.5x gain

// Number of scan channels
#define NUM_INPUTS                        6
#define IADC_LDMA_CHANNEL                 0

#define I_CONFIG                          1
#define U_CONFIG                          0

#if (USE_LETIMER_AS_SAMPLING_TRIGGER)
// ADC sample rate @2340.6Hz //256sps/s
#define SAMPLING_TIME_TICK                14u // top value for timer @ 32768Hz = CMU_ClockFreqGet(cmuClock_LETIMER0) / SAMPLING_FREQ_HZ
//#define LETIMER_TRIGGER_PRS_CHANNEL       1         //PRS channel 1 as trigger for LETIMER operations
#else
// Set IADC timer cycles
#define TIMER_CYCLES                      8545// expected 8545 to achieve the 2.4kHz sampling rate @20MHz src clock
#endif

/*
 * Specify the IADC input using the IADC_PosInput_t typedef.  This
 * must be paired with a corresponding macro definition that allocates
 * the corresponding ABUS to the IADC.  These are...
 *
 * GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AEVEN0_ADC0
 * GPIO->ABUSALLOC |= GPIO_ABUSALLOC_AODD0_ADC0
 * GPIO->BBUSALLOC |= GPIO_BBUSALLOC_BEVEN0_ADC0
 * GPIO->BBUSALLOC |= GPIO_BBUSALLOC_BODD0_ADC0
 * GPIO->CDBUSALLOC |= GPIO_CDBUSALLOC_CDEVEN0_ADC0
 * GPIO->CDBUSALLOC |= GPIO_CDBUSALLOC_CDODD0_ADC0
 *
 * ...for port A, port B, and port C/D pins, even and odd, respectively.
 */
/// ////////////////////////////////////////////////////////////////////////////

#define IADC_POS_INPUT_0_PORT_PIN         iadcPosInputPortBPin5;//EXP #3
#define IADC_POS_INPUT_0_BUS              BBUSALLOC
#define IADC_POS_INPUT_0_BUSALLOC         GPIO_BBUSALLOC_BODD0_ADC0

#define IADC_NEG_INPUT_0_PORT_PIN         iadcNegInputGnd;
#define IADC_NEG_INPUT_0_BUS              BBUSALLOC
#define IADC_NEG_INPUT_0_BUSALLOC         GPIO_BBUSALLOC_BODD0_ADC0

/// ////////////////////////////////////////////////////////////////////////////

#define IADC_POS_INPUT_1_PORT_PIN         iadcPosInputPortBPin1;//BUTTON 0
#define IADC_POS_INPUT_1_BUS              BBUSALLOC
#define IADC_POS_INPUT_1_BUSALLOC         GPIO_BBUSALLOC_BODD0_ADC0

#define IADC_NEG_INPUT_1_PORT_PIN         iadcNegInputGnd;
#define IADC_NEG_INPUT_1_BUS              BBUSALLOC
#define IADC_NEG_INPUT_1_BUSALLOC         GPIO_BBUSALLOC_BODD0_ADC0

/// ////////////////////////////////////////////////////////////////////////////

#define IADC_POS_INPUT_2_PORT_PIN         iadcPosInputPortBPin2;//LED 0
#define IADC_POS_INPUT_2_BUS              BBUSALLOC
#define IADC_POS_INPUT_2_BUSALLOC         GPIO_BBUSALLOC_BEVEN0_ADC0

#define IADC_NEG_INPUT_2_PORT_PIN         iadcNegInputGnd;
#define IADC_NEG_INPUT_2_BUS              BBUSALLOC
#define IADC_NEG_INPUT_2_BUSALLOC         GPIO_BBUSALLOC_BEVEN0_ADC0

/// ////////////////////////////////////////////////////////////////////////////

#define IADC_POS_INPUT_3_PORT_PIN         iadcPosInputPortAPin0;//EXP #5
#define IADC_POS_INPUT_3_BUS              ABUSALLOC
#define IADC_POS_INPUT_3_BUSALLOC         GPIO_ABUSALLOC_AEVEN0_ADC0

#define IADC_NEG_INPUT_3_PORT_PIN         iadcNegInputPortAPin5;//EXP #7
#define IADC_NEG_INPUT_3_BUS              ABUSALLOC
#define IADC_NEG_INPUT_3_BUSALLOC         GPIO_ABUSALLOC_AODD0_ADC0

/// ////////////////////////////////////////////////////////////////////////////

#define IADC_POS_INPUT_4_PORT_PIN         iadcPosInputPortAPin8;//EXP #12 - Do not use with default VCOM
#define IADC_POS_INPUT_4_BUS              ABUSALLOC
#define IADC_POS_INPUT_4_BUSALLOC         GPIO_ABUSALLOC_AEVEN0_ADC0

#define IADC_NEG_INPUT_4_PORT_PIN         iadcNegInputPortAPin9;//EXP #14 - Do not use with default VCOM
#define IADC_NEG_INPUT_4_BUS              ABUSALLOC
#define IADC_NEG_INPUT_4_BUSALLOC         GPIO_ABUSALLOC_AODD0_ADC0

/// ////////////////////////////////////////////////////////////////////////////

#define IADC_POS_INPUT_5_PORT_PIN         iadcPosInputPortAPin6;//EXP #11
#define IADC_POS_INPUT_5_BUS              ABUSALLOC
#define IADC_POS_INPUT_5_BUSALLOC         GPIO_ABUSALLOC_AEVEN0_ADC0

#define IADC_NEG_INPUT_5_PORT_PIN         iadcNegInputPortAPin7;//EXP #13
#define IADC_NEG_INPUT_5_BUS              ABUSALLOC
#define IADC_NEG_INPUT_5_BUSALLOC         GPIO_ABUSALLOC_AODD0_ADC0

/// ////////////////////////////////////////////////////////////////////////////

#define DEBUG_LETIMER_TRIGGER_GPIO        1

#define LETIMER_TRIGGER_GPIO_PORT         gpioPortB//LED 1
#define LETIMER_TRIGGER_GPIO_PIN          4//LED 1

#endif//#if TEST_IADC

// Number of 1 KHz ULFRCO clocks between BURTC interrupts
// Must fit into uint32_t
#define BURTC_IRQ_PERIOD_MS               5000// Will be 60s

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

#if TEST_IADC
// Globally declared LDMA link descriptor
LDMA_Descriptor_t descriptor;

// buffer to store IADC samples
uint32_t scanBuffer[NUM_SAMPLES] = {0xFF};
uint32_t iadcPrsChannel = 0;
LDMA_TransferCfg_t transferCfg = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_IADC0_IADC_SCAN);
#endif //#if TEST_IADC



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
static bool startedSampling = false;

// -----------------------------------------------------------------------------
//                          Function Definitions
// -----------------------------------------------------------------------------
//                             Init Functions                                 //

#if TEST_IADC
#if (USE_LETIMER_AS_SAMPLING_TRIGGER)
static void letimerInit(void)
{
  // Declare init struct
  LETIMER_Init_TypeDef init = LETIMER_INIT_DEFAULT;

  // Select LETimer0 clock to run off LFXO
  // Reference: EFR32xG22 RM, Figure 8.3
  CMU_ClockSelectSet(cmuClock_EM23GRPACLK, cmuSelect_LFRCO);
  // Enable LETimer0 clock
  CMU_ClockEnable(cmuClock_LETIMER0, true);

  // Initialize letimer to run in free running mode
  // Reference: EFR32xG22 RM, Section 18.3.2
  init.repMode = letimerRepeatFree;
  // Pulse output for PRS
  init.ufoa0 = letimerUFOAPulse;
  // Set frequency
  //init.topValue = CMU_ClockFreqGet(cmuClock_LETIMER0) / LETIMER_TRIGGER_FREQ_HZ;
  // Warning : Trig each Topvalue+1, so minus 1 is needed, but not done in LETIMER_Init
  init.topValue = SAMPLING_TIME_TICK-1; //CMU_ClockFreqGet(cmuClock_LETIMER0) / SAMPLING_FREQ_HZ;

  // Enable letimer
  //init.enable = true;
  init.debugRun = false;

  // Initialize free-running letimer
  LETIMER_Init(LETIMER0, &init);
}

static void initPrs(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_PRS, true);

  iadcPrsChannel = PRS_GetFreeChannel(prsTypeAsync);

  // LETIMER --------- PRS CHx --------> IADC0

  PRS_SourceAsyncSignalSet(iadcPrsChannel,
                           PRS_ASYNC_CH_CTRL_SOURCESEL_LETIMER0,
                           PRS_ASYNC_CH_CTRL_SIGSEL_LETIMER0CH0);
  // Select PRS channel 1 as trigger for IADC Single trigger
  PRS_ConnectConsumer(iadcPrsChannel,
                      prsTypeAsync,
                      prsConsumerIADC0_SCANTRIGGER);

#if DEBUG_LETIMER_TRIGGER_GPIO
  // STR : disable pin output
  GPIO_PinModeSet(LETIMER_TRIGGER_GPIO_PORT, LETIMER_TRIGGER_GPIO_PIN, gpioModePushPull,1);
  PRS_PinOutput(iadcPrsChannel, prsTypeAsync, LETIMER_TRIGGER_GPIO_PORT, LETIMER_TRIGGER_GPIO_PIN);
#endif//#if DEBUG_LETIMER_TRIGGER_GPIO
}
#endif//#if (USE_LETIMER_AS_SAMPLING_TRIGGER)

/**************************************************************************//**
 * @brief  IADC Initializer
 *****************************************************************************/
void initIADC (void)
{
  // Declare init structs
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;

  IADC_InitScan_t initScan = IADC_INITSCAN_DEFAULT;
  IADC_ScanTable_t initScanTable = IADC_SCANTABLE_DEFAULT; // Scan Table

  // Enable IADC clock
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_IADC0, true);

  // Reset IADC to reset configuration in case it has been modified
  IADC_reset(IADC0);

  // Configure IADC clock source for use while in EM2
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO); // 20MHz
  //CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_EM23GRPACLK);

#if (USE_LETIMER_AS_SAMPLING_TRIGGER)
  init.iadcClkSuspend0 = true;//Turn off clocks between scan acquisitions
#endif

  // Modify init structs and initialize
  init.warmup = iadcWarmupNormal;

  // Set the HFSCLK prescale value here
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

#if !(USE_LETIMER_AS_SAMPLING_TRIGGER)
  // Set timer cycles to configure sampling rate
  init.timerCycles = TIMER_CYCLES;
#endif

/// ////////////////////////////////////////////////////////////////////////////
  /*
     * Configuration 0 is used by both scan and single conversions by
     * default.  Use internal bandgap as the reference and specify the
     * reference voltage in mV.
     *
     * Resolution is not configurable directly but is based on the
     * selected oversampling ratio (osrHighSpeed), which defaults to
     * 2x and generates 12-bit results.
     */
    //I measurements
    initAllConfigs.configs[I_CONFIG].reference = iadcCfgReferenceInt1V2;
    initAllConfigs.configs[I_CONFIG].vRef = 1210;

    initAllConfigs.configs[I_CONFIG].osrHighSpeed = iadcCfgOsrHighSpeed32x;
    initAllConfigs.configs[I_CONFIG].analogGain = iadcCfgAnalogGain4x;

    // Divides CLK_SRC_ADC to set the CLK_ADC frequency
    // Default oversampling (OSR) is 2x, and Conversion Time = ((4 * OSR) + 2) / fCLK_ADC
    // Combined with the 2 cycle delay when switching input channels, total sample rate is 833ksps
    initAllConfigs.configs[I_CONFIG].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                       CLK_ADC_FREQ_GAIN_4X,
                                                                       0,
                                                                       iadcCfgModeNormal,
                                                                       init.srcClkPrescale);

/// ////////////////////////////////////////////////////////////////////////////

    //U measurements
    initAllConfigs.configs[U_CONFIG].reference = iadcCfgReferenceInt1V2;
    initAllConfigs.configs[U_CONFIG].vRef = 1210;

    initAllConfigs.configs[U_CONFIG].osrHighSpeed = iadcCfgOsrHighSpeed2x;
    initAllConfigs.configs[U_CONFIG].analogGain = iadcCfgAnalogGain0P5x;

    // Divides CLK_SRC_ADC to set the CLK_ADC frequency
    // Default oversampling (OSR) is 2x, and Conversion Time = ((4 * OSR) + 2) / fCLK_ADC
    // Combined with the 2 cycle delay when switching input channels, total sample rate is 833ksps
    initAllConfigs.configs[U_CONFIG].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                       CLK_ADC_FREQ_GAIN_0P5X,
                                                                       0,
                                                                       iadcCfgModeNormal,
                                                                       init.srcClkPrescale);

/// ////////////////////////////////////////////////////////////////////////////

  // Scan initialization
  // On every trigger, start conversion
  initScan.triggerAction = iadcTriggerActionOnce;

#if (USE_LETIMER_AS_SAMPLING_TRIGGER)
  // Set conversions to trigger from letimer/PRS
  initScan.triggerSelect = iadcTriggerSelPrs0PosEdge;
#else
  // Set conversions to trigger from IADC internal timer
  initScan.triggerSelect = iadcTriggerSelTimer;
#endif

  initScan.dataValidLevel = iadcFifoCfgDvl8;

  // Set alignment to the left 16 bits
  initScan.alignment = iadcAlignRight16;

  // Enable triggering of scan conversion
  //initScan.start = true;

  // Set to run in EM2
  initScan.fifoDmaWakeup = true;

  initScan.showId = true;

  // Configure entries in scan table
  // 0, 1 & 2 -> Single ended, U inputs
  // 3, 4 & 5 -> Differential , I inputs
  // Takes Errata IADC_E306 into account
  initScanTable.entries[0].posInput = IADC_POS_INPUT_0_PORT_PIN;//U1
  initScanTable.entries[0].negInput = IADC_NEG_INPUT_0_PORT_PIN;
  initScanTable.entries[0].includeInScan = true;
  initScanTable.entries[0].configId = U_CONFIG;

  initScanTable.entries[1].posInput = IADC_POS_INPUT_1_PORT_PIN;//U2
  initScanTable.entries[1].negInput = IADC_NEG_INPUT_1_PORT_PIN;
  initScanTable.entries[1].includeInScan = true;
  initScanTable.entries[1].configId = U_CONFIG;

  initScanTable.entries[2].posInput = IADC_POS_INPUT_2_PORT_PIN;//U3
  initScanTable.entries[2].negInput = IADC_NEG_INPUT_2_PORT_PIN;
  initScanTable.entries[2].includeInScan = true;
  initScanTable.entries[2].configId = U_CONFIG;

  initScanTable.entries[3].posInput = IADC_POS_INPUT_3_PORT_PIN;//I1
  initScanTable.entries[3].negInput = IADC_NEG_INPUT_3_PORT_PIN;
  initScanTable.entries[3].includeInScan = true;
  initScanTable.entries[3].configId = I_CONFIG;

  initScanTable.entries[4].posInput = IADC_POS_INPUT_4_PORT_PIN;//I2
  initScanTable.entries[4].negInput = IADC_NEG_INPUT_4_PORT_PIN;
  initScanTable.entries[4].includeInScan = true;
  initScanTable.entries[4].configId = I_CONFIG;

  initScanTable.entries[5].posInput = IADC_POS_INPUT_5_PORT_PIN;//I3
  initScanTable.entries[5].negInput = IADC_NEG_INPUT_5_PORT_PIN;
  initScanTable.entries[5].includeInScan = true;
  initScanTable.entries[5].configId = I_CONFIG;

  initScanTable.entries[6].posInput = iadcPosInputGnd;
  initScanTable.entries[6].negInput = iadcNegInputGnd;
  initScanTable.entries[6].includeInScan = true;
  initScanTable.entries[6].configId = U_CONFIG;

  initScanTable.entries[7].posInput = iadcPosInputGnd;
  initScanTable.entries[7].negInput = iadcNegInputGnd;
  initScanTable.entries[7].includeInScan = true;
  initScanTable.entries[7].configId = U_CONFIG;

  // Initialize IADC
  IADC_init(IADC0, &init, &initAllConfigs);

  // Initialize Scan
  IADC_initScan(IADC0, &initScan, &initScanTable);

  // Allocate the analog bus for ADC0 inputs
  GPIO->IADC_POS_INPUT_0_BUS |= IADC_POS_INPUT_0_BUSALLOC;
  GPIO->IADC_NEG_INPUT_0_BUS |= IADC_NEG_INPUT_0_BUSALLOC;

  GPIO->IADC_POS_INPUT_1_BUS |= IADC_POS_INPUT_1_BUSALLOC;
  GPIO->IADC_NEG_INPUT_1_BUS |= IADC_NEG_INPUT_1_BUSALLOC;

  GPIO->IADC_POS_INPUT_2_BUS |= IADC_POS_INPUT_2_BUSALLOC;
  GPIO->IADC_NEG_INPUT_2_BUS |= IADC_NEG_INPUT_2_BUSALLOC;

  GPIO->IADC_POS_INPUT_3_BUS |= IADC_POS_INPUT_3_BUSALLOC;
  GPIO->IADC_NEG_INPUT_3_BUS |= IADC_NEG_INPUT_3_BUSALLOC;

  GPIO->IADC_POS_INPUT_3_BUS |= IADC_POS_INPUT_3_BUSALLOC;
  GPIO->IADC_NEG_INPUT_3_BUS |= IADC_NEG_INPUT_3_BUSALLOC;

  GPIO->IADC_POS_INPUT_3_BUS |= IADC_POS_INPUT_3_BUSALLOC;
  GPIO->IADC_NEG_INPUT_3_BUS |= IADC_NEG_INPUT_3_BUSALLOC;
}

/**************************************************************************//**
 * @brief
 *   LDMA Initializer
 *
 * @param[in] buffer
 *   pointer to the array where ADC data will be stored.
 * @param[in] size
 *   size of the array
 *****************************************************************************/
void initLDMA(uint32_t *buffer, uint32_t size)
{
  // Declare LDMA init structs
  LDMA_Init_t init = LDMA_INIT_DEFAULT;

  // Enable LDMA clock branch
  CMU_ClockEnable(cmuClock_LDMA, true);

  // Initialize LDMA with default configuration
  LDMA_Init(&init);

  // Configure LDMA for transfer from IADC to memory
  // LDMA will loop continuously
//transferCfg = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_IADC0_IADC_SCAN);

  // Set up descriptors for buffer transfer
  descriptor = (LDMA_Descriptor_t)LDMA_DESCRIPTOR_LINKREL_P2M_WORD(&IADC0->SCANFIFODATA, buffer, size, 0);

  // Set descriptor to loop NUM_SAMPLES times and run continuously
  descriptor.xfer.decLoopCnt = 0;
  descriptor.xfer.xferCnt = NUM_SAMPLES;
  descriptor.xfer.blockSize = ldmaCtrlBlockSizeUnit8;

  // Interrupt after transfer complete
  descriptor.xfer.doneIfs = 1;
  descriptor.xfer.ignoreSrec = 1;

  // Start transfer, LDMA will sample the IADC NUM_SAMPLES time, and then interrupt
  //LDMA_StartTransfer(IADC_LDMA_CHANNEL, (void*)&transferCfg, (void*)&descriptor);
}
#endif //#if TEST_IADC

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
  //burtcInit.em4comp = true;     // BURTC compare interrupt wakes from EM4 (causes reset)

  burtcInit.start = false;
  BURTC_Init(&burtcInit);// Adds 15ms ? @EM1


  //BURTC_CounterReset();//Starts BURTC !
  //BURTC->CNT = 0U;
  BURTC_CompareSet(0, BURTC_IRQ_PERIOD_MS);

  BURTC_IntEnable(BURTC_IEN_COMP);    // compare match
  NVIC_EnableIRQ(BURTC_IRQn);
}

static void lowPowerInit(void)
{
  //EMU_EM4Init_TypeDef em4Init = EMU_EM4INIT_DEFAULT;
  EMU_EM23Init_TypeDef em23Init = EMU_EM23INIT_DEFAULT;

  //em4Init.retainUlfrco = true;
  em23Init.vScaleEM23Voltage = emuVScaleEM23_LowPower;

  // Initialize EM23 energy modes
  EMU_EM23Init(&em23Init);
  //EMU_EM4Init(&em4Init);
}

static void iadcStart(void)
{
  if( !startedSampling )
  {

      IADC_command(IADC0, iadcCmdStartScan);

      // Start timer
      LETIMER_Enable(LETIMER0, true);

      // Start LDMA
      LDMA_StartTransfer(IADC_LDMA_CHANNEL, (void*)&transferCfg, (void*)&descriptor);
      // One of the desc has doneIfs set, so interrupt should be activated on the channel :
      LDMA->IEN |= 1UL << (uint8_t)IADC_LDMA_CHANNEL; // Allow  interrupt to be fired by any desc
      // Set flag to indicate sampling is occuring.
      startedSampling = true;
  }
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

void RAILCb_AssertFailed(RAIL_Handle_t railHandle, uint32_t errorCode)
{
  static const char* railErrorMessages[] = RAIL_ASSERT_ERROR_MESSAGES;
  const char *errorMessage = "Unknown";
  // If this error code is within the range of known error messages then use
  // the appropriate error message.
  if (errorCode < (sizeof(railErrorMessages) / sizeof(char*))) {
    errorMessage = railErrorMessages[errorCode];//If error is RAIL_ASSERT_FAILED_RTCC_SYNC_STALE_DATA
                                                //Ensure no call to EMU_EnterEM2 has been made outside the
  }

  while(1);
  //printf(errorMessage);
  // Reset the chip since an assert is a fatal error
  //NVIC_SystemReset();
}
#endif //#if TEST_RADIO

#if TEST_IADC
/**************************************************************************//**
 * @brief  LDMA Handler
 *****************************************************************************/
void LDMA_IRQHandler(void)
{
  // Clear interrupt flags
  LDMA_IntClear(LDMA_IF_DONE0);
}
#endif //#if TEST_IADC

//                            ISR Functions                                   //
void BURTC_IRQHandler(void)
{
  BURTC_IntClear(BURTC_IF_COMP); // compare match
  BURTC_Stop();//Make it one shot in EM2
#if TEST_RADIO
  txSequenceINitiated = false;//For EM2 purposes only
#else
  shouldHibernate = true;
#endif
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

#if TEST_IADC

  //Init PRS
  initPrs();

  // Initialize the IADC
  initIADC();

  // Initialize LDMA
  initLDMA(scanBuffer, NUM_SAMPLES);

#if (USE_LETIMER_AS_SAMPLING_TRIGGER)
  //Init LETIMER
  letimerInit();

  iadcStart();

#else
  // IADC single already enabled; must enable timer block in order to trigger
    IADC_command(IADC0, iadcCmdEnableTimer);
#endif//#if (USE_LETIMER_AS_SAMPLING_TRIGGER)
#endif//#if TEST_IADC

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
            BURTC_CounterReset();//Enable next TX
            shouldHibernate = false;//Disable re-enabling the radio TX in case we were woken up in between
        }

      //From ref man 9.4.3.1:
      // The DPLL is disabled automatically when entering EM2, EM3, or EM4. Note that disabling the DPLL will not automatically turn off the
      // reference clock. The CLKSEF field in CMU_DPLLREFCLKCTRL must be set to DISABLED before entering EM2 or the selected
      // REFCLK may continue to run in EM2.

      //Disable PLL REF
      CMU_CLOCK_SELECT_SET(SYSCLK, FSRCO);
      CMU_CLOCK_SELECT_SET(EM01GRPACLK, FSRCO);//Necessary ?
      CMU_CLOCK_SELECT_SET(EM01GRPCCLK, FSRCO);//Necessary ?

      CMU_CLOCK_SELECT_SET(DPLLREFCLK, DISABLED);
      //At this point the MCU is ready for sleep with HCLK on FSRCO
      //Need to check with the team if that is the best config

      // Go to sleep
      EMU_EnterEM2(true);

      //Enable PLL REF
      CMU_CLOCK_SELECT_SET(DPLLREFCLK, HFXO);
      CMU_CLOCK_SELECT_SET(SYSCLK, HFRCODPLL);
      CMU_CLOCK_SELECT_SET(EM01GRPACLK, HFRCODPLL);//Necessary ?
      CMU_CLOCK_SELECT_SET(EM01GRPCCLK, HFRCODPLL);//Necessary ?

    }
    // Wakeup and sync the RAIL timebase back up
    RAIL_Wake(0);
  }

  CORE_EXIT_CRITICAL();

#else// !#if TEST_RADIO
  CORE_DECLARE_IRQ_STATE;
  // Go critical to assess sleep decisions
  CORE_ENTER_CRITICAL();

  if(shouldHibernate)//When no radio is used, shouldHibernate is allowing to restart the BURTC
    {
      BURTC_CounterReset();//Set Period timer
      shouldHibernate = false;
    }

    //From ref man 9.4.3.1:
    // The DPLL is disabled automatically when entering EM2, EM3, or EM4. Note that disabling the DPLL will not automatically turn off the
    // reference clock. The CLKSEF field in CMU_DPLLREFCLKCTRL must be set to DISABLED before entering EM2 or the selected
    // REFCLK may continue to run in EM2.

    //Disable PLL REF
    CMU_CLOCK_SELECT_SET(SYSCLK, FSRCO);
    CMU_CLOCK_SELECT_SET(EM01GRPACLK, FSRCO);//Necessary ?
    CMU_CLOCK_SELECT_SET(EM01GRPCCLK, FSRCO);//Necessary ?

    CMU_CLOCK_SELECT_SET(DPLLREFCLK, DISABLED);
    //At this point the MCU is ready for sleep with HCLK on FSRCO
    //Need to check with the team if that is the best config

    // Go to sleep
    EMU_EnterEM2(true);

    //Enable PLL REF
    CMU_CLOCK_SELECT_SET(DPLLREFCLK, HFXO);
    CMU_CLOCK_SELECT_SET(SYSCLK, HFRCODPLL);
    CMU_CLOCK_SELECT_SET(EM01GRPACLK, HFRCODPLL);//Necessary ?
    CMU_CLOCK_SELECT_SET(EM01GRPCCLK, HFRCODPLL);//Necessary ?

    CORE_EXIT_CRITICAL();
#endif//#if TEST_RADIO
}
