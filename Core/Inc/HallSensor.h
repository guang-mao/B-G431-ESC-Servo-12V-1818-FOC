#ifndef __HALLSENSOR_H_
#define __HALLSENSOR_H_
#include "main.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_gpio.h"
#include <stdbool.h>

/** Revolutions Per Minute: 1 Hz is 60 RPM */
#define U_RPM									60
/** Tenth of Hertz: 1 Hz is 10 01Hz */
#define U_01HZ								10

#define SPEED_UNIT						U_01HZ

#define HALL_SPEED_FIFO_SIZE  ((uint8_t)64)

/* HALL SENSORS PLACEMENT ----------------------------------------------------*/
#define DEGREES_120						0u
#define DEGREES_60						1u

/** @brief Not initialized pointer */
#define MC_NULL    (void *)(0x0)
	
/**
  * @brief  SpeednPosFdbk  handles definitions of mechanical and electrical speed, mechanical acceleration, mechanical and electrical angle and all
  *                        constants and scale values for a reliable measure and computation in appropriated unit.
  */
typedef struct
{

  uint8_t bSpeedErrorNumber;          /*!< Number of time the average mechanical speed is not valid. */
  uint8_t bElToMecRatio;              /*!< Coefficient used to transform electrical to mechanical quantities and viceversa.
                                           It usually coincides with motor pole pairs number. */
  uint8_t SpeedUnit;                  /*!< The speed unit value is defined into mc_stm_types.h by [SPEED_UNIT](measurement_units.md) in tenth of Hertz.*/
  uint8_t bMaximumSpeedErrorsNumber;  /*!< Maximum value of not valid speed measurements before an error is reported.*/
  int16_t hElAngle;                   /*!< Estimated electrical angle reported by the implemented speed and position method. */
  int16_t hMecAngle;                  /*!< Instantaneous measure of rotor mechanical angle. */
  int32_t wMecAngle;                  /*!< Mechanical angle frame based on coefficient #bElToMecRatio. */
  int16_t hAvrMecSpeedUnit;           /*!< Average mechanical speed expressed in the unit defined by [SPEED_UNIT](measurement_units.md). */
  int16_t hElSpeedDpp;                /*!< Instantaneous electrical speed expressed in Digit Per control Period ([dpp](measurement_units.md)), expresses
                                           the angular speed as the variation of the electrical angle. */
  int16_t InstantaneousElSpeedDpp;    /*!< Instantaneous computed electrical speed, expressed in [dpp](measurement_units.md). */
  int16_t hMecAccelUnitP;             /*!< Average mechanical acceleration expressed in the unit defined by #SpeedUnit,
                                           only reported with encoder implementation */
  uint16_t hMaxReliableMecSpeedUnit;  /*!< Maximum value of measured mechanical speed that is considered to be valid.
                                           Expressed in the unit defined by [SPEED_UNIT](measurement_units.md). */
  uint16_t hMinReliableMecSpeedUnit;  /*!< Minimum value of measured mechanical speed that is considered to be valid.
                                           Expressed in the unit defined by [SPEED_UNIT](measurement_units.md).*/
  uint16_t hMaxReliableMecAccelUnitP; /*!< Maximum value of measured acceleration that is considered to be valid.
                                           Constant value equal to 65535, expressed in the unit defined by [SPEED_UNIT](measurement_units.md). */
  uint16_t hMeasurementFrequency;     /*!< Frequency at which the user will request a measurement of the rotor electrical
                                           angle. Expressed in PWM_FREQ_SCALING * Hz. */
  uint32_t DPPConvFactor;             /*!< Conversion factor (65536/#PWM_FREQ_SCALING) used to convert measured speed
                                           from the unit defined by [SPEED_UNIT](measurement_units.md) to [dpp](measurement_units.md). */


} SpeednPosFdbk_Handle_t;

/**
  * @brief HALL component parameters definition
  *
  *  <Type @p type represents a thing that needs to be detailed more. Additional details
  * are provided in the detailed section of the doxygen comment block.
  *
  * The brief line should be brief and light. It should avoid useless repetitions and expression such as
  * "the CCC_Type_t type...". Expressions like "This type ..." are tolerated especially for key types
  * (usually structures) where we may want ot be more formal.
  *
  * In general: be direct, avoid the obvious, tell the hidden.>
  */

typedef struct
{
  SpeednPosFdbk_Handle_t _Super;
  /* SW Settings */
  uint8_t  SensorPlacement; /*!< Define here the mechanical position of the sensors
                             with reference to an electrical cycle.
                             Allowed values are: DEGREES_120 or DEGREES_60.*/

  int16_t  PhaseShift;  /*!< Define here in s16degree the electrical phase shift
                             between the low to high transition of signal H1 and
                             the maximum of the Bemf induced on phase A.*/

  uint16_t SpeedSamplingFreqHz; /*!< Frequency (Hz) at which motor speed is to
                             be computed. It must be equal to the frequency
                             at which function SPD_CalcAvrgMecSpeedUnit
                             is called.*/

  uint8_t  SpeedBufferSize; /*!< Size of the buffer used to calculate the average
                             speed. It must be less than 18.*/

  /* HW Settings */
  uint32_t TIMClockFreq; /*!< Timer clock frequency express in Hz.*/

  TIM_TypeDef *TIMx;    /*!< Timer used for HALL sensor management.*/

  GPIO_TypeDef *H1Port;
  /*!< HALL sensor H1 channel GPIO input port (if used,
       after re-mapping). It must be GPIOx x= A, B, ...*/

  uint32_t  H1Pin;      /*!< HALL sensor H1 channel GPIO output pin (if used,
                             after re-mapping). It must be GPIO_Pin_x x= 0, 1,
                             ...*/

  GPIO_TypeDef *H2Port;
  /*!< HALL sensor H2 channel GPIO input port (if used,
       after re-mapping). It must be GPIOx x= A, B, ...*/

  uint32_t  H2Pin;      /*!< HALL sensor H2 channel GPIO output pin (if used,
                             after re-mapping). It must be GPIO_Pin_x x= 0, 1,
                             ...*/

  GPIO_TypeDef *H3Port;
  /*!< HALL sensor H3 channel GPIO input port (if used,
       after re-mapping). It must be GPIOx x= A, B, ...*/

  uint32_t H3Pin;      /*!< HALL sensor H3 channel GPIO output pin (if used,
                             after re-mapping). It must be GPIO_Pin_x x= 0, 1,
                             ...*/

  uint8_t ICx_Filter;               /*!< Input Capture filter selection */

  bool SensorIsReliable;            /*!< Flag to indicate a wrong configuration
                                         of the Hall sensor signanls.*/

  volatile bool RatioDec;           /*!< Flag to avoid consecutive prescaler
                                         decrement.*/
  volatile bool RatioInc;           /*!< Flag to avoid consecutive prescaler
                                         increment.*/
  volatile uint8_t FirstCapt;      /*!< Flag used to discard first capture for
                                         the speed measurement*/
  volatile uint8_t BufferFilled;   /*!< Indicate the number of speed measuremt
                                         present in the buffer from the start.
                                         It will be max bSpeedBufferSize and it
                                         is used to validate the start of speed
                                         averaging. If bBufferFilled is below
                                         bSpeedBufferSize the instantaneous
                                         measured speed is returned as average
                                         speed.*/
  volatile uint8_t OVFCounter;     /*!< Count overflows if prescaler is too low
                                         */

  int32_t SensorPeriod[HALL_SPEED_FIFO_SIZE];/*!< Holding the last period captures */

  uint8_t SpeedFIFOIdx;/*!< Pointer of next element to be stored in
                                         the speed sensor buffer*/

  int32_t  ElPeriodSum; /* Period accumulator used to speed up the average speed computation*/

  int16_t PrevRotorFreq; /*!< Used to store the last valid rotor electrical
                               speed in dpp used when HALL_MAX_PSEUDO_SPEED
                               is detected */
  int8_t Direction;          /*!< Instantaneous direction of rotor between two
                               captures*/

  int16_t AvrElSpeedDpp; /*!< It is the averaged rotor electrical speed express
                               in s16degree per current control period.*/

  uint8_t HallState;     /*!< Current HALL state configuration */

  int16_t DeltaAngle;    /*!< Delta angle at the Hall sensor signal edge between
                               current electrical rotor angle of synchronism.
                               It is in s16degrees.*/
  int16_t MeasuredElAngle;/*!< This is the electrical angle  measured at each
                               Hall sensor signal edge. It is considered the
                               best measurement of electrical rotor angle.*/

  int16_t CompSpeed;     /*!< Speed compensation factor used to syncronize
                               the current electrical angle with the target
                               electrical angle. */

  uint16_t HALLMaxRatio; /*!< Max TIM prescaler ratio defining the lowest
                             expected speed feedback.*/
  uint16_t SatSpeed;     /*!< Returned value if the measured speed is above the
                             maximum realistic.*/
  uint32_t PseudoFreqConv;/*!< Conversion factor between time interval Delta T
                             between HALL sensors captures, express in timer
                             counts, and electrical rotor speed express in dpp.
                             Ex. Rotor speed (dpp) = wPseudoFreqConv / Delta T
                             It will be ((CKTIM / 6) / (SAMPLING_FREQ)) * 65536.*/

  uint32_t MaxPeriod;  /*!< Time delay between two sensor edges when the speed
                             of the rotor is the minimum realistic in the
                             application: this allows to discriminate too low
                             freq for instance.
                             This period shoud be expressed in timer counts and
                             it will be:
                             wMaxPeriod = ((10 * CKTIM) / 6) / MinElFreq(0.1Hz).*/

  uint32_t MinPeriod;
  /*!< Time delay between two sensor edges when the speed
       of the rotor is the maximum realistic in the
       application: this allows discriminating glitches
       for instance.
       This period shoud be expressed in timer counts and
       it will be:
       wSpeedOverflow = ((10 * CKTIM) / 6) / MaxElFreq(0.1Hz).*/

  uint16_t HallTimeout;/*!< Max delay between two Hall sensor signal to assert
                             zero speed express in milliseconds.*/

  uint16_t OvfFreq;   /*!< Frequency of timer overflow (from 0 to 0x10000)
                             it will be: hOvfFreq = CKTIM /65536.*/
  uint16_t PWMNbrPSamplingFreq; /*!< Number of current control periods inside
                             each speed control periods it will be:
                             (hMeasurementFrequency / hSpeedSamplingFreqHz) - 1.*/
  uint8_t PWMFreqScaling; /*!< Scaling factor to allow to store a PWMFrequency greater than 16 bits */

  bool HallMtpa; /* if true at each sensor toggling, the true angle is set without ramp*/

} HALL_Handle_t;

void *HALL_TIMx_UP_IRQHandler(void *pHandleVoid);
void *HALL_TIMx_CC_IRQHandler(void *pHandleVoid);
void HALL_Init(HALL_Handle_t *pHandle);
void HALL_Clear(HALL_Handle_t *pHandle);
int16_t HALL_CalcElAngle(HALL_Handle_t *pHandle);
bool HALL_CalcAvrgMecSpeedUnit(HALL_Handle_t *pHandle, int16_t *hMecSpeedUnit);

extern HALL_Handle_t HALL_M1;
#endif
