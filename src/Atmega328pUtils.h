// Helpers for working with Atmega328p.

// Cleaner setting of bits
#define BV(x) (1<<x)

namespace Atmega328p {

/// Possible prescaler values for timers.
enum TimerPrescalerValue {
  PSV_1,
  PSV_8,
  PSV_32,
  PSV_64,
  PSV_128,
  PSV_256,
  PSV_1024,
};

// Possible waveform generation modes for times.
enum WaveformGenerationMode {
  NORMAL,
  PWM_PHASE_CORRECT,
  PWM_FAST,
  CTC
};

/// Initializes timer 0 by setting waveform generation mode and prescaler.
///
/// This function assumes that neither Clock Select nor Waveform Generation bits
/// have not been touched yet. Note that timer 0 does not support all
/// Timer0PrescalerValue values. If illegal value is entered, prescaler is not
/// set to any value.
///
/// Parameter topAtOcra toggles using OCRA register values as the TOP for pulse
/// width modulation. It it only used if mode is either PWM_PHASE_CORRECT or
/// PWM_FAST.
///
/// \param prescalerValue
///    Requested prescaler value
///
/// \param mode
///    Waveform generation mode
///
/// \param topAtOcra
///    If TOP value should be set to OCRA register value
void initializeTimer0(TimerPrescalerValue prescalerValue,
                      WaveformGenerationMode mode,
                      bool topAtOcra);

/// Initializes timer 2 by setting waveform generation mode and prescaler.
///
/// This function assumes that neither Clock Select nor Waveform Generation bits
/// have not been touched yet.
///
/// Parameter topAtOcra toggles using OCRA register values as the TOP for pulse
/// width modulation. It it only used if mode is either PWM_PHASE_CORRECT or
/// PWM_FAST.
///
/// \param prescalerValue
///    Requested prescaler value
///
/// \param mode
///    Waveform generation mode
///
/// \param topAtOcra
///    If TOP value should be set to OCRA register value
void initializeTimer2(TimerPrescalerValue prescalerValue,
                      WaveformGenerationMode mode,
                      bool topAtOcra);

// Possible voltage references
enum VoltageReference {
  // Aref pin
  VREF_AREF,
  // Vcc with external capacitor
  VREF_VCC,
  // Internal 1.1 V with external capacitor
  VREF_INTERNAL_1_1V
};

// Sets analog voltage reference to requested type.
//
// This function assumes that Reference Selection bits have not been touched
// yet.
//
// \param reference
//    Requested voltage reference
void setVoltageReference(VoltageReference reference);

// Possible analog-to-digital converter prescaler values
enum AdcPrescalerValue {
  ADC_PSV_2,
  ADC_PSV_4,
  ADC_PSV_8,
  ADC_PSV_16,
  ADC_PSV_32,
  ADC_PSV_64,
  ADC_PSV_128
};

// Sets analog-to-digital converter prescaler value.
//
// This function assumes that ADV Prescaler Selection bits have not been touched
// yet.
//
// \param value
//    Requested prescaler value
void setAdcPrescalerValue(AdcPrescalerValue value);

} // namespace