// ESP32 Torque Estimator for Exoskeleton Joint
// -------------------------------------------
// * Measures two phase currents via INA240 differential amplifiers and low‑side shunts
// * Executes Clarke–Park transforms to obtain Iq
// * Multiplies by motor torque constant, gear ratio and efficiency → joint torque estimate
// * Sends data over UART every control cycle (10 kHz)
//
// Hardware assumed
//   – ESP32‑S3 or S2 (ADC1 available on GPIO32/33)
//   – Two 1 mΩ low‑side shunt resistors
//   – Two INA240A1 (gain = 20 V/V) amps feeding the ADC
//   – Gate driver + MOSFET bridge driven by MCPWM (not shown)
//   – Quadrature encoder or Hall sensors to provide rotor electrical angle
//
// Build with ESP‑IDF v5.x  ×  “idf.py -p PORT flash monitor”

#include <stdio.h>
#include <math.h>
#include "driver/adc.h"
#include "driver/mcpwm_prelude.h"
#include "esp_timer.h"
#include "esp_log.h"

// ---------------- Motor / Gear parameters ----------------
#define K_TORQUE      0.231f     // N·m per amp (231 mNm/A)
#define GEAR_RATIO    50.0f      // output / motor
#define GEAR_EFF      0.92f      // constant efficiency (planetary)

// ---------------- Sensing chain --------------------------
#define SHUNT_R       0.001f     // Ω (1 mΩ)
#define INA_GAIN      20.0f      // INA240 gain
#define V_REF         3.3f       // ADC reference
#define ADC_MAX       4095.0f    // 12‑bit ADC

// ADC channels – GPIO32/33 (ADC1_CH4/CH5)
#define ADC_PHASE_A   ADC1_CHANNEL_4
#define ADC_PHASE_B   ADC1_CHANNEL_5

// Control loop frequencies
#define PWM_FREQ_HZ       20000
#define CONTROL_FREQ_HZ   10000   // 10 kHz torque update

static const char *TAG = "torque";

static float offset_a = 0.0f, offset_b = 0.0f;

// Forward declaration – user must implement with encoder or Hall sensors
float get_electrical_angle_rad(void);

// Convert raw ADC count to phase current in amps
static inline float raw_to_current(uint32_t raw)
{
    float v_adc = (raw / ADC_MAX) * V_REF;
    float v_shunt = v_adc / INA_GAIN;
    return v_shunt / SHUNT_R;
}

// Offset calibration with PWM disabled
static void calibrate_offsets(void)
{
    const int N = 1024;
    uint64_t sum_a = 0, sum_b = 0;
    for (int i = 0; i < N; ++i) {
        sum_a += adc1_get_raw(ADC_PHASE_A);
        sum_b += adc1_get_raw(ADC_PHASE_B);
    }
    offset_a = (float)sum_a / N;
    offset_b = (float)sum_b / N;
    ESP_LOGI(TAG, "Offsets: A=%.1f  B=%.1f", offset_a, offset_b);
}

// High‑frequency FOC/tau ISR (runs CONTROL_FREQ_HZ)
static void IRAM_ATTR foc_isr(void *arg)
{
    // 1. Sample phase currents (mid‑PWM)
    uint32_t raw_a = adc1_get_raw(ADC_PHASE_A);
    uint32_t raw_b = adc1_get_raw(ADC_PHASE_B);
    float I_a = raw_to_current(raw_a - offset_a);
    float I_b = raw_to_current(raw_b - offset_b);

    // 2. Clarke transform (3‑phase → αβ)
    float I_alpha = I_a;
    float I_beta  = (I_a + 2.0f * I_b) * 0.57735027f;   // 1/√3

    // 3. Park transform using rotor angle
    float theta_e = get_electrical_angle_rad();
    float sin_t, cos_t;
    sincosf(theta_e, &sin_t, &cos_t);
    float I_q = -I_alpha * sin_t + I_beta * cos_t;

    // 4. Torque estimate
    float tau_motor = K_TORQUE * I_q;
    float tau_joint = tau_motor * GEAR_RATIO * GEAR_EFF;

    // 5. (Optional) transmit or log
    uart_write_bytes(UART_NUM_0, (const char *)&tau_joint, sizeof(tau_joint));
}

void app_main(void)
{
    // ------------- ADC config ----------------
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_PHASE_A, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC_PHASE_B, ADC_ATTEN_DB_11);

    calibrate_offsets();

    // ------------- MCPWM setup (motor drive) ------------
    //  (left out – drive your 3‑phase bridge as usual)

    // ------------- High‑freq timer for ISR --------------
    const esp_timer_create_args_t t_args = {
        .callback = foc_isr,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_ISR,
        .name = "foc" };
    esp_timer_handle_t foc_timer;
    esp_timer_create(&t_args, &foc_timer);
    esp_timer_start_periodic(foc_timer, 1000000 / CONTROL_FREQ_HZ); // in µs

    ESP_LOGI(TAG, "Torque estimator running @%d Hz", CONTROL_FREQ_HZ);
}

//----------------------------------------------------------------
// Dummy electrical‑angle provider – replace with real encoder code
//----------------------------------------------------------------
#include "esp_system.h"
float get_electrical_angle_rad(void)
{
    static float theta = 0.0f;
    theta += 0.001f;               // placeholder ramp
    if (theta > 2*M_PI) theta -= 2*M_PI;
    return theta;
}

