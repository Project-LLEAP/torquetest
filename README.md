# ESP32 Torque Estimator

Uses ESP-IDF

* Measures two phase currents via INA240 differential amplifiers and low‑side shunts
* Executes Clarke–Park transforms to obtain Iq
* Multiplies by motor torque constant, gear ratio and efficiency → joint torque estimate
* Sends data over UART every control cycle (10 kHz)


Hardware assumed
  * ESP32
  * Two 1 mΩ low‑side shunt resistors
  * Two INA240A1 (gain = 20 V/V or 50V/V) amps feeding the ADC
  * Gate driver + MOSFET bridge driven by MCPWM (not shown)
  * Capacitive Encoder 