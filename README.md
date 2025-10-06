# EML_NB_Logger
Small Narrowband IoT logger for river level and rainfall measurements

NB-IoT vs LTE-M: flip USE_NBIOT (if your MKRNB core exposes setRadioAccessTechnology — some versions don’t; if absent, the network will choose based on availability/APN).

PSM/eDRX: the u-blox modem can be tuned via AT (PSM timers, eDRX). MKRNB doesn’t expose all knobs; you can still send raw AT commands with nbAccess if you want to squeeze more battery life later.

TLS: once you’re happy, switch to port 8883 and use ArduinoBearSSL with MqttClient’s secure variant (RAM is tighter but OK on MKR).

Analog scaling: set ADS1115 gain to match your sensor’s max voltage for best resolution (e.g., GAIN_TWOTHIRDS, GAIN_ONE, etc.).

Long cables: add TVS diodes and/or an opto-isolator on the rain input if cables run outdoors.

Power: with hourly uploads and deep sleep, a 2000 mAh cell will last a long time; add a small 2–5 W solar for near-infinite runtime.
