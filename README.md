# ESP32-C6 Temperature Sensor (ADC GPIO4)

Projet ESP-IDF minimal pour lire un module thermistance NTC (type KY-028/KY-013) connecte sur `GPIO4` d'une carte ESP32-C6.

Le code affiche:
- la valeur ADC brute (`raw`)
- la tension estimee en millivolts (`mV`) quand la calibration ADC est disponible
- la resistance NTC estimee (`ohm`)
- la temperature calculee (`°C`) avec l'equation Beta

## Sources (exemples officiels)

- ADC One-shot mode (ESP-IDF): https://docs.espressif.com/projects/esp-idf/en/stable/esp32c6/api-reference/peripherals/adc_oneshot.html
- GPIO summary ESP32-C6 (GPIO4 supporte ADC1_CH4): https://docs.espressif.com/projects/esp-idf/en/stable/esp32c6/api-reference/peripherals/gpio.html

## Cablage (sortie analogique)

- Sortie analogique capteur -> `GPIO4`
- `VCC` capteur -> `3V3`
- `GND` capteur -> `GND`

Note: `GPIO4` est aussi une broche de strapping. Evite une forte contrainte de niveau au demarrage (pas de tirage fort permanent vers GND/VCC).

## Build / Flash / Monitor

```bash
cd c6-temperature-sensor
. $HOME/esp/esp-idf/export.sh
idf.py set-target esp32c6
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

Dans le monitor serie, tu verras des lignes du type:

```text
I (1234) c6-temp-adc: GPIO4 raw=1031 voltage=1030 mV ntc=4545 ohm temp=44.83 C
```

## Calibration NTC

Les constantes sont dans `main/main.c`:
- `NTC_SERIES_RESISTOR_OHM`
- `NTC_NOMINAL_RESISTANCE_OHM`
- `NTC_NOMINAL_TEMPERATURE_C`
- `NTC_BETA_COEFFICIENT`
- `NTC_SUPPLY_VOLTAGE_MV`
- `NTC_TO_GND`

Valeurs par defaut:
- NTC 2.7k a 21.5°C (calibration actuelle du projet)
- Beta 3950
- resistance serie 10k

Si la temperature affichee est incoherente, ajuste d'abord `NTC_BETA_COEFFICIENT` et `NTC_SERIES_RESISTOR_OHM` selon la reference exacte du module.
