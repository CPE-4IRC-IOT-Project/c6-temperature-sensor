# ESP32-C6 Temperature Sensor (ADC GPIO4)

Projet ESP-IDF minimal pour lire un module thermistance NTC (type KY-028/KY-013) connecte sur `GPIO4` d'une carte ESP32-C6.

Le code affiche:
- la valeur ADC brute (`raw`)
- la tension estimee en millivolts (`mV`) quand la calibration ADC est disponible
- la resistance NTC estimee (`ohm`)
- la temperature calculee (`°C`) avec l'equation Beta
- publie la temperature sur BLE dans un payload chiffre (AES-CCM, 16 octets)

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

## Chiffrement BLE applicatif

Le serveur BLE chiffre la temperature avant lecture/notification GATT.

Format du payload chiffre:
- Taille totale: `16` octets
- Octet `0`: version (`0x01`)
- Octet `1`: key id (`0x01`)
- Octets `2..9`: nonce (8 octets)
- Octets `10..11`: temperature chiffree (int16 little-endian en centi-degres)
- Octets `12..15`: tag d'authentification (4 octets)

La cle partagee est definie dans `main/main.c` (`BLE_SEC_PSK`) et doit etre identique cote client.

## Calibration NTC

Les constantes sont dans `main/main.c`:
- `NTC_SERIES_RESISTOR_OHM`
- `NTC_NOMINAL_RESISTANCE_OHM`
- `NTC_NOMINAL_TEMPERATURE_C`
- `NTC_BETA_COEFFICIENT`
- `NTC_SUPPLY_VOLTAGE_MV`
- `NTC_TO_GND`

Valeurs par defaut:
- NTC 10k a 25°C (profil standard B3950)
- Beta 3950
- resistance serie 10k

Si la temperature affichee est incoherente, ajuste d'abord `NTC_NOMINAL_RESISTANCE_OHM`, `NTC_NOMINAL_TEMPERATURE_C`, `NTC_BETA_COEFFICIENT` et `NTC_SERIES_RESISTOR_OHM` selon la reference exacte du module.

Si les logs montrent des sauts `raw` proches de `0` puis `4095` (ou `v_inst` proche de `0 mV` puis `3300 mV`), verifie en priorite:
- que la broche connectee est bien la sortie analogique `AO` du module (pas `DO`)
- que la masse est commune entre capteur et ESP32-C6
- qu'il n'y a pas de faux contact sur `GPIO4`
