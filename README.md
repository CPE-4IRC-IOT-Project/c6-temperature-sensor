# c6-temperature-sensor

Serveur BLE sur ESP32-C6 qui mesure une NTC sur `GPIO4` et publie la température.

## Fonctionnement rapide

1. Lecture ADC (multi-échantillons + filtrage) sur `GPIO4`.
2. Conversion tension -> résistance NTC -> température (équation Beta).
3. Publication BLE via service/characteristic custom.
4. Payload de température chiffré en AES-CCM (trame 16 octets).

Fichier principal: `main/main.c`.

## Format BLE publié

- Octet `0`: version
- Octet `1`: key id
- Octets `2..9`: nonce
- Octets `10..11`: température chiffrée (centi-degrés, int16 LE)
- Octets `12..15`: tag d'authentification

La clé partagée est `BLE_SEC_PSK` dans `main/main.c` (doit matcher le client).

## Build / Flash

```bash
. $HOME/esp/esp-idf/export.sh
idf.py set-target esp32c6
idf.py build
idf.py -p <PORT_C6_SENSOR> flash monitor
```

## Câblage

- `AO` du capteur NTC -> `GPIO4`
- `VCC` -> `3V3`
- `GND` -> `GND`
