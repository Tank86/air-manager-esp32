# Code pour Purificateur d'air (Wemos-s2, Wemos-c3, Wemos D1-32)

Pour purificateur d'air fait maison, idée originale des frères Poulain
https://www.youtube.com/watch?v=WbyZMVf7Pek

Modifiée pour un style rond et moins "mastoc".

Ajout d'électronique pour le domotiser avec un mode automatique et ainsi qu'un mode fonctionnant avec HomeAssistant (ou tout autre domotique mqtt)
L'electronique permet de piloter/moduler également la vitesse du moteur.

<p align="center">
 <img src="/doc/PurificateurRaw.png" width="200"/>
</p>

## Liens des composants:
 - Filtre charbon : https://amzn.to/3bw7p8R
 - Filtre HEPA 13 : https://amzn.to/3HPSuT1
 - Turbine Variable : https://amzn.to/3yemHH5
 - Tissu 3D : https://lfp.yt/tissu
 - Contreplaqué
 - Tiges filletés M6
 - Ecrou
 - Rondelles

## Libraries utilisées:
- Bsec 2 (avec l'ajout du support des esp32-s2 et esp32-c3)
- MCP40xx [https://github.com/dzalf/MCP40xx_Digital_Potentiometer]
- home assistant integration (My Fork for Light support and Fan icon)[https://github.com/Tank86/arduino-home-assistant]
- FastLed
 
## Capteurs:
- Capteur de particules : https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y1010au_appl_e.pdf
- Capteur de qualité de l'air: https://www.adafruit.com/product/3660
- Led WS2818: https://www.adafruit.com/product/1138


## Valeurs remontées
Les valeurs sont remontée toutes les 5 minutes afin de ne pas noyer la communication mqtt.

- Temperature (°C)
- Humidité  (%)
- Pression atmosphérique (mBar)
- co2 equivalent (ppm)
- IAQ (Indice de qualité de l'air)
- IAQ Accuracy
- Breath VOC (composé organic volatile) (ppb)
- PM 2.5 (dust sensor) (µg/m3)


## Mode automatique/Manuel
Le code démarre par defaut avec le mode automatique d'activée, la régulation se fait en fonction des valeurs de PM2.5 et d'IAQ
Le mode automatique est désactivable via un topic MQTT.

## Topics MQTT

 - LedStrip    => homeassistant/light/'MACADDRESS'/
 - Motor       => homeassistant/fan/'MACADDRESS'/
 - AutoMode    => homeassistant/switch/'MACADDRESS'/
 - Motor       => homeassistant/fan/'MACADDRESS'/
 - All sensors => homeassistant/sensor/'MACADDRESS'/

| Topic                 |   Commande    |  Status               | Format                |
|-----------------------|:-------------:|:---------------------:|:---------------------:|
| AutoMode config       |  N/A          |   AutoMode/config     |   Homeassistant json  |
| AutoMode ON/OFF       | AutoMode/cmd  |   AutoMode/state      |   'ON'/'OFF'          |
| LedStrip config       |  N/A          |   ledStrip/config     |   Homeassistant json  |
| LedStrip ON/OFF       | ledStrip/cmd  |   ledStrip/state      |   'ON'/'OFF'          |
| LedStrip brightness   | ledStrip/bct  |   ledStrip/bst        |   '0' <> '100'        |
| LedStrip color        | ledStrip/rct  |   ledStrip/rst        |   r,g,b(decimal) or #RRGGBB(hexa)|
| Motor config          |  N/A          |   motor/config        |   Homeassistant json  |
| Motor ON/OFF          | motor/cmd     |   motor/state         |   'ON'/'OFF'          |
| Motor speed           | motor/sct     |   motor/sst           |   '0' <> '100'        |
| temperature config    | N/A           |   temperature/config  |   Homeassistant json  |
| temperature value     | N/A           |   temperature/state   |   float string        |
| humidity config       | N/A           |   humidity/config     |   Homeassistant json  |
| humidity value        | N/A           |   humidity/state      |   float string        |
| pressure config       | N/A           |   pressure/config     |   Homeassistant json  |
| pressure value        | N/A           |   pressure/state      |   float string        |
| co2_equivalent config | N/A           |   co2_equivalent/config  |   Homeassistant json  |
| co2_equivalent value  | N/A           |   co2_equivalent/state   |   float string        |
| iaq config            | N/A           |   iaq/config          |   Homeassistant json  |
| iaq value             | N/A           |   iaq/state           |   float string        |
| iaqAccuracy config    | N/A           |   iaqAccuracy/config  |   Homeassistant json  |
| iaqAccuracy value     | N/A           |   iaqAccuracy/state   |   float string        |
| vocEquivalent config  | N/A           |   vocEquivalent/config  |   Homeassistant json  |
| vocEquivalent value   | N/A           |   vocEquivalent/state   |   float string        |
| pm25 config           | N/A           |   pm25/config         |   Homeassistant json  |
| pm25 value            | N/A           |   pm25/state          |   float string        |

## Homeassistant
Le soft va fournir toutes les configuration nécéssaire a homeassistant (icone, unité, nom, plage d'ajustement).
Il va aussi s'authentifer automatiquement et apparaitre sous la forme d'une novuelle entitée "air-manager".

<p align="center">
 <img src="doc/HomeAssistantSummary.png" width="300"/>
 <img src="doc/HomeAssistantColor.png" width="300"/>
 <img src="doc/HomeAssistantAirQualityCards.png" width="300"/>
</p>

Note: Pour avoir ces cartes customisées j'ai utilisées les custom cards 
- custom:slider-button-card
- custom:bar-card

Voir : [Fan Speed](doc/AirControlSpeedCard.yml)  [Led strip color](doc/AirControlColorCard.yml)  [Quality cards](doc/AirQualityCards.yml) 

## TODO
- Sauvegarder le mode auto dans l'eeprom
- More Modes (mode fast, continious slow, ...)


## PCB:

<p align="center">
 <img src="doc/AirPurifierBoardV1.1.png" width="350"/>
</p>

*Note: Send me message if you are interested on a PCB.*
