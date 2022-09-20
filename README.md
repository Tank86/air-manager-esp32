# Code pour Purificateur d'air (Wemos-s2, Wemos-c3, Wemos D1-32)

Pour purificateur d'air fait maison, idée originale des frères Poulain
https://www.youtube.com/watch?v=WbyZMVf7Pek

Modifiée pour un style rond et moins "mastoc".

Ajout d'électronique pour le domotiser avec un mode automatique et ainsi qu'un mode fonctionnant avec HomeAssistant (ou tout autre domotique mqtt)
L'electronique permet de piloter/moduler également la vitesse du moteur.

<p align="center">
 <img src="/doc/PurificateurRaw.jpg" width="200"/>
</p>

Liens des composants:
 - Filtre charbon : https://amzn.to/3bw7p8R
 - Filtre HEPA 13 : https://amzn.to/3HPSuT1
 - Turbine Variable : https://amzn.to/3yemHH5
 - Tissu 3D : https://lfp.yt/tissu
 - Contreplaqué
 - Tiges filletés M6
 - Ecrou
 - Rondelles
 
Capteurs:
- Capteur de particules : https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y1010au_appl_e.pdf
- Capteur de qualité de l'air: https://www.adafruit.com/product/3660
- Led WS2818: https://www.adafruit.com/product/1138

Libraries utilisées:
- Bsec 2
- MCP40xx
- home assistant integration (My Fork for Light support and Fan icon)[https://github.com/Tank86/arduino-home-assistant]
- FastLed




PCB:

<p align="center">
 <img src="doc/AirPurifierBoardV1.1.png" width="350"/>
</p>

*Note: Send me message if you are interested on a PCB.*
