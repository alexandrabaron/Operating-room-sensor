# Capteur IoT pour la Surveillance Environnementale en Bloc Opératoire

## Auteurs
- **Alexandra Baron**
- **Ewen Breton**

## Date
Janvier 2025

## Contexte
Les blocs opératoires nécessitent une surveillance continue des paramètres environnementaux pour garantir des conditions stériles et sécurisées. Ce projet vise à concevoir un capteur IoT capable de surveiller en temps réel la température, l'humidité, la pression et la qualité de l'air dans un bloc opératoire.

## Objectifs
- Mesurer avec précision les paramètres environnementaux critiques.
- Transmettre les données via le protocole LoRaWAN pour une communication longue portée et basse consommation.
- Visualiser les données en temps réel sur une plateforme cloud pour un suivi efficace et des alertes rapides en cas d'anomalie.
- Garantir une solution autonome, adaptée aux exigences des environnements médicaux.

## Architecture du Système
Le système repose sur une architecture modulaire comprenant :
- **Capteurs** : 
  - **LM94021** pour la température.
  - **BME680** pour l'humidité, la pression et la qualité de l'air.
- **Microcontrôleur** : **STM32L432** pour le traitement des données.
- **Communication** : Module LoRa **RFM95W** pour la transmission des données via LoRaWAN.
- **Visualisation** : Plateforme **Datacake** pour la visualisation en temps réel des données.

## Fonctionnement
1. **Acquisition des données** : Les capteurs mesurent les paramètres environnementaux.
2. **Traitement des données** : Le microcontrôleur STM32L432 traite et formate les données au format Cayenne LPP.
3. **Transmission des données** : Le module LoRa RFM95W transmet les données vers The Things Network (TTN).
4. **Visualisation des données** : Les données sont visualisées en temps réel sur Datacake.

## Résultats
- Les tests ont démontré la fiabilité et la régularité de la transmission des données.
- Les données sont correctement acheminées vers TTN et accessibles sur Datacake.
- L'utilisation du format Cayenne LPP facilite l'interprétation et l'affichage structuré des données.



Pour plus d'informations, veuillez consulter le rapport de conception complet dans le fichier `Rapport_de_Conception_Baron_Breton-1.pdf`.
