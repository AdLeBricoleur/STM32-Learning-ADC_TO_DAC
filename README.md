# STM32-Learning-ADC_TO_DAC
Ce dépôt correspond à mon apprentissage de l'ADC et du DAC sur STM32L476RG.

Je travaille sur un Nucleo-L476RG.

Pour connaître la configuration des Horloges et des Pinouts, il faut afficher le fichier .ioc sur CubeMX.

Ensuite, j'ai codé dans le fichier Core/Src/main.c. J'utilise l'IDE CubeIDE.

J'ai noté mes observations dans le Dossier Compte_Rendu.

Le but de cette manipulation est de rejouer avec le DAC un signal échantillonné par l'ADC.

J'avais plusieurs objectifs :
- Jouer un sinus avec le DAC d'après une LUT
- Maitriser la fréquence d'échantillonnage de l'ADC
- Utiliser un double buffer pour faire la transition entre l'ADC et le DAC
- Utiliser un DMA pour l'ADC et un DMA pour le DAC
- Synchroniser l'ADC et le DAC avec un Timer 
