#Accès SSH 

## Buggy 

Le point d'accès pour le buggy est : 

* SSID : "BuggyTraxxas"
* MOt de passe "SecureBuggy"
* L'ip du buggy est  10.42.0.1

## Carte de test

La carte de test est connectée en ethernet au réseau "Modélisme". Le PC doit être connecté en WiFi au réseau modélisme l'IP de la carte de test sur le réseau Modélisme est **10.0.10.49** 

Backup Nvidia : ip = 

# Video côté client

.\gst-launch-1.0 -v udpsrc port=5000 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! autovideosink


.\gst-launch-1.0 -v udpsrc port=5500 caps = "application/x-rtp" ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! videoscale ! autovideosink sync=false
    