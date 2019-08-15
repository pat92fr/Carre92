C:\gstreamer\1.0\x86_64\bin\gst-launch-1.0 -v udpsrc port=5500 caps = "application/x-rtp" ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! videoscale ! autovideosink sync=false
pause
