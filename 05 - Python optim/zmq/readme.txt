
###########################################################################################
# Install zmq on rasperry:
#
# sudo apt-get install libtool pkg-config build-essential autoconf automake
#
# wget https://github.com/jedisct1/libsodium/releases/download/1.0.3/libsodium-1.0.3.tar.gz
# tar -zxvf libsodium-1.0.3.tar.gz
# cd libsodium-1.0.3/
# ./configure
# make
# sudo make install
#
# wget http://download.zeromq.org/zeromq-4.1.3.tar.gz
# tar -zxvf zeromq-4.1.3.tar.gz
# cd zeromq-4.1.3/
# ./configure
# make
# sudo make install
# sudo ldconfig
#
# sudo apt-get install python-dev
# sudo pip install pyzmq
# ou
# sudo apt-get python-zmq
#
###########################################################################################

lidar_sensor.py 	: génère et publie une matrice numpy sur le port 5556, fréquence 100Hz
speed_sensor.py 	: génère et publie un entier sur le port 5557, fréqunce 1Hz
serial_sensor.py        : génère et publie une chaine de caractère sur le port 5558, fréquence 0,2Hz
main_sink.py		: réception les flux en écoutant sur les 3 ports (5556, 5557 et 5558)


Pour lancer le test : 4 termiaux

python3 lidar_sensor.py
python3 speed_sensor.py
python3 serial_sensor.py
python3 main_sink.py

On peut créer d'autre instance de main_sink.py

# Eof