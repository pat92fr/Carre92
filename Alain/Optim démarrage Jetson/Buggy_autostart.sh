#! /bin/sh
### BEGIN INIT INFO
# Provides:          buggy
# Required-Start:    
# Required-Stop:     
# Default-Start:     2 3 4 5
# Default-Stop:
# Short-Description: Start Buggy services
# Description:  Start Buggy services
### END INIT INFO

PATH=/bin:/usr/bin:/sbin:/usr/sbin

test -x /usr/bin/python3 || exit 0

. /lib/lsb/init-functions

case "$1" in
  start)
    echo "Demarrage a $(date)" >> /home/patrick/RobotRaceToulouse/demarrage.log
    log_daemon_msg "Starting Buggy services"
    ;;
  restart|force-reload|reload)
	# nothing to do
    :
    ;;
  stop)
	# nothing to do
    :
    ;;
  status)
	# nothing to do
    :
    ;;
  *)
    echo "Usage: /etc/init.d/Buggy_autostart {start|stop|restart|force-reload|reload}"
    exit 2
    ;;
esac

exit 0
