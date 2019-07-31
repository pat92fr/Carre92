from threading import Thread
import time
import sys

pid_params = {'kp':1.0, 'kd':0.0, 'kd2':0.0}
run_ai = True

def menu():
    global run_ai
    
    def print_menu():
        print("current PID params are:" + str(pid_params))
        print("0: exit")
        print("1: change Kp")
        print("2: change Kd")
        print("3: change Kd2")

    loop = True

    while loop:
        
        print_menu()
        choice = input("Enter your choice [0,3]:")
        if choice == '0':
            loop = False
            run_ai = False
        elif choice == '1':
            print("current value of Kp is:" + str(pid_params['kp']))
            pid_params['kp'] = float( input("Enter Kp:") )
            print(str(pid_params['kp']))
        elif choice == '2':
            print("current value of Kd is:" + str(pid_params['kd']))
            pid_params['kd'] = float( input("Enter Kd:") )
            print(str(pid_params['kd']))
        elif choice == '3':
            print("current value of Kd2 is:" + str(pid_params['kd2']))
            pid_params['kd2'] = float( input("Enter Kd2:") )
            print(str(pid_params['kd2']))
        else:
            print("Wrong menu choice.")
        
class cli(Thread):
    def __init__(self):
        Thread.__init__(self)

    def run(self):
        menu()

class ai(Thread):
    
    def __init(self):
        Thread.__init__(self)

    def run(self):
        global run_ai
        x = 0
        while run_ai:
            time.sleep(1)
            # placeholder
            #print(x)
            sys.stdout.write(str(x))
            sys.stdout.flush()
            x += 1
            # placeholder end
        
##main
thread_cli = cli()
thread_ai = ai()

thread_cli.start()
thread_ai.start()
#menu()

thread_cli.join()
thread_ai.join()

