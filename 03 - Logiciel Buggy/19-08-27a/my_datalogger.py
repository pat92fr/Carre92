import time

class datalogger:
    
    def __init__(self, path):
        self._logging = False
        self._directory = path
        self._file = None
        
    def start(self):
        if self._logging:
            self._file.close()
        filename =  self._directory + "/" + "log_" + time.asctime().replace(' ', '_').replace(':', '-')  + ".csv"
        print(filename)
        self._file = open(filename, 'w')
        self._logging = True
        print("Data logger started... ["+str(filename) + "]")
        
    def stop(self):
        if self._logging:
            self._file.close()
            self._logging = False
            print("Data logger stopped!")

    def record(self, list):
        if self._logging:
            if list is not None:
                for field in list:
                ####self._file.write("{:f};".format(float(field)).encode('ascii'))
                    self._file.write(str(field)+";")
                self._file.write("\r\n")
                self._file.flush()
