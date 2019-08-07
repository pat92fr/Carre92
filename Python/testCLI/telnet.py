import socketserver
from telnetsrv.threaded import TelnetHandler, command

class MyHandler(TelnetHandler):
    WELCOME = "Welcome to my server."

    @command(['echo', 'copy', 'repeat'])
    def command_echo(self, params):
        '''<text to echo>
        Echo text back to the console.
        '''
        self.writeresponse( ' '.join(params) )
            
#class TelnetServer(SocketServer.TCPServer):
#    allow_reuse_address = True

server = TelnetServer(("0.0.0.0", 8023), MyHandler)
server.serve_forever()
