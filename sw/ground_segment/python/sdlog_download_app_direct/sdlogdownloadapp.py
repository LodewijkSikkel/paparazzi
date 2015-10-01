#!/usr/bin/env python

import wx
import sdlogdownloadframe
import getopt
import sys

def Usage(scmd):
    lpathitem = scmd.split('/')
    fmt = '''Usage: %s [-h | --help] [-a AC_ID | --ac_id=AC_ID] 
\t [-s PORT | --port=PORT] [-b BAUD | --baud=BAUD]
where
\t-h | --help print this message
\t-a AC_ID | --ac_id=AC_ID where AC_ID is an aircraft ID to use for 
\t\tdownloading log data
\t-s PORT | --port=PORT where PORT is the serial port device ('/dev/ttyUSB0')
\t-b BAUD | --baud=BAUD where BAUD is the baudrate of the serial port
'''
    print(fmt % lpathitem[-1])

def GetOptions():
    options = {'ac_id':[], 'port':[], 'baud':[], 'start':[]}
    try:
        optlist, left_args = getopt.getopt(sys.argv[1:],'h:a:p:b:s:', ['help', 'ac_id=', 'port=', 'baud=', 'start='])
    except getopt.GetoptError:
        # print help information and exit:
        Usage(sys.argv[0])
        sys.exit(2)
    for o, a in optlist:
        if o in ("-h", "--help"):
            Usage(sys.argv[0])
            sys.exit()
        elif o in ("-a", "--ac_id"):
            options['ac_id'].append(int(a))
        elif o in ("-p", "--port"):
            options['port'].append(a)
        elif o in ("-b", "--baud"):
            options['baud'].append(int(a))
        elif o in ("-s", "--start"):
            options['start'].append(int(a))

    if not options['port']:
        print "DEFAULT PORT"
        options['port'].append('/dev/ttyACM1')
    if not options['baud']:
        print "DEFAULT BAUD"
        options['baud'].append(int(115200))
    if not options['start']:
        options['start'].append(int(0))
    return options

class SDLogDownloadApp(wx.App):
    def OnInit(self):
        options = GetOptions()
        if not options['ac_id']:
            Usage(sys.argv[0])
            sys.exit("Error: Please specify at least one aircraft ID.")
        self.main = sdlogdownloadframe.SDLogDownloadFrame(options)
        self.main.Show()
        self.SetTopWindow(self.main)
        return True

def main():
    application = SDLogDownloadApp(0)
    application.MainLoop()

if __name__ == '__main__':
    main()
