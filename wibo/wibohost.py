# $Id$
"""
wibo host utility

Usage:
    python wibo_host.py [OPTIONS]

    Options:
      -P PORT : Port string ("port" or "port:baudrate"), e.g. /dev/ttyS0, COM1:38400
                [default COM2:38400]
      -h      : show help and exit
      -V      : show version and exit
      -A ADDR : set current node addresses, e.g. -A 1, -A 1,3,5 or -A 1:10,15
                [default 1:8]
      -u FILE : selectively update nodes selected by ADDR with FILE
      -U FILE : broadcast update nodes with FILE
      -S      : scan for nodes in range min(ADDR):max(ADDR),
      -J      : send jump_to_bootloader frame to nodes selected by ADDR
      -E      : send exit_from_bootloader frame to nodes selected by ADDR
      -c CHANS: issue jump bootloader over the given channels, default: [11]
      -v      : increase verbose level

      Examples:

"""
import serial, string, re, time, sys, getopt
HISTORY = "wibohost.hist"
VERSION = 0.01
PORT = "COM2"
BAUDRATE = 38400
ADDRESSES = [1,2,3,4,5,6,7,8]
WHOST = None
VERBOSE = 0
BL_CHANNEL = 11
CHANNELS = [11]

class WIBORxDevice(serial.Serial):
    """ Basic class to interface an WIBO Host on serial line """

    def __init__(self, *args, **kwargs):
        serial.Serial.__init__(self, *args, **kwargs)
        self.flt = re.compile("(?P<code>[A-Z]+)(?P<data>.*)")
        self.VERBOSE = 1

    def _flush(self):
        self.read(self.inWaiting())

    def _sendcommand(self, cmd, *args):
        self._flush()
        cmd = string.join(map(str,[cmd]+list(args)+['\n']))
        if self.VERBOSE: print "TX:",cmd
        self.write(cmd)
        # TODO evaluate returning line and parse for parameters
        s = self.readline().strip()
        if self.VERBOSE: print "RX:",s
        m = self.flt.match(s)
        if m == None:
            return None
        else:
            return m.groupdict()

class WIBOHostDevice(WIBORxDevice):
    """ Class to implement commands for WIBO Host """

    def __init__(self, *args, **kwargs):
        WIBORxDevice.__init__(self, *args, **kwargs)

    def ping(self, nodeid):
        ret=self._sendcommand('ping', hex(nodeid))
        if self.VERBOSE: print ret
        if ret['code'] == 'OK':
            return eval(ret['data'])
        else:
            return None

    def finish(self, nodeid):
        self._sendcommand('finish', hex(nodeid))

    def feedhex(self, nodeid, ln):
        self._sendcommand('feedhex', hex(nodeid), ln)

    def reset(self):
        self._sendcommand('reset')

    def exit(self, nodeid):
        self._sendcommand('exit', hex(nodeid))

    def crc(self):
        return int(self._sendcommand('crc')['data'],16)

    def echo(self, dstr):
        return self._sendcommand('echo', dstr)

    def info(self, nodeid):
        self._sendcommand('info', hex(nodeid))

    def xmpljbootl(self, nodeid):
        self._sendcommand('xmpljbootl', hex(nodeid))

    def channel(self, channel):
        self._sendcommand('channel', str(channel))


class WIBOHost(WIBOHostDevice):
    """ Class to represent a list of WIBO nodes """

    def __init__(self, *args, **kwargs):
        WIBOHostDevice.__init__(self, *args, **kwargs)
        self.devicelist = []

    def scan(self, scanrange=[]):
        return [i for i in scanrange if self.ping(i) != None]

    def flashhex(self, nodeid=0xFFFF, fname=None, delay=0.1):
        f=open(fname)
        self.reset()
        for i, ln in enumerate(f):
            self.feedhex(nodeid, ln.strip())
            time.sleep(delay)
            if VERBOSE:
                print i, ln.strip(), hex(self.crc())
        time.sleep(delay)
        self.finish(nodeid)
        f.close()
        return

    def checkcrc(self, nodeid):
        hostcrc = self.crc()
        print "Host CRC:", hostcrc
        p = self.ping(nodeid)
        print "Host:", hostcrc , "Node%X: " % nodeid , p['crc']
        return hostcrc == p['crc']


def init_prompt():
    global HISTORY
    try:
        import readline, rlcompleter, atexit, sys, os
        sys.ps1 = "wibo>"
        readline.parse_and_bind("tab:complete")
        save_hist = lambda history : readline.write_history_file(history)
        atexit.register(readline.write_history_file, HISTORY)
        if os.path.exists(HISTORY):
            readline.read_history_file(HISTORY)
    except:
        print "No libreadline support"
        traceback.print_exc()

def param_evaluate_list(arg):
    lst = []
    for a in arg.split(","):
        a = a.split(":")
        if len(a)>1:
            lst += eval("range(%s,%s+1)" % (a[0],a[-1]))
        else:
            lst.append(eval(a[0]))
    lst.sort()
    return lst

def process_command_line():
    global PORT, BAUDRATE, ADDRESSES, WHOST, VERBOSE
    opts,args = getopt.getopt(sys.argv[1:],"C:P:A:U:u:hVSJvE")
    ret = False
    for o,v in opts:
        if o == "-h":
            print __doc__
            ret = True
            break
        elif o == "-V":
            print "wibohost v.", VERSION
            ret = True
            break
        elif o == "-v":
            VERBOSE+=1
        elif o == "-P":
            try:
                p,b = v.split(":")
                b = eval(b)
                PORT = p
                BAUDRATE = b
            except ValueError:
                PORT = v
            WHOST.close()
            WHOST.setPort(PORT)
            WHOST.setBaudrate(BAUDRATE)
            WHOST.open()
            print WHOST.echo("HalloWibo") # currently no spaces in echo allowed
        elif o == "-A":
            ADDRESSES = param_evaluate_list(v)
        elif o == "-C":
            CHANNELS = param_evaluate_list(v)
        elif o == "-u":
            print "selective flash"
            for n in ADDRESSES:
                print "selective flashing of node 0x%04x" % n
                WHOST.xmpljbootl(n)
                print "PING:", n, WHOST.ping(n)
                print "FLASH:", n, WHOST.flashhex(n,v)
                print "CRC:", n, WHOST.checkcrc(n)
                print "EXIT:", n, WHOST.exit(n)
        elif o == "-U":
            print "broadcast flashing of nodes"
            ADDRESSES = WHOST.scan(range(min(ADDRESSES), max(ADDRESSES)+1))
            print "SCAN:", ADDRESSES
            for n in ADDRESSES:
                WHOST.xmpljbootl(n)
                print "PING:",n, WHOST.ping(n)
            WHOST.flashhex(0xffff,v)
            for n in ADDRESSES:
                print "CRC:",n, WHOST.checkcrc(n)
                print "EXIT:",n, WHOST.exit(n)
        elif o == "-S":
            ADDRESSES = WHOST.scan(range(min(ADDRESSES), max(ADDRESSES)+1))
            print "SCAN:", ADDRESSES
        elif o == "-J":
            print "Issue jbootl on channel:",
            for c in CHANNELS:
                WHOST.channel(c)
                print c ,
                for n in ADDRESSES:
                    WHOST.xmpljbootl(n)
            WHOST.channel(BL_CHANNEL)
            print
            for n in ADDRESSES:
                res = WHOST.ping(n)
                if res != None:
                    print "PING:",n,res
        elif o == "-E":
            for n in ADDRESSES:
                WHOST.exit(n)
                print "EXIT:",n, WHOST.exit(n)
    return ret

if __name__ == "__main__":
    WHOST = WIBOHost()
    do_exit = process_command_line()
    if do_exit:
        sys.exit(0)
    init_prompt()
