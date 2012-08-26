# $Id$
# This file is generated automatically from wibo_gen_py.tpl
#
"""
Dumps a intel hex file for a node config structure.

    python nodeaddr.py [OPTIONS]

    Options:
     -a SHORTADDR
        Fixed node short source address (16 bit value).
     -A LONGADDR
        Fixed node short source address (64 bit value).
     -p PANID
        Default pan id (16 bit value).
     -c CHANNEL
        A Channel hint for applications.
     -C CFGFILE:LINE
        The record parameters are read from a file CFGFILE
        at line LINE. The format of CFGFILE is:
        =====================================
        1 00:00:00:00:00:00:00:01 0x4242 17
        =====================================
        (sort addr, long addr, panid, channel)
        The field delimitter is a space.

     -B BOARDNAME
        create the record at flash-end for the MMCU
        of the given BOARDNAME (see also -O option)
     -M MMCU
        give the name of the MMCU (instead of -B)
     -O OFFSET
        per default the address of the config record is flashend.
        With -O an explicit offset can be specified.

     -f HEXFILE
        Name of alternative IHEX file to patch.
     -o NEWHEXFILE
         Name of the outputfile, if '-' stdout is used.
     -h
        Display help and exit.
     -V
        Show version and exit.
     -l
        List board names and exit
     -L
        List MMCU names and exit

Example:
  - Add an address record to a hexfile and flash it to the target.
     python nodeaddr.py -b rdk230 -f wibo_rdk230.hex -a 42 -p 1 > a42.hex
     avrdude -P usb -p m1281 -c jtag2 -U a42.hex

  - Pipe the generated hexfile directly into avrdude.
     python nodeaddr.py -b rdk230 -f xmpl_leds_rdk230.hex -a 42 -p 1 |\
            avrdude -P usb -p m1281 -c jtag2 -U fl:w:-:i
  - Flash the record w/o erasing
    python nodeaddr.py -a 1 | avrdude -P usb -p m1281 -c jtag2 -V -D -U fl:w:-:i


   Writes source address 1 into the device via a pipe to avrdude.

"""




# === import ==================================================================
import struct, getopt, sys
import Tkinter

# === globals =================================================================
VERSION = "0.2.0"

# I/O file parameters
INFILE = None
OUTFILE = sys.stdout

# contents of the config records
PANID = 0xffff
SADDR = 0xffff
LADDR = 0xffffffffffffffffL
CHANNEL  = 0xff

# memory related variables
BOARD = "UNKNOWN"
MMCU_TABLE = {'any2400': 'atmega1281',
 'any2400st': 'atmega1281',
 'any900': 'atmega1281',
 'any900st': 'atmega1281',
 'bitbean': 'atmega1281',
 'cbb212': 'atxmega256a3',
 'cbb230': 'atxmega256a3',
 'cbb230b': 'atxmega256a3',
 'cbb231': 'atxmega256a3',
 'cbb232': 'atxmega256a3',
 'derfa1': 'atmega128rfa1',
 'ibdt212': 'atmega644',
 'ibdt231': 'atmega644',
 'ibdt232': 'atmega644',
 'icm230_11': 'atmega1281',
 'icm230_12a': 'atmega1281',
 'icm230_12b': 'atmega1281',
 'icm230_12c': 'atmega128',
 'ics230_11': 'atmega1281',
 'ics230_12': 'atmega128',
 'ict230': 'atmega1281',
 'im240a': 'atmega328',
 'im240a_eval': 'atmega328',
 'lgee231': 'atmega88',
 'lgee231_v2': 'atmega88',
 'mnb900': 'atmega1281',
 'muse231': 'atmega88pa',
 'psk212': 'atmega1281',
 'psk230': 'atmega1281',
 'psk230b': 'atmega1281',
 'psk231': 'atmega1281',
 'psk232': 'atmega1281',
 'radiofaro': 'atmega128rfa1',
 'radiofaro_v1': 'atmega128rfa1',
 'ravrf230a': 'atmega1284p',
 'ravrf230b': 'atmega1284p',
 'rbb128rfa1': 'atmega128rfa1',
 'rbb212': 'atmega1281',
 'rbb230': 'atmega1281',
 'rbb230b': 'atmega1281',
 'rbb231': 'atmega1281',
 'rdk212': 'atmega1281',
 'rdk230': 'atmega1281',
 'rdk230b': 'atmega1281',
 'rdk231': 'atmega1281',
 'rdk232': 'atmega1281',
 'rose231': 'atmega328p',
 'rzusb': 'at90usb1287',
 'sparkfun': 'atmega128rfa1',
 'stb128rfa1': 'atmega128rfa1',
 'stb212': 'atmega1281',
 'stb230': 'atmega1281',
 'stb230b': 'atmega1281',
 'stb231': 'atmega1281',
 'stkm16': 'atmega16',
 'stkm8': 'atmega8',
 'tiny230': 'attiny84',
 'tiny231': 'attiny84',
 'wdba1281': 'atmega1281',
 'xxo': 'atmega128rfa1',
 'zgbh212': 'atmega1281',
 'zgbh230': 'atmega1281',
 'zgbh231': 'atmega1281',
 'zgbl212': 'atmega1281',
 'zgbl230': 'atmega1281',
 'zgbl231': 'atmega1281',
 'zgbt1281a2nouart': 'atmega1281',
 'zgbt1281a2uart0': 'atmega1281',
 'zgbt1281a2uart1': 'atmega1281',
 'zigduino': 'atmega128rfa1'}
MMCU = MMCU_TABLE.get(BOARD,"?")
OFFSET = None


# This a workaround for obtaining the flashends for the current used MCU's.
# avr-gcc is used to extract the address.
# for i in $(awk -F'=' '/^cpu/{print $2}' Src/Lib/Inc/boards/board.cfg|sort|uniq);do x=$(echo "#include <avr/io.h>"|avr-gcc -mmcu=$i -E -dM -|awk '/FLASHEND/{print  $NF}');echo "\"$i\" : $x,"; done
#
FLASHEND = {
    "at90usb1287" : 0x1FFFF,
    "atmega1281" : 0x1FFFF,
    "atmega1284p" : 0x1FFFF,
    "atmega128rfa1" : (0x1ffff),
    "atmega16" : 0x3FFF,
    "atmega644" : 0xFFFF,
    "atmega8" : 0x1FFF,
    "atmega88" : 0x1FFF,
    "atmega88pa" : 0x1FFF,
    "attiny84" : 0x1FFF,
}

# === functions ===============================================================
##
# Format an intel hex record
# For a description of the intel hex format see:
#  http://en.wikipedia.org/wiki/Intel_HEX
#
# @param rtype
#        record type
#          00     Data Record
#          01     End of File Record
#          02     Extended Segment Address Record
#          03     Start Segment Address Record
#          04     Extended Linear Address Record
#          05     Start Linear Address Record
# @param addr
#           16 bit address value
# @param data
#           list with 8 bit values.
# @return string of the formated record.
#
def ihex_record(rtype, addr,data = []):

    dlen = len(data) & 0xff
    darr  = [ dlen,
             (addr >> 8) & 0xff,
             (addr & 0xff) ,
              rtype & 0xff]
    darr.extend(data)
    crc = 0
    for d in darr:
        crc += d
    crc = ((crc &0xff)^0xff) + 1
    darr.append(crc & 0xff)
    return ":"+"".join(["%02X" %d  for d in darr])

##
# Dallas ibutton crc8.
#
# This implementation is based on avr-libc
# _crc_ibutton_update(), see http://www.nongnu.org/avr-libc/
#
# @param data
#           array of numbers or raw binary string.
# @return The computed crc8 value.
#
# The Dallas iButton test vector must return 0
# ibutton_crc( [ 0x02, 0x1c, 0xb8, 0x01, 0, 0, 0, 0xa2 ] )
#
def ibutton_crc(data):
    if isinstance(data,str):
        idata = map(ord, data)
    else:
        idata = data
    crc =  0
    for d in idata:
        crc = crc ^ d
        for i in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc

##
# Generating the node config structure.
#
# The format of the structure node_config_t is defined in board.h.
# @param memaddr
#           address, where to locate the record
#           Per default memaddr=None and the location FLASHEND is
#           used.
def generate_nodecfg_record(memaddr = None):
    ret = []
    # payload of record w/o crc
    struct_fmt ="<HHQB2x"
    if memaddr == None:
        memaddr = (FLASHEND[MMCU] - struct.calcsize(struct_fmt+"B") + 1)
    extaddr = (memaddr >> 16)
    if (extaddr > 0):
        # is extended addr record needed ?
        #  a  b    c  d    e
        # :02 0000 02 1000 EC
        data = map(ord, struct.pack("<H",extaddr<<4))
        ret.append(ihex_record(2, 0, data))
    data = map(ord,struct.pack(struct_fmt, SADDR, PANID, LADDR, CHANNEL))
    crc8 = ibutton_crc( data )
    data.append(crc8)
    ret.append(ihex_record(0, memaddr, data))
    return ret

##
# add a node cofg structure at the end of the flash.
#
def patch_hexfile(ifile, fo, offset):
    END_RECORD = ":00000001FF"
    if ifile != None:
        fi = open(ifile,"r")
        for l in fi.xreadlines():
            if l.find(END_RECORD) == 0:
                # end record found
                break
            # write regular record
            fo.write(l)
        fi.close()
    nodecfg = generate_nodecfg_record(offset)
    fo.write("\n".join(nodecfg)+"\n")
    fo.write(END_RECORD)
    if fo != sys.stdout:
        fo.close()

def list_boards():
    bl = MMCU_TABLE.keys()
    bl.sort()
    print "BOARD            MMCU                 FLASH-END"
    for b in bl:
        mmcu = MMCU_TABLE[b]
        print "%-16s %-20s 0x%x" % (b, mmcu, FLASHEND.get(mmcu,0))
def list_mmcus():
    mmcus = FLASHEND.keys()
    mmcus.sort()
    print "BOARD               FLASH-END"
    for mmcu in mmcus:
        print "%-20s 0x%x" % (mmcu, FLASHEND.get(mmcu,0))


class EntryField(Tkinter.Frame):
    def __init__(self, master, text, textvariable=None):
        Tkinter.Frame.__init__(self, master=master)
        Tkinter.Label(master=self, text=text).pack(side=Tkinter.LEFT)
        Tkinter.Entry(master=self, textvariable=textvariable).pack(side=Tkinter.RIGHT)

class GUI(object):
    def __init__(self, parent):
        self.frame = Tkinter.Frame(parent)
        self.frame.pack()
        Tkinter.Label(master=self.frame, text=BOARD).pack()
        self.shortaddr = Tkinter.StringVar(value=str(hex(SADDR))) # start with default
        self.shortaddr.trace("w", self.cb_changed)
        self.longaddr = Tkinter.StringVar(value=str(hex(LADDR)))
        self.longaddr.trace("w", self.cb_changed)
        self.panid = Tkinter.StringVar(value=str(hex(PANID)))
        self.panid.trace("w", self.cb_changed)
        self.channel = Tkinter.StringVar(value=str(hex(CHANNEL)))
        self.channel.trace("w", self.cb_changed)
        EntryField(master=self.frame, text="Short Addr", textvariable=self.shortaddr).pack()
        EntryField(master=self.frame, text="Long Addr", textvariable=self.longaddr).pack()
        EntryField(master=self.frame, text="PAN Id", textvariable=self.panid).pack()
        EntryField(master=self.frame, text="Channel", textvariable=self.channel).pack()
        Tkinter.Button(master=self.frame, text="Done", command=self.frame.quit).pack()

    def cb_changed(self, *args):
        global SADDR, LADDR, PANID, CHANNEL
        SADDR = int(self.shortaddr.get(), 16)
        LADDR = int(self.longaddr.get(), 16)
        PANID = int(self.panid.get(), 16)
        CHANNEL = int(self.channel.get(), 16)

def call_gui():
    """ Pop up Tkinter GUI to enter parameters """
    root = Tkinter.Tk()
    app=GUI(root)
    root.mainloop()

# === classes =================================================================

if __name__ == "__main__":
    try:
        opts,args = getopt.getopt(sys.argv[1:],"a:p:A:f:hVo:c:O:B:lLM:C:")
    except:
        opts = (('-h',''),('-V',''))

    if opts == []:
        call_gui()

    #test vectors
    #x = ":1001600060E070E000E010E020E030E00E94D705A1"
    #addr = 0x0160
    #data = [ 0x60, 0xE0, 0x70, 0xE0, 0x00, 0xE0, 0x10, 0xE0,
    #         0x20, 0xE0, 0x30, 0xE0, 0x0E, 0x94, 0xD7, 0x05 ]
    #y = ihex_record(0,addr, data)
    doexit = False
    for o,v in opts:
        if o == "-a":
            SADDR = eval(v)
        elif o == "-A":
            LADDR = eval(v)
        elif o == "-p":
            PANID = eval(v)
        elif o == "-c":
            CHANNEL = eval(v)
        elif o == "-f":
            INFILE = v
        elif o == "-B":
            if MMCU_TABLE.has_key(v):
                BOARD = v
                MMCU = MMCU_TABLE.get(BOARD,"?")
        elif o == "-M":
            if FLASHEND.has_key(v):
                BOARD = "??"
                MMCU = v
        elif o == "-C":
            fn, ln = v.rsplit(":",1)
            f = open(fn,"r")
            ln = eval(ln)
            SADDR,LADDR,PANID,CHANNEL = f.readlines()[ln-1].split(" ",4)
            SADDR = eval(SADDR)
            LADDR = eval("0x"+LADDR.replace(":",""))
            PANID = eval(PANID)
            CHANNEL = eval(CHANNEL)
        elif o == "-h":
            print __doc__
            doexit = True
        elif o == "-V":
            print "Version",VERSION
            doexit = True
        elif o == "-l":
            list_boards()
            doexit = True
        elif o == "-L":
            list_mmcus()
            doexit = True
        elif o == "-o":
            if v != '-':
                x = open(v,'w')
                OUTFILE = x
            sys.stderr.write("Outfile %s\n" % OUTFILE.name)
        elif o == "-O":
            OFFSET = eval(v)

    if doexit:
        sys.exit(0)

    sys.stderr.write("convert %s to %s\n" %(INFILE, OUTFILE.name))
    print "offset",OFFSET
    patch_hexfile(INFILE, OUTFILE, OFFSET)
