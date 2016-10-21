#!/usr/bin/python
# Copyright (c) 2013, Eislab, Lulea University of Technology

from __future__ import print_function
from __future__ import division

import sys, time, os.path, operator, serial, struct, os, math
__author__ = "Henrik Makitaavola"

# Some constants
serial_speed = 38400
SEND_MORE_BYTE = 0x76
SEND_OK_BYTE = 0x94
SEND_RESEND_BYTE = 0x52
# Buffer size in the MCU
REMOTE_BUF_SIZE = 400

def encode(val, dim):
    output = []
    for i in range(dim):
        output.append(val & 0xFF)
        val = val >> 8
    output.reverse()
    return output

# Creates a binary image from the information contained in the file specified
# by 'ihex'
def buildImage(ihex):
    all = []
    section = []
    end_addr = None
    offset = 0

    print("Making image from %s" % (ihex,))
    with open(ihex, 'r') as fp:
        image = fp.read()

    for line in image.split():
        #print "DEBUG:", line
        try:
            length = int(line[1:3], 16)
            addr = int(line[3:7], 16) + offset
            rectype = int(line[7:9], 16)
            data = []
            if len(line) > 11:
                data = [int(line[i:i+2], 16) for i in range(9, len(line)-2, 2)]
            crc = int(line[-2:], 16)
            if rectype in [0x00, 0x03]:
                if not end_addr:
                    end_addr = addr
                    start_addr = addr
                if end_addr != addr:
                    all.append((start_addr, section))
                    if rectype == 0x03:
                        # This last record updates the first 4 bytes which
                        # holds some low level configuration. They are the
                        # same all the time so I guess that's why they are
                        # skipped.
                        break
                    section = []
                    start_addr = addr
                section += data
                end_addr = addr + length
            elif rectype == 0x02:
                offset = int(line[9:9+4], 16) << 4
            elif rectype == 0x01:
                all.append((start_addr, section))
                print("0x01 start_addr: %x" % (start_addr,))
                section = []
                start_addr = addr
        except:
            print("Unable to parse \"%s\", check that it really is a ihex file." % (ihex,))
            return []

    #Pop all sections except the first
    for e in all:
      print("start_addr: 0x%x" % (e[0],))
    for i in range(1, len(all)):
      all.pop()
    print('Ihex read complete:')
    print(('  ' + '\n  '.join(["%6d bytes starting at 0x%X" % (len(l), a) for (a, l) in all])))
    #print('  %d bytes in %d sections\n' % (reduce(operator.add, [len(l) for (_, l) in all]), len(all)))

    #all_data = []
    #all_data += encode(len(all), 2)
    #for (addr, data) in all:
    #    all_data += encode(addr, 4) + encode(len(data), 4) + data
    return all

# Function to calculate checksums
def calc_crc(crc, data):
    lo8 = crc & 0x00FF
    hi8 = (crc >> 8) & 0x00FF
    data ^= lo8
    data ^= data << 4
    data &= 0xFF

    return (((data << 8) | hi8 ) ^ (data >> 4) ^ (data << 3))


def calc_crc32(data):
    crc = 0xFFFFFFFF;
    for byte in data:
        crc = crc ^ byte
        for j in range(0,8): # Do eight times.
            mask = (-(crc & 1)) & 0xFFFFFFFF
            crc = (crc >> 1) ^ (0xEDB88320 & mask)
    return crc ^ 0xFFFFFFFF


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: %s <ihex> <output> <start address>" % sys.argv[0])
        print("  <ihex> is the application program in ihex format.")
        print("  <output> is the name of the file to store the uploadable ihex data in.")
        print("  <start address> expected start address of application in memory")
    else:
        try:
            os.stat(sys.argv[1])         # Checks whether the specified file is valid
        except:
            print("ERROR: Unable to open file \"%s\"." % sys.argv[1])
            exit(1)
        #  Create the binary image
        data = buildImage(sys.argv[1])
        if len(data) != 1:
            print("ERROR: Only able to handle one section")
            exit(1)
        #print data[0][0]
        start = int(sys.argv[3], 16)
        if (data[0][0] != start):
            print("ERROR: Expected start address 0x%X but got 0x%X" %(start, data[0][0]))
            exit(1)
        with open(sys.argv[2], 'wb') as f:
            cnr = 0
            crc = 0

            num_blocks = (len(data[0][1]) + 511) // 512 # floored division
            size = len(data[0][1]) + num_blocks * 2
            f.write(struct.pack(">I", size))
            crc32 = calc_crc32(data[0][1])
            f.write(struct.pack(">I", crc32))

            for d in data[0][1]:
                f.write(struct.pack("B", d))
                crc = calc_crc(crc, d)
                cnr += 1
                if cnr == 512:
                    crc = calc_crc(crc, (crc32 >> 24) & 0xFF)
                    crc = calc_crc(crc, (crc32 >> 16) & 0xFF)
                    crc = calc_crc(crc, (crc32 >>  8) & 0xFF)
                    crc = calc_crc(crc, (crc32 >>  0) & 0xFF)
                    f.write(struct.pack(">H", crc))
                    crc = 0
                    cnr = 0
            if cnr != 0:
                crc = calc_crc(crc, (crc32 >> 24) & 0xFF)
                crc = calc_crc(crc, (crc32 >> 16) & 0xFF)
                crc = calc_crc(crc, (crc32 >>  8) & 0xFF)
                crc = calc_crc(crc, (crc32 >>  0) & 0xFF)
                f.write(struct.pack(">H", crc))
