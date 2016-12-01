#!/usr/bin/env python3

import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools", "testrunner")))
import testrunner

def testfunc(child):
    cpuid = None
    uuid = None
    eui64 = None
    ipv6ll = None

    searching = True
    try:
        while searching:
            try:
                child.expect(u"CPUID:\s*(?:0x)?([0-9A-Fa-f]+)\r\n")
                cpuid, = child.match.groups()
                child.expect(u"UID:\s*(?:0x)?([0-9A-Fa-f\s]+)\r\n")
                uuid, = child.match.groups()
                child.expect(u"EUI64[^:]*:\s*(?:0x)?([-0-9A-Fa-f\s]+)\r\n")
                eui64, = child.match.groups()
                child.expect(u"link-local IPv6 address[:]?\s*([0-9A-Fa-f:]+)\r\n")
                ipv6ll, = child.match.groups()
                searching = False
            except UnicodeDecodeError:
                # Sometimes there's garbage on the UART after a reset from low power
                # mode which causes UnicodeDecodeError in the pexpect library.
                print("Ignoring UART noise")
                # Flush the input buffer and try again
                child.expect(r'.+')
                continue
    finally:
        print("MCU CPUID: {}".format(cpuid))
        print("MCU UUID: {}".format(uuid))
        print("EUI64: {}".format(eui64))
        print("IPv6LL: {}".format(ipv6ll))

if __name__ == "__main__":
    try:
        print("PROGRAMMER_SERIAL={}".format(os.environ['PROGRAMMER_SERIAL']))
    except KeyError:
        print("PROGRAMMER_SERIAL is not set!")
    sys.exit(testrunner.run(testfunc, echo=False, timeout=3))
