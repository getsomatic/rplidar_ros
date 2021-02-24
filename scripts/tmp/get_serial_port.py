#!/usr/bin/python
""" Arduino device port getter """

import sys
import serial.tools.list_ports


def main():
    dev_name = sys.argv[1].strip()
    print("devname=", dev_name)
    dev_no = 1
    if len(sys.argv) >= 3:
        dev_no = int(sys.argv[2].strip())

    for p in serial.tools.list_ports.comports():
        if p[1].strip() == dev_name:
            dev_no = dev_no - 1
            if dev_no == 0:
                print("sys.stdout.write", p[0])
                sys.stdout.write(p[0])
                return


if __name__ == "__main__":
    main()
