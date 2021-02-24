#!/usr/bin/python
""" Arduino device port getter """

import sys
import serial.tools.list_ports


def main():
    dev_name = sys.argv[1].strip()
    for p in serial.tools.list_ports.comports():
        if p.product.strip() == dev_name:
            sys.stdout.write(p[0])
            return


if __name__ == "__main__":
    main()
