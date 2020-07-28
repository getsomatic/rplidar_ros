#!/usr/bin/python
""" Arduino device port getter """

import sys
import serial.tools.list_ports


def main():
    for p in serial.tools.list_ports.comports():
	print("[", p[1], "][", p[0], "]")


if __name__ == "__main__":
    main()
