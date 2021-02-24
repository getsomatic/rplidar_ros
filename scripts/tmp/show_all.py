#!/usr/bin/python
""" Arduino device port getter """

import sys
import serial.tools.list_ports


def main():
    for p in serial.tools.list_ports.comports():
        print("[", p.product, "][", p[0], "]")
        print("name="+p.name)
        print("hwid="+p.hwid)
        print("pid="+str(p.pid))
        print("vid="+str(p.vid))
        print("device="+p.device)
        print("description="+p.description)
        print("interface="+p.interface)
        print("product="+p.product)
        print("serial_number="+p.serial_number)
        print("//////////////////////////////////")


if __name__ == "__main__":
    main()
