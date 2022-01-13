import serial
import serial.tools.list_ports
import time
from contextlib import contextmanager

# todo convert to conntext manager


@contextmanager
def serialConn():

    opened = False
    while not opened:
        portList = serial.tools.list_ports.comports()
        # look for usb serial port
        for port in portList:
            if "USB" in port.description:
                ser = serial.Serial(port.device, 9600)
                opened = True
                yield ser
                ser.close()
                break


def main():
    """sit in a loop, and open serial port if it becomes available. disconnect when it goes away."""
    while True:
        try:
            with serialConn() as ser:
                if ser:
                    print("serial port open")
                    while True:
                        data = ser.readline()
                        print(data)
                else:
                    time.sleep(1)
        except serial.SerialException:
            print("serial exception")
            time.sleep(1)


if __name__ == "__main__":
    print("starting")
    main()
    print("done")
