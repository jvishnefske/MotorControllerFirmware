import serial
import serial.tools.list_ports
import time
import json
from contextlib import contextmanager

# todo convert to conntext manager


# noinspection PyPep8Naming
@contextmanager
def serialConn():
    opened = False
    while not opened:
        port_list = serial.tools.list_ports.comports()
        # look for usb serial port
        for port in port_list:
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
                    assert isinstance(ser, serial.Serial)

                    print("serial port open at {}".format(ser.baudrate))
                    last_time = time.time()
                    data_count = 0
                    while True:
                        now = time.time()
                        data = ser.readline()
                        data_count += len(data)
                        elapsed = now - last_time
                        if elapsed > 0:
                            rate = data_count / elapsed
                        else:
                            rate = 0
                        if now - last_time > 1 and data_count > 0:
                            data_count = 0
                            last_time = now
                        # attempt to read as json.
                        try:
                            data = json.loads(data)
                            print(f"{rate:.1f} b/s {data}", end="\r")

                        # except json.decoder.JSONDecodeError or UnicodeDecodeError:
                        except (UnicodeDecodeError , json.decoder.JSONDecodeError):
                            if data_count == 0:
                                print(f"{rate:.1f} b/s raw:{data}", end="\r")
                else:
                    time.sleep(1)
        except serial.SerialException:
            print("serial exception")
            time.sleep(1)


if __name__ == "__main__":
    print("starting")
    main()
    print("done")
