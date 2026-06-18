import serial

from protocol import build_get_status, build_ping, parse_frame, print_frame


def main():
    ser = serial.Serial("COM3", 115200, timeout=1)

    ser.write(build_ping(seq=1))
    resp = ser.read(64)
    print(resp.hex(" "))
    print_frame(parse_frame(resp))

    ser.write(build_get_status(seq=2))
    resp = ser.read(64)
    print(resp.hex(" "))
    print_frame(parse_frame(resp))


if __name__ == "__main__":
    main()
