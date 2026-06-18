import hid

from protocol import PARAM_KP,PARAM_KI,OP_RUN, build_frame, build_get_config, build_get_status, build_ping, parse_frame, print_frame


def main():
    dev = hid.device()
    dev.open(0x0483, 0xEC01)

    pkt = build_ping(seq=1)

    # En hidapi/Windows suele requerirse report ID al inicio.
    dev.write(bytes([0x00]) + pkt)
    resp = bytes(dev.read(64, timeout_ms=1000))
    print(resp.hex(" "))
    print_frame(parse_frame(resp))

    pkt = build_frame(seq=10, opcode=OP_RUN)
    dev.write(bytes([0x00]) + pkt)
    resp = bytes(dev.read(64, timeout_ms=1000))
    print(resp.hex(" "))
    print_frame(parse_frame(resp))


if __name__ == "__main__":
    main()
