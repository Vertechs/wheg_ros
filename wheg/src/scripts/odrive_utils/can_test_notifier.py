
import time
import can


def main():
    with can.Bus(channel="can0",bustype="socketcan") as bus:
        print_listener = can.Printer()
        can.Notifier(bus, [print_listener])

        time.sleep(1.0)


if __name__ == "__main__":
    main()