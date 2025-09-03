from gpiozero import Buzzer
from time import sleep
from signal import pause
import argparse

#!/usr/bin/env python3
# Controls an active buzzer on Raspberry Pi using gpiozero (BCM pin numbering).
# Default pin: 13 (GPIO13). Requires gpiozero and a compatible GPIO backend.


def beep(buzzer: Buzzer, count: int = 3, on_time: float = 0.2, off_time: float = 0.2) -> None:
    for _ in range(count):
        buzzer.on()
        sleep(on_time)
        buzzer.off()
        sleep(off_time)

def main() -> None:
    parser = argparse.ArgumentParser(description="Control an active buzzer on BCM pin 13 using gpiozero.")
    parser.add_argument("--pin", type=int, default=13, help="BCM pin number (default: 13)")
    sub = parser.add_subparsers(dest="cmd")

    sub.add_parser("on", help="Hold the buzzer on until Ctrl+C")
    sub.add_parser("off", help="Turn the buzzer off and exit")

    p_beep = sub.add_parser("beep", help="Beep N times")
    p_beep.add_argument("-n", "--count", type=int, default=3, help="Number of beeps")
    p_beep.add_argument("--on-time", type=float, default=0.2, help="Seconds ON per beep")
    p_beep.add_argument("--off-time", type=float, default=0.2, help="Seconds OFF between beeps")

    args = parser.parse_args()

    buzzer = Buzzer(args.pin)
    try:
        if args.cmd == "on":
            buzzer.on()
            print("Buzzer ON (Ctrl+C to stop)")
            try:
                pause()
            except KeyboardInterrupt:
                pass
        elif args.cmd == "off":
            buzzer.off()
        else:
            # Default action: beep pattern
            beep(buzzer, getattr(args, "count", 3), getattr(args, "on_time", 0.2), getattr(args, "off_time", 0.2))
    finally:
        buzzer.off()
        buzzer.close()

if __name__ == "__main__":
    main()