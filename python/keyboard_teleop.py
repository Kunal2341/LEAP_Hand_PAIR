import time

import leap_hand_utils.leap_hand_utils as lhu
import numpy as np
from keyboard_listener import KeyboardListener
from leap_node import LeapNode


# init the node
def main(**kwargs):
    listener = KeyboardListener()
    leap_hand = LeapNode()

    homeRad = np.array([3.1492627, 1.6444274, 4.847379,  3.1768742, 
                            4.7522726, 3.1415927, 3.1400588, 4.715457, 
                            3.118583, 3.0771654, 3.103243, 3.0771654, 
                            3.0802333, 3.104777, 3.028078,  3.0633597])

    curr_pos = homeRad
    curr_pos = np.array(curr_pos)
    pos_stride = 0.2

    add_key_mappings = {
        "q": 0,
        "a": 1,
        "z": 2,
        "e": 3,
        "d": 4,
        "c": 5,
        "t": 6,
        "g": 7,
        "b": 8,
        "u": 9,
        "j": 10,
        "o": 11,
        "1": 12,
        "3": 13,
        "5": 14,
        "7": 15,
    }

    minus_key_mappings = {
        "w": 0,
        "s": 1,
        "x": 2,
        "r": 3,
        "f": 4,
        "v": 5,
        "y": 6,
        "h": 7,
        "n": 8,
        "i": 9,
        "k": 10,
        "p": 11,
        "2": 12,
        "4": 13,
        "6": 14,
        "8": 15,
    }

    frequency = 20
    while True:
        try:
            if listener.key in add_key_mappings:
                index = add_key_mappings[listener.key]
                curr_pos[index] += pos_stride
            elif listener.key in minus_key_mappings:
                index = minus_key_mappings[listener.key]
                curr_pos[index] -= pos_stride
            else:
                continue

            leap_hand.set_leap(curr_pos)
            print("Position: " + str(leap_hand.read_pos()))
        except KeyboardInterrupt:
            break

        listener.key = ""
        time.sleep(1 / frequency)


if __name__ == "__main__":
    main()
