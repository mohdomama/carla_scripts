
#!/usr/bin/env python

from __future__ import print_function

import threading
import numpy as np


import sys, select, termios, tty

ACTION_KEYS = {
    'w': np.array([0.05,     0, 0   ]),
    'a': np.array([0.00, -0.05, 0   ]),
    's': np.array([0.00,     0, 0.05]),
    'd': np.array([0.00,  0.05, 0   ]),
}


def getKey(key_timeout, settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)

    key_timeout = 0.1
    
    # throttle, steer, break
    control = np.array([0.0, 0.0, 0.0])

    if key_timeout == 0.0:
        key_timeout = None

    print('here')

    try:
        while(1):
            key = getKey(key_timeout, settings)
            if key in ACTION_KEYS:
                control += ACTION_KEYS[key]
                control = np.clip(control, -1, 1)

            else:
                control = np.array([0.0, 0.0, 0.0])
                
                if (key == '\x03'):
                    break

            print(control)
    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__=="__main__":
    main()
