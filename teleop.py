
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


class Keyboard:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)

        self.key_timeout = 0.1
        
        # throttle, steer, break
        self.control = np.array([0.0, 0.0, 0.0])

        if self.key_timeout == 0.0:
            self.key_timeout = None

    def _get_key(self, key_timeout, settings):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def read(self):
        try:
            key = self._get_key(self.key_timeout, self.settings)
            if key in ACTION_KEYS:
                self.control += ACTION_KEYS[key]
                self.control = np.clip(self.control, -1, 1)

            else:
                self.control = np.array([0.0, 0.0, 0.0])
                
                if (key == '\x03'):
                    print('Control C')

            return key

        except Exception as e:
            print(e)

        
        
    def teardown(self):
        # What is this?
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)



def main():
    keyboard = Keyboard()
    while True:
        key = keyboard.read()
        if (key == '\x03'): # Control C
            break
        print(key)


if __name__=="__main__":
    main()
