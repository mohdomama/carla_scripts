import curses
import time


def motion(key):
    print('Pressed: ',  key)


def main(stdscr):
        print('In the main loop.')
        stdscr.nodelay(True)
        stdscr.clear()

        while True:
            c = stdscr.getch()
            curses.flushinp()
            if c == -1:
                stdscr.clear()
                print('Hold')
            else:
                stdscr.clear()
                motion(chr(c))


            time.sleep(0.1)


curses.wrapper(main)