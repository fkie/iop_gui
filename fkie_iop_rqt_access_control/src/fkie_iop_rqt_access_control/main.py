#!/usr/bin/env python3

import sys

from rqt_gui.main import Main


def main():
    sys.exit(Main().main(sys.argv, standalone='fkie_iop_rqt_access_control.iop_rqt_access_control.AccessControlClient'))


if __name__ == '__main__':
    main()
