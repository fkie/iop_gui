#!/usr/bin/env python3

import sys

from rqt_gui.main import Main


def main():
    sys.exit(Main().main(sys.argv, standalone='fkie_iop_rqt_digital_resource_viewer.iop_rqt_digital_resource_viewer.DigitalResourceViewer'))


if __name__ == '__main__':
    main()
