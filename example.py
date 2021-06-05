import argparse

from radiacode import RadiaCode


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bluetooth-mac', type=str, required=False, help='bluetooth MAC address of radiascan device')
    args = parser.parse_args()

    if args.bluetooth_mac:
        print('will use Bluetooth connection')
        rc = RadiaCode(bluetooth_mac=args.bluetooth_mac)
    else:
        print('will use USB connection')
        rc = RadiaCode()

    serial = rc.serial_number()
    print(f'### Serial number: {serial}')
    print('--------')

    spectrum = rc.spectrum()
    print(f'### Spectrum: {spectrum}')
    print('--------')

    print('### DataBuf:')
    for v in rc.data_buf():
        print(v)


if __name__ == '__main__':
    main()
