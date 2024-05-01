# Import necessary modules
import datetime
import struct
from typing import List, Optional, Union

# Importing specific functions and types from other modules
from radiacode.bytes_buffer import BytesBuffer
from radiacode.decoders.databuf import decode_VS_DATA_BUF
from radiacode.decoders.spectrum import decode_RC_VS_SPECTRUM
from radiacode.transports.bluetooth import Bluetooth
from radiacode.transports.usb import Usb
from radiacode.types import CTRL, VS, VSFR, DisplayDirection, DoseRateDB, Event, RareData, RawData, RealTimeData, Spectrum

# Function to convert spectrum channel number to energy
def spectrum_channel_to_energy(channel_number: int, a0: float, a1: float, a2: float) -> float:
    """
    Calculate energy from spectrum channel number using provided coefficients.
    
    Args:
        channel_number (int): The channel number.
        a0 (float): Coefficient a0.
        a1 (float): Coefficient a1.
        a2 (float): Coefficient a2.
        
    Returns:
        float: Energy corresponding to the channel number.
    """
    return a0 + a1 * channel_number + a2 * channel_number * channel_number


class RadiaCode:
    _connection: Union[Bluetooth, Usb]
    """
    Class representing a Radiacode device.

    Attributes:
        _connection (Union[Bluetooth, Usb]): Connection type (Bluetooth or USB).
        _seq (int): Sequence number for requests.
        _base_time (datetime.datetime): Base time for device operations.
        _spectrum_format_version (int): Version of spectrum format used.
    """

    def __init__(
        self,
        bluetooth_mac: Optional[str] = None,
        serial_number: Optional[str] = None,
        ignore_firmware_compatibility_check: bool = False,
    ):
        """
        Initialize RadiaCode instance.

        Args:
            bluetooth_mac (Optional[str]): Bluetooth MAC address.
            serial_number (Optional[str]): Serial number for USB connection.
            ignore_firmware_compatibility_check (bool): If True, skips firmware compatibility check.

        Raises:
            Exception: If firmware version is incompatible.
        """
        # Initialize sequence number
        self._seq = 0
        # Set connection type based on input
        if bluetooth_mac is not None:
            self._connection = Bluetooth(bluetooth_mac)
        else:
            self._connection = Usb(serial_number=serial_number)

        # Initialization routine
        self.execute(b'\x07\x00', b'\x01\xff\x12\xff')  # Execute initialization command
        self._base_time = datetime.datetime.now()  # Set base time to current time
        self.set_local_time(self._base_time)  # Set device local time
        self.device_time(0)  # Initialize device time

        # Check firmware compatibility
        (_, (vmaj, vmin, _)) = self.fw_version()
        if ignore_firmware_compatibility_check is False and vmaj < 4 or (vmaj == 4 and vmin < 8):
            raise Exception(
                f'Incompatible firmware version {vmaj}.{vmin}, >=4.8 required. Upgrade device firmware or use radiacode==0.2.2'
            )

        # Get spectrum format version from device configuration
        self._spectrum_format_version = 0
        for line in self.configuration().split('\n'):
            if line.startswith('SpecFormatVersion'):
                self._spectrum_format_version = int(line.split('=')[1])
                break

    def base_time(self) -> datetime.datetime:
        """
        Get the base time of the device.

        Returns:
            datetime.datetime: The base time of the device.
        """
        return self._base_time

    def execute(self, reqtype: bytes, args: Optional[bytes] = None) -> BytesBuffer:
        """
        Execute a command on the device.

        Args:
            reqtype (bytes): The request type.
            args (Optional[bytes]): Additional arguments for the command.

        Returns:
            BytesBuffer: The response from the device.
        """
        # Ensure request type has correct length
        assert len(reqtype) == 2
        # Generate sequence number
        req_seq_no = 0x80 + self._seq
        self._seq = (self._seq + 1) % 32

        # Construct request packet
        req_header = reqtype + b'\x00' + struct.pack('<B', req_seq_no)
        request = req_header + (args or b'')
        full_request = struct.pack('<I', len(request)) + request

        # Send request and receive response
        response = self._connection.execute(full_request)
        # Extract response header
        resp_header = response.unpack('<4s')[0]
        # Check if response header matches request header
        assert req_header == resp_header, f'req={req_header.hex()} resp={resp_header.hex()}'
        return response

    def read_request(self, command_id: Union[int, VS, VSFR]) -> BytesBuffer:
        """
        Read a request from the device.

        Args:
            command_id (Union[int, VS, VSFR]): Command ID.

        Returns:
            BytesBuffer: The response from the device.
        """
        r = self.execute(b'\x26\x08', struct.pack('<I', int(command_id)))
        retcode, flen = r.unpack('<II')
        assert retcode == 1, f'{command_id}: got retcode {retcode}'
        # HACK: workaround for new firmware bug(?)
        if r.size() == flen + 1 and r._data[-1] == 0x00:
            r._data = r._data[:-1]
        # END OF HACK
        assert r.size() == flen, f'{command_id}: got size {r.size()}, expect {flen}'
        return r

    def write_request(self, command_id: Union[int, VSFR], data: Optional[bytes] = None) -> None:
        """
        Write a request to the device.

        Args:
            command_id (Union[int, VSFR]): Command ID.
            data (Optional[bytes]): Data to be sent.

        Raises:
            AssertionError: If response code is not as expected.
        """
        r = self.execute(b'\x25\x08', struct.pack('<I', int(command_id)) + (data or b''))
        retcode = r.unpack('<I')[0]
        assert retcode == 1
        assert r.size() == 0

    def batch_read_vsfrs(self, vsfr_ids: List[VSFR]) -> List[int]:
        """
        Batch read VSFRs from the device.

        Args:
            vsfr_ids (List[VSFR]): List of VSFR IDs to read.

        Returns:
            List[int]: List of VSFR values.
        """
        assert len(vsfr_ids)
        r = self.execute(b'\x2a\x08', b''.join(struct.pack('<I', int(c)) for c in vsfr_ids))
        ret = [r.unpack('<I')[0] for _ in range(len(vsfr_ids))]
        assert r.size() == 0
        return ret

    def status(self) -> str:
        """
        Get the status of the device.

        Returns:
            str: The status of the device.
        """
        r = self.execute(b'\x05\x00')
        flags = r.unpack('<I')
        assert r.size() == 0
        return f'status flags: {flags}'

    def set_local_time(self, dt: datetime.datetime) -> None:
        """
        Set the local time of the device.

        Args:
            dt (datetime.datetime): The local time to set.
        """
        d = struct.pack('<BBBBBBBB', dt.day, dt.month, dt.year - 2000, 0, dt.second, dt.minute, dt.hour, 0)
        self.execute(b'\x04\x0a', d)

    def fw_signature(self) -> str:
        """
        Get the firmware signature.

        Returns:
            str: The firmware signature.
        """
        r = self.execute(b'\x01\x01')
        signature = r.unpack('<I')[0]
        filename = r.unpack_string()
        idstring = r.unpack_string()
        return f'Signature: {signature:08X}, FileName="{filename}", IdString="{idstring}"'

    def fw_version(self) -> tuple[tuple[int, int, str], tuple[int, int, str]]:
        """
        Get the firmware version.

        Returns:
            tuple[tuple[int, int, str], tuple[int, int, str]]: The boot and target firmware versions.
        """
        r = self.execute(b'\x0a\x00')
        boot_minor, boot_major = r.unpack('<HH')
        boot_date = r.unpack_string()
        target_minor, target_major = r.unpack('<HH')
        target_date = r.unpack_string()
        assert r.size() == 0
        return ((boot_major, boot_minor, boot_date), (target_major, target_minor, target_date.strip('\x00')))

    def hw_serial_number(self) -> str:
        """
        Get the hardware serial number.

        Returns:
            str: The hardware serial number.
        """
        r = self.execute(b'\x0b\x00')
        serial_len = r.unpack('<I')[0]
        assert serial_len % 4 == 0
        serial_groups = [r.unpack('<I')[0] for _ in range(serial_len // 4)]
        assert r.size() == 0
        return '-'.join(f'{v:08X}' for v in serial_groups)

    def configuration(self) -> str:
        """
        Get the device configuration.

        Returns:
            str: The device configuration.
        """
        r = self.read_request(VS.CONFIGURATION)
        return r.data().decode('cp1251')

    def text_message(self) -> str:
        """
        Get the text message from the device.

        Returns:
            str: The text message.
        """
        r = self.read_request(VS.TEXT_MESSAGE)
        return r.data().decode('ascii')

    def serial_number(self) -> str:
        """
        Get the serial number of the device.

        Returns:
            str: The serial number.
        """
        r = self.read_request(8)
        return r.data().decode('ascii')

    def commands(self) -> str:
        """
        Get the available commands on the device.

        Returns:
            str: The available commands.
        """
        br = self.read_request(257)
        return br.data().decode('ascii')

    # called with 0 after init!
    def device_time(self, v: int) -> None:
        """
        Set the device time.

        Args:
            v (int): The time value to set.
        """
        self.write_request(VSFR.DEVICE_TIME, struct.pack('<I', v))

    def data_buf(self) -> List[Union[DoseRateDB, RareData, RealTimeData, RawData, Event]]:
        """
        Get data buffer from the device.

        Returns:
            List[Union[DoseRateDB, RareData, RealTimeData, RawData, Event]]: List of data buffer items.
        """
        r = self.read_request(VS.DATA_BUF)
        return decode_VS_DATA_BUF(r, self._base_time)

    def spectrum(self) -> Spectrum:
        """
        Get the spectrum from the device.

        Returns:
            Spectrum: The spectrum data.
        """
        r = self.read_request(VS.SPECTRUM)
        return decode_RC_VS_SPECTRUM(r, self._spectrum_format_version)

    def spectrum_accum(self) -> Spectrum:
        """
        Get the accumulated spectrum from the device.

        Returns:
            Spectrum: The accumulated spectrum data.
        """
        r = self.read_request(VS.SPEC_ACCUM)
        return decode_RC_VS_SPECTRUM(r, self._spectrum_format_version)

    def dose_reset(self) -> None:
        """Reset dose on the device."""
        self.write_request(VSFR.DOSE_RESET)

    def spectrum_reset(self) -> None:
        """Reset spectrum on the device."""
        r = self.execute(b'\x27\x08', struct.pack('<II', int(VS.SPECTRUM), 0))
        retcode = r.unpack('<I')[0]
        assert retcode == 1
        assert r.size() == 0

    # used in spectrum_channel_to_energy
    def energy_calib(self) -> List[float]:
        """
        Get energy calibration data.

        Returns:
            List[float]: Energy calibration coefficients.
        """
        r = self.read_request(VS.ENERGY_CALIB)
        return list(r.unpack('<fff'))

    def set_energy_calib(self, coef: List[float]) -> None:
        """
        Set energy calibration coefficients.

        Args:
            coef (List[float]): Energy calibration coefficients.

        Raises:
            AssertionError: If number of coefficients is not equal to 3.
        """
        assert len(coef) == 3
        pc = struct.pack('<fff', *coef)
        r = self.execute(b'\x27\x08', struct.pack('<II', int(VS.ENERGY_CALIB), len(pc)) + pc)
        retcode = r.unpack('<I')[0]
        assert retcode == 1

    def set_language(self, lang='ru') -> None:
        """
        Set the device language.

        Args:
            lang (str, optional): Language to set ('ru' or 'en'). Defaults to 'ru'.

        Raises:
            AssertionError: If language value is not supported.
        """
        assert lang in {'ru', 'en'}, 'unsupported lang value - use "ru" or "en"'
        self.write_request(VSFR.DEVICE_LANG, struct.pack('<I', bool(lang == 'en')))

    def set_device_on(self, on: bool):
        """
        Turn the device on or off.

        Args:
            on (bool): If True, turns the device on. If False, turns the device off.

        Raises:
            AssertionError: If on value is True.
        """
        assert not on, 'only False value accepted'
        self.write_request(VSFR.DEVICE_ON, struct.pack('<I', bool(on)))

    def set_sound_on(self, on: bool) -> None:
        """
        Turn sound on or off.

        Args:
            on (bool): If True, turns sound on. If False, turns sound off.
        """
        self.write_request(VSFR.SOUND_ON, struct.pack('<I', bool(on)))

    def set_vibro_on(self, on: bool) -> None:
        """
        Turn vibration on or off.

        Args:
            on (bool): If True, turns vibration on. If False, turns vibration off.
        """
        self.write_request(VSFR.SOUND_ON, struct.pack('<I', bool(on)))

    def set_sound_ctrl(self, ctrls: List[CTRL]) -> None:
        """
        Set sound control flags.

        Args:
            ctrls (List[CTRL]): List of sound control flags.
        """
        flags = 0
        for c in ctrls:
            flags |= int(c)
        self.write_request(VSFR.SOUND_CTRL, struct.pack('<I', flags))

    def set_display_off_time(self, seconds: int) -> None:
        """
        Set display off time.

        Args:
            seconds (int): Display off time in seconds (5, 10, 15, or 30).

        Raises:
            AssertionError: If seconds value is not valid.
        """
        assert seconds in {5, 10, 15, 30}
        v = 3 if seconds == 30 else (seconds // 5) - 1
        self.write_request(VSFR.DISP_OFF_TIME, struct.pack('<I', v))

    def set_display_brightness(self, brightness: int) -> None:
        """
        Set display brightness.

        Args:
            brightness (int): Brightness level (0-9).

        Raises:
            AssertionError: If brightness value is not valid.
        """
        assert 0 <= brightness and brightness <= 9
        self.write_request(VSFR.DISP_BRT, struct.pack('<I', brightness))

    def set_display_direction(self, direction: DisplayDirection) -> None:
        """
        Set display direction.

        Args:
            direction (DisplayDirection): Display direction.
        """
        assert isinstance(direction, DisplayDirection)
        self.write_request(VSFR.DISP_DIR, struct.pack('<I', int(direction)))

    def set_vibro_ctrl(self, ctrls: List[CTRL]) -> None:
        """
        Set vibration control flags.

        Args:
            ctrls (List[CTRL]): List of vibration control flags.

        Raises:
            AssertionError: If CTRL.CLICKS is present in ctrls.
        """
        flags = 0
        for c in ctrls:
            assert c != CTRL.CLICKS, 'CTRL.CLICKS not supported for vibro'
            flags |= int(c)
        self.write_request(VSFR.VIBRO_CTRL, struct.pack('<I', flags))
