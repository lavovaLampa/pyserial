#! python
#
# Base class and support functions used by various backends.
#
# This file is part of pySerial. https://github.com/pyserial/pyserial
# (C) 2001-2020 Chris Liechti <cliechti@gmx.net>
#
# SPDX-License-Identifier:    BSD-3-Clause

from __future__ import annotations

import io
import time
from abc import ABC, abstractmethod, abstractproperty
from enum import Enum
from typing import Any, Final, Iterable, Optional, TypedDict

from _typeshed import WriteableBuffer

from serial.rs485 import RS485Settings


# "for byte in data" fails for python3 as it returns ints instead of bytes
def iterbytes(b: bytes) -> Iterable[bytes]:
    """Iterate over bytes, returning bytes instead of ints (python3)"""
    if isinstance(b, memoryview):
        b = b.tobytes()
    i = 0
    while True:
        a = b[i : i + 1]
        i += 1
        if a:
            yield a
        else:
            break


# all Python versions prior 3.x convert ``str([17])`` to '[17]' instead of '\x11'
# so a simple ``bytes(sequence)`` doesn't work for all versions
def to_bytes(seq: bytes | bytearray | memoryview | list[int]) -> bytes:
    """convert a sequence to a bytes type"""
    if isinstance(seq, bytes):
        return seq
    elif isinstance(seq, bytearray):
        return bytes(seq)
    elif isinstance(seq, memoryview):
        return seq.tobytes()
    else:
        # handle list of integers and bytes (one or more items) for Python 2 and 3
        return bytes(bytearray(seq))


# create control bytes
XON: Final = bytes([17])
XOFF: Final = bytes([19])

CR: Final = bytes([13])
LF: Final = bytes([10])


class SerialException(IOError):
    """Base class for serial port related exceptions."""


class SerialTimeoutException(SerialException):
    """Write timeouts give an exception"""


class PortNotOpenError(SerialException):
    """Port is not open"""

    def __init__(self):
        super(PortNotOpenError, self).__init__(
            "Attempting to use a port that is not open"
        )


class Timeout:
    """\
    Abstraction for timeout operations. Using time.monotonic() if available
    or time.time() in all other cases.

    The class can also be initialized with 0 or None, in order to support
    non-blocking and fully blocking I/O operations. The attributes
    is_non_blocking and is_infinite are set accordingly.
    """

    is_infinite: bool
    is_non_blocking: bool
    target_time: Optional[float]
    duration: Optional[float | int]

    def __init__(self, duration: Optional[int | float]):
        """Initialize a timeout with given duration"""
        self.is_infinite = duration is None
        self.is_non_blocking = duration == 0
        self.duration = duration
        if duration is not None:
            self.target_time = time.monotonic() + duration
        else:
            self.target_time = None

    def expired(self) -> bool:
        """Return a boolean, telling if the timeout has expired"""
        return self.target_time is not None and self.time_left() <= 0

    def time_left(self) -> Optional[int | float]:
        """Return how many seconds are left until the timeout expires"""
        if self.is_non_blocking:
            return 0
        elif self.is_infinite:
            return None
        else:
            delta = self.target_time - time.monotonic()
            if delta > self.duration:
                # clock jumped, recalculate
                self.target_time = time.monotonic() + self.duration
                return self.duration
            else:
                return max(0, delta)

    def restart(self, duration: int | float) -> None:
        """\
        Restart a timeout, only supported if a timeout was already set up
        before.
        """
        self.duration = duration
        self.target_time = time.monotonic() + duration


class ByteSize(Enum):
    FIVE_BITS = 5
    SIX_BITS = 6
    SEVEN_BITS = 7
    EIGHT_BITS = 8


class Parity(Enum):
    NONE = "N"
    EVEN = "E"
    ODD = "O"
    MARK = "M"
    SPACE = "S"


class StopBits(Enum):
    ONE = 1
    ONE_POINT_FIVE = 1.5
    TWO = 2


class SerialBase(ABC, io.RawIOBase):
    """\
    Serial port base class. Provides __init__ function and properties to
    get/set port settings.
    """

    is_open: bool
    portstr: Optional[str]
    name: Optional[str]
    _port: Optional[str]
    _baudrate: int
    _bytesize: ByteSize
    _parity: Parity
    _stopbits: StopBits
    _timeout: Optional[int | float]
    _write_timeout: Optional[int | float]
    _xonxoff: bool
    _rtscts: bool
    _dsrdtr: bool
    _inter_byte_timeout: Optional[Any]
    _rs485_mode: Optional[RS485Settings]
    _rts_state: bool
    _dtr_state: bool
    _break_state: bool
    _exclusive: bool

    # default values, may be overridden in subclasses that do not support all values
    # fmt: off
    BAUDRATES = (50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 4800,
                 9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000,
                 576000, 921600, 1000000, 1152000, 1500000, 2000000, 2500000,
                 3000000, 3500000, 4000000)
    # fmt: on

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = 9600,
        bytesize: ByteSize = ByteSize.EIGHT_BITS,
        parity: Parity = Parity.NONE,
        stopbits: StopBits = StopBits.ONE,
        timeout: Optional[float] = None,
        xonxoff: bool = False,
        rtscts: bool = False,
        write_timeout: Optional[float] = None,
        dsrdtr: bool = False,
        inter_byte_timeout: Optional[float] = None,
        exclusive: bool = False,
    ):

        """\
        Initialize comm port object. If a "port" is given, then the port will be
        opened immediately. Otherwise a Serial port object in closed state
        is returned.
        """

        self.is_open = False
        self.portstr = None
        self.name = None
        # correct values are assigned below through properties
        self._port = None
        self._baudrate = 0
        self._timeout = None
        self._write_timeout = None
        self._inter_byte_timeout = None
        self._rs485_mode = None  # disabled by default
        self._rts_state = True
        self._dtr_state = True
        self._break_state = False

        # assign values using get/set methods using the properties feature
        self.port = port
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.xonxoff = xonxoff
        self.rtscts = rtscts
        self.dsrdtr = dsrdtr
        self.inter_byte_timeout = inter_byte_timeout
        self.exclusive = exclusive

        if port is not None:
            self.open()

    #  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    # to be implemented by subclasses:

    @abstractmethod
    def open(self) -> None:
        ...

    @abstractmethod
    def close(self) -> None:
        ...

    @abstractmethod
    def _reconfigure_port(self, force_update: bool = ...) -> None:
        ...

    @abstractmethod
    def _update_rts_state(self) -> None:
        ...

    @abstractmethod
    def _update_dtr_state(self) -> None:
        ...

    @abstractmethod
    def _update_break_state(self) -> None:
        ...

    @abstractproperty
    def in_waiting(self) -> int:
        ...

    #  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    @property
    def port(self):
        """\
        Get the current port setting. The value that was passed on init or using
        setPort() is passed back.
        """
        return self._port

    @port.setter
    def port(self, port: Optional[str]) -> None:
        """\
        Change the port.
        """
        was_open = self.is_open
        if was_open:
            self.close()
        self.portstr = port
        self._port = port
        self.name = self.portstr
        if was_open:
            self.open()

    @property
    def baudrate(self) -> int:
        """Get the current baud rate setting."""
        return self._baudrate

    @baudrate.setter
    def baudrate(self, baudrate: int) -> None:
        """\
        Change baud rate. It raises a ValueError if the port is open and the
        baud rate is not possible. If the port is closed, then the value is
        accepted and the exception is raised when the port is opened.
        """
        if baudrate < 0:
            raise ValueError(f"Not a valid baudrate: {baudrate:!r}")
        self._baudrate = baudrate
        if self.is_open:
            self._reconfigure_port()

    @property
    def bytesize(self) -> ByteSize:
        """Get the current byte size setting."""
        return self._bytesize

    @bytesize.setter
    def bytesize(self, bytesize: ByteSize):
        """Change byte size."""
        self._bytesize = bytesize
        if self.is_open:
            self._reconfigure_port()

    @property
    def exclusive(self) -> bool:
        """Get the current exclusive access setting."""
        return self._exclusive

    @exclusive.setter
    def exclusive(self, exclusive: bool):
        """Change the exclusive access setting."""
        self._exclusive = exclusive
        if self.is_open:
            self._reconfigure_port()

    @property
    def parity(self) -> Parity:
        """Get the current parity setting."""
        return self._parity

    @parity.setter
    def parity(self, parity: Parity) -> None:
        """Change parity setting."""
        self._parity = parity
        if self.is_open:
            self._reconfigure_port()

    @property
    def stopbits(self) -> StopBits:
        """Get the current stop bits setting."""
        return self._stopbits

    @stopbits.setter
    def stopbits(self, stopbits: StopBits):
        """Change stop bits size."""
        self._stopbits = stopbits
        if self.is_open:
            self._reconfigure_port()

    @property
    def timeout(self) -> Optional[int | float]:
        """Get the current timeout setting."""
        return self._timeout

    @timeout.setter
    def timeout(self, timeout: Optional[int | float]):
        """Change timeout setting."""
        if timeout is not None and timeout < 0:
            raise ValueError(f"Not a valid timeout: {timeout:!r}")
        self._timeout = timeout
        if self.is_open:
            self._reconfigure_port()

    @property
    def write_timeout(self) -> Optional[int | float]:
        """Get the current timeout setting."""
        return self._write_timeout

    @write_timeout.setter
    def write_timeout(self, timeout: Optional[int | float]) -> None:
        """Change timeout setting."""
        if timeout is not None and timeout < 0:
            raise ValueError(f"Not a valid timeout: {timeout:!r}")

        self._write_timeout = timeout
        if self.is_open:
            self._reconfigure_port()

    @property
    def inter_byte_timeout(self) -> Optional[int | float]:
        """Get the current inter-character timeout setting."""
        return self._inter_byte_timeout

    @inter_byte_timeout.setter
    def inter_byte_timeout(self, ic_timeout: Optional[int | float]) -> None:
        """Change inter-byte timeout setting."""
        if ic_timeout is not None and ic_timeout < 0:
            raise ValueError(f"Not a valid timeout: {ic_timeout:!r}")

        self._inter_byte_timeout = ic_timeout
        if self.is_open:
            self._reconfigure_port()

    @property
    def xonxoff(self) -> bool:
        """Get the current XON/XOFF setting."""
        return self._xonxoff

    @xonxoff.setter
    def xonxoff(self, xonxoff: bool) -> None:
        """Change XON/XOFF setting."""
        self._xonxoff = xonxoff
        if self.is_open:
            self._reconfigure_port()

    @property
    def rtscts(self) -> bool:
        """Get the current RTS/CTS flow control setting."""
        return self._rtscts

    @rtscts.setter
    def rtscts(self, rtscts: bool) -> None:
        """Change RTS/CTS flow control setting."""
        self._rtscts = rtscts
        if self.is_open:
            self._reconfigure_port()

    @property
    def dsrdtr(self) -> bool:
        """Get the current DSR/DTR flow control setting."""
        return self._dsrdtr

    @dsrdtr.setter
    def dsrdtr(self, dsrdtr: Optional[bool] = None) -> None:
        """Change DsrDtr flow control setting."""
        if dsrdtr is None:
            # if not set, keep backwards compatibility and follow rtscts setting
            self._dsrdtr = self._rtscts
        else:
            # if defined independently, follow its value
            self._dsrdtr = dsrdtr
        if self.is_open:
            self._reconfigure_port()

    @property
    def rts(self) -> bool:
        return self._rts_state

    @rts.setter
    def rts(self, value: bool) -> None:
        self._rts_state = value
        if self.is_open:
            self._update_rts_state()

    @property
    def dtr(self) -> bool:
        return self._dtr_state

    @dtr.setter
    def dtr(self, value: bool) -> None:
        self._dtr_state = value
        if self.is_open:
            self._update_dtr_state()

    @property
    def break_condition(self) -> bool:
        return self._break_state

    @break_condition.setter
    def break_condition(self, value: bool) -> None:
        self._break_state = value
        if self.is_open:
            self._update_break_state()

    #  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
    # functions useful for RS-485 adapters

    @property
    def rs485_mode(self) -> Optional[RS485Settings]:
        """\
        Enable RS485 mode and apply new settings, set to None to disable.
        See serial.rs485.RS485Settings for more info about the value.
        """
        return self._rs485_mode

    @rs485_mode.setter
    def rs485_mode(self, rs485_settings: Optional[RS485Settings]):
        self._rs485_mode = rs485_settings
        if self.is_open:
            self._reconfigure_port()

    #  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    class SavedSettings(TypedDict, total=True):
        baudrate: int
        bytesize: ByteSize
        parity: Parity
        stopbits: StopBits
        xonxoff: bool
        dsrdtr: bool
        rtscts: bool
        timeout: Optional[int | float]
        write_timeout: Optional[int | float]
        inter_byte_timeout: Optional[int | float]

    def get_settings(self) -> SavedSettings:
        """\
        Get current port settings as a dictionary. For use with
        apply_settings().
        """
        return dict([(key, getattr(self, "_" + key)) for key in self._SAVED_SETTINGS])

    def apply_settings(self, d: SavedSettings) -> None:
        """\
        Apply stored settings from a dictionary returned from
        get_settings(). It's allowed to delete keys from the dictionary. These
        values will simply left unchanged.
        """
        for key, val in d.items():
            if val != getattr(self, "_" + key):  # check against internal "_" value
                setattr(
                    self, key, val
                )  # set non "_" value to use properties write function

    #  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    def __repr__(self) -> str:
        """String representation of the current port settings and its state."""
        return (
            "{name}<id=0x{id:x}, open={p.is_open}>(port={p.portstr!r}, "
            "baudrate={p.baudrate!r}, bytesize={p.bytesize!r}, parity={p.parity!r}, "
            "stopbits={p.stopbits!r}, timeout={p.timeout!r}, xonxoff={p.xonxoff!r}, "
            "rtscts={p.rtscts!r}, dsrdtr={p.dsrdtr!r})".format(
                name=self.__class__.__name__, id=id(self), p=self
            )
        )

    #  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
    # compatibility with io library
    # pylint: disable=invalid-name,missing-docstring

    def readable(self):
        return True

    def writable(self):
        return True

    def seekable(self):
        return False

    def readinto(self, b: WriteableBuffer) -> int:
        data = self.read(len(b))
        assert data is not None
        n = len(data)
        try:
            b[:n] = data
        except TypeError as err:
            import array

            if not isinstance(b, array.array):
                raise err
            b[:n] = array.array("b", data)
        return n

    #  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
    # context manager

    def __enter__(self):
        if self._port is not None and not self.is_open:
            self.open()
        return self

    def __exit__(self, *args, **kwargs):
        self.close()

    #  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    def send_break(self, duration: float = 0.25) -> None:
        """\
        Send break condition. Timed, returns to idle state after given
        duration.
        """
        if not self.is_open:
            raise PortNotOpenError()
        self.break_condition = True
        time.sleep(duration)
        self.break_condition = False

    #  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
    # additional functionality

    def read_all(self):
        """\
        Read all bytes currently available in the buffer of the OS.
        """
        return self.read(self.in_waiting)

    def read_until(self, expected: bytes = LF, size: Optional[int] = None):
        """\
        Read until an expected sequence is found (line feed by default), the size
        is exceeded or until timeout occurs.
        """
        lenterm = len(expected)
        line = bytearray()
        timeout = Timeout(self._timeout)
        while True:
            c = self.read(1)
            if c:
                line += c
                if line[-lenterm:] == expected:
                    break
                if size is not None and len(line) >= size:
                    break
            else:
                break
            if timeout.expired():
                break
        return bytes(line)

    def iread_until(
        self, expected: bytes = LF, size: Optional[int] = None
    ) -> Iterable[bytes]:
        """\
        Read lines, implemented as generator. It will raise StopIteration on
        timeout (empty read).
        """
        while True:
            line = self.read_until(expected, size)
            if not line:
                break
            yield line


#  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
if __name__ == "__main__":
    import sys

    s = SerialBase()
    sys.stdout.write("port name:  {}\n".format(s.name))
    sys.stdout.write("baud rates: {}\n".format(s.BAUDRATES))
    sys.stdout.write("byte sizes: {}\n".format(s.BYTESIZES))
    sys.stdout.write("parities:   {}\n".format(s.PARITIES))
    sys.stdout.write("stop bits:  {}\n".format(s.STOPBITS))
    sys.stdout.write("{}\n".format(s))
