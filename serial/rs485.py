#!/usr/bin/env python

# RS485 support
#
# This file is part of pySerial. https://github.com/pyserial/pyserial
# (C) 2015 Chris Liechti <cliechti@gmx.net>
#
# SPDX-License-Identifier:    BSD-3-Clause

"""\
The settings for RS485 are stored in a dedicated object that can be applied to
serial ports (where supported).
NOTE: Some implementations may only support a subset of the settings.
"""

import time
from dataclasses import dataclass
from typing import Optional

from _typeshed import ReadableBuffer

import serial


@dataclass(frozen=True)
class RS485Settings:
    rts_level_for_tx: bool = True
    rts_level_for_rx: bool = False
    loopback: bool = False
    delay_before_tx: Optional[int] = None
    delay_before_rx: Optional[int] = None


class RS485(serial.Serial):
    """\
    A subclass that replaces the write method with one that toggles RTS
    according to the RS485 settings.

    NOTE: This may work unreliably on some serial ports (control signals not
          synchronized or delayed compared to data). Using delays may be
          unreliable (varying times, larger than expected) as the OS may not
          support very fine grained delays (no smaller than in the order of
          tens of milliseconds).

    NOTE: Some implementations support this natively. Better performance
          can be expected when the native version is used.

    NOTE: The loopback property is ignored by this implementation. The actual
          behavior depends on the used hardware.

    Usage:

        ser = RS485(...)
        ser.rs485_mode = RS485Settings(...)
        ser.write(b'hello')
    """

    _alternate_rs485_settings: Optional[RS485Settings]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._alternate_rs485_settings = None

    def write(self, data: ReadableBuffer):
        """Write to port, controlling RTS before and after transmitting."""
        if self._alternate_rs485_settings is not None:
            # apply level for TX and optional delay
            self.rts = self._alternate_rs485_settings.rts_level_for_tx
            if self._alternate_rs485_settings.delay_before_tx is not None:
                time.sleep(self._alternate_rs485_settings.delay_before_tx)
            # write and wait for data to be written
            self.write(data)
            self.flush()
            # optional delay and apply level for RX
            if self._alternate_rs485_settings.delay_before_rx is not None:
                time.sleep(self._alternate_rs485_settings.delay_before_rx)
            self.rts = self._alternate_rs485_settings.rts_level_for_rx
        else:
            self.write(data)

    # redirect where the property stores the settings so that underlying Serial
    # instance does not see them
    @property
    def rs485_mode(self) -> Optional[RS485Settings]:
        """\
        Enable RS485 mode and apply new settings, set to None to disable.
        See serial.rs485.RS485Settings for more info about the value.
        """
        return self._alternate_rs485_settings

    @rs485_mode.setter
    def rs485_mode(self, rs485_settings: Optional[RS485Settings]) -> None:
        self._alternate_rs485_settings = rs485_settings
