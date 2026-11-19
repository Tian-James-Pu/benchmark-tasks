"""Force-derived gram readout for the balance reconstruction.

HT-120 specifies10Hz display updates,0.01g resolution, a stability indicator,
and selectable response/stability. The exact firmware filter is unpublished.
This model uses an explicitly approximate1s moving mean and0.5s/0.03g stability
window. It consumes sensor force only, never object identity or fault state.
Only normal gram mode/RE-ZERO are implemented here, not configuration firmware.
"""
from collections import deque
import math

class Readout:
    def __init__(self,sample_hz=20.,empty_reference_g=55.):
        self.sample_hz=float(sample_hz)
        self.empty_reference_g=float(empty_reference_g)
        self.samples=deque(maxlen=round(self.sample_hz))
        self.history=deque(maxlen=5)
        self.offset_g=self.empty_reference_g
        self.steps=0
        self.reading_g=None
        self.filtered_g=None
        self.stable=False
        self.error=None
        self._pressed=False

    def update(self,raw_equivalent_g,rezero_pressed=False):
        raw=float(raw_equivalent_g)
        if not math.isfinite(raw):
            self.error='sensor invalid';self.stable=False;self.reading_g=None
            return
        self.steps+=1
        self.samples.append(raw)
        self.filtered_g=sum(self.samples)/len(self.samples)
        # The electronics refresh at10Hz; simulation control may be faster.
        if self.steps%max(1,round(self.sample_hz/10.))==0:
            self.history.append(self.filtered_g)
            self.stable=(len(self.samples)==self.samples.maxlen and
                         len(self.history)==self.history.maxlen and
                         max(self.history)-min(self.history)<=.03)
            self.reading_g=round(self.filtered_g-self.offset_g,2)
        pressed=bool(rezero_pressed)
        if pressed and not self._pressed:
            if not self.stable:
                self.error='unstable'
            elif self.filtered_g-self.empty_reference_g < -2.4:
                self.error='below zero range'
            elif self.filtered_g-self.empty_reference_g > 120.:
                self.error='overload'
            else:
                # Negative weight beyond the documented±2% zero range is not
                # tared. Positive loads above2% are tared normally. No permanent
                # penalty: a later valid zero request can clear an error.
                self.offset_g=self.filtered_g
                self.reading_g=0.
                self.error=None
        self._pressed=pressed

    @property
    def text(self):
        if self.error or self.reading_g is None:return '------'
        if self.reading_g>120.:return 'EEEEEE'
        return f'{self.reading_g:6.2f}'
