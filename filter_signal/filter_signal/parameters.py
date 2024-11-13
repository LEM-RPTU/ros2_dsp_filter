#!/usr/bin/env python3
from dataclasses import dataclass

@dataclass
class ParameterSet():
    signal_topic: str
    lowpass_cuttoff_frequency: float