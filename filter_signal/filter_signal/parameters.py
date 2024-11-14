#!/usr/bin/env python3
from dataclasses import dataclass

@dataclass
class ParameterSet():
    signal_topic: str
    lowpass_cuttoff_frequency: float #Hz
    number_of_messages_to_average_sample_time_over: int = 10