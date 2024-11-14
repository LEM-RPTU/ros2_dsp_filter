#!/usr/bin/env python3
from dataclasses import dataclass

@dataclass
class ParameterSet():
    signal_topic: str
    frequency_band: list[float] #Hz
    number_of_messages_to_average_sample_time_over: int
    filter_type: str
    filter_order: int