#!/usr/bin/env python3
from dataclasses import dataclass

@dataclass
class ParameterSet():
    topic: str
    config_path: str
    config_file_name: str
    number_of_messages_to_average_sample_time_over: int