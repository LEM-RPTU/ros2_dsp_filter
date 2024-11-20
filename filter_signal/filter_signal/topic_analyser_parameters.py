#!/usr/bin/env python3
from dataclasses import dataclass

@dataclass
class ParameterSet():
    topic: str
    config_path: str
    result_file_name: str