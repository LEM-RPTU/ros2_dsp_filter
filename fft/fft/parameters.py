#!/usr/bin/env python3 
from dataclasses import dataclass

@dataclass
class ParameterSet():
    topic: str
    number_of_samples: int