#!/usr/bin/env python3
from scipy import signal

def highpass(frequency: float, order: int, sample_frequency: float):
        b, a = signal.butter(N=order, 
                             Wn=frequency, 
                             btype='highpass', 
                             analog=False, 
                             fs=sample_frequency)
        return [b, a, signal.lfilter_zi(b, a)]
    
def lowpass(frequency: float, order: int, sample_frequency: float):
    b, a = signal.butter(N=order, 
                        Wn=frequency, 
                        btype='lowpass', 
                        analog=False, 
                        fs=sample_frequency)
    return [b, a, signal.lfilter_zi(b, a)]

def bandpass(frequency: list[float], order: int, sample_frequency: float):
    b, a = signal.butter(N=order, 
                        Wn=frequency, 
                        btype='bandpass', 
                        analog=False, 
                        fs=sample_frequency)
    return [b, a, signal.lfilter_zi(b, a)]

def bandstop(frequency: list[float], order: int, sample_frequency: float):
    b, a = signal.butter(N=order, 
                        Wn=frequency, 
                        btype='bandstop', 
                        analog=False, 
                        fs=sample_frequency)
    return [b, a, signal.lfilter_zi(b, a)]