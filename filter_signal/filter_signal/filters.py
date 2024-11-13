#!/usr/bin/env python3
import numpy as np

class LowPassFilter():
    def __init__(self, cuttoff_frequency, sample_time):
        """
            Implements a first-order low-pass filter.
            y_k = x_k (delta_t/(tau + delta_t)) + y_{k-1} (tau/(tau + delta_t))
            where:
            delta_t = sample_time
            tau = 1/(2*pi*cuttoff_frequency)

            according to: https://en.wikipedia.org/wiki/Low-pass_filter
        """
        self.cuttoff_frequency = cuttoff_frequency
        self.sample_time = sample_time
        self.tau = 1/(2*np.pi*self.cuttoff_frequency)
        self.past_filtered_signal = 0.0

    def filter(self, current_signal):
        filtered_signal = current_signal*(self.sample_time/(self.tau + self.sample_time)) + self.past_filtered_signal*(self.tau/(self.tau + self.sample_time))
        self.past_filtered_signal = filtered_signal
        return filtered_signal  
    
class HighPassFilter():
    def __init__(self, cuttoff_frequency, sample_time):
        """
            Implements a first-order low-pass filter.
            y_k = alpha*(y_{k-1} + x_k - x_{k-1})
            where:
            delta_t = sample_time
            tau = 1/(2*pi*cuttoff_frequency)
            alpha = tau/(tau + delta_t)

            according to: https://en.wikipedia.org/wiki/High-pass_filter
        """
        self.cuttoff_frequency = cuttoff_frequency
        self.sample_time = sample_time
        self.tau = 1/(2*np.pi*self.cuttoff_frequency)
        self.alpha = self.tau/(self.tau + self.sample_time)
        self.past_filtered_signal = 0.0
        self.past_signal = 0.0

    def filter(self, current_signal):
        filtered_signal = self.alpha*(self.past_filtered_signal + current_signal - self.past_signal)
        self.past_signal = current_signal
        self.past_filtered_signal = filtered_signal
        return filtered_signal  