import rclpy
import numpy as np
import matplotlib.pyplot as plt
import rclpy.duration
from rclpy.node import Node

from std_msgs.msg import Float32


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Float32,'sine_wave', self.listener_callback, 10)
        self.subscription
        self.publisher_ = self.create_publisher(Float32, 'check', 10)
        
        self.samples_to_collect = 1000  # Collect 1000 samples before processing
        self.window_multipliers = [1.0]*self.samples_to_collect
        self.fading_interval = 1
        self.jamm() 
        self.signal_buffer = []
        self.counter = 0
        self.subscription
        self.got_first_message = True

        
        
    def jamm(self):
        fade_in = np.linspace(0, 1, self.fading_interval)
        fade_out = np.linspace(1, 0, self.fading_interval)
        self.window_multipliers[:self.fading_interval] = fade_in
        self.window_multipliers[-self.fading_interval:] = fade_out

    def listener_callback(self, msg):
        if(self.got_first_message):
            self.start_time = self.get_clock().now()
            self.got_first_message = False
        self.signal_buffer.append(msg.data * self.window_multipliers[self.counter])
        msg = Float32()
        msg.data = self.signal_buffer[self.counter]
        self.publisher_.publish(msg)
        self.counter = self.counter + 1
        if len(self.signal_buffer) >= self.samples_to_collect:
            self.time_difference = self.get_clock().now() - self.start_time
            self.time = float(self.time_difference.nanoseconds / 1e9)  # Convert nanoseconds to milliseconds 
            self.get_logger().info(f'Duration: {self.time} s')
            self.sample_frequency = self.samples_to_collect/self.time
            self.get_logger().info(f'Sample Frequency: {self.sample_frequency:.6f} Hz')
            self.nyquist_frequency = self.sample_frequency/2
            self.perform_fft()
            self.signal_buffer.clear()
    def perform_fft(self):
        signal = np.array(self.signal_buffer)
        N = len(signal)  # Number of samples
        T = 1/self.sample_frequency  # Sample interval

        fft_result = np.fft.fft(signal)
        frequencies = np.fft.fftfreq(N, T)  # Frequency in Hz
        self._logger
        # Normalize the FFT amplitude and scale
        fft_amplitude = 2.0 / N * np.abs(fft_result[:N // 2])

        # Only plot the positive half of the frequencies
        plt.plot(frequencies[:N // 2], fft_amplitude)
        plt.title('Frequency Domain Signal')
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Amplitude')
        plt.grid()
        plt.show()

        self.get_logger().info(f'FFT computed for signal with {N} samples')

        


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
