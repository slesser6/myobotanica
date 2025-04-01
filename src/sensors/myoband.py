class Myoband:
    

    def connect():
        return 0

    def getData():
        return []
    
    def disconnect():
        return 0
    

import socket
import struct
import time
from collections import deque
import numpy as np

class Myoband:
    """
    Python version of the MyoUdp class from MATLAB.

    It listens for two UDP streams of raw EMG data + IMU data. The first
    stream corresponds to channels 1-8, and the second stream corresponds
    to channels 9-16. It can handle the Thalmic Myo streaming protocols:
    
    1) MyoUdp.exe protocol (packets are 48 bytes)
       - 8 bytes of EMG, then 4 floats for orientation, 3 floats for accel,
         3 floats for gyro
    2) MyoUdp.py BLE protocol (packets are either 16 or 20 bytes)
       - 16-byte packet => 8 EMG channels, 2 samples each (8 int8 values)
       - 20-byte packet => 10 int16 values for orientation, accelerometer,
         gyroscope

    Usage Example:
        myo = MyoUdp(udp_port_8=15001, udp_port_16=15002, verbose=True)
        myo.initialize()

        # Pull data in a loop:
        while True:
            myo.update()           # read from the UDP buffers
            data = myo.get_data(100)  # get latest 100 samples (all 16 channels)
            # do something with `data` ...
            time.sleep(0.01)       # small pause

        # to finish
        myo.close()
    """
    def __init__(self, cfg):
        self.ip = cfg.ip
        self.port1 = cfg.port1
        self.port2 = cfg.port2
        self.sample_freq = cfg.sample_freq
        self.input_freq = cfg.input_freq
        self.verbose = cfg.verbose

        # 16 EMG channels total
        self.num_channels = 16
        # Gains and scaling factors
        self.EMG_GAIN = 0.01

        # IMU data
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.accelerometer = np.array([0.0, 0.0, 0.0], dtype=float)
        self.gyroscope = np.array([0.0, 0.0, 0.0], dtype=float)

        # For second Myo (if used), store orientation, etc. as well
        self.second_myo = {
            'orientation': np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
            'accelerometer': np.array([0.0, 0.0, 0.0], dtype=float),
            'gyroscope': np.array([0.0, 0.0, 0.0], dtype=float)
        }

        # Create ring buffer for EMG data. Here we’ll store up to 5000 samples
        # in a deque or np.array. We store shape = (max_samples, num_channels).
        self.max_buffer_samples = 5000
        self.data_buffer = deque(maxlen=self.max_buffer_samples)

        # Internal state
        self.is_initialized = False
        self.num_packets_received = 0
        self.num_valid_packets = 0

        # Socket references
        self.sock_8 = None
        self.sock_16 = None

    def connect(self):
        """
        Initialize two UDP sockets for channels 1-8 and 9-16.
        """
        if self.is_initialized:
            if self.verbose:
                print("[MyoUdp] Already initialized.")
            return

        # Create socket for channels 1-8
        self.sock_8 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Make it non-blocking or set a small timeout
        self.sock_8.settimeout(0.0)
        self.sock_8.bind(("", self.port1))

        # Create socket for channels 9-16
        self.sock_16 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_16.settimeout(0.0)
        self.sock_16.bind(("", self.port2))

        self.is_initialized = True

        if self.verbose:
            print(f"[MyoUdp] Listening on ports {self.port1} and {self.port2}")
            print("[MyoUdp] Initialization complete.")

    def disconnect(self):
        """
        Close the UDP sockets and clean up.
        """
        if self.sock_8 is not None:
            self.sock_8.close()
            self.sock_8 = None
        if self.sock_16 is not None:
            self.sock_16.close()
            self.sock_16 = None
        self.is_initialized = False
        if self.verbose:
            print("[MyoUdp] Closed all sockets.")

    def _read_all_data(self, sock, max_reads=1000):
        """
        Helper: Read up to `max_reads` packets from a given socket.
        Returns a list of byte-strings.
        Non-blocking read: if no data, returns empty list.
        """
        packets = []
        for _ in range(max_reads):
            try:
                data, _ = sock.recvfrom(1024)
                packets.append(data)
            except socket.error:
                break
        return packets

    def _parse_myo_udp_exe_packet(self, packet):
        """
        Parse a 48-byte packet from MyoUdp.exe.

        Data layout (48 bytes total):
          Bytes 0..7:   int8[8]    => EMG
          Bytes 8..23:  float[4]   => Quaternion
          Bytes 24..35: float[3]   => Accelerometer (g)
          Bytes 36..47: float[3]   => Gyroscope (deg/s)
        """
        # First 8 bytes: EMG as int8
        emg_int8 = struct.unpack('8b', packet[0:8])
        emg = np.array(emg_int8, dtype=float)

        # Next 4 floats: quaternion
        quaternion = struct.unpack('4f', packet[8:8+4*4])
        quat = np.array(quaternion, dtype=float)

        # Next 3 floats: accelerometer
        accel = struct.unpack('3f', packet[8+4*4:8+4*4+3*4])
        accel = np.array(accel, dtype=float)

        # Next 3 floats: gyro
        gyro = struct.unpack('3f', packet[8+4*4+3*4:8+4*4+3*4+3*4])
        gyro = np.array(gyro, dtype=float)

        return emg, quat, accel, gyro

    def _parse_myo_ble_packet(self, packet):
        """
        Parse the 16- or 20-byte BLE streaming packets (MyoUdp.py).
          - 16 bytes => 8 EMG samples (2 samples per packet)
          - 20 bytes => orientation(4), accel(3), gyro(3), each as int16
        """
        if len(packet) == 16:
            # 16 bytes of 8 int8 => 2 EMG samples
            emg_int8 = struct.unpack('8b', packet)
            emg = np.array(emg_int8, dtype=float)
            return emg, None, None, None  # IMU not in this packet
        elif len(packet) == 20:
            # 20 bytes => 10 int16
            #   orientation(4), accel(3), gyro(3)
            data_int16 = struct.unpack('10h', packet)
            data_int16 = np.array(data_int16, dtype=float)

            # Scales
            ORI_SCALE = 16384.0
            ACC_SCALE = 2048.0
            GYR_SCALE = 16.0

            quat = data_int16[0:4] / ORI_SCALE
            accel = data_int16[4:7] / ACC_SCALE
            gyro = data_int16[7:10] / GYR_SCALE

            # No EMG in this packet
            return None, quat, accel, gyro
        else:
            # Unrecognized packet size
            return None, None, None, None

    def update(self):
        """
        Read all available packets from both UDP sockets,
        parse them, and store EMG data / IMU data internally.

        For the MyoUdp.exe approach, each packet is 48 bytes.
        For the BLE approach (MyoUdp.py), packets are 16 or 20 bytes.
        """
        if not self.is_initialized:
            return

        # 1) Read from the first socket (channels 1-8)
        packets_8 = self._read_all_data(self.sock_8)
        for pkt in packets_8:
            pkt_len = len(pkt)
            if pkt_len == 48:
                # MyoUdp.exe approach
                emg, quat, acc, gyr = self._parse_myo_udp_exe_packet(pkt)
                self.orientation = quat
                self.accelerometer = acc
                self.gyroscope = gyr

                # Add EMG data to buffer
                emg_scaled = emg * self.EMG_GAIN
                # Upsample or store as is?
                # For direct 48-byte approach, typically 1 EMG sample per packet:
                # Just store once
                self._append_emg_data(emg_scaled, ch_start=0)

                self.num_packets_received += 1

            elif pkt_len == 16 or pkt_len == 20:
                # Myo BLE approach
                emg, quat, acc, gyr = self._parse_myo_ble_packet(pkt)
                if emg is not None:
                    # 8 EMG values => effectively 2 EMG samples (8 channels)
                    emg_scaled = emg * self.EMG_GAIN
                    # The BLE streaming lumps 2 sets of 8 channels in one packet
                    # (2 time samples). One way is to reshape it to (2 x 8).
                    # But for simplicity, interpret them linearly or store as is.
                    # 
                    # Let's handle it as 8 channels, 1 "time-slice" * 2. 
                    # We'll just store them as consecutive rows:
                    # e.g. If 8 int8 => that's 8 channels * 1 sample, repeated 2x
                    # In the original MATLAB code, they do reshape(8,2).T
                    # so we'll do something similar below:
                    emg_reshaped = emg_scaled.reshape(8, -1).T
                    for row in emg_reshaped:
                        self._append_emg_data(row, ch_start=0)

                if quat is not None:
                    # IMU data
                    self.orientation = quat
                    self.accelerometer = acc
                    self.gyroscope = gyr

                self.num_packets_received += 1

            else:
                if self.verbose:
                    print(f"[MyoUdp] Unknown packet size {pkt_len} on port {self.udp_port_8}")

        # 2) Read from the second socket (channels 9-16)
        packets_16 = self._read_all_data(self.sock_16)
        for pkt in packets_16:
            pkt_len = len(pkt)
            if pkt_len == 48:
                emg, quat, acc, gyr = self._parse_myo_udp_exe_packet(pkt)
                self.second_myo['orientation'] = quat
                self.second_myo['accelerometer'] = acc
                self.second_myo['gyroscope'] = gyr

                emg_scaled = emg * self.EMG_GAIN
                self._append_emg_data(emg_scaled, ch_start=8)

                self.num_packets_received += 1

            elif pkt_len == 16 or pkt_len == 20:
                emg, quat, acc, gyr = self._parse_myo_ble_packet(pkt)
                if emg is not None:
                    emg_scaled = emg * self.EMG_GAIN
                    emg_reshaped = emg_scaled.reshape(8, -1).T
                    for row in emg_reshaped:
                        self._append_emg_data(row, ch_start=8)

                if quat is not None:
                    self.second_myo['orientation'] = quat
                    self.second_myo['accelerometer'] = acc
                    self.second_myo['gyroscope'] = gyr

                self.num_packets_received += 1

            else:
                if self.verbose:
                    print(f"[MyoUdp] Unknown packet size {pkt_len} on port {self.udp_port_16}")

    def _append_emg_data(self, emg_vals, ch_start=0):
        """
        Store EMG row data into our ring buffer.
        :param emg_vals: array of length up to 8
        :param ch_start: 0 if channels 1-8, 8 if channels 9-16
        """
        # We want to produce a 16-length row (for all 16 channels),
        # then fill the relevant portion with these EMG values.
        row = np.zeros(self.num_channels, dtype=float)
        row[ch_start:ch_start+len(emg_vals)] = emg_vals[:]
        self.data_buffer.append(row)

    def get_data(self, num_samples, channel_indices=None):
        """
        Return the most recent `num_samples` from the buffer
        (across the requested channel indices). 
        Data shape => (num_samples, len(channel_indices)).

        This also handles upsampling if the user wants 1000 Hz data
        but Myo is streaming at ~200 or 300 Hz.
        """
        if channel_indices is None:
            channel_indices = range(self.num_channels)

        # Convert the ring buffer to a numpy array
        buf_array = np.array(self.data_buffer)
        if buf_array.shape[0] < 1:
            # If no data, return zeros
            return np.zeros((num_samples, len(channel_indices)))

        # We'll take the last slice from the buffer
        # but we must also handle upsampling to self.sample_frequency
        # from self.input_frequency if needed.
        desired_rate = self.sample_frequency
        actual_rate = self.input_frequency

        n_buffered = buf_array.shape[0]
        # We only have n_buffered frames at the actual Myo rate
        # We'll decide how many we need from the ring buffer to 
        # produce `num_samples` after upsampling
        if actual_rate <= 0:
            # Fallback
            actual_rate = desired_rate
        up_factor = desired_rate / actual_rate
        # The number of "raw" frames needed:
        raw_needed = int(np.ceil(num_samples / up_factor))
        if raw_needed > n_buffered:
            raw_needed = n_buffered

        # Extract raw data (the last `raw_needed` frames)
        raw_data = buf_array[-raw_needed:, :]
        # Now upsample from `raw_needed` to `num_samples`
        if raw_needed == 1:
            # With only one frame, just repeat
            out_data = np.tile(raw_data, (num_samples, 1))
        else:
            # We can do a linear interpolation along the "time" axis
            x_original = np.linspace(0, 1, raw_needed)
            x_new = np.linspace(0, 1, num_samples)
            out_data = np.zeros((num_samples, self.num_channels), dtype=float)
            for ch in range(self.num_channels):
                out_data[:, ch] = np.interp(x_new, x_original, raw_data[:, ch])

        # Now subselect channels
        out_data = out_data[:, channel_indices]
        return out_data

    def is_ready(self, num_samples=1):
        """
        Check if there's enough data in the buffer for `num_samples`.
        Return True or False.
        """
        return len(self.data_buffer) >= num_samples

    # ----------------------------------------------------------------------
    # Optional helpers to get rotation matrix from the quaternion:
    # ----------------------------------------------------------------------
    def get_rotation_matrix(self, second=False):
        """
        Return a 3x3 rotation matrix derived from the current quaternion.
        If second=True, return the rotation matrix for second Myo.
        """
        if not second:
            q = self.orientation
        else:
            q = self.second_myo['orientation']
        R = self.quaternion_to_rotation_matrix(q)
        # Attempt to orthogonalize via SVD if needed:
        U, _, Vt = np.linalg.svd(R)
        R_ortho = U @ Vt
        return R_ortho

    @staticmethod
    def quaternion_to_rotation_matrix(q):
        """
        Convert quaternion [w, x, y, z] to a 3x3 rotation matrix.
        """
        w, x, y, z = q
        R = np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
            [2*(x*y + w*z),     1 - 2*(x*x + z*z),     2*(y*z - w*x)],
            [2*(x*z - w*y),         2*(y*z + w*x), 1 - 2*(x*x + y*y)]
        ], dtype=float)
        return R