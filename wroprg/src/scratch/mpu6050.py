"""Custom mpu6050 for fast read """
from adafruit_bus_device.i2c_device import I2CDevice

class MPU6050:
    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG  = 0x1B

    def __init__(self, i2c, address=0x68):
        self.i2c_device = I2CDevice(i2c, address)

        # Wake up MPU6050 (clear sleep bit in power mgmt register 0x6B)
        self._write_register(0x6B, 0x00)

        # Get scaling factors from config registers
        self._accel_scale = self._get_accel_scale()
        self._gyro_scale = self._get_gyro_scale()

        # Cache
        self._accel = (0.0, 0.0, 0.0)
        self._gyro = (0.0, 0.0, 0.0)
        self._accel_raw = (0.0, 0.0, 0.0)
        self._gyro_raw = (0.0, 0.0, 0.0)
        self._temp = 0.0
    def _write_register(self, reg, value):
        with self.i2c_device as i2c:
            i2c.write(bytes([reg, value]))

    def _read_register(self, reg, length=1):
        buf = bytearray(length)
        with self.i2c_device as i2c:
            i2c.write_then_readinto(bytes([reg]), buf)
        return buf

    def _get_accel_scale(self):
        val = self._read_register(self.ACCEL_CONFIG)[0]
        fs_sel = (val >> 3) & 0x03
        scale = {0: 16384.0, 1: 8192.0, 2: 4096.0, 3: 2048.0}
        return scale.get(fs_sel, 16384.0)

    def _get_gyro_scale(self):
        val = self._read_register(self.GYRO_CONFIG)[0]
        fs_sel = (val >> 3) & 0x03
        scale = {0: 131.0, 1: 65.5, 2: 32.8, 3: 16.4}
        return scale.get(fs_sel, 131.0)

    def read(self):
        """Perform one burst read and update cached accel, gyro, temp."""
        data = self._read_register(0x3B, 14)

        def to_signed(h, l):
            value = (h << 8) | l
            if value >= 0x8000:
                value -= 0x10000
            return value

        ax = to_signed(data[0], data[1]) / self._accel_scale
        ay = to_signed(data[2], data[3]) / self._accel_scale
        az = to_signed(data[4], data[5]) / self._accel_scale
        temp = (to_signed(data[6], data[7]) / 340.0) + 36.53
        # Preserve raw (signed) integer values for debug/validation
        gx_raw = to_signed(data[8], data[9])
        gy_raw = to_signed(data[10], data[11])
        gz_raw = to_signed(data[12], data[13])

        # Convert to physical units: gyro in deg/s, accel in g
        gx = gx_raw / self._gyro_scale
        gy = gy_raw / self._gyro_scale
        gz = gz_raw / self._gyro_scale

        self._accel = (ax, ay, az)
        self._gyro = (gx, gy, gz)
        # Store raw values for debugging and validation
        self._accel_raw = (to_signed(data[0], data[1]), to_signed(data[2], data[3]), to_signed(data[4], data[5]))
        self._gyro_raw = (gx_raw, gy_raw, gz_raw)
        self._temp = temp

        return self._accel, self._gyro, self._temp

    @property
    def acceleration(self):
        return self._accel

    @property
    def accel_raw(self):
        """Return raw accelerometer signed integers (LSB)."""
        return getattr(self, '_accel_raw', (0, 0, 0))

    @property
    def gyro(self):
        return self._gyro

    @property
    def gyro_raw(self):
        """Return raw gyroscope signed integers (LSB)."""
        return getattr(self, '_gyro_raw', (0, 0, 0))

    @property
    def temperature(self):
        return self._temp
        