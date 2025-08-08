#include "imu_mpu_9250.hpp"

bool IMU_MPU9250::begin()
{
    if (!cal.begin())
    {
        Serial.println("[IMU_MPU_9250] Failed to initialize calibration helper");
    }
    else if (!cal.loadCalibration())
    {
        Serial.println("[IMU_MPU_9250] No calibration loaded / found");
    }

    // Initialize sensors
    int retval = init_sensors();
    if (retval == 1)
    {
        Serial.println("[IMU_MPU_9250] Failed to find sensor");
        while (1)
            return false;
    }
    else if (retval == 2)
    {
        Serial.println("[IMU_MPU_9250] Failed to identify sensor");
        while (1)
            return false;
    }
    else if (retval == 3)
    {
        Serial.println("[IMU_MPU_9250] Failed to identify AK8963");
        while (1)
            return false;
    }
    else if (retval != 0)
    {
        Serial.printf("[IMU_MPU_9250] Unknown error %d\n", retval);
        while (1)
            return false;
    }

    _accelerometer = _imu.getAccelerometerSensor();
    _gyroscope = _imu.getGyroSensor();
    _magnetometer = _imu.getMagnetometerSensor();

    // default sensor setup should be ok ?
    _filter.begin(FILTER_UPDATE_RATE_HZ);

    Wire.setClock(400000); // 400kHz
    return true;
}

bool IMU_MPU9250::sample(TelemetrySample &out)
{
    float ax, ay, az, gx, gy, gz;

    sensors_event_t accel, gyro, mag;
    _accelerometer->getEvent(&accel);
    _gyroscope->getEvent(&gyro);
    _magnetometer->getEvent(&mag);

    cal.calibrate(accel);
    cal.calibrate(gyro);
    cal.calibrate(mag);

    // Gyro needs conversion from  Rad/s to Deg/s
    // The rest are not unit important
    gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
    gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
    gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

    // Update sensor fusion filter
    _filter.update(gx, gy, gz,
                   accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                   mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

    float qw, qx, qy, qz;
    _filter.getQuaternion(&qw, &qx, &qy, &qz);

    _encoder.begin();
    _encoder.add("roll", _filter.getRoll());
    _encoder.add("pitch", _filter.getPitch());
    _encoder.add("yaw", _filter.getYaw());
    _encoder.add("qw", qw);
    _encoder.add("qx", qx);
    _encoder.add("qy", qy);
    _encoder.add("qz", qz);

    const char *payload;
    size_t len;
    if (!_encoder.finalize(payload, len))
    {
        Serial.println("[IMU_MPU_9250] Failed to encode JSON");
        return false;
    }

    out.topic_suffix = "imu";
    out.payload = payload;
    out.payload_length = len;
    out.is_binary = false; // JSON format
    return true;
}

int IMU_MPU9250::init_sensors(void)
{
    int retval = _imu.begin();
    if (retval == 0)
    {
        _accelerometer = _imu.getAccelerometerSensor();
        _gyroscope = _imu.getGyroSensor();
        _magnetometer = _imu.getMagnetometerSensor();
    }

    return retval;
}