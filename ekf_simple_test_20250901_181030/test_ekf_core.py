#!/usr/bin/env python3

import sys
import os
sys.path.append('/u/12/akyuret1/unix/drone_sim')

import numpy as np
from ekf_parameters import EKFParameters
from ekf_core import ExtendedKalmanFilter
from ekf_sensor_model import SensorModel

def test_ekf_core():
    """Test EKF core functionality"""
    print("Testing EKF core functionality...")
    
    # Initialize parameters
    params = EKFParameters()
    print(f"✅ Parameters initialized: {params.profile}")
    
    # Initialize EKF
    ekf = ExtendedKalmanFilter(params)
    print("✅ EKF initialized")
    
    # Initialize sensor model
    sensor_model = SensorModel(params)
    print("✅ Sensor model initialized")
    
    # Test initialization
    x0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])
    ekf.initialize(x0)
    print("✅ EKF initialized with zero state")
    
    # Test prediction step
    imu = np.array([0, 0, -9.81, 0, 0, 0])  # At rest
    dt = 0.01
    x_pred, P_pred = ekf.predict(imu, dt)
    print(f"✅ Prediction step: state shape {x_pred.shape}, covariance shape {P_pred.shape}")
    
    # Test measurement updates
    gps_meas = np.array([1.0, 2.0, 3.0])
    x_est, P = ekf.update_gps(x_pred, P_pred, gps_meas)
    print(f"✅ GPS update: state shape {x_est.shape}")
    
    baro_meas = 3.0
    x_est, P = ekf.update_baro(x_est, P, baro_meas)
    print(f"✅ Barometer update: state shape {x_est.shape}")
    
    mag_meas = 0.1
    x_est, P = ekf.update_mag(x_est, P, mag_meas)
    print(f"✅ Magnetometer update: state shape {x_est.shape}")
    
    # Test complete step
    x_est, P = ekf.step(imu, dt, gps_meas=gps_meas, baro_meas=baro_meas, mag_meas=mag_meas)
    print(f"✅ Complete EKF step: state shape {x_est.shape}")
    
    # Test sensor model
    x_true = np.array([1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.05, 0.1, 0.2])
    sensors = sensor_model.generate_sensors(x_true, 0.0)
    print(f"✅ Sensor model: generated {len(sensors)} sensor types")
    
    # Test innovation statistics
    stats = ekf.get_innovation_stats()
    print(f"✅ Innovation statistics: {len(stats)} sensor types")
    
    print("\n🎉 EKF core functionality test completed successfully!")
    return True

if __name__ == '__main__':
    try:
        test_ekf_core()
    except Exception as e:
        print(f"❌ EKF core test failed: {e}")
        sys.exit(1)
