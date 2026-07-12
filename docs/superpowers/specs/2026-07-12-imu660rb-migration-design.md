# IMU660RB Migration Design

## Goal

Switch the application from the existing IMU660RA calls to the IMU660RB driver already included in the project, without changing line-following or yaw-control behavior.

## Scope

- Replace IMU initialization with `imu660rb_init()`.
- Replace gyro sampling with `imu660rb_get_gyro()`.
- Read the Z-axis sample from `imu660rb_gyro_z`.
- Convert raw gyro samples with `imu660rb_gyro_transition()`.
- Update stale application comments that name IMU660RA.

The existing zero-bias calibration, sampling cadence, yaw integration, Ring API, motor control, servo configuration, and race logic remain unchanged. The IMU660RB driver and its Keil project entries already exist and will not be modified.

## Data Flow

Initialization calls the IMU660RB driver. Calibration and runtime sampling then obtain `imu660rb_gyro_z`, subtract the existing measured bias, convert the result through the RB driver's scale conversion function, and feed the unchanged yaw integrator.

## Verification

- Static search finds no executable IMU660RA application calls or variables.
- Changed application units compile with C251 without errors.
- The full Keil target rebuild completes without errors.
- Physical HIL confirms successful IMU initialization and the expected yaw sign; HIL remains blocked until the car is available.

## Known Constraints

The migration assumes the physical IMU660RB uses the SPI3 and CS pin mapping declared by `zf_device_imu660rb.h`. No pin or driver-register changes are in scope.
