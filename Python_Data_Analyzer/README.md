# Python Data Processor

The Python data processor has not yet been written.  Here are some notes of what could potentially be implemented and done.

See also the [Main Project README](../README.md) and the accompanying [Flight Data Reference](/Python_Data_Analyzer/Flight_Data_Reference.md) documents for more details on analyzing the log file.

## To Implement
- Create a Python program to then process the logged data (csv format) from the MCU!
- Create function to determine timestamps for each stage/phase of flight based on sensor data:
    - Stationary/pad (acceleration = 1g in gravitational direction, pressure at or near its maximum).
    - Boost/launch (boost shock on acceleration data).
    - Burnout/coast (burnout should show up as large drop in acceleration data, you stop accelerating upward, derivative of acceleration (jerk) usually has large spike since you suddenly stop accelerating).
    - Apogee (acceleration will switch to 0g, unless you have a parachute, pressure will reach its minimum).
    - Descent/free fall (all accelerations will be 0g).
    - Landing (landing shock, acceleration returns to constant 1g in gravitational direction, pressure returns to at or near max).
- For all the following remember to include initial condition, and subtract off initial DC bias, etc.
- Estimate velocity along longitudinal axis by doing numerical integration from acceleration data
    - If you subtract out initial bias during stationary/pad stage, that will remove both the bias and gravitational acceleration, which is what you want.  (Note: See the trapezoidal rule for numerical integration).
- Estimate altitude from pressure data, consider accounting for non-standard temperature lapse rate etc.  Also consider applying filtering or averaging etc. if needed.
- Estimate vertical velocity from pressure/altitude/position data by taking the derivative.
- For more accurate vertical velocity: determine orientation using a quaternion (possibly use scipy.spatial.transform.Rotation) based on the gyroscope data, then combine that with the acceleration data to determine actual vertical axis (will be a combination of all three axes), then numerically integrate that to determine estimate vertical velocity.  Compare this to the vertical velocity determined from the pressure data.
- Graph/plot all this data vs time, as well as the raw acceleration, temperature, and pressure data.  Include the phases/stages of flight markers.