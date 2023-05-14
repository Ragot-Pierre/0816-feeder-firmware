# Servo speed control after v0.4

All feeders have a settle time (U parameter) configuration. This is safe and slow. All servo has an operating speed. If the firmware could calculate the servo position then the settle time is only a safety parameter.

Another advantage is that if the firmware calculates the position, it is also possible to execute slower advance than the operating speed, preventing component jumping.

All servo has an operating speed. For example SG90 servo it's about 0.1s/60° (or 600°/s).

The firmware use degree/ms. So 60°/0.1s is 60°/100ms is 0.6°/1ms.

## M620 two speed parameter

The new parameters are in degrees/milliseconds (°/ms) unit. It's a float number.

- It's range is from: 0.004 to 256 °/ms range. (4°/s .. 256000°/s)
- Resolution is 1/256 °/ms.
- 0 value means speed control disabled (default value)

After parameter write a rounding is applied and stored.

If speed control used, then the minimum speed for advance and retract is the max speed, that the servo can handle at that direction.  
In this case the settle time could lower to 30..50 ms, because servo signal repeat time and motor movement is settle after 30..50 ms.

### S parameter

Advance speed in °/ms. It's minimum the operating speed, but can be slower.

### R parameter

Retract speed in °/ms. It's minimum the operating speed.

## Example

With half slowed advance and full speed retract on SG90:

M620 N0 A90 B44 C15 F4 **S0.301 R0.602 U30** V544 W2440 X0

S0.301 = Advance speed: 30.1 °/0.1s
S0.602 = Retract speed: 60.2 °/0.1s
U30 = Settle time: 30 ms

For a 90 degree advance this is: 299 + 30 ms = 329 ms
For a 90 degree retract this is: 150 + 30 ms = 180 ms

The default settings are:
S0.000 R0.000 U240

Advance and retract speed disabled, settle time 240 ms
