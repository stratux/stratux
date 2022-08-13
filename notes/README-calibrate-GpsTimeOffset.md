# For developer:

## How to find GPSTimeOffsetPPS for GPS devices

in `main/gps/osTime.go` set `SHOW_TIME_DIFFERENCE_ONLY` to true
```go
			const SHOW_TIME_DIFFERENCE_ONLY = true
```
This will prevent stratux from setting the time based on GPS, but will print time offset.

For installing `chrony` see below.
On a seperate terminal run `sudo chronyd -n` and let it run for a while, Yet another terminal run `watch chronyc tracking` to see if it's operational.
You should beable to see that it's tracking time well and that the offset is fairly small, look for `Root dispersion`

#### Example of tracked time good enough for stratux
```
Every 2.0s: chronyc tracking                                                                                                                                            stratux: Thu Jul 28 20:46:16 2022

Reference ID    : 55BFC1F8 (85.191.193.248)
Stratum         : 3
Ref time (UTC)  : Thu Jul 28 19:46:00 2022
System time     : 0.000099761 seconds fast of NTP time
Last offset     : +0.000108447 seconds
RMS offset      : 0.008619240 seconds
Frequency       : 15.880 ppm fast
Residual freq   : +1.650 ppm
Skew            : 3.180 ppm
Root delay      : 0.037157908 seconds
Root dispersion : 0.001388310 seconds
Update interval : 64.2 seconds
Leap status     : Normal
```

Now start running stratux and let it run for a while so stratux will try to set time (but actually won't). Since the time is correctly synced by chrony,
we should see a small offset in the form of : `2022/08/02 19:58:29 PPS Calibration mode: Based on your time source (Chrony??): Use GPSTimeOffsetPPS=245ms for your device`,
this is the time Stratux the thinks it should be. If the value is around `76` ms, you can set the `GPSTimeOffsetPPS`
to `76 * time.Millisecond`. It does not have to be exact for our purpose, just good enough...

The delay is caused by various delays in the chain, serial lines, baudrates and other items that might scew the clock otherwise, hence the correction in the form of `GPSTimeOffsetPPS`.

Note: GPS devices often have a PPS pin, that's a specific PIN used by the GPS to give any microcontroller exact timing information. Unfortunatly we cannot use this and we have to use the NMEA sentence for it, but that is good enough for stratux.


## How to verify if stratux set time correctly

Change SHOW_TIME_DIFFERENCE_ONLY so stratux will start setting time again
```go
			const SHOW_TIME_DIFFERENCE_ONLY = false
```
run `chrony` with like this `sudo chronyd -n -x` and on a seperate terminal run `watch chronyc tracking`

Let stratux run for a few minutes, the chronyc command should show a very Root dispersion, in the order of < 25ms

You can also enable the line `log.Printf("Difference %v moving average %.2fms\n", time.Since(newTime), movingTimeDifference)` to
get an idea of what stratux things the difference in time is.

## To install `chronyc`

Note: We must disable `chrony` so it will not affect operation of stratux.

```bash
sudo apt-get install chronyc
sudo systemctl disable chrony
```
