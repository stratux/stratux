# For developer:

## How to find GpsTimeOffsetPpsMs for GPS devices

in `main/gps.go` in the function `systemTimeSetterHandler` after the following line add a continue
```go
			log.Printf("setting system time from %s to: '%s' difference %s\n", time.Now().Format("20060102 15:04:05.000"), setStr, time.Since(newTime))
			continue // <== Add this line
			...
			...
			log.Printf("Time not set, difference %v\n", time.Since(newTime)) // <== Enable this line
```
This will prevent stratux from setting the time based on GPS, but will print time offset.

For installing `chrony` see below.
On a seperate terminal run `sudo chronyd -n` and let it run for a while, Yet another terminal run `watch chronyc tracking` to see if it's operational.
You should beable to see that it's tracking time well.

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

For the GPS device you want to find out a correct `GpsTimeOffsetPpsMs` set the value to `0 * time.Millisecond`
Now start running stratux and let it run so stratux will try to set time (but actually won't). Since the time is correctly synced by chrony,
we should see a small offset in the form of : `2022/07/28 20:42:45 Time not set, difference 76.156638ms`,
this is the time Stratux the thinks it should be. If the value is around `76` ms, you can set the `GpsTimeOffsetPpsMs`
to `76 * time.Millisecond`. It does not have to be super presice for our purpose, just good enough...

The delay is caused by various delays in the chain, serial lines, baudrates and other items that might scew the clock otherwise,
hence the correction in the form of `GpsTimeOffsetPpsMs`.

## To install `chronyc`

Note: We must disable `chrony` so it will not affect operation of stratux.

```bash
sudo apt-get install chronyc
sudo systemctl disable chrony
```
