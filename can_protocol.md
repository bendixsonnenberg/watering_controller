# CAN Bus Id Format
This document shows the commands that can be sent by can bus, which the moisture sensors will obey
## General Format
We use a standard frame Id, which consists of 11 bits. 
``` 
111 1111 1111
```
The later 8 bits identify which moisture sensor is addressed, we can therefore address up to 254, with adress 0 adressing the control unit sensors.
The first 3 bits together with the remote flag determine the command. 
## commands
the last byte of the message body always contains the src id

|command| contents| description|
|0|threshold_low;threshold_high;backoff_time_low; backoff_time_high| the state of the settings|
|1|red_low;red_high;green_low;geen_high;blue_low;blue_high;flags|Light settings flags: 1:LIGHT_RANDOM(rgb ignored) 2: LIGHT_OFF(rgb ignored) 4: LIGHT_RGB|
|2|moisture_low;moisture_high(u16);temperature_low;temperature_high(i16);humidity_low;humidity_high(u16);flags|Sensor readings, the flags byte describes which sensors do exist, low endian in the same order as the bytes in the message body|
|5|flags;src_id|Announce flags are the same as for sensors|

the flags are combined with bitwise or into a single byte, therefore the order of 1, 2, 4 (seperate bits)
