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
| bit 1 | bit 2 | bit 3 | remote flag | command | implemented |
| --------------- | --------------- | --------------- | --------------- | --------------- | -------------- |
| 0 | 0 | 0 | 0 | set threshold to data, set sensor threshold to data  | [] |
| 0 | 0 | 0 | 1 | request threshold, sensor sends threshold value | [] |
| 0 | 0 | 1 | 0 | set hystereses to data, set sensor hystereses to data  | [] |
| 0 | 0 | 1 | 1 | request hystereses, sensor sends hystereses value | [] |
| 0 | 1 | 0 | 1 | request moisture level, sensor send moisture level | [] |


```
```
```
