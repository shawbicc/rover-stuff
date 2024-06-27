# rover-flysky

Folder for all the Arduino codes developed for controlling the rover with FlySky RC transceiver. 

> **Note**: Always use the latest iteration available. If the latest iteration does not work, the PCB for wheel board or wiring scheme has probably changed.

## Current Pinout

Updated for *navigation_flysky_v4.ino* 

|   BTS  | Arduino |
|--------|---------|
| ARPWM  |    6    | 
| ALPWM  |    9    | 
| A1ENRF |    7    | 
| A2ENBL |   12    | 
| BRPWM  |    3    | 
| BLPWM  |    5    | 
| B1ENLF |   A3    | 
| B2ENBR |    2    | 

| FlySky FS-iA6b | Arduino |
|----------------|---------|
| Channel 1 (ch1)|   10    |
| Channel 2 (ch2)|   11    |