# ESP32-S3 12V Battery Monitor
Records a 2 day history of battery voltages on two analog pins.

Implemented using the Arduino IDE for the Adafruit ESP32-S3 Reverse TFT Feather although I anticipate the code would port pretty easily for most ESP32 modules.
* https://www.adafruit.com/products/5691

## Operation
There are three top-level pages: a summary of both battery voltages, and a rolling history of each battery.

When active, the current voltages are updated every 2.5s. The voltage history records a sample every 5 minutes.
* After 15 minutes of no button activity, the device enters idle mode, switches off the display and enters deep sleep. In this state it wakes up every 5 minutes, without enabling the display, to record another voltage sample and then returns to deep sleep.
* While idle, the D1 or D2 buttons can be used to return to active mode and enable the display.

Data preserved between sleep cycles is located in RTC memory (limited to 8KB) which limits length of history (or sample rate).

### Summary Page
A summary display of both battery voltages and their percent state of charge (based on typical AGM battery chemistry).

![PXL_20260126_001109261](https://github.com/user-attachments/assets/11b304bd-0ffb-4584-96aa-58aee903571a)

* D1 (middle) button enters calibration page (see below).
* D2 (bottom) button cycles between summary page and voltage history for battery 1 and 2.

### Battery History Page
Displays voltage history for a single battery.

![PXL_20260117_213506372 PORTRAIT ORIGINAL](https://github.com/user-attachments/assets/7bc5ef19-2b25-4e54-b3f4-1460b4cc0b57)

* D0 (top) button cycles between displaying 3hr, 24hr and 2 day history.
* D1 (middle) button enters options menu (see below).
* D2 (bottom) button cycles between summary page and voltage history for battery 1 and 2.

### History Options Page
The options menu allows the history display to be customized.

![PXL_20260225_011426824](https://github.com/user-attachments/assets/dbb7af0f-0d35-4fcb-9f24-9cdbc3503d50)

* Show Stats - controls whether history page displays min/max/avg voltage over history.
* Dynamic Scale - controls whether vertical axis scales dynamically or always shows 11-15V.
* Back - exits the options menu.
  
To navigate use:
* D0 (top) cursor up
* D1 (middle) cursor down
* D2 (bottom) select

### Calibration Page
Allows the factory calibration of the ADC to be adjusted to compensate for small errors. The setting is saved persistently in flash.

![PXL_20260225_005056576](https://github.com/user-attachments/assets/28ebab39-e756-4233-9f44-80843a772ccd)

* D0 (top) increase voltage reading by 2%
* D1 (middle) decrease voltage reading by 2%
* D2 (bottom) save changes. Press again to confirm, any other key to cancel.

## Power Usage
Power measurements were using a 13V power supply and included the ESP32-S3 Feather, a 12-5V buck converter and resistor dividers for the ADC inputs.
* Current draw when active (display on): 37.3mA (485mW).
* Current draw when awake and idle (display off): 26.5mA (345mW).
* Current draw in deep sleep: 0.25mA (3.25mW).

Wake time is approximately 3.5s every HISTORY_SAMPLE_INTERVAL_SECS. At the default sample interval of 5mins, the awake ratio is 1.17%, so average current draw is:
> 0.0117*37.3 + (1-0.0117)*0.25 = 0.682mA (8.87mW)

## Links
See the [wiki](https://github.com/magerbak/battery_monitor/wiki) for more information on design, schematics, 3D models and CAD files.
Theoretically, leaving this battery monitor connected for one year would consume about 6Ah of battery capacity.

