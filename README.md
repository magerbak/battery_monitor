# ESP32-S3 12V Battery Monitor
Records a 2 day history of battery voltages on two analog pins.

Implemented using the Arduino IDE for the Adafruit ESP32-S3 Reverse TFT Feather although I anticipate the code would port pretty easily for most ESP32 modules.
* https://www.adafruit.com/products/5691

## Operation
### Summary Page
Initial screen is a summary display of both battery voltages.

![PXL_20260126_001109261](https://github.com/user-attachments/assets/11b304bd-0ffb-4584-96aa-58aee903571a)


* D1 button toggles between Summary and Battery History page

### Battery History Page
Displays voltage history over 3hrs, 24hrs or 2 days.

![PXL_20260117_213506372 PORTRAIT ORIGINAL](https://github.com/user-attachments/assets/7bc5ef19-2b25-4e54-b3f4-1460b4cc0b57)

* D0 button toggles between BAT1 (engine) and BAT2 (house). 
* D1 button toggles between Summary and Battery History page
* D2 enters Options menu:
  * Show Stats - controls whether history page displays min/max/avg voltage over history.
  * Dynamic Scale - controls whether vertical axis scales dynamically or always shows 11-15V.
  * History - cycles between 3hrs, 24hrs and 2 days of history (does not affect recorded data).
  * Back - exits the options menu.

After 15 minutes of no button activity, the device enters deep sleep and then wakes up every 5 minutes, samples voltage and then returns to deep sleep.
Each sample is an average of 50 analog readings taken over 5s to reduce noise.

Data preserved between sleep cycles is located in RTC memory (limited to 8KB) which limits length of history (or sample rate).

## Power Usage
Power measurements were using a 13V power supply and included the ESP32-S3 Feather and a 12-5V buck converter
* Current draw when active (display on): 37.3mA (485mW).
* Current draw when awake and idle (display off): 26.5mA (345mW).
* Current draw in deep sleep: 0.25mA (3.25mW).

Device is awake for ~6s every HISTORY_SAMPLE_INTERVAL_SECS. At the default sample interval of 5 mins, the awake ratio is 1.96%, so average current draw is:
> 0.0196*37.3 + (1-0.0196)*0.25 = ~0.976mA (12.7mW)

Therefore average current draw from a nominal 12V battery is ~1mA. Theoretically, leaving this battery monitor connected to a 100Ah battery would deplete it by 10% in 10,000 hours or a little over 1 year.

