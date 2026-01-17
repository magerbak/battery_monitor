# ESP32-S3 12V Battery Monitor
Records a 2 day history of battery voltages on two analog pins.

Implemented using the Arduino IDE for the Adafruit ESP32-S3 Reverse TFT Feather although I anticipate the code would port pretty easily for most ESP32 modules.
* https://www.adafruit.com/products/5691

## Operation
Initial screen is a summary display of both battery voltages.

D1 button toggles display of a battery history page, showing voltage over 3hrs, 24hrs or 2 days. 

From the history page, D0 toggles between BAT1 (engine) and BAT2 (house). 

D2 enters options menu:
* Show Stats - controls whether history page displays min/max/avg voltage over history.
* Dynamic Scale - controls whether vertical axis sizes dynamically or not.
* History - cycles displayed history between 3hrs, 24hrs and 2 days (does not affect recorded data).
* Back - exits the options menu.

After 15 minutes of no button presses, the device enters deep sleep and then periodically wakes up, samples voltage and then returns to deep sleep.
Each sample is an average of 50 analog readings taken over 5s to reduce noise.

Data preserved between sleep cycles is located in RTC memory (limited to 8KB) which limits length of history (or sample rate).

## Power Usage
* 13V current draw when active (display on) is 37.3mA (485mW).
* 13V current draw when awake and idle is 26.5mA (345mW).
* 13V current draw when in deep sleep is 0.25mA (3.25mW).

We are awake for ~6s every HISTORY_SAMPLE_INTERVAL_SECS. At the default sample interval of 5 mins, the awake ratio is 1.96%, so average current draw is:
> 0.0196*37.3 + (1-0.0196)*0.25 = ~0.976mA (12.7mW)

Therefore average current draw from a nominal 12V battery is ~1mA. Leaving this battery monitor connected to a 100Ah battery would deplete it by 10% in 10,000 hours or a little over 1 year.

