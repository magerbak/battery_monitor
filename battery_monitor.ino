/**************************************************************************
  Records a 2 day history of battery voltage on two analog pins.

  Implemented for the Adafruit ESP32-S3 Reverse TFT Feather
    ----> https://www.adafruit.com/products/5691

  At initial startup or when woken with a button-press, displays a summary of
  current battery voltages.

  D1 button toggles display of battery history. From the history page, D0 toggles
  between BAT1 and BAT2. D2 enters options menu.

  After a specified idle interval, enters deep sleep and then periodically wakes
  up, samples voltage and then returns to deep sleep.

  Data preserved between sleep cycles is located in RTC memory (limited to 8KB) which
  limits length of history (or sample rate).

  13V current draw when active (display on) is 37.3mA (0.485 W).
  13V current draw when awake and idle is 26.5mA (0.345 W).
  13V current draw when in deep sleep is 0.25mA (0.00325 W).

  We are awake for ~6s every HISTORY_SAMPLE_INTERVAL_SECS. At 5mins, the awake
  ratio is 1.96%, so average current draw is:
    0.0196*37.3 + (1-0.0196)*0.25 = ~0.976mA (12.7W)

  Therefore average power draw from a nominal 12V battery is ~1mA.

  Deep sleep power could possibly be improved by shutting down more peripherals. It's
  not immediately obvious how much already gets disabled by default.

  Uses the Adafruit GFX library and the ST7789 display driver.

 **************************************************************************/
//#define TESTING

#include "driver/rtc_io.h"   // For low level RTC config for deep sleep
                             //
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

#include "debounced_button.h"
#include "battery.h"
#include "simple_timer.h"
#include "avg_data_history.h"

#define BUTTON_D0_PIN       GPIO_NUM_0
#define BUTTON_D1_PIN       GPIO_NUM_1
#define BUTTON_D2_PIN       GPIO_NUM_2

#define BAT1_ADC_PIN        A1
#define BAT2_ADC_PIN        A0

// For each average sample in history we average data samples every INTER_SAMPLE_INTERVAL_MS
// for HISTORY_AVG_INTERVAL_MS.
#define INTER_SAMPLE_INTERVAL_MS    100
#define HISTORY_AVG_INTERVAL_MS     5000

// Every HISTORY_SAMPLE_INTERVAL_SECS we record an averaged sample. When idle,
// we expect to sleep for the balance of this interval.
#ifdef TESTING
    // For testing we pretend time has sped up 20x
    #define HISTORY_SAMPLE_INTERVAL_SECS    (15)
    #define IDLE_TIMEOUT_MS                 (1 * 60 * 1000)
#else
    #define HISTORY_SAMPLE_INTERVAL_SECS    (5 * 60)
    #define IDLE_TIMEOUT_MS                 (15 * 60 * 1000)
#endif

// 2 days, assuming sampling every 5mins.
// This consumes 4608 bytes of the 8kB RTC memory
#define HISTORY_NUM_DATA_POINTS         (2 * 24 * 12)

#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  135

// UI color coding
#define GREEN_PSOC      70.0
#define YELLOW_PSOC     50.0
#define RED_PSOC        0.0

enum BatteryId {
    BAT1,
    BAT2,
    NUM_BATTERIES
};

enum Event {
    EVT_NONE,
    EVT_D0_PRESS,
    EVT_D0_RELEASE,
    EVT_D1_PRESS,
    EVT_D1_RELEASE,
    EVT_D2_PRESS,
    EVT_D2_RELEASE,
};

enum Page {
    PAGE_NONE,
    PAGE_SUMMARY,
    PAGE_HISTORY,
    PAGE_OPTIONS,
    PAGE_END
};

// Make corresponding edits to g_optionsTable
enum Options {
    OPT_SHOW_STATS,
    OPT_DYNAMIC_SCALE,
    OPT_RANGE,
    OPT_BACK,
};

// Make corresponding edits to g_rangeOptionsTable and displayHistory()
enum HistRange {
    RANGE_3_HRS,
    RANGE_24_HRS,
    RANGE_2_DAYS,
    NUM_RANGE_OPTIONS
};

enum TextLayout {
    TXT_JUSTIFIED,
    TXT_CENTERED,
};

struct HistWindowContext {
    // Position of top-left of history window.
    int16_t x;
    int16_t y;
    // Width and height of history window.
    int16_t w;
    int16_t h;

    size_t count = 0;
    double prev_val = 0.0;

    unsigned int min_x; // Minimum x axis data offset
    double min_y;       // Minimum y axis value
    unsigned int max_x; // Maximum x axis data offset
    double max_y;       // Maximum y axis value
    size_t size = 0;    // Max num history samples
};

struct HistStatsContext {
    bool bFirst = true;

    unsigned int min_x = 0; // Minimum x axis data offset
    unsigned int max_x = 0; // Maximum x axis data offset
    size_t size = 0;    // Max num history samples

    // Stats for current history samples.
    double min = 0.0;
    double max = 1.0;
    double avg = 0.0;
    int count = 0;
};

struct HistOptions {
    int cursor = OPT_SHOW_STATS;

    bool bShowStats = true;
    bool bDynamicScale = true;
    HistRange range = RANGE_3_HRS;
};

const char* g_optionsTable[OPT_BACK + 1] = {
    "Show Stats ",
    "Dynamic Scale ",
    "History ",
    "Back"
};

const char* g_rangeOptionsTable[NUM_RANGE_OPTIONS] = {
    "3hrs",
    "24hrs",
    "2days",
};

// Use dedicated hardware SPI pins
Adafruit_ST7789 g_tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

SimpleTimer g_samplingTimer;
SimpleTimer g_historyTimer;

// UI buttons. On the Adafruit ESP32-S3 reverse TFT feather, these are on pins D0, D1 and D2.
DebouncedButton g_buttonD0(BUTTON_D0_PIN, LOW);
DebouncedButton g_buttonD1(BUTTON_D1_PIN);
DebouncedButton g_buttonD2(BUTTON_D2_PIN);

// We are active when the user has interacted with us recently. Display is on
// and we don't go to sleep until we're idle.
bool g_bActive = false;
uint32_t g_lastActive = 0;

// UI state preserved during deep sleeps
RTC_DATA_ATTR Page g_page = PAGE_NONE;
RTC_DATA_ATTR BatteryId g_selBattery = BAT2;
RTC_DATA_ATTR HistOptions g_histOptions;

RTC_DATA_ATTR float g_bat1History[HISTORY_NUM_DATA_POINTS];
RTC_DATA_ATTR float g_bat2History[HISTORY_NUM_DATA_POINTS];

// Battery state
RTC_DATA_ATTR Battery g_batteries[NUM_BATTERIES];

void handleButtonEvents(Event e);
bool samplingCallback(void* user);
bool updateCallback(void* user);

uint16_t getPSoCColor(float psoc);
void drawJustifiedText(const char* str, int x, int y, TextLayout fmt);
void drawJustifiedVal(double val, int precision, const char* suffix, int x, int y, TextLayout fmt);

void statsHistoryCallback(void* user, const double* dataMin, const double* dataMax,
                          size_t len, size_t offset);
void drawHistoryCallback(void* user, const double* dataMin, const double* dataMax,
                         size_t len, size_t offset);
void displayHistory(const char* title, double val, const AvgDataHistory<double>* hist);

void displaySplashScreen();

void displayUpdate();

////////////////////////////////////////////////////////////////////////////

void setup(void) {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("Starting battery monitor"));

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED) {
      // Cold boot (power loss or hard reset)
      Serial.println("Cold boot");
      g_bActive = true;
      g_lastActive = millis();

      g_batteries[BAT1].begin("Engine", BAT1_ADC_PIN, g_bat1History, HISTORY_NUM_DATA_POINTS,
                              HISTORY_AVG_INTERVAL_MS / INTER_SAMPLE_INTERVAL_MS);
      g_batteries[BAT2].begin("House",  BAT2_ADC_PIN, g_bat2History, HISTORY_NUM_DATA_POINTS,
                              HISTORY_AVG_INTERVAL_MS / INTER_SAMPLE_INTERVAL_MS);
  }
  else if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
      // Button press. We're active but batteries don't need to be initialized.
      Serial.println("Wake from buttonpress");
      g_bActive = true;
      g_lastActive = millis();
  }
  else {
      // ESP_SLEEP_WAKEUP_TIMER
      Serial.println("Wake from timer");
      g_bActive = false;
  }

  // Deconfigure RTC config of button pins during deep sleep
  rtc_gpio_deinit(BUTTON_D1_PIN);
  rtc_gpio_deinit(BUTTON_D2_PIN);


  g_buttonD0.begin();
  g_buttonD1.begin();
  g_buttonD2.begin();

  g_samplingTimer.begin(nullptr, INTER_SAMPLE_INTERVAL_MS, samplingCallback);
  // First sample is after initial averaging period. If we're active, subsequent
  // period is set to HISTORY_SAMPLE_INTERVAL_SECS.
  g_historyTimer.begin(&g_historyTimer, HISTORY_AVG_INTERVAL_MS, updateCallback);

  // ADC setup
  pinMode(BAT1_ADC_PIN, INPUT);
  pinMode(BAT2_ADC_PIN, INPUT);

  if (g_bActive) {
      initDisplay();
  }
}

void loop() {
    uint32_t t = millis();

    // Drive our timers
    g_samplingTimer.tick(t);
    g_historyTimer.tick(t);

    // Poll buttons and drive UI
    if (g_buttonD0.updateState()) {
        handleButtonEvents(g_buttonD0.isPressed() ? EVT_D0_PRESS : EVT_D0_RELEASE);
    }
    if (g_buttonD1.updateState()) {
        handleButtonEvents(g_buttonD1.isPressed() ? EVT_D1_PRESS : EVT_D1_RELEASE);
    }
    if (g_buttonD2.updateState()) {
        handleButtonEvents(g_buttonD2.isPressed() ? EVT_D2_PRESS : EVT_D2_RELEASE);
    }
}

////////////////////////////////////////////////////////////////////////////

void handleButtonEvents(Event e) {
    // D0 cycles selecting starter, house battery.
    // D1 toggle between summary and history
    // D2 option menu (display stats on/off, dynamic scale on/off, 3hr/24hr/7day history, back).

    if (!g_bActive) {
        g_bActive = true;
        g_lastActive = millis();

        initDisplay();
    }

    switch (g_page) {
        case PAGE_NONE:
            return;

        case PAGE_SUMMARY:
            switch (e) {
                case EVT_D1_PRESS:
                    g_page = PAGE_HISTORY;
                    break;

                default:
                    return;
            }
            break;

        case PAGE_HISTORY:
            switch (e) {
                case EVT_D0_PRESS:
                    {
                        int b = g_selBattery;
                        if (++b == NUM_BATTERIES) {
                            b = 0;
                        }
                        g_selBattery = (BatteryId)b;
                    }
                    break;

                case EVT_D1_PRESS:
                    g_page = PAGE_SUMMARY;
                    break;

                case EVT_D2_PRESS:
                    g_histOptions.cursor = OPT_SHOW_STATS;
                    g_page = PAGE_OPTIONS;
                    break;

                default:
                    return;
            }
            break;

        case PAGE_OPTIONS:
            switch (e) {
                case EVT_D0_PRESS:
                    if (g_histOptions.cursor > 0) {
                        g_histOptions.cursor--;
                    }
                    break;

                case EVT_D1_PRESS:
                    if (g_histOptions.cursor < OPT_BACK) {
                        g_histOptions.cursor++;
                    }
                    break;

                case EVT_D2_PRESS:
                    switch (g_histOptions.cursor) {
                        case OPT_SHOW_STATS:
                            g_histOptions.bShowStats = !g_histOptions.bShowStats;
                            break;

                        case OPT_DYNAMIC_SCALE:
                            g_histOptions.bDynamicScale = !g_histOptions.bDynamicScale;
                            break;

                        case OPT_RANGE:
                            {
                                int r = g_histOptions.range;
                                if (++r == HistRange::NUM_RANGE_OPTIONS) {
                                    r = 0;
                                }
                                g_histOptions.range = (HistRange)r;
                            }
                            break;

                        case OPT_BACK:
                            g_page = PAGE_HISTORY;
                            break;
                    }
                    break;

                default:
                    return;
            }
            break;
    }

    displayUpdate();
}


bool samplingCallback(void* user) {

    (void)user;

    bool bDone1 = g_batteries[BAT1].updateVoltageData();
    bool bDone2 = g_batteries[BAT2].updateVoltageData();

    if (g_bActive && (bDone1 || bDone2)) {
        if (g_page == PAGE_NONE) {
            g_page = PAGE_SUMMARY;
        }
        displayUpdate();
    }

    // Continue running
    return true;
}

bool updateCallback(void* user) {

    SimpleTimer* pTimer = (SimpleTimer*)user;

#ifdef TESTING
    Serial.print("Update ");
    Serial.println(g_batteries[BAT2].getVoltage(), 2);
#endif

    // Update the voltage history with the latest average.
    g_batteries[BAT1].updateVoltageHistory();
    g_batteries[BAT2].updateVoltageHistory();

    uint32_t t = millis();
    if (g_bActive == false || (t - g_lastActive) > IDLE_TIMEOUT_MS) {
        g_bActive = false;

        Serial.println("Sleep ");
        delay(100);
        // Deep sleep

        // Wake up on button press of D1 or D2 (can't use D0 because it's active low
        // on the the TFT reverse feather).
        rtc_gpio_pullup_dis(BUTTON_D1_PIN);
        rtc_gpio_pulldown_en(BUTTON_D1_PIN);
        rtc_gpio_pullup_dis(BUTTON_D2_PIN);
        rtc_gpio_pulldown_en(BUTTON_D2_PIN);
        esp_sleep_enable_ext1_wakeup_io(BIT(BUTTON_D1_PIN) |
                                        BIT(BUTTON_D2_PIN),
                                        ESP_EXT1_WAKEUP_ANY_HIGH);

        // We want to wake up every HISTORY_SAMPLE_INTERVAL_SECS and take a sample.
        // However, we need to take account for the startup and averaging time
        // to avoid a time drift.
        esp_sleep_enable_timer_wakeup(1000ULL * (HISTORY_SAMPLE_INTERVAL_SECS * 1000 -
                                                 (HISTORY_AVG_INTERVAL_MS + 1000)));
        esp_deep_sleep_start();
        // No return from this
    }

    pTimer->setInterval(HISTORY_SAMPLE_INTERVAL_SECS * 1000);
    // Continue running (if we are still active)
    return true;
}

void initDisplay() {
    // turn on backlite
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);

    // turn on the TFT / I2C power supply
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
    delay(10);

    // initialize TFT
    g_tft.init(DISPLAY_HEIGHT, DISPLAY_WIDTH);
    g_tft.setRotation(3);

    Serial.println(F("TFT Initialized"));

    displayUpdate();
}

// Determine the UI color for a given percent state of charge
uint16_t getPSoCColor(float psoc) {

    if (psoc >= GREEN_PSOC) {
        return ST77XX_GREEN;
    }
    else if (psoc >= YELLOW_PSOC) {
        return ST77XX_YELLOW;
    }
    else {
        return ST77XX_RED;
    }
}

// Helper function to print text using a calculated starting position based on
// its length and a layout option.
void drawJustifiedText(const char* str, int x, int y, TextLayout fmt) {
    int16_t minx;
    int16_t miny;
    uint16_t w;
    uint16_t h;

    g_tft.getTextBounds(str, 0, 0, &minx, &miny, &w, &h);
    switch (fmt) {
        // x and y are either 0 or the rightmost/bottommost extent of the justified
        // text respectively. This allows text to be placed flush with the edge of
        // the display in all 4 corners.
        case TXT_JUSTIFIED:
            if (x) {
                x -= w + 1;
            }
            if (y) {
                y -= h + 1;
            }
            break;

        case TXT_CENTERED:
            // Note y is not adjusted, to allow manual vertical placement.
            x -= (w + 1) / 2;
            break;
    }
    g_tft.setCursor(x, y);
    g_tft.print(str);
}

// Helper function to print a floating point value using a calculated starting
// position based on its displayed length and a layout option.
void drawJustifiedVal(double val, int precision, const char* suffix, int x, int y, TextLayout fmt) {
    char str[64];

    if (!suffix) {
        suffix = "";
    }

    int rc = snprintf(str, sizeof(str), "%.*f%s", precision, val, suffix);
    drawJustifiedText(str, x, y, fmt);
}


// Callback to calculate statistics of history data.
void statsHistoryCallback(void* user, const float* data, size_t len, size_t offset) {
    HistStatsContext* ctx = (HistStatsContext *)user;

    for (unsigned int i = 0; i < len; i++) {
        if (offset + i < ctx->min_x) {
            continue;
        }
        if (offset + i >= ctx->max_x) {
            break;;
        }

        if (ctx->bFirst) {
            ctx->max = data[i];
            ctx->min = data[i];
        }
        else {
            if (ctx->max < data[i]) {
                ctx->max = data[i];
            }
            if (ctx->min > data[i]) {
                ctx->min = data[i];
            }
        }
        ctx->avg += data[i];
        ctx->count++;
        ctx->bFirst = false;
    }

    if (ctx->count && len && offset + len == ctx->size) {
        // We're finished.
        ctx->avg = ctx->avg / ctx->count;
    }
}

// Callback to draw history data scaled to the display window defined in the
// context structure. Note that if data points exceed range_x (in number) or
// range_y (in value) then drawing will exceed the defined window.
void drawHistoryCallback(void* user, const float* data, size_t len, size_t offset) {
    HistWindowContext* ctx = (HistWindowContext *)user;

    // Scale the number of samples to fit the display window.
    double scale_x = ctx->w / (double)(ctx->max_x - ctx->min_x);
    double scale_y = ctx->h / (double)(ctx->max_y - ctx->min_y);

    // This ensures we have seamless transition between first and second callback
    // after the history has wrapped.
    int prev_idx;

    for (unsigned int i = 0; i < len; i++) {
        if (offset + i < ctx->min_x) {
            continue;
        }
        if (offset + i >= ctx->max_x) {
            break;;
        }

        uint16_t color = getPSoCColor(Battery::calcPSoC(data[i]));
        int16_t val = round((data[i] - ctx->min_y) * scale_y);

        if (ctx->count == 0) {
            prev_idx = i;
            ctx->prev_val = val;
        }
        else {
            prev_idx = i - 1;
        }

        g_tft.drawLine(ctx->x + round((offset - ctx->min_x + prev_idx) * scale_x), ctx->y + ctx->h - ctx->prev_val,
                       ctx->x + round((offset - ctx->min_x + i) * scale_x), ctx->y + ctx->h - val,
                       color);

        ctx->prev_val = val;
        ctx->count++;
    }
}

// Display history
void displayHistory(const Battery* bat) {
    struct HistStatsContext hist_stats;
    struct HistWindowContext hist_window;
    const AvgDataHistory<float>* hist = bat->getHistory();

    g_tft.setTextWrap(false);
    g_tft.fillScreen(ST77XX_BLACK);

    double cv = hist->getLatestData();
    double psoc = bat->calcPSoC(cv);
    uint16_t color = getPSoCColor(psoc);

    // Figure out what range of history data we are displaying.
    size_t minOffset = 0;
    size_t maxOffset = hist->getSize();
    int stepx = 10;

    switch (g_histOptions.range) {
        case RANGE_3_HRS:
            minOffset = maxOffset - (maxOffset / 24);
            stepx = DISPLAY_WIDTH / 12;
            break;

        case RANGE_24_HRS:
            minOffset = maxOffset - (maxOffset / 2);
            stepx = DISPLAY_WIDTH / 24;
            break;

        case RANGE_2_DAYS:
            minOffset = 0;
            stepx = DISPLAY_WIDTH / 8;
            break;
    }

    // Calculate history statistics.
    hist_stats.min_x = minOffset;
    hist_stats.max_x = maxOffset;
    hist_stats.size = hist->getSize();
    hist->forEachData(statsHistoryCallback, &hist_stats);

    // Figure out what vertical range of data we are displaying.
    double minValue = MIN_VOLTAGE;
    double maxValue = MAX_VOLTAGE;
    if (g_histOptions.bDynamicScale) {
        // Round min/max to next integer
        minValue = floor(hist_stats.min);
        if (minValue < MIN_VOLTAGE) {
            minValue = MIN_VOLTAGE;
        }
        maxValue = ceil(hist_stats.max);
        if (maxValue <= minValue) {
            maxValue = minValue + 1.0;
        }
    }

    hist_window.x = 0;
    hist_window.y = 0;
    hist_window.w = DISPLAY_WIDTH;
    hist_window.h = DISPLAY_HEIGHT - 3;
    hist_window.min_x = minOffset;
    hist_window.min_y = minValue;
    hist_window.max_x = maxOffset;
    hist_window.max_y = maxValue;

    double scale_x = hist_window.w / (double)(hist_window.max_x - hist_window.min_x);
    double scale_y = hist_window.h / (double)(hist_window.max_y - hist_window.min_y);

    // Vertical scale: make a crude attempt to pick appropriate values for labels.
    int stepi = maxValue - minValue;
    double stepy = (maxValue - minValue) / ((stepi == 3) ? 3.0 : 2.0);
    double v = maxValue;

    g_tft.setTextColor(ST77XX_WHITE);
    g_tft.setTextSize(1);
    while (v > minValue) {
        int16_t val = round((v - hist_window.min_y) * scale_y);

        // Label vertical scale under each line
        drawJustifiedVal(v, 1, "", DISPLAY_WIDTH, hist_window.y + hist_window.h - val + 11, TXT_JUSTIFIED);
        g_tft.drawFastHLine(0, hist_window.y + hist_window.h - val, DISPLAY_WIDTH, ST77XX_WHITE);
        v -= stepy;
    }
    // Except for last line, which goes above.
    //drawJustifiedVal(v, 1, "", DISPLAY_WIDTH, DISPLAY_HEIGHT - 3, TXT_JUSTIFIED);

    // Horizontal scale:
    g_tft.drawFastHLine(0, DISPLAY_HEIGHT - 3, DISPLAY_WIDTH, ST77XX_WHITE);
    int t;
    for (t = 0; t < DISPLAY_WIDTH; t += stepx) {
        g_tft.drawFastVLine(DISPLAY_WIDTH - t, DISPLAY_HEIGHT - 2, 2, ST77XX_WHITE);
    }

    g_tft.setTextSize(2);
    g_tft.setTextColor(color);
    g_tft.setCursor(0, 1);
    g_tft.print(bat->getName());
    g_tft.print(" ");
    g_tft.print(cv, 2);
    g_tft.print(" ");
    g_tft.print(psoc, 0);
    g_tft.println("%");

    if (g_histOptions.bShowStats) {
        // Stats at top-left
        g_tft.setTextColor(ST77XX_CYAN);
        g_tft.print("Max ");
        g_tft.println(hist_stats.max, 2);

        g_tft.print("Avg ");
        g_tft.println(hist_stats.avg, 2);

        g_tft.print("Min ");
        g_tft.println(hist_stats.min, 2);
    }

    g_tft.setTextColor(ST77XX_WHITE);
    g_tft.setTextSize(2);
    g_tft.setCursor(0, DISPLAY_HEIGHT - 3 - 16);
    drawJustifiedText(g_rangeOptionsTable[g_histOptions.range], 0, DISPLAY_HEIGHT - 3, TXT_JUSTIFIED);

    // Draw history data
    hist->forEachData(drawHistoryCallback, &hist_window);
}

void drawBatterySummary(const Battery* bat, int16_t y0) {
    char buffer[32];

    double v = bat->getHistory()->getLatestData();
    double psoc = bat->calcPSoC(v);
    uint16_t color = getPSoCColor(psoc);
    int16_t w = round(DISPLAY_WIDTH * psoc / 100.0);

    g_tft.setTextColor(ST77XX_WHITE);
    snprintf(buffer, sizeof(buffer), "%s %.2fV", bat->getName(), v);
    drawJustifiedText(buffer, DISPLAY_WIDTH / 2,  y0, TXT_CENTERED);
    g_tft.fillRect(0, y0 + 26, w, 32, color);
    g_tft.drawRect(0, y0 + 26, DISPLAY_WIDTH, 32, ST77XX_WHITE);
    g_tft.setTextColor(ST77XX_BLUE);
    drawJustifiedVal(psoc, 0, "%", DISPLAY_WIDTH / 2,  y0 + 30, TXT_CENTERED);
}

void displaySummary() {
    g_tft.setTextWrap(false);
    g_tft.fillScreen(ST77XX_BLACK);
    g_tft.setTextSize(3);

    drawBatterySummary(&g_batteries[BAT1], 0);
    drawBatterySummary(&g_batteries[BAT2], DISPLAY_HEIGHT / 2);
}

void displayOptions() {
    g_tft.setTextWrap(false);
    g_tft.fillScreen(ST77XX_BLACK);

    g_tft.setTextColor(ST77XX_WHITE);
    g_tft.setTextSize(2);
    g_tft.setCursor(0, 0);

    g_tft.print(g_optionsTable[OPT_SHOW_STATS]);
    g_tft.println(g_histOptions.bShowStats ? "Yes" : "No");
    g_tft.print(g_optionsTable[OPT_DYNAMIC_SCALE]);
    g_tft.println(g_histOptions.bDynamicScale ? "Yes" : "No");
    g_tft.print(g_optionsTable[OPT_RANGE]);
    g_tft.println(g_rangeOptionsTable[g_histOptions.range]);
    g_tft.print(g_optionsTable[OPT_BACK]);

    int16_t y = 16 * g_histOptions.cursor;
    g_tft.drawRect(0, y, DISPLAY_WIDTH, 15, ST77XX_WHITE);
}

void displaySplashScreen() {
    g_tft.setTextWrap(false);
    g_tft.fillScreen(ST77XX_BLACK);

    g_tft.setTextColor(ST77XX_WHITE);
    g_tft.setTextSize(3);
    g_tft.setCursor(10, 48);
    g_tft.print("Initializing");
}

// Redraw the display.
void displayUpdate() {

    if (g_bActive) {
        switch (g_page) {
            case PAGE_NONE:
                displaySplashScreen();
                break;

            case PAGE_SUMMARY:
                displaySummary();
                break;

            case PAGE_HISTORY:
                displayHistory(&g_batteries[g_selBattery]);
                break;

            case PAGE_OPTIONS:
                displayOptions();
                break;
        }
    }
}

