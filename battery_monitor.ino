/**************************************************************************
  Monitors analog voltage on A0 and displays a history of recorded data.

  Works with the Adafruit ESP32-S3 Reverse TFT Feather
    ----> https://www.adafruit.com/products/5691

  Uses the Adafruit GFX library and the ST7789 display driver.

 **************************************************************************/
#define TESTING

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

#include "debounced_button.h"
#include "simple_timer.h"
#include "avg_data_history.h"

#define BAT1_ADC_PIN         A1
#define BAT2_ADC_PIN         A0

// We assume we're monitoring a nominal 12v battery.
#define MAX_VOLTAGE     15.0
#define MIN_VOLTAGE     11.0

// Percent State of Charge table for a typical 12v AGM battery
// Ref: https://lifelinebatteries.com/wp-content/uploads/2015/12/6-0101G_Lifeline_Technical_Manual.pdf
//
// Adjust the values below based on specific battery chemistry and model. May need
// different tables for each battery.
const struct PSoC {
    double psoc;
    double v;
} g_psocTable[] = {
    { 100, MAX_VOLTAGE },
    { 100, 12.78 },
    {  90, 12.66 },
    {  80, 12.54 },
    {  70, 12.42 },
    {  60, 12.30 },
    {  50, 12.18 },
    {  40, 12.06 },
    {  30, 11.94 },
    {  20, 11.82 },
    {  10, 11.70 },
    {   0, 11.58 },
    {   0, 0 },
};

// UI color coding
#define GREEN_PSOC      70.0
#define YELLOW_PSOC     50.0
#define RED_PSOC        0.0


// Adjust values to measured resistance of specific resistors in your circuit.
//
// TODO. May need different tables for each battery.
//
// 12v----
//        |
//        R1
// A0-----|
//        R2
//        |
// GND----
//
// We want ADC input to be less than 3.1v, which is about 20% of MAX_VOLTAGE.
// R2 / (R1 + R2) = 18.8% is close.
#define DIVIDER_R1      200.4   // Nominally 200K Ohms
#define DIVIDER_R2      46.3    // Nominally 47K Ohms

#define IDLE_TIMEOUT_SECS       (5 * 60)

// For each average sample in history we average data samples every INTER_SAMPLE_INTERVAL_MS
// for HISTORY_AVG_INTERVAL_MS.
#define INTER_SAMPLE_INTERVAL_MS    100
#define HISTORY_AVG_INTERVAL_MS     5000

// Every HISTORY_SAMPLE_INTERVAL_SECS we record an averaged sample. When idle,
// we expect to sleep for the balance of this interval.
#ifdef TESTING
#define HISTORY_SAMPLE_INTERVAL_SECS    (5)
#define HISTORY_DURATION_SECS           (300)
#else
#define HISTORY_SAMPLE_INTERVAL_SECS    (60)
#define HISTORY_DURATION_SECS           (7 * 24 * 60 * 60)
#endif
#define HISTORY_NUM_DATA_POINTS         (HISTORY_DURATION_SECS / HISTORY_SAMPLE_INTERVAL_SECS)

// Min data points to display represents 90mins of history
#define MIN_DISPLAY_EXTENT              (5400 / HISTORY_SAMPLE_INTERVAL_SECS)

#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  135

// Use dedicated hardware SPI pins
Adafruit_ST7789 g_tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

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
    PAGE_SUMMARY,
    PAGE_HISTORY,
    PAGE_END
};

enum BatteryId {
    BAT1,
    BAT2,
    NUM_BATTERIES
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


class Battery {
public:
    Battery() = default;
    ~Battery() = default;

    void begin(const char* name, int pin, size_t histLen);

    void updateVoltageData();
    void updateVoltageHistory();

    const char* getName() const { return m_pName; }
    double getVoltage()const { return m_voltage; }
    double getPSoC() const { return m_psoc; }
    const AvgDataHistory<double>* getHistory() const { return &m_history; }

    static double calcPSoC(double voltage);
    static uint16_t getPSoCColor(double psoc);

private:
    static double readVoltage(int pin);

    const char* m_pName = nullptr;
    int m_pin = 0;

    double m_voltage = 0.0;
    double m_psoc = 0.0;
    AvgDataHistory<double> m_history;
};

// UI buttons. On the Adafruit ESP32-S3 reverse TFT feather, these are on pins 0, 1 and 2.
DebouncedButton g_buttonD0(0, LOW);
DebouncedButton g_buttonD1(1);
DebouncedButton g_buttonD2(2);

// History can be displayed over 1.5hr to HISTORY_DURATION_SECS. Measure this
// in number of samples of history.
double g_displayExtent = MIN_DISPLAY_EXTENT;

// We are active when the user has interacted with us recently. Display is on
// and we don't go to sleep until we're idle.
bool g_bActive = true;
uint32_t g_lastActive = 0;
Page g_page = PAGE_SUMMARY;
BatteryId g_selBattery = BAT2;

SimpleTimer g_samplingTimer;
SimpleTimer g_updateTimer;

Battery g_batteries[NUM_BATTERIES];

void handleButtonEvents(Event e);
bool samplingCallback(void* user);
bool updateCallback(void* user);

void displaySplashScreen();
void drawJustifiedText(const char* str, int x, int y, TextLayout fmt);
void drawJustifiedVal(double val, int precision, const char* suffix, int x, int y, TextLayout fmt);

void statsHistoryCallback(void* user, const double* dataMin, const double* dataMax,
                          size_t len, size_t offset);
void drawHistoryCallback(void* user, const double* dataMin, const double* dataMax,
                         size_t len, size_t offset);
void displayHistory(const char* title, double val, const AvgDataHistory<double>* hist);

void displayUpdate();


void setup(void) {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("Starting battery monitor"));

  g_buttonD0.begin();
  g_buttonD1.begin();
  g_buttonD2.begin();

  g_samplingTimer.begin(nullptr, INTER_SAMPLE_INTERVAL_MS, samplingCallback);
  g_updateTimer.begin(nullptr, HISTORY_AVG_INTERVAL_MS, updateCallback);

  g_batteries[BAT1].begin("Engine", BAT1_ADC_PIN, HISTORY_NUM_DATA_POINTS);
  g_batteries[BAT2].begin("House",  BAT2_ADC_PIN, HISTORY_NUM_DATA_POINTS);

  // ADC setup
  pinMode(BAT1_ADC_PIN, INPUT);
  pinMode(BAT2_ADC_PIN, INPUT);

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

  displaySplashScreen();
  delay(2000);
  Serial.println(F("TFT Initialized"));

  displayUpdate();
}

void loop() {
    uint32_t t = millis();

    // Drive our timers
    g_samplingTimer.tick(t);
    g_updateTimer.tick(t);

    // Poll buttons and drive UI("(
    if (g_buttonD0.updateState()) {
        Serial.println("d0");
        handleButtonEvents(g_buttonD0.isPressed() ? EVT_D0_PRESS : EVT_D0_RELEASE);
    }
    if (g_buttonD1.updateState()) {
        Serial.println("d1");
        handleButtonEvents(g_buttonD1.isPressed() ? EVT_D1_PRESS : EVT_D1_RELEASE);
    }
    if (g_buttonD2.updateState()) {
        Serial.println("d2");
        handleButtonEvents(g_buttonD2.isPressed() ? EVT_D2_PRESS : EVT_D2_RELEASE);
    }

    if (t - g_lastActive > IDLE_TIMEOUT_SECS) {
        // Sleep
    }
}

void handleButtonEvents(Event e) {
    // D0 cycles selecting starter, house battery.
    // D1 toggle between summary and history
    // D2 option menu (display stats on/off, dynamic scale on/off, 3hr/24hr/7day history, back).
    switch (g_page) {
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
                    // MA! TODO: Options menu
                    break;

                default:
                    return;
            }
            break;
    }

    displayUpdate();
}


//////////////////////////////////////////////////////////////////////////////

void Battery::begin(const char* name, int pin, size_t histLen) {
    m_pName = name;
    m_pin = pin;

    m_history.begin(histLen);
}

void Battery::updateVoltageData() {
    m_voltage = readVoltage(m_pin);
    m_psoc = calcPSoC(m_voltage);
    m_history.updateData(m_voltage);
}

void Battery::updateVoltageHistory() {
    m_history.updateHistory();
}

double Battery::readVoltage(int pin) {
    // Read 12-bit ADC
    int val = analogRead(pin);
    //Serial.println(val);

    // From empirical data, ADC value is approximately 1079/volt
    double adcVoltage = (2.429 * val / 2621);
    //Serial.println(adcVoltage, 2);

    // Based on resistor divider, calculate the true voltage.
    double batteryVoltage = adcVoltage * (DIVIDER_R1 + DIVIDER_R2) / DIVIDER_R2;
    //Serial.println(batteryVoltage, 2);

    return batteryVoltage;
}

double Battery::calcPSoC(double voltage) {
    // Estimate Percent State of Charge based on battery voltage.
    size_t len = sizeof(g_psocTable) / sizeof(struct PSoC);
    double psoc = g_psocTable[0].psoc;

    for (auto i = 0; g_psocTable[i].v > 0.0; i++) {
        if (voltage > g_psocTable[i + 1].v) {
            psoc = (voltage - g_psocTable[i + 1].v) / (g_psocTable[i].v - g_psocTable[i + 1].v) *
                   (g_psocTable[i].psoc - g_psocTable[i + 1].psoc) + g_psocTable[i + 1].psoc;
            return psoc;
        }
    }
    return 0.0;
}

uint16_t Battery::getPSoCColor(double psoc) {

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

//////////////////////////////////////////////////////////////////////////////

bool samplingCallback(void* user) {

    (void)user;

    g_batteries[BAT1].updateVoltageData();
    g_batteries[BAT2].updateVoltageData();

    // Continue running
    return true;
}

bool updateCallback(void* user) {

    (void)user;

    g_batteries[BAT1].updateVoltageHistory();
    g_batteries[BAT2].updateVoltageHistory();

    displayUpdate();

    // Continue running
    return true;
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
void statsHistoryCallback(void* user, const double* data, size_t len, size_t offset) {
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
void drawHistoryCallback(void* user, const double* data, size_t len, size_t offset) {
    HistWindowContext* ctx = (HistWindowContext *)user;

    // Scale the number of samples to fit the display window.
    double scale_x = ctx->w / (double)(ctx->max_x - ctx->min_x);
    double scale_y = ctx->h / (double)(ctx->max_y - ctx->min_y);

    int prev_idx = 0;

    for (unsigned int i = 0; i < len; i++) {
        if (offset + i < ctx->min_x) {
            continue;
        }
        if (offset + i >= ctx->max_x) {
            break;;
        }

        uint16_t color = Battery::getPSoCColor(Battery::calcPSoC(data[i]));
        int16_t val = round((data[i] - ctx->min_y) * scale_y);

        if (ctx->count == 0) {
            prev_idx = i;
            ctx->prev_val = round((data[i] - ctx->min_y) * scale_y);
        }

        g_tft.drawLine(ctx->x + round((offset + prev_idx) * scale_x), ctx->y + ctx->h - ctx->prev_val,
                       ctx->x + round((offset + i) * scale_x), ctx->y + ctx->h - val,
                       color);

        prev_idx = i;
        ctx->prev_val = val;
        ctx->count++;
    }
}

// Display history
void displayHistory(const Battery* bat) {
    struct HistStatsContext hist_stats;
    struct HistWindowContext hist_window;
    const AvgDataHistory<double>* hist = bat->getHistory();

    g_tft.setTextWrap(false);
    g_tft.fillScreen(ST77XX_BLACK);

    double cv = hist->getLastData();
    double psoc = bat->calcPSoC(cv);
    uint16_t color = bat->getPSoCColor(psoc);

    // Figure out what range of history data we are displaying.
    // Default to displaying entire history
    size_t minOffset = 0;
    size_t maxOffset = hist->getSize();

    // Now reduce it to the desired display extent so we can zoom in.
    if (g_displayExtent < maxOffset) {
        minOffset = maxOffset - g_displayExtent;
    }

    // Calculate history statistics.
    hist_stats.min_x = minOffset;
    hist_stats.max_x = maxOffset;
    hist_stats.size = hist->getSize();
    hist->forEachData(statsHistoryCallback, &hist_stats);

    hist_window.x = 0;
    hist_window.y = 0;
    hist_window.w = DISPLAY_WIDTH;
    hist_window.h = DISPLAY_HEIGHT - 3;
    hist_window.min_x = minOffset;
    hist_window.min_y = MIN_VOLTAGE;
    hist_window.max_x = maxOffset;
    hist_window.max_y = MAX_VOLTAGE;

    double scale_x = hist_window.w / (double)(hist_window.max_x - hist_window.min_x);
    double scale_y = hist_window.h / (double)(hist_window.max_y - hist_window.min_y);

    // Vertical scale: horizontal line every volt
    double v = MAX_VOLTAGE;
    g_tft.setTextColor(ST77XX_WHITE);
    g_tft.setTextSize(1);
    while (v > MIN_VOLTAGE) {
        int16_t val = round(v - hist_window.min_y) * scale_y;

        drawJustifiedVal(v, 0, "", DISPLAY_WIDTH, hist_window.y + hist_window.h - val, TXT_JUSTIFIED);
        g_tft.drawFastHLine(0, hist_window.y + hist_window.h - val, DISPLAY_WIDTH, ST77XX_WHITE);
        v -= 1.0;
    }
    drawJustifiedVal(v, 0, "", DISPLAY_WIDTH, DISPLAY_HEIGHT - 4, TXT_JUSTIFIED);

    // Horizontal scale: MA! TODO, need a dynamic scale based on zoom
    g_tft.drawFastHLine(0, DISPLAY_HEIGHT - 3, DISPLAY_WIDTH, ST77XX_WHITE);
    int t;
    for (t = 0; t < DISPLAY_WIDTH; t += 10) {
        g_tft.drawFastVLine(DISPLAY_WIDTH - t, DISPLAY_HEIGHT - 2, 2, ST77XX_WHITE);
    }
    //for (t = 0; t < hist->getSize(); t += 10) {
    //    g_tft.drawFastVLine(DISPLAY_WIDTH - round(t * DISPLAY_WIDTH / hist->getSize()),
    //                        DISPLAY_HEIGHT - 2, 2, ST77XX_WHITE);
    //}

    g_tft.setTextSize(2);

    // Stats at top-left
    g_tft.setTextColor(color);
    g_tft.setCursor(0, 1);
    g_tft.print(bat->getName());
    g_tft.print(" ");
    g_tft.print(cv, 2);
    g_tft.print(" ");
    g_tft.print(psoc, 0);
    g_tft.println("%");

    g_tft.setTextColor(ST77XX_CYAN);
    g_tft.print("Max ");
    g_tft.println(hist_stats.max, 2);

    g_tft.print("Avg ");
    g_tft.println(hist_stats.avg, 2);

    g_tft.print("Min ");
    g_tft.println(hist_stats.min, 2);

    g_tft.setTextColor(ST77XX_WHITE);
    g_tft.setTextSize(2);
    g_tft.setCursor(0, DISPLAY_HEIGHT - 3 - 16);
    drawJustifiedVal(g_displayExtent * HISTORY_SAMPLE_INTERVAL_SECS / 3600.0, 1, "hrs", 0, DISPLAY_HEIGHT - 3,
                     TXT_JUSTIFIED);

    // Draw history data
    hist->forEachData(drawHistoryCallback, &hist_window);
}

void drawBatterySummary(const Battery* bat, int16_t y0) {
    char buffer[32];

    double v = bat->getHistory()->getLastData();
    double psoc = bat->calcPSoC(v);
    uint16_t color = bat->getPSoCColor(psoc);
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

void displaySplashScreen() {
    g_tft.setTextWrap(false);
    g_tft.fillScreen(ST77XX_BLACK);

    g_tft.setTextColor(ST77XX_WHITE);
    g_tft.setTextSize(3);
    g_tft.setCursor(0, 48);
    g_tft.print("Initializing");
}

// Redraw the display.
void displayUpdate() {

    switch (g_page) {
        case PAGE_SUMMARY:
            displaySummary();
            break;

        case PAGE_HISTORY:
            displayHistory(&g_batteries[g_selBattery]);
            break;
    }
}

