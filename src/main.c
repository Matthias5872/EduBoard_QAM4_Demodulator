/*********************************************************************************************
 * EduBoard QAM4-Demodulator (Empfaenger)
 * Juventus Technikerschule
 *
 * Empfaengt das QAM4-Signal ueber AN0, sucht die Barker-7-Praeambel,
 * demoduliert 18 QAM4-Symbole und zeigt Rohwert + Temperatur auf dem Display.
 *
 * Sender: 156.25 Hz Traeger, 12.8 ms/Symbol, 36 Bit = 18 Symbole
 * Empfaenger: 20 kSamples/s (50 us), 256 ADC-Samples/Symbol
 *
 * Aenderungen gegenueber Version 1:
 *  - Praeambelsuche ueber 2 Bloecke: Paket wird erst angenommen, wenn das
 *    Korrelationsmaximum sicher gefunden ist (vorher: erster Treffer ueber
 *    der Schwelle -> Symbolstart oft 30..800 Samples zu frueh).
 *  - Normierte Korrelation: Teilueberlappungen der Praeambel und zufaellige
 *    Daten loesen keine Fehldetektion mehr aus.
 *  - Feinsynchronisation: Symbolstart wird mit voller Korrelation
 *    (alle 256 Samples) auf +-32 Samples genau nachgeregelt.
 *  - Demodulation mit allen 256 Samples statt 8 Punkten (rauschfester).
 *  - Phasennachfuehrung pro Symbol (gleicht kleine Frequenzabweichungen aus).
 *  - Q_SIGN: Vorzeichen von Q umschaltbar, falls der Sender I*cos - Q*sin nutzt.
 *  - Diagnose-Ausgaben: Symbole, Phase, Pegel und Blockzeit im Log.
 *********************************************************************************************/
#include "eduboard2.h"
#include "esp_timer.h"

#define TAG "QAM4_RX"

// ---------------------------------------------------------------------------------------------
// Einstellungen
// ---------------------------------------------------------------------------------------------

// +1: Sender sendet s(t) = I*cos(wt) + Q*sin(wt)
// -1: Sender sendet s(t) = I*cos(wt) - Q*sin(wt)
// Werden 01 und 10 immer vertauscht empfangen -> hier umstellen.
#define Q_SIGN              (+1)

// Mindestwert der normierten Praeambel-Korrelation (0..1).
// Echte Praeambel ~0.9..1.0, Teilueberlappung / Daten deutlich tiefer.
#define CORR_MIN            0.75f

// Phasennachfuehrung pro Symbol (0 = aus, typ. 0.2..0.4)
#define PHASE_TRACK_GAIN    0.3f

// Zusaetzliche Diagnose-Ausgaben im Log
#define DEBUG_LOG           1

// ---------------------------------------------------------------------------------------------
// Konstanten
// ---------------------------------------------------------------------------------------------

#define ADC_INPUT           AN0
#define ADC_BLOCK           256
#define SYMBOL_SAMPLES      256
#define CARRIER_PERIOD      128             // 20 kHz / 156.25 Hz
#define PREAMBLE_SYMBOLS    7
#define PREAMBLE_SAMPLES    (PREAMBLE_SYMBOLS * SYMBOL_SAMPLES)
#define DATA_SYMBOLS        18
#define DATA_SAMPLES        (DATA_SYMBOLS * SYMBOL_SAMPLES)

// Suchbereich = 2 Bloecke. Ein Treffer zaehlt nur, wenn das Maximum
// im ersten Block liegt; sonst wird der naechste Block abgewartet.
#define SEARCH_SAMPLES      (2 * ADC_BLOCK)
#define WINDOW_SAMPLES      (PREAMBLE_SAMPLES + SEARCH_SAMPLES)
#define WINDOW_BLOCKS       (WINDOW_SAMPLES / ADC_BLOCK)

// Feinsynchronisation: +- so viele Samples um den Grobtreffer
#define FINE_RANGE          32

#define PACKET_START        0xA5A
#define PI                  3.14159265f

// Barker-7 des Senders: -1 = Symbol 11, +1 = Symbol 00
static const int8_t barker[PREAMBLE_SYMBOLS] = {-1, -1, -1, +1, +1, -1, +1};

static int16_t window[WINDOW_SAMPLES];
static int16_t dataSamples[DATA_SAMPLES];
static uint16_t adcBlock[ADC_BLOCK];

// cos/sin-Tabellen fuer ein Symbol (2 Traegerperioden)
static float cosTab[SYMBOL_SAMPLES];
static float sinTab[SYMBOL_SAMPLES];

static volatile bool capture = false;
static volatile bool packetReady = false;
static volatile int windowBlocks = 0;
static int dataCount = 0;
static float phase = 0.0f;
static float lastCorr = 0.0f;
static int32_t lastPP = 0;

// Diagnose: Zeit zwischen zwei ADC-Bloecken (soll 12800 us sein)
static volatile int64_t lastBlockTime = 0;
static volatile int32_t blockPeriodUs = 0;


// ---------------------------------------------------------------------------------------------
// Statistik
// ---------------------------------------------------------------------------------------------

static uint32_t okPackets = 0;
static uint32_t badPackets = 0;

static uint16_t lastRaw = 0;
static float lastTemp = 0.0f;
static bool hasValue = false;


// Gray Code wieder in den normalen 12-Bit-Wert umwandeln
static uint16_t from_gray(uint16_t gray)
{
    uint16_t value = 0;

    for (; gray; gray >>= 1) {
        value ^= gray;
    }

    return value & 0x0FFF;
}


// Gleiche Pruefsumme wie beim Sender
static uint16_t checksum(uint16_t raw)
{
    return (((raw >> 8) & 0xF)
          + ((raw >> 4) & 0xF)
          + (raw & 0xF)) & 0xF;
}


// Schnelle I/Q-Schaetzung aus 8 Punkten (fuer die Grobsuche).
// Differenzen 0-180 Grad bzw. 90-270 Grad -> DC-Offset faellt weg.
static inline void get_iq_fast(const int16_t *s, int32_t *i, int32_t *q)
{
    *i = (s[0]  - s[64])  + (s[128] - s[192]);
    *q = (s[32] - s[96])  + (s[160] - s[224]);
    *q *= Q_SIGN;
}


// Genaue I/Q-Schaetzung mit allen 256 Samples (Korrelation mit cos/sin).
// cos und sin haben ueber 2 volle Perioden Mittelwert 0 -> DC faellt weg.
static void get_iq_full(const int16_t *s, float *i, float *q)
{
    float si = 0.0f;
    float sq = 0.0f;

    for (int n = 0; n < SYMBOL_SAMPLES; n++) {
        si += s[n] * cosTab[n];
        sq += s[n] * sinTab[n];
    }

    *i = si;
    *q = sq * Q_SIGN;
}


// Barker-Korrelation mit voller Genauigkeit an einer Position
static float preamble_corr_full(int pos, float *sumI, float *sumQ)
{
    float sI = 0.0f;
    float sQ = 0.0f;

    for (int b = 0; b < PREAMBLE_SYMBOLS; b++) {
        float i, q;
        get_iq_full(&window[pos + b * SYMBOL_SAMPLES], &i, &q);
        sI += barker[b] * i;
        sQ += barker[b] * q;
    }

    *sumI = sI;
    *sumQ = sQ;

    return sI * sI + sQ * sQ;
}


// Barker-7 im laufenden ADC-Signal suchen.
// Rueckgabe true: *start = Beginn der Praeambel im Fenster,
//                 *rxPhase = Phasendrehung der Strecke.
static bool find_preamble(int *start, float *rxPhase)
{
    int16_t min = window[0];
    int16_t max = window[0];

    for (int n = 1; n < WINDOW_SAMPLES; n++) {
        if (window[n] < min) min = window[n];
        if (window[n] > max) max = window[n];
    }

    int32_t pp = max - min;

    // Praktisch kein Signal vorhanden
    if (pp < 100) {
        return false;
    }

    // Absolute Mindeststaerke (wie Version 1)
    int64_t limit = (int64_t)pp * 6;
    int64_t limit2 = limit * limit;

    int bestPos = -1;
    int64_t bestScore = 0;
    float bestCorr = 0.0f;

    // --- Grobsuche ueber 2 Bloecke ---
    for (int pos = 0; pos < SEARCH_SAMPLES; pos++) {

        int64_t sumI = 0;
        int64_t sumQ = 0;
        float mag = 0.0f;

        for (int b = 0; b < PREAMBLE_SYMBOLS; b++) {

            int32_t i, q;
            get_iq_fast(&window[pos + b * SYMBOL_SAMPLES], &i, &q);

            sumI += barker[b] * i;
            sumQ += barker[b] * q;
            mag  += sqrtf((float)i * i + (float)q * q);
        }

        int64_t score = sumI * sumI + sumQ * sumQ;

        if (score <= limit2 || score <= bestScore || mag <= 0.0f) {
            continue;
        }

        // Normierte Korrelation: 1.0 = perfekte Praeambel
        float corr = sqrtf((float)score) / mag;

        if (corr >= CORR_MIN) {
            bestScore = score;
            bestPos = pos;
            bestCorr = corr;
        }
    }

    // Nichts gefunden, oder Maximum liegt im zweiten Block:
    // dann kommt der echte Peak evtl. erst im naechsten Block.
    if (bestPos < 0 || bestPos >= ADC_BLOCK) {
        return false;
    }

    // --- Feinsynchronisation mit voller Korrelation ---
    int from = bestPos - FINE_RANGE;
    int to   = bestPos + FINE_RANGE;

    if (from < 0) from = 0;
    if (to > SEARCH_SAMPLES - 1) to = SEARCH_SAMPLES - 1;

    float fineBest = -1.0f;
    float fineI = 0.0f;
    float fineQ = 0.0f;
    int finePos = bestPos;

    for (int pos = from; pos <= to; pos++) {
        float sI, sQ;
        float sc = preamble_corr_full(pos, &sI, &sQ);

        if (sc > fineBest) {
            fineBest = sc;
            finePos = pos;
            fineI = sI;
            fineQ = sQ;
        }
    }

    *start = finePos;

    // Sender-Symbol 00 liegt ideal bei I=+1, Q=+1 = 45 Grad.
    // Die Differenz dazu ist die Phasendrehung des Empfangssignals.
    *rxPhase = atan2f(fineQ, fineI) - PI / 4.0f;

    lastCorr = bestCorr;
    lastPP = pp;

    return true;
}


// Wird vom ADC-Treiber nach jedem 256er-Block aufgerufen
static void adc_callback(void)
{
    int64_t now = esp_timer_get_time();

    if (lastBlockTime != 0) {
        blockPeriodUs = (int32_t)(now - lastBlockTime);
    }

    lastBlockTime = now;

    adc_get_buffer(ADC_INPUT, adcBlock);

    // Nach gefundener Praeambel nur noch die
    // 18 Datensymbole einsammeln.
    if (capture) {

        int n = DATA_SAMPLES - dataCount;

        if (n > ADC_BLOCK) {
            n = ADC_BLOCK;
        }

        for (int i = 0; i < n; i++) {
            dataSamples[dataCount + i] = adcBlock[i];
        }

        dataCount += n;

        if (dataCount >= DATA_SAMPLES) {
            capture = false;
            packetReady = true;
        }

        return;
    }

    if (packetReady) {
        return;
    }

    // Rollendes Fenster:
    // alter Block raus, neuer Block hinten anhaengen
    memmove(
        window,
        &window[ADC_BLOCK],
        (WINDOW_SAMPLES - ADC_BLOCK) * sizeof(int16_t)
    );

    for (int i = 0; i < ADC_BLOCK; i++) {
        window[WINDOW_SAMPLES - ADC_BLOCK + i] = adcBlock[i];
    }

    // Erst suchen, wenn das ganze Fenster mit neuen Daten gefuellt ist
    if (windowBlocks < WINDOW_BLOCKS) {
        windowBlocks++;
        return;
    }

    int preambleStart;

    if (find_preamble(&preambleStart, &phase)) {

        // Im aktuellen Fenster liegen nach der Praeambel
        // bereits einige Datensamples.
        int firstData = preambleStart + PREAMBLE_SAMPLES;

        dataCount = WINDOW_SAMPLES - firstData;

        for (int i = 0; i < dataCount; i++) {
            dataSamples[i] = window[firstData + i];
        }

        capture = true;
    }
}


// Ein QAM4-Symbol aus 256 ADC-Samples in zwei Bits zurueckwandeln.
// *ph wird dabei nachgefuehrt (Phasenregelung).
static uint8_t demod_symbol(const int16_t *s, float *ph)
{
    float i;
    float q;

    get_iq_full(s, &i, &q);

    // Phasendrehung der Strecke korrigieren
    float c = cosf(*ph);
    float sn = sinf(*ph);

    float I =  i * c + q * sn;
    float Q = -i * sn + q * c;

    uint8_t sym;
    float ideal;    // Winkel des idealen Konstellationspunktes

    if (I >= 0 && Q >= 0)      { sym = 0x0; ideal =  PI / 4.0f; }          // 00
    else if (I < 0 && Q >= 0)  { sym = 0x1; ideal =  3.0f * PI / 4.0f; }   // 01
    else if (I >= 0 && Q < 0)  { sym = 0x2; ideal = -PI / 4.0f; }          // 10
    else                       { sym = 0x3; ideal = -3.0f * PI / 4.0f; }   // 11

    // Phasenfehler zum entschiedenen Punkt (-pi..pi)
    float err = atan2f(Q, I) - ideal;

    if (err >  PI) err -= 2.0f * PI;
    if (err < -PI) err += 2.0f * PI;

    *ph += PHASE_TRACK_GAIN * err;

    return sym;
}


// Display mit letztem gueltigem Messwert
// und Empfangsstatistik aktualisieren.
static void show_value(void)
{
    char text[40];

    uint32_t total = okPackets + badPackets;

    float errorRate = 0.0f;

    if (total > 0) {
        errorRate = 100.0f * (float)badPackets / (float)total;
    }

    lcdFillScreen(BLACK);

    lcdDrawString(fx32M, 20, 40, "QAM4 Empfaenger", GREEN);

    if (hasValue) {
        snprintf(text, sizeof(text), "RAW: 0x%03X", lastRaw);
        lcdDrawString(fx24M, 20, 110, text, WHITE);

        snprintf(text, sizeof(text), "Temp: %.2f C", lastTemp);
        lcdDrawString(fx24M, 20, 150, text, WHITE);
    }
    else {
        lcdDrawString(fx24M, 20, 110, "Noch kein Paket", WHITE);
    }

    snprintf(text, sizeof(text), "OK: %lu", (unsigned long)okPackets);
    lcdDrawString(fx24M, 20, 220, text, GREEN);

    snprintf(text, sizeof(text), "Fehler: %lu", (unsigned long)badPackets);
    lcdDrawString(fx24M, 20, 260, text, RED);

    snprintf(text, sizeof(text), "Fehlerquote: %.1f %%", errorRate);
    lcdDrawString(fx24M, 20, 300, text, WHITE);

    lcdUpdateVScreen();
}


// Haupttask des Empfaengers
static void receiver_task(void *param)
{
    for (;;) {

        if (packetReady) {

            uint64_t packet = 0;
            uint8_t syms[DATA_SYMBOLS];
            float ph = phase;
            float phStart = phase;

            // 18 QAM4-Symbole = 36 Bit zusammensetzen,
            // das hoechstwertige Bitpaar zuerst.
            for (int s = 0; s < DATA_SYMBOLS; s++) {
                syms[s] = demod_symbol(&dataSamples[s * SYMBOL_SAMPLES], &ph);
                packet = (packet << 2) | syms[s];
            }

            uint16_t start = from_gray((packet >> 24) & 0x0FFF);
            uint16_t raw   = from_gray((packet >> 12) & 0x0FFF);
            uint16_t check = from_gray(packet & 0x0FFF);

#if DEBUG_LOG
            char symText[DATA_SYMBOLS * 3 + 1];
            for (int s = 0; s < DATA_SYMBOLS; s++) {
                symText[s * 3 + 0] = '0' + ((syms[s] >> 1) & 1);
                symText[s * 3 + 1] = '0' + (syms[s] & 1);
                symText[s * 3 + 2] = ' ';
            }
            symText[DATA_SYMBOLS * 3] = '\0';

            // Phasendrift ueber das Paket: grosse Werte -> Frequenzabweichung
            ESP_LOGI(TAG,
                "Symbole: %s| Korr=%.2f PP=%ld Phase=%.0f->%.0f Grad Block=%ld us",
                symText,
                lastCorr,
                (long)lastPP,
                phStart * 180.0f / PI,
                ph * 180.0f / PI,
                (long)blockPeriodUs);
#endif

            if (start == PACKET_START && check == checksum(raw)) {

                okPackets++;

                // 12-Bit-Zweierkomplement vorzeichenrichtig erweitern
                int16_t signedRaw = (int16_t)(raw << 4) >> 4;

                lastRaw = raw;
                lastTemp = signedRaw * 0.0625f;
                hasValue = true;

                ESP_LOGI(TAG, "Paket OK: RAW=0x%03X Temperatur=%.2f C", lastRaw, lastTemp);
            }
            else {

                badPackets++;

                ESP_LOGW(TAG,
                    "Paket verworfen: START=0x%03X RAW=0x%03X CHECK=0x%03X",
                    start, raw, check);
            }

            show_value();

            // Fuer das naechste Paket wieder neu synchronisieren
            windowBlocks = 0;
            packetReady = false;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void app_main(void)
{
    // cos/sin-Tabellen fuer 2 Traegerperioden
    for (int n = 0; n < SYMBOL_SAMPLES; n++) {
        float w = 2.0f * PI * (float)n / (float)CARRIER_PERIOD;
        cosTab[n] = cosf(w);
        sinTab[n] = sinf(w);
    }

    eduboard2_init();

    lcdFillScreen(BLACK);
    lcdDrawString(fx32M, 20, 55, "QAM4 Empfaenger", GREEN);
    lcdDrawString(fx24M, 20, 120, "Warte auf Signal ...", WHITE);
    lcdUpdateVScreen();

    // Callback fuer das kontinuierliche ADC-Streaming setzen
    adc_set_stream_callback(adc_callback);

    // Empfaenger-Task starten
    xTaskCreate(receiver_task, "receiver", 4096, NULL, 5, NULL);
}
