/*********************************************************************************************
 * EduBoard QAM4-Demodulator (Empfaenger)
 * Juventus Technikerschule
 *
 * Empfaengt das QAM4-Signal ueber AN0, sucht die Barker-7-Praeambel,
 * demoduliert 18 QAM4-Symbole und zeigt Rohwert + Temperatur auf dem Display.
 *
 * Sender: 156.25 Hz Traeger, 12.8 ms/Symbol, 36 Bit = 18 Symbole
 * Empfaenger: 20 kSamples/s (50 us), 256 ADC-Samples/Symbol
 *********************************************************************************************/
#include "eduboard2.h"

#define TAG "QAM4_RX"

#define ADC_INPUT           AN0
#define ADC_BLOCK           256
#define SYMBOL_SAMPLES      256
#define PREAMBLE_SYMBOLS    7
#define PREAMBLE_SAMPLES    (PREAMBLE_SYMBOLS * SYMBOL_SAMPLES)
#define DATA_SYMBOLS        18
#define DATA_SAMPLES        (DATA_SYMBOLS * SYMBOL_SAMPLES)
#define WINDOW_SAMPLES      (PREAMBLE_SAMPLES + ADC_BLOCK)
#define PACKET_START        0xA5A
#define PI                  3.14159265f

// Barker-7 des Senders: -1 = Symbol 11, +1 = Symbol 00
static const int8_t barker[PREAMBLE_SYMBOLS] = {-1, -1, -1, +1, +1, -1, +1};

static int16_t window[WINDOW_SAMPLES];
static int16_t dataSamples[DATA_SAMPLES];
static uint16_t adcBlock[ADC_BLOCK];

static volatile bool capture = false;
static volatile bool packetReady = false;
static int windowBlocks = 0;
static int dataCount = 0;
static float phase = 0.0f;


// ---------------------------------------------------------------------------------------------
// Statistik
// ---------------------------------------------------------------------------------------------

// Anzahl korrekt und fehlerhaft empfangener Pakete
static uint32_t okPackets = 0;
static uint32_t badPackets = 0;

// Letzter korrekt empfangener Messwert
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


// I und Q aus zwei Traegerperioden bestimmen.
// Durch die Differenzen 0-180 Grad bzw. 90-270 Grad
// verschwindet der DC-Offset automatisch.
static void get_iq(const int16_t *s, float *i, float *q)
{
    *i = (s[0]  - s[64])
       + (s[128] - s[192]);

    *q = (s[32] - s[96])
       + (s[160] - s[224]);
}


// Barker-7 im laufenden ADC-Signal suchen.
// Gleichzeitig wird aus der Praeambel die Phasendrehung
// der Funkstrecke bestimmt.
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

    int bestPos = 0;
    int64_t bestI = 0;
    int64_t bestQ = 0;
    int64_t bestScore = 0;

    // In einem ADC-Block alle moeglichen Symbolstarts pruefen
    for (int pos = 0; pos < ADC_BLOCK; pos++) {

        int64_t sumI = 0;
        int64_t sumQ = 0;

        for (int b = 0; b < PREAMBLE_SYMBOLS; b++) {

            int k = pos + b * SYMBOL_SAMPLES;

            int32_t i =
                (window[k] - window[k + 64]) +
                (window[k + 128] - window[k + 192]);

            int32_t q =
                (window[k + 32] - window[k + 96]) +
                (window[k + 160] - window[k + 224]);

            sumI += barker[b] * i;
            sumQ += barker[b] * q;
        }

        int64_t score = sumI * sumI + sumQ * sumQ;

        if (score > bestScore) {
            bestScore = score;
            bestPos = pos;
            bestI = sumI;
            bestQ = sumQ;
        }
    }

    // Ein echter Barker-Treffer ist deutlich staerker
    // als der normale Ruhetraeger.
    int64_t limit = (int64_t)pp * 6;

    if (bestScore < limit * limit) {
        return false;
    }

    *start = bestPos;

    // Sender-Symbol 00 liegt ideal bei I=+1, Q=+1 = 45 Grad.
    // Die Differenz dazu ist die Phasendrehung des Empfangssignals.
    *rxPhase = atan2f((float)bestQ, (float)bestI) - PI / 4.0f;

    return true;
}


// Wird vom ADC-Treiber nach jedem 256er-Block aufgerufen
static void adc_callback(void)
{
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
        PREAMBLE_SAMPLES * sizeof(int16_t)
    );

    for (int i = 0; i < ADC_BLOCK; i++) {
        window[PREAMBLE_SAMPLES + i] = adcBlock[i];
    }

    if (windowBlocks < 8) {
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


// Ein QAM4-Symbol aus 256 ADC-Samples
// in zwei Bits zurueckwandeln.
static uint8_t demod_symbol(const int16_t *s)
{
    float i;
    float q;

    get_iq(s, &i, &q);

    // Phasendrehung der Funkstrecke korrigieren
    float c = cosf(phase);
    float sn = sinf(phase);

    float I =  i * c + q * sn;
    float Q = -i * sn + q * c;

    if (I >= 0 && Q >= 0) return 0x0;   // 00
    if (I <  0 && Q >= 0) return 0x1;   // 01
    if (I >= 0 && Q <  0) return 0x2;   // 10

    return 0x3;                         // 11
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

    lcdDrawString(
        fx32M,
        20,
        40,
        "QAM4 Empfaenger",
        GREEN
    );

    // Letzten gueltigen Messwert anzeigen
    if (hasValue) {

        snprintf(
            text,
            sizeof(text),
            "RAW: 0x%03X",
            lastRaw
        );

        lcdDrawString(
            fx24M,
            20,
            110,
            text,
            WHITE
        );

        snprintf(
            text,
            sizeof(text),
            "Temp: %.2f C",
            lastTemp
        );

        lcdDrawString(
            fx24M,
            20,
            150,
            text,
            WHITE
        );
    }
    else {

        lcdDrawString(
            fx24M,
            20,
            110,
            "Noch kein Paket",
            WHITE
        );
    }


    // Anzahl korrekt empfangener Pakete
    snprintf(
        text,
        sizeof(text),
        "OK: %lu",
        (unsigned long)okPackets
    );

    lcdDrawString(
        fx24M,
        20,
        220,
        text,
        GREEN
    );


    // Anzahl fehlerhafter Pakete
    snprintf(
        text,
        sizeof(text),
        "Fehler: %lu",
        (unsigned long)badPackets
    );

    lcdDrawString(
        fx24M,
        20,
        260,
        text,
        RED
    );


    // Prozentuale Fehlerquote
    snprintf(
        text,
        sizeof(text),
        "Fehlerquote: %.1f %%",
        errorRate
    );

    lcdDrawString(
        fx24M,
        20,
        300,
        text,
        WHITE
    );


    // Virtuellen Bildpuffer wirklich auf das Display uebertragen
    lcdUpdateVScreen();
}


// Haupttask des Empfaengers
static void receiver_task(void *param)
{
    for (;;) {

        if (packetReady) {

            uint64_t packet = 0;

            // 18 QAM4-Symbole = 36 Bit zusammensetzen,
            // das hoechstwertige Bitpaar zuerst.
            for (int s = 0; s < DATA_SYMBOLS; s++) {

                packet =
                    (packet << 2) |
                    demod_symbol(
                        &dataSamples[s * SYMBOL_SAMPLES]
                    );
            }


            // Drei 12-Bit-Felder aus dem Paket lesen
            uint16_t start =
                from_gray(
                    (packet >> 24) & 0x0FFF
                );

            uint16_t raw =
                from_gray(
                    (packet >> 12) & 0x0FFF
                );

            uint16_t check =
                from_gray(
                    packet & 0x0FFF
                );


            // Paket pruefen
            if (
                start == PACKET_START &&
                check == checksum(raw)
            ) {

                // Korrektes Paket zaehlen
                okPackets++;

                // 12-Bit-Zweierkomplement
                // vorzeichenrichtig erweitern
                int16_t signedRaw =
                    (int16_t)(raw << 4) >> 4;

                // Letzten korrekten Wert speichern
                lastRaw = raw;
                lastTemp =
                    signedRaw * 0.0625f;

                hasValue = true;

                ESP_LOGI(
                    TAG,
                    "Paket OK: RAW=0x%03X Temperatur=%.2f C",
                    lastRaw,
                    lastTemp
                );
            }
            else {

                // Fehlerhaftes Paket zaehlen
                badPackets++;

                ESP_LOGW(
                    TAG,
                    "Paket verworfen: START=0x%03X RAW=0x%03X CHECK=0x%03X",
                    start,
                    raw,
                    check
                );
            }


            // Display nach jedem erkannten Paket aktualisieren
            show_value();


            // Fuer das naechste Paket wieder neu synchronisieren
            packetReady = false;
            windowBlocks = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void app_main(void)
{
    eduboard2_init();

    lcdFillScreen(BLACK);

    lcdDrawString(
        fx32M,
        20,
        55,
        "QAM4 Empfaenger",
        GREEN
    );

    lcdDrawString(
        fx24M,
        20,
        120,
        "Warte auf Signal ...",
        WHITE
    );

    // Bildpuffer auf das Display uebertragen
    lcdUpdateVScreen();

    // Callback fuer das kontinuierliche ADC-Streaming setzen
    adc_set_stream_callback(adc_callback);

    // Empfaenger-Task starten
    xTaskCreate(
        receiver_task,
        "receiver",
        4096,
        NULL,
        5,
        NULL
    );
}