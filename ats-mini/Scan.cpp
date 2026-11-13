#include "Common.h"
#include "Utils.h"
#include "Menu.h"
#include "Draw.h"
#include "Memories.h"
#include "Storage.h"

#define SCAN_POINTS      200 // Number of frequencies to scan

#define SCAN_OFF    0   // Scanner off, no data
#define SCAN_RUN    1   // Scanner running
#define SCAN_DONE   2   // Scanner done, valid data in scanData[]

struct ScanPoint
{
  uint8_t rssi;
  uint8_t snr;
};

static ScanPoint *scanData = nullptr;

static uint32_t scanTime = millis();
static uint32_t scanTuneStarted;
static uint8_t  scanStatus = SCAN_OFF;

static uint32_t scanStartFreq; // Hz
static uint32_t scanStep;      // Hz
static uint16_t scanCount;
static uint8_t  scanMinRSSI;
static uint8_t  scanMaxRSSI;
static uint8_t  scanMinSNR;
static uint8_t  scanMaxSNR;

static inline uint8_t min(uint8_t a, uint8_t b) { return(a<b? a:b); }
static inline uint8_t max(uint8_t a, uint8_t b) { return(a>b? a:b); }

float scanGetRSSI(uint32_t freq)
{
  // Input frequency must be in range of existing data
  if((scanStatus!=SCAN_DONE) || (freq<scanStartFreq) || ((freq-scanStartFreq)/scanStep>=scanCount))
    return(0.0);

  uint8_t result = scanData[(freq - scanStartFreq) / scanStep].rssi;
  return((result - scanMinRSSI) / (float)(scanMaxRSSI - scanMinRSSI + 1));
}

float scanGetSNR(uint32_t freq)
{
  // Input frequency must be in range of existing data
  if((scanStatus!=SCAN_DONE) || (freq<scanStartFreq) || ((freq-scanStartFreq)/scanStep>=scanCount))
    return(0.0);

  uint8_t result = scanData[(freq - scanStartFreq) / scanStep].snr;
  return((result - scanMinSNR) / (float)(scanMaxSNR - scanMinSNR + 1));
}

static void scanInit(uint32_t centerFreq, uint32_t step)
{
  scanStep    = step;
  scanCount   = 0;
  scanMinRSSI = 255;
  scanMaxRSSI = 0;
  scanMinSNR  = 255;
  scanMaxSNR  = 0;
  scanStatus  = SCAN_RUN;
  scanTime    = millis();

  const Band *band = getCurrentBand();
  int64_t freq = ((int64_t)(centerFreq / scanStep) - SCAN_POINTS / 2) * scanStep;
  int64_t span = (int64_t)scanStep * (SCAN_POINTS - 1);

  // Adjust to band boundaries
  if(freq + span > band->maximumFreq)
    freq = band->maximumFreq - span;
  if(freq < band->minimumFreq)
    freq = band->minimumFreq;
  scanStartFreq = freq;

  // Clear scan data
  memset(scanData, 0, SCAN_POINTS * sizeof(*scanData));

  // A preceding confirmed tune has acknowledged STC. Start our own tune
  // before waiting for completion, even if already at the first frequency.
  scanTuneStarted = millis();
  rx.setFrequency(freqFromHz(scanStartFreq, currentMode));
}

static bool scanTickTime()
{
  // Scan must be on
  if((scanStatus!=SCAN_RUN) || (scanCount>=SCAN_POINTS)) return(false);

  if(consumeAbortPending() || millis() - scanTuneStarted >= TUNE_TIMEOUT_MS)
  {
    scanStatus = SCAN_DONE;
    return(false);
  }

  // Wait for the right time
  if(millis() - scanTime < TUNE_POLL_INTERVAL_MS) return(true);

  // This is our current frequency to scan
  uint32_t freq = scanStartFreq + scanStep * scanCount;

  // Poll for the tuning status
  if(!rx.pollTuneStatus())
  {
    scanTime = millis();
    return(true);
  }

  // Do not attach measurements to a stale or unexpected frequency.
  if(rx.getCurrentFrequency() != freqFromHz(freq, currentMode))
  {
    scanStatus = SCAN_DONE;
    return(false);
  }

  // Measure RSSI/SNR values
  rx.getCurrentReceivedSignalQuality();
  scanData[scanCount].rssi = rx.getCurrentRSSI();
  scanData[scanCount].snr  = rx.getCurrentSNR();

  // Measure range of values
  scanMinRSSI = min(scanData[scanCount].rssi, scanMinRSSI);
  scanMaxRSSI = max(scanData[scanCount].rssi, scanMaxRSSI);
  scanMinSNR  = min(scanData[scanCount].snr, scanMinSNR);
  scanMaxSNR  = max(scanData[scanCount].snr, scanMaxSNR);

  // Next frequency to scan
  freq += scanStep;

  // Set next frequency to scan or expire scan
  if((++scanCount >= SCAN_POINTS) || !isFreqInBand(getCurrentBand(), freq) || consumeAbortPending())
    scanStatus = SCAN_DONE;
  else
  {
    scanTuneStarted = millis();
    rx.setFrequency(freqFromHz(freq, currentMode));
  }

  // Save last scan time
  scanTime = millis() - TUNE_POLL_INTERVAL_MS;

  // Return current scan status
  return(scanStatus==SCAN_RUN);
}

//
// Run entire scan once
//
void scanRun(uint32_t centerFreq, uint32_t step)
{
  // Allocate once and retain the samples for drawing after the scan.
  if(!scanData)
  {
    scanData = static_cast<ScanPoint *>(ps_malloc(SCAN_POINTS * sizeof(*scanData)));
    if(!scanData)
      return;
  }

  // Mute the audio
  muteOn(MUTE_TEMP, true);
  // Flag is set by rotary encoder and cleared on seek/scan entry
  seekStop = false;
  // Save current frequency
  uint32_t curFreq = currentFrequency;
  // Scan receiver frequencies without the SSB tuning or calibration offset.
  if(isSSB()) rx.setSSBBfo(0);
  // Scan the whole range
  for(scanInit(centerFreq, step) ; scanTickTime();) delay(1);
  // Restore current frequency
  if(isSSB()) rx.setFrequencyConfirmed(getReceiverFrequency(curFreq));
  // Restore the derived BFO and calibration before unmuting.
  updateFrequency(curFreq, false);
  // Unmute the audio
  muteOn(MUTE_TEMP, false);
}

//
// Automatic station storage for the current FM or MW band
//

#define ATS_MIN_RSSI_FM      10
#define ATS_MIN_SNR_FM       10
#define ATS_MIN_RSSI_AM       6
#define ATS_MIN_SNR_AM        8
#define ATS_SAMPLE_COUNT      3
#define ATS_SAMPLE_DELAY_MS  50
#define ATS_SETTLE_MS       100

static bool atsAborted = false;

static bool atsFindEmptySlot(uint8_t &slot)
{
  for(uint8_t i=0 ; i<getTotalMemories() ; i++)
    if(!getMemory(i).freq)
    {
      slot = i;
      return(true);
    }
  return(false);
}

static bool atsStopSeeking()
{
  if(consumeAbortPending())
  {
    atsAborted = true;
    return(true);
  }
  return(false);
}

static uint8_t atsMedian(uint8_t a, uint8_t b, uint8_t c)
{
  if(a > b) { uint8_t t = a; a = b; b = t; }
  if(b > c) { uint8_t t = b; b = c; c = t; }
  if(a > b) { uint8_t t = a; a = b; b = t; }
  return(b);
}

static bool atsAcceptSignal()
{
  uint8_t rssiValues[ATS_SAMPLE_COUNT];
  uint8_t snrValues[ATS_SAMPLE_COUNT];

  for(uint8_t i=0 ; i<ATS_SAMPLE_COUNT ; i++)
  {
    rx.getCurrentReceivedSignalQuality();
    rssiValues[i] = rx.getCurrentRSSI();
    snrValues[i] = rx.getCurrentSNR();
    if(i + 1 < ATS_SAMPLE_COUNT) delay(ATS_SAMPLE_DELAY_MS);
  }

  uint8_t rssiValue = atsMedian(rssiValues[0], rssiValues[1], rssiValues[2]);
  uint8_t snrValue  = atsMedian(snrValues[0], snrValues[1], snrValues[2]);
  uint8_t squelch   = currentSquelch[currentMode] & 0x7f;

  if(squelch)
    return((currentSquelch[currentMode] & 0x80)? snrValue>=squelch : rssiValue>=squelch);

  if(currentMode == FM)
    return(rssiValue>=ATS_MIN_RSSI_FM && snrValue>=ATS_MIN_SNR_FM);

  return(rssiValue>=ATS_MIN_RSSI_AM && snrValue>=ATS_MIN_SNR_AM);
}

void atsRun()
{
  drawScreen();

  const Band *band = getCurrentBand();
  if((currentMode != FM && currentMode != AM) ||
     (band->bandType != FM_BAND_TYPE && band->bandType != MW_BAND_TYPE))
  {
    statusShow("ATS", "FM/MW only");
    drawMessage("FM/MW only");
    return;
  }

  uint8_t slot;
  if(!atsFindEmptySlot(slot))
  {
    statusShow("ATS", "Memory full");
    drawMessage("Memory full");
    return;
  }

  uint32_t savedFrequency = currentFrequency;
  uint8_t stored = 0;
  atsAborted = false;
  seekStop = false;
  drawMessage("ATS...");
  muteOn(MUTE_TEMP, true);

  if(updateFrequency(band->minimumFreq, false))
  {
    uint32_t previous = currentFrequency;
    for(;;)
    {
      if(consumeAbortPending())
      {
        atsAborted = true;
        break;
      }

      delay(ATS_SETTLE_MS);
      if(atsAcceptSignal())
      {
        Memory memory = {};
        memory.freq = currentFrequency;
        memory.band = bandIdx;
        memory.mode = currentMode;
        setMemory(slot, memory);
        stored++;

        char message[16];
        snprintf(message, sizeof(message), "ATS %u", (unsigned)stored);
        drawMessage(message);

        if(!atsFindEmptySlot(slot)) break;
      }

      if(previous >= band->maximumFreq) break;
      rx.seekStationProgress(nullptr, atsStopSeeking, 1);
      if(atsAborted) break;

      uint32_t target = freqToHz(rx.getCurrentFrequency(), currentMode);
      if(target <= previous || target > band->maximumFreq) break;
      if(!updateFrequency(target, false)) break;
      previous = currentFrequency;
    }
  }

  updateFrequency(savedFrequency, false);
  clearStationInfo();
  identifyFrequency(currentFrequency);
  muteOn(MUTE_TEMP, false);

  char status[32];
  if(stored) snprintf(status, sizeof(status), "%u station%s stored", (unsigned)stored, stored==1? "" : "s");
  else strlcpy(status, "No stations found", sizeof(status));
  statusShow("ATS", status);
  if(stored) prefsRequestSave(SAVE_MEMORIES);
}
