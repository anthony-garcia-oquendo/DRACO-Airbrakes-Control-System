#include <Wire.h>
#include <SPIFFS.h>

#define I2C_ADDR 0x28

// Register map: 20 bytes
// 0x00-0x03: t_ms (u32)
// 0x04-0x07: alt_m_x1000 (i32)
// 0x08-0x0B: ax_ftps2_x1000 (i32)
// 0x0C-0x0F: ay_ftps2_x1000 (i32)
// 0x10-0x13: az_ftps2_x1000 (i32)
static const uint8_t REG_LEN = 20;

volatile uint8_t regs[REG_LEN];
volatile uint8_t reg_ptr = 0;

struct Sample {
  uint32_t t_ms;
  int32_t alt_m_x1000;
  int32_t ax_ftps2_x1000;
  int32_t ay_ftps2_x1000;
  int32_t az_ftps2_x1000;
};

static Sample* samples = nullptr;
static size_t sample_count = 0;
static size_t idx = 0;

static uint32_t playback_start_ms = 0;
static bool running = true;

// ---------- Helpers: little-endian packing ----------
static inline void write_le_u32(volatile uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}
static inline void write_le_i32(volatile uint8_t* p, int32_t v) {
  write_le_u32(p, (uint32_t)v);
}

// ---------- I2C callbacks ----------
void onReceive(int len) {
  if (len <= 0) return;

  // First byte is register pointer (sensor-style)
  reg_ptr = (uint8_t)Wire.read();
  len--;

  // Optional control: write to 0x1F with 0/1 to stop/start
  // Example from Pi: i2cset -y 1 0x28 0x1F 0x00
  if (len >= 1) {
    uint8_t v = (uint8_t)Wire.read();
    if (reg_ptr == 0x1F) {
      running = (v != 0);
      if (running) {
        playback_start_ms = millis();
      }
    }
  }
}

void onRequest() {
  uint8_t p = reg_ptr;
  if (p >= REG_LEN) p = 0;
  Wire.write((uint8_t)regs[p]);
  reg_ptr = (uint8_t)(p + 1);
}

// ---------- CSV parsing ----------
static bool parse_csv_line(const String& line, Sample& out) {
  // Expect: t_s,alt_m,ax,ay,az
  // Allow spaces.
  char buf[192];
  size_t n = line.length();
  if (n >= sizeof(buf)) return false;
  line.toCharArray(buf, sizeof(buf));

  // Skip empty/comment lines
  char* s = buf;
  while (*s == ' ' || *s == '\t') s++;
  if (*s == '\0' || *s == '#') return false;

  // Tokenize
  char* tok = strtok(s, ",");
  if (!tok) return false; float t_s = atof(tok);

  tok = strtok(nullptr, ",");
  if (!tok) return false; float alt_m = atof(tok);

  tok = strtok(nullptr, ",");
  if (!tok) return false; float ax = atof(tok);

  tok = strtok(nullptr, ",");
  if (!tok) return false; float ay = atof(tok);

  tok = strtok(nullptr, ",");
  if (!tok) return false; float az = atof(tok);

  // Convert to fixed-point
  out.t_ms = (uint32_t)lroundf(t_s * 1000.0f);
  out.alt_m_x1000 = (int32_t)lroundf(alt_m * 1000.0f);
  out.ax_ftps2_x1000 = (int32_t)lroundf(ax * 1000.0f);
  out.ay_ftps2_x1000 = (int32_t)lroundf(ay * 1000.0f);
  out.az_ftps2_x1000 = (int32_t)lroundf(az * 1000.0f);

  return true;
}

static bool load_csv(const char* path) {
  File f = SPIFFS.open(path, "r");
  if (!f) {
    Serial.printf("ERROR: couldn't open %s\n", path);
    return false;
  }

  // First pass: count valid data lines (skip header/comments)
  size_t count = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    Sample tmp;
    if (parse_csv_line(line, tmp)) count++;
  }
  f.close();

  if (count == 0) {
    Serial.println("ERROR: No valid samples found in CSV.");
    return false;
  }

  samples = (Sample*)malloc(count * sizeof(Sample));
  if (!samples) {
    Serial.println("ERROR: malloc failed for samples.");
    return false;
  }

  // Second pass: load
  f = SPIFFS.open(path, "r");
  if (!f) return false;

  size_t i = 0;
  while (f.available() && i < count) {
    String line = f.readStringUntil('\n');
    line.trim();
    Sample s;
    if (parse_csv_line(line, s)) {
      samples[i++] = s;
    }
  }
  f.close();

  sample_count = i;
  Serial.printf("Loaded %u samples from %s\n", (unsigned)sample_count, path);

  // Basic sanity: ensure nondecreasing time
  for (size_t k = 1; k < sample_count; k++) {
    if (samples[k].t_ms < samples[k - 1].t_ms) {
      Serial.printf("WARN: time decreased at row %u\n", (unsigned)k);
      break;
    }
  }

  return true;
}

// ---------- Update registers from current sample ----------
static void publish_sample(const Sample& s) {
  write_le_u32(&regs[0], s.t_ms);
  write_le_i32(&regs[4], s.alt_m_x1000);
  write_le_i32(&regs[8], s.ax_ftps2_x1000);
  write_le_i32(&regs[12], s.ay_ftps2_x1000);
  write_le_i32(&regs[16], s.az_ftps2_x1000);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("ESP32 I2C flight profile sim (CSV from SPIFFS)");

  // Init regs
  for (int i = 0; i < REG_LEN; i++) regs[i] = 0;

  // Mount SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("ERROR: SPIFFS mount failed");
    while (true) delay(1000);
  }

  // Load CSV
  if (!load_csv("/flight.csv")) {
    Serial.println("ERROR: Failed to load /flight.csv");
    while (true) delay(1000);
  }

  idx = 0;
  playback_start_ms = millis();
  publish_sample(samples[0]);

  // I2C slave init (the form that worked for you)
  Wire.begin(I2C_ADDR);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

  Serial.printf("I2C slave ready at 0x%02X\n", I2C_ADDR);
}

void loop() {
  if (!running || sample_count < 2) {
    delay(5);
    return;
  }

  uint32_t elapsed = millis() - playback_start_ms;

  // Advance idx while the next sample time has passed
  while (idx + 1 < sample_count && elapsed >= samples[idx + 1].t_ms) {
    idx++;
    publish_sample(samples[idx]);
  }

  // Loop back to start when done
  if (idx + 1 >= sample_count) {
    idx = 0;
    playback_start_ms = millis();
    publish_sample(samples[0]);
  }

  delay(1);
}
