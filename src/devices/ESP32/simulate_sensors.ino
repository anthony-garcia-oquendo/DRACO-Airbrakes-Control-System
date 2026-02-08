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
// helper: find first index whose t_ms >= target
static size_t find_index_for_time(uint32_t target_ms) {
  if (!samples || sample_count == 0) return 0;
  // simple linear scan is fine for small files; upgrade to binary search if big
  for (size_t i = 0; i < sample_count; i++) {
    if (samples[i].t_ms >= target_ms) return i;
  }
  return sample_count - 1;
}

void onReceive(int len) {
  if (len <= 0) return;

  // First byte is register pointer (sensor-style)
  reg_ptr = (uint8_t)Wire.read();
  len--;

  // Only act on control register 0x1F
  if (reg_ptr != 0x1F || len <= 0) {
    // Drain any remaining bytes so Wire buffer stays clean
    while (len-- > 0) (void)Wire.read();
    return;
  }

  uint8_t cmd = (uint8_t)Wire.read();
  len--;

  switch (cmd) {
    case 0x00:  // STOP / PAUSE
      running = false;
      break;

    case 0x01:  // START (sync): reset to beginning and start NOW
      idx = 0;
      playback_start_ms = millis();
      running = true;
      if (samples && sample_count > 0) publish_sample(samples[0]);
      break;

    case 0x02:  // RESET to beginning but stay paused
      running = false;
      idx = 0;
      playback_start_ms = millis();
      if (samples && sample_count > 0) publish_sample(samples[0]);
      break;

    case 0x03: { // SEEK: next 4 bytes = target t_ms (LE). Start running.
      if (len < 4) break;
      uint32_t t_ms = 0;
      t_ms |= (uint32_t)Wire.read();
      t_ms |= (uint32_t)Wire.read() << 8;
      t_ms |= (uint32_t)Wire.read() << 16;
      t_ms |= (uint32_t)Wire.read() << 24;
      len -= 4;

      idx = find_index_for_time(t_ms);
      playback_start_ms = millis() - samples[idx].t_ms; // align elapsed so current sample matches target time
      running = true;
      publish_sample(samples[idx]);
      break;
    }

    case 0x04: { // SYNC NOW: next 4 bytes = t0_ms (LE), define time zero.
      // This is optional: lets Pi say "t=0 should be right now minus t0_ms".
      if (len < 4) break;
      uint32_t t0_ms = 0;
      t0_ms |= (uint32_t)Wire.read();
      t0_ms |= (uint32_t)Wire.read() << 8;
      t0_ms |= (uint32_t)Wire.read() << 16;
      t0_ms |= (uint32_t)Wire.read() << 24;
      len -= 4;

      // If Pi sends t0_ms=0, playback start = now (classic start alignment)
      playback_start_ms = millis() - t0_ms;
      break;
    }

    default:
      // unknown command, ignore
      break;
  }

  // Drain anything extra
  while (len-- > 0) (void)Wire.read();
}

void onRequest() {
  uint8_t p = reg_ptr;
  if (p >= REG_LEN) p = 0;
  Wire.write((uint8_t)regs[p]);
  reg_ptr = (uint8_t)(p + 1);
}

// ---------- CSV parsing ----------
static const float G_FTPS2 = 32.174f; // standard gravity in ft/s^2
static const bool FLIP_Z = false;     // set true if your IMU Z axis is opposite

static bool parse_csv_line(const String& line, Sample& out) {
  // Format:
  // time_s, altitude_m, altitude_asl_m, vertical_velocity_ftps, vertical_accel_ftps2

  String s = line;
  s.trim();
  if (s.length() == 0) return false;
  if (s[0] == '#') return false;

  char buf[192];
  if (s.length() >= (int)sizeof(buf)) return false;
  s.toCharArray(buf, sizeof(buf));

  char* tok = strtok(buf, ",");
  if (!tok) return false; float t_s = atof(tok);

  tok = strtok(nullptr, ",");
  if (!tok) return false; float alt_m = atof(tok);

  tok = strtok(nullptr, ",");
  if (!tok) return false; (void)atof(tok); // alt_asl ignored

  tok = strtok(nullptr, ",");
  if (!tok) return false; (void)atof(tok); // v_ftps ignored

  tok = strtok(nullptr, ",");
  if (!tok) return false; float a_vert_ftps2 = atof(tok); // signed vertical accel

  // IMU-like proper acceleration on Z:
  // - At rest: a_vert ~ 0, so az_imu ~ +g
  // - Free-fall: a_vert ~ -g, so az_imu ~ 0
  float az_imu = a_vert_ftps2 + G_FTPS2;

  // If your IMU Z axis is mounted opposite (down-positive), flip:
  if (FLIP_Z) az_imu = -az_imu;

  out.t_ms = (uint32_t)lroundf(t_s * 1000.0f);
  out.alt_m_x1000 = (int32_t)lroundf(alt_m * 1000.0f);

  // Only Z provided (1D sim)
  out.ax_ftps2_x1000 = 0;
  out.ay_ftps2_x1000 = 0;
  out.az_ftps2_x1000 = (int32_t)lroundf(az_imu * 1000.0f);

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
