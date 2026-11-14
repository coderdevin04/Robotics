// ReRo Round 2 - line follower v2. Paste into the editor as user_code.cpp.
// Closed-loop wheel RPM (setWheelRPM), per-frame contrast normalisation (no
// calibrateIR sweep needed), a separate mode for the inverted (white-on-black) patch,
// gap / sharp-corner / search handling, and rich serial logging for tuning.
// NOT RUN ON THE REAL ROBOT: every number in TUNING is a starting guess.

#include <UserAPI.h>

// Per-sensor raw calibration. At boot, keep the robot lifted or motors disabled
// and sweep the sensor bar across BOTH white floor and black tape for ~4 seconds.
static int irWhite[8], irBlack[8];
static bool irCalibrated = false;

void userSetup() {
  brakeMotors();
  for (int i = 0; i < 8; ++i) { irWhite[i] = 2500; irBlack[i] = 0; }
  userserial.printf("IR_CAL,START: sweep all 8 sensors over white floor AND black tape for 4s\\n");
  for (int n = 0; n < 400; ++n) {
    int sample[8];
    readIRRawArray(sample);
    for (int i = 0; i < 8; ++i) {
      if (sample[i] < irWhite[i]) irWhite[i] = sample[i];
      if (sample[i] > irBlack[i]) irBlack[i] = sample[i];
    }
    waitMS(10);
  }
  int good = 0;
  for (int i = 0; i < 8; ++i) {
    int span = irBlack[i] - irWhite[i];
    if (span >= 250) ++good;
    userserial.printf("IR_CAL,S%d,white=%d,black=%d,span=%d\\n",
                      i, irWhite[i], irBlack[i], span);
  }
  irCalibrated = (good >= 6);
  userserial.printf("IR_CAL,%s\\n", irCalibrated ? "OK" : "WEAK: repeat sweep with every sensor crossing tape");
}

static int calibratedIR(int raw, int i) {
  if (!irCalibrated) return raw; // safe fallback if calibration contrast is insufficient
  int span = irBlack[i] - irWhite[i];
  if (span < 250) return raw;
  float q = 2500.0f * (float)(raw - irWhite[i]) / (float)span;
  if (q < 0.0f) q = 0.0f;
  if (q > 2500.0f) q = 2500.0f;
  return (int)q;
}

static float absf(float x) { return x < 0.0f ? -x : x; }
static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

// ============================ TUNING (edit me) =============================
// Speeds are wheel RPM (hard cap 70). Raise BASE_RPM in small steps once it holds the line.
static const float BASE_RPM  = 24.0f;   // straight-line speed
static const float MIN_RPM   = 16.0f;   // slowest tracking speed
static const float SLOW_RPM  = 20.0f;   // gaps, crossings, black patch
static const float PIVOT_RPM = 22.0f;   // each wheel, opposite directions, when searching
static const float SLOWDOWN  = 3.5f;    // RPM removed per sensor unit of |position|
static const float KP        = 6.5f;    // steering RPM per sensor unit (line right => +)
static const float KD        = 0.04f;   // steering RPM per (sensor unit / second)
static const float INV_KP    = 5.0f;    // steering gain while following the white stripe
// At the 2x2 circle junction, use damped position prediction so the sensor
// bar stays on the selected ring/connector instead of jumping to an adjacent ring.
static const float CIRCLE_PREDICT_GAIN = 0.50f;
static const float CIRCLE_FILTER_OLD   = 0.78f;
static const float LEFT_SCALE  = 1.0f;  // trim if the RPM feedback shows L/R mismatch
static const float RIGHT_SCALE = 1.0f;

static const float LOST_CONTRAST = 220.0f;  // (peak - floor) below this = no line structure
static const float DARK_ABS      = 1500.0f; // mean raw above this with no structure = all black
static const int   WIDE_N        = 6;       // >= this many dark sensors = wide patch/crossing
static const float EDGE_POS      = 1.8f;    // line lost with peak-held |pos| >= this = sharp corner
static const float EDGE_DECAY    = 0.98f;   // per-frame decay of that peak-held position
static const float TURN_DECAY    = 0.995f;  // per-frame decay of the remembered steering direction
static const float WIDE_RPM      = 20.0f;   // speed while a wide dark area is not yet the patch
static const float WIDE_KP       = 8.0f;    // steering gain in that state
static const uint32_t INV_ENTER_MS = 300;   // wide-dark this long => inverted patch
static const uint32_t INV_EXIT_MS  = 200;   // normal line this long => back to normal
static const uint32_t ADVANCE_MS = 800;     // after losing the line, first drive straight this long (bar is ahead of the axle)
static const uint32_t GAP_MS   = 900;       // straight this long when line vanishes near centre
static const uint32_t SWEEP_MS = 600;       // first search sweep length after GAP_MS (grows each pass)
static const uint32_t PRINT_MS = 200;       // serial log period (keep >= 100)
static const uint32_t MAX_RUN_MS = 0;       // 0 = never auto-stop
// ===========================================================================

void userLoop() {
  static bool started = false;
  static uint32_t t0, lastT, lastSeen, lastPrint, darkSince, brightSince, lostSince, seenSince;
  static bool darkOn = false, brightOn = false, inverted = false, wasLost = false, tracking = false;
  static float pos = 0.0f, prevPos = 0.0f, edgeMem = 0.0f, turnMem = 0.0f;   // peak-held raw position / steering

  uint32_t now = getTimeMS();
  if (!started) {
    started = true;
    t0 = lastT = lastSeen = lastPrint = seenSince = lostSince = now;
    bool lok = false, rok = false;
    bool ok = motorsOnline(&lok, &rok);
    userserial.printf("RUN_START,motors_ok=%d,L=%d,R=%d\n", (int)ok, (int)lok, (int)rok);
  }
  if (MAX_RUN_MS && (now - t0) > MAX_RUN_MS) {
    stopMotors();
    waitMS(50);
    return;
  }
  float dt = (float)(now - lastT) / 1000.0f;
  if (dt < 0.001f) dt = 0.001f;
  lastT = now;

  // ---- read and describe the frame (0 white .. 2500 black) ----
  int raw[8];
  readIRRawArray(raw);
  int val[8];
  int s[8];
  for (int i = 0; i < 8; i++) { val[i] = calibratedIR(raw[i], i); s[i] = val[i]; }
  for (int i = 1; i < 8; i++) {               // insertion sort
    int v = s[i], j = i - 1;
    while (j >= 0 && s[j] > v) { s[j + 1] = s[j]; j--; }
    s[j + 1] = v;
  }
  float base = (float)(s[0] + s[1] + s[2]) / 3.0f;   // floor level in this frame
  float peak = (float)s[7];
  float contrast = peak - base;
  float mean = 0.0f;
  for (int i = 0; i < 8; i++) mean += (float)val[i];
  mean /= 8.0f;

  bool structured = contrast >= LOST_CONTRAST;
  float thr = base + 0.45f * contrast;
  int nDark = 0, nLight = 0;
  if (structured) {
    for (int i = 0; i < 8; i++) { if ((float)val[i] > thr) nDark++; else nLight++; }
  }
  bool allDark   = (!structured) && (mean > DARK_ABS);
  // The four-circle cluster and parallel-line crossings can also produce a
  // wide/multi-run sensor frame.  Do NOT identify those as the inverted patch.
  // The actual patch has a centred white stripe: both middle sensors see white
  // while dark material is visible on both outer sides of the sensor bar.
  int darkLeft = 0, darkRight = 0;
  if (structured) {
    for (int i = 0; i < 3; ++i) if ((float)val[i] > thr) darkLeft++;
    for (int i = 5; i < 8; ++i) if ((float)val[i] > thr) darkRight++;
  }
  bool centeredStripe = structured && val[3] <= thr && val[4] <= thr;
  bool patchSignature = centeredStripe && nDark >= 4 && darkLeft >= 2 && darkRight >= 2 && mean > 850.0f;
  bool enterLike = patchSignature;
  bool stayLike  = inverted && (patchSignature || allDark || (structured && nDark >= 5 && mean > 1000.0f));

  // ---- inverted-patch mode, with hysteresis ----
  if (!inverted) {
    if (enterLike) {
      if (!darkOn) { darkOn = true; darkSince = now; }
      if ((now - darkSince) >= INV_ENTER_MS) { inverted = true; brightOn = false; }
    } else {
      darkOn = false;
    }
  } else {
    if (!stayLike) {
      if (!brightOn) { brightOn = true; brightSince = now; }
      if ((now - brightSince) >= INV_EXIT_MS) { inverted = false; brightOn = false; darkOn = false; }
    } else {
      brightOn = false;
    }
  }
  bool darkish = inverted ? stayLike : enterLike;

  float cl = 0.0f, cr = 0.0f;
  const char* st = "";
  bool lostNow = false;

  if (darkish) {
    lastSeen = now;
    tracking = false;
    if (inverted && structured && nLight >= 1 && nLight <= 4) {
      // Inside the black patch: follow the white stripe (the LIGHT sensors).
      float wsum = 0.0f, wpos = 0.0f;
      for (int i = 0; i < 8; i++) {
        if ((float)val[i] <= thr) {
          float w = peak - (float)raw[i];
          wsum += w;
          wpos += w * ((float)i - 3.5f);
        }
      }
      float p = (wsum > 0.0f) ? (wpos / wsum) : 0.0f;   // stripe right of centre => +
      float steer = INV_KP * p;
      cl = SLOW_RPM + steer;
      cr = SLOW_RPM - steer;
      st = "INV";
    } else if (inverted) {
      cl = cr = SLOW_RPM;                               // inside the patch, stripe not visible: straight
      st = "INVBLIND";
    } else {
      // Wide dark area that is not (yet) the patch. A crossing is symmetric (centroid ~ 0 => straight);
      // a sharp corner is lopsided (centroid off to one side => turn that way).
      float p = 0.0f;
      if (structured) {
        float wsum = 0.0f, wpos = 0.0f;
        for (int i = 0; i < 8; i++) {
          if ((float)val[i] > thr) {
            float w = (float)raw[i] - thr;
            wsum += w;
            wpos += w * ((float)i - 3.5f);
          }
        }
        if (wsum > 0.0f) p = wpos / wsum;
        if (absf(p) >= absf(edgeMem)) edgeMem = p; else edgeMem *= EDGE_DECAY;
      }
      float steer = WIDE_KP * p;
      cl = WIDE_RPM + steer;
      cr = WIDE_RPM - steer;
      st = "WIDE";
    }
  } else if (structured) {
    // Normal black line on white. pos > 0 means the line is to the right of centre.
    st = "TRACK";
    float tt = base + 0.28f * contrast;
    float wsum = 0.0f, wpos = 0.0f;
    int runs = 0; bool inRun = false;
    for (int i = 0; i < 8; i++) {
      bool dark = (float)val[i] > tt;
      if (dark && !inRun) { runs++; inRun = true; }
      if (!dark) inRun = false;
    }
    bool circleAmbiguous = (runs >= 2 && nDark >= 2 && nDark <= 7);
    float predicted = tracking ? clampf(pos + CIRCLE_PREDICT_GAIN * (pos - prevPos), -3.5f, 3.5f) : pos;
    float rawPos = predicted;
    if (circleAmbiguous && tracking) {
      // At the four-ring junction, do not average every visible circle edge.
      // Select the contiguous dark run nearest the line position predicted from
      // the previous frames, then calculate the centroid of that run only.
      float bestDist = 1.0e9f;
      int i = 0;
      while (i < 8) {
        while (i < 8 && (float)val[i] <= tt) ++i;
        if (i >= 8) break;
        int a = i;
        float rw = 0.0f, rp = 0.0f;
        while (i < 8 && (float)val[i] > tt) {
          float w = (float)raw[i] - tt;
          rw += w;
          rp += w * ((float)i - 3.5f);
          ++i;
        }
        if (rw > 1.0f) {
          float c = rp / rw;
          float dist = absf(c - predicted);
          if (dist < bestDist) { bestDist = dist; rawPos = c; }
        }
      }
    } else {
      for (int i = 0; i < 8; i++) {
        if ((float)val[i] > tt) {
          float sensorPos = (float)i - 3.5f;
          float w = (float)raw[i] - tt;
          wsum += w;
          wpos += w * sensorPos;
        }
      }
      rawPos = (wsum > 1.0f) ? (wpos / wsum) : predicted;
    }
    if (!tracking) { pos = rawPos; prevPos = rawPos; }
    else           { pos = circleAmbiguous ? (CIRCLE_FILTER_OLD * pos + (1.0f - CIRCLE_FILTER_OLD) * rawPos) : (0.5f * pos + 0.5f * rawPos); }
    float d = (pos - prevPos) / dt;
    if (circleAmbiguous) d = clampf(d, -12.0f, 12.0f);
    prevPos = pos;
    tracking = true;
    if (absf(rawPos) >= absf(edgeMem)) edgeMem = rawPos; else edgeMem *= EDGE_DECAY;
    lastSeen = now;

    float steer = KP * pos + KD * d;
    if (absf(steer) >= absf(turnMem)) turnMem = steer; else turnMem *= TURN_DECAY;
    float v = BASE_RPM - SLOWDOWN * absf(pos);
    if (v < MIN_RPM) v = MIN_RPM;
    cl = v + steer;                                      // line on the right -> left wheel faster
    cr = v - steer;
  } else {
    // Line lost.
    tracking = false;
    lostNow = true;
    uint32_t lost = now - lastSeen;
    float sd = (edgeMem >= 0.0f) ? 1.0f : -1.0f;
    float sdTurn = (turnMem >= 0.0f) ? 1.0f : -1.0f;   // which way we were already turning
    if (lost < ADVANCE_MS) {
      st = "ADVANCE";                                    // bar is ahead of the axle: bring the axle to the corner
      cl = cr = SLOW_RPM;
    } else if (absf(edgeMem) >= EDGE_POS) {
      st = "CORNER";                                     // lost at bar edge: spin toward it
      cl =  sd * PIVOT_RPM;
      cr = -sd * PIVOT_RPM;
    } else if (lost < GAP_MS) {
      st = "GAP";                                        // lost near centre: drive straight
      cl = cr = SLOW_RPM;
    } else {
      st = "SEARCH";                                     // gap too long: expanding left/right sweep
      uint32_t tt = lost - GAP_MS, k = 0, span = SWEEP_MS;
      while (tt >= span && k < 60) { tt -= span; k++; span += SWEEP_MS; }
      float sw = (k % 2 == 0) ? sdTurn : -sdTurn;
      cl =  sw * PIVOT_RPM;
      cr = -sw * PIVOT_RPM;
    }
  }

  // ---- log dash / gap durations (use them to set GAP_MS) ----
  if (lostNow && !wasLost) {
    wasLost = true;
    lostSince = now;
    if ((now - seenSince) >= 30) userserial.printf("EVT,DASH_MS,%ld\n", (long)(now - seenSince));
  } else if (!lostNow && wasLost) {
    wasLost = false;
    seenSince = now;
    if ((now - lostSince) >= 30) userserial.printf("EVT,GAP_MS,%ld\n", (long)(now - lostSince));
  }

  setWheelRPM(clampf(cl * LEFT_SCALE, -70.0f, 70.0f), clampf(cr * RIGHT_SCALE, -70.0f, 70.0f));

  if (now - lastPrint >= PRINT_MS) {
    lastPrint = now;
    float fl = 0.0f, fr = 0.0f;
    getWheelRPMNow(&fl, &fr);
    userserial.printf("RUN,%ld,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                      (long)(now - t0), st, nDark, (int)(pos * 100.0f),
                      (int)cl, (int)cr, (int)fl, (int)fr,
                      raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6], raw[7]);
  }
  waitMS(5);
}
