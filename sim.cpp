// Host simulator: runs the unmodified race.cpp against a model of the track + a diff-drive robot.
#include <UserAPI.h>
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <string>
#include <random>
#include <algorithm>
using namespace std;

// ---------------- parameters (override with key=value on the command line) ----------------
static double S = 0.005;        // metres per diagram unit
static double WHEEL_D = 0.065;  // wheel diameter, m (GUESS)
static double WHEELBASE = 0.14; // m (GUESS)
static double SENS_FWD = 0.07;  // sensor bar ahead of axle, m (GUESS)
static double PITCH = 0.0095;   // QTR-8RC pitch, m
static double LINE_W = 0.02;    // tape width, m (GUESS)
static double DASH = 0.06, GAP = 0.08;  // m (from photo, rough)
static double TAU = 0.12;       // wheel speed lag, s
static double RSCALE = 1.0;     // right wheel speed factor
static double NOISE = 40.0;     // raw sensor noise sigma
static unsigned SEED = 1;
static double TMAX = 900.0;
static int PLOT = 0;

struct P { double x, y; };
static double hyp(P a, P b) { return hypot(a.x - b.x, a.y - b.y); }

// ---------------- track raster ----------------
static const double GX0 = -0.3, GY0 = -3.2, GRES = 0.001;
static const int GW = 3000, GH = 3400;
static vector<unsigned char> grid;
static void stampSeg(P a, P b, double hw) {
  int x0 = (int)floor((min(a.x, b.x) - hw - GX0) / GRES), x1 = (int)ceil((max(a.x, b.x) + hw - GX0) / GRES);
  int y0 = (int)floor((min(a.y, b.y) - hw - GY0) / GRES), y1 = (int)ceil((max(a.y, b.y) + hw - GY0) / GRES);
  x0 = max(x0, 0); y0 = max(y0, 0); x1 = min(x1, GW - 1); y1 = min(y1, GH - 1);
  double dx = b.x - a.x, dy = b.y - a.y, L2 = dx * dx + dy * dy;
  for (int iy = y0; iy <= y1; iy++) for (int ix = x0; ix <= x1; ix++) {
    double px = GX0 + ix * GRES, py = GY0 + iy * GRES;
    double t = L2 > 0 ? ((px - a.x) * dx + (py - a.y) * dy) / L2 : 0; t = max(0.0, min(1.0, t));
    double qx = a.x + t * dx - px, qy = a.y + t * dy - py;
    if (qx * qx + qy * qy <= hw * hw) grid[(size_t)iy * GW + ix] = 1;
  }
}
static bool blackAt(double x, double y) {
  int ix = (int)floor((x - GX0) / GRES), iy = (int)floor((y - GY0) / GRES);
  if (ix < 0 || iy < 0 || ix >= GW || iy >= GH) return false;
  return grid[(size_t)iy * GW + ix] != 0;
}

// ---------------- main path (for progress / lap detection) ----------------
static vector<P> mp; static vector<double> mps;
struct Mark { string name; double s; double tFirst; };
static vector<Mark> marks;
static P W(double xi, double yi) { return {xi * S, -yi * S}; }
static void addMain(P p) { if (mp.empty()) mps.push_back(0); else mps.push_back(mps.back() + hyp(mp.back(), p)); mp.push_back(p); }
static void mark(const char* n) { marks.push_back({n, mps.back(), -1}); }
// mode 0 solid, 1 dashed, 2 invisible (white stripe inside the patch)
static void lineTo(P to, int mode = 0) {
  P from = mp.back(); addMain(to);
  if (mode == 0) stampSeg(from, to, LINE_W / 2);
  else if (mode == 1) {
    double L = hyp(from, to), pos = 0;
    while (pos < L) {
      double e = min(pos + DASH, L);
      P a = {from.x + (to.x - from.x) * pos / L, from.y + (to.y - from.y) * pos / L};
      P b = {from.x + (to.x - from.x) * e / L,   from.y + (to.y - from.y) * e / L};
      stampSeg(a, b, LINE_W / 2);
      pos += DASH + GAP;
    }
  }
}
// arc in diagram coords: x = cx + r cos a, y = cy + sgn*r sin a  (a in degrees)
static void arcTo(double cx, double cy, double r, double a0, double a1, double sgn, int n = 40) {
  for (int k = 1; k <= n; k++) {
    double a = (a0 + (a1 - a0) * k / n) * M_PI / 180.0;
    lineTo(W(cx + r * cos(a), cy + sgn * r * sin(a)), 0);
  }
}
static void polyStamp(const vector<P>& v) { for (size_t i = 0; i + 1 < v.size(); i++) stampSeg(v[i], v[i + 1], LINE_W / 2); }

static void buildTrack() {
  grid.assign((size_t)GW * GH, 0);
  addMain(W(166, 392)); mark("start");
  lineTo(W(166, 420)); lineTo(W(247, 420)); lineTo(W(247, 475)); lineTo(W(35, 475));
  mark("left_side"); lineTo(W(35, 350));
  mark("1_zigzag");
  lineTo(W(13, 342)); lineTo(W(47, 314)); lineTo(W(13, 286)); lineTo(W(47, 258)); lineTo(W(35, 232));
  mark("approach_loop"); lineTo(W(35, 38));
  mark("2_first_loop");
  arcTo(23, 38, 12, 0, 270, -1, 60);                    // counter-clockwise loop, exits heading east
  mark("after_loop"); lineTo(W(115, 50));
  mark("3_parallel_lines"); lineTo(W(135, 52)); lineTo(W(388, 52));
  mark("4_dashed"); lineTo(W(388, 70)); lineTo(W(388, 330), 1); lineTo(W(388, 340));
  arcTo(360, 340, 28, 0, 180, +1, 40);                  // U-turn at the bottom right
  mark("up_to_circles"); lineTo(W(332, 152));
  mark("5_four_circles");
  // Original diagram: horizontal approach meets the RIGHT column's tangent
  // point (x=180,y=152); the lower-left circle exits at its bottom
  // (x=152,y=180). The old model incorrectly routed through the centre gap.
  lineTo(W(180, 152));
  lineTo(W(152, 152));
  arcTo(152, 166, 14, -90, 90, +1, 48);  // follow the lower-left circle's right arc
  lineTo(W(152, 207));
  mark("6_inverted_patch"); lineTo(W(152, 392), 2);
  mark("finish");
  // decoy upper branch of the parallel lines
  polyStamp({W(115, 50), W(135, 22), W(330, 22), W(355, 43)});
  // four circles
  const double cc[4][2] = {{152, 138}, {180, 138}, {152, 166}, {180, 166}};
  for (int c = 0; c < 4; c++) {
    vector<P> v;
    for (int k = 0; k <= 72; k++) { double a = 2 * M_PI * k / 72; v.push_back(W(cc[c][0] + 14 * cos(a), cc[c][1] + 14 * sin(a))); }
    polyStamp(v);
  }
  // black patch with a white stripe down the middle
  P pa = W(152 - 12.5, 392), pb = W(152 + 12.5, 207);
  for (int iy = 0; iy < GH; iy++) for (int ix = 0; ix < GW; ix++) {
    double x = GX0 + ix * GRES, y = GY0 + iy * GRES;
    if (x >= pa.x && x <= pb.x && y >= pa.y && y <= pb.y) grid[(size_t)iy * GW + ix] = (fabs(x - 152 * S) < LINE_W / 2) ? 0 : 1;
  }
}

// ---------------- robot + firmware ----------------
static double rx, ry, th, stL = 0, stR = 0, cmdL = 0, cmdR = 0;
static uint32_t simMs = 0;
static mt19937 rng;
static normal_distribution<double> gauss(0.0, 1.0);

struct Rec { double t, x, y, s; string state; double th, ax, ay, cl, cr; int raw[8]; };
static int lastRaw[8];
static vector<Rec> traj;
static string lastState = "?";
static vector<int> gaps, dashes;
static int segIdx = 0;
static double progS = 0, maxS = 0, offDist = 0, offTime = 0;
static bool done = false; static string outcome = "RUNNING"; static double endT = 0;

static P barCentre() { return {rx + SENS_FWD * cos(th), ry + SENS_FWD * sin(th)}; }
static void updateProgress() {
  P b = barCentre();
  int N = (int)mp.size() - 1, lo = max(0, segIdx - 40), hi = min(N - 1, segIdx + 80);
  double best = 1e9; int bi = segIdx; double bs = progS;
  for (int i = lo; i <= hi; i++) {
    double dx = mp[i + 1].x - mp[i].x, dy = mp[i + 1].y - mp[i].y, L2 = dx * dx + dy * dy;
    double t = L2 > 0 ? ((b.x - mp[i].x) * dx + (b.y - mp[i].y) * dy) / L2 : 0; t = max(0.0, min(1.0, t));
    double qx = mp[i].x + t * dx, qy = mp[i].y + t * dy, d = hypot(b.x - qx, b.y - qy);
    if (d < best) { best = d; bi = i; bs = mps[i] + t * sqrt(L2); }
  }
  segIdx = bi; progS = bs; offDist = best;
  if (progS > maxS) maxS = progS;
  for (auto& m : marks) if (m.tFirst < 0 && progS >= m.s) m.tFirst = simMs / 1000.0;
}
static void checkEnd() {
  updateProgress();
  double t = simMs / 1000.0;
  if (offDist > 0.15) offTime += 0.02; else offTime = 0;
  if (progS >= mps.back() - 0.03 && t > 5) { done = true; outcome = "FINISHED"; }
  else if (offTime >= 3.0) { done = true; outcome = "LOST_LINE"; }
  else if (progS < maxS - 1.0) { done = true; outcome = "WRONG_WAY"; }
  else if (t > TMAX) { done = true; outcome = "TIMEOUT"; }
  if (done) endT = t;
}
static void physStep() {
  const double dt = 0.001;
  stL += (max(-70.0, min(70.0, cmdL)) - stL) * dt / TAU;
  stR += (max(-70.0, min(70.0, cmdR)) - stR) * dt / TAU;
  double vl = stL * M_PI * WHEEL_D / 60.0, vr = stR * RSCALE * M_PI * WHEEL_D / 60.0;
  double v = (vl + vr) / 2, w = (vr - vl) / WHEELBASE;
  rx += v * cos(th) * dt; ry += v * sin(th) * dt; th += w * dt;
}

UserSerial userserial;
void waitMS(uint32_t ms) {
  for (uint32_t k = 0; k < ms && !done; k++) {
    physStep(); simMs++;
    if (simMs % 20 == 0) { checkEnd(); if (PLOT) { P b = barCentre(); Rec r; r.t = simMs / 1000.0; r.x = b.x; r.y = b.y; r.s = progS; r.state = lastState; r.th = th; r.ax = rx; r.ay = ry; r.cl = stL; r.cr = stR * RSCALE; for (int q = 0; q < 8; q++) r.raw[q] = lastRaw[q]; traj.push_back(r); } }
  }
}
uint32_t getTimeMS() { return simMs; }
void UserSerial::print(const char*) {} void UserSerial::print(int) {} void UserSerial::print(float, int) {}
void UserSerial::println(const char* m) { if (getenv("SIMVERBOSE")) ::printf("%s\n", m); }
void UserSerial::println(int) {} void UserSerial::println(float, int) {}
void UserSerial::printf(const char* fmt, ...) {
  char buf[400]; va_list a; va_start(a, fmt); vsnprintf(buf, sizeof buf, fmt, a); va_end(a);
  if (!strncmp(buf, "RUN,", 4)) {
    const char* p = strchr(buf + 4, ','); if (p) { const char* q = strchr(p + 1, ','); if (q) lastState.assign(p + 1, q - p - 1); }
  } else if (!strncmp(buf, "EVT,GAP_MS,", 11)) gaps.push_back(atoi(buf + 11));
  else if (!strncmp(buf, "EVT,DASH_MS,", 12)) dashes.push_back(atoi(buf + 12));
  if (getenv("SIMVERBOSE")) ::printf("%s", buf);
}
int getIRSensorCount() { return 8; }
void readIRRawArray(int* out) {
  P b = barCentre(); double lx = -sin(th), ly = cos(th), fx = cos(th), fy = sin(th);
  const double off[5][2] = {{0, 0}, {0.003, 0}, {-0.003, 0}, {0, 0.003}, {0, -0.003}};
  for (int i = 0; i < 8; i++) {
    double lat = (3.5 - i) * PITCH; int cnt = 0;
    for (int k = 0; k < 5; k++) {
      double x = b.x + lx * (lat + off[k][1]) + fx * off[k][0], y = b.y + ly * (lat + off[k][1]) + fy * off[k][0];
      if (blackAt(x, y)) cnt++;
    }
    double v = 150 + 2050 * (cnt / 5.0) + NOISE * gauss(rng);
    out[i] = (int)max(0.0, min(2500.0, v)); lastRaw[i] = out[i];
  }
}
void readIRArray(int* out) { int r[8]; readIRRawArray(r); for (int i = 0; i < 8; i++) out[i] = r[i] > 1250; }
void setWheelRPM(float l, float r) { cmdL = l; cmdR = r; }
void stopMotors() { cmdL = cmdR = 0; }
void getWheelRPMNow(float* l, float* r) { *l = (float)stL; *r = (float)(stR * RSCALE); }
bool motorsOnline(bool* l, bool* r) { *l = *r = true; return true; }
bool readMotorMode(int* l, int* r) { *l = *r = 2; return true; }

void userLoop();
int main(int argc, char** argv) {
  for (int i = 1; i < argc; i++) {
    char k[32]; double v;
    if (sscanf(argv[i], "%31[^=]=%lf", k, &v) != 2) continue;
    if (!strcmp(k, "scale")) S = v; else if (!strcmp(k, "wd")) WHEEL_D = v; else if (!strcmp(k, "wb")) WHEELBASE = v;
    else if (!strcmp(k, "fwd")) SENS_FWD = v; else if (!strcmp(k, "lw")) LINE_W = v; else if (!strcmp(k, "dash")) DASH = v;
    else if (!strcmp(k, "gap")) GAP = v; else if (!strcmp(k, "tau")) TAU = v; else if (!strcmp(k, "rscale")) RSCALE = v;
    else if (!strcmp(k, "noise")) NOISE = v; else if (!strcmp(k, "seed")) SEED = (unsigned)v;
    else if (!strcmp(k, "tmax")) TMAX = v; else if (!strcmp(k, "plot")) PLOT = (int)v;
  }
  rng.seed(SEED);
  buildTrack();
  // start: sensor bar 3 cm past the start line, heading south
  P b0 = {mp[0].x, mp[0].y - 0.03}; th = -M_PI / 2; rx = b0.x - SENS_FWD * cos(th); ry = b0.y - SENS_FWD * sin(th);
  updateProgress();
  while (!done) userLoop();
  const char* sec = "start";
  for (auto& m : marks) if (progS >= m.s) sec = m.name.c_str();
  printf("RESULT scale=%.4f wb=%.3f wd=%.3f lw=%.3f rscale=%.2f seed=%u | %-9s t=%6.1fs  progress=%.2f/%.2f m  last_section=%s\n",
         S, WHEELBASE, WHEEL_D, LINE_W, RSCALE, SEED, outcome.c_str(), endT, progS, mps.back(), sec);
  if (getenv("SIMMARKS")) {
    for (auto& m : marks) printf("  MARK %-18s s=%6.2f m  first reached t=%s\n", m.name.c_str(), m.s,
                                 m.tFirst < 0 ? "never" : (to_string((int)(m.tFirst * 10) / 10.0).substr(0, 6) + " s").c_str());
    if (!gaps.empty()) { printf("  gap events seen by controller (ms):"); for (int g : gaps) printf(" %d", g); printf("\n"); }
    if (!dashes.empty()) { printf("  dash events seen by controller (ms):"); for (int g : dashes) printf(" %d", g); printf("\n"); }
  }
  if (PLOT) {
    FILE* f = fopen("traj.csv", "w"); fprintf(f, "t,x,y,s,state,th,ax,ay,cl,cr,r0,r1,r2,r3,r4,r5,r6,r7\n");
    for (auto& r : traj) {
      fprintf(f, "%.2f,%.4f,%.4f,%.3f,%s,%.4f,%.4f,%.4f,%.1f,%.1f", r.t, r.x, r.y, r.s, r.state.c_str(), r.th, r.ax, r.ay, r.cl, r.cr);
      for (int q = 0; q < 8; q++) fprintf(f, ",%d", r.raw[q]);
      fprintf(f, "\n");
    }
    fclose(f);
    f = fopen("grid.bin", "wb"); fwrite(grid.data(), 1, grid.size(), f); fclose(f);
  }
  return 0;
}
