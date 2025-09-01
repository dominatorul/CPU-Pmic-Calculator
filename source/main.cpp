#include <switch.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <stdarg.h>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>
#include <random>
#include <algorithm>
#include <thread>
#include <mutex>
#include <atomic>
#include <arm_neon.h>

// Only include HID for newer libnx versions
#include <switch/services/hid.h>
#include <switch/services/sm.h>

#include "max17050.h"
#include "tmp451.h"
#include "sysclk/board.h"
#include "sysclk/clock_manager.h"
#include "sysclk/client/ipc.h"

#define CPU_CURRENT_LIMIT_A 5.0f
#define MEASUREMENT_DURATION_SEC 10
#define STABILIZATION_TIME_SEC 5
#define CPU_STRESS_THREADS 4

// CPU stress test parameters
#define RAYTRACING_ITERATIONS 100
#ifndef MATRIX_SIZE
#define MATRIX_SIZE 512
#endif
#define PRIME_SEARCH_LIMIT 100000

typedef struct {
    float voltage;
    float current;
    float power;
    float temperature_soc;
    float temperature_pcb;
    uint32_t cpu_freq;
    uint32_t cpu_voltage_mv;  // CPU voltage in millivolts
} PowerMeasurement;

// Global state
static bool g_isAppletMode = true;
static std::atomic<bool> g_stopStressTest{false};

// CPU stress test functions
static void runCpuStressTest();
static void stopCpuStressTest();

// Power measurement functions
static Result measurePowerAverage(PowerMeasurement* measurement, int duration_sec, FILE* log);
static Result measurePowerSingle(PowerMeasurement* measurement);
static void printMeasurement(const char* label, const PowerMeasurement* measurement);
static float calculateCpuCurrent(const PowerMeasurement* idle, const PowerMeasurement* load);

// sys-clk wrapper functions
static bool checkSysClkService();
static Result setSysClkOverride(SysClkModule module, uint32_t hz);

// Forward declaration for logging function
static void log_msgf(FILE* log, const char* fmt, ...) {
    if (!log) return;
    va_list ap;
    va_start(ap, fmt);
    vfprintf(log, fmt, ap);
    va_end(ap);
    fflush(log);
}

static Result getCurrentSysClkContext(SysClkContext* context);

// sys-clk service handle
static Service g_sysclkSrv;
static bool g_sysclkInitialized = false;

// CPU Stress Test Implementation - Raytracing + Matrix + Prime Search
constexpr double infinity_d = std::numeric_limits<double>::infinity();
constexpr double pi = 3.14159265358979323846;

inline double degrees_to_radians(double degrees) { return degrees * pi / 180.0; }

inline double clamp01(double x) {
    if (x < 0.0) return 0.0;
    if (x > 1.0) return 1.0;
    return x;
}

thread_local std::mt19937_64 tl_rng{std::random_device{}()};
thread_local std::uniform_real_distribution<double> tl_dist(0.0, 1.0);

inline double random_double() {
    return tl_dist(tl_rng);
}

inline double random_double(double minv, double maxv) {
    return minv + (maxv - minv) * random_double();
}

class alignas(16) vec3 {
public:
    union {
        double e[4];
        float64x2_t neon_lo, neon_hi;
    };

    vec3() { e[0] = e[1] = e[2] = e[3] = 0.0; }
    vec3(double e0, double e1, double e2) {
        e[0] = e0; e[1] = e1; e[2] = e2; e[3] = 0.0;
    }

    double x() const { return e[0]; }
    double y() const { return e[1]; }
    double z() const { return e[2]; }

    vec3 operator-() const {
        vec3 result;
        float64x2_t zero = vdupq_n_f64(0.0);
        float64x2_t lo = vld1q_f64(e);
        vst1q_f64(result.e, vsubq_f64(zero, lo));
        result.e[2] = -e[2];
        return result;
    }

    double operator[](int i) const { return e[i]; }
    double& operator[](int i) { return e[i]; }

    vec3& operator+=(const vec3& v) {
        float64x2_t a_lo = vld1q_f64(e);
        float64x2_t b_lo = vld1q_f64(v.e);
        vst1q_f64(e, vaddq_f64(a_lo, b_lo));
        e[2] += v.e[2];
        return *this;
    }

    vec3& operator*=(double t) {
        float64x2_t scalar = vdupq_n_f64(t);
        float64x2_t lo = vld1q_f64(e);
        vst1q_f64(e, vmulq_f64(lo, scalar));
        e[2] *= t;
        return *this;
    }

    double length_squared() const {
        float64x2_t lo = vld1q_f64(e);
        float64x2_t sq = vmulq_f64(lo, lo);
        double sum = vgetq_lane_f64(sq, 0) + vgetq_lane_f64(sq, 1) + e[2] * e[2];
        return sum;
    }

    double length() const {
        return std::sqrt(length_squared());
    }

    static vec3 random() {
        return vec3(random_double(), random_double(), random_double());
    }

    static vec3 random(double minv, double maxv) {
        return vec3(random_double(minv,maxv), random_double(minv,maxv), random_double(minv,maxv));
    }
};

using point3 = vec3;
using color = vec3;

inline vec3 operator+(const vec3& u, const vec3& v) {
    vec3 result;
    float64x2_t u_lo = vld1q_f64(u.e);
    float64x2_t v_lo = vld1q_f64(v.e);
    vst1q_f64(result.e, vaddq_f64(u_lo, v_lo));
    result.e[2] = u.e[2] + v.e[2];
    return result;
}

inline vec3 operator-(const vec3& u, const vec3& v) {
    vec3 result;
    float64x2_t u_lo = vld1q_f64(u.e);
    float64x2_t v_lo = vld1q_f64(v.e);
    vst1q_f64(result.e, vsubq_f64(u_lo, v_lo));
    result.e[2] = u.e[2] - v.e[2];
    return result;
}

inline vec3 operator*(const vec3& u, const vec3& v) {
    vec3 result;
    float64x2_t u_lo = vld1q_f64(u.e);
    float64x2_t v_lo = vld1q_f64(v.e);
    vst1q_f64(result.e, vmulq_f64(u_lo, v_lo));
    result.e[2] = u.e[2] * v.e[2];
    return result;
}

inline vec3 operator*(double t, const vec3& v) {
    vec3 result;
    float64x2_t scalar = vdupq_n_f64(t);
    float64x2_t v_lo = vld1q_f64(v.e);
    vst1q_f64(result.e, vmulq_f64(scalar, v_lo));
    result.e[2] = t * v.e[2];
    return result;
}

inline vec3 operator*(const vec3& v, double t) { return t * v; }
inline vec3 operator/(const vec3& v, double t) { return (1.0/t) * v; }

inline double dot(const vec3& u, const vec3& v) {
    float64x2_t u_lo = vld1q_f64(u.e);
    float64x2_t v_lo = vld1q_f64(v.e);
    float64x2_t prod = vmulq_f64(u_lo, v_lo);
    double sum = vgetq_lane_f64(prod, 0) + vgetq_lane_f64(prod, 1) + u.e[2] * v.e[2];
    return sum;
}

inline vec3 unit_vector(const vec3& v) {
    double len = v.length();
    return len > 0.0 ? v / len : vec3(0,0,0);
}

// CPU stress test workloads
static void cpuStressRaytracing() {
    // Simplified raytracing computation
    for (int iter = 0; iter < RAYTRACING_ITERATIONS && !g_stopStressTest.load(); iter++) {
        vec3 origin(0, 0, 0);
        vec3 direction = vec3::random(-1, 1);
        direction = unit_vector(direction);
        
        // Simulate ray-sphere intersection
        vec3 sphere_center = vec3::random(-5, 5);
        double sphere_radius = random_double(0.5, 2.0);
        
        vec3 oc = origin - sphere_center;
        double a = dot(direction, direction);
        double b = 2.0 * dot(oc, direction);
        double c = dot(oc, oc) - sphere_radius * sphere_radius;
        
        double discriminant = b * b - 4 * a * c;
        if (discriminant >= 0) {
            double sqrt_discriminant = std::sqrt(discriminant);
            double t1 = (-b - sqrt_discriminant) / (2 * a);
            // t2 is calculated but not used in this simplified version
            // double t2 = (-b + sqrt_discriminant) / (2 * a);
            
            // Simulate lighting calculations
            vec3 hit_point = origin + t1 * direction;
            vec3 normal = unit_vector(hit_point - sphere_center);
            vec3 light_dir = unit_vector(vec3::random(-1, 1));
            double intensity = std::max(0.0, dot(normal, light_dir));
            
            // Simulate color calculation
            vec3 color = vec3(intensity, intensity * 0.8, intensity * 0.6);
            volatile double result = color.length(); // Prevent optimization
            (void)result;
        }
    }
}

static void cpuStressMatrix() {
    // Matrix multiplication stress test
    const int size = MATRIX_SIZE / 8; // Reduced size for performance
    std::vector<std::vector<double>> A(size, std::vector<double>(size));
    std::vector<std::vector<double>> B(size, std::vector<double>(size));
    std::vector<std::vector<double>> C(size, std::vector<double>(size, 0.0));
    
    // Initialize matrices with random values
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            A[i][j] = random_double(-1, 1);
            B[i][j] = random_double(-1, 1);
        }
    }
    
    // Matrix multiplication
    for (int i = 0; i < size && !g_stopStressTest.load(); i++) {
        for (int j = 0; j < size; j++) {
            for (int k = 0; k < size; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    
    // Calculate sum to prevent optimization
    volatile double sum = 0.0;
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            sum += C[i][j];
        }
    }
}

static bool isPrime(uint64_t n) {
    if (n <= 1) return false;
    if (n <= 3) return true;
    if (n % 2 == 0 || n % 3 == 0) return false;
    
    for (uint64_t i = 5; i * i <= n; i += 6) {
        if (n % i == 0 || n % (i + 2) == 0) return false;
    }
    return true;
}

static void cpuStressPrimeSearch() {
    // Prime number search
    uint64_t count = 0;
    for (uint64_t n = 2; n < PRIME_SEARCH_LIMIT && !g_stopStressTest.load(); n++) {
        if (isPrime(n)) {
            count++;
        }
    }
    volatile uint64_t result = count; // Prevent optimization
    (void)result;
}

static void cpuStressFloatingPoint() {
    // Floating point intensive calculations
    for (int i = 0; i < 50000 && !g_stopStressTest.load(); i++) {
        double x = random_double(0, 2 * pi);
        volatile double result = std::sin(x) * std::cos(x) + std::tan(x / 2) + std::exp(x / 10) + std::log(x + 1);
        (void)result;
    }
}

static void cpuStressThread() {
    while (!g_stopStressTest.load()) {
        cpuStressRaytracing();
        cpuStressMatrix();
        cpuStressPrimeSearch();
        cpuStressFloatingPoint();
    }
}

int main(int argc, char* argv[]) {
    Result rc = 0;
    FILE* log = NULL;
    bool sysclkRunning = false;
    const int maxRetries = 2;
    int retryCount = 0;
    bool initSuccess = false;
    
    // Check if we're running in applet mode
    AppletType appletType = appletGetAppletType();
    g_isAppletMode = (appletType != AppletType_Application && appletType != AppletType_SystemApplication);
    
    // Open debug log file
    log = fopen("sdmc:/cpu_pmic_calculator_debug.log", "w");
    if (log) {
        fprintf(log, "=== CPU PMIC Calculator Debug Log ===\n");
        fprintf(log, "Step 1: Application started\n");
        fprintf(log, "Mode: %s (AppletType: %d)\n", g_isAppletMode ? "Applet" : "Application", appletType);
        fflush(log);
    }
    
    // Initialize console first - important for libnx 4.9.0
    consoleInit(NULL);
    printf("CPU PMIC Calculator Starting...\n");
    printf("Mode: %s\n", g_isAppletMode ? "Applet (Limited RAM)" : "Application (Full RAM)");
    consoleUpdate(NULL);
    
    if (log) {
        fprintf(log, "Step 2: Console initialized\n");
        fflush(log);
    }
    
    // Small delay to see console output
    svcSleepThread(1000000000); // 1 second
    
    printf("Nintendo Switch CPU PMIC Calculator\n");
    printf("===================================\n\n");
    printf("Debug: Initializing services...\n");
    consoleUpdate(NULL);
    
    // Initialize pad for input handling with proper configuration for libnx 4.9.0
    padConfigureInput(1, HidNpadStyleSet_NpadStandard);
    
    PadState pad;
    padInitializeDefault(&pad);
    
    if (log) {
        fprintf(log, "Step 5: Pad initialized\n");
        fflush(log);
    }
    
    printf("Debug: Pad initialized\n");
    consoleUpdate(NULL);
    
    // Initialize I2C
    printf("Debug: Initializing I2C...\n");
    consoleUpdate(NULL);
    
    rc = i2cInitialize();
    if (R_FAILED(rc)) {
        printf("ERROR: Failed to initialize I2C: 0x%x\n", rc);
        printf("This is required for power measurement.\n");
        printf("Press + to exit\n");
        
        if (log) {
            fprintf(log, "ERROR: i2cInitialize failed: 0x%x\n", rc);
            fflush(log);
        }
        
        while (appletMainLoop()) {
            padUpdate(&pad);
            u64 kDown = padGetButtonsDown(&pad);
            if (kDown & HidNpadButton_Plus) break;
            consoleUpdate(NULL);
            svcSleepThread(16666666);
        }
        consoleExit(NULL);
        if (log) {
            fprintf(log, "Cleanup: consoleExit\n");
            fprintf(log, "=== End of debug log ===\n");
            fclose(log);
        }
        return 1;
    }
    
    if (log) {
        fprintf(log, "Step 6: I2C initialized successfully\n");
        fflush(log);
    }
    
    printf("Debug: I2C initialized successfully\n");
    consoleUpdate(NULL);
    
    // Check sys-clk service availability
    printf("Debug: Checking sys-clk service availability...\n");
    consoleUpdate(NULL);
    
    sysclkRunning = checkSysClkService();
    printf("Debug: sys-clk service check: %s\n", sysclkRunning ? "YES" : "NO");
    if (log) {
        fprintf(log, "Step 6.1: sys-clk service check: %s\n", sysclkRunning ? "YES" : "NO");
        fflush(log);
    }
    
    // Initialize sys-clk service with retry
    if (sysclkRunning) {
        printf("Debug: Initializing sys-clk service...\n");
        consoleUpdate(NULL);
        
        while (retryCount < maxRetries && !initSuccess) {
            rc = smGetService(&g_sysclkSrv, SYSCLK_IPC_SERVICE_NAME);
            if (R_SUCCEEDED(rc)) {
                g_sysclkInitialized = true;
                initSuccess = true;
                if (log) {
                    fprintf(log, "Step 7: sys-clk service initialized on attempt %d\n", retryCount + 1);
                    fflush(log);
                }
            } else {
                retryCount++;
                printf("Attempt %d: Failed to initialize sys-clk service: 0x%x\n", retryCount, rc);
                if (log) {
                    fprintf(log, "Attempt %d: smGetService failed: 0x%x\n", retryCount, rc);
                    fflush(log);
                }
                if (retryCount < maxRetries) {
                    printf("Retrying in 500ms...\n");
                    consoleUpdate(NULL);
                    svcSleepThread(500000000); // 500ms delay
                }
            }
        }
    } else {
        printf("WARNING: sys-clk service not detected.\n");
        if (log) {
            fprintf(log, "WARNING: sys-clk service not detected.\n");
            fflush(log);
        }
    }
    
    if (!initSuccess && sysclkRunning) {
        printf("WARNING: Failed to initialize sys-clk service after attempts. Continuing in fallback mode (no CPU frequency control).\n");
        printf("Power measurements may be inaccurate.\n");
        printf("Press A to continue, + to exit\n");
        
        if (log) {
            fprintf(log, "WARNING: sys-clk service init failed after attempts. Using fallback mode\n");
            fflush(log);
        }
        
        while (appletMainLoop()) {
            padUpdate(&pad);
            u64 kDown = padGetButtonsDown(&pad);
            if (kDown & HidNpadButton_Plus) {
                i2cExit();
                consoleExit(NULL);
                if (log) {
                    fprintf(log, "Cleanup: i2cExit\n");
                    fprintf(log, "Cleanup: consoleExit\n");
                    fprintf(log, "=== End of debug log ===\n");
                    fclose(log);
                }
                return 1;
            }
            if (kDown & HidNpadButton_A) break;
            consoleUpdate(NULL);
            svcSleepThread(16666666);
        }
    } else if (initSuccess) {
        // Verify sys-clk service is working by getting API version
        printf("Debug: Verifying sys-clk service...\n");
        consoleUpdate(NULL);
        
        u32 apiVersion = 0;
        rc = serviceDispatchOut(&g_sysclkSrv, SysClkIpcCmd_GetApiVersion, apiVersion);
        if (R_SUCCEEDED(rc)) {
            printf("Debug: sys-clk API version: %u\n", apiVersion);
            if (log) {
                fprintf(log, "Step 8: sys-clk API version: %u\n", apiVersion);
                fflush(log);
            }
        } else {
            printf("WARNING: Failed to get sys-clk API version: 0x%x\n", rc);
            printf("Continuing in fallback mode (no CPU frequency control).\n");
            printf("Press A to continue, + to exit\n");
            
            if (log) {
                fprintf(log, "WARNING: Failed to get sys-clk API version: 0x%x\n", rc);
                fflush(log);
            }
            
            while (appletMainLoop()) {
                padUpdate(&pad);
                u64 kDown = padGetButtonsDown(&pad);
                if (kDown & HidNpadButton_Plus) {
                    if (g_sysclkInitialized) serviceClose(&g_sysclkSrv);
                    i2cExit();
                    consoleExit(NULL);
                    if (log) {
                        fprintf(log, "Cleanup: serviceClose\n");
                        fprintf(log, "Cleanup: i2cExit\n");
                        fprintf(log, "Cleanup: consoleExit\n");
                        fprintf(log, "=== End of debug log ===\n");
                        fclose(log);
                    }
                    return 1;
                }
                if (kDown & HidNpadButton_A) break;
                consoleUpdate(NULL);
                svcSleepThread(16666666);
            }
            initSuccess = false; // Switch to fallback mode
        }
    }
    
    if (log) {
        fprintf(log, "Step 9: Ready for main loop\n");
        fflush(log);
    }
    
    printf("\nInitialization complete!%s\n", initSuccess ? "" : " (Fallback mode)");
    printf("CPU Current Limit: %.1fA\n\n", CPU_CURRENT_LIMIT_A);
    printf("This tool will:\n");
    printf("1. Measure idle power consumption at 204 MHz\n");
    printf("2. Test at user's current sys-clk frequency setting\n");
    printf("3. Calculate CPU current draw\n\n");
    
    printf("Press A to start measurement\n");
    printf("Press + to exit\n");
    consoleUpdate(NULL);
    
    // Main loop with proper button handling
    while (appletMainLoop()) {
        // Update the pad state
        padUpdate(&pad);
        
        // Get the current button presses
        u64 kDown = padGetButtonsDown(&pad);
        
        if (kDown & HidNpadButton_Plus) {
            log_msgf(log, "DEBUG: Button + pressed\n");
            printf("\nExiting...\n");
            consoleUpdate(NULL);
            if (log) {
                fprintf(log, "User requested exit\n");
                fflush(log);
            }
            break;
        }
        
        if (kDown & HidNpadButton_A) {
            log_msgf(log, "DEBUG: Button A pressed\n");
            printf("\nStarting power measurement...\n");
            consoleUpdate(NULL);
            
            if (log) {
                fprintf(log, "Step 10: Starting measurement\n");
                fflush(log);
            }
            
            // Declare variables
            PowerMeasurement idleMeasurement = {0};
            PowerMeasurement loadMeasurement = {0};
            uint32_t userCpuFreq = 0;
            uint32_t originalOverride = 0;
            bool hadOverride = false;
            
            // Get user's current CPU frequency setting before testing
            if (initSuccess) {
                // Check if there's an existing override
                SysClkContext context;
                rc = getCurrentSysClkContext(&context);
                if (R_SUCCEEDED(rc)) {
                    originalOverride = context.overrideFreqs[SysClkModule_CPU];
                    hadOverride = (originalOverride > 0);
                    
                    // Use override if present, otherwise use profile frequency
                    userCpuFreq = hadOverride ? originalOverride : context.freqs[SysClkModule_CPU];
                    
                    printf("\nUser's current CPU frequency: %.1f MHz %s\n", 
                           userCpuFreq / 1000000.0f,
                           hadOverride ? "(override)" : "(profile)");
                    consoleUpdate(NULL);
                    
                    if (log) {
                        fprintf(log, "User's current CPU frequency: %.1f MHz %s\n", 
                                userCpuFreq / 1000000.0f,
                                hadOverride ? "(override)" : "(profile)");
                        fflush(log);
                    }
                }
            }
            
            // Step 1: Measure idle power (no CPU load, minimum frequency)
            printf("\n1. Measuring idle power consumption...\n");
            consoleUpdate(NULL);
            
            if (log) {
                fprintf(log, "Step 11: Starting idle power measurement\n");
                fflush(log);
            }
            
            if (initSuccess) {
                printf("   Setting minimum CPU frequency (204 MHz) temporarily...\n");
                consoleUpdate(NULL);
                
                rc = setSysClkOverride(SysClkModule_CPU, 204000000); // 204 MHz
                if (R_FAILED(rc)) {
                    printf("   WARNING: Failed to set CPU frequency: 0x%x\n", rc);
                    printf("   Continuing without CPU frequency control\n");
                    consoleUpdate(NULL);
                    if (log) {
                        fprintf(log, "WARNING: Failed to set CPU freq to 204MHz: 0x%x\n", rc);
                        fflush(log);
                    }
                }
            } else {
                printf("   Skipping CPU frequency setting (fallback mode)\n");
                consoleUpdate(NULL);
            }
            
            printf("   Stabilizing system (no CPU load) for %d seconds...\n", STABILIZATION_TIME_SEC);
            consoleUpdate(NULL);
            
            // Wait for stabilization with no CPU activity
            for (int i = STABILIZATION_TIME_SEC; i > 0; i--) {
                printf("   Stabilizing... %d seconds remaining\n", i);
                consoleUpdate(NULL);
                sleep(1);
            }
            
            printf("   Measuring idle power for %d seconds...\n", MEASUREMENT_DURATION_SEC);
            consoleUpdate(NULL);
            
            if (log) {
                fprintf(log, "Step 11.1: About to call measurePowerAverage for idle\n");
                fflush(log);
            }
            
            rc = measurePowerAverage(&idleMeasurement, MEASUREMENT_DURATION_SEC, log);
            if (R_FAILED(rc)) {
                printf("   ERROR: Failed to measure idle power: 0x%x\n", rc);
                consoleUpdate(NULL);
                
                if (log) {
                    fprintf(log, "ERROR: measurePowerAverage (idle) failed: 0x%x\n", rc);
                    fflush(log);
                }
                goto measurement_cleanup;
            }
            
            if (log) {
                fprintf(log, "Step 12: Idle measurement complete\n");
                fflush(log);
            }
            
            printMeasurement("Idle", &idleMeasurement);
            consoleUpdate(NULL);
            
            // Step 2: Measure load power with CPU stress test at user's frequency
            printf("\n2. Measuring load power consumption with CPU stress test...\n");
            consoleUpdate(NULL);
            
            if (initSuccess) {
                // Set CPU frequency to user's setting
                printf("   Setting CPU frequency to user's setting: %.1f MHz\n", userCpuFreq / 1000000.0f);
                consoleUpdate(NULL);
                
                rc = setSysClkOverride(SysClkModule_CPU, userCpuFreq);
                if (R_FAILED(rc)) {
                    printf("   WARNING: Failed to set CPU frequency: 0x%x\n", rc);
                    printf("   Continuing without CPU frequency control\n");
                    consoleUpdate(NULL);
                    
                    if (log) {
                        fprintf(log, "WARNING: Failed to set CPU freq to %.1fMHz: 0x%x\n", userCpuFreq / 1000000.0f, rc);
                        fflush(log);
                    }
                }
            } else {
                printf("   Skipping CPU frequency setting (fallback mode)\n");
                consoleUpdate(NULL);
            }
            
            printf("   Starting CPU stress test (raytracing + matrix + primes + floating point)...\n");
            printf("   Using %d threads for maximum CPU load...\n", CPU_STRESS_THREADS);
            consoleUpdate(NULL);
            
            if (log) {
                fprintf(log, "Step 13: About to start CPU stress test\n");
                fflush(log);
            }
            
            // Start CPU stress test
            g_stopStressTest.store(false);
            runCpuStressTest();
            
            printf("   Stabilizing CPU load for %d seconds...\n", STABILIZATION_TIME_SEC);
            consoleUpdate(NULL);
            
            // Stabilize for a few seconds
            sleep(STABILIZATION_TIME_SEC);
            
            if (log) {
                fprintf(log, "Step 15: CPU stabilization complete, starting load measurement\n");
                fflush(log);
            }
            
            printf("   Measuring load power for %d seconds while running CPU stress test...\n", MEASUREMENT_DURATION_SEC);
            consoleUpdate(NULL);
            
            rc = measurePowerAverage(&loadMeasurement, MEASUREMENT_DURATION_SEC, log);
            if (R_FAILED(rc)) {
                printf("   ERROR: Failed to measure load power: 0x%x\n", rc);
                consoleUpdate(NULL);
                
                if (log) {
                    fprintf(log, "ERROR: measurePowerAverage (load) failed: 0x%x\n", rc);
                    fflush(log);
                }
                stopCpuStressTest();
                goto measurement_cleanup;
            }
            
            // Stop CPU stress test
            stopCpuStressTest();
            
            if (log) {
                fprintf(log, "Step 16: Load measurement complete\n");
                fflush(log);
            }
            
            printMeasurement("Load", &loadMeasurement);
            consoleUpdate(NULL);
            
            // Step 3: Calculate CPU current
            printf("\n3. CPU Power Analysis:\n");
            printf("   =====================\n");
            
            {
                float cpuPower = loadMeasurement.power - idleMeasurement.power;
                float cpuCurrent = calculateCpuCurrent(&idleMeasurement, &loadMeasurement);
                float currentPercentage = (cpuCurrent / CPU_CURRENT_LIMIT_A) * 100.0f;
                
                printf("   CPU Power Draw: %.2fW\n", cpuPower);
                printf("   CPU Current Draw: %.2fA\n", cpuCurrent);
                printf("   Current Usage: %.1f%% of %.1fA limit\n", currentPercentage, CPU_CURRENT_LIMIT_A);
                
                if (cpuCurrent > CPU_CURRENT_LIMIT_A) {
                    printf("   WARNING: CPU current exceeds safe limit!\n");
                } else if (currentPercentage > 90.0f) {
                    printf("   CAUTION: CPU current is near the limit!\n");
                } else {
                    printf("   Status: CPU current is within safe limits\n");
                }
                
                consoleUpdate(NULL);
                
                if (log) {
                    fprintf(log, "Measurement complete:\n");
                    fprintf(log, "  CPU Power: %.2fW\n", cpuPower);
                    fprintf(log, "  CPU Current: %.2fA\n", cpuCurrent);
                    fprintf(log, "  Current Usage: %.1f%%\n", currentPercentage);
                    fflush(log);
                }
            }
            
            measurement_cleanup:
            // Restore user's original settings
            if (initSuccess) {
                printf("\n   Restoring user settings...\n");
                consoleUpdate(NULL);
                
                if (hadOverride) {
                    // Restore the original override
                    Result cleanup_rc = setSysClkOverride(SysClkModule_CPU, originalOverride);
                    if (R_FAILED(cleanup_rc)) {
                        printf("   WARNING: Failed to restore CPU override: 0x%x\n", cleanup_rc);
                        if (log) {
                            fprintf(log, "WARNING: Failed to restore CPU override: 0x%x\n", cleanup_rc);
                            fflush(log);
                        }
                    }
                } else {
                    // Remove the override we set (set to 0 to remove)
                    Result cleanup_rc = setSysClkOverride(SysClkModule_CPU, 0);
                    if (R_FAILED(cleanup_rc)) {
                        printf("   WARNING: Failed to remove CPU override: 0x%x\n", cleanup_rc);
                        if (log) {
                            fprintf(log, "WARNING: Failed to remove CPU override: 0x%x\n", cleanup_rc);
                            fflush(log);
                        }
                    }
                }
                
                printf("   User settings restored\n");
                consoleUpdate(NULL);
            }
            
            if (log) {
                fprintf(log, "Step 17: Measurement cycle complete\n");
                fflush(log);
            }
            
            printf("\nMeasurement complete. Press A to measure again, or + to exit\n");
            consoleUpdate(NULL);
        }
        
        consoleUpdate(NULL);
        svcSleepThread(16666666); // ~60 FPS for responsive controls
    }
    
    // Cleanup
    stopCpuStressTest();
    
    if (g_sysclkInitialized) serviceClose(&g_sysclkSrv);
    i2cExit();
    consoleExit(NULL);
    if (log) {
        fprintf(log, "Cleanup: %s\n", g_sysclkInitialized ? "serviceClose" : "skipping serviceClose");
        fprintf(log, "Cleanup: i2cExit\n");
        fprintf(log, "Cleanup: consoleExit\n");
        fprintf(log, "=== End of debug log ===\n");
        fclose(log);
    }
    
    return 0;
}

// CPU stress test implementation
static std::vector<std::thread> g_stressThreads;

static void runCpuStressTest() {
    g_stopStressTest.store(false);
    g_stressThreads.clear();
    g_stressThreads.reserve(CPU_STRESS_THREADS);
    
    for (int i = 0; i < CPU_STRESS_THREADS; i++) {
        g_stressThreads.emplace_back(cpuStressThread);
    }
}

static void stopCpuStressTest() {
    g_stopStressTest.store(true);
    for (auto& thread : g_stressThreads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    g_stressThreads.clear();
}

// sys-clk wrapper functions
static bool checkSysClkService() {
    Service testSrv;
    Result rc = smGetService(&testSrv, SYSCLK_IPC_SERVICE_NAME);
    
    if (R_SUCCEEDED(rc)) {
        serviceClose(&testSrv);
        return true;
    }
    
    return false;
}

static Result setSysClkOverride(SysClkModule module, uint32_t hz) {
    if (!g_sysclkInitialized)
        return MAKERESULT(Module_Libnx, LibnxError_NotInitialized);
    
    SysClkIpc_SetOverride_Args args = {
        .module = module,
        .hz = hz
    };
    
    return serviceDispatchIn(&g_sysclkSrv, SysClkIpcCmd_SetOverride, args);
}

static Result getCurrentSysClkContext(SysClkContext* context) {
    if (!g_sysclkInitialized || !context)
        return MAKERESULT(Module_Libnx, LibnxError_NotInitialized);
    
    return serviceDispatchOut(&g_sysclkSrv, SysClkIpcCmd_GetCurrentContext, *context);
}

static Result measurePowerSingle(PowerMeasurement* measurement) {
    if (!measurement) return -1;
    
    Result rc = 0;
    u16 regValue = 0;
    
// inside measurePowerSingle, where you get SysClkContext
if (g_sysclkInitialized) {
    SysClkContext context;
    rc = getCurrentSysClkContext(&context);
    if (R_SUCCEEDED(rc)) {
        measurement->cpu_freq = context.realFreqs[SysClkModule_CPU];

        // Sys-clk may return voltage in microvolts (µV). If value is large (> 100000),
        // we assume it's in µV and convert to mV. Otherwise assume it's already mV.
        uint32_t raw_volt = context.realVolts[0];
        if (raw_volt > 100000) {            // >100000 likely means µV (e.g. 590000)
            measurement->cpu_voltage_mv = raw_volt / 1000u;
        } else {
            measurement->cpu_voltage_mv = raw_volt;
        }
    } else {
        measurement->cpu_freq = 0;
        measurement->cpu_voltage_mv = 0;
    }
} else {
    measurement->cpu_freq = 0;
    measurement->cpu_voltage_mv = 0;
}

    
    // Read voltage (VCELL register)
    rc = Max17050ReadReg(MAX17050_VCELL, &regValue);
    if (R_FAILED(rc)) {
        return rc;
    }
    measurement->voltage = (float)regValue * 78.125e-6; // 78.125µV per LSB
    
    // Read current (Current register)
    rc = Max17050ReadReg(MAX17050_Current, &regValue);
    if (R_FAILED(rc)) {
        return rc;
    }
    
    // Convert current reading (signed 16-bit, 1.5625µV / Rsense per LSB)
    int16_t currentRaw = (int16_t)regValue;
    measurement->current = -(float)currentRaw * 1.5625e-6 / (max17050SenseResistor * 1e-3);  // Note the negative sign
    measurement->power = fabs(measurement->voltage * measurement->current);  // Use absolute value for power
    
    // Read temperatures
    rc = Tmp451GetSocTemp(&measurement->temperature_soc);
    if (R_FAILED(rc)) {
        measurement->temperature_soc = 0.0f;
    }
    
    rc = Tmp451GetPcbTemp(&measurement->temperature_pcb);
    if (R_FAILED(rc)) {
        measurement->temperature_pcb = 0.0f;
    }
    
    return 0;
}

static Result measurePowerAverage(PowerMeasurement* measurement, int duration_sec, FILE* log) {
    if (!measurement) return -1;
    
    float voltageSum = 0.0f;
    float currentSum = 0.0f;
    float powerSum = 0.0f;
    float tempSocSum = 0.0f;
    float tempPcbSum = 0.0f;
    uint32_t cpuFreqSum = 0;
    uint32_t cpuVoltageSum = 0;
    int validMeasurements = 0;
    
    time_t startTime = time(NULL);
    time_t lastUpdate = startTime;
    
    if (log) {
        fprintf(log, "Starting power measurement for %d seconds\n", duration_sec);
        fflush(log);
    }
    
    printf("   Taking measurements");
    consoleUpdate(NULL);
    
    while (time(NULL) - startTime < duration_sec) {
        PowerMeasurement temp = {0};
        
        Result rc = measurePowerSingle(&temp);
        if (R_SUCCEEDED(rc)) {
            voltageSum += temp.voltage;
            currentSum += fabs(temp.current);
            powerSum += fabs(temp.power);
            tempSocSum += temp.temperature_soc;
            tempPcbSum += temp.temperature_pcb;
            cpuFreqSum += temp.cpu_freq;
            cpuVoltageSum += temp.cpu_voltage_mv;
            validMeasurements++;
            
            if (log && validMeasurements % 5 == 0) {
                fprintf(log, "Measurement %d: %.2fW (V=%.3f, I=%.3f, CPU_V=%d mV)\n", 
                        validMeasurements, temp.power, temp.voltage, temp.current, temp.cpu_voltage_mv);
                fflush(log);
            }
        } else {
            if (log) {
                fprintf(log, "Failed measurement: 0x%x\n", rc);
                fflush(log);
            }
        }
        
        // Show progress
        time_t currentTime = time(NULL);
        if (currentTime != lastUpdate) {
            printf(".");
            consoleUpdate(NULL);
            lastUpdate = currentTime;
        }
        
        svcSleepThread(200000000); // 200ms between measurements
    }
    
    printf(" Done!\n");
    consoleUpdate(NULL);
    
    if (validMeasurements == 0) {
        if (log) {
            fprintf(log, "ERROR: No valid measurements taken\n");
            fflush(log);
        }
        return -2;
    }
    
    // Calculate averages
    measurement->voltage = voltageSum / validMeasurements;
    measurement->current = currentSum / validMeasurements;
    measurement->power = powerSum / validMeasurements;
    measurement->temperature_soc = tempSocSum / validMeasurements;
    measurement->temperature_pcb = tempPcbSum / validMeasurements;
    measurement->cpu_freq = cpuFreqSum / validMeasurements;
    measurement->cpu_voltage_mv = cpuVoltageSum / validMeasurements;
    
    if (log) {
        fprintf(log, "Power measurement complete: %d valid measurements\n", validMeasurements);
        fprintf(log, "Average power: %.2fW, CPU voltage: %d mV\n", measurement->power, measurement->cpu_voltage_mv);
        fflush(log);
    }
    
    return 0;
}

static void printMeasurement(const char* label, const PowerMeasurement* measurement) {
    printf("   %s Measurement:\n", label);
    printf("   - Battery Voltage: %.3fV\n", measurement->voltage);
    printf("   - Battery Current: %.3fA\n", measurement->current);
    printf("   - Total Power: %.2fW\n", measurement->power);
    if (measurement->cpu_freq) {
        printf("   - CPU Freq: %.1f MHz\n", measurement->cpu_freq / 1000000.0f);
    } else {
        printf("   - CPU Freq: Unknown (sys-clk unavailable)\n");
    }
    if (measurement->cpu_voltage_mv) {
        printf("   - CPU Voltage: %d mV\n", measurement->cpu_voltage_mv);
    } else {
        printf("   - CPU Voltage: Unknown (sys-clk unavailable)\n");
    }
    printf("   - SOC Temp: %.1f°C\n", measurement->temperature_soc);
    printf("   - PCB Temp: %.1f°C\n", measurement->temperature_pcb);
}

static float calculateCpuCurrent(const PowerMeasurement* idle, const PowerMeasurement* load) {
    float powerDifference = load->power - idle->power;
    
    // Use CPU voltage if available, otherwise use battery voltage
    float cpuVoltage = 0.0f;
    if (load->cpu_voltage_mv > 0) {
        cpuVoltage = load->cpu_voltage_mv / 1000.0f; // Convert mV to V
    } else {
        // Fallback to battery voltage
        cpuVoltage = (load->voltage + idle->voltage) / 2.0f;
    }
    
    if (cpuVoltage > 0.0f) {
        return powerDifference / cpuVoltage;
    }
    return 0.0f;
}