/*
 * load_sweep.h
 *
 * Load-curve instrumentation: drives the speed target through a staircase or a
 * continuous ramp in closed loop and records averaged operating points into a
 * RAM table the host reads back. Only measured quantities are stored (speed,
 * duty, DC-bus voltage and current from the INA219); any model-based estimate is
 * left to the host.
 */

#ifndef INC_LOAD_SWEEP_H_
#define INC_LOAD_SWEEP_H_

#include <stdbool.h>
#include <stdint.h>

#define LOAD_SWEEP_CAPACITY 128U
/** Period of the speed/duty/reference samples inside a window. */
#define LOAD_SWEEP_SAMPLE_MS 10U
/** Idle-current averaging done before starting the motor from IDLE. */
#define LOAD_SWEEP_IQ_MS 500U
/** Maximum time from the start request to CLOSEDLOOP. */
#define LOAD_SWEEP_STARTUP_TIMEOUT_MS 15000U

#define LOAD_SWEEP_MIN_WINDOW_MS 100U
#define LOAD_SWEEP_MAX_WINDOW_MS 10000U
#define LOAD_SWEEP_MAX_SETTLE_MS 30000U
#define LOAD_SWEEP_MIN_STEP_RPM 10U
#define LOAD_SWEEP_MIN_RAMP_RPM_S 1U
#define LOAD_SWEEP_MAX_RAMP_RPM_S 5000U

/** Stop the motor when the sweep completes normally. */
#define LOAD_SWEEP_OPT_STOP_AT_END 0x01U

/* Point flags. */
#define LOAD_POINT_FLAG_NO_POWER 0x01U      /**< No INA219 sample in the window. */
#define LOAD_POINT_FLAG_PARTIAL_POWER 0x02U /**< Fewer than half the expected INA219 samples. */
#define LOAD_POINT_FLAG_DUTY_SATURATED 0x04U /**< Duty reached the PI output ceiling. */

typedef enum {
  LOAD_SWEEP_MODE_STAIRCASE = 0,
  LOAD_SWEEP_MODE_RAMP = 1,
} LoadSweepMode;

typedef enum {
  LOAD_SWEEP_STATE_IDLE = 0,
  LOAD_SWEEP_STATE_MEASURING_IQ = 1,
  LOAD_SWEEP_STATE_STARTING = 2,
  LOAD_SWEEP_STATE_MOVING = 3,
  LOAD_SWEEP_STATE_SETTLING = 4,
  LOAD_SWEEP_STATE_MEASURING = 5,
  LOAD_SWEEP_STATE_DONE = 6,
  LOAD_SWEEP_STATE_ABORTED = 7,
} LoadSweepState;

typedef enum {
  LOAD_SWEEP_ABORT_NONE = 0,
  LOAD_SWEEP_ABORT_USER = 1,
  LOAD_SWEEP_ABORT_MOTOR_LOST = 2,      /**< Left CLOSEDLOOP or auto-restarted. */
  LOAD_SWEEP_ABORT_STARTUP_TIMEOUT = 3,
  LOAD_SWEEP_ABORT_TABLE_FULL = 4,
  LOAD_SWEEP_ABORT_STOPPED = 5,         /**< STOP or ESTOP command. */
} LoadSweepAbortReason;

typedef enum {
  LOAD_SWEEP_START_OK = 0,
  LOAD_SWEEP_START_UNKNOWN_PARAM,
  LOAD_SWEEP_START_UNDERLIMIT,
  LOAD_SWEEP_START_OVERLIMIT,
  LOAD_SWEEP_START_INVALID_STATE,
} LoadSweepStartResult;

typedef struct {
  uint8_t  mode;          /**< LoadSweepMode. */
  uint16_t start_rpm;
  uint16_t end_rpm;
  uint16_t step_or_rate;  /**< Staircase: step in RPM. Ramp: slew rate in RPM/s. */
  uint16_t settle_ms;     /**< Wait after the reference reaches each step (ramp: before starting). */
  uint16_t window_ms;     /**< Averaging window per point. */
  uint8_t  options;       /**< LOAD_SWEEP_OPT_* bits. */
} LoadSweepConfig;

/** One averaged operating point; 16 bytes, little-endian on the wire. */
typedef struct __attribute__((packed)) {
  uint16_t reference_rpm;  /**< Mean slew-limited reference. */
  uint16_t rpm_mean;
  uint16_t duty_e4;        /**< Mean duty, 1/10000. */
  uint16_t bus_mv;
  int16_t  bus_ma_x10;     /**< Mean DC-bus current, 0.1 mA. */
  uint16_t duty_std_e4;    /**< Standard deviation of the duty, 1/10000. */
  uint16_t bus_std_ma_x10; /**< Standard deviation of the bus current, 0.1 mA. */
  uint8_t  rpm_std;        /**< Standard deviation of the speed, RPM (saturates at 255). */
  uint8_t  flags;          /**< LOAD_POINT_FLAG_* bits. */
} LoadSweepPoint;

typedef struct {
  uint8_t         state;        /**< LoadSweepState. */
  uint8_t         abort_reason; /**< LoadSweepAbortReason. */
  bool            idle_current_valid;
  LoadSweepConfig config;
  uint16_t        point_count;
  uint16_t        target_rpm;   /**< Current commanded target. */
  int32_t         idle_current_ua; /**< Bus current measured in IDLE with the power stage off. */
  uint32_t        elapsed_ms;   /**< Since the start request. */
} LoadSweepStatus;

LoadSweepStartResult load_sweep_start(const LoadSweepConfig* config);
void load_sweep_abort(LoadSweepAbortReason reason);
bool load_sweep_is_active(void);
void load_sweep_process(void);
void load_sweep_get_status(LoadSweepStatus* status);
bool load_sweep_get_point(uint16_t index, LoadSweepPoint* point);


#endif /* INC_LOAD_SWEEP_H_ */
