/*
 * telemetry.c - sample loop, command handling and UART DMA transport.
 *
 * Timing contract, per sample k (every TLM_TICK_HZ / rate ticks):
 *   1. t_k = tlm_micros(), read sensors            -> y_k
 *   2. pick u_k = latest MOTOR_CMD (or idle if disarmed / timed out), clamp
 *   3. tlm_port_write_motors(u_k)
 *   4. send telemetry frame {seq=k, t_k, y_k, u_k, cmd_seq}
 * So each CSV row is (t_k, y_k, u_k) with u_k acting on the plant after t_k.
 * A host controller that answers frame k lands its command at sample k+1
 * or later; cmd_seq in the telemetry tells you exactly which.
 */
#include "telemetry.h"
#include <string.h>

static UART_HandleTypeDef *s_uart;

static uint8_t  s_rx_buf[TLM_RX_BUF];
static uint32_t s_rx_tail;
static tlm_parser_t s_parser;

static uint8_t  s_tx_buf[2][TLM_MAX_FRAME];
static uint8_t  s_tx_idx;
static uint8_t  s_tx_overrun;

static uint16_t s_rate_hz = TLM_DEFAULT_RATE_HZ;
static uint16_t s_div     = TLM_TICK_HZ / TLM_DEFAULT_RATE_HZ;
static uint16_t s_div_cnt;

static volatile uint8_t s_armed;
static uint16_t s_cmd_us[4] = { TLM_US_MIN, TLM_US_MIN, TLM_US_MIN, TLM_US_MIN };
static uint16_t s_cmd_seq;
static uint8_t  s_cmd_clamped;
static uint8_t  s_cmd_fresh;        /* a command arrived since arming */
static uint32_t s_cmd_last_ms;
static uint32_t s_seq;

/* ---------------- time ---------------- */

/* DWT cycle counter: 72 MHz -> 1 us resolution, wraps every ~59.6 s, so we
 * extend it. Called at least every sample, far more often than the wrap. */
static uint32_t s_cyc_last, s_us_hi_acc, s_cyc_rem;

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_cyc_last = 0;
}

uint32_t tlm_micros(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t now = DWT->CYCCNT;
    uint32_t cycles = now - s_cyc_last + s_cyc_rem;  /* unsigned math handles the wrap */
    uint32_t cpu_mhz = SystemCoreClock / 1000000u;
    s_us_hi_acc += cycles / cpu_mhz;
    s_cyc_rem = cycles % cpu_mhz;
    s_cyc_last = now;
    if (!primask) __enable_irq();
    return s_us_hi_acc;
}

/* ---------------- TX ---------------- */

static void tx_frame(uint8_t type, const void *payload, uint8_t len)
{
    if (s_uart->gState != HAL_UART_STATE_READY) {   /* previous frame still going */
        s_tx_overrun = 1;
        return;
    }
    s_tx_idx ^= 1u;
    size_t n = tlm_encode(type, payload, len, s_tx_buf[s_tx_idx]);
    HAL_UART_Transmit_DMA(s_uart, s_tx_buf[s_tx_idx], (uint16_t)n);
}

static void send_hello(void)
{
    tlm_hello_t h = {
        .proto_version = TLM_PROTO_VERSION,
        .rate_hz = s_rate_hz,
        .us_min = TLM_US_MIN,
        .us_max = TLM_US_MAX,
    };
    tx_frame(TLM_T_HELLO, &h, sizeof h);
}

void tlm_text(const char *s)
{
    size_t n = strlen(s);
    if (n > TLM_MAX_PAYLOAD) n = TLM_MAX_PAYLOAD;
    tx_frame(TLM_T_TEXT, s, (uint8_t)n);
}

/* ---------------- commands ---------------- */

static uint16_t clamp_us(uint16_t v, uint8_t *clamped)
{
    if (v < TLM_US_MIN) { *clamped = 1; return TLM_US_MIN; }
    if (v > TLM_US_MAX) { *clamped = 1; return TLM_US_MAX; }
    return v;
}

void tlm_disarm(void)
{
    s_armed = 0;
    s_cmd_fresh = 0;
    for (int i = 0; i < 4; i++) s_cmd_us[i] = TLM_US_MIN;
}

uint8_t tlm_is_armed(void) { return s_armed; }

static void handle_frame(uint8_t type, const uint8_t *p, uint8_t len)
{
    switch (type) {
    case TLM_T_MOTOR_CMD:
        if (len == sizeof(tlm_motor_cmd_t)) {
            tlm_motor_cmd_t c;
            memcpy(&c, p, sizeof c);
            uint8_t clamped = 0;
            for (int i = 0; i < 4; i++) s_cmd_us[i] = clamp_us(c.motor_us[i], &clamped);
            s_cmd_seq = c.seq;
            s_cmd_clamped = clamped;
            s_cmd_last_ms = HAL_GetTick();
            s_cmd_fresh = 1;
        }
        break;
    case TLM_T_ARM:
        if (!tlm_port_can_arm()) {
            tlm_text("ARM refused: ESCs still arming, KEY1 fault, or killed");
            break;
        }
        /* Arm with motors at idle; nothing spins until a fresh MOTOR_CMD arrives. */
        for (int i = 0; i < 4; i++) s_cmd_us[i] = TLM_US_MIN;
        s_cmd_fresh = 0;
        s_armed = 1;
        break;
    case TLM_T_DISARM:
        tlm_disarm();
        break;
    case TLM_T_SET_RATE:
        if (len == 2) {
            uint16_t r = (uint16_t)(p[0] | (p[1] << 8));
            if (r > 0 && r <= TLM_TICK_HZ && (TLM_TICK_HZ % r) == 0) {
                s_rate_hz = r;
                s_div = (uint16_t)(TLM_TICK_HZ / r);
                s_div_cnt = 0;
            }
            send_hello();   /* host reads back the rate actually in effect */
        }
        break;
    case TLM_T_PING:
        send_hello();
        break;
    default:
        break;
    }
}

/* ---------------- public ---------------- */

void tlm_init(UART_HandleTypeDef *huart)
{
    s_uart = huart;
    dwt_init();
    tlm_parser_init(&s_parser);
    /* RX DMA channel must be configured as CIRCULAR in CubeMX. */
    HAL_UART_Receive_DMA(s_uart, s_rx_buf, TLM_RX_BUF);
    s_rx_tail = 0;
    tlm_disarm();
    tlm_port_write_motors(s_cmd_us);
    send_hello();
}

void tlm_poll(void)
{
    /* HAL aborts RX DMA on UART errors (overrun, framing). Restart it. */
    if (s_uart->RxState == HAL_UART_STATE_READY) {
        HAL_UART_Receive_DMA(s_uart, s_rx_buf, TLM_RX_BUF);
        s_rx_tail = 0;
    }
    uint32_t head = TLM_RX_BUF - __HAL_DMA_GET_COUNTER(s_uart->hdmarx);
    while (s_rx_tail != head) {
        if (tlm_parser_feed(&s_parser, s_rx_buf[s_rx_tail]))
            handle_frame(s_parser.type, s_parser.buf, s_parser.len);
        if (++s_rx_tail >= TLM_RX_BUF) s_rx_tail = 0;
    }
}

void tlm_tick(void)
{
    if (++s_div_cnt < s_div) return;
    s_div_cnt = 0;

    tlm_telemetry_t t;
    tlm_sensors_t s;
    memset(&s, 0, sizeof s);

    /* 1. sample */
    t.t_us = tlm_micros();
    tlm_port_read_sensors(&s);

    /* 2. choose command */
    uint8_t flags = 0;
    uint16_t out[4] = { TLM_US_MIN, TLM_US_MIN, TLM_US_MIN, TLM_US_MIN };
    if (s_armed) {
        flags |= TLM_F_ARMED;
        if (s_cmd_fresh && (HAL_GetTick() - s_cmd_last_ms) <= TLM_CMD_TIMEOUT_MS) {
            memcpy(out, s_cmd_us, sizeof out);
        } else if (s_cmd_fresh) {
            flags |= TLM_F_CMD_TIMEOUT;   /* stays armed; resumes on the next command */
        }
    }

    /* 3. apply */
    tlm_port_write_motors(out);

    /* 4. report */
    if (!s.enc_ok)     flags |= TLM_F_ENC_FAULT;
    if (!s.imu_ok)     flags |= TLM_F_IMU_FAULT;
    if (s_tx_overrun)  flags |= TLM_F_TX_OVERRUN;
    if (s_cmd_clamped) flags |= TLM_F_CLAMPED;
    if (tlm_port_killed()) flags |= TLM_F_KILLED;

    t.seq = s_seq++;
    memcpy(t.q, s.q, sizeof t.q);
    memcpy(t.dq, s.dq, sizeof t.dq);
    memcpy(t.acc, s.acc, sizeof t.acc);
    memcpy(t.gyro, s.gyro, sizeof t.gyro);
    memcpy(t.motor_us, out, sizeof t.motor_us);
    t.cmd_seq = s_cmd_seq;
    t.flags = flags;
    t.reserved = 0;

    s_tx_overrun = 0;
    tx_frame(TLM_T_TELEMETRY, &t, sizeof t);
}
