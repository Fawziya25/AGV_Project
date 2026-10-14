#include "radar_client.h"

#include "link_protocol.h"
#include "radar_uart_port.h"

#include <string.h>

/*
 * 实现约束
 * --------
 * - 除两个中断通知函数外，所有状态都只在RadarClient_Poll()上下文修改。
 * - UART中断只置位，不解析协议、不调用用户回调，也不继续发送下一帧。
 * - 普通命令共用一个transaction，避免多个ACK和最终事件相互串扰。
 * - STOP使用独立的urgent_tx，可丢弃排队中的普通命令并优先发出。
 * - 所有时间比较都允许32位毫秒计数自然回绕。
 */

/* 缓冲区大小和协议定时参数，单位均为字节或毫秒。 */
#define CLIENT_RX_DMA_SIZE 512U
#define CLIENT_TX_QUEUE_DEPTH 4U
#define CLIENT_HELLO_RETRY_MS 200U
#define CLIENT_HELLO_PROBE_MS 2000U
#define CLIENT_HEARTBEAT_MS 500U
#define CLIENT_ACK_TIMEOUT_MS 1500U
#define CLIENT_OPERATION_TIMEOUT_MS 8000U
#define CLIENT_LINK_TIMEOUT_MS 5000U
#define CLIENT_UART_RETRY_MS 100U
#define CLIENT_ZONE_RETRY_MS 500U
#define CLIENT_POINT_BITMAP_SIZE ((RADAR_CLIENT_MAX_DISPLAY_POINTS + 7U) / 8U)

typedef struct {
    uint16_t length;
    uint8_t data[LINK_MAX_ENCODED_FRAME];
} client_tx_frame_t;

/*
 * 一个命令事务可能由多种事件共同完成。例如：
 * - SCAN_ONCE：ACK + GRID + RESULT + 已锁定STATUS；
 * - CALIBRATE：ACK + CALIBRATION + 已写Flash的PERSISTED_STATE；
 * - SELECT_ZONE：ACK + 目标区域HELLO，随后还要读取一次保存状态。
 * seen标志允许这些事件以任意先后顺序到达。
 */
typedef struct {
    bool active;
    bool internal;
    bool ack_ok;
    bool hello_seen;
    bool persisted_seen;
    bool grid_seen;
    bool result_seen;
    bool locked_seen;
    uint8_t command;
    uint8_t requested_zone;
    uint8_t requested_node_id;
    uint8_t requested_value;
    uint16_t request_sequence;
    uint32_t deadline_ms;
    radar_client_operation_t operation;
} client_transaction_t;

/* RX DMA循环缓冲区；rx_read_position是软件已消费到的位置。 */
static uint8_t rx_dma[CLIENT_RX_DMA_SIZE];
static uint16_t rx_read_position;
static link_decoder_t decoder;

/* 普通发送队列和独立高优先级STOP槽。 */
static client_tx_frame_t tx_queue[CLIENT_TX_QUEUE_DEPTH];
static uint8_t tx_head;
static uint8_t tx_tail;
static client_tx_frame_t urgent_tx;
static bool urgent_pending;
static bool tx_busy;
static bool tx_active_urgent;
static volatile bool tx_complete_pending;
static volatile bool uart_error_pending;
static volatile uint32_t uart_error_code;

/* 节点、握手、心跳和超时状态。desired_zone是调用方明确要求的区域。 */
static uint16_t next_sequence;
static uint8_t current_node_id;
static uint8_t desired_zone;
static uint32_t next_hello_ms;
static uint32_t next_heartbeat_ms;
static uint32_t last_probe_ms;
static uint32_t last_valid_message_ms;
static uint32_t next_uart_retry_ms;
static uint32_t next_zone_retry_ms;
static bool startup_read_pending;
static client_transaction_t transaction;
static radar_client_callbacks_t callbacks;
static radar_client_state_t state;

/* GRID和RESULT是两条独立事件，按scan_sequence配对后才发布障碍结果。 */
static radar_client_grid_result_t pending_grid;
static bool pending_grid_valid;
static radar_client_obstacle_result_t pending_result;
static bool pending_result_valid;

/* 显示点云由BEGIN/多个POINTS/END重组，位图用于识别缺块和重复块。 */
static radar_client_point_cloud_t point_cloud;
static uint8_t point_received[CLIENT_POINT_BITMAP_SIZE];
static uint16_t point_expected;
static bool point_cloud_active;
static bool point_cloud_enabled;

static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t read_i16(const uint8_t *data)
{
    return (int16_t)read_u16(data);
}

static uint32_t read_u32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static void write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static void write_i16(uint8_t *data, int16_t value)
{
    write_u16(data, (uint16_t)value);
}

/* 使用有符号差值比较时间，HAL tick在uint32_t回绕后仍然有效。 */
static bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static bool zone_valid(uint8_t zone)
{
    return zone == RADAR_CLIENT_ZONE_1 || zone == RADAR_CLIENT_ZONE_2;
}

static bool pose_valid(const radar_client_pose_t *pose)
{
    if (pose == NULL || !zone_valid(pose->zone) || pose->heading_q6 >= 23040U)
        return false;
    if (pose->zone == RADAR_CLIENT_ZONE_1)
        return pose->x_mm >= -200 && pose->x_mm <= 500 &&
               pose->y_mm >= 1900 && pose->y_mm <= 2600;
    return pose->x_mm >= -200 && pose->x_mm <= 500 &&
           pose->y_mm >= -200 && pose->y_mm <= 500;
}

static void notify_state(void)
{
    /* busy/ready/operation是派生状态，每次回调前统一刷新。 */
    state.busy = transaction.active;
    state.ready = state.connected &&
                  state.link_state == RADAR_CLIENT_LINK_READY;
    state.operation = transaction.active ?
                      transaction.operation : RADAR_CLIENT_OPERATION_NONE;
    if (callbacks.state != NULL) callbacks.state(&state);
}

static void report_error(radar_client_error_kind_t kind,
                         radar_client_operation_t operation,
                         uint8_t code, uint16_t detail)
{
    radar_client_error_t error;
    error.kind = kind;
    error.operation = operation;
    error.code = code;
    error.detail = detail;
    if (callbacks.error != NULL) callbacks.error(&error);
}

static void reset_point_cloud(void)
{
    point_cloud_active = false;
    point_expected = 0U;
    memset(point_received, 0, sizeof(point_received));
}

static void clear_results(void)
{
    pending_grid_valid = false;
    pending_result_valid = false;
    reset_point_cloud();
}

static void clear_transaction(void)
{
    memset(&transaction, 0, sizeof(transaction));
}

static void drop_link(radar_client_error_kind_t reason, uint16_t detail)
{
    /*
     * 断链后不能继续使用旧ACK、旧扫描块或旧工作区域。保留目标节点ID和
     * desired_zone，使下一次HELLO后可以重新选择调用方明确指定的区域。
     */
    radar_client_operation_t operation = transaction.operation;
    RadarUartPort_AbortTransmit();
    tx_head = 0U;
    tx_tail = 0U;
    urgent_pending = false;
    tx_busy = false;
    tx_active_urgent = false;
    clear_transaction();
    clear_results();
    state.connected = false;
    state.ready = false;
    state.running = false;
    state.selected_zone = 0U;
    state.link_state = RADAR_CLIENT_LINK_WAIT_HELLO;
    state.operation = RADAR_CLIENT_OPERATION_NONE;
    state.frame_count = 0U;
    startup_read_pending = false;
    next_hello_ms = RadarUartPort_GetTick();
    last_valid_message_ms = RadarUartPort_GetTick();
    link_decoder_reset_partial(&decoder);
    if (reason != 0) report_error(reason, operation, 0U, detail);
    notify_state();
}

static bool queue_encoded(uint8_t node_id, uint8_t type,
                          const uint8_t *payload, uint16_t payload_length,
                          bool urgent, uint16_t *sequence_out)
{
    uint16_t sequence = next_sequence++;
    size_t length;
    client_tx_frame_t *frame;

    /* urgent槽只用于STOP，不受普通环形队列阻塞。 */
    if (urgent) {
        if (urgent_pending) return false;
        frame = &urgent_tx;
    } else {
        uint8_t next = (uint8_t)((tx_head + 1U) % CLIENT_TX_QUEUE_DEPTH);
        if (next == tx_tail) return false;
        frame = &tx_queue[tx_head];
        tx_head = next;
    }

    length = link_encode_message(node_id, type, sequence, payload,
                                 payload_length, frame->data,
                                 sizeof(frame->data));
    if (length == 0U) {
        if (!urgent) {
            tx_head = (uint8_t)((tx_head + CLIENT_TX_QUEUE_DEPTH - 1U) %
                                CLIENT_TX_QUEUE_DEPTH);
        }
        return false;
    }
    frame->length = (uint16_t)length;
    if (urgent) urgent_pending = true;
    if (sequence_out != NULL) *sequence_out = sequence;
    return true;
}

static void service_tx(void)
{
    client_tx_frame_t *frame;

    /* 中断只置tx_complete_pending；真正出队仍在主循环完成。 */
    if (tx_complete_pending) {
        tx_complete_pending = false;
        if (tx_busy) {
            if (tx_active_urgent) {
                urgent_pending = false;
            } else if (tx_tail != tx_head) {
                tx_tail = (uint8_t)((tx_tail + 1U) % CLIENT_TX_QUEUE_DEPTH);
            }
        }
        tx_busy = false;
        tx_active_urgent = false;
    }

    if (tx_busy) return;
    /* 每次启动新DMA前都优先检查STOP。 */
    if (urgent_pending) {
        frame = &urgent_tx;
        tx_active_urgent = true;
    } else if (tx_tail != tx_head) {
        frame = &tx_queue[tx_tail];
        tx_active_urgent = false;
    } else {
        return;
    }

    if (RadarUartPort_StartTransmit(frame->data, frame->length)) {
        tx_busy = true;
    } else {
        report_error(RADAR_CLIENT_ERROR_UART, transaction.operation, 0U, 1U);
        uart_error_pending = true;
    }
}

static bool start_transaction(uint8_t command, const uint8_t *payload,
                              uint16_t payload_length,
                              radar_client_operation_t operation,
                              bool internal)
{
    uint16_t sequence;
    uint32_t now_ms;

    /* 单事务设计：上一个业务没有完成时绝不复用sequence或完成标志。 */
    if (transaction.active) return false;
    if (!queue_encoded(current_node_id, command, payload, payload_length,
                       false, &sequence)) {
        report_error(RADAR_CLIENT_ERROR_QUEUE_FULL, operation, 0U, command);
        return false;
    }

    now_ms = RadarUartPort_GetTick();
    memset(&transaction, 0, sizeof(transaction));
    transaction.active = true;
    transaction.internal = internal;
    transaction.command = command;
    transaction.operation = operation;
    transaction.request_sequence = sequence;
    transaction.deadline_ms = now_ms + CLIENT_ACK_TIMEOUT_MS;
    notify_state();
    return true;
}

static void finish_transaction(bool success)
{
    bool internal = transaction.internal;
    uint8_t command = transaction.command;
    radar_client_operation_t operation = transaction.operation;

    clear_transaction();
    if (!success)
        report_error(RADAR_CLIENT_ERROR_PROTOCOL, operation, 0U, command);
    /* 启动阶段的内部状态读取结束后，才允许进入READY。 */
    if (internal && command == LINK_CMD_GET_PERSISTED_STATE) {
        if (zone_valid(state.selected_zone)) {
            state.link_state = RADAR_CLIENT_LINK_READY;
        } else {
            state.link_state = RADAR_CLIENT_LINK_NEED_ZONE;
        }
    }
    notify_state();
}

static void start_read_persisted(bool internal)
{
    if (start_transaction(LINK_CMD_GET_PERSISTED_STATE, NULL, 0U,
                          RADAR_CLIENT_OPERATION_READ_STATE, internal)) {
        state.link_state = RADAR_CLIENT_LINK_READING_STATE;
        startup_read_pending = false;
    }
}

static void begin_zone_selection(uint8_t zone, bool internal)
{
    uint8_t payload = zone;
    if (!start_transaction(LINK_CMD_SELECT_ZONE, &payload, 1U,
                           RADAR_CLIENT_OPERATION_SELECT_ZONE, internal))
        return;
    transaction.requested_zone = zone;
    next_zone_retry_ms = RadarUartPort_GetTick() + CLIENT_ZONE_RETRY_MS;
    /* 切换确认完成前主动清零，防止业务误用旧区域位姿。 */
    state.selected_zone = 0U;
    state.link_state = RADAR_CLIENT_LINK_WAIT_ZONE_CONFIRM;
}

static bool all_points_received(uint16_t count)
{
    uint16_t index;
    for (index = 0U; index < count; ++index) {
        if ((point_received[index >> 3] & (uint8_t)(1U << (index & 7U))) == 0U)
            return false;
    }
    return true;
}

static void maybe_publish_obstacles(void)
{
    uint8_t index;

    /* 固件分别发送方格和坐标；只有序号和数量都一致才对外发布。 */
    if (!pending_grid_valid || !pending_result_valid ||
        pending_grid.scan_sequence != pending_result.scan_sequence)
        return;
    if (pending_grid.count != pending_result.obstacle_count) {
        report_error(RADAR_CLIENT_ERROR_BAD_EVENT, transaction.operation,
                     LINK_EVT_RESULT, pending_result.obstacle_count);
        pending_grid_valid = false;
        pending_result_valid = false;
        return;
    }

    for (index = 0U; index < pending_result.obstacle_count; ++index)
        pending_result.obstacles[index].grid_id = pending_grid.grid_ids[index];
    if (callbacks.obstacles != NULL) callbacks.obstacles(&pending_result);
    if (transaction.active &&
        transaction.operation == RADAR_CLIENT_OPERATION_SCAN_ONCE) {
        transaction.result_seen = true;
        transaction.grid_seen = true;
        if (transaction.ack_ok && transaction.locked_seen)
            finish_transaction(true);
    }
    pending_grid_valid = false;
    pending_result_valid = false;
}

static bool parse_hello(const link_message_t *message,
                        radar_client_hello_t *hello)
{
    const uint8_t *p = message->payload;
    if (message->payload_length != 13U) return false;
    hello->firmware_major = p[0];
    hello->firmware_minor = p[1];
    hello->protocol = p[2];
    hello->pose.zone = p[3];
    hello->pose.x_mm = read_i16(&p[4]);
    hello->pose.y_mm = read_i16(&p[6]);
    hello->pose.heading_q6 = read_u16(&p[8]);
    hello->running = p[10] != 0U;
    hello->max_scan_points = read_u16(&p[11]);
    return hello->protocol == LINK_PROTOCOL_VERSION &&
           (hello->pose.zone == 0U || zone_valid(hello->pose.zone));
}

static void handle_hello(const link_message_t *message)
{
    radar_client_hello_t hello;
    bool was_connected = state.connected;

    if (!parse_hello(message, &hello)) {
        report_error(RADAR_CLIENT_ERROR_PROTOCOL, transaction.operation,
                     LINK_EVT_HELLO, message->payload_length);
        return;
    }

    /* SET_NODE_ID成功后第一帧HELLO来自新ID，是唯一允许的跨ID事件。 */
    if (transaction.active &&
        transaction.operation == RADAR_CLIENT_OPERATION_SET_NODE_ID &&
        message->node_id == transaction.requested_node_id) {
        current_node_id = transaction.requested_node_id;
        state.node_id = current_node_id;
        state.connected = true;
        state.selected_zone = hello.pose.zone;
        transaction.hello_seen = true;
        if (transaction.ack_ok) finish_transaction(true);
    } else if (message->node_id != current_node_id) {
        return;
    } else {
        state.connected = true;
        state.running = hello.running;
        if (transaction.active &&
            (transaction.operation == RADAR_CLIENT_OPERATION_SAVE_POSE ||
             transaction.operation == RADAR_CLIENT_OPERATION_RESET_CALIBRATION) &&
            hello.pose.zone == transaction.requested_zone) {
            transaction.hello_seen = true;
            if (transaction.ack_ok && transaction.persisted_seen)
                finish_transaction(true);
        }
        /* HELLO中的zone是本次上电会话区域，source_zone不能替代它。 */
        if (zone_valid(hello.pose.zone)) {
            state.selected_zone = hello.pose.zone;
            if (transaction.active &&
                transaction.operation == RADAR_CLIENT_OPERATION_SELECT_ZONE &&
                hello.pose.zone == transaction.requested_zone) {
                transaction.hello_seen = true;
                if (transaction.ack_ok) {
                    clear_transaction();
                    startup_read_pending = true;
                    state.link_state = RADAR_CLIENT_LINK_READING_STATE;
                    notify_state();
                }
            } else if (zone_valid(desired_zone) &&
                       hello.pose.zone != desired_zone &&
                       !transaction.active) {
                /* 调用方已明确要求另一区域，握手后再执行真正的切换。 */
                startup_read_pending = false;
                state.link_state = RADAR_CLIENT_LINK_NEED_ZONE;
                next_zone_retry_ms = RadarUartPort_GetTick();
            } else if (!was_connected ||
                       state.link_state == RADAR_CLIENT_LINK_WAIT_HELLO ||
                       state.link_state == RADAR_CLIENT_LINK_NEED_ZONE) {
                startup_read_pending = true;
                state.link_state = RADAR_CLIENT_LINK_READING_STATE;
            }
        } else {
            state.selected_zone = 0U;
            if (!transaction.active ||
                transaction.operation != RADAR_CLIENT_OPERATION_SELECT_ZONE)
                state.link_state = RADAR_CLIENT_LINK_NEED_ZONE;
        }
    }

    if (callbacks.hello != NULL) callbacks.hello(&hello);
    notify_state();
}

static void handle_ack(const link_message_t *message)
{
    uint8_t command;
    uint8_t result;
    if (message->payload_length != 2U) {
        report_error(RADAR_CLIENT_ERROR_BAD_EVENT, transaction.operation,
                     LINK_EVT_ACK, message->payload_length);
        return;
    }
    command = message->payload[0];
    result = message->payload[1];

    if (!transaction.active || command != transaction.command) return;
    if (result != 0U) {
        radar_client_operation_t operation = transaction.operation;
        bool select_failed = operation == RADAR_CLIENT_OPERATION_SELECT_ZONE;
        clear_transaction();
        if (select_failed) {
            state.selected_zone = 0U;
            state.link_state = RADAR_CLIENT_LINK_NEED_ZONE;
            next_zone_retry_ms = RadarUartPort_GetTick() +
                                 CLIENT_ZONE_RETRY_MS;
        }
        report_error(RADAR_CLIENT_ERROR_DEVICE, operation, result, command);
        notify_state();
        return;
    }

    /*
     * ACK仅表示命令被接受。扫描、校准、路径、选区和Flash类操作必须继续
     * 等待各自的最终事件；下面的switch集中定义每类事务的完成条件。
     */
    transaction.ack_ok = true;
    transaction.deadline_ms = RadarUartPort_GetTick() +
                              CLIENT_OPERATION_TIMEOUT_MS;

    switch (transaction.operation) {
    case RADAR_CLIENT_OPERATION_SELECT_ZONE:
        if (transaction.hello_seen) {
            clear_transaction();
            startup_read_pending = true;
            state.link_state = RADAR_CLIENT_LINK_READING_STATE;
            notify_state();
        }
        break;
    case RADAR_CLIENT_OPERATION_READ_STATE:
        if (transaction.persisted_seen) finish_transaction(true);
        break;
    case RADAR_CLIENT_OPERATION_SCAN_ONCE:
        if (transaction.grid_seen && transaction.result_seen &&
            transaction.locked_seen) finish_transaction(true);
        break;
    case RADAR_CLIENT_OPERATION_GRID_SCAN:
        if (transaction.grid_seen && transaction.locked_seen)
            finish_transaction(true);
        break;
    case RADAR_CLIENT_OPERATION_CALIBRATE:
        if (transaction.result_seen && transaction.persisted_seen)
            finish_transaction(true);
        break;
    case RADAR_CLIENT_OPERATION_PLAN_PATH:
        if (transaction.result_seen) finish_transaction(true);
        break;
    case RADAR_CLIENT_OPERATION_SAVE_POSE:
    case RADAR_CLIENT_OPERATION_RESET_CALIBRATION:
        if (transaction.hello_seen && transaction.persisted_seen)
            finish_transaction(true);
        break;
    case RADAR_CLIENT_OPERATION_SET_NODE_ID:
        break;
    case RADAR_CLIENT_OPERATION_SET_POINT_CLOUD:
        point_cloud_enabled = transaction.requested_value != 0U;
        if (!point_cloud_enabled) reset_point_cloud();
        finish_transaction(true);
        break;
    default:
        finish_transaction(true);
        break;
    }
}

static void handle_status(const link_message_t *message)
{
    const uint8_t *p = message->payload;
    if (message->payload_length != 5U || p[2] > 1U || p[4] > 1U) {
        report_error(RADAR_CLIENT_ERROR_BAD_EVENT, transaction.operation,
                     LINK_EVT_STATUS, message->payload_length);
        return;
    }
    state.status = p[0];
    state.mode = p[1];
    state.running = p[2] != 0U;
    state.frame_count = p[3];
    state.overflow = p[4] != 0U;

    if (transaction.active && p[0] == 4U) {
        transaction.locked_seen = true;
        if (transaction.operation == RADAR_CLIENT_OPERATION_SCAN_ONCE &&
            transaction.ack_ok && transaction.grid_seen &&
            transaction.result_seen)
            finish_transaction(true);
        else if (transaction.operation == RADAR_CLIENT_OPERATION_GRID_SCAN &&
                 transaction.ack_ok && transaction.grid_seen)
            finish_transaction(true);
    }
    notify_state();
}

static void handle_grid(const link_message_t *message)
{
    radar_client_grid_result_t result;
    uint8_t index;
    uint8_t count;
    const uint8_t *p = message->payload;

    if (message->payload_length < 5U) goto bad;
    count = p[4];
    if (count > RADAR_CLIENT_MAX_GRID_IDS ||
        message->payload_length != (uint16_t)(5U + count))
        goto bad;
    memset(&result, 0, sizeof(result));
    result.scan_sequence = read_u32(p);
    result.count = count;
    for (index = 0U; index < count; ++index) {
        if (p[5U + index] > 24U &&
            p[5U + index] != RADAR_CLIENT_GRID_OUTSIDE) goto bad;
        if (transaction.active &&
            transaction.operation == RADAR_CLIENT_OPERATION_GRID_SCAN &&
            p[5U + index] == RADAR_CLIENT_GRID_OUTSIDE) goto bad;
        result.grid_ids[index] = p[5U + index];
    }

    /* GRID_SCAN直接回调；SCAN_ONCE还会等待同序号RESULT进行组合。 */
    pending_grid = result;
    pending_grid_valid = true;
    if (callbacks.grid != NULL) callbacks.grid(&result);
    if (transaction.active &&
        transaction.operation == RADAR_CLIENT_OPERATION_GRID_SCAN) {
        transaction.grid_seen = true;
        if (transaction.ack_ok && transaction.locked_seen)
            finish_transaction(true);
    }
    maybe_publish_obstacles();
    return;

bad:
    report_error(RADAR_CLIENT_ERROR_BAD_EVENT, transaction.operation,
                 LINK_EVT_OBSTACLE_GRID, message->payload_length);
}

static void handle_result(const link_message_t *message)
{
    radar_client_obstacle_result_t result;
    uint8_t count;
    uint8_t index;
    const uint8_t *p = message->payload;

    if (message->payload_length < 8U) goto bad;
    count = p[7];
    if (count > RADAR_CLIENT_MAX_OBSTACLES ||
        message->payload_length != (uint16_t)(8U + (uint16_t)count * 8U))
        goto bad;
    memset(&result, 0, sizeof(result));
    result.scan_sequence = read_u32(p);
    result.frame_count = p[4];
    result.locked = p[5] != 0U;
    result.overflow = p[6] != 0U;
    result.obstacle_count = count;
    for (index = 0U; index < count; ++index) {
        const uint8_t *item = &p[8U + (uint16_t)index * 8U];
        result.obstacles[index].x_mm = read_i16(item);
        result.obstacles[index].y_mm = read_i16(&item[2]);
        result.obstacles[index].frame_hits = item[4];
        result.obstacles[index].confidence_percent = item[5];
        result.obstacles[index].point_count = read_u16(&item[6]);
    }
    /* grid_id不在RESULT载荷中，必须等待同序号GRID后再调用obstacles。 */
    pending_result = result;
    pending_result_valid = true;
    maybe_publish_obstacles();
    return;

bad:
    report_error(RADAR_CLIENT_ERROR_BAD_EVENT, transaction.operation,
                 LINK_EVT_RESULT, message->payload_length);
}

static void handle_scan_begin(const link_message_t *message)
{
    const uint8_t *p = message->payload;
    uint16_t transmitted;
    /* 点云默认关闭；关闭时直接忽略高带宽显示事件。 */
    if (!point_cloud_enabled) return;
    if (message->payload_length != 14U) goto bad;
    transmitted = read_u16(&p[10]);
    if (transmitted > RADAR_CLIENT_MAX_DISPLAY_POINTS) goto bad;
    memset(&point_cloud, 0, sizeof(point_cloud));
    memset(point_received, 0, sizeof(point_received));
    point_cloud.scan_sequence = read_u32(p);
    point_cloud.timestamp_ms = read_u32(&p[4]);
    point_cloud.frequency_deci_hz = read_u16(&p[8]);
    point_cloud.point_count = transmitted;
    point_cloud.source_point_count = read_u16(&p[12]);
    point_expected = transmitted;
    point_cloud_active = true;
    return;
bad:
    reset_point_cloud();
    report_error(RADAR_CLIENT_ERROR_BAD_EVENT, transaction.operation,
                 LINK_EVT_SCAN_BEGIN, message->payload_length);
}

static void handle_scan_points(const link_message_t *message)
{
    const uint8_t *p = message->payload;
    uint32_t sequence;
    uint16_t offset;
    uint8_t count;
    uint8_t index;
    if (!point_cloud_enabled) return;
    if (message->payload_length < 7U) goto bad;
    sequence = read_u32(p);
    offset = read_u16(&p[4]);
    count = p[6];
    if (!point_cloud_active || sequence != point_cloud.scan_sequence ||
        (uint32_t)offset + count > point_expected ||
        message->payload_length != (uint16_t)(7U + (uint16_t)count * 5U))
        goto bad;
    for (index = 0U; index < count; ++index) {
        uint16_t target = (uint16_t)(offset + index);
        const uint8_t *item = &p[7U + (uint16_t)index * 5U];
        radar_client_point_t value;
        value.x_mm = read_i16(item);
        value.y_mm = read_i16(&item[2]);
        value.intensity = item[4];
        if ((point_received[target >> 3] &
             (uint8_t)(1U << (target & 7U))) != 0U) {
            if (point_cloud.points[target].x_mm != value.x_mm ||
                point_cloud.points[target].y_mm != value.y_mm ||
                point_cloud.points[target].intensity != value.intensity)
                goto bad;
        } else {
            point_cloud.points[target] = value;
            point_received[target >> 3] |=
                (uint8_t)(1U << (target & 7U));
        }
    }
    return;
bad:
    reset_point_cloud();
    report_error(RADAR_CLIENT_ERROR_BAD_EVENT, transaction.operation,
                 LINK_EVT_SCAN_POINTS, message->payload_length);
}

static void handle_scan_end(const link_message_t *message)
{
    const uint8_t *p = message->payload;
    if (!point_cloud_enabled) return;
    if (message->payload_length != 9U || !point_cloud_active ||
        read_u32(p) != point_cloud.scan_sequence ||
        read_u16(&p[4]) != point_expected || p[6] != 0U ||
        !all_points_received(point_expected)) {
        reset_point_cloud();
        report_error(RADAR_CLIENT_ERROR_BAD_EVENT, transaction.operation,
                     LINK_EVT_SCAN_END, message->payload_length);
        return;
    }
    point_cloud.nearest_mm = read_u16(&p[7]);
    if (callbacks.point_cloud != NULL) callbacks.point_cloud(&point_cloud);
    reset_point_cloud();
}

static void handle_stats(const link_message_t *message)
{
    radar_client_stats_t stats;
    const uint8_t *p = message->payload;
    if (!point_cloud_enabled) return;
    if (message->payload_length != 25U) {
        report_error(RADAR_CLIENT_ERROR_BAD_EVENT, transaction.operation,
                     LINK_EVT_STATS, message->payload_length);
        return;
    }
    stats.scan_sequence = read_u32(p);
    stats.reported_frequency_deci_hz = read_u16(&p[4]);
    stats.measured_frequency_deci_hz = read_u16(&p[6]);
    stats.point_count = read_u16(&p[8]);
    stats.good_packets = read_u32(&p[10]);
    stats.bad_checksums = read_u32(&p[14]);
    stats.discarded_bytes = read_u32(&p[18]);
    stats.processing_ms = read_u16(&p[22]);
    stats.flags = p[24];
    if (callbacks.stats != NULL) callbacks.stats(&stats);
}

static void handle_calibration(const link_message_t *message)
{
    radar_client_calibration_t result;
    const uint8_t *p = message->payload;
    if (message->payload_length != 15U) goto bad;
    result.pose.zone = p[0];
    result.pose.x_mm = read_i16(&p[1]);
    result.pose.y_mm = read_i16(&p[3]);
    result.pose.heading_q6 = read_u16(&p[5]);
    result.x_offset_mm = read_i16(&p[7]);
    result.y_offset_mm = read_i16(&p[9]);
    result.heading_trim_q6 = read_i16(&p[11]);
    result.residual_mm = p[13];
    result.pair_distance_error_mm = p[14];
    if (!zone_valid(result.pose.zone)) goto bad;
    if (transaction.active &&
        transaction.operation == RADAR_CLIENT_OPERATION_CALIBRATE &&
        result.pose.zone != transaction.requested_zone)
        goto bad;
    if (callbacks.calibration != NULL) callbacks.calibration(&result);
    if (transaction.active &&
        transaction.operation == RADAR_CLIENT_OPERATION_CALIBRATE) {
        transaction.result_seen = true;
        if (transaction.ack_ok && transaction.persisted_seen)
            finish_transaction(true);
    }
    return;
bad:
    report_error(RADAR_CLIENT_ERROR_BAD_EVENT, transaction.operation,
                 LINK_EVT_CALIBRATION, message->payload_length);
}

static void handle_persisted(const link_message_t *message)
{
    radar_client_persisted_state_t result;
    const uint8_t *p = message->payload;
    uint8_t index;

    if (message->payload_length < 33U) goto bad;
    memset(&result, 0, sizeof(result));
    result.storage_version = read_u16(p);
    result.record_sequence = read_u32(&p[2]);
    result.source_zone = p[6];
    result.grid_result_valid = p[9] != 0U;
    result.grid_count = p[10];
    if (result.grid_count > RADAR_CLIENT_MAX_GRID_IDS ||
        message->payload_length != (uint16_t)(33U + result.grid_count))
        goto bad;
    result.zone1_pose.x_mm = read_i16(&p[11]);
    result.zone1_pose.y_mm = read_i16(&p[13]);
    result.zone1_pose.heading_q6 = read_u16(&p[15]);
    result.zone1_pose.zone = RADAR_CLIENT_ZONE_1;
    result.zone2_pose.x_mm = read_i16(&p[18]);
    result.zone2_pose.y_mm = read_i16(&p[20]);
    result.zone2_pose.heading_q6 = read_u16(&p[22]);
    result.zone2_pose.zone = RADAR_CLIENT_ZONE_2;
    for (index = 0U; index < result.grid_count; ++index) {
        if (p[33U + index] > 24U) goto bad;
        result.grid_ids[index] = p[33U + index];
    }
    /*
     * 先把完整保存状态交给调用方，再用source_zone确认校准/保存/重置事务。
     * source_zone只表示最近修改的位姿槽，绝不能当作当前工作区域。
     */
    if (callbacks.persisted != NULL) callbacks.persisted(&result);
    if (transaction.active) {
        if (transaction.operation == RADAR_CLIENT_OPERATION_READ_STATE) {
            transaction.persisted_seen = true;
            if (transaction.ack_ok) finish_transaction(true);
        } else if ((transaction.operation == RADAR_CLIENT_OPERATION_SAVE_POSE ||
                    transaction.operation == RADAR_CLIENT_OPERATION_RESET_CALIBRATION) &&
                   result.source_zone == transaction.requested_zone) {
            transaction.persisted_seen = true;
            if (transaction.ack_ok && transaction.hello_seen)
                finish_transaction(true);
        } else if (transaction.operation == RADAR_CLIENT_OPERATION_CALIBRATE &&
                   result.source_zone == transaction.requested_zone) {
            transaction.persisted_seen = true;
            if (transaction.ack_ok && transaction.result_seen)
                finish_transaction(true);
        }
    }
    return;
bad:
    report_error(RADAR_CLIENT_ERROR_BAD_EVENT, transaction.operation,
                 LINK_EVT_PERSISTED_STATE, message->payload_length);
}

static void handle_path(const link_message_t *message)
{
    radar_client_path_result_t result;
    const uint8_t *p = message->payload;
    uint8_t index;
    if (message->payload_length < 6U) goto bad;
    memset(&result, 0, sizeof(result));
    result.request_sequence = read_u16(p);
    result.status = p[2];
    result.start = p[3];
    result.goal = p[4];
    result.waypoint_count = p[5];
    if (result.waypoint_count > RADAR_CLIENT_MAX_WAYPOINTS ||
        message->payload_length != (uint16_t)(6U + result.waypoint_count))
        goto bad;
    for (index = 0U; index < result.waypoint_count; ++index) {
        if (p[6U + index] > 24U) goto bad;
        result.waypoints[index] = p[6U + index];
    }
    if (callbacks.path != NULL) callbacks.path(&result);
    if (transaction.active &&
        transaction.operation == RADAR_CLIENT_OPERATION_PLAN_PATH &&
        result.request_sequence == transaction.request_sequence) {
        transaction.result_seen = true;
        if (transaction.ack_ok) finish_transaction(true);
    }
    return;
bad:
    report_error(RADAR_CLIENT_ERROR_BAD_EVENT, transaction.operation,
                 LINK_EVT_PATH_RESULT, message->payload_length);
}

static void handle_error_event(const link_message_t *message)
{
    uint8_t code;
    uint16_t detail;
    radar_client_operation_t operation = transaction.operation;
    if (message->payload_length != 3U) {
        report_error(RADAR_CLIENT_ERROR_BAD_EVENT, operation,
                     LINK_EVT_ERROR, message->payload_length);
        return;
    }
    code = message->payload[0];
    detail = read_u16(&message->payload[1]);
    report_error(RADAR_CLIENT_ERROR_DEVICE, operation, code, detail);
    if (code == 10U) {
        clear_transaction();
        state.selected_zone = 0U;
        state.link_state = RADAR_CLIENT_LINK_NEED_ZONE;
        next_zone_retry_ms = RadarUartPort_GetTick() + CLIENT_ZONE_RETRY_MS;
        notify_state();
    } else if (transaction.active && transaction.operation != RADAR_CLIENT_OPERATION_CONNECT) {
        clear_transaction();
        notify_state();
    }
}

/* ======================== 协议解码回调函数 ======================== */

/*
 * link_decoder_feed()每解析出一帧完整且CRC正确的消息就调用本函数。
 * 这里先过滤非目标节点，再按事件类型交给对应解析函数；用户注册的
 * 业务回调最终仍由这些handle函数在RadarClient_Poll()上下文中触发。
 */
static void on_message(const link_message_t *message, void *context)
{
    bool accepted_new_node = transaction.active &&
                             transaction.operation ==
                                 RADAR_CLIENT_OPERATION_SET_NODE_ID &&
                             message->node_id == transaction.requested_node_id &&
                             message->type == LINK_EVT_HELLO;
    (void)context;

    /* 总线上其他节点的合法帧也必须静默忽略，且不能刷新本节点5秒看门狗。 */
    if (message->node_id != current_node_id && !accepted_new_node) return;
    last_valid_message_ms = RadarUartPort_GetTick();

    switch (message->type) {
    case LINK_EVT_HELLO: handle_hello(message); break;
    case LINK_EVT_ACK: handle_ack(message); break;
    case LINK_EVT_STATUS: handle_status(message); break;
    case LINK_EVT_SCAN_BEGIN: handle_scan_begin(message); break;
    case LINK_EVT_SCAN_POINTS: handle_scan_points(message); break;
    case LINK_EVT_SCAN_END: handle_scan_end(message); break;
    case LINK_EVT_RESULT: handle_result(message); break;
    case LINK_EVT_CALIBRATION: handle_calibration(message); break;
    case LINK_EVT_STATS: handle_stats(message); break;
    case LINK_EVT_ERROR: handle_error_event(message); break;
    case LINK_EVT_OBSTACLE_GRID: handle_grid(message); break;
    case LINK_EVT_PERSISTED_STATE: handle_persisted(message); break;
    case LINK_EVT_PATH_RESULT: handle_path(message); break;
    default:
        /* v6 reserves administrator and future event numbers. */
        break;
    }
}

static void service_rx(void)
{
    /*
     * DMA写指针可能位于软件读指针前方，也可能已经绕回缓冲区开头。
     * 两段分别送入流式解码器，不复制、不假设一帧恰好一次到达。
     */
    uint16_t write_position = RadarUartPort_GetReceivePosition(CLIENT_RX_DMA_SIZE);
    if (write_position == rx_read_position) return;
    if (write_position > rx_read_position) {
        link_decoder_feed(&decoder, &rx_dma[rx_read_position],
                          write_position - rx_read_position,
                          on_message, NULL);
    } else {
        link_decoder_feed(&decoder, &rx_dma[rx_read_position],
                          CLIENT_RX_DMA_SIZE - rx_read_position,
                          on_message, NULL);
        if (write_position > 0U)
            link_decoder_feed(&decoder, rx_dma, write_position,
                              on_message, NULL);
    }
    rx_read_position = write_position;
}

static void service_uart_recovery(uint32_t now_ms)
{
    if (!uart_error_pending && RadarUartPort_ReceiveIsActive()) return;
    if (!time_reached(now_ms, next_uart_retry_ms)) return;

    /* 100 ms限速恢复，避免硬件故障时在主循环中无间隔反复初始化DMA。 */
    uart_error_pending = false;
    next_uart_retry_ms = now_ms + CLIENT_UART_RETRY_MS;
    rx_read_position = 0U;
    link_decoder_reset_partial(&decoder);
    if (RadarUartPort_StartReceive(rx_dma, sizeof(rx_dma))) {
        drop_link(RADAR_CLIENT_ERROR_UART, (uint16_t)uart_error_code);
        uart_error_code = 0U;
    }
}

static void service_connection(uint32_t now_ms)
{
    /* 未连接时快速HELLO；连接后维持心跳并做低频HELLO健康探测。 */
    if (!state.connected) {
        if (time_reached(now_ms, next_hello_ms)) {
            if (queue_encoded(current_node_id, LINK_CMD_HELLO, NULL, 0U,
                              false, NULL))
                next_hello_ms = now_ms + CLIENT_HELLO_RETRY_MS;
        }
        return;
    }

    if (time_reached(now_ms, next_heartbeat_ms)) {
        (void)queue_encoded(current_node_id, LINK_CMD_HEARTBEAT, NULL, 0U,
                            false, NULL);
        next_heartbeat_ms = now_ms + CLIENT_HEARTBEAT_MS;
    }
    if (time_reached(now_ms, last_probe_ms + CLIENT_HELLO_PROBE_MS)) {
        (void)queue_encoded(current_node_id, LINK_CMD_HELLO, NULL, 0U,
                            false, NULL);
        last_probe_ms = now_ms;
    }
    if (time_reached(now_ms, last_valid_message_ms + CLIENT_LINK_TIMEOUT_MS)) {
        drop_link(RADAR_CLIENT_ERROR_LINK_LOST, 0U);
        return;
    }

    /* 选区完成后自动读取Flash；若调用方预先选区，则握手后自动执行。 */
    if (startup_read_pending && !transaction.active)
        start_read_persisted(true);
    else if (state.link_state == RADAR_CLIENT_LINK_NEED_ZONE &&
             zone_valid(desired_zone) && !transaction.active &&
             time_reached(now_ms, next_zone_retry_ms))
        begin_zone_selection(desired_zone, true);
}

static void service_transaction_timeout(uint32_t now_ms)
{
    radar_client_operation_t operation;
    uint8_t command;
    if (!transaction.active || !time_reached(now_ms, transaction.deadline_ms))
        return;
    operation = transaction.operation;
    command = transaction.command;
    /* 超时只终止本事务；选区超时会回到NEED_ZONE等待重试。 */
    clear_transaction();
    if (operation == RADAR_CLIENT_OPERATION_SELECT_ZONE) {
        state.selected_zone = 0U;
        state.link_state = RADAR_CLIENT_LINK_NEED_ZONE;
        next_zone_retry_ms = now_ms + CLIENT_ZONE_RETRY_MS;
    } else if (operation == RADAR_CLIENT_OPERATION_READ_STATE &&
               !zone_valid(state.selected_zone)) {
        state.link_state = RADAR_CLIENT_LINK_NEED_ZONE;
    }
    report_error(RADAR_CLIENT_ERROR_TIMEOUT, operation, 0U, command);
    notify_state();
}

void RadarClient_Init(const radar_client_callbacks_t *new_callbacks,
                      uint8_t node_id)
{
    uint32_t now_ms = RadarUartPort_GetTick();

    /* 再次调用Init等价于重新建立一个干净客户端，不继承旧事务和结果。 */
    memset(&callbacks, 0, sizeof(callbacks));
    if (new_callbacks != NULL) callbacks = *new_callbacks;
    memset(&state, 0, sizeof(state));
    memset(&transaction, 0, sizeof(transaction));
    link_decoder_init(&decoder);
    clear_results();

    current_node_id = (node_id >= 1U && node_id <= LINK_MAX_NODE_ID) ?
                      node_id : LINK_DEFAULT_NODE_ID;
    desired_zone = 0U;
    state.node_id = current_node_id;
    state.link_state = RADAR_CLIENT_LINK_WAIT_HELLO;
    tx_head = 0U;
    tx_tail = 0U;
    urgent_pending = false;
    tx_busy = false;
    tx_complete_pending = false;
    uart_error_pending = false;
    uart_error_code = 0U;
    next_sequence = 0U;
    rx_read_position = 0U;
    next_hello_ms = now_ms;
    next_heartbeat_ms = now_ms + CLIENT_HEARTBEAT_MS;
    last_probe_ms = now_ms;
    last_valid_message_ms = now_ms;
    next_uart_retry_ms = now_ms;
    next_zone_retry_ms = now_ms;
    startup_read_pending = false;
    point_cloud_enabled = false;

    if (!RadarUartPort_StartReceive(rx_dma, sizeof(rx_dma))) {
        uart_error_pending = true;
        report_error(RADAR_CLIENT_ERROR_UART,
                     RADAR_CLIENT_OPERATION_CONNECT, 0U, 0U);
    }
    notify_state();
}

void RadarClient_Poll(void)
{
    /* 顺序不可随意调整：先恢复/收包，再判超时和建链，最后启动发送。 */
    uint32_t now_ms = RadarUartPort_GetTick();
    service_uart_recovery(now_ms);
    if (RadarUartPort_ReceiveIsActive()) service_rx();
    service_transaction_timeout(now_ms);
    service_connection(now_ms);
    service_tx();
}

bool RadarClient_GetState(radar_client_state_t *out)
{
    uint32_t key;
    if (out == NULL) return false;
    key = RadarUartPort_EnterCritical();
    *out = state;
    RadarUartPort_ExitCritical(key);
    return true;
}

bool RadarClient_IsReady(void)
{
    return state.connected && state.link_state == RADAR_CLIENT_LINK_READY;
}

bool RadarClient_IsBusy(void)
{
    return transaction.active;
}

radar_client_request_result_t RadarClient_RequestHello(void)
{
    if (!queue_encoded(current_node_id, LINK_CMD_HELLO, NULL, 0U,
                       false, NULL))
        return RADAR_CLIENT_REQUEST_QUEUE_FULL;
    return RADAR_CLIENT_REQUEST_ACCEPTED;
}

radar_client_request_result_t RadarClient_RequestReadPersistedState(void)
{
    if (!state.connected) return RADAR_CLIENT_REQUEST_NOT_READY;
    if (transaction.active) return RADAR_CLIENT_REQUEST_BUSY;
    if (!start_transaction(LINK_CMD_GET_PERSISTED_STATE, NULL, 0U,
                           RADAR_CLIENT_OPERATION_READ_STATE, false))
        return RADAR_CLIENT_REQUEST_QUEUE_FULL;
    return RADAR_CLIENT_REQUEST_ACCEPTED;
}

radar_client_request_result_t RadarClient_RequestSelectZone(uint8_t zone)
{
    if (!zone_valid(zone)) return RADAR_CLIENT_REQUEST_BAD_PARAMETER;
    if (transaction.active) return RADAR_CLIENT_REQUEST_BUSY;
    desired_zone = zone;
    if (!state.connected) return RADAR_CLIENT_REQUEST_ACCEPTED;
    begin_zone_selection(zone, false);
    return transaction.active ? RADAR_CLIENT_REQUEST_ACCEPTED :
                                RADAR_CLIENT_REQUEST_QUEUE_FULL;
}

static radar_client_request_result_t request_ready_command(
    uint8_t command, const uint8_t *payload, uint16_t payload_length,
    radar_client_operation_t operation)
{
    /* 所有普通业务统一执行READY、单事务和队列容量检查。 */
    if (!RadarClient_IsReady()) return RADAR_CLIENT_REQUEST_NOT_READY;
    if (transaction.active) return RADAR_CLIENT_REQUEST_BUSY;
    if (!start_transaction(command, payload, payload_length, operation, false))
        return RADAR_CLIENT_REQUEST_QUEUE_FULL;
    return RADAR_CLIENT_REQUEST_ACCEPTED;
}

radar_client_request_result_t RadarClient_RequestStartContinuous(void)
{
    return request_ready_command(LINK_CMD_START, NULL, 0U,
                                 RADAR_CLIENT_OPERATION_START_CONTINUOUS);
}

radar_client_request_result_t RadarClient_RequestScanOnce(void)
{
    clear_results();
    return request_ready_command(LINK_CMD_SCAN_LOCKED, NULL, 0U,
                                 RADAR_CLIENT_OPERATION_SCAN_ONCE);
}

radar_client_request_result_t RadarClient_RequestGridScan(void)
{
    clear_results();
    return request_ready_command(LINK_CMD_SINGLE_MEASURE, NULL, 0U,
                                 RADAR_CLIENT_OPERATION_GRID_SCAN);
}

radar_client_request_result_t RadarClient_RequestStop(void)
{
    uint16_t sequence;
    if (!state.connected) return RADAR_CLIENT_REQUEST_NOT_READY;

    /* STOP取消旧事务、清空普通待发帧，并通过urgent槽抢占下一次发送。 */
    clear_transaction();
    clear_results();
    tx_tail = tx_head;
    point_cloud_enabled = false;
    if (!queue_encoded(current_node_id, LINK_CMD_STOP, NULL, 0U,
                       true, &sequence))
        return RADAR_CLIENT_REQUEST_QUEUE_FULL;
    memset(&transaction, 0, sizeof(transaction));
    transaction.active = true;
    transaction.command = LINK_CMD_STOP;
    transaction.operation = RADAR_CLIENT_OPERATION_STOP;
    transaction.request_sequence = sequence;
    transaction.deadline_ms = RadarUartPort_GetTick() + CLIENT_ACK_TIMEOUT_MS;
    notify_state();
    return RADAR_CLIENT_REQUEST_ACCEPTED;
}

radar_client_request_result_t RadarClient_RequestCalibrate(uint8_t zone)
{
    uint8_t payload = zone;
    radar_client_request_result_t result;
    if (!zone_valid(zone) || zone != state.selected_zone)
        return RADAR_CLIENT_REQUEST_BAD_PARAMETER;
    result = request_ready_command(LINK_CMD_CALIBRATE, &payload, 1U,
                                   RADAR_CLIENT_OPERATION_CALIBRATE);
    if (result == RADAR_CLIENT_REQUEST_ACCEPTED)
        transaction.requested_zone = zone;
    return result;
}

static radar_client_request_result_t request_pose(
    uint8_t command, const radar_client_pose_t *pose,
    radar_client_operation_t operation)
{
    uint8_t payload[7];
    if (!pose_valid(pose) || pose->zone != state.selected_zone)
        return RADAR_CLIENT_REQUEST_BAD_PARAMETER;
    payload[0] = pose->zone;
    write_i16(&payload[1], pose->x_mm);
    write_i16(&payload[3], pose->y_mm);
    write_u16(&payload[5], pose->heading_q6);
    return request_ready_command(command, payload, sizeof(payload), operation);
}

radar_client_request_result_t RadarClient_RequestSetPose(
    const radar_client_pose_t *pose)
{
    return request_pose(LINK_CMD_SET_POSE, pose,
                        RADAR_CLIENT_OPERATION_SET_POSE);
}

radar_client_request_result_t RadarClient_RequestSavePose(
    const radar_client_pose_t *pose)
{
    radar_client_request_result_t result =
        request_pose(LINK_CMD_SAVE_POSE, pose,
                     RADAR_CLIENT_OPERATION_SAVE_POSE);
    if (result == RADAR_CLIENT_REQUEST_ACCEPTED)
        transaction.requested_zone = pose->zone;
    return result;
}

radar_client_request_result_t RadarClient_RequestResetCalibration(uint8_t zone)
{
    uint8_t payload = zone;
    radar_client_request_result_t result;
    if (!zone_valid(zone) || zone != state.selected_zone)
        return RADAR_CLIENT_REQUEST_BAD_PARAMETER;
    result = request_ready_command(LINK_CMD_RESET_CALIBRATION, &payload, 1U,
                                   RADAR_CLIENT_OPERATION_RESET_CALIBRATION);
    if (result == RADAR_CLIENT_REQUEST_ACCEPTED)
        transaction.requested_zone = zone;
    return result;
}

radar_client_request_result_t RadarClient_RequestClear(void)
{
    clear_results();
    return request_ready_command(LINK_CMD_CLEAR, NULL, 0U,
                                 RADAR_CLIENT_OPERATION_CLEAR);
}

radar_client_request_result_t RadarClient_RequestPlanPath(uint8_t start,
                                                         uint8_t goal)
{
    uint8_t payload[2];
    if (start > 24U || goal > 24U)
        return RADAR_CLIENT_REQUEST_BAD_PARAMETER;
    payload[0] = start;
    payload[1] = goal;
    return request_ready_command(LINK_CMD_PLAN_PATH, payload, sizeof(payload),
                                 RADAR_CLIENT_OPERATION_PLAN_PATH);
}

radar_client_request_result_t RadarClient_RequestSetNodeId(uint8_t new_node_id)
{
    radar_client_request_result_t result;
    if (new_node_id < 1U || new_node_id > LINK_MAX_NODE_ID)
        return RADAR_CLIENT_REQUEST_BAD_PARAMETER;
    result = request_ready_command(LINK_CMD_SET_NODE_ID, &new_node_id, 1U,
                                   RADAR_CLIENT_OPERATION_SET_NODE_ID);
    if (result == RADAR_CLIENT_REQUEST_ACCEPTED)
        transaction.requested_node_id = new_node_id;
    return result;
}

radar_client_request_result_t RadarClient_RequestPointCloudOutput(bool enabled)
{
    uint8_t payload = enabled ? 1U : 0U;
    radar_client_request_result_t result =
        request_ready_command(LINK_CMD_SET_POINT_CLOUD_OUTPUT, &payload, 1U,
                              RADAR_CLIENT_OPERATION_SET_POINT_CLOUD);
    if (result == RADAR_CLIENT_REQUEST_ACCEPTED)
        transaction.requested_value = payload;
    return result;
}

void RadarClient_OnUartTxCompleteInterrupt(void)
{
    /* ISR中只置位；出队和下一帧发送由Poll完成。 */
    tx_complete_pending = true;
}

void RadarClient_OnUartErrorInterrupt(uint32_t error_code)
{
    /* ISR中保留HAL错误码；协议清理和DMA重启由Poll按100 ms限速执行。 */
    uart_error_code = error_code;
    uart_error_pending = true;
}
