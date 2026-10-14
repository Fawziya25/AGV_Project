#ifndef RADAR_CLIENT_H
#define RADAR_CLIENT_H

/*
 * 雷达控制端非阻塞客户端
 * ======================
 *
 * 本模块位于应用Demo与link_protocol/radar_uart_port之间，负责：
 * 1. 与指定节点完成HELLO握手并维持500 ms心跳；
 * 2. 显式选择工作区域，并等待ACK、HELLO和保存状态确认；
 * 3. 保证同一时间最多只有一个普通业务事务在执行；
 * 4. 将分块协议事件重组为障碍、方格、点云、校准和路径结果；
 * 5. 在串口异常或5秒无有效帧时清理旧事务并自动重新握手。
 *
 * 推荐调用顺序：
 *   RadarClient_Init(&callbacks, node_id);
 *   RadarClient_RequestSelectZone(1U);       // 业务Demo必须显式选区
 *   for (;;) {
 *       RadarClient_Poll();                  // 必须高频、非阻塞调用
 *       if (RadarClient_IsReady()) { ... }   // READY后才能提交普通业务
 *   }
 *
 * 所有Request函数都只提交请求并立即返回。返回ACCEPTED只表示请求已被
 * 接受，不表示雷达已完成操作；最终数据和错误必须通过回调取得。
 */

#include <stdbool.h>
#include <stdint.h>

#define RADAR_CLIENT_MAX_OBSTACLES 25U       /* 单次结果最多25个障碍。 */
#define RADAR_CLIENT_MAX_GRID_IDS 25U        /* 场地共有25个方格。 */
#define RADAR_CLIENT_MAX_WAYPOINTS 24U       /* 路径不回传起点，最多24项。 */
#define RADAR_CLIENT_GRID_OUTSIDE 0xFFU      /* 完整扫描中表示障碍不在25格内。 */
#define RADAR_CLIENT_MAX_DISPLAY_POINTS 120U /* 仅用于显示的抽样点上限。 */

/* 工作区域编号。NONE表示本次上电还没有确认工作区域。 */
typedef enum {
    RADAR_CLIENT_ZONE_NONE = 0,
    RADAR_CLIENT_ZONE_1 = 1,
    RADAR_CLIENT_ZONE_2 = 2
} radar_client_zone_t;

typedef enum {
    RADAR_CLIENT_LINK_DISCONNECTED = 0, /* 尚未初始化或链路已主动停用。 */
    RADAR_CLIENT_LINK_WAIT_HELLO,       /* 正在周期发送HELLO寻找目标节点。 */
    RADAR_CLIENT_LINK_NEED_ZONE,        /* 已连接，但本次尚未确认工作区域。 */
    RADAR_CLIENT_LINK_WAIT_ZONE_CONFIRM,/* 等待选区ACK及目标区域HELLO。 */
    RADAR_CLIENT_LINK_READING_STATE,    /* 选区后正在读取Flash保存状态。 */
    RADAR_CLIENT_LINK_READY             /* 可提交扫描、校准和路径等业务。 */
} radar_client_link_state_t;

/* 当前正在等待完成的事务类型；NONE表示没有普通事务占用客户端。 */
typedef enum {
    RADAR_CLIENT_OPERATION_NONE = 0,
    RADAR_CLIENT_OPERATION_CONNECT,
    RADAR_CLIENT_OPERATION_READ_STATE,
    RADAR_CLIENT_OPERATION_SELECT_ZONE,
    RADAR_CLIENT_OPERATION_START_CONTINUOUS,
    RADAR_CLIENT_OPERATION_SCAN_ONCE,
    RADAR_CLIENT_OPERATION_GRID_SCAN,
    RADAR_CLIENT_OPERATION_CALIBRATE,
    RADAR_CLIENT_OPERATION_SET_POSE,
    RADAR_CLIENT_OPERATION_SAVE_POSE,
    RADAR_CLIENT_OPERATION_RESET_CALIBRATION,
    RADAR_CLIENT_OPERATION_CLEAR,
    RADAR_CLIENT_OPERATION_PLAN_PATH,
    RADAR_CLIENT_OPERATION_SET_NODE_ID,
    RADAR_CLIENT_OPERATION_SET_POINT_CLOUD,
    RADAR_CLIENT_OPERATION_STOP
} radar_client_operation_t;

typedef enum {
    RADAR_CLIENT_REQUEST_ACCEPTED = 0, /* 已登记或入队，等待后续回调。 */
    RADAR_CLIENT_REQUEST_NOT_READY,    /* 未连接或工作区域尚未确认。 */
    RADAR_CLIENT_REQUEST_BUSY,         /* 已有一个普通业务事务在执行。 */
    RADAR_CLIENT_REQUEST_BAD_PARAMETER,/* 参数范围或区域不符合要求。 */
    RADAR_CLIENT_REQUEST_QUEUE_FULL    /* 发送队列暂时没有可用空间。 */
} radar_client_request_result_t;

typedef enum {
    RADAR_CLIENT_ERROR_DEVICE = 1, /* 雷达通过ACK或ERROR事件拒绝请求。 */
    RADAR_CLIENT_ERROR_PROTOCOL,   /* 协议版本或事务完成条件异常。 */
    RADAR_CLIENT_ERROR_TIMEOUT,    /* ACK或最终业务事件等待超时。 */
    RADAR_CLIENT_ERROR_UART,       /* UART/DMA异常，客户端将尝试恢复。 */
    RADAR_CLIENT_ERROR_QUEUE_FULL, /* 内部发送队列已满。 */
    RADAR_CLIENT_ERROR_BAD_EVENT,  /* 事件长度、字段或分块关系非法。 */
    RADAR_CLIENT_ERROR_LINK_LOST   /* 5秒没有收到目标节点有效帧。 */
} radar_client_error_kind_t;

typedef struct {
    int16_t x_mm;       /* 雷达中心的场地X坐标，单位mm。 */
    int16_t y_mm;       /* 雷达中心的场地Y坐标，单位mm。 */
    uint16_t heading_q6;/* 朝向角×64，范围0～23039，对应0～<360°。 */
    uint8_t zone;       /* RADAR_CLIENT_ZONE_1或ZONE_2。 */
} radar_client_pose_t;

typedef struct {
    uint8_t firmware_major;      /* 固件主版本号。 */
    uint8_t firmware_minor;      /* 固件次版本号。 */
    uint8_t protocol;            /* 必须等于当前LINK_PROTOCOL_VERSION。 */
    radar_client_pose_t pose;    /* 当前区域及其实际会话位姿；zone可为0。 */
    bool running;                /* 下位机当前是否正在处理雷达数据。 */
    uint16_t max_scan_points;    /* 下位机单圈点容量，仅用于能力显示。 */
} radar_client_hello_t;

typedef struct {
    bool connected;                 /* 已收到目标节点的有效HELLO。 */
    bool ready;                     /* 区域和保存状态已确认，可做业务。 */
    bool busy;                      /* 当前存在尚未完成的普通事务。 */
    radar_client_link_state_t link_state;
    radar_client_operation_t operation;
    uint8_t node_id;
    uint8_t selected_zone;          /* 当前已确认区域；0表示未确认。 */
    uint8_t status;                 /* 雷达原始状态码，来自STATUS事件。 */
    uint8_t mode;                   /* 雷达原始工作模式，来自STATUS事件。 */
    bool running;                   /* 雷达下位机当前是否正在采集。 */
    uint8_t frame_count;            /* 当前操作已取得的完整圈数。 */
    bool overflow;                  /* 本次采集是否发生有界容量溢出。 */
} radar_client_state_t;

typedef struct {
    int16_t x_mm;              /* 障碍中心场地X坐标，单位mm。 */
    int16_t y_mm;              /* 障碍中心场地Y坐标，单位mm。 */
    uint8_t grid_id;           /* 方格编号0～24；0xFF表示方格区域外。 */
    uint8_t frame_hits;        /* 14圈窗口中命中的圈数。 */
    uint8_t confidence_percent;/* 下位机给出的0～100置信度。 */
    uint16_t point_count;      /* 合并到该障碍的点数量。 */
} radar_client_obstacle_t;

typedef struct {
    uint32_t scan_sequence; /* 用于关联同一次扫描的GRID与RESULT。 */
    uint8_t frame_count;    /* 本结果使用的完整圈数。 */
    bool locked;            /* true表示一次14圈锁定结果。 */
    bool overflow;          /* true时本结果不应作为完整业务结果使用。 */
    uint8_t obstacle_count; /* obstacles数组中的有效项数。 */
    radar_client_obstacle_t obstacles[RADAR_CLIENT_MAX_OBSTACLES];
} radar_client_obstacle_result_t;

typedef struct {
    uint32_t scan_sequence; /* 扫描序号；不能用到达时间代替关联。 */
    uint8_t count;          /* 0也是有效结果，表示没有障碍方格。 */
    uint8_t grid_ids[RADAR_CLIENT_MAX_GRID_IDS];
} radar_client_grid_result_t;

typedef struct {
    int16_t x_mm;      /* 显示点的场地X坐标，单位mm。 */
    int16_t y_mm;      /* 显示点的场地Y坐标，单位mm。 */
    uint8_t intensity; /* 原始强度，仅供显示或诊断。 */
} radar_client_point_t;

typedef struct {
    uint32_t scan_sequence;
    uint32_t timestamp_ms;
    uint16_t frequency_deci_hz;
    uint16_t source_point_count;
    uint16_t point_count;        /* 本事件实际回传的显示点数量。 */
    uint16_t nearest_mm;
    radar_client_point_t points[RADAR_CLIENT_MAX_DISPLAY_POINTS];
} radar_client_point_cloud_t;

typedef struct {
    uint32_t scan_sequence;
    uint16_t reported_frequency_deci_hz;
    uint16_t measured_frequency_deci_hz;
    uint16_t point_count;
    uint32_t good_packets;
    uint32_t bad_checksums;
    uint32_t discarded_bytes;
    uint16_t processing_ms;
    uint8_t flags;
} radar_client_stats_t;

typedef struct {
    radar_client_pose_t pose;       /* 本次校准后保存的当前区实际位姿。 */
    int16_t x_offset_mm;            /* 相对当前区默认位姿的X偏移。 */
    int16_t y_offset_mm;            /* 相对当前区默认位姿的Y偏移。 */
    int16_t heading_trim_q6;        /* 朝向偏移×64，可为负数。 */
    uint8_t residual_mm;            /* 两柱重投影残差，单位mm。 */
    uint8_t pair_distance_error_mm; /* 两柱间距误差，单位mm。 */
} radar_client_calibration_t;

typedef struct {
    uint16_t storage_version;
    uint32_t record_sequence;
    uint8_t source_zone; /* 最近一次修改位姿的区域，不代表当前工作区域。 */
    bool grid_result_valid; /* true时grid_count=0也表示有效空扫描。 */
    uint8_t grid_count; /* grid_ids有效项数；空结果时可以为0。 */
    radar_client_pose_t zone1_pose; /* 区域1独立保存的位姿。 */
    radar_client_pose_t zone2_pose; /* 区域2独立保存的位姿。 */
    uint8_t grid_ids[RADAR_CLIENT_MAX_GRID_IDS];
} radar_client_persisted_state_t;

typedef struct {
    uint16_t request_sequence; /* 必须与本次PLAN_PATH命令序号匹配。 */
    uint8_t status;            /* 0表示成功，其他值为路径业务状态。 */
    uint8_t start;             /* 请求中的起点方格0～24。 */
    uint8_t goal;              /* 请求中的终点方格0～24。 */
    uint8_t waypoint_count;    /* 返回的拐点和终点数量，不含起点。 */
    uint8_t waypoints[RADAR_CLIENT_MAX_WAYPOINTS];
} radar_client_path_result_t;

typedef struct {
    radar_client_error_kind_t kind;       /* 错误来源类别。 */
    radar_client_operation_t operation;  /* 出错时正在执行的事务。 */
    uint8_t code;                        /* 雷达错误码或本地辅助码。 */
    uint16_t detail;                     /* 命令、事件、长度等附加信息。 */
} radar_client_error_t;

/*
 * 所有回调都在RadarClient_Poll()所在的任务或主循环中执行。
 * 回调内不能延时、不能再次调用Poll，也不要进行耗时计算。
 * data指针只在本次回调期间有效，需要长期使用时应立即复制内容。
 */
typedef struct {
    void (*hello)(const radar_client_hello_t *data);       /* 握手及健康探测。 */
    void (*state)(const radar_client_state_t *data);       /* 链路/事务状态变化。 */
    void (*persisted)(const radar_client_persisted_state_t *data);/* Flash状态。 */
    void (*obstacles)(const radar_client_obstacle_result_t *data);/* 完整障碍结果。 */
    void (*grid)(const radar_client_grid_result_t *data);  /* 障碍方格结果。 */
    void (*point_cloud)(const radar_client_point_cloud_t *data);/* 完整显示点云。 */
    void (*stats)(const radar_client_stats_t *data);       /* 点云开启时的统计。 */
    void (*calibration)(const radar_client_calibration_t *data);/* 校准结果。 */
    void (*path)(const radar_client_path_result_t *data);  /* 路径拐点/终点。 */
    void (*error)(const radar_client_error_t *data);       /* 本地或雷达错误。 */
} radar_client_callbacks_t;

/*
 * 初始化客户端并启动RX DMA。
 * node_id有效范围1～254，无效值会退回默认ID 1。
 * 可以在重新初始化前再次调用，但它会清空旧事务、结果和点云缓存；调用方
 * 应先保证旧的业务结果不再被使用。本函数不选择工作区域。
 */
void RadarClient_Init(const radar_client_callbacks_t *callbacks,
                      uint8_t node_id);
/*
 * 非阻塞服务入口。裸机主循环应持续调用；FreeRTOS中建议由单独任务调用，
 * 周期不宜超过10 ms。所有用户回调均从本函数上下文执行。
 */
void RadarClient_Poll(void);
/* 复制当前状态快照；返回false只表示传入了NULL。 */
bool RadarClient_GetState(radar_client_state_t *state);
/* 工作区域和Flash状态均确认后返回true。 */
bool RadarClient_IsReady(void);
/* 有普通事务正在等待ACK或最终事件时返回true。 */
bool RadarClient_IsBusy(void);

/* 主动发送一次HELLO；通常无需调用，客户端会自动握手和健康探测。 */
radar_client_request_result_t RadarClient_RequestHello(void);
/* 读取雷达Flash保存状态，结果通过persisted回调返回。 */
radar_client_request_result_t RadarClient_RequestReadPersistedState(void);
/*
 * 选择工作区域。可以紧接RadarClient_Init调用：尚未连接时先登记请求，
 * 完成HELLO握手后自动发送；已连接时立即进入非阻塞选区流程。
 */
radar_client_request_result_t RadarClient_RequestSelectZone(uint8_t zone);
/* 持续扫描：ACK成功后持续通过obstacles回调返回结果，使用STOP结束。 */
radar_client_request_result_t RadarClient_RequestStartContinuous(void);
/*
 * 14圈完整扫描：必须同时收到匹配序号的GRID、RESULT和完成状态后结束，
 * obstacles回调返回坐标、方格和置信度；该操作不保存方格结果到Flash。
 */
radar_client_request_result_t RadarClient_RequestScanOnce(void);
/*
 * 14圈方格扫描：通过grid回调只返回障碍方格编号。雷达会把去重后的
 * 方格结果保存到自身Flash；count=0会保存为有效空结果并清除旧方格。
 */
radar_client_request_result_t RadarClient_RequestGridScan(void);
/* STOP具有高优先级，可取消扫描或校准；它不会关闭物理串口。 */
radar_client_request_result_t RadarClient_RequestStop(void);
/*
 * 校准当前区域。雷达成功时会自动把校准位姿保存到自身Flash；客户端
 * 必须收到ACK、CALIBRATION及source_zone匹配的PERSISTED_STATE才完成。
 */
radar_client_request_result_t RadarClient_RequestCalibrate(uint8_t zone);
/* 仅修改当前会话位姿，掉电丢失，不写Flash。 */
radar_client_request_result_t RadarClient_RequestSetPose(const radar_client_pose_t *pose);
/* 将给定当前区位姿写入Flash，并等待HELLO和保存状态双重确认。 */
radar_client_request_result_t RadarClient_RequestSavePose(const radar_client_pose_t *pose);
/* 只把当前确认区域恢复为默认位姿，不影响另一区域。 */
radar_client_request_result_t RadarClient_RequestResetCalibration(uint8_t zone);
/* 清除当前雷达业务结果；不会格式化整个参数Flash。 */
radar_client_request_result_t RadarClient_RequestClear(void);
/* 路径规划前必须先完成一次有效方格扫描；结果通过path回调返回。 */
radar_client_request_result_t RadarClient_RequestPlanPath(uint8_t start, uint8_t goal);
/* 修改并持久化节点ID；成功ACK仍来自旧ID，随后以新ID HELLO完成切换。 */
radar_client_request_result_t RadarClient_RequestSetNodeId(uint8_t new_node_id);
/* 只控制显示点云，默认关闭且不保存到Flash；结果由point_cloud/stats回调返回。 */
radar_client_request_result_t RadarClient_RequestPointCloudOutput(bool enabled);

/* HAL_UART_TxCpltCallback中仅转发完成标志，不在中断里继续发送。 */
void RadarClient_OnUartTxCompleteInterrupt(void);
/* HAL_UART_ErrorCallback中登记错误；DMA恢复由下一次Poll完成。 */
void RadarClient_OnUartErrorInterrupt(uint32_t error_code);

#endif
