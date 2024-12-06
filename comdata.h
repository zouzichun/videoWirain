#ifndef COMDATA_H
#define COMDATA_H
#include <QString>

typedef struct {
    int netType;  // 0-tcp, 1-udp client, 2-serial, 3-modbus can
    
    // net config
    QString ip;
    int portAddr;
    bool isReplyTimeout;

    // serial config
    QString serialName;
    qint32 baudRate;

    // modbus config
    QString modbusName;
    qint32 modbusRate;

    float camera_height;
    float camera_angle;
    float camera_abs_x;
    float camera_abs_y;
    int point1_x;
    int point1_y;
    int point2_x;
    int point2_y;

    int canny_1;
    int canny_2;
    int canny_3;
    int hgline_1;
    int hgline_2;
    int hgline_3;
    int blur_kernel;

    int line1_ang;
    int line1_roh;
    float line1_ang_delta;
    int line1_sel_low;
    int line2_ang;
    int line2_roh;
    float line2_ang_delta;
    int line2_sel_low;
    float line_abs;
    int lines_num;
} ConfigData;

extern ConfigData configData;

///!!!注意：由于下面的结构体会用于通信中
///所以需要按照1字节对齐（或者通信两端采用相同的字节对齐方式）
///以避免出现通信错误。
#pragma pack(push)//保存对齐状态
#pragma pack(1)//按照1字节对齐,以便于通信
/*-----
功能: 通信数据结构体
说明: 示意用
-----*/
typedef enum {
    GETADDR = 1,
    SETADDR = 2,
    SETVEC = 3,
    FORWARD = 4,
    FORWARDTO = 5,
    BACKWARD = 6,
    BACKWARDTO = 7,
    GETttADDR = 8,
    STOP = 9,
}CMDDEF;

typedef struct
{
    unsigned char sof0;  // 0x55
    unsigned char sof1;  // 0xaa
    unsigned char cmd;
    unsigned char rev;
    unsigned long size;
    unsigned char * data_ptr;
    unsigned short crc;
} MsgStruct;

typedef enum {
    ACK_RESP = 0X2,
    GET_ENCODER_VAL = 0X30,
    GET_INPUT_PAUSE_NUM = 0X33,
    GET_MOTOR_POSITION = 0X36,
    GET_POSITION_DIFF = 0X39,
    GET_ENABLEMENT_STATUS = 0X3A,
    GET_STUCK_STATUS = 0X3E,
    SET_ENABLE_MOTOR = 0XF3,
    SET_MOTOR_SPEED_MODE = 0XF6,
    SET_MOTOR_POSITION_MODE = 0XFD,
    ERROR_RESP = 0XEE,
} UARTCMDDEF;

typedef struct {
    unsigned char uart_slv_id;
    unsigned char cmd;
    union {
        struct {
            unsigned char dir_speed_high;
            unsigned char speed_low;
            unsigned char acc_speed;
            unsigned char pulse_high;
            unsigned char pulse_middle;
            unsigned char pulse_low;
        } set_positon;

        struct {
            unsigned char dir_speed_high;
            unsigned char speed_low;
            unsigned char acc_speed;
        } set_speed;

        unsigned char set_enable_motor;
    } param;
    unsigned char crc;
} UartCmdFrame;

#pragma pack(pop)

#endif // COMDATA_H
