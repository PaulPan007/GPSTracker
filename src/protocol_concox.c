#include "gps.h"
#include "protocol_concox.h"
#include <stdio.h>
#include "applied_math.h"
#include "utility.h"
#include "gm_stdlib.h"
#include "gm_memory.h"
#include "gm_type.h"
#include "gsm.h"
#include "log_service.h"
#include "config_service.h"
#include "hard_ware.h"
#include "protocol.h"
#include "gps.h"
#include "gps_service.h"
#include "sms.h"
#include "system_state.h"
#include "command.h"
#include "g_sensor.h"
#include "agps_service.h"

typedef enum
{
    PROTOCCOL_CONCOX_LOGIN = 0x01,  // 登录包
    PROTOCCOL_CONCOX_LOCAT_GT06 = 0x12,  // 位置包
    PROTOCCOL_CONCOX_HEART = 0x13,  // 心跳
    PROTOCCOL_CONCOX_STRING_MSG = 0x15, 
    PROTOCCOL_CONCOX_ALARM_GT06 = 0x16, 
    PROTOCCOL_CONCOX_CH_LOCAT_ACK = 0x17, //中文地址请求或者报警回复
    PROTOCCOL_CONCOX_REMOTE_MSG = 0x21,
    PROTOCCOL_CONCOX_LOCAT = 0x22,
    PROTOCCOL_CONCOX_ALARM = 0x26, 
    PROTOCCOL_CONCOX_ZONE_TIE = 0x27, 
    PROTOCCOL_CONCOX_FULL_LBS = 0x28,  // 多基站数据
    PROTOCCOL_CONCOX_LOCAT_REQ = 0x2A,
    PROTOCCOL_CONCOX_HEART_EXPAND = 0x36, //康凯斯扩展心跳
    PROTOCCOL_CONCOX_REMOTE_MSG_REQ = 0x80,
    PROTOCCOL_CONCOX_STATUS_MSG = 0x94, //信息传输通用包
    PROTOCCOL_CONCOX_EN_LOCAT_ACK = 0x97, //英文地址请求或者报警回复
}ProtocolConcoxCmdEnum;

typedef enum
{
    DEV_STATUS_EX_POWER = 0x00,
    DEV_STATUS_SYNC = 0x04,
    DEV_STATUS_DOOR = 0x05,
    DEV_STATUS_SELF_TEST = 0x08,
    DEV_STATUS_FIXED_SAT = 0x09,
    DEV_STATUS_ICCID_IMEI = 0x0A,
    DEV_STATUS_MAX
}ProtocolConcoxDeviceStatusEnum;


/*用于记录发送消息的序列号, 收到消息的序列号等*/
typedef struct
{
    AlarmTypeEnum last_alarm;  //最近一个报警消息
    u16 msg_serial;  //上传数据序列号, 每次发消息加一
    u8 msg_flag;     // 信息标识, 服务器传过来, 原样传回
    u8 server_id[4]; // 服务器标志  , 服务器传过来, 原样传回
}ConcoxMsgSave;

static ConcoxMsgSave s_concox_msg_save = {ALARM_NONE, 0 , };

static protocol_concox_pack_head(u8 *pdata, u16 *idx, u16 len, bool new);
static protocol_concox_pack_tail(u8 *pdata, u16 *idx, u16 len);
static void protocol_concox_pack_imei(U8* pdata, u16 *idx, u16 len);
static void protocol_concox_pack_imsi(U8* pdata, u16 *idx, u16 len);
static void protocol_concox_pack_iccid(U8* pdata, u16 *idx, u16 len);
static void protocol_concox_pack_type_id(U8* pdata, u16 *idx, u16 len);
static void protocol_concox_pack_language(U8* pdata, u16 *idx, u16 len);
static void protocol_concox_pack_id_len(U8* pdata, u8 id, u16 len, bool new);
static void protocol_concox_pack_device_status(u8 *pdata, u16 *idx, u16 len);
static void protocol_concox_pack_voltage_status(u8 *pdata, u16 *idx, u16 len);
static void protocol_concox_pack_signal_status(u8 *pdata, u16 *idx, u16 len);
static void protocol_concox_pack_alarm_language(u8 *pdata, u16 *idx, u16 len, AlarmInfo *alarm);
static void protocol_concox_pack_power_volt(u8 *pdata, u16 *idx, u16 len);

static void protocol_concox_pack_device_status_imei(u8 *pdata, u16 *idx, u16 len);

static void protocol_concox_parse_login_response(U8* pdata, u16 len, bool new);
static void protocol_concox_parse_ch_locate_ack(U8* pdata, u16 len, bool ucs2);
static void protocol_concox_alarm_sms_ask(U8* pdata, u16 len, bool ucs2);
static void protocol_concox_parse_remote_msg(U8* pdata, u16 len, bool new);
static void protocol_concox_parse_en_locate_ack(U8* pdata, u16 len);


static protocol_concox_pack_head(u8 *pdata, u16 *idx, u16 len, bool new)
{
    u8 tag = PROTOCOL_HEADER_CONCOX;
    if(new)
    {
        tag = PROTOCOL_HEADER_CONCOX_NEW;
    }
    
    if((*idx) + 4 > len)
    {
        LOG(WARN,"protocol_concox_pack_head assert(len(%d)) failed.", len);
        return;
    }
    
    pdata[(*idx)++] = tag;
    pdata[(*idx)++] = tag;

    if(new)
    {
        (*idx) = (*idx) + 3; //  len 2   id 1
    }
    else
    {
        (*idx) = (*idx) + 2; //  len 1   id 1
    }
}

static protocol_concox_pack_tail(u8 *pdata, u16 *idx, u16 len)
{
    if((*idx) + 6 > len)
    {
        LOG(WARN,"protocol_concox_pack_tail assert(len(%d)) failed.", len);
        return;
    }
    
    ++(s_concox_msg_save.msg_serial);
    
    pdata[(*idx)++] = BHIGH_BYTE(s_concox_msg_save.msg_serial);
    pdata[(*idx)++] = BLOW_BYTE(s_concox_msg_save.msg_serial);

    (*idx) += 2;    //checksum
    
    pdata[(*idx)++] = 0x0D;
    pdata[(*idx)++] = 0x0A;
}

static void protocol_concox_pack_imei(U8* pdata, u16 *idx, u16 len)
{
    GM_ERRCODE ret = GM_SUCCESS;
    u8 imei[GM_IMEI_LEN + 1] = {0};
    if(GM_SUCCESS != (ret = gsm_get_imei(imei)))
    {
        LOG(INFO,"protocol_concox_pack_imei can not get imei, ret:%d.", ret);
    }
    
    if (0 == GM_strlen((const char *)imei))
    {
        GM_memset(imei, 0, sizeof(imei));
    }

    if((*idx) + 8 > len)
    {
        LOG(WARN,"protocol_concox_pack_imei assert(len(%d)) failed.", len);
        return;
    }
    
    pdata[(*idx)++] = util_chr(imei[0]);
    pdata[(*idx)++] = MERGEBCD(util_chr(imei[1]), util_chr(imei[2]));
    pdata[(*idx)++] = MERGEBCD(util_chr(imei[3]), util_chr(imei[4]));
    pdata[(*idx)++] = MERGEBCD(util_chr(imei[5]), util_chr(imei[6]));
    pdata[(*idx)++] = MERGEBCD(util_chr(imei[7]), util_chr(imei[8]));
    pdata[(*idx)++] = MERGEBCD(util_chr(imei[9]), util_chr(imei[10]));
    pdata[(*idx)++] = MERGEBCD(util_chr(imei[11]), util_chr(imei[12]));
    pdata[(*idx)++] = MERGEBCD(util_chr(imei[13]), util_chr(imei[14]));
}


static void protocol_concox_pack_imsi(U8* pdata, u16 *idx, u16 len)
{
    GM_ERRCODE ret = GM_SUCCESS;
    u8 imsi[GM_IMSI_LEN + 1] = {0};
    if(GM_SUCCESS != (ret = gsm_get_imsi(imsi)))
    {
        LOG(INFO,"protocol_concox_pack_imsi can not get imsi, ret:%d.", ret);
    }
    
    if (0 == GM_strlen((const char *)imsi))
    {
        GM_memset(imsi, 0, sizeof(imsi));
    }

    if((*idx) + 8 > len)
    {
        LOG(WARN,"protocol_concox_pack_imsi assert(len(%d)) failed.", len);
        return;
    }
    
    pdata[(*idx)++] = util_chr(imsi[0]);
    pdata[(*idx)++] = MERGEBCD(util_chr(imsi[1]), util_chr(imsi[2]));
    pdata[(*idx)++] = MERGEBCD(util_chr(imsi[3]), util_chr(imsi[4]));
    pdata[(*idx)++] = MERGEBCD(util_chr(imsi[5]), util_chr(imsi[6]));
    pdata[(*idx)++] = MERGEBCD(util_chr(imsi[7]), util_chr(imsi[8]));
    pdata[(*idx)++] = MERGEBCD(util_chr(imsi[9]), util_chr(imsi[10]));
    pdata[(*idx)++] = MERGEBCD(util_chr(imsi[11]), util_chr(imsi[12]));
    pdata[(*idx)++] = MERGEBCD(util_chr(imsi[13]), util_chr(imsi[14]));
}

static void protocol_concox_pack_iccid(U8* pdata, u16 *idx, u16 len)
{
    GM_ERRCODE ret = GM_SUCCESS;
    u8 iccid[GM_ICCID_LEN + 1] = {0};
    if(GM_SUCCESS != (ret = gsm_get_iccid(iccid)))
    {
        LOG(INFO,"protocol_concox_pack_iccid can not get iccid, ret:%d.", ret);
    }
    
    if (0 == GM_strlen((const char *)iccid))
    {
        GM_memset(iccid, 0, sizeof(iccid));
    }

    if((*idx) + 10 > len)
    {
        LOG(WARN,"protocol_concox_pack_iccid assert(len(%d)) failed.", len);
        return;
    }
    
    pdata[(*idx)++] = MERGEBCD(util_chr(iccid[0]), util_chr(iccid[1]));
    pdata[(*idx)++] = MERGEBCD(util_chr(iccid[2]), util_chr(iccid[3]));
    pdata[(*idx)++] = MERGEBCD(util_chr(iccid[4]), util_chr(iccid[5]));
    pdata[(*idx)++] = MERGEBCD(util_chr(iccid[6]), util_chr(iccid[7]));
    pdata[(*idx)++] = MERGEBCD(util_chr(iccid[8]), util_chr(iccid[9]));
    pdata[(*idx)++] = MERGEBCD(util_chr(iccid[10]), util_chr(iccid[11]));
    pdata[(*idx)++] = MERGEBCD(util_chr(iccid[12]), util_chr(iccid[13]));
    pdata[(*idx)++] = MERGEBCD(util_chr(iccid[14]), util_chr(iccid[15]));
    pdata[(*idx)++] = MERGEBCD(util_chr(iccid[16]), util_chr(iccid[17]));
    pdata[(*idx)++] = MERGEBCD(util_chr(iccid[18]), util_chr(iccid[19]));
}


static void protocol_concox_pack_type_id(U8* pdata, u16 *idx, u16 len)
{
    u8 value_u8;
    /*
    0x20 0x00 GT02D/GT220 带断油电功能
    0x20 0x01 GT02D/GT220 不带断油电功能
    */

    if((*idx) + 2 > len)
    {
        LOG(WARN,"protocol_concox_pack_type_id assert(len(%d)) failed.", len);
        return;
    }

    config_service_get(CFG_IS_RELAY_ENABLE, TYPE_BOOL, &value_u8, sizeof(value_u8));
    pdata[(*idx)++] = 0x20;  //固定GT02D
    pdata[(*idx)++] = (value_u8) ? 0x00 : 0x01;
}


static void protocol_concox_pack_language(U8* pdata, u16 *idx, u16 len)
{
    u16 language = 0;
    s8 zone;
    u16 zone_data;
    u8 value_u8 = 0;

    if((*idx) + 2 > len)
    {
        LOG(WARN,"protocol_concox_pack_language assert(len(%d)) failed.", len);
        return;
    }

	zone = config_service_get_zone();

    zone_data = (zone&0x7F)*100*16;
    
    pdata[(*idx)++] = BHIGH_BYTE(zone_data);
    value_u8 = BLOW_BYTE(zone_data);
    value_u8 = value_u8 & 0xF0;
    
    if (zone & 0x80)
    {
        SET_BIT3(value_u8);  // 西时区  Bit3 0:东时区  1:西时区
    }
    
    config_service_get(CFG_LANGUAGE, TYPE_SHORT, &language, sizeof(language));
    if (1 == language)
    {
        SET_BIT1(value_u8);  // 10简体中文
    }
    else
    {
        SET_BIT2(value_u8);  // 100英文
    }

    pdata[(*idx)++] = value_u8;
}


//长度=协议号+信息内容+信息序列号+错误校验
static void protocol_concox_pack_id_len(U8* pdata, u8 id, u16 len, bool new)
{
    u16 checksum=0;

    if(new)
    {
        pdata[2] = BHIGH_BYTE(len);
        pdata[3] = BLOW_BYTE(len);
        pdata[4] = id;
    }
    else
    {
        pdata[2] = (u8)len;
        pdata[3] = id;
    }

    checksum = applied_math_calc_common_crc16(&pdata[2], len-1);
    pdata[len + 1] = BHIGH_BYTE(checksum);
    pdata[len + 2] = BLOW_BYTE(checksum);
}




void protocol_concox_pack_login_msg(u8 *pdata, u16 *idx, u16 len)
{
    u8 app_ver;
    protocol_concox_pack_head(pdata, idx, len, false); //4bytes  0x78 0x78 len(1) id(1)
    protocol_concox_pack_imei(pdata, idx, len);  //8bytes
    config_service_get(CFG_PROTOCOL_VER, TYPE_BYTE, &app_ver, sizeof(app_ver));
    if(app_ver == PROTOCOL_VER_GT02)
    {
        protocol_concox_pack_type_id(pdata, idx, len);  //2bytes
        protocol_concox_pack_language(pdata, idx, len);  //2bytes
    }
    protocol_concox_pack_tail(pdata, idx, len);  //6bytes  serial(2) checksum(2) stop(2)

    //len not include 0x78 0x78 len(1) stop(2)
    protocol_concox_pack_id_len(pdata, PROTOCCOL_CONCOX_LOGIN, (*idx) - 5, false); 

    log_service_print_hex((const char *) pdata, *idx);
    return;
}


static void protocol_concox_pack_device_status_imei(u8 *pdata, u16 *idx, u16 len)
{
    pdata[(*idx)++] = DEV_STATUS_ICCID_IMEI;
    protocol_concox_pack_imei(pdata, idx, len);   // 8bytes
    protocol_concox_pack_imsi(pdata, idx, len);  // 8bytes
    protocol_concox_pack_iccid(pdata, idx, len); // 10bytes
}


void protocol_concox_pack_device_status_msg(u8 *pdata, u16 *idx, u16 len)
{
    protocol_concox_pack_head(pdata, idx, len, true); //5bytes  0x79 0x79 len(2) id(1)
    protocol_concox_pack_device_status_imei(pdata, idx, len);  // 27bytes
    protocol_concox_pack_tail(pdata, idx, len);  //6bytes  serial(2) checksum(2) stop(2)

    //len not include 0x79 0x79 len(2) stop(2)
    protocol_concox_pack_id_len(pdata, PROTOCCOL_CONCOX_STATUS_MSG, (*idx) - 6, true); 

    log_service_print_hex((const char *) pdata, *idx);
    return;
}


static void protocol_concox_pack_device_status(u8 *pdata, u16 *idx, u16 len)
{
    u8 app_ver;
    u8 value_u8 = 0;
    bool value_bool = false;
    u32 sys_state;

    if((*idx) + 1 > len)
    {
        LOG(WARN,"protocol_concox_pack_device_status assert(len(%d)) failed.", len);
        return;
    }

    sys_state = system_state_get_status_bits();

    // 油电状态
    if(SYSBIT_RELAY_STATE & sys_state)
    {
        SET_BIT7(value_u8);
    }

    // 定位状态
    if (gps_is_fixed())
    {
        SET_BIT6(value_u8);
    }


    // 充电器插入状态
    if (system_state_get_has_started_charge())
    {
        SET_BIT2(value_u8);
    }

    if(SYSBIT_ALARM_SPEED & sys_state)
    {
        config_service_get(CFG_PROTOCOL_VER, TYPE_BYTE, &app_ver, sizeof(app_ver));
        if(app_ver == PROTOCOL_VER_GT02)
        {
            SET_BIT5(value_u8);
            SET_BIT4(value_u8);
        }
    }
    else if (SYSBIT_ALARM_LOW_POWER & sys_state)
    {
        SET_BIT4(value_u8);
        SET_BIT3(value_u8);
    }
    else if (SYSBIT_ALARM_NO_POWER & sys_state)
    {
        SET_BIT4(value_u8);
    }
    else if (SYSBIT_ALARM_SHOCK & sys_state)
    {
        SET_BIT3(value_u8);
    }

    
    // ACC状态
    if(GM_SUCCESS  !=  hard_ware_get_acc_level(&value_bool))
    {
        LOG(WARN,"protocol_concox_pack_device_status assert(hard_ware_get_acc_level) failed.");
    }
    
    if (value_bool)
    {
        SET_BIT1(value_u8);
    }
    else
    {
        CLR_BIT1(value_u8);
    }

    //设防状态
    config_service_get(CFG_IS_MANUAL_DEFENCE, TYPE_BOOL, &value_bool, sizeof(value_bool));
    if (value_bool)
    {
        SET_BIT0(value_u8);
    }
    else
    {
        CLR_BIT0(value_u8);
    }


    pdata[(*idx)++] = value_u8;
}


static void protocol_concox_pack_voltage_status(u8 *pdata, u16 *idx, u16 len)
{
    u8 level = 0;

    if((*idx) + 1 > len)
    {
        LOG(WARN,"protocol_concox_pack_voltage_status assert(len(%d)) failed.", len);
        return;
    }

    if(GM_SUCCESS != hard_ware_get_internal_battery_level(&level))
    {
        LOG(WARN,"protocol_concox_pack_voltage_status assert(hard_ware_get_power_voltage) failed.");
    }
    pdata[(*idx)++] = level;
}


static void protocol_concox_pack_signal_status(u8 *pdata, u16 *idx, u16 len)
{
    u8 value_u8 = 0;
    u8 val;

    if((*idx) + 1 > len)
    {
        LOG(WARN,"protocol_concox_pack_signal_status assert(len(%d)) failed.", len);
        return;
    }
    
    val = gsm_get_csq();
    
    if (val < 5)
    {
        value_u8 = 0;    // 0x00: 无信号
    }
    else if (val <= 10)
    {
        value_u8 = 1;    // 0x01:信号极弱
    }
    else if (val <= 16)
    {
        value_u8 = 2;    // 0x02:信号较弱
    }
    else if (val <= 25)
    {
        value_u8 = 3;    // 0x03:信号良好
    }
    else if (val <= 31)
    {
        value_u8 = 4;    // 0x04:信号强
    }
    else if (val > 31)
    {
        value_u8 = 0;    // 0x00: 无信号
    }
    else
    {
        value_u8 = 0;    // 0x00: 无信号
    }

    pdata[(*idx)++] = value_u8;
}


static void protocol_concox_pack_alarm_language(u8 *pdata, u16 *idx, u16 len, AlarmInfo *alarm)
{
    u16 language = 0;
    u8 value_u8 = 0;


    if((*idx) + 2 > len)
    {
        LOG(WARN,"protocol_concox_pack_alarm_language assert(len(%d)) failed.", len);
        return;
    }

    //每次只能发送一个报警
    value_u8 = 0;
    if(alarm)
    {
        s_concox_msg_save.last_alarm = alarm->type;
        switch(alarm->type)
        {
            case ALARM_POWER_OFF:
                value_u8 = 0x02;
                break;
            case ALARM_SHOCK:
                value_u8 = 0x03;
                break;
            case ALARM_SPEED:
                value_u8 = 0x06;
                break;
            case ALARM_MOVE:
                value_u8 = 0x09;
                break;
            case ALARM_FAKE_CELL:
                value_u8 = 0x0A;
                break;
            case ALARM_BATTERY_LOW:
                value_u8 = 0x19;
                break;
            case ALARM_POWER_HIGH:
                value_u8 = 0x40;
                break;
            case ALARM_COLLISION:
                switch(alarm->info)
                {
                    case SLIGHT_COLLISION:
                        value_u8 = 0x41;
                        break;
                    case NORMAL_COLLISION:
                        value_u8 = 0x42;
                        break;
                    case SERIOUS_COLLISION:
                        value_u8 = 0x43;
                        break;
                    default: //just for compatable or error.
                        value_u8 = 0x42;
                        break;
                }
                break;
            case ALARM_SPEED_UP:
                value_u8 = 0x44;
                break;
            case ALARM_SPEED_DOWN:
                value_u8 = 0x45;
                break;
            case ALARM_TURN_OVER:
                value_u8 = 0x47;
                break;
            case ALARM_SHARP_TURN:
                value_u8 = 0x46;
                break;
            case ALARM_REMOVE:
                value_u8 = 0x0C;
                break;
            default:
                break;
        }
    }
    pdata[(*idx)++] = value_u8;

    value_u8 = 0;
    config_service_get(CFG_LANGUAGE, TYPE_SHORT, &language, sizeof(language));
    if (1 == language)
    {
        SET_BIT0(value_u8);  // 10简体中文
    }
    else
    {
        SET_BIT1(value_u8);  // 01英文
    }

    pdata[(*idx)++] = value_u8;
}



static void protocol_concox_pack_power_volt(u8 *pdata, u16 *idx, u16 len)
{
    float value_float;
    u16 value_u16 = 0;
    
    if((*idx) + 5 > len)
    {
        LOG(WARN,"protocol_concox_pack_power_volt assert(len(%d)) failed.", len);
        return;
    }


    if(GM_SUCCESS  !=  hard_ware_get_power_voltage(&value_float))
    {
        LOG(WARN,"protocol_concox_pack_power_volt assert(hard_ware_get_power_voltage) failed.");
    }

    pdata[(*idx)++] = 0x00;
    pdata[(*idx)++] = 0x27;
    pdata[(*idx)++] = 0x02;

    value_u16 = (u16)(value_float * 100.0f);
    pdata[(*idx)++] = (u8)UPPER_BYTE(value_u16);
    pdata[(*idx)++] = (u8)LOWER_BYTE(value_u16);
}


void protocol_concox_pack_heartbeat_msg(u8 *pdata, u16 *idx, u16 len)
{
    protocol_concox_pack_head(pdata, idx, len, false); //4bytes  0x78 0x78 len(1) id(1)
    protocol_concox_pack_device_status(pdata, idx, len);  //1bytes
    protocol_concox_pack_voltage_status(pdata, idx, len);  //1bytes
    protocol_concox_pack_signal_status(pdata, idx, len);  //1bytes

    //原版本是心跳与LBS数据都报一条报警, 但由于这个版本中报警消息独立上报,这里就不再带alarm了
    protocol_concox_pack_alarm_language(pdata, idx, len, NULL);  //2bytes

	if(HEART_EXPAND == config_service_get_heartbeat_protocol())
	{
        protocol_concox_pack_power_volt(pdata, idx, len);  //5bytes
        protocol_concox_pack_tail(pdata, idx, len);  //6bytes  serial(2) checksum(2) stop(2)
        protocol_concox_pack_id_len(pdata, PROTOCCOL_CONCOX_HEART_EXPAND, (*idx) - 5, false); 
	}
    else
    {
        protocol_concox_pack_tail(pdata, idx, len);  //6bytes  serial(2) checksum(2) stop(2)

        //len not include 0x78 0x78 len(1) stop(2)
        protocol_concox_pack_id_len(pdata, PROTOCCOL_CONCOX_HEART, (*idx) - 5, false); 
    }

    log_service_print_hex((const char *) pdata, *idx);

    return;
}


static void protocol_concox_pack_gps(const GPSData *gps, U8* pdata, u16 *idx, u16 len)
{
    u8 zone = 0, value_u8 = 0;
    bool value_bool = false;
    
    u32 latitudev = 0;
    u32 longitudev = 0;
    u16 gpsrspeed = 0;
    u16 gpsangle = 0;
    u16 t_idx = *idx;

    if((*idx) + 18 > len)
    {
        LOG(WARN,"protocol_concox_pack_gps assert(len(%d)) failed.", len);
        return;
    }

    //zone = config_service_get_zone();
    util_get_current_local_time(&pdata[(*idx)], NULL, zone);
    for(t_idx = *idx; t_idx < (*idx + 6); ++t_idx)
    {
        pdata[t_idx] = BCD2HEX(pdata[t_idx]);
    }
    (*idx) += 6;
    
    pdata[(*idx)] = 0xC0;
    pdata[(*idx)++] += gps->satellites;
    LOG(DEBUG,"protocol_concox_pack_gps satellites(%d).", gps->satellites);
    
    // 纬度
    if (gps->lat < 0.0)
    {
        latitudev = (u32)(gps->lat * (-1800000.0f));
    }
    else
    {
        latitudev = (u32)(gps->lat * 1800000.0f);
    }

    pdata[(*idx)++] = BHIGH_BYTE(WHIGH_WORD(latitudev));
    pdata[(*idx)++] = BLOW_BYTE(WHIGH_WORD(latitudev));
    pdata[(*idx)++] = BHIGH_BYTE(WLOW_WORD(latitudev));
    pdata[(*idx)++] = BLOW_BYTE(WLOW_WORD(latitudev));
    
    // 经度
    if (gps->lng < 0.0)
    {
        longitudev = (u32)(gps->lng*(-1800000.0f));
    }
    else
    {
        longitudev = (u32)(gps->lng*1800000.0f);
    }
    pdata[(*idx)++] = BHIGH_BYTE(WHIGH_WORD(longitudev));
    pdata[(*idx)++] = BLOW_BYTE(WHIGH_WORD(longitudev));
    pdata[(*idx)++] = BHIGH_BYTE(WLOW_WORD(longitudev));
    pdata[(*idx)++] = BLOW_BYTE(WLOW_WORD(longitudev));
    
    
    // 速度
    gpsrspeed = gps->speed;
    if (gpsrspeed > 180)
    {
        gpsrspeed = 180;
    }
    pdata[(*idx)++] = (u8)gpsrspeed;

     // 角度
    gpsangle = (u16)gps->course;
    if (gpsangle >= 360)
    {
        gpsangle = 0;
    }

    // 纬度 负数为南纬
    value_u8 = 0;
    if (gps->lat >= 0.00001f)
    {
        SET_BIT2(value_u8);  //北纬
    }
    
    // 经度  负数为西经
    if (gps->lng < 0.00001f)
    {
        SET_BIT3(value_u8);  //西经
    }

    switch(gps_get_state())
    {
        case GM_GPS_FIX_3D:
            SET_BIT4(value_u8);  //已定位
            break;
        case GM_GPS_FIX_3D_DIFF:
            SET_BIT4(value_u8);  //已定位
            SET_BIT5(value_u8);  //差分定位
            break;
        default:
            break;
    }
        
    // ACC状态
    if(GM_SUCCESS == hard_ware_get_acc_level(&value_bool))
    {
        SET_BIT6(value_u8);  // ACC有效
        if (value_bool)
        {
            SET_BIT7(value_u8);
        }
    }
    
    pdata[(*idx)++] = BHIGH_BYTE(gpsangle) | value_u8;
    pdata[(*idx)++] = BLOW_BYTE(gpsangle);
}



static void protocol_concox_pack_lbs(U8* pdata, u16 *idx, u16 len)
{
    gm_cell_info_struct lbs;
    GM_ERRCODE ret;
    
    GM_memset(&lbs,0, sizeof(lbs));
    
    if((*idx) + 8 > len)
    {
        LOG(WARN,"protocol_concox_pack_lbs assert(len(%d)) failed.", len);
        return;
    }

    ret = gsm_get_cell_info(&lbs);
    LOG(DEBUG,"clock(%d) protocol_concox_pack_lbs ret(%d) lbs(%d).", util_clock(), ret, lbs.nbr_cell_num);
    
    pdata[(*idx)++] = BHIGH_BYTE(lbs.serv_info.mcc);  // MCC 2
    pdata[(*idx)++] = BLOW_BYTE(lbs.serv_info.mcc);
    
    pdata[(*idx)++] = lbs.serv_info.mnc; // MNC
    
    pdata[(*idx)++] = BHIGH_BYTE(lbs.serv_info.lac);  // LAC 2
    pdata[(*idx)++] = BLOW_BYTE(lbs.serv_info.lac);
    
    
    pdata[(*idx)++] = 0x00;                            //CI 3
    pdata[(*idx)++] = BHIGH_BYTE(lbs.serv_info.ci);
    pdata[(*idx)++] = BLOW_BYTE(lbs.serv_info.ci);
}


static void protocol_concox_pack_acc(GpsDataModeEnum mode, U8* pdata, u16 *idx, u16 len)
{
    bool value_bool = false;

    if((*idx) + 3 > len)
    {
        LOG(WARN,"protocol_concox_pack_acc assert(len(%d)) failed.", len);
        return;
    }
    
    // ACC状态
    if(GM_SUCCESS  !=  hard_ware_get_acc_level(&value_bool))
    {
        LOG(WARN,"protocol_concox_pack_acc assert(hard_ware_get_acc_level) failed.");
    }
    
    if (value_bool)
    {
        pdata[(*idx)++] = 1;
    }
    else
    {
        pdata[(*idx)++] = 0;
    }

    
    // 数据上报模式
    pdata[(*idx)++] = mode;
    
    // GPS 实时补传   现在还不知道状态, 要实际发送时才知道 , 先打包成实时发送. 补发时再纠正
    // 0x00?实时上传 0x01?补传 
    pdata[(*idx)++] = 0;
}


void protocol_concox_pack_gps_msg(GpsDataModeEnum mode, const GPSData *gps, u8 *pdata, u16 *idx, u16 len)
{
    u8 app_ver;
    protocol_concox_pack_head(pdata, idx, len, false); //4bytes  0x78 0x78 len(1) id(1)
    protocol_concox_pack_gps(gps, pdata, idx, len);  //18 bytes
    protocol_concox_pack_lbs(pdata, idx, len);  //8 bytes
    
    config_service_get(CFG_PROTOCOL_VER, TYPE_BYTE, &app_ver, sizeof(app_ver));
    if(app_ver == PROTOCOL_VER_GT02)
    {
        protocol_concox_pack_acc(mode, pdata, idx, len);  //3 bytes
        
        protocol_concox_pack_tail(pdata, idx, len);  //6bytes  serial(2) checksum(2) stop(2)
        
        //len not include 0x78 0x78 len(1) stop(2)
        protocol_concox_pack_id_len(pdata, PROTOCCOL_CONCOX_LOCAT, (*idx) - 5, false); 
        
    }
    else
    {
        protocol_concox_pack_tail(pdata, idx, len);  //6bytes  serial(2) checksum(2) stop(2)
        
        //len not include 0x78 0x78 len(1) stop(2)
        protocol_concox_pack_id_len(pdata, PROTOCCOL_CONCOX_LOCAT_GT06, (*idx) - 5, false); 
    }

    log_service_print_hex((const char *) pdata, *idx);
    return;
}


#if 0
static void protocol_concox_pack_lbses(U8* pdata, u16 *idx, u16 len)
{
    gm_cell_info_struct lbs;
    GM_ERRCODE ret;
    u16 k = 0;
    u16 num=0;
    u16 lbsidx=0;
    u8 zone;
    
    GM_memset(&lbs,0, sizeof(lbs));
    
    if((*idx) + 51 > len)
    {
        LOG(WARN,"protocol_concox_pack_lbses assert(len(%d)) failed.", len);
        return;
    }

    ret = gsm_get_cell_info(&lbs);
    LOG(DEBUG,"clock(%d) protocol_concox_pack_lbses ret(%d) lbs(%d).", util_clock(), ret, lbs.nbr_cell_num);

    zone = config_service_get_zone();
    util_get_current_local_time(&pdata[(*idx)], NULL, zone);
    (*idx) += 6;

    //主基站
    pdata[(*idx)++] = BHIGH_BYTE(lbs.serv_info.mcc);  // MCC 2
    pdata[(*idx)++] = BLOW_BYTE(lbs.serv_info.mcc);
    
    pdata[(*idx)++] = lbs.serv_info.mnc; // MNC
    
    pdata[(*idx)++] = BHIGH_BYTE(lbs.serv_info.lac);  // LAC 2
    pdata[(*idx)++] = BLOW_BYTE(lbs.serv_info.lac);
    
    
    pdata[(*idx)++] = 0x00;                            //CI 3
    pdata[(*idx)++] = BHIGH_BYTE(lbs.serv_info.ci);
    pdata[(*idx)++] = BLOW_BYTE(lbs.serv_info.ci);
    
    pdata[(*idx)++] = lbs.serv_info.rxlev;// 信号强度

    
    if (lbs.nbr_cell_num > 6)
    {
        num = 6;
    }
    else
    {
        num = lbs.nbr_cell_num;
    }
    
    // 剩余6个基站 LAC(2B)+CI(3B)+RSSI(1B)
    lbsidx = (*idx);
    GM_memset(&pdata[lbsidx], 0, 36);
    for (k=0; k<num; k++) // 24B
    {
        pdata[lbsidx++] = BHIGH_BYTE(lbs.nbr_cell_info[k].lac);  // LAC
        pdata[lbsidx++] = BLOW_BYTE(lbs.nbr_cell_info[k].lac);
        
        pdata[lbsidx++] = 0x00;  // CELL ID
        pdata[lbsidx++] = BHIGH_BYTE(lbs.nbr_cell_info[k].ci);  // CELL ID
        pdata[lbsidx++] = BLOW_BYTE(lbs.nbr_cell_info[k].ci);
        
        pdata[lbsidx++] = lbs.nbr_cell_info[k].rxlev;// 信号强度
    }
    
    (*idx) = (*idx) + 36;   // 6个基站空间 每个6字节
}
#endif

void protocol_concox_pack_lbs_msg(u8 *pdata, u16 *idx, u16 len)
{
    GPSData gps_data;
    GM_memset(&gps_data, 0, sizeof(gps_data));
    gps_get_last_data(&gps_data);
    
    if(gps_data.gps_time == (time_t)0)
    {
        system_state_get_last_gps(&gps_data);
    }
    
    if(gps_data.gps_time == (time_t)0)
    {
        gps_data.gps_time = util_get_utc_time();
        gps_data.lat = agps_service_get_unfix_lat();
        gps_data.lng = agps_service_get_unfix_lng();
    }
    
    //立即上传一条gps数据
    protocol_concox_pack_gps_msg(GPS_MODE_POWER_UP, &gps_data, pdata, idx, len);
    return;

    #if 0
    protocol_concox_pack_head(pdata, idx, len, false); //4bytes  0x78 0x78 len(1) id(1)
    protocol_concox_pack_lbses(pdata, idx, len);  //51 bytes

    //原版本是心跳与LBS数据都报一条报警, 但由于这个版本中报警消息独立上报,这里就不再带alarm了
    protocol_concox_pack_alarm_language(pdata, idx, len, NULL); //2 bytes
    protocol_concox_pack_tail(pdata, idx, len);  //6bytes  serial(2) checksum(2) stop(2)
    
    //len not include 0x78 0x78 len(1) stop(2)
    protocol_concox_pack_id_len(pdata, PROTOCCOL_CONCOX_FULL_LBS, (*idx) - 5, false); 
    return;
    #endif 
}


void protocol_concox_pack_alarm_msg(AlarmInfo *alarm,u8 *pdata, u16 *idx, u16 len)
{
    GPSData gps_data;
    u8 app_ver = 0;
    
    protocol_concox_pack_head(pdata, idx, len, false); //4bytes  0x78 0x78 len(1) id(1)

    gps_get_last_data(&gps_data);
    if(gps_data.gps_time == (time_t)0)
    {
        system_state_get_last_gps(&gps_data);
    }
    
    if(gps_data.gps_time == (time_t)0)
    {
        gps_data.gps_time = util_get_utc_time();
        gps_data.lat = agps_service_get_unfix_lat();
        gps_data.lng = agps_service_get_unfix_lng();
    }
    
    protocol_concox_pack_gps(&gps_data, pdata, idx, len);  //18 bytes
    pdata[(*idx)++] = 8; // lbyte lbs length
    protocol_concox_pack_lbs(pdata, idx, len);  //8 bytes
    protocol_concox_pack_device_status(pdata, idx, len);  //1bytes
    protocol_concox_pack_voltage_status(pdata, idx, len);  //1bytes
    protocol_concox_pack_signal_status(pdata, idx, len);  //1bytes
    protocol_concox_pack_alarm_language(pdata, idx, len, alarm); //2 bytes
    config_service_get(CFG_PROTOCOL_VER, TYPE_BYTE, &app_ver, sizeof(app_ver));
    if(app_ver == PROTOCOL_VER_GT02)
    {
        protocol_concox_pack_tail(pdata, idx, len);  //6bytes  serial(2) checksum(2) stop(2)
        
        //len not include 0x78 0x78 len(1) stop(2)
        protocol_concox_pack_id_len(pdata, PROTOCCOL_CONCOX_ALARM, (*idx) - 5, false); 
        
    }
    else
    {
        protocol_concox_pack_tail(pdata, idx, len);  //6bytes  serial(2) checksum(2) stop(2)
        
        //len not include 0x78 0x78 len(1) stop(2)
        protocol_concox_pack_id_len(pdata, PROTOCCOL_CONCOX_ALARM_GT06, (*idx) - 5, false); 
    }
    
    log_service_print_hex((const char *) pdata, *idx);
    return;
}


void protocol_concox_parse_msg(u8 *pdata, u16 len)
{
    bool new = false;
    int cmd_idx = 3;
    
    if(PROTOCOL_HEADER_CONCOX == pdata[0] && PROTOCOL_HEADER_CONCOX == pdata[1])
    {
        new = false;
        cmd_idx = 3;
    }
    else if(PROTOCOL_HEADER_CONCOX_NEW == pdata[0] && PROTOCOL_HEADER_CONCOX_NEW == pdata[1])
    {
        new = true;
        cmd_idx = 4;
    }
    else
    {
        LOG(WARN,"protocol_concox_parse_msg assert(head(%02x %02x)) failed.", pdata[0],pdata[1]);
        return;
    }

    //协议号
    switch(pdata[cmd_idx])
    {
        case PROTOCCOL_CONCOX_LOGIN:
            LOG(DEBUG,"clock(%d) protocol_concox_parse_msg(PROTOCCOL_CONCOX_LOGIN).", util_clock());
            protocol_concox_parse_login_response(pdata,len,new);
            break;
        case PROTOCCOL_CONCOX_HEART:
            LOG(DEBUG,"clock(%d) protocol_concox_parse_msg(PROTOCCOL_CONCOX_HEART).", util_clock());
            gps_service_after_receive_heartbeat();
            break;
        case PROTOCCOL_CONCOX_HEART_EXPAND:
            LOG(DEBUG,"clock(%d) protocol_concox_parse_msg(PROTOCCOL_CONCOX_HEART_EXPAND).", util_clock());
            gps_service_after_receive_heartbeat();
            break;
        case PROTOCCOL_CONCOX_LOCAT:
            LOG(DEBUG,"clock(%d) protocol_concox_parse_msg(PROTOCCOL_CONCOX_LOCAT).", util_clock());
            gps_service_after_receive_heartbeat();
            break;
        case PROTOCCOL_CONCOX_LOCAT_GT06:
            LOG(DEBUG,"clock(%d) protocol_concox_parse_msg(PROTOCCOL_CONCOX_LOCAT_GT06).", util_clock());
            gps_service_after_receive_heartbeat();
            break;
        case PROTOCCOL_CONCOX_ALARM:
            LOG(DEBUG,"clock(%d) protocol_concox_parse_msg(PROTOCCOL_CONCOX_ALARM).", util_clock());
            break;
        case PROTOCCOL_CONCOX_ALARM_GT06:
            LOG(DEBUG,"clock(%d) protocol_concox_parse_msg(PROTOCCOL_CONCOX_ALARM_GT06).", util_clock());
            break;
        case PROTOCCOL_CONCOX_STATUS_MSG:
            LOG(DEBUG,"clock(%d) protocol_concox_parse_msg(PROTOCCOL_CONCOX_STATUS_MSG).", util_clock());
            break;
        case PROTOCCOL_CONCOX_REMOTE_MSG_REQ:
            LOG(DEBUG,"clock(%d) protocol_concox_parse_msg(PROTOCCOL_CONCOX_REMOTE_MSG_REQ).", util_clock());
            protocol_concox_parse_remote_msg(pdata,len,new);
            break;
        case PROTOCCOL_CONCOX_CH_LOCAT_ACK:
            LOG(DEBUG,"clock(%d) protocol_concox_parse_msg(PROTOCCOL_CONCOX_CH_LOCAT_ACK).", util_clock());
            protocol_concox_parse_ch_locate_ack(pdata,len,true);
            break;
        case PROTOCCOL_CONCOX_EN_LOCAT_ACK:
            LOG(DEBUG,"clock(%d) protocol_concox_parse_msg(PROTOCCOL_CONCOX_EN_LOCAT_ACK).", util_clock());
            protocol_concox_parse_en_locate_ack(pdata,len);
            break;
        default:
            LOG(WARN,"protocol_concox_parse_msg assert(msgid(%d)) failed.", pdata[cmd_idx]);
            break;
    }
    return;
}

static void protocol_concox_parse_remote_msg(U8* pdata, u16 len, bool new)
{
    U8* pOut = NULL;
    u16 out_len = 0;
    u8 idx = 0;
    u16 msg_len = 0;
    

    idx = 5;  //7878(2) len(1) proto_no(1) total_msglen(1) service_id(4)
    if(new)
    {
        idx +=1;  //len(2)
    }
    GM_memcpy(s_concox_msg_save.server_id, &pdata[idx], sizeof(s_concox_msg_save.server_id));  // 服务器标志  
    idx += 4;
    // total_msglen(1) service_id(4) content(total_msglen- 4), now idx point to start of the content 
    msg_len = pdata[idx - 5] - 4;
    
    if(pdata[idx - 5] + 11 > len)
    {
        LOG(WARN,"clock(%d) protocol_concox_parse_remote_msg assert(len(%d) <= %d) failed.", util_clock(),pdata[idx - 5],len);
        return;
    }
    
    pOut = GM_MemoryAlloc(CMD_MAX_LEN + 1);
    if (NULL == pOut)
    {
        LOG(WARN,"protocol_concox_parse_remote_msg assert(GM_MemoryAlloc(%d)) failed.", CMD_MAX_LEN + 1);
        return;
    }
    
    GM_memset(pOut, 0x00, CMD_MAX_LEN + 1);

    LOG(DEBUG,"clock(%d) protocol_concox_parse_remote_msg start:%d, len:%d.", util_clock(),idx,msg_len);
	command_on_receive_data(COMMAND_GPRS, (char *)&pdata[idx], msg_len, (char *)pOut,NULL);
    out_len = GM_strlen((char *)pOut);
    
    if (out_len > 0)
    {
        gps_service_after_receive_remote_msg(pOut, out_len);
    }

    GM_MemoryFree(pOut);
    return;
}



void protocol_concox_pack_remote_ack(u8 *pdata, u16 *idx, u16 len, u8 *pRet, u16 retlen)
{
    bool app_new = false;
    u16 language;
    
    protocol_concox_pack_head(pdata, idx, len, app_new); //4|5bytes  0x78 0x78 len(1|2) id(1)
    pdata[(*idx) ++ ] = retlen + 4;
    GM_memcpy(&pdata[(*idx)], s_concox_msg_save.server_id, sizeof(s_concox_msg_save.server_id));
    (*idx) += sizeof(s_concox_msg_save.server_id);
    GM_memcpy(&pdata[(*idx)], pRet, retlen);
    (*idx) += retlen;
    
    config_service_get(CFG_LANGUAGE, TYPE_SHORT, &language, sizeof(language));
    if (1 == language)
    {
        pdata[(*idx) ++ ] = 00;
        pdata[(*idx) ++ ] = 01; // 00 01简体中文
    }
    else
    {
        pdata[(*idx) ++ ] = 00;
        pdata[(*idx) ++ ] = 02; // 00 02英文
    }
    protocol_concox_pack_tail(pdata, idx, len);  //6bytes  serial(2) checksum(2) stop(2)

    //len not include 0x78 0x78 len(1) stop(2)
    protocol_concox_pack_id_len(pdata, PROTOCCOL_CONCOX_STRING_MSG, (*idx) - 5, app_new); 
}

static void protocol_concox_parse_login_response(U8* pdata, u16 len, bool new)
{
    // 0x78 0x78   0x05    0x01    0x00 0x01   0xD9 0xDC   0x0D 0x0A
    // 起始位   长度  协议号   序列号   错误校验    停止位
    
    if(len != 10)
    {
        LOG(WARN,"protocol_concox_parse_login_response assert(len(%d) != 7) failed.", len);
        return;
    }

    
    //发送iccid
    gps_service_after_login_response();
    
    return;
}

static void protocol_concox_parse_ch_locate_ack(U8* pdata, u16 len, bool ucs2)
{
    u16 idx = 0;
    u8 alarm_flag[10];
    u8 recv_msg[250];
    u16 recv_msg_w[125];
    u8 msg_len = 0;
    u8 sms_num[GM_ICCID_LEN + 1];

    GM_memset(alarm_flag, 0x00, sizeof(alarm_flag));
    GM_memset(recv_msg, 0x00, sizeof(recv_msg));
    GM_memset(sms_num, 0x00, sizeof(sms_num));
    
    idx += 9;

    if(len <= idx + 33)  // tail 6bytes.  && && ## 6bytes phone_number 21bytes
    {
        LOG(WARN,"clock(%d) protocol_concox_parse_ch_locate_ack assert(len(%d) > %d) failed.", util_clock(), len,idx + 33);
        return;
    }
    if (('#' != pdata[len - 8]) || ('#' != pdata[len - 7]))
    {   
        LOG(WARN,"clock(%d) protocol_concox_parse_ch_locate_ack assert(##) failed.", util_clock());
        return;
    }


    //协议文档中是8个字节，实际是变长
    for (msg_len=0; msg_len<(len-idx - 33); msg_len++)
    {
        if (('&' == pdata[idx+msg_len]) && ('&' == pdata[idx+msg_len+1]))
        {
            break;
        }
    }

    if (('&' != pdata[idx+msg_len]) || ('&' != pdata[idx+msg_len+1]) || msg_len >(sizeof(alarm_flag) - 1))
    {   
        LOG(WARN,"clock(%d) protocol_concox_parse_ch_locate_ack assert(content&&) failed. msg_len(%d).", util_clock(), msg_len);
        return;
    }

    GM_memcpy(alarm_flag, &pdata[idx], msg_len);
    alarm_flag[msg_len] = 0x00;
    idx += msg_len;
    
    idx += 2;
    for (msg_len=0; msg_len<(len-idx - 8); msg_len++)
    {
        if (('&' == pdata[idx+msg_len]) && ('&' == pdata[idx+msg_len+1]))
        {
            break;
        }
    }

    if (('&' != pdata[idx+msg_len]) || ('&' != pdata[idx+msg_len+1]) || msg_len >(sizeof(recv_msg) - 1))
    {   
        LOG(WARN,"clock(%d) protocol_concox_parse_ch_locate_ack assert(content&&) failed. msg_len(%d).", util_clock(), msg_len);
        return;
    }

    GM_memcpy(recv_msg, &pdata[idx], msg_len);
    recv_msg[msg_len] = 0x00;

    idx = idx + msg_len + 2;

    GM_memcpy(sms_num, &pdata[idx], sizeof(sms_num));
    sms_num[GM_ICCID_LEN] = 0;
    idx = idx + sizeof(sms_num);
    if (('#' != pdata[idx]) || ('#' != pdata[idx+1]))
    {   
        LOG(WARN,"clock(%d) protocol_concox_parse_ch_locate_ack assert(content##) failed.", util_clock());
        return;
    }

    ucs2 = true;  //acctually in english, it still use unicode.
    if(ucs2)
    {
        u16 index;

        //把UTF-8转成UNICODE
        index = util_utf8_to_unicode(recv_msg, msg_len, recv_msg_w, msg_len);    
        
        //把双字节的UNICODE转成单字节的UNICODE
        msg_len = util_ucs2_u16_to_u8(recv_msg_w, index,recv_msg );
    }
    

    if (0 == GM_memcmp("ALARMSMS", alarm_flag, GM_strlen((const char* )alarm_flag)))
    {
        LOG(DEBUG,"clock(%d) protocol_concox_parse_ch_locate_ack alarm_sms_ask msglen(%d).", util_clock(),msg_len);
        protocol_concox_alarm_sms_ask((u8 *)recv_msg_w, msg_len, ucs2);
    }
    else if (0 == GM_memcmp("ADDRESS", &alarm_flag, GM_strlen((const char* )&alarm_flag)))
    {
        LOG(DEBUG,"clock(%d) protocol_concox_parse_ch_locate_ack msglen(%d) sms_num(%s).", util_clock(),msg_len,sms_num);
        sms_send((char*)recv_msg_w, msg_len, (char*)sms_num, (ucs2?GM_UCS2_DCS:GM_8BIT_DCS));
    }
    else
    {
        LOG(WARN,"clock(%d) protocol_concox_parse_ch_locate_ack unknown alarm_flag(%s).", util_clock(),alarm_flag);
    }
}


static void protocol_concox_parse_en_locate_ack(U8* pdata, u16 len)
{
    protocol_concox_parse_ch_locate_ack(pdata, len, false);
}




static void protocol_concox_alarm_sms_ask(u8 *pdata, u16 len, bool ucs2)
{
    u8 number[GM_ICCID_LEN + 1];
    u16 number_len = 0;
    u16 index;
    u8 value_u8;
    
    if(len <=0)
    {
        LOG(DEBUG,"clock(%d) protocol_goome_alarm_ask msglen(%d).", util_clock(), len);
        return;
    }

    switch(s_concox_msg_save.last_alarm)
    {
        case ALARM_POWER_OFF:
        case ALARM_POWER_HIGH:
        case ALARM_BATTERY_LOW:
            config_service_get(CFG_POWER_ALARM_MODE, TYPE_BYTE, &value_u8, sizeof(value_u8));
            if(value_u8 != PWRALM_GPRS_SMS || value_u8 != PWRALM_GPRS_SMS_CALL)
            {
                return ;
            }
            break;
        case ALARM_SHOCK:
            config_service_get(CFG_SENSOR_ALARM_MODE, TYPE_BYTE, &value_u8, sizeof(value_u8));
            if(value_u8 != PWRALM_GPRS_SMS || value_u8 != PWRALM_GPRS_SMS_CALL)
            {
                return ;
            }
            break;
        default:
            return;
    }

    
    for (index=0; index<=3; index++)
    {
        number_len = config_service_get_sos_number(index, number, sizeof(number));
        number[number_len] = 0;
        if (number_len > 0)
        {
            sms_send((char*)pdata, len, (char*)number, (ucs2?GM_UCS2_DCS:GM_8BIT_DCS));
        }
        
        if (config_service_is_alarm_sms_to_one())
        {
            break;  //只发第一个号
        }
    }
    
}

void protocol_concox_pack_position_request_msg(u8 *mobile_num, u8 num_len, u8 *command, u8 cmd_len,u8 *pdata, u16 *idx, u16 len)
{
    GPSData gps;
    GM_memset(&gps,0,sizeof(gps));
    
    gps_get_last_data(&gps);
    if(gps.gps_time == (time_t)0)
    {
        system_state_get_last_gps(&gps);
    }
    
    if(gps.gps_time == (time_t)0)
    {
        gps.gps_time = util_get_utc_time();
        gps.lat = agps_service_get_unfix_lat();
        gps.lng = agps_service_get_unfix_lng();
    }

    protocol_concox_pack_head(pdata, idx, len, false); //4bytes  0x78 0x78 len(1) id(1)
    protocol_concox_pack_gps(&gps, pdata, idx, len);  //18 bytes
    
    
    GM_memset(&pdata[(*idx)],0,21);
    num_len = num_len < 21?num_len:20;
    GM_memcpy(&pdata[(*idx)],mobile_num,num_len);
    (*idx)+=21;   //21 bytes
    
    protocol_concox_pack_alarm_language(pdata, idx, len, NULL);  //2 bytes
    protocol_concox_pack_tail(pdata, idx, len);  //6bytes  serial(2) checksum(2) stop(2)
    
    //len not include 0x78 0x78 len(1) stop(2)
    protocol_concox_pack_id_len(pdata, PROTOCCOL_CONCOX_LOCAT_REQ, (*idx) - 5, false); 
}




