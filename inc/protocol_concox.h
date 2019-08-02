/**
 * Copyright @ Goome Technologies Co., Ltd. 2009-2019. All rights reserved.
 * File name:        protocol_concox.h
 * Author:           梁震       
 * Version:          1.0
 * Date:             2019-03-01
 * Description:      
 * Others:      
 * Function List:    
    1. 创建protocol_concox模块
    2. 销毁protocol_concox模块
    3. protocol_concox模块定时处理入口
 * History: 
    1. Date:         2019-03-01
       Author:       梁震
       Modification: 创建初始版本
    2. Date: 		 
	   Author:		 
	   Modification: 

 */

#ifndef __PROTOCOL_CONCOX_H__
#define __PROTOCOL_CONCOX_H__

#include "gm_type.h"
#include "error_code.h"

#include "gps_service.h"

/*plen 是输入输出参数*/
void protocol_concox_pack_login_msg(u8 *pdata, u16 *idx, u16 len);
void protocol_concox_pack_device_status_msg(u8 *pdata, u16 *idx, u16 len);
void protocol_concox_pack_heartbeat_msg(u8 *pdata, u16 *idx, u16 len);
void protocol_concox_pack_remote_ack(u8 *pdata, u16 *idx, u16 len, u8 *pRet, u16 retlen);


struct GPSData;
void protocol_concox_pack_gps_msg(GpsDataModeEnum mode, const GPSData *gps, u8 *pdata, u16 *idx, u16 len);
void protocol_concox_pack_lbs_msg(u8 *pdata, u16 *idx, u16 len);
void protocol_concox_pack_alarm_msg(AlarmInfo *alarm,u8 *pdata, u16 *idx, u16 len);
void protocol_concox_pack_position_request_msg(u8 *mobile_num, u8 num_len, u8 *command, u8 cmd_len,u8 *pdata, u16 *idx, u16 len);

void protocol_concox_parse_msg(u8 *pdata, u16 len);



#endif



