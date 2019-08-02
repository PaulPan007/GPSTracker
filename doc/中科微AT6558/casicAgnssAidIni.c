#include <string.h>
#include <stdio.h>
#include <casicAgnssAidIni.h>

/************************************************************
�������ƣ�isLeapYear
�������ܣ������жϡ��жϹ�������һ�򣬰��겻���İ�������
�������룺year�����ж����
���������1, ���꣬0�������꣨ƽ�꣩
************************************************************/
int isLeapYear(int year)
{	
	if ((year & 0x3) != 0)
	{						// ���year����4�ı�����һ����ƽ��
		return 0;
	}	
	else if ((year % 400) == 0)
	{						// year��400�ı���
		return 1;
	}	
	else if ((year % 100) == 0)
	{						// year��100�ı���
		return 0;
	}	
	else
	{						// year��4�ı���
		return 1;
	}
}
/*************************************************************************
�������ƣ�	gregorian2SvTime
�������ܣ�	ʱ���ʽת��, ��Ҫ����UTC��������
			�����ʱ���ʽ�ǳ����������ʱ�����ʽ��ʱ�䣻
			ת�����ʱ���ʽ��GPSʱ���ʽ��������������ʱ��ʾ��GPS��ʱ�������1980.1.6
			GPSʱ��û��������������������ʱ�䣬������ʱ���Ǿ�������������
			2016�����������ֵ��17��
�������룺	pDateTime,	�ṹ��ָ�룬������ʱ�����ʽ��ʱ��
���������	pAidIni,	�ṹ��ָ�룬����������ʱ����������������ʱ��ʱ���ʽ
*************************************************************************/
void gregorian2SvTime(DATETIME_STR *pDateTime, AID_INI_STR *pAidIni)
{	
	int DayMonthTable[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
	int i, dn, wn;
	double tod, tow;

	// ����ʱ��
	tod = pDateTime->hour * 3600 + pDateTime->minute * 60 + pDateTime->second + pDateTime->ms / 1000.0;
	
	// �ο�ʱ��: 1980.1.6
	dn = pDateTime->day;
	// ��->��
	for (i = 1980; i < (pDateTime->year); i++)
	{
		if (isLeapYear(i))
		{
			dn += 366;
		}			
		else
		{
			dn += 365;
		}
	}
	dn -= 6;
	// ��->��
	if (isLeapYear(pDateTime->year))
	{
		DayMonthTable[1] = 29;
	}		
	for (i = 1; i < pDateTime->month; i++)
	{
		dn += DayMonthTable[i-1];
	}		

	// ����+����ʱ��
	wn	= (dn / 7);						// ����
	tow = (dn % 7) * 86400 + tod + 17;	// ����ʱ�䣬��������

	if (tow >= 604800)
	{
		wn++;
		tow -= 604800;
	}

	pAidIni->wn 	= wn;
	pAidIni->tow 	= tow;
}
/*************************************************************************
�������ƣ�casicAgnssAidIni
�������ܣ��Ѹ���λ�ú͸���ʱ������ר�õ����ݸ�ʽ����������Ϣ��ʽ���������
�������룺dateTime��������ʱ�䣬������Ч��־��1��Ч��
		  lla, ��γ�ȱ�־��������Ч��־��1��Ч��
���������aidIniMsg[66]���ַ����飬������Ϣ���ݰ������ȹ̶�
*************************************************************************/
void casicAgnssAidIni(DATETIME_STR dateTime, POS_LLA_STR lla, char aidIniMsg[66])
{
	AID_INI_STR aidIni;
	int ckSum, i;
	int *pDataBuff = (int*)&aidIni;

	gregorian2SvTime(&dateTime, &aidIni);
	
	aidIni.df			= 0;
	aidIni.xOrLat		= lla.lat;
	aidIni.yOrLon		= lla.lon;
	aidIni.zOrAlt		= lla.alt;
	aidIni.fAcc			= 0;
	aidIni.posAcc		= 0;
	aidIni.tAcc			= 0;
	aidIni.timeSource	= 0;
	
	aidIni.flags 		= 0x20;											// λ�ø�ʽ��LLA��ʽ���߶���Ч��Ƶ�ʺ�λ�þ��ȹ�����Ч
	aidIni.flags 		= aidIni.flags | ((lla.valid 	  == 1) << 0); 	// BIT0��λ����Ч��־
	aidIni.flags 		= aidIni.flags | ((dateTime.valid == 1) << 1); 	// BIT1��ʱ����Ч��־

	// �������ݴ��
	ckSum = 0x010B0038;//���ֽڱ������ڴ�ĵ͵�ַ�У���С��ģʽ
	for (i = 0; i < 14; i++)
	{
		ckSum += pDataBuff[i];
	}
	
	aidIniMsg[0] = 0xBA;
	aidIniMsg[1] = 0xCE;
	aidIniMsg[2] = 0x38;		// LENGTH
	aidIniMsg[3] = 0x00;
	aidIniMsg[4] = 0x0B;		// CLASS	ID
	aidIniMsg[5] = 0x01;		// MESSAGE	ID
	
	memcpy(&aidIniMsg[6],  (char*)(&aidIni), 56);
	memcpy(&aidIniMsg[62], (char*)(&ckSum),  4);
	
	return;
}