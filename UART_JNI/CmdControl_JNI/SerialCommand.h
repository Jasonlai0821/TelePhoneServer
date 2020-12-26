/*
 * SerialCommand.h
 *
 *  Created on: 2017-11-29
 *      Author: Administrator
 */

#ifndef SERIALCOMMAND_H_
#define SERIALCOMMAND_H_
#include <utils/RefBase.h>
//#include <utils/StrongPointer.h>
#include <utils/Mutex.h>
#include "MyUtils.h"
using namespace android;
namespace cmdcontrol {

#define COM0       0
#define COM1       1
#define COM2       2
#define COM3       3

#define INVALID_HANDLE_VALUE	-1
#define VOLUME_NUM 7
#define DEFAULT_COMM_PORT "/dev/ttyHSL1"

#define AUDIO_FIFO_C	"/data/vendor/audio_virtual/audio_fifo_c"
#define AUDIO_FIFO_P	"/data/vendor/audio_virtual/audio_fifo_p"

//
#define NOW_COM 	COM0

#define W_TIMEOUTMS	5		// millisecond
#define R_TIMEOUTMS	18		// millisecond
#define B_TIMEOUTMS	1000    //(millisecond)the maxinum of response time after 'write'
#define MAX_BUF_LEN 256
#define BUF_LEN_16	16

#define MAXlength_serwritedata	MAX_WRITE_BUF_LEN
#define MINlength_serwritedata	4

#define			BC_LEADCODE		0
#define			BC_LEHGTH		1
#define 		BC_CMD			2
#define 		BC_CMD_TYPE		3


#define 		M_LCHEAD		0x5A
#define 		S_LCHEAD		0x5B

#define 		REM_ACTIVE_RING				0x00
#define 		REM_ACTIVE_CALLID			0x01
#define 		REM_ACTIVE_BUSY				0x02
#define 		REM_ACTIVE_AUDIO			0x03
#define 		REM_ACTIVE_CANCEL			0x09

#define 		TERMINAL_ACTIVE_RING		0x04
#define 		TERMINAL_ACTIVE_CALLID		0x05
#define 		TERMINAL_ACTIVE_PARTY_NUM	0x06
#define 		TERMINAL_ACTIVE_AUDIO		0x07
#define 		TERMINAL_ON_HOOK			0x08
//#define 		TERMINAL_ON_HOOK			0x09

#define         START_COMMUNICATION			0xFE
#define			STOP_COMMUNICATION			0xFF

#define			UART_PPACK_LENGTH  			165
#define			PCM_PPACK_LENGTH  			160

#define 	 	PCM_ACCEPT_ERROR_LEN		7

#define 		MAX_PACKET_LEN 		(UART_PPACK_LENGTH * 50)
#define 		PIPE_MAX_BUF_LEN 	(MAX_BUF_LEN * 2)


/*0: running ,1: need to exit, 2: exited */

#define 		THREAD_IDLE				0
#define 		THREAD_NEED_EXIT		1
#define 		THREAD_EXIT				2
#define			THREAD_WORKING			3


typedef enum {
	SERCMD_unknown = 0,
	SERCMD_md_setparameter = 0x0A,
	SERCMD_md_getparameter = 0x0B,
	SERCMD_sd_response	   = 0x0C
} ser_cmd_e;


typedef struct {
	ser_cmd_e ser_cmd_type;
	unsigned char ser_cmd_data;
	int value;
} ser_data_s;

typedef struct{
	unsigned char pcm_buf[PIPE_MAX_BUF_LEN];
	bool is_empty;
	int len;
}pcm_pipedata;

class SerialCommand: virtual public RefBase {
public:
	SerialCommand();
	virtual ~SerialCommand();
	int iniSer();
	int deInitSer();
	int onExecuteCmd(const char * cmd);
	int onProcessPCMData(unsigned char *pcmData,int len);
	void setListener(const sp<CmdControlListener>& listener);
	sp<CmdControlListener> getListener() {
		return mListener;
	}

	//static function as tools
	static int SerOpen(int PortNo);

	static int SerWrite_SlaveDevice(int PortNo, const char *w_pBuf,unsigned long TimeOut,int len);
	
	static void SerClear(int PortNo);

	static int SerWrite(int PortNo, unsigned char *pszBuf, unsigned int SendCnt,
			unsigned long TimeOut);

	static void SerClose(int PortNo);
	
	static int SerRead(int PortNo, unsigned char *pszBuf, int ReadCnt,
			unsigned long TimeOutMS);
	
	static int CheckIsValid(const unsigned char *w_pBuf,int len);

	static int StringtoHex(const char *w_pBuf,int len,unsigned char* des_Buf);
	static int HextoString(unsigned char *w_pBuf,int len, char* des_Buf);
	static int getCRCValue(unsigned char* des_Buf,int len);

	static int SerParse_Reponse(const unsigned char *r_pBuf,int len,ser_data_s *pBuf);

	static int getCRC16(unsigned char *pszBuf, int ReadCnt);
	
	static sp<SerialCommand> getInstance();
private:
	SerialCommand(const SerialCommand&);
	SerialCommand& operator=(const SerialCommand&);
	static SerialCommand* mInstance;
	sp<CmdControlListener> mListener;
	bool mSerInitialized;
};

}

#endif /* SERIALCOMMAND_H_ */
