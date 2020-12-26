/*
 * SerialCommand.cpp
 *
 *  Created on: 2017-12-5
 *      Author: Administrator
 */
#define LOG_TAG "SerialCommand"

#include <sys/ioctl.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <syslog.h>
#include <signal.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <sched.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <sys/time.h>
#include <errno.h>
#include <semaphore.h>
#include <sys/stat.h>
#include <utils/StrongPointer.h>
#include "SerialCommand.h"
#include "MyUtils.h"
namespace cmdcontrol {

#define PCM_FILE_DEBUG 1

#ifdef PCM_FILE_DEBUG
static int pcm_8bitfile_c = 0;
static int pcm_16bitfile_c = 0;
static int pcm_8bitfile_p = 0;
static int pcm_16bitfile_p = 0;

static const char *pcm8bfilename_c  = "/data/audio_capture_8bit.pcm";
static const char *pcm16bfilename_c  = "/data/audio_capture_16bit.pcm";
static const char *pcm8bfilename_p  = "/data/audio_playback_8bit.pcm";
static const char *pcm16bfilename_p  = "/data/audio_playback_16bit.pcm";
#endif

static int m_serial_handle;
//static sem_t m_singal;
static pthread_mutex_t m_mutex;
static pthread_cond_t m_cond;
static char CallidNumber[BUF_LEN_16] = {0};
static ser_cmd_e Now_cmd_e;
static ser_data_s S_Read;
static unsigned char FLAG_READ_THREAD = THREAD_IDLE; /*0: running ,1: need to exit, 2: exited */
static unsigned char FLAG_WRITE_THREAD = THREAD_IDLE; /*0: running ,1: need to exit, 2: exited*/
static unsigned char FLAG_CAPTURE_THREAD = THREAD_IDLE; /*0: running ,1: need to exit, 2: exited*/
static unsigned char FLAG_PLAYBACK_THREAD = THREAD_IDLE; /*0: running ,1: need to exit, 2: exited*/

static int audio_fifo_capture =-1;
static int audio_fifo_playback = -1;

static pthread_t com_read_tid = (pthread_t)0L;
static pthread_t com_write_tid = (pthread_t)0L;


static pthread_t capture_tid = (pthread_t)0L;
static pthread_t playback_tid = (pthread_t)0L;
static pcm_pipedata capture_buf = {{0},true,0};
static pcm_pipedata playback_buf = {{0},true,0};

SerialCommand* SerialCommand::mInstance = NULL;

static unsigned short const wCRC16Table[MAX_BUF_LEN] = {      
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,     
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,     
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,      
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,     
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,       
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,     
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,     
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,     
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,     
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,        
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,     
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,     
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,     
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,     
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,        
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,     
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,     
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,     
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,     
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,        
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,     
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,     
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,     
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,     
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,       
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,     
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,     
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,     
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,     
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,       
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,     
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

SerialCommand::SerialCommand() :
		mListener(NULL) {
	LOGD("SerialCommand Constructor");
	mInstance = this;
	mSerInitialized = false;
}

SerialCommand::~SerialCommand() {
	LOGD("SerialCommand Destructor");
	mInstance = NULL;
	mListener = NULL;
	mSerInitialized = false;
}

sp<SerialCommand> SerialCommand::getInstance() {
	if (mInstance != NULL) {
		return mInstance;
	}
	return NULL;
}

void SerialCommand::setListener(const sp<CmdControlListener>& listener) {
	mListener = listener;
}

static int tty_dev_set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios newtio,oldtio;
	if(tcgetattr( fd,&oldtio)  !=  0){
	 	printf("tty_dev_set_opt() fd =%d set option failed!\n",fd);
	  	perror("tty_dev_set_opt() error\n");
	  	return -1;
	}
	bzero(&newtio, sizeof( newtio ));
	oldtio.c_cflag  |=  CLOCAL | CREAD; //设置控制模式状态，本地连接，接收使能
	oldtio.c_cflag &= ~CSIZE; ;//字符长度，设置数据位之前一定要屏掉这个位
	oldtio.c_cflag &= ~CRTSCTS;//无硬件流控
	oldtio.c_lflag = 0; //不激活终端模式 
	 
	 switch( nBits )
	 {
	 case 7:
	  	oldtio.c_cflag |= CS7;
	  	break;
	 case 8:
	  	oldtio.c_cflag |= CS8;
	  	break;
	 }
 
	 switch( nEvent )
	 {
	 case 'O':
	  	oldtio.c_cflag |= PARENB; //允许输出产生奇偶信息以及输入到奇偶校验
	  	oldtio.c_cflag |= PARODD;  //输入和输出是奇及校验
	  	oldtio.c_iflag |= (INPCK | ISTRIP); // INPACK:启用输入奇偶检测；ISTRIP：去掉第八位
	  	break;
	 case 'E':
	  	oldtio.c_iflag |= (INPCK | ISTRIP);
	  	oldtio.c_cflag |= PARENB;
	  	oldtio.c_cflag &= ~PARODD;
	  	break;
	 case 'N': 
	  	oldtio.c_cflag &= ~PARENB;
		oldtio.c_iflag &= ~(IXON | IXOFF | IXANY);
	 	oldtio.c_iflag &= ~(INLCR | ICRNL | IGNCR);
		oldtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
		oldtio.c_oflag &= ~(OPOST); // 设置输出标志：不执行输出处理
	  	break;
	 default:
	 	oldtio.c_iflag |= IGNPAR;//无奇偶检验位 
		break;
	 }
 
	 switch( nSpeed )
	 {
	 case 2400:
	  	cfsetispeed(&oldtio, B2400);
	  	cfsetospeed(&oldtio, B2400);
	  	break;
	 case 4800:
	  	cfsetispeed(&oldtio, B4800);
	  	cfsetospeed(&oldtio, B4800);
	  	break;
	 case 9600:
	  	cfsetispeed(&oldtio, B9600);
	  	cfsetospeed(&oldtio, B9600);
	  	break;
	 case 115200:
	  	cfsetispeed(&oldtio, B115200);
	  	cfsetospeed(&oldtio, B115200);
	  	break;
	 case 460800:
	  	cfsetispeed(&oldtio, B460800);
	  	cfsetospeed(&oldtio, B460800);
	  	break;
	 default:
	  	cfsetispeed(&oldtio, B9600);
	  	cfsetospeed(&oldtio, B9600);
	  	break;
	 }
 
	 if( nStop == 1 )
	  	oldtio.c_cflag &=  ~CSTOPB; //CSTOPB:设置两个停止位，而不是一个
	 else if ( nStop == 2 )
	 	oldtio.c_cflag |=  CSTOPB;

	 //oldtio.c_oflag = 0; //输出模式 
	 oldtio.c_cc[VTIME]  = 0; //VTIME:非cannoical模式读时的延时，以十分之一秒位单位
	 oldtio.c_cc[VMIN] = 0; //VMIN:非canonical模式读到最小字符数
	 tcflush(fd,TCIFLUSH); // 改变在所有写入 fd 引用的对象的输出都被传输后生效，所有已接受但未读入的输入都在改变发生前丢弃。
	 if((tcsetattr(fd,TCSANOW,&oldtio))!=0) //TCSANOW:改变立即发生
	 {
		printf("tty_dev_set_opt() fd =%d set option failed!\n",fd);
	  	return -1;
	 }
	 printf("tty_dev_set_opt() fd =%d set option success!\n",fd);
	 
	 return 0;
}


int SerialCommand::SerOpen(int PortNo) {
	int ret;
	struct termios options;

	// m_serial_handle = open(DEFAULT_COMM_PORT, O_RDWR|O_NOCTTY|O_NDELAY);
	m_serial_handle = open(DEFAULT_COMM_PORT, O_RDWR);	//|O_NOCTTY|O_NDELAY);
	if (m_serial_handle == INVALID_HANDLE_VALUE) {
		LOGE("Open file err:%s errno:%d", DEFAULT_COMM_PORT, errno);
		return -1;
	}

	//tty_dev_set_opt(m_serial_handle,460800,8,'N',1);
//	
	tcgetattr(m_serial_handle, &options);

	bzero(&options,sizeof(struct termios));
	options.c_lflag = 0;
	options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
	options.c_iflag &= ~(IXON | IXOFF | IXANY);
	options.c_oflag &= ~(ONLCR | OCRNL);
	options.c_cc[VTIME] = 0;
	options.c_cc[VMIN] = 0;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //rockchip add
	
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_cflag |= (CLOCAL | CREAD); //rockchip modify

	options.c_cflag &= ~PARENB; 
	options.c_iflag &= ~INPCK;
	options.c_cflag &= ~CSTOPB;

	//cfsetispeed(&options, B115200);
	//cfsetospeed(&options, B115200);
	cfsetispeed(&options, B460800);
	cfsetospeed(&options, B460800);
	ret = tcsetattr(m_serial_handle, TCSANOW, &options);
	if (ret == -1) {
		LOGE("SetArrt tcsetattr err: %d", errno);
		return -1;
	}
		
	return 0;
}

void SerialCommand::SerClose(int PortNo) {
	if (m_serial_handle != INVALID_HANDLE_VALUE) {
		close(m_serial_handle);
	}
	m_serial_handle = INVALID_HANDLE_VALUE;
}

void SerialCommand::SerClear(int PortNo) {
	LOGV("SerClear: clear the data of uart....");
	tcflush(m_serial_handle, TCIOFLUSH);
	LOGV("SerClear: clear end....");
}

int SerialCommand::SerWrite(int PortNo, unsigned char *pszBuf,
		unsigned int SendCnt, unsigned long TimeOutMS) {
	int ret = 0;
	struct timeval CurTime;
	fd_set wwrite;

	FD_ZERO(&wwrite);
	FD_SET(m_serial_handle, &wwrite);

	CurTime.tv_sec = (TimeOutMS / 1000);
	CurTime.tv_usec = (TimeOutMS % 1000) * 1000;

	ret = select(m_serial_handle + 1, NULL, &wwrite, NULL, &CurTime);
	if (ret == 0) {
		LOGE("SerWrite select timeout");
		return -1;
	} else if (ret < 0) {
		LOGE("SerWrite select error: %d", errno);
		return -1;
	}

	ret = write(m_serial_handle, pszBuf, SendCnt);
	if (ret <= 0) {
		SerialCommand::SerClear(NOW_COM);
		LOGE("SerWrite write buffer error:%d ", errno);
		return -1;
	}
	return 0;
}

int SerialCommand::SerRead(int PortNo, unsigned char *pszBuf, int ReadCnt,
		unsigned long TimeOutMS) {
	int ret = 0;
	int index = 0;
	struct timeval CurTime;
	fd_set wread;

	FD_ZERO(&wread);
	FD_SET(m_serial_handle, &wread);

	CurTime.tv_sec = (TimeOutMS / 1000);
	CurTime.tv_usec = (TimeOutMS % 1000) * 1000;

	ret = select(m_serial_handle + 1, &wread, NULL, NULL, &CurTime);
	if (ret == 0) {
		LOGE("SerRead select timeout");
		return -1;
	} else if (ret < 0) {
		LOGE("SerRead select error: %d", errno);
		return -1;
	}

	ret = read(m_serial_handle, pszBuf, ReadCnt);
	if (ret <= 0) {
		LOGE("SerRead read buffer error:%d ", errno);
		return -1;
	}
	return ret;
}

int SerialCommand::getCRCValue(unsigned char* des_Buf,int len)
{
	int sum = 0;
	int crc = 0;
	for(int i = 0; i < len; i++){
		sum += des_Buf[i];
	}

	crc = sum & 0xff;
	LOGD("getCRCValue() crc:0x%x", crc);
	return crc;
}

int SerialCommand::getCRC16(unsigned char *pszBuf, int ReadCnt) {
	unsigned short wResult = 0;     
    unsigned short wTableNo = 0;    

     int i = 0;
    for( i = 0; i < ReadCnt; i++)     
    {     
        wTableNo = ((wResult & 0xff) ^ (pszBuf[i] & 0xff));     
        wResult = ((wResult >> 8) & 0xff) ^ wCRC16Table[wTableNo];     
    }     
    
    //*pCRCOut = wResult;  
	return wResult;
}		

static void Clearser_data_s(ser_data_s *pBuf)
{
	pBuf->ser_cmd_type = SERCMD_unknown;
	pBuf->ser_cmd_data = 0;
	pBuf->value = 0;
}

static int extractPcmdata(pcm_pipedata *pPcmdata,unsigned char *w_pBuf)
{
	int len =0;
	//LOGV("extractPcmdata() IN");
	pthread_mutex_lock(&m_mutex);
	memcpy(w_pBuf,pPcmdata->pcm_buf,pPcmdata->len);
	len = pPcmdata->len;
	pPcmdata->is_empty = true;
	pPcmdata->len = 0;
	memset(pPcmdata->pcm_buf,0x00,pPcmdata->len);
	pthread_mutex_unlock(&m_mutex);
	return len;
}

static int fillPcmdata(pcm_pipedata *pPcmdata,const unsigned char *w_pBuf,int len)
{
	//LOGV("fillPcmdata() IN");
	pthread_mutex_lock(&m_mutex);
	memcpy(pPcmdata->pcm_buf,w_pBuf,len);
	pPcmdata->is_empty = false;
	pPcmdata->len = len;
	pthread_mutex_unlock(&m_mutex);
	//LOGV("fillPcmdata() OUT");
	return len;
}

static int cleanPcmdata(pcm_pipedata *pPcmdata)
{
	//LOGV("cleanPcmdata() IN");
	pthread_mutex_lock(&m_mutex);
	memset(pPcmdata->pcm_buf,0x00,PIPE_MAX_BUF_LEN);
	pPcmdata->is_empty = true;
	pPcmdata->len = 0;
	pthread_mutex_unlock(&m_mutex);
	return 0;
}


int SerialCommand::StringtoHex(const char *w_pBuf,int len,unsigned char* des_Buf)
{
	int ret = -1;
	int i =0;
	int fu,su;
	for(i = 0; i < len; i= i +2){
		if(w_pBuf[i] >= '0' && w_pBuf[i] <= '9'){
			fu = w_pBuf[i] - '0';
		}else if(w_pBuf[i] >= 'A' && w_pBuf[i] <= 'F'){
			fu = w_pBuf[i] - 'A' + 10;
		}else if(w_pBuf[i] >= 'a' && w_pBuf[i] <= 'f'){
			fu = w_pBuf[i] - 'a' + 10;
		}

		if(w_pBuf[i+1] >= '0' && w_pBuf[i+1] <= '9'){
			su = w_pBuf[i+1] - '0';
		}else if(w_pBuf[i+1] >= 'A' && w_pBuf[i+1] <= 'F'){
			su = w_pBuf[i+1] - 'A' + 10;
		}else if(w_pBuf[i+1] >= 'a' && w_pBuf[i+1] <= 'f'){
			su = w_pBuf[i+1] - 'a' + 10;
		}
		des_Buf[i/2] =fu * 16 + su;
		//LOGE("StringtoHex() fu:0x%x, su:0x%x", fu,su);
	}

	//for(i = 0; i < len / 2; i++){
	//	LOGE("StringtoHex() desBuf:0x%x, i:%d", des_Buf[i],i);
	//}

	des_Buf[len/2] = getCRC16(des_Buf,len/2);
	return ret;
}
int SerialCommand::HextoString(unsigned char *w_pBuf,int len, char* des_Buf)
{
	int ret = -1;
	int i =0;
	int fu,su;
	for(i = 0; i < len; i++){
		if(w_pBuf[i] >= 0 && w_pBuf[i] <= 9){
			des_Buf[i] = w_pBuf[i] + '0';
		}

		//LOGE("StringtoHex() fu:0x%x, su:0x%x", fu,su);
	}
	//LOGE("des_Buf():%s\n",des_Buf);
	// for(i = 0; i < len; i++){
	// 	LOGE("StringtoHex() desBuf:0x%x, i:%d", des_Buf[i],i);
	// }

	return ret;
}

static void setThreadReadState(int value)
{
	FLAG_READ_THREAD = value;
}

static void setThreadWriteState(int value)
{
	FLAG_WRITE_THREAD = value;
}

static void setThreadCaptureState(int value)
{
	FLAG_CAPTURE_THREAD = value;
}

static void setThreadPlaybackState(int value)
{
	FLAG_PLAYBACK_THREAD = value;
}

static void StartCommunication()
{
	LOGV("StartCommunication()");

	setThreadWriteState(THREAD_WORKING);
	//setThreadPlaybackState(THREAD_WORKING);
	//setThreadCaptureState(THREAD_WORKING);
}


static void StopCommunication()
{
	LOGV("StopCommunication()");

	setThreadWriteState(THREAD_IDLE);
	//setThreadPlaybackState(THREAD_IDLE);
	//setThreadCaptureState(THREAD_IDLE);
}



int SerialCommand::CheckIsValid(const unsigned char * w_pBuf, int len)
{
	unsigned char cmd_temp[MAX_BUF_LEN *4] = {0};
	if(len > MAX_BUF_LEN *4){
		len = MAX_BUF_LEN *4;
	}
	memcpy(cmd_temp,w_pBuf,len);
	int ret = -1;

	if(cmd_temp[BC_CMD] == START_COMMUNICATION){
		StartCommunication();
	}else if(cmd_temp[BC_CMD] == STOP_COMMUNICATION){
		StopCommunication();
	}

	if(M_LCHEAD == cmd_temp[BC_LEADCODE] || S_LCHEAD == cmd_temp[BC_LEADCODE]){
		if(cmd_temp[BC_CMD] >= REM_ACTIVE_RING &&  cmd_temp[BC_CMD] <= REM_ACTIVE_CANCEL){//注意检测最后一个指令值大小，替换TERMINAL_ON_HOOK
			//添加crc校验函数
			ret = 0;
		}else{
			ret = -1;
		}
	}else{
		ret = -1;
	}

	return ret;
}

static int openin_readAudioPipe()//playback for stm32 read from audio cards.
{
	int fd_fifo;
	//LOGV("openin_readAudioPipe() IN");
	#ifdef PCM_FILE_DEBUG
	if(pcm_8bitfile_p == 0){
		pcm_8bitfile_p = open(pcm8bfilename_p,O_RDWR);
	}
	if(pcm_16bitfile_p == 0){
		pcm_16bitfile_p = open(pcm16bfilename_p,O_RDWR);
	}
	
	#endif
	
	fd_fifo = open(AUDIO_FIFO_P,O_RDONLY | O_NONBLOCK);
	if(fd_fifo < 0){
		LOGE("openin_readAudioPipe() pipe %s: open failed,errno=%d", AUDIO_FIFO_P,errno);
		return -1;
	}else{
		return fd_fifo;
	}
}


static int openout_writeAudioPipe()//recorder for stm32 write to audio cards.
{
	int fd_fifo;
	//LOGV("openout_writeAudioPipe() IN");

	#ifdef PCM_FILE_DEBUG
	if(pcm_8bitfile_c == 0){
		pcm_8bitfile_c = open(pcm8bfilename_c,O_RDWR);
	}

	if(pcm_16bitfile_c == 0){
		pcm_16bitfile_c = open(pcm16bfilename_c,O_RDWR);
	}
	
	#endif
	
	fd_fifo = open(AUDIO_FIFO_C,O_WRONLY | O_NONBLOCK);
	if(fd_fifo < 0){
		LOGE("openout_writeAudioPipe() pipe %s: open failed,errno=%d", AUDIO_FIFO_C,errno);
		return -1;
	}else{
		return fd_fifo;
	}
}

static int closeAudioPipe()
{
	LOGV("closeAudioPipe() IN");

	#ifdef PCM_FILE_DEBUG
	close(pcm_8bitfile_c);
	close(pcm_16bitfile_c);
	pcm_8bitfile_c = 0;
	pcm_16bitfile_c = 0;

	close(pcm_8bitfile_p);
	close(pcm_16bitfile_p);
	pcm_8bitfile_p = 0;
	pcm_16bitfile_p = 0;
	#endif

	if(audio_fifo_capture >= 0){
		close(audio_fifo_capture);
	}

	if(audio_fifo_playback >= 0){
		close(audio_fifo_playback);
	}
	return 0;
}

static int out_writeAudioPipe(const unsigned char *pszBuf,int len)
{
	unsigned char cmd_temp[PIPE_MAX_BUF_LEN] = {0};
	memcpy(cmd_temp,pszBuf,len);
	int ret = -1;

	if(audio_fifo_capture >= 0){
		pthread_mutex_lock(&m_mutex);
		ret = write(audio_fifo_capture,cmd_temp,len);
		pthread_mutex_unlock(&m_mutex);
		if(ret < 0){
			LOGV("out_writeAudioPipe() failed errno = %d",errno);
		}
	}else{
		audio_fifo_capture= openout_writeAudioPipe();
	}
	//LOGV("out_writeAudioPipe() ret =%d",ret);
	return ret;
}

static int in_readAudioPipe(unsigned char *pszBuf,int len)
{
	unsigned char cmd_temp[PIPE_MAX_BUF_LEN] = {0};
	int ret = -1;

	if(audio_fifo_playback >= 0){
		pthread_mutex_lock(&m_mutex);
		ret = read(audio_fifo_playback,cmd_temp,len);
		pthread_mutex_unlock(&m_mutex);
		if(ret > 0){
			memcpy(pszBuf,cmd_temp,ret);
		}else if(ret < 0){
			LOGV("in_readAudioPipe() failed errno = %d",errno);
		}
	}else{
		audio_fifo_playback = openin_readAudioPipe();
	}
	//LOGV("in_readAudioPipe() ret =%d",ret);
	return ret;
}

static void ParserPhoneNumber(const unsigned char *r_pBuf,int len)
{
	char temp[BUF_LEN_16] = {0};

	if(len > BUF_LEN_16 / 2){
		len = BUF_LEN_16 / 2;
	}
	
	for(int i = 0;i< len;i++)
	{
		temp[2*i] = (r_pBuf[i]>>4)&0x0f;
		temp[2*i+1] = (r_pBuf[i])&0x0f;
	}

	for(int i = 0; i < 2*len; i++){
		CallidNumber[i] = temp[i] + '0';
	}
	LOGV("ParserPhoneNumber() CallidNumber:%s\n",CallidNumber);
}

int SerialCommand::SerWrite_SlaveDevice(int PortNo, const char *w_pBuf,unsigned long TimeOut,int len) {
	unsigned char buffer[MAX_BUF_LEN *2] = {0};
	int ret;
	int b_len = 0;

	//LOGE("SerWrite_SlaveDevice() CMD len:%d",len);	
	StringtoHex(w_pBuf,len,buffer);

	b_len = (len / 2) + 1; //1:just for crc value
	if(CheckIsValid(buffer,b_len) < 0){
		LOGE("SerWrite_SlaveDevice() CMD is invalid!!!");
		return -1;
	}

	//for(int i = 0; i <b_len; i++){
	//	LOGE("StringtoHex() buffer:0x%x, i:%d", buffer[i],i);
	//}

	if(buffer[BC_CMD] == TERMINAL_ON_HOOK || buffer[BC_CMD] == TERMINAL_ACTIVE_RING)
	{
		StopCommunication();
		usleep(20000);
	}
	
	ret = SerWrite(PortNo, buffer,b_len, TimeOut);
	if(buffer[BC_CMD] == TERMINAL_ACTIVE_RING)
	{
		StartCommunication();
	}


	if (ret < 0) {
		LOGE("SerWrite err");
		return -1;
	} else {
		if(buffer[BC_CMD_TYPE] == SERCMD_md_getparameter || buffer[BC_CMD_TYPE] == SERCMD_md_setparameter){
			return buffer[BC_CMD_TYPE];//getparameter should sync the result
		}
		return 0;
	}
}


static int processPCMData_Capture(const unsigned char *r_pBuf,int len)
{
	unsigned char cmd_temp[MAX_BUF_LEN] = {0};
	int i =0;
	unsigned char pcmbuf16Bit[PCM_PPACK_LENGTH*2] = {0};
	unsigned short pcmbuf8Bit[PCM_PPACK_LENGTH] = {0};

	LOGV("processPCMData_Capture() len= %d\n",len);
	memcpy(cmd_temp,r_pBuf,len);

	for(i = 0; i < PCM_PPACK_LENGTH; i++){
		pcmbuf8Bit[i] = PcmProcess::alaw2linear(cmd_temp[i]);
		pcmbuf16Bit[i*2] = pcmbuf8Bit[i];
		pcmbuf16Bit[i*2+1] = pcmbuf8Bit[i] >> 8;
	}

	//int ret_len = fillPcmdata(&capture_buf, pcmbuf16Bit, PCM_PPACK_LENGTH*2);
	int ret_len =out_writeAudioPipe(pcmbuf16Bit, PCM_PPACK_LENGTH*2);
	#ifdef PCM_FILE_DEBUG
	if(pcm_8bitfile_c != 0){
		write(pcm_8bitfile_c,cmd_temp,len);
	}

	if(pcm_16bitfile_c != 0){
		write(pcm_16bitfile_c,pcmbuf16Bit,PCM_PPACK_LENGTH*2);
	}
	#endif
	
	return ret_len;
}


static int processPCMData_Playback(unsigned char *r_pBuf,int len)
{
	unsigned char cmd_temp[MAX_BUF_LEN*2] = {0};
	int i =0;
	short pcm_val = 0;
	int ret_len =0;
	unsigned char pcmbuf8Bit[PCM_PPACK_LENGTH] = {0};

	//LOGV("processPCMData_Playback() len =%d\n",len);
	memcpy(cmd_temp,r_pBuf,len);

	for(i = 0; i < len / 2; i++){
		pcm_val =  (cmd_temp[2*i+1] << 8) | cmd_temp[2*i];
		pcmbuf8Bit[i] =PcmProcess::linear2alaw(pcm_val);
	}

	//LOGV("processPCMData_Playback() SerWrite PCM Data\n");
	ret_len = SerialCommand::SerWrite(NOW_COM, pcmbuf8Bit,len/2, W_TIMEOUTMS);
	
	#ifdef PCM_FILE_DEBUG
	if(pcm_8bitfile_p != 0){
		write(pcm_8bitfile_p,pcmbuf8Bit,len/2);
	}

	if(pcm_16bitfile_p != 0){
		write(pcm_16bitfile_p,cmd_temp,len);
	}
	#endif
	
	return ret_len;
}


int SerialCommand::SerParse_Reponse(const unsigned char *r_pBuf,int len,ser_data_s *pBuf)
{
	unsigned char cmd_temp[MAX_BUF_LEN*2] = {0};
	if(len > MAX_BUF_LEN*2){
		len = MAX_BUF_LEN*2;
	}
	memcpy(cmd_temp,r_pBuf,len);

	LOGV("SerParse_Reponse() case: 0x%x\n",cmd_temp[BC_CMD]);
	switch(cmd_temp[BC_CMD]){
		case REM_ACTIVE_RING:
			pBuf->ser_cmd_type = SERCMD_sd_response;
			pBuf->ser_cmd_data = cmd_temp[BC_CMD];
			pBuf->value = 1;
			//StartCommunication();
			break;
		case REM_ACTIVE_BUSY:
			pBuf->ser_cmd_type = SERCMD_sd_response;
			pBuf->ser_cmd_data = cmd_temp[BC_CMD];
			pBuf->value = 1;
			StopCommunication();
			LOGV("on busy ,on hook!\n");
			break;
		case REM_ACTIVE_CALLID:
			pBuf->ser_cmd_type = SERCMD_sd_response;
			pBuf->ser_cmd_data = cmd_temp[BC_CMD];
			pBuf->value = 1;
			//注意callid数据格式
			//包引  导码   	包长度	命令代码	      年/月/日	时分	callerID CRC校验码
			ParserPhoneNumber(cmd_temp+9,cmd_temp[BC_LEHGTH]-11);
			break;
		case REM_ACTIVE_AUDIO:
			pBuf->ser_cmd_type = SERCMD_unknown;
			pBuf->ser_cmd_data = cmd_temp[BC_CMD];
			pBuf->value = 1;
			processPCMData_Capture(r_pBuf+BC_CMD_TYPE,PCM_PPACK_LENGTH);//跳过头三个字节数据包头
			break;
		case REM_ACTIVE_CANCEL:
			StopCommunication();
			pBuf->ser_cmd_type = SERCMD_sd_response;
			pBuf->ser_cmd_data = cmd_temp[BC_CMD];
			pBuf->value = 1;
			break;	
		default:
			break;
	}
	
	return 0;
}


//#define MAX_PACKET_LEN (32)
static void* thread_read_sercom_fun(void *arg)
{
	int ret = 0;
	int i = 0;
	int n_read_bytes = 0;
	int sum_pre_nread = 0;
	int n_store_bytes = 0;
	int p_n_store_pos = 0;
	int crc16_result = 0;
	
	bool read_completed = false;	
	unsigned char pReponse[MAX_PACKET_LEN] = {0};
	unsigned char PcmReadBuf[UART_PPACK_LENGTH] = {0};
	unsigned char *p_posProcessBuf;
	int processLen = 0;
	ser_data_s ser_data;
	
	LOGV("thread_read_sercom_fun() IN");
	while (1)// var
	{
		if (THREAD_NEED_EXIT == FLAG_READ_THREAD){
			LOGD("thread_read_sercom_fun exit thread");
			break;
		}

		if(FLAG_READ_THREAD == THREAD_IDLE){
			usleep(50);
			continue;
		}
		
		p_n_store_pos = n_store_bytes;
		//LOGE("thread_read_sercom_fun() p_n_store_pos= %d",p_n_store_pos);
		n_read_bytes = SerialCommand::SerRead(NOW_COM, pReponse + n_store_bytes, MAX_PACKET_LEN - n_store_bytes, R_TIMEOUTMS);
		
		//LOGV("thread_read_sercom_fun() n_read_bytes =%d\n",n_read_bytes);
		
		if(n_read_bytes <= 0){
			if(sum_pre_nread != 0){//若读到一部分数据就超时，则抛弃之前读到的数据					
				memset(PcmReadBuf, 0x0, UART_PPACK_LENGTH);
				processLen = 0;
				sum_pre_nread = 0;
			}
			continue;//未读到数据
		}
		
		crc16_result = SerialCommand::getCRC16(pReponse + n_store_bytes,n_read_bytes-2);
		if((((crc16_result >> 8) & 0xff) == pReponse[n_store_bytes+n_read_bytes-2])&&((crc16_result & 0xff) == pReponse[n_store_bytes+n_read_bytes-1]))
		{
			read_completed = true;	
			sum_pre_nread = 0;
			//LOGE("thread_read_sercom_fun() crc16_result is right!!!");
		}else{
			read_completed = false;
			LOGE("thread_read_sercom_fun() n_read_bytes=%d,sum_pre_nread= %d",n_read_bytes,sum_pre_nread);
			if(n_read_bytes < (UART_PPACK_LENGTH - PCM_ACCEPT_ERROR_LEN) && ((n_read_bytes + sum_pre_nread) == UART_PPACK_LENGTH)){
				read_completed = true;
				memcpy(PcmReadBuf,pReponse+n_store_bytes-sum_pre_nread,UART_PPACK_LENGTH);
			}else if(n_read_bytes >= (UART_PPACK_LENGTH - PCM_ACCEPT_ERROR_LEN) && n_read_bytes <= UART_PPACK_LENGTH){//数据补齐到165长度
				memcpy(PcmReadBuf+sum_pre_nread,pReponse+n_store_bytes,n_read_bytes);
				unsigned char fill_data = PcmReadBuf[n_read_bytes-1];
				for(i = 0; i < UART_PPACK_LENGTH-n_read_bytes;i++ ){
					PcmReadBuf[n_read_bytes+i] = fill_data;//数据位补齐为附近的数值;
				}
				read_completed = true;
			}/*else if(sum_pre_nread > UART_PPACK_LENGTH && sum_pre_nread < 2 * UART_PPACK_LENGTH){
				//数据超过一个数据包UART_PPACK_LENGTH，则只取第一个数据包长度，其他数据抛弃
				read_completed = true;
				memcpy(PcmReadBuf,pReponse+n_store_bytes-sum_pre_nread,UART_PPACK_LENGTH);
			}*/else if(n_read_bytes > UART_PPACK_LENGTH && ((n_read_bytes % UART_PPACK_LENGTH) == 0)){
				//数据超过一个数据包UART_PPACK_LENGTH，且是UART_PPACK_LENGTH整数倍
				read_completed = true;
			}

			sum_pre_nread += n_read_bytes;

			if(read_completed == true){
				if(sum_pre_nread < UART_PPACK_LENGTH){
					LOGE("thread_read_sercom_fun() fill pcm data 0x00!!!");
				}else if(sum_pre_nread > UART_PPACK_LENGTH && ((sum_pre_nread % UART_PPACK_LENGTH) == 0)){
					LOGE("thread_read_sercom_fun() read pcm data %d package!!!",sum_pre_nread/UART_PPACK_LENGTH);
					//读取到整数倍的数据包，作为数据未分散收取处理
					sum_pre_nread = 0;
				}
			}
		}
		
		if(read_completed == true){//已收取到N个完整的数据包
			//LOGE("thread_read_sercom_fun() n_read_bytes=%d,p_n_store_pos= %d",n_read_bytes,p_n_store_pos);
			if(sum_pre_nread != 0){
				p_posProcessBuf = PcmReadBuf;
				processLen = UART_PPACK_LENGTH;
			}else{
				p_posProcessBuf = pReponse+p_n_store_pos;
				processLen = n_read_bytes;
			}

			if(SerialCommand::CheckIsValid(p_posProcessBuf,processLen) < 0){
				LOGE("SerRead() CMD is invalid!!!");
				if(processLen == UART_PPACK_LENGTH){
					SerialCommand::SerParse_Reponse(p_posProcessBuf,processLen,&ser_data);
					if(sum_pre_nread != 0){					
						memset(PcmReadBuf, 0x0, UART_PPACK_LENGTH);
						processLen = 0;
						sum_pre_nread = 0;
					}
				}
			}else{
				//parse the pReponse
				int n_package = processLen / UART_PPACK_LENGTH;
				if(n_package > 1){//处理接收到的N数据包
					LOGE("thread_read_sercom_fun() process pcm data %d package!!!",n_package);
					for(int i =0; i < n_package; i++){
						SerialCommand::SerParse_Reponse(p_posProcessBuf+(i*UART_PPACK_LENGTH),UART_PPACK_LENGTH,&ser_data);
					}
				}else{
					SerialCommand::SerParse_Reponse(p_posProcessBuf,processLen,&ser_data);
				}
						
				if(Now_cmd_e == SERCMD_md_getparameter){
					if(ser_data.ser_cmd_type == SERCMD_md_getparameter){
						LOGV("SerParse_Reponse() return getparameter result ser_data ser_cmd_data:0x%x,value:0x%x",ser_data.ser_cmd_data,ser_data.value);
						S_Read.value = ser_data.value;
						S_Read.ser_cmd_type = Now_cmd_e;
						pthread_cond_signal(&m_cond);
					}
				}else{
					if(ser_data.ser_cmd_type == SERCMD_sd_response){
						sp<CmdControlListener> listener = SerialCommand::getInstance()->getListener();
						LOGV("SerParse_Reponse() ser_data ser_cmd_data:0x%x,value:0x%x",ser_data.ser_cmd_data,ser_data.value);
						if(listener != NULL){
							listener->notify(ser_data.ser_cmd_data,ser_data.value,CallidNumber);
						}
					}
				}

				if(sum_pre_nread != 0){					
					memset(PcmReadBuf, 0x0, UART_PPACK_LENGTH);
					processLen = 0;
					sum_pre_nread = 0;
				}
				
			}
		}else{
			//未收取到一个完整数据包
		}
		
		n_store_bytes += n_read_bytes;

		if(n_store_bytes + UART_PPACK_LENGTH >= MAX_PACKET_LEN)
		{
			n_store_bytes = 0;
			memset(pReponse, 0x0, MAX_PACKET_LEN);
			SerialCommand::SerClear(NOW_COM);
			Clearser_data_s(&ser_data);
			LOGE("thread_read_sercom_fun: messy code......");
			continue;
		}
				
		//usleep(50);
	}

	FLAG_READ_THREAD = THREAD_EXIT;
	
	return (void*) 0;
}

static void* thread_write_sercom_fun(void *arg)
{
	unsigned char pcm_buf[PIPE_MAX_BUF_LEN] = {0};
	int len = 0;
	int ret = 0;
	LOGV("thread_write_sercom_fun() IN");
	
	while(1)
	{
		if (THREAD_NEED_EXIT == FLAG_WRITE_THREAD){
			LOGD("thread_write_sercom_fun exit thread");
			break;
		}

		if(FLAG_WRITE_THREAD == THREAD_IDLE){
			usleep(50);
			continue;
		}
		
		//if(playback_buf.is_empty == true){
		if(false){
			usleep(50);	
		}else{
			usleep(18000);//时间需和暂停发送数据同步延时
			//len = extractPcmdata(&playback_buf,pcm_buf);
			len = in_readAudioPipe(pcm_buf,PCM_PPACK_LENGTH*2);
			
			if(len > 0){
				ret = processPCMData_Playback(pcm_buf,len);
				
				memset(pcm_buf,0x00,len);
			}
		}
	}

	FLAG_WRITE_THREAD = THREAD_EXIT;
	return (void*) 0;
}

static void* thread_audio_caption_fun(void *arg)
{
	LOGV("thread_audio_caption_fun() IN");
	while(1)
	{
		if (THREAD_NEED_EXIT == FLAG_CAPTURE_THREAD){
			LOGD("thread_audio_caption_fun exit thread");
			break;
		}

		if(FLAG_CAPTURE_THREAD == THREAD_IDLE){
			usleep(50);
			continue;
		}
		
		pthread_mutex_lock(&m_mutex);
		if(capture_buf.is_empty == true){
			usleep(50);
		}else{
			out_writeAudioPipe(capture_buf.pcm_buf,capture_buf.len);
			cleanPcmdata(&capture_buf);
		}
		pthread_mutex_unlock(&m_mutex);	
	}

	FLAG_CAPTURE_THREAD = THREAD_EXIT;
	return (void*) 0;
}

static void* thread_audio_playback_fun(void *arg)
{
	unsigned char pcm_buf[PIPE_MAX_BUF_LEN] = {0};
	int len = 0;
	LOGV("thread_audio_playback_fun() IN");
	while(1)
	{
		if (THREAD_NEED_EXIT == FLAG_PLAYBACK_THREAD){
			LOGD("thread_audio_playback_fun exit thread");
			break;
		}

		if(FLAG_PLAYBACK_THREAD == THREAD_IDLE){
			usleep(50);
			continue;
		}
		
		pthread_mutex_lock(&m_mutex);
		if(playback_buf.is_empty == true){
			cleanPcmdata(&playback_buf);
			len = in_readAudioPipe(pcm_buf,PCM_PPACK_LENGTH*2);
			if(len > 0){
				fillPcmdata(&playback_buf, pcm_buf, len);
			}else{
				//LOGV("in_readAudioPipe() read playback_buf len = 0");
			}
		}else{
			usleep(50);			
		}
		pthread_mutex_unlock(&m_mutex);
	}

	FLAG_PLAYBACK_THREAD = THREAD_EXIT;
	return (void*) 0;
}

static int initAudioPipe()
{
	char Cmdbuf[MAX_BUF_LEN] = {0};
	
	if(mkfifo(AUDIO_FIFO_C,0777) != 0){
		if(errno == EEXIST){
			LOGV("initAudioPipe() pipe %s: exist", AUDIO_FIFO_C);
		}else{
			LOGE("initAudioPipe() pipe %s: mkfifo failed errno:%d", AUDIO_FIFO_C,errno);
			//return -1;
		}
	}

	if(mkfifo(AUDIO_FIFO_P,0777) != 0){
		if(errno == EEXIST){
			LOGV("initAudioPipe() pipe %s: exist", AUDIO_FIFO_P);
		}else{
			LOGE("initAudioPipe() pipe %s: mkfifo failed errno:%d", AUDIO_FIFO_P,errno);
			return -1;
		}
	}

	sprintf(Cmdbuf,"chmod 777 %s",AUDIO_FIFO_C);
	system(Cmdbuf);

	memset(Cmdbuf,0x00,MAX_BUF_LEN);
	
	sprintf(Cmdbuf,"chmod 777 %s",AUDIO_FIFO_P);
	system(Cmdbuf);

	audio_fifo_playback = openin_readAudioPipe();
	audio_fifo_capture= openout_writeAudioPipe();

	/*FLAG_CAPTURE_THREAD = THREAD_IDLE;
	if(pthread_equal(capture_tid, (pthread_t)0L)){
		if (pthread_create(&capture_tid, NULL, thread_audio_caption_fun, NULL) != 0) {
			LOGE("initAudioPipe() pthread_create thread_audio_caption_fun failed");
			return -1;
		}
	}else{
		LOGD("initAudioPipe() thread_audio_caption_fun exist");
	}

	FLAG_PLAYBACK_THREAD = THREAD_IDLE;
	if(pthread_equal(playback_tid, (pthread_t)0L)){
		if (pthread_create(&playback_tid, NULL, thread_audio_playback_fun, NULL) != 0) {
			LOGE("initAudioPipe() pthread_create thread_audio_playback_fun failed");
			return -1;
		}
	}else{
		LOGD("initAudioPipe() thread_audio_playback_fun exist");
	}*/

	return 0;
}

static int deInitAudioPipe()
{
	void *ret;

	LOGV("deInitAudioPipe() IN");
	closeAudioPipe();
	/*FLAG_CAPTURE_THREAD = THREAD_NEED_EXIT;
	FLAG_PLAYBACK_THREAD = THREAD_NEED_EXIT;
	if (pthread_equal(capture_tid, (pthread_t)0L))
	{
		LOGD("deInitAudioPipe() thread_audio_caption_fun not exist");
	}else{
		pthread_join(capture_tid, &ret);
		capture_tid = (pthread_t)0L;
		
	}

	if (pthread_equal(playback_tid, (pthread_t)0L))
	{
		LOGD("deInitAudioPipe() thread_audio_playback_fun not exist");
	}else{
		pthread_join(playback_tid, &ret);
		playback_tid = (pthread_t)0L;
		
	}

	while (1) {
		if (FLAG_CAPTURE_THREAD == THREAD_EXIT && FLAG_PLAYBACK_THREAD == THREAD_EXIT) {
			LOGV("deInitSer capture and playback thread exited");
			break;
		}
		usleep(100000);
	}*/
	return 0;
}

int SerialCommand::iniSer() {
	LOGD("Enter iniSer()");
	if(mSerInitialized)
	{
		LOGE("iniSer():initialized already!!!!!!!");
		return 0;
	}
	
	int ret = 0;
	pthread_t tid;
	pthread_t tpid;
	pthread_mutexattr_t ma;

	ret = SerOpen(NOW_COM);
	if (ret < 0) {
		LOGE("iniSer->Seropen Failed");
		return -1;
	}
	SerialCommand::SerClear(NOW_COM);
	pthread_mutexattr_init(&ma);
	pthread_mutexattr_settype(&ma, PTHREAD_MUTEX_RECURSIVE);
	pthread_mutex_init(&m_mutex, &ma);
	pthread_mutexattr_destroy(&ma);

	mSerInitialized = true;

	pthread_cond_init(&m_cond, NULL);
	Now_cmd_e = SERCMD_unknown;
	memset(&S_Read, 0x0, sizeof(S_Read));

	FLAG_READ_THREAD = THREAD_IDLE; /*thread running flag*/

	if(pthread_equal(com_read_tid, (pthread_t)0L)){
		if (pthread_create(&com_read_tid, NULL, thread_read_sercom_fun, NULL) != 0) {
			LOGE("iniSer() pthread_create thread_read_sercom_fun failed");
			return -1;
		}
	}else{
		LOGD("iniSer() thread_read_sercom_fun exist");
	}

	FLAG_WRITE_THREAD = THREAD_IDLE;
	if(pthread_equal(com_write_tid, (pthread_t)0L)){
		if (pthread_create(&com_write_tid, NULL, thread_write_sercom_fun, NULL) != 0) {
			LOGE("iniSer() pthread_create thread_write_sercom_fun failed");
			return -1;
		}
	}else{
		LOGD("iniSer() thread_write_sercom_fun exist");
	}

	initAudioPipe();

	setThreadReadState(THREAD_WORKING);
	//setThreadWriteState(THREAD_WORKING);
	return 0;
}

int SerialCommand::deInitSer() 
{
	void *ret;

	LOGD("Enter deInitSer()");
	
	deInitAudioPipe();

	FLAG_READ_THREAD = THREAD_NEED_EXIT;
	FLAG_WRITE_THREAD = THREAD_NEED_EXIT;
	if (pthread_equal(com_read_tid, (pthread_t)0L))
	{
		LOGD("initAudioPipe() thread_read_sercom_fun not exist");
	}else{
		pthread_join(com_read_tid, &ret);
		com_read_tid = (pthread_t)0L;
		
	}
	
	if (pthread_equal(com_write_tid, (pthread_t)0L))
	{
		LOGD("initAudioPipe() thread_write_sercom_fun not exist");
	}else{
		pthread_join(com_write_tid, &ret);
		com_write_tid = (pthread_t)0L;
		
	}
	
	while (1) {
		if (FLAG_READ_THREAD == THREAD_EXIT && FLAG_WRITE_THREAD == THREAD_EXIT) {
			LOGV("deInitSer serial read and write thread exited");
			break;
		}
		usleep(100000);
	}
	SerClose(NOW_COM);

	mSerInitialized = false;

	pthread_cond_destroy(&m_cond);
	pthread_mutex_destroy(&m_mutex);
	LOGD("Leave deInitSer()");
	return 0;
}

int SerialCommand::onExecuteCmd(const char * cmd)
{
	int ret = 0;	
	int index = 0;
	
	LOGD("Enter onExecuteCmd() cmd len = %d",strlen(cmd));//字符串内容由应用填入，且应用确保数据的准确性

	if(!mSerInitialized && cmd != NULL)
	{
		LOGE("onExecuteCmd():ser not initialized already!!!!!!!");
		return -1;
	}

	pthread_mutex_lock(&m_mutex);
	for (index = 0; index < 3; index++){		
		Now_cmd_e = SERCMD_unknown;	
		//check cmd shoud change to Hex data
		ret = SerWrite_SlaveDevice(NOW_COM, cmd, W_TIMEOUTMS,strlen(cmd));
		LOGD("SerWrite_SlaveDevice() ret:%d",ret);
		if (ret >= 0) {
			struct timeval delta;
			struct timespec abstime;
			gettimeofday(&delta, NULL);
			abstime.tv_sec = delta.tv_sec + R_TIMEOUTMS / 1000;
			abstime.tv_nsec = (delta.tv_usec + (R_TIMEOUTMS % 1000) * 1000) * 1000;
			if (abstime.tv_nsec > 1000000000) {
				abstime.tv_sec += 1;
				abstime.tv_nsec -= 1000000000;
			}

			if (pthread_cond_timedwait(&m_cond, &m_mutex, &abstime) == 0) {
				LOGV(" onExcuteCmd:  SUCC...");//getparameter should sync the result to API
				if(ret != 0){
					Now_cmd_e = (ser_cmd_e)ret;
				}else{
					Now_cmd_e = SERCMD_unknown;
				}
				ret = S_Read.value;
				break;
			} else {
				LOGV(" onExcuteCmd:  no response %d...", index);
				Now_cmd_e = SERCMD_unknown;
				ret = -2;
				break;
			}
		} else {
			LOGV("onExecuteCmd: SerWrite_SlaveDevice err...");
			Now_cmd_e = SERCMD_unknown;
			ret = -1;
			break;
		}
	}
	
	pthread_mutex_unlock(&m_mutex);
	
	LOGV("Leave onExecuteCmd() ret: %d", ret);
	return ret;

}
int SerialCommand::onProcessPCMData(unsigned char *pcmData,int len){
	if(len == PCM_PPACK_LENGTH * 2){
		LOGE("onProcssPCMData() buffer is PCM data,len = %d !!!",len);
		//just for test Playback data
		//fillPcmdata(&playback_buf, pcmData, len);
		processPCMData_Playback(pcmData,len);
	}
	return 0;
}

} // namespace soundbar

