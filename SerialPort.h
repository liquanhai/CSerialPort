/*
**	FILENAME			CSerialPort.h
**
**	PURPOSE				This class can read, write and watch one serial port.
**						It sends messages to its owner when something happends on the port
**						The class creates a thread for reading and writing so the main
**						program is not blocked.
**
**	CREATION DATE		15-09-1997
**	LAST MODIFICATION	12-11-1997
**
**	AUTHOR				Remon Spekreijse
**
**
************************************************************************************
**  author: mrlong date:2007-12-25
**
**  �Ľ�
**	1) ���� ClosePort
**	2) ���� WriteToPort ��������
**	3) ���� SendData �� RecvData ����
**************************************************************************************
***************************************************************************************
**  author��liquanhai date:2011-11-06
**
**  �Ľ�
**	1) ���� ClosePort �н�������Ȩ����ֹ��������
**	2) ���� ReceiveChar �з�ֹ�߳�����
**************************************************************************************
***************************************************************************************
**  author��viruscamp date:2013-12-04
**
**  �Ľ�
**	1) ���� IsOpen �ж��Ƿ��
**	2) ���� InitPort �� parity Odd Even ����ȡֵ����
**	3) �޸� InitPort �� portnr ȡֵ��Χ��portnr>9 ʱ���⴦��
**	4) ȡ���� MFC ��������ʹ�� HWND ��� CWnd��ʹ�� win32 thread ���������� MFC ��
**	5) �����û���Ϣ����Զ��壬�������� CnComm
*************************************************************************************** 
***************************************************************************************
**  author: itas109  date:2014-01-10
**  Blog��blog.csdn.net/itas109
**
**  �Ľ�
**    1) ���COM10���϶˿��޷���ʾ������
**    2) ��չ��ѡ��˿ڣ����ֵMaxSerialPortNum�����Զ���
**    3) ���QueryKey()��Hkey2ComboBox���������������Զ���ѯ��ǰ��Ч�Ĵ��ںš�
** 
*/

#ifndef __SERIALPORT_H__
#define __SERIALPORT_H__

#ifndef WM_COMM_MSG_BASE 
	#define WM_COMM_MSG_BASE		WM_USER + 617		//!< ��Ϣ��ŵĻ���  
#endif

#define WM_COMM_BREAK_DETECTED		WM_COMM_MSG_BASE + 1	// A break was detected on input.
#define WM_COMM_CTS_DETECTED		WM_COMM_MSG_BASE + 2	// The CTS (clear-to-send) signal changed state. 
#define WM_COMM_DSR_DETECTED		WM_COMM_MSG_BASE + 3	// The DSR (data-set-ready) signal changed state. 
#define WM_COMM_ERR_DETECTED		WM_COMM_MSG_BASE + 4	// A line-status error occurred. Line-status errors are CE_FRAME, CE_OVERRUN, and CE_RXPARITY. 
#define WM_COMM_RING_DETECTED		WM_COMM_MSG_BASE + 5	// A ring indicator was detected. 
#define WM_COMM_RLSD_DETECTED		WM_COMM_MSG_BASE + 6	// The RLSD (receive-line-signal-detect) signal changed state. 
#define WM_COMM_RXCHAR				WM_COMM_MSG_BASE + 7	// A character was received and placed in the input buffer. 
#define WM_COMM_RXFLAG_DETECTED		WM_COMM_MSG_BASE + 8	// The event character was received and placed in the input buffer.  
#define WM_COMM_TXEMPTY_DETECTED	WM_COMM_MSG_BASE + 9	// The last character in the output buffer was sent.  

#define MaxSerialPortNum 20   ///��Ч�Ĵ����ܸ��������Ǵ��ڵĺ� //add by itas109 2014-01-09
class CSerialPort
{														 
public:
	// contruction and destruction
	CSerialPort();
	virtual		~CSerialPort();

	// port initialisation											
	BOOL		InitPort(HWND pPortOwner, UINT portnr = 1, UINT baud = 19200, 
				TCHAR parity = _T('N'), UINT databits = 8, UINT stopsbits = 1, 
				DWORD dwCommEvents = EV_RXCHAR | EV_CTS, UINT nBufferSize = 512,
			
				DWORD ReadIntervalTimeout = 1000,
				DWORD ReadTotalTimeoutMultiplier = 1000,
				DWORD ReadTotalTimeoutConstant = 1000,
				DWORD WriteTotalTimeoutMultiplier = 1000,
				DWORD WriteTotalTimeoutConstant = 1000);

	// start/stop comm watching
	///���ƴ��ڼ����߳�
	BOOL		 StartMonitoring();//��ʼ����
	BOOL		 RestartMonitoring();//���¼���
	BOOL		 StopMonitoring();//ֹͣ����

	DWORD		 GetWriteBufferSize();///��ȡд�����С
	DWORD		 GetCommEvents();///��ȡ�¼�
	DCB			 GetDCB();///��ȡDCB

///д���ݵ�����
	//void		WriteToPort(char* string);
	void		WriteToPort(char* string,size_t nBytesOfTheBuffer); // add by mrlong 2007-12-25
	//void		WriteToPort(char* string);	 // add by mrlong 2007-12-25
    //void		WriteToPort(char* string,int n);//add by mrlong 2007-12-2
	void		WriteToPort(PBYTE Buffer, size_t n);// add by mrlong
	void		ClosePort();					 // add by mrlong 2007-12-2  
	BOOL		IsOpen();

	void SendData(const PBYTE lpszData, DWORD nLength);   //���ڷ��ͺ��� by mrlong 2008-2-15
	BOOL RecvData(PBYTE lpszData, DWORD nSize);	  //���ڽ��պ��� by mrlong 2008-2-15
	void QueryKey(HKEY hKey);///��ѯע���Ĵ��ںţ���ֵ����������
	void Hkey2ComboBox(CComboBox& m_PortNO);///��QueryKey��ѯ���Ĵ��ں���ӵ�CComboBox�ؼ���

protected:
	// protected memberfunctions
	void		ProcessErrorMessage(TCHAR* ErrorText);///������
	static DWORD WINAPI CommThread(LPVOID pParam);///�̺߳���
	static void	ReceiveChar(CSerialPort* port);
	static void	WriteChar(CSerialPort* port);

	// thread
	//CWinThread*			m_Thread;
	HANDLE			  m_Thread;
	BOOL                m_bIsSuspened;///thread�����߳��Ƿ����

	// synchronisation objects
	CRITICAL_SECTION	m_csCommunicationSync;///�ٽ���Դ
	BOOL				m_bThreadAlive;///�����߳����б�־

	// handles
	HANDLE				m_hShutdownEvent;  //stop�������¼�
	HANDLE				m_hComm;		   // ���ھ�� 
	HANDLE				m_hWriteEvent;	 // write

	// Event array. 
	// One element is used for each event. There are two event handles for each port.
	// A Write event and a receive character event which is located in the overlapped structure (m_ov.hEvent).
	// There is a general shutdown when the port is closed. 
	///�¼����飬����һ��д�¼��������¼����ر��¼�
	///һ��Ԫ������һ���¼����������¼��̴߳���˿ڡ�
	///д�¼��ͽ����ַ��¼�λ��overlapped�ṹ�壨m_ov.hEvent����
	///���˿ڹر�ʱ����һ��ͨ�õĹرա�
	HANDLE				m_hEventArray[3];

	// structures
	OVERLAPPED			m_ov;///�첽I/O
	COMMTIMEOUTS		m_CommTimeouts;///��ʱ����
	DCB					m_dcb;///�豸���ƿ�

	// owner window
	//CWnd*				m_pOwner;
	HWND				m_pOwner;


	// misc
	UINT				m_nPortNr;		//?????
	PBYTE				m_szWriteBuffer;///д������
	DWORD				m_dwCommEvents;
	DWORD				m_nWriteBufferSize;///д�����С

	size_t				 m_nWriteSize;//д���ֽ��� //add by mrlong 2007-12-25
};

#endif __SERIALPORT_H__
