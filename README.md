CSerialPort
===========
a lightweight library of serial port, which can easy to read and write serical port on windows with C++

1. 2021-01-13 the typical demo to use this class: https://github.com/liquanhai/LSDComm

2. First Version by Remon Spekreijse on 2000-02-08
http://www.codeguru.com/cpp/i-n/network/serialcommunications/article.php/c2483/A-communication-class-for-serial-port.htm


3. Second Version by mrlong on 2007-12-25
https://code.google.com/p/mycom/
* 增加 ClosePort
* 增加 WriteToPort 两个方法
* 增加 SendData 与 RecvData 方法


4. by liquanhai on 2011-11-04
http://blog.csdn.net/liquanhai/article/details/4955253
* 增加 ClosePort 中交出控制权，防止死锁问题


5. by liquanhai on 2011-11-06
http://blog.csdn.net/liquanhai/article/details/6941574
* 增加 ReceiveChar 中防止线程死锁


6. by viruscamp on 2013-12-04
https://github.com/viruscamp/CSerialPort
* 增加 IsOpen 判断是否打开
* 修正 InitPort 中 parity Odd Even 参数取值错误
* 修改 InitPort 中 portnr 取值范围，portnr>9 时特殊处理
* 取消对 MFC 的依赖，使用 HWND 替代 CWnd，使用 win32 thread 函数而不是 MFC 的
* 增加用户消息编号自定义，方法来自 CnComm


7. by itas109 on 2014-01-10
http://blog.csdn.net/itas109/article/details/18358297
* 解决COM10以上端口无法显示的问题 
* 扩展可选择端口，最大值MaxSerialPortNum可以自定义 
* 添加QueryKey()和Hkey2ComboBox两个方法，用于自动查询当前有效的串口号。


8. by liquanhai on 2014-12-18
* 增加一些处理措施，主要是对减少CPU占用率
