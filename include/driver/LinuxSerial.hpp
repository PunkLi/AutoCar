#pragma once

#ifndef UINT
typedef unsigned int UINT;
#endif

#ifndef UCHAR
typedef unsigned char UCHAR;
#endif

/**
* @brief: CLinuxSerial class　实现串口通信
*/
class CLinuxSerial
{
public:
    CLinuxSerial();
    CLinuxSerial(UINT portNo = 0 , UINT baudRate = 115200 );
    ~CLinuxSerial();

    bool InitPort(UINT portNo = 0, UINT baudRate = 115200);
    UINT ReadData(UCHAR *data, UINT length);
    UINT WriteData(UCHAR *data, UINT length);
    UINT GetBytesInCom();
    int getfd2car();

private:
    int m_iSerialID;
    bool OpenPort(UINT portNo);
    void ClosePort();
};

