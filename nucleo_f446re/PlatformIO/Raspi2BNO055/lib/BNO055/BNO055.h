#ifndef BNO055_H
#define BNO055_H
 
#include "mbed.h"
 
//UART通信に使用するバッファの最大サイズ
#define BNO055_UART_BUF_MAXLEN      24      //[byte]
//I2Cデフォルトスレーブアドレス
#define BNO055_I2C_DEFADDR          0x28
 
#define BNO055_PAGE_ID              0x07
 
#define BNO055P0_CHIP_ID            0x00
#define BNO055P0_ACC_ID             0x01
#define BNO055P0_MAG_ID             0x02
#define BNO055P0_GYR_ID             0x03
#define BNO055P0_SW_REV_ID_LSB      0x04
#define BNO055P0_SW_REV_ID_MSB      0x05
#define BNO055P0_BL_REV_ID          0x06
#define BNO055P0_ACC_DATA_X_LSB     0x08
#define BNO055P0_ACC_DATA_X_MSB     0x09
#define BNO055P0_ACC_DATA_Y_LSB     0x0A
#define BNO055P0_ACC_DATA_Y_MSB     0x0B
#define BNO055P0_ACC_DATA_Z_LSB     0x0C
#define BNO055P0_ACC_DATA_Z_MSB     0x0D
#define BNO055P0_MAG_DATA_X_LSB     0x0E
#define BNO055P0_MAG_DATA_X_MSB     0x0F
#define BNO055P0_MAG_DATA_Y_LSB     0x10
#define BNO055P0_MAG_DATA_Y_MSB     0x11
#define BNO055P0_MAG_DATA_Z_LSB     0x12
#define BNO055P0_MAG_DATA_Z_MSB     0x13
#define BNO055P0_GYR_DATA_X_LSB     0x14
#define BNO055P0_GYR_DATA_X_MSB     0x15
#define BNO055P0_GYR_DATA_Y_LSB     0x16
#define BNO055P0_GYR_DATA_Y_MSB     0x17
#define BNO055P0_GYR_DATA_Z_LSB     0x18
#define BNO055P0_GYR_DATA_Z_MSB     0x19
#define BNO055P0_EUL_HEADING_LSB    0x1A
#define BNO055P0_EUL_HEADING_MSB    0x1B
#define BNO055P0_EUL_ROLL_LSB       0x1C
#define BNO055P0_EUL_ROLL_MSB       0x1D
#define BNO055P0_EUL_PITCH_LSB      0x1E
#define BNO055P0_EUL_PITCH_MSB      0x1F
#define BNO055P0_QUA_DATA_W_LSB     0x20
#define BNO055P0_QUA_DATA_W_MSB     0x21
#define BNO055P0_QUA_DATA_X_LSB     0x22
#define BNO055P0_QUA_DATA_X_MSB     0x23
#define BNO055P0_QUA_DATA_Y_LSB     0x24
#define BNO055P0_QUA_DATA_Y_MSB     0x25
#define BNO055P0_QUA_DATA_Z_LSB     0x26
#define BNO055P0_QUA_DATA_Z_MSB     0x27
#define BNO055P0_LIA_DATA_X_LSB     0x28
#define BNO055P0_LIA_DATA_X_MBS     0x29
#define BNO055P0_LIA_DATA_Y_LSB     0x2A
#define BNO055P0_LIA_DATA_Y_MBS     0x2B
#define BNO055P0_LIA_DATA_Z_LSB     0x2C
#define BNO055P0_LIA_DATA_Z_MBS     0x2D
#define BNO055P0_GRV_DATA_X_LSB     0x2E
#define BNO055P0_GRV_DATA_X_MSB     0x2F
#define BNO055P0_GRV_DATA_Y_LSB     0x30
#define BNO055P0_GRV_DATA_Y_MSB     0x31
#define BNO055P0_GRV_DATA_Z_LSB     0x32
#define BNO055P0_GRV_DATA_Z_MSB     0x33
#define BNO055P0_TEMP               0x34
#define BNO055P0_CALIB_STAT         0x35
#define BNO055P0_ST_RESULT          0x36
#define BNO055P0_INT_STA            0x37
#define BNO055P0_SYS_CLK_STATUS     0x38
#define BNO055P0_SYS_STATUS         0x39
#define BNO055P0_SYS_ERR            0x3A
#define BNO055P0_UNIT_SEL           0x3B
#define BNO055P0_OPR_MODE           0x3D
#define BNO055P0_PWR_MODE           0x3E
#define BNO055P0_SYS_TRIGGER        0x3F
#define BNO055P0_TEMP_SOURCE        0x40
#define BNO055P0_AXIS_MAP_CONFIG    0x41
#define BNO055P0_AXIS_MAP_SIGN      0x42
#define BNO055P0_ACC_OFFSET_X_LSB   0x55
#define BNO055P0_ACC_OFFSET_X_MSB   0x56
#define BNO055P0_ACC_OFFSET_Y_LSB   0x57
#define BNO055P0_ACC_OFFSET_Y_MSB   0x58
#define BNO055P0_ACC_OFFSET_Z_LSB   0x59
#define BNO055P0_ACC_OFFSET_Z_MSB   0x5A
#define BNO055P0_MAG_OFFSET_X_LSB   0x5B
#define BNO055P0_MAG_OFFSET_X_MSB   0x5C
#define BNO055P0_MAG_OFFSET_Y_LSB   0x5D
#define BNO055P0_MAG_OFFSET_Y_MSB   0x5E
#define BNO055P0_MAG_OFFSET_Z_LSB   0x5F
#define BNO055P0_MAG_OFFSET_Z_MSB   0x60
#define BNO055P0_GYR_OFFSET_X_LSB   0x61
#define BNO055P0_GYR_OFFSET_X_MSB   0x62
#define BNO055P0_GYR_OFFSET_Y_LSB   0x63
#define BNO055P0_GYR_OFFSET_Y_MSB   0x64
#define BNO055P0_GYR_OFFSET_Z_LSB   0x65
#define BNO055P0_GYR_OFFSET_Z_MSB   0x66
#define BNO055P0_ACC_RADIUS_LSB     0x67
#define BNO055P0_ACC_RADIUS_MSB     0x68
#define BNO055P0_MAG_RADIUS_LSB     0x69
#define BNO055P0_MAG_RADIUS_MSB     0x6A
 
#define BNO055P1_ACC_CONFIG         0x08
#define BNO055P1_MAG_CONFIG         0x09
#define BNO055P1_GYR_CONFIG_0       0x0A
#define BNO055P1_GYR_CONFIG_1       0x0B
#define BNO055P1_ACC_SLEEP_CONFIG   0x0C
#define BNO055P1_GYR_SLEEP_CONFIG   0x0D
#define BNO055P1_INT_MSK            0x0F
#define BNO055P1_INT_EN             0x10
#define BNO055P1_ACC_AM_THRES       0x11
#define BNO055P1_ACC_INT_SETTINGS   0x12
#define BNO055P1_ACC_HG_DURATION    0x13
#define BNO055P1_ACC_HG_THRES       0x14
#define BNO055P1_ACC_NM_THRES       0x15
#define BNO055P1_ACC_NM_SET         0x16
#define BNO055P1_GYR_INT_SETING     0x17
#define BNO055P1_GYR_HR_X_SET       0x18
#define BNO055P1_GYR_DUR_X          0x19
#define BNO055P1_GYR_HR_Y_SET       0x1A
#define BNO055P1_GYR_DUR_Y          0x1B
#define BNO055P1_GYR_HR_Z_SET       0x1C
#define BNO055P1_GYR_DUR_Z          0x1D
#define BNO055P1_GYR_AM_THRES       0x1E
#define BNO055P1_GYR_AM_SET         0x1F
 
 
class BNO055_CTRL{
public:
    BNO055_CTRL();
    virtual ~BNO055_CTRL();
protected:
    bool page1;
    char *ary;
    char lastError;
    char lastLength;
public:
    char getNowPage();
    char getLastError();
    char getLastLength();
    virtual void init();
    virtual char rr(bool isPage1, char regAddr);
    virtual char rrc(bool isPage1, char startRegAddr, unsigned char *receiveBytes, char length);
    virtual char wr(bool isPage1, char regAddr, char wBytes);
    virtual char wrc(bool isPage1, char startRegAddr, char *Bytes, char length);
};
 
class BNO055_UART_CTRL : public BNO055_CTRL{
public:
    BNO055_UART_CTRL(RawSerial *uart);
    virtual ~BNO055_UART_CTRL();
private:
    RawSerial *iface;
    short rxd;
    bool read_mark;
 
    void rxInterrupt();
public:
    virtual void init();
    virtual char rr(bool isPage1, char regAddr);
    virtual char rrc(bool isPage1, char startRegAddr, unsigned char *receiveBytes, char length);
    virtual char wr(bool isPage1, char regAddr, char wBytes);
    virtual char wrc(bool isPage1, char startRegAddr, char *Bytes, char length);
};
 
class BNO055_I2C_CTRL : public BNO055_CTRL{
public:
    BNO055_I2C_CTRL(I2C *iic, char addr, unsigned int freq);
    virtual ~BNO055_I2C_CTRL();
private:
    I2C *iface;
    char i2c_writeAddr;
    char i2c_readAddr;
    unsigned int i2c_freq;
public:
    virtual void init();
    virtual char rr(bool isPage1, char regAddr);
    virtual char rrc(bool isPage1, char startRegAddr, unsigned char *receiveBytes, char length);
    virtual char wr(bool isPage1, char regAddr, char wBytes);
    virtual char wrc(bool isPage1, char startRegAddr, char *Bytes, char length);
};
 
class BOARDC_BNO055{
public:
    BOARDC_BNO055(PinName tx, PinName rx);
    BOARDC_BNO055(RawSerial *uart);
    BOARDC_BNO055(PinName scl, PinName sda, char addr=BNO055_I2C_DEFADDR, unsigned int freq=100000);
    BOARDC_BNO055(I2C *iic, char addr=BNO055_I2C_DEFADDR, unsigned int freq=100000);
    ~BOARDC_BNO055();
 
private:
    BNO055_CTRL *ctrl;
    float scaleACC;
    float scaleMAG; //fixed
    float scaleGYRO;
    float scaleTEMP;
    float scaleEuler;
    float scaleLIA; //=scaleACC
    float scaleGV; //=scaleACC
    double scaleQuaternion; //fixed
    unsigned char axisRemap;
    unsigned char axisSign;
    bool clkExt;
 
public:
    char initialize(bool resetIface=true);
    char getIfaceLastError();
    char getIfaceLastLength();
 
    char customRead(bool isPage1, char regAddr);
    char customReadC(bool isPage1, char startRegAddr, unsigned char *receiveBytes, unsigned char length);
    char customWrite(bool isPage1, char regAddr, char wBytes);
    char customWriteC(bool isPage1, char startRegAddr, char *Bytes, unsigned char length);
 
    char getPage();
    void setPage(unsigned char pageNo);
 
    char getChipID();
    char getAccChipID();
    char getMagChipID();
    char getGyroChipID();
 
    short getRevision();
    char getBootRevision();
 
    float getAccScale();
    float getMagScale();
    float getGyroScale();
    float getTempScale();
    float getEulerScale();
    float getLinearScale();
    float getGVScale();
    double getQuaternionScale();
 
    void getAccDataAll(short &accX, short &accY, short &accZ);
    short getAccDataX();
    short getAccDataY();
    short getAccDataZ();
 
    void getMagDataAll(short &magX, short &magY, short &magZ);
    short getMagDataX();
    short getMagDataY();
    short getMagDataZ();
 
    void getGyroDataAll(short &gyroX, short &gyroY, short &gyroZ);
    short getGyroDataX();
    short getGyroDataY();
    short getGyroDataZ();
 
    void getEulerDataAll(short &E_heading, short &E_roll, short &E_pitch);
    short getEulerDataHeading();
    short getEulerDataYaw();
    short getEulerDataRoll();
    short getEulerDataPitch();
 
    void get9Axis(short *box);
    void get9AxisAndEUL(short *box);
 
    void getQuaternion(short &q1, short &q2, short &q3, short &q4);
    void getEulerFromQ(double &E_heading, double &E_roll, double &E_pitch);
 
    void getLinearAccDataAll(short &L_accX, short &L_accY, short &L_accZ);
    short getLinearAccDataX();
    short getLinearAccDataY();
    short getLinearAccDataZ();
 
    void getGVectorDataAll(short &gvX, short &gvY, short &gvZ);
    short getGVectorDataX();
    short getGVectorDataY();
    short getGVectorDataZ();
 
    char getTemperature();
 
    void getCalibStatusAll(char &sys, char &acc, char &mag, char &gyro);
    char getCalibStatusSys();
    char getCalibStatusAcc();
    char getCalibStatusMag();
    char getCalibStatusGyro();
 
    char getSelfTestResultAll();
    bool getSelfTestResultMCU();
    bool getSelfTestResultAcc();
    bool getSelfTestResultMag();
    bool getSelfTestResultGyro();
 
    char triggeredIntALL();
    bool triggeredACC_NM();
    bool triggeredACC_AM();
    bool triggeredACC_HIGH_G();
    bool triggeredGYR_HIGH_RATE();
    bool triggeredGYRO_AM();
 
    bool isSystemClockFixed();
 
    char getSystemStatus();
    char getSystemError();
 
    char getUNIT_SEL();
    char setUNIT_SEL(char selectValue);
    char setUNIT_AccUnit(bool isMeterPerSec2=true);
    char setUNIT_GyroUnit(bool isDps=true);
    char setUNIT_EulerUnit(bool isDegrees=true);
    char setUNIT_Temperature(bool isCelsius=true);
    char setUNIT_OrientationMode(bool ori_Android=true);
 
    char getOperationMode();
    char setOperationMode(char modeValue);
    char setOperation_CONFIG();
    char setOperation_ACCONRY();
    char setOperation_MAGONRY();
    char setOperation_GYROONRY();
    char setOperation_ACCMAG();
    char setOperation_ACCGYRO();
    char setOperation_MAGGYRO();
    char setOperation_AMG();
    char setOperation_Fusion_IMU();
    char setOperation_Fusion_COMPASS();
    char setOperation_Fusion_M4G();
    char setOperation_Fusion_NDOF_FMC_OFF();
    char setOperation_Fusion_NDOF();
 
    char getPowerMode();
    char setPowerMode(unsigned char modeValue);
    char setPowerMode_Normal();
    char setPowerMode_LowPower();
    char setPowerMode_Suspend();
 
    char setSysTrigger(char regVal);
    char setSys_ExternalCrystal(bool isExternal=true);
    char resetInterrupt();
    char soft_reset();
    char execSelfTest();
 
    char getTempSource();
    char setTempSource(bool Accelerometer=true);
 
    char getAxisMapConfig();
    char setAxisMapConfig(char val);
    char getAxisMapSign();
    char setAxisMapSign(char val);
    char setAxisRemap_topview_topleft();
    char setAxisRemap_topview_topright();
    char setAxisRemap_topview_bottomleft();
    char setAxisRemap_topview_bottomright();
    char setAxisRemap_bottomview_topleft();
    char setAxisRemap_bottomview_topright();
    char setAxisRemap_bottomview_bottomleft();
    char setAxisRemap_bottomview_bottomright();
    char getAxisRemap_type();
 
    void getAccOffsetAll(float &offsetX, float &offsetY, float &offsetZ);
    float getAccOffsetX();
    float getAccOffsetY();
    float getAccOffsetZ();
    char setAccOffsetAll(float offsetX, float offsetY, float offsetZ);
    char setAccOffsetX(float offset);
    char setAccOffsetY(float offset);
    char setAccOffsetZ(float offset);
 
    void getMagOffsetAll(float &offsetX, float &offsetY, float &offsetZ);
    float getMagOffsetX();
    float getMagOffsetY();
    float getMagOffsetZ();
    char setMagOffsetAll(float offsetX, float offsetY, float offsetZ);
    char setMagOffsetX(float offset);
    char setMagOffsetY(float offset);
    char setMagOffsetZ(float offset);
 
    void getGyroOffsetAll(float &offsetX, float &offsetY, float &offsetZ);
    float getGyroOffsetX();
    float getGyroOffsetY();
    float getGyroOffsetZ();
    char setGyroOffsetAll(float offsetX, float offsetY, float offsetZ);
    char setGyroOffsetX(float offset);
    char setGyroOffsetY(float offset);
    char setGyroOffsetZ(float offset);
 
    short getAccRadius();
    char setAccRadius(short LSB);
 
    short getMagRadius();
    char setMagRadius(short LSB);
 
    char getAccConfig();
    char setAccConfig(char regVal);
    char setAccConfig(char gRange, char bandWidth, char powMode);
    char setAccRange(unsigned char G);
 
    char getMagConfig();
    char setMagConfig(char regVal);
    char setMagConfig(char rate, char oprMode, char powMode);
 
    char getGyroConfig_0();
    char setGyroConfig_0(char regVal);
    char setGyroConfig_0(char range, char bandWidth);
    char getGyroConfig_1();
    char setGyroConfig_1(char powMode);
    char setGyroRange(unsigned short dps);
 
    char getAccSleepConfig();
    char setAccSleepConfig(char regVal);
    char setAccSleepConfig(char duration, char mode);
 
    char getGyroSleepConfig();
    char setGyroSleepConfig(char regVal);
    char setGyroSleepConfig(char autoSleepDuration, char duration);
 
    char getInterruptMask();
    char setInterruptMask(char mask);
 
    char getInterruptEnable();
    char setInterruptEnable(char mask);
 
    float getAccAnyMotionThreashold(bool ismg=true);
    char setAccAnyMotionThreashold(bool ismg, float threashold);
 
    char getAccInterruptSettings();
    char setAccInterruptSettings(char settings);
 
    unsigned short getAccHighGduration();
    char setAccHighGduration(short ms);
 
    float getAccHighGThreashold(bool ismg=true);
    char setAccHighGThreashold(bool ismg, float threashold);
 
    float getAccNMThreashold(bool ismg=true);
    char setAccNMThreashold(bool ismg, float threashold);
 
    char getAccNMsetting();
    char setAccNMsetting(char setting);
 
    char getGyroInterruptSettings();
    char setGyroInterruptSettings(char settings);
 
    char getGyroHighRateXsetting();
    void getGyroHighRateXsetting_dps(float &hyst, float &thres);
    char setGyroHighRateXsetting(char setting);
    char setGyroHighRateXsetting_dps(float hystVal, float thresVal);
    float getGyroHighRateXduration();
    char setGyroHighRateXduration(float duration);
 
    char getGyroHighRateYsetting();
    void getGyroHighRateYsetting_dps(float &hyst, float &thres);
    char setGyroHighRateYsetting(char setting);
    char setGyroHighRateYsetting_dps(float hystVal, float thresVal);
    float getGyroHighRateYduration();
    char setGyroHighRateYduration(float duration);
 
    char getGyroHighRateZsetting();
    void getGyroHighRateZsetting_dps(float &hyst, float &thres);
    char setGyroHighRateZsetting(char setting);
    char setGyroHighRateZsetting_dps(float hystVal, float thresVal);
    float getGyroHighRateZduration();
    char setGyroHighRateZduration(float duration);
 
    float getGyroAnyMotionThreashold();
    char setGyroAnyMotionThreashold(float threashold);
 
    char getAccAnyMotionSetting();
    char setAccAnyMotionSetting(char setting);
};
 
#endif