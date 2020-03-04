#include "BNO055.h"
 
//CLASS:BNO055_CTRL//////////////////////////////////////////////////
/* ------------------------------------------------------------------
 * BNO055_UART_CTRLクラスとBNO055_I2C_CTRLクラスの基底クラス(インターフェース)
 * UARTとI2Cをヘッダ書き換えなしで実現するために無理するとこうなった
 */
 
/* ==================================================================
 * デフォルトコンストラクタ
 */
BNO055_CTRL::BNO055_CTRL(){
    lastError = 0;
    lastLength = 0;
}
 
/* ==================================================================
 * デフォルトデストラクタ
 */
BNO055_CTRL::~BNO055_CTRL(){}
 
/* ==================================================================
 * 現在のページIDを取得する
 */
char BNO055_CTRL::getNowPage(){
    return page1 ? 1 : 0;
}
 
/* ==================================================================
 * UARTまたはI2Cで取得した最後のエラーを取得する：通信がうまくいかないときの原因追及
 */
char BNO055_CTRL::getLastError(){
    return lastError;
}
 
/* ==================================================================
 * UARTまたはI2Cで通信した際の受信バイト数を取得する：通信がうまくいかないときの原因追及
 */
char BNO055_CTRL::getLastLength(){
    return lastLength;
}
 
/* ==================================================================
 * 未実装関数(子クラスで実装される)
 */
void BNO055_CTRL::init(){}
char BNO055_CTRL::rr(bool isPage1, char regAddr){return 0;}
char BNO055_CTRL::rrc(bool isPage1, char startRegAddr, unsigned char *receiveBytes, char length){return 0;}
char BNO055_CTRL::wr(bool isPage1, char regAddr, char wBytes){return 0;}
char BNO055_CTRL::wrc(bool isPage1, char startRegAddr, char *Bytes, char length){return 0;}
 
 
 
 
 
 
 
 
 
 
//CLASS:BNO055_UART_CTRL/////////////////////////////////////////////
/* ------------------------------------------------------------------
 * BNO055_CTRLクラス(インターフェース)を継承(実装)したクラス
 * UARTで命令を送受信するためのコントロール用クラス
 */
 
/* ==================================================================
 * BNO055をUARTでコントロールするためのクラス：コンストラクタ
 */
BNO055_UART_CTRL::BNO055_UART_CTRL(RawSerial *uart){
    iface = uart;
    rxd = 0xFFFF;
    read_mark = true;
 
    page1 = true;
 
    ary = new char[BNO055_UART_BUF_MAXLEN + 5];
    memset(ary, 0, BNO055_UART_BUF_MAXLEN + 5);
    lastError = 0;
}
 
/* ==================================================================
 * BNO055をUARTでコントロールするためのクラス：デストラクタ
 */
BNO055_UART_CTRL::~BNO055_UART_CTRL(){
    delete iface;
}
 
/* ==================================================================
 * UART受信割り込み用関数
 * RX受信トリガがONになると、カウンタを加算する
 */
void BNO055_UART_CTRL::rxInterrupt(){
    if(read_mark){
        rxd = iface->getc();
        read_mark = false;
    }
}
 
/* ==================================================================
 * <UART>
 * レジスタの内容を読み取り(1byteのみ)
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * それ以外     成功した際に取得したデータ
 */
char BNO055_UART_CTRL::rr(bool isPage1, char regAddr){
    //ページが異なるならページ変更命令を発行
    if(page1 != isPage1){
        wr(page1, BNO055_PAGE_ID, (isPage1) ? 1 : 0);
        page1 = isPage1;
    }
 
    //送信可能になるまで待つ
    do{wait_ms(1);}while(!iface->writeable());
 
    //コマンドをセット
    ary[0] = 0xAA;      //StartByte(固定)
    ary[1] = 0x01;      //読み取り
    ary[2] = regAddr;   //レジスタアドレス
    ary[3] = 1;         //バイト長
 
    //送信
    iface->putc(ary[0]);
    iface->putc(ary[1]);
    iface->putc(ary[2]);
    iface->putc(ary[3]);
 
    //レスポンスを受信するまで待つ
    while(read_mark) wait_us(100);
 
    lastError = 0;
    memset(ary, 0, BNO055_UART_BUF_MAXLEN + 5); //配列ゼロクリア
 
    ary[0] = rxd & 0xFF;
 
    //残りを受信
    int i = 1;
    int cnt = (ary[0] == 0xBB) ? 2 : 1;
    while(i < cnt && iface->readable()){
        ary[i++] = iface->getc();
    }
 
    //レスポンスが0xBB以外:通信失敗
    if(ary[0] != 0xBB){
        lastLength = 2;
        lastError = ary[1];
        //受信用割り込みマークをリセット
        rxd = 0xFFFF;
        read_mark = true;
        return -1;
    }
 
    lastLength = ary[1] + 2;
 
    //受信用割り込みマークをリセット
    rxd = 0xFFFF;
    read_mark = true;
 
    //通信成功時、取得データを返す
    return ary[2];
}
 
/* ==================================================================
 * <UART>
 * レジスタの内容を読み取り(複数可)
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * -2           返答バイト不一致
 * -4           レスポンスエラー
 * それ以外     成功した際の戻り値バイト数
 */
char BNO055_UART_CTRL::rrc(bool isPage1, char startRegAddr, unsigned char *receiveBytes, char length){
    //読み取りバイト数が1未満またはBNO055_UART_BUF_MAXLEN以上はバッファが足りないので読み取れない
    if(length < 1 || length > BNO055_UART_BUF_MAXLEN) return -1;
 
    //ページが異なるならページ変更命令を発行
    if(page1 != isPage1){
        wr(page1, BNO055_PAGE_ID, (isPage1) ? 1 : 0);
        page1 = isPage1;
    }
 
    //送信可能になるまで待つ
    do{wait_ms(1);}while(!iface->writeable());
 
    //コマンドをセット
    ary[0] = 0xAA;          //StartByte(固定)
    ary[1] = 0x01;          //読み取り
    ary[2] = startRegAddr;  //読み取り開始レジスタアドレス
    ary[3] = length;        //バイト長
 
    //送信
    iface->putc(ary[0]);
    iface->putc(ary[1]);
    iface->putc(ary[2]);
    iface->putc(ary[3]);
 
    //レスポンスを受信するまで待つ
    while(read_mark) wait_us(100);
 
    lastError = 0;
    memset(ary, 0, BNO055_UART_BUF_MAXLEN + 5); //配列ゼロクリア
 
    ary[0] = rxd & 0xFF;
 
    //残りを受信
    int i = 1;
    int cnt = (ary[0] == 0xBB) ? 2 : 1;
    while(i < cnt && iface->readable()){
        ary[i++] = iface->getc();
    }
 
    //レスポンスが0xBB以外:通信失敗
    if(ary[0] != 0xBB){
        lastLength = 2;
        lastError = ary[1];
        rxd = 0xFFFF;
        read_mark = true;
        return -1;
    }
 
    //返答バイト長がlengthと一致しない:通信失敗
    if(ary[1] != length){
        lastLength = ary[1];
        lastError = 0;
        //受信用割り込みマークをリセット
        rxd = 0xFFFF;
        read_mark = true;
        return -2;
    }
 
    lastLength = ary[1] + 2;
 
    memcpy(receiveBytes, ary+2, ary[1]);
 
    //受信用割り込みマークをリセット
    rxd = 0xFFFF;
    read_mark = true;
 
    return ary[1];
}
 
/* ==================================================================
 * <UART>
 * レジスタ書き込み(1byteのみ)
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BNO055_UART_CTRL::wr(bool isPage1, char regAddr, char wBytes){
    //ページが異なるならページ変更命令を発行(再帰処理)
    if(page1 != isPage1){
        wr(page1, BNO055_PAGE_ID, (isPage1) ? 1 : 0);
        page1 = isPage1;
    }
 
    //送信可能になるまで待つ
    do{wait_ms(1);}while(!iface->writeable());
 
    //コマンドをセット
    ary[0] = 0xAA;      //StartByte(固定)
    ary[1] = 0x00;      //書き込み
    ary[2] = regAddr;   //レジスタアドレス
    ary[3] = 1;         //バイト長
    ary[4] = wBytes;    //送信データ
 
    //送信
    iface->putc(ary[0]);
    iface->putc(ary[1]);
    iface->putc(ary[2]);
    iface->putc(ary[3]);
    iface->putc(ary[4]);
 
    //システムリブートが発生するレジスタの場合は1200ms待つ
    if(regAddr == 0x3F) wait_ms(1200);
 
    //レスポンスを受信するまで待つ
    while(read_mark) wait_us(100);
 
    lastError = 0;
    memset(ary, 0, BNO055_UART_BUF_MAXLEN + 5); //配列ゼロクリア
 
    ary[0] = rxd & 0xFF;
 
    //残りを受信
    while(iface->readable()){
        ary[1] = iface->getc();
    }
 
    //レスポンスが0xEE以外もしくはステータスが0x01以外:書き込み失敗
    if(ary[0] != 0xEE || ary[1] != 0x01){
        lastLength = 2;
        lastError = ary[1];
        //受信用割り込みマークをリセット
        rxd = 0xFFFF;
        read_mark = true;
        return -1;
    }
 
    //受信用割り込みマークをリセット
    rxd = 0xFFFF;
    read_mark = true;
 
    return 1;
}
 
/* ==================================================================
 * <UART>
 * レジスタ書き込み(複数可)
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * -4           レスポンスエラー
 * 1            成功
 */
char BNO055_UART_CTRL::wrc(bool isPage1, char startRegAddr, char *Bytes, char length){
    //書き込みバイト数が1未満またはBNO055_UART_BUF_MAXLEN以上はバッファが足りないので読み取れない
    if(length < 1 || length > BNO055_UART_BUF_MAXLEN) return -1;
 
    //ページが異なるならページ変更命令を発行(再帰処理)
    if(page1 != isPage1){
        wr(page1, BNO055_PAGE_ID, (isPage1) ? 1 : 0);
        page1 = isPage1;
    }
 
    //送信可能になるまで待つ
    do{wait_ms(1);}while(!iface->writeable());
 
    //コマンドをセット
    ary[0] = 0xAA;          //StartByte(固定)
    ary[1] = 0x00;          //書き込み
    ary[2] = startRegAddr;  //レジスタアドレス
    ary[3] = length;        //バイト長
 
    //前部分送信
    iface->putc(ary[0]);
    iface->putc(ary[1]);
    iface->putc(ary[2]);
    iface->putc(ary[3]);
 
    //データ内容送信
    for(int cnt=0; cnt<length; cnt++){
        iface->putc(Bytes[cnt]);
    }
 
    //レスポンスを受信するまで待つ
    while(read_mark) wait_us(100);
 
    lastError = 0;
    memset(ary, 0, BNO055_UART_BUF_MAXLEN + 5); //配列ゼロクリア
 
    ary[0] = rxd & 0xFF;
 
    //残りを受信
    while(iface->readable()){
        ary[1] = iface->getc();
    }
 
    //レスポンスが0xEE以外もしくはステータスが0x01以外:書き込み失敗
    if(ary[0] != 0xEE || ary[1] != 0x01){
        lastLength = 2;
        lastError = ary[1];
        //受信用割り込みマークをリセット
        rxd = 0xFFFF;
        read_mark = true;
        return -1;
    }
 
    //受信用割り込みマークをリセット
    rxd = 0xFFFF;
    read_mark = true;
 
    return 1;
}
 
/* ==================================================================
 * <UART>
 * インターフェース設定を初期化する
 */
void BNO055_UART_CTRL::init(){
    iface->format(); //8N1 = default
    iface->baud(115200);
    wait_ms(5);
    iface->attach(this, &BNO055_UART_CTRL::rxInterrupt);
 
    //送信可能になるまで待つ
    do{wait_ms(1);}while(!iface->writeable());
 
    //UARTリセット時に0xFFがつく現象および前回送信時の残りがあるため同期がとれない
    //複数回読み取りを行い、正しいレスポンス(0xBB)が返るまで送信
    iface->putc(0x01); //dummy
    iface->putc(0x01); //dummy
    iface->putc(0x01); //dummy
    iface->putc(0x01); //dummy
 
    //レスポンスを受信するまで待つ
    while(read_mark) wait_us(100);
 
    char rslt = rxd & 0xFF; //レスポンス
 
    do{
        //レスポンスが異常:残りの情報を無視
        if(rslt != 0xBB){
            while(iface->readable()){
                iface->getc();
            }
        }
 
        //受信用割り込みマークをリセット
        rxd = 0xFFFF;
        read_mark = true;
 
        //送信可能になるまで待つ
        do{wait_ms(5);}while(!iface->writeable());
 
        iface->putc(0xAA); //START
        iface->putc(0x01); //読み取り
        iface->putc(0x07); //ページID
        iface->putc(0x01); //length 1byte
 
        //レスポンスを受信するまで待つ
        while(read_mark) wait_us(100);
 
        rslt = rxd & 0xFF; //レスポンス
 
    }while(rslt != 0xBB);
 
    //残りの情報を無視
    while(iface->readable()){
        iface->getc();
    }
 
    //受信用割り込みマークをリセット
    rxd = 0xFFFF;
    read_mark = true;
}
 
 
 
 
 
 
 
 
//CLASS:BNO055_I2C_CTRL//////////////////////////////////////////////
/* ------------------------------------------------------------------
 * BNO055_CTRLクラス(インターフェース)を継承(実装)したクラス
 * I2Cで命令を送受信するためのコントロール用クラス
 */
 
/* ==================================================================
 * BNO055をI2Cでコントロールするためのクラス：コンストラクタ
 */
BNO055_I2C_CTRL::BNO055_I2C_CTRL(I2C* iic, char addr, unsigned int freq){
    iface = iic;
    i2c_writeAddr = addr << 1;
    i2c_readAddr = i2c_writeAddr + 1;
    i2c_freq = freq;
    page1 = true;
    ary = new char[2];
    memset(ary, 0, 2);
    lastError = 0;
}
 
/* ==================================================================
 * BNO055をI2Cでコントロールするためのクラス：デストラクタ
 */
BNO055_I2C_CTRL::~BNO055_I2C_CTRL(){
    delete iface;
    delete[] ary;
}
 
/* ==================================================================
 * <I2C>
 * レジスタの内容を読み取り(1byteのみ)
 */
char BNO055_I2C_CTRL::rr(bool isPage1, char regAddr){
    //ページが異なるならページ変更命令を発行
    if(page1 != isPage1){
        ary[0] = BNO055_PAGE_ID;
        ary[1] = (isPage1) ? 1 : 0;
        page1 = isPage1;
        iface->write(i2c_writeAddr, ary, 2);
    }
 
    ary[0] = regAddr;
    iface->write(i2c_writeAddr, ary, 1, true);
    iface->read(i2c_readAddr, ary, 1, false);
 
    return ary[0];
}
 
/* ==================================================================
 * <I2C>
 * レジスタの内容を読み取り(複数可)
 */
char BNO055_I2C_CTRL::rrc(bool isPage1, char startRegAddr, unsigned char *receiveBytes, char length){
    //読み取りバイト数が1未満
    if(length < 1) return -1;
 
    //ページが異なるならページ変更命令を発行
    if(page1 != isPage1){
        ary[0] = BNO055_PAGE_ID;
        ary[1] = (isPage1) ? 1 : 0;
        page1 = isPage1;
        iface->write(i2c_writeAddr, ary, 2);
    }
 
    ary[0] = startRegAddr;
    iface->write(i2c_writeAddr, ary, 1, true);
    iface->read(i2c_readAddr, (char *)receiveBytes, length, false);
 
    return receiveBytes[0];
}
 
/* ==================================================================
 * <I2C>
 * レジスタ書き込み(1byteのみ)
 */
char BNO055_I2C_CTRL::wr(bool isPage1, char regAddr, char wBytes){
    //ページが異なるならページ変更命令を発行
    if(page1 != isPage1){
        ary[0] = BNO055_PAGE_ID;
        ary[1] = (isPage1) ? 1 : 0;
        page1 = isPage1;
        iface->write(i2c_writeAddr, ary, 2);
    }
 
    ary[0] = regAddr;
    ary[1] = wBytes;
 
    iface->write(i2c_writeAddr, ary, 2);
 
    return ary[0];
}
 
/* ==================================================================
 * <I2C>
 * レジスタ書き込み(複数可)
 */
char BNO055_I2C_CTRL::wrc(bool isPage1, char startRegAddr, char *Bytes, char length){
    //書き込みバイト数が1未満
    if(length < 1) return -1;
 
    //ページが異なるならページ変更命令を発行
    if(page1 != isPage1){
        ary[0] = BNO055_PAGE_ID;
        ary[1] = (isPage1) ? 1 : 0;
        page1 = isPage1;
        iface->write(i2c_writeAddr, ary, 2);
    }
 
    ary[0] = startRegAddr;
    iface->write(i2c_writeAddr, ary, 1, true);
    iface->write(i2c_writeAddr, Bytes, length, false);
 
    return Bytes[0];
}
 
/* ==================================================================
 * <I2C>
 * インターフェース設定を初期化する
 */
void BNO055_I2C_CTRL::init(){
    iface->frequency(i2c_freq);
    wait_ms(5);
}
 
 
 
 
 
 
 
 
 
//CLASS:BOARDC_BNO055////////////////////////////////////////////////
/* ------------------------------------------------------------------
 * メインクラス
 * BNO055をUARTまたはI2Cで使用するための命令をまとめている
 * 内部にBNO055_CTRLクラスのインスタンスを持ち、コンストラクタの引数によって
 * UARTとI2Cのどちらを使用するか決定する
 */
 
/* ==================================================================
 * コンストラクタ (オーバーロード+3)
 * UARTで使用する際のコンストラクタ：ピン別名を指定する
 */
BOARDC_BNO055::BOARDC_BNO055(PinName tx, PinName rx){
    ctrl = new BNO055_UART_CTRL(new RawSerial(tx, rx));
 
    scaleACC = 0.01f; // = 1 / 100
    scaleMAG = 0.0625f; // = 1 / 16
    scaleGYRO = 0.0625f; // = 1 / 16
    scaleEuler = 0.0625f; // = 1 / 16
    scaleTEMP = 1.0f;
    scaleLIA = scaleACC;
    scaleGV = scaleACC;
    scaleQuaternion = 0.00006103515; // = 1 / (2^14)
 
    clkExt = false;
}
 
/* ==================================================================
 * コンストラクタ (オーバーロード+3)
 * UARTで使用する際のコンストラクタ：RawSerialクラスのインスタンスを指定する
 */
BOARDC_BNO055::BOARDC_BNO055(RawSerial *uart){
    ctrl = new BNO055_UART_CTRL(uart);
 
    scaleACC = 0.01f; // = 1 / 100
    scaleMAG = 0.0625f; // = 1 / 16
    scaleGYRO = 0.0625f; // = 1 / 16
    scaleEuler = 0.0625f; // = 1 / 16
    scaleTEMP = 1.0f;
    scaleLIA = scaleACC;
    scaleGV = scaleACC;
    scaleQuaternion = 0.00006103515; // = 1 / (2^14)
 
    clkExt = false;
}
 
/* ==================================================================
 * コンストラクタ (オーバーロード+3)
 * I2Cで使用する際のコンストラクタ：ピン別名を指定する
 */
BOARDC_BNO055::BOARDC_BNO055(PinName sda, PinName scl, char addr, unsigned int freq){
    ctrl = new BNO055_I2C_CTRL(new I2C(sda, scl), addr, freq);
 
    scaleACC = 0.01f; // = 1 / 100
    scaleMAG = 0.0625f; // = 1 / 16
    scaleGYRO = 0.0625f; // = 1 / 16
    scaleEuler = 0.0625f; // = 1 / 16
    scaleTEMP = 1.0f;
    scaleLIA = scaleACC;
    scaleGV = scaleACC;
    scaleQuaternion = 0.00006103515; // = 1 / (2^14)
 
    clkExt = false;
}
 
/* ==================================================================
 * コンストラクタ (オーバーロード+3)
 * I2Cで使用する際のコンストラクタ：I2Cクラスのインスタンスを指定する
 */
BOARDC_BNO055::BOARDC_BNO055(I2C* iic, char addr, unsigned int freq){
    ctrl = new BNO055_I2C_CTRL(iic, addr, freq);
 
    scaleACC = 0.01f; // = 1 / 100
    scaleMAG = 0.0625f; // = 1 / 16
    scaleGYRO = 0.0625f; // = 1 / 16
    scaleEuler = 0.0625f; // = 1 / 16
    scaleTEMP = 1.0f;
    scaleLIA = scaleACC;
    scaleGV = scaleACC;
    scaleQuaternion = 0.00006103515; // = 1 / (2^14)
 
    clkExt = false;
}
 
/* ==================================================================
 * デストラクタ
 */
BOARDC_BNO055::~BOARDC_BNO055(){
    delete ctrl;
}
 
/* ==================================================================
 * デフォルト設定での初期化
 */
char BOARDC_BNO055::initialize(bool resetIface){
    if(resetIface) ctrl->init();
 
    //CONFIGモードに設定
    setOperation_CONFIG();
 
    //外部発振子設定,セルフテストの実行,セルフテスト完了まで待つ
    setSysTrigger(0x81);
 
    //加速度センサーレンジ設定(+-8G)
    //地磁気や角速度センサーのレンジやバンドはFusionモードにより自動設定される
    if(!setAccRange(16)) return -1;
 
    //各軸リマップ設定(BNO055の1pinマークが表側右下)
    if(!setAxisRemap_topview_bottomright()) return -2;
 
    //単位系セット
    //ORI_Android_Windows : Windows
    //TEMP_Unit : Celcius
    //EUL_Unit : Degree
    //GYR_Unit : deg/s
    //ACC_Unit : m/s^2
    if(!setUNIT_SEL(0x00)) return -3;
 
    //動作モード設定(9軸Fusionモード)
    if(!setOperation_Fusion_NDOF()) return -4;
 
    return 0;
}
 
/* ==================================================================
 * 通信で発生した最後のエラー番号を返す
 */
char BOARDC_BNO055::getIfaceLastError(){
    return ctrl->getLastError();
}
 
/* ==================================================================
 * 通信で発生した最後のエラーの通信バイト数を返す
 */
char BOARDC_BNO055::getIfaceLastLength(){
    return ctrl->getLastLength();
}
 
/* ==================================================================
 * ユーザー定義読み取り(1byte)
 * レジスタを指定して値を直接読み取る
 */
char BOARDC_BNO055::customRead(bool isPage1, char regAddr){
    return ctrl->rr(isPage1, regAddr);
}
 
/* ==================================================================
 * ユーザー定義読み取り(連続)
 * レジスタを指定して値を直接読み取る
 */
char BOARDC_BNO055::customReadC(bool isPage1, char startRegAddr, unsigned char *receiveBytes, unsigned char length){
    return ctrl->rrc(isPage1, startRegAddr, receiveBytes, length);
}
 
/* ==================================================================
 * ユーザー定義書き込み(1byte)
 * レジスタを指定して値を直接書き込む
 */
char BOARDC_BNO055::customWrite(bool isPage1, char regAddr, char wBytes){
    return ctrl->wr(isPage1, regAddr, wBytes);
}
 
/* ==================================================================
 * ユーザー定義書き込み(連続)
 * レジスタを指定して値を直接書き込む
 */
char BOARDC_BNO055::customWriteC(bool isPage1, char startRegAddr, char *Bytes, unsigned char length){
    return ctrl->wrc(isPage1, startRegAddr, Bytes, length);
}
 
/* ==================================================================
 * レジスタ：ページIDの現在値を読む
 * 実際はコントロールクラスが保持している内部変数の値を返しているだけ
 */
char BOARDC_BNO055::getPage(){
    return ctrl->getNowPage();
}
 
/* ==================================================================
 * レジスタ：ページIDを変更する
 * ページIDが現在値と同じであれば何もしない
 */
void BOARDC_BNO055::setPage(unsigned char pageNo){
    pageNo = (pageNo == 0) ? 0 : 1;
    if(getPage() == pageNo) return;
    ctrl->wr((getPage() == 0) ? true : false, BNO055_PAGE_ID, pageNo);
}
 
/* ==================================================================
 * レジスタ：チップIDの値を取得する
 */
char BOARDC_BNO055::getChipID(){
    return ctrl->rr(0, BNO055P0_CHIP_ID);
}
 
/* ==================================================================
 * レジスタ：加速度センサーIDの値を取得する
 */
char BOARDC_BNO055::getAccChipID(){
    return ctrl->rr(0, BNO055P0_ACC_ID);
}
 
/* ==================================================================
 * レジスタ：地磁気センサーIDの値を取得する
 */
char BOARDC_BNO055::getMagChipID(){
    return ctrl->rr(0, BNO055P0_MAG_ID);
}
 
/* ==================================================================
 * レジスタ：ジャイロセンサーIDの値を取得する
 */
char BOARDC_BNO055::getGyroChipID(){
    return ctrl->rr(0, BNO055P0_GYR_ID);
}
 
/* ==================================================================
 * レジスタ：内部ソフトウェアリビジョンの値を取得する
 */
short BOARDC_BNO055::getRevision(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_SW_REV_ID_LSB, rsv, 2);
 
    return (rsv[1] << 8) | rsv[0];
}
 
/* ==================================================================
 * レジスタ：内部ブートリビジョンの値を取得する
 */
char BOARDC_BNO055::getBootRevision(){
    return ctrl->rr(0, BNO055P0_BL_REV_ID);
}
 
/* ==================================================================
 * 加速度センサーのRAW値に乗算する係数を取得する
 */
float BOARDC_BNO055::getAccScale(){
    return scaleACC;
}
 
/* ==================================================================
 * 地磁気センサーのRAW値に乗算する係数を取得する
 */
float BOARDC_BNO055::getMagScale(){
    return scaleMAG;
}
 
/* ==================================================================
 * 角速度センサーのRAW値に乗算する係数を取得する
 */
float BOARDC_BNO055::getGyroScale(){
    return scaleGYRO;
}
 
/* ==================================================================
 * 温度センサーのRAW値に乗算する係数を取得する
 */
float BOARDC_BNO055::getTempScale(){
    return scaleTEMP;
}
 
/* ==================================================================
 * オイラー角のRAW値に乗算する係数を取得する
 */
float BOARDC_BNO055::getEulerScale(){
    return scaleEuler;
}
 
/* ==================================================================
 * 加速度センサーの線形加速度のRAW値に乗算する係数を取得する
 */
float BOARDC_BNO055::getLinearScale(){
    return scaleLIA;
}
 
/* ==================================================================
 * 重力ベクトルのRAW値に乗算する係数を取得する
 */
float BOARDC_BNO055::getGVScale(){
    return scaleGV;
}
 
/* ==================================================================
 * 四元数のRAW値に乗算する係数を取得する
 */
double BOARDC_BNO055::getQuaternionScale(){
    return scaleQuaternion;
}
 
 
/* ==================================================================
 * 加速度センサの値を取得する
 * ------------------------------------------------------------------
 * &accX: アドレス参照引数:関数実行後、この変数にX軸の値が格納される
 * &accY: アドレス参照引数:関数実行後、この変数にY軸の値が格納される
 * &accZ: アドレス参照引数:関数実行後、この変数にZ軸の値が格納される
 */
void BOARDC_BNO055::getAccDataAll(short &accX, short &accY, short &accZ){
    //連続6byte読み取り
    unsigned char rsv[6];
    ctrl->rrc(0, BNO055P0_ACC_DATA_X_LSB, rsv, 6);
 
    accX = (rsv[1] << 8) | rsv[0];
    accY = (rsv[3] << 8) | rsv[2];
    accZ = (rsv[5] << 8) | rsv[4];
}
 
/* ==================================================================
 * 加速度センサの値(X軸のみ)を取得する
 */
short BOARDC_BNO055::getAccDataX(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_ACC_DATA_X_LSB, rsv, 2);
 
    return (rsv[1] << 8) | rsv[0];
}
 
/* ==================================================================
 * 加速度センサの値(Y軸のみ)を取得する
 */
short BOARDC_BNO055::getAccDataY(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_ACC_DATA_Y_LSB, rsv, 2);
 
    return (rsv[1] << 8) | rsv[0];
}
 
/* ==================================================================
 * 加速度センサの値(Z軸のみ)を取得する
 */
short BOARDC_BNO055::getAccDataZ(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_ACC_DATA_Z_LSB, rsv, 2);
 
    return (rsv[1] << 8) | rsv[0];
}
 
/* ==================================================================
 * 地磁気センサの値を取得する
 * ------------------------------------------------------------------
 * &magX: アドレス参照引数:関数実行後、この変数にX軸の値が格納される
 * &magY: アドレス参照引数:関数実行後、この変数にY軸の値が格納される
 * &magZ: アドレス参照引数:関数実行後、この変数にZ軸の値が格納される
 */
void BOARDC_BNO055::getMagDataAll(short &magX, short &magY, short &magZ){
    //連続6byte読み取り
    unsigned char rsv[6];
    ctrl->rrc(0, BNO055P0_MAG_DATA_X_LSB, rsv, 6);
 
    magX = (rsv[1] << 8) | rsv[0];
    magY = (rsv[3] << 8) | rsv[2];
    magZ = (rsv[5] << 8) | rsv[4];
}
 
/* ==================================================================
 * 地磁気センサの値(X軸のみ)を取得する
 */
short BOARDC_BNO055::getMagDataX(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_MAG_DATA_X_LSB, rsv, 2);
 
    return (rsv[1] << 8) | rsv[0];
}
 
/* ==================================================================
 * 地磁気センサの値(Y軸のみ)を取得する
 */
short BOARDC_BNO055::getMagDataY(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_MAG_DATA_Y_LSB, rsv, 2);
 
    return (rsv[1] << 8) | rsv[0];
}
 
/* ==================================================================
 * 地磁気センサの値(Z軸のみ)を取得する
 */
short BOARDC_BNO055::getMagDataZ(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_MAG_DATA_Z_LSB, rsv, 2);
 
    return (rsv[1] << 8) | rsv[0];
}
 
/* ==================================================================
 * ジャイロセンサの値を取得する
 * ------------------------------------------------------------------
 * &gyroX: アドレス参照引数:関数実行後、この変数にX軸の値が格納される
 * &gyroY: アドレス参照引数:関数実行後、この変数にY軸の値が格納される
 * &gyroZ: アドレス参照引数:関数実行後、この変数にZ軸の値が格納される
 */
void BOARDC_BNO055::getGyroDataAll(short &gyroX, short &gyroY, short &gyroZ){
    //連続6byte読み取り
    unsigned char rsv[6];
    ctrl->rrc(0, BNO055P0_GYR_DATA_X_LSB, rsv, 6);
 
    gyroX = (rsv[1] << 8) | rsv[0];
    gyroY = (rsv[3] << 8) | rsv[2];
    gyroZ = (rsv[5] << 8) | rsv[4];
}
 
/* ==================================================================
 * ジャイロセンサの値(X軸のみ)を取得する
 */
short BOARDC_BNO055::getGyroDataX(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_GYR_DATA_X_LSB, rsv, 2);
 
    return (rsv[1] << 8) | rsv[0];
}
 
/* ==================================================================
 * ジャイロセンサの値(Y軸のみ)を取得する
 */
short BOARDC_BNO055::getGyroDataY(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_GYR_DATA_Y_LSB, rsv, 2);
 
    return (rsv[1] << 8) | rsv[0];
}
 
/* ==================================================================
 * ジャイロセンサの値(Z軸のみ)を取得する
 */
short BOARDC_BNO055::getGyroDataZ(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_GYR_DATA_Z_LSB, rsv, 2);
 
    return (rsv[1] << 8) | rsv[0];
}
 
/* ==================================================================
 * FusionSensing:オイラー角の値を取得する
 * FusionSensing機能(内部演算機能)を使用していない場合は不定の値を返す
 * Roll軸およびPitch軸が+-45degの範囲を超える場合、Yaw軸の値は不安定となる
 * (その場合はgetEulerFromQを使用することで四元数からオイラー角を導出可能)
 * ------------------------------------------------------------------
 * &E_heading: アドレス参照引数:関数実行後、この変数にYaw軸(heading)の値が格納される
 * &E_roll: アドレス参照引数:関数実行後、この変数にroll軸の値が格納される
 * &E_pitch: アドレス参照引数:関数実行後、この変数にpitch軸の値が格納される
 */
void BOARDC_BNO055::getEulerDataAll(short &E_heading, short &E_roll, short &E_pitch){
    //連続6byte読み取り
    unsigned char rsv[6];
    ctrl->rrc(0, BNO055P0_EUL_HEADING_LSB, rsv, 6);
 
    E_heading = (rsv[1] << 8) | rsv[0];
    E_roll = (rsv[3] << 8) | rsv[2];
    E_pitch = (rsv[5] << 8) | rsv[4];
}
 
/* ==================================================================
 * FusionSensing:オイラー角の値(Yaw軸(heading)のみ)を取得する
 * FusionSensing機能(内部演算機能)を使用していない場合は不定の値を返す
 */
short BOARDC_BNO055::getEulerDataHeading(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_EUL_HEADING_LSB, rsv, 2);
 
    return (rsv[1] << 8) | rsv[0];
}
 
/* ==================================================================
 * getEulerDataHeading()のエイリアス
 * FusionSensing:オイラー角の値(Yaw軸(heading)のみ)を返す
 */
short BOARDC_BNO055::getEulerDataYaw(){
    return getEulerDataHeading();
}
 
/* ==================================================================
 * FusionSensing:オイラー角の値(pitch軸のみ)を取得する
 * FusionSensing機能(内部演算機能)を使用していない場合は不定の値を返す
 */
short BOARDC_BNO055::getEulerDataRoll(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_EUL_ROLL_LSB, rsv, 2);
 
    return (rsv[1] << 8) | rsv[0];
}
 
/* ==================================================================
 * FusionSensing:オイラー角の値(pitch軸のみ)を取得する
 * FusionSensing機能(内部演算機能)を使用していない場合は不定の値を返す
 */
short BOARDC_BNO055::getEulerDataPitch(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_EUL_PITCH_LSB, rsv, 2);
 
    return (rsv[1] << 8) | rsv[0];
}
 
/* ==================================================================
 * FusionSensing:9軸センサーのすべての値の値を取得する
 */
void BOARDC_BNO055::get9Axis(short *box){
    unsigned char rsv[18];
    ctrl->rrc(0, BNO055P0_ACC_DATA_X_LSB, rsv, 18);
 
    box[0] = (rsv[1] << 8) | rsv[0];
    box[1] = (rsv[3] << 8) | rsv[2];
    box[2] = (rsv[5] << 8) | rsv[4];
    box[3] = (rsv[7] << 8) | rsv[6];
    box[4] = (rsv[9] << 8) | rsv[8];
    box[5] = (rsv[11] << 8) | rsv[10];
    box[6] = (rsv[13] << 8) | rsv[12];
    box[7] = (rsv[15] << 8) | rsv[14];
    box[8] = (rsv[17] << 8) | rsv[16];
}
 
/* ==================================================================
 * FusionSensing:9軸センサーのすべての値とオイラー角のすべての値を取得する
 * FusionSensing機能(内部演算機能)を使用していない場合は不定の値を返す
 */
void BOARDC_BNO055::get9AxisAndEUL(short *box){
    unsigned char rsv[24];
    ctrl->rrc(0, BNO055P0_ACC_DATA_X_LSB, rsv, 24);
 
    box[0] = (rsv[1] << 8) | rsv[0];
    box[1] = (rsv[3] << 8) | rsv[2];
    box[2] = (rsv[5] << 8) | rsv[4];
    box[3] = (rsv[7] << 8) | rsv[6];
    box[4] = (rsv[9] << 8) | rsv[8];
    box[5] = (rsv[11] << 8) | rsv[10];
    box[6] = (rsv[13] << 8) | rsv[12];
    box[7] = (rsv[15] << 8) | rsv[14];
    box[8] = (rsv[17] << 8) | rsv[16];
    box[9] = (rsv[19] << 8) | rsv[18];
    box[10] = (rsv[21] << 8) | rsv[20];
    box[11] = (rsv[23] << 8) | rsv[22];
}
 
/* ==================================================================
 * FusionSensing:四元数(Quaternion)を取得する
 * FusionSensing機能(内部演算機能)を使用していない場合は不定の値を返す
 * ------------------------------------------------------------------
 * &q1, &q2, &q3, &q4:アドレス参照引数:関数実行後、この変数に四元数が格納される
 */
void BOARDC_BNO055::getQuaternion(short &q1, short &q2, short &q3, short &q4){
    //連続8byte読み取り
    unsigned char rsv[8];
    ctrl->rrc(0, BNO055P0_QUA_DATA_W_LSB, rsv, 8);
 
    q1 = (rsv[1] << 8) | rsv[0];
    q2 = (rsv[3] << 8) | rsv[2];
    q3 = (rsv[5] << 8) | rsv[4];
    q4 = (rsv[7] << 8) | rsv[6];
}
 
/* ==================================================================
 * FusionSensing:四元数(Quaternion)を取得し、オイラー角を算出して返す
 * FusionSensing機能(内部演算機能)を使用していない場合は不定の値を返す
 * 出力数値範囲 -180.0 to 180.0[deg] CW(時計回り)- CCW(反時計回り)+
 * ------------------------------------------------------------------
 * &E_heading, &E_roll, &E_pitch:アドレス参照引数:関数実行後、この変数にオイラー角が格納される
 */
void BOARDC_BNO055::getEulerFromQ(double &E_heading, double &E_roll, double &E_pitch){
    //連続8byte読み取り
    unsigned char rsv[8];
    ctrl->rrc(0, BNO055P0_QUA_DATA_W_LSB, rsv, 8);
 
    //四元数を実際の数値に変換(1 / 2^14を掛けるより2^14で割ったほうが早い・・・)
    double q1, q2, q3, q4;
    q1 = (double)((short)((rsv[1] << 8) | rsv[0]) / 16384.0);
    q2 = (double)((short)((rsv[3] << 8) | rsv[2]) / 16384.0);
    q3 = (double)((short)((rsv[5] << 8) | rsv[4]) / 16384.0);
    q4 = (double)((short)((rsv[7] << 8) | rsv[6]) / 16384.0);
    
    //四元数からオイラー角に変換
    //ref: Wikipedia
    //Conversion between quaternions and Euler angles(Quaternion to Euler Angles Conversion)
    double q3q3 = q3 * q3;
    
    double m1 = +2.0 * (q1 * q2 + q3 * q4);
    double m2 = +1.0 - 2.0 * (q2 * q2 + q3q3);
    E_roll = atan2(m1, m2) * 57.2957795131;
    
    m1 = +2.0 * (q1 * q3 - q4 * q2);
    m1 = (m1 > 1.0)? 1.0 : m1;
    m1 = (m1 < -1.0)? -1.0 : m1;
    E_pitch = asin(m1) * 57.2957795131;
    
    m1 = +2.0 * (q1 * q4 + q2 * q3);
    m2 = +1.0 - 2.0 * (q3q3 + q4 * q4);
    E_heading = atan2(m1, m2) * 57.2957795131;
}
 
/* ==================================================================
 * 線形加速度(LinearAcceleration)での加速度センサの値を取得する
 * ------------------------------------------------------------------
 * &L_accX: アドレス参照引数:関数実行後、この変数にX軸の値が格納される
 * &L_accY: アドレス参照引数:関数実行後、この変数にY軸の値が格納される
 * &L_accZ: アドレス参照引数:関数実行後、この変数にZ軸の値が格納される
 */
void BOARDC_BNO055::getLinearAccDataAll(short &L_accX, short &L_accY, short &L_accZ){
    //連続6byte読み取り
    unsigned char rsv[6];
    ctrl->rrc(0, BNO055P0_LIA_DATA_X_LSB, rsv, 6);
 
    L_accX = (rsv[1] << 8) | rsv[0];
    L_accY = (rsv[3] << 8) | rsv[2];
    L_accZ = (rsv[5] << 8) | rsv[4];
}
 
/* ==================================================================
 * 線形加速度(LinearAcceleration)での加速度センサの値(X軸のみ)を取得する
 */
short BOARDC_BNO055::getLinearAccDataX(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_LIA_DATA_X_LSB, rsv, 2);
 
    return (rsv[1] << 8) | rsv[0];
}
 
/* ==================================================================
 * 線形加速度(LinearAcceleration)での加速度センサの値(Y軸のみ)を取得する
 */
short BOARDC_BNO055::getLinearAccDataY(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_LIA_DATA_Y_LSB, rsv, 2);
 
    return (rsv[1] << 8) | rsv[0];
}
 
/* ==================================================================
 * 線形加速度(LinearAcceleration)での加速度センサの値(Z軸のみ)を取得する
 */
short BOARDC_BNO055::getLinearAccDataZ(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_LIA_DATA_Z_LSB, rsv, 2);
 
    return (rsv[1] << 8) | rsv[0];
}
 
/* ==================================================================
 * 重力ベクトル情報を取得する
 * ------------------------------------------------------------------
 * &gvX: アドレス参照引数:関数実行後、この変数にX軸の値が格納される
 * &gvY: アドレス参照引数:関数実行後、この変数にY軸の値が格納される
 * &gvZ: アドレス参照引数:関数実行後、この変数にZ軸の値が格納される
 */
void BOARDC_BNO055::getGVectorDataAll(short &gvX, short &gvY, short &gvZ){
    //連続6byte読み取り
    unsigned char rsv[6];
    ctrl->rrc(0, BNO055P0_GRV_DATA_X_LSB, rsv, 6);
 
    gvX = (rsv[1] << 8) | rsv[0];
    gvY = (rsv[3] << 8) | rsv[2];
    gvZ = (rsv[5] << 8) | rsv[4];
}
 
/* ==================================================================
 * 重力ベクトル情報(X軸のみ)を取得する
 */
short BOARDC_BNO055::getGVectorDataX(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_GRV_DATA_X_LSB, rsv, 2);
 
    return (rsv[1] << 8) | rsv[0];
}
 
/* ==================================================================
 * 重力ベクトル情報(Y軸のみ)を取得する
 */
short BOARDC_BNO055::getGVectorDataY(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_GRV_DATA_Y_LSB, rsv, 2);
 
    return (rsv[1] << 8) | rsv[0];
}
 
/* ==================================================================
 * 重力ベクトル情報(Z軸のみ)を取得する
 */
short BOARDC_BNO055::getGVectorDataZ(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_GRV_DATA_Z_LSB, rsv, 2);
 
    return (rsv[1] << 8) | rsv[0];
}
 
/* ==================================================================
 * センサー内部温度情報を取得する
 * 内部温度のセンサーは2つあり、どちらか一方のみの温度を返す
 * (設定レジスタTEMP_SOURCEにて選択可)
 * (setTempSource(bool)にて選択可)
 */
char BOARDC_BNO055::getTemperature(){
    return ctrl->rr(0, BNO055P0_TEMP);
}
 
/* ==================================================================
 * キャリブレーション(補正)の状態を取得する
 * ------------------------------------------------------------------
 * &sys: アドレス参照引数:関数実行後、この変数にsystem補正の状態が格納される
 * &acc: アドレス参照引数:関数実行後、この変数に加速度センサー補正の状態が格納される
 * &mag: アドレス参照引数:関数実行後、この変数に地磁気センサー補正の状態が格納される
 * &gyro: アドレス参照引数:関数実行後、この変数に角速度センサー補正の状態が格納される
 * ------------------------------------------------------------------
 * 各補正情報は0 - 100[%]の百分率で返答される。それぞれ2bitであるため
 * 値は0[%],32[%],66[%],100[%]のいずれか
 */
void BOARDC_BNO055::getCalibStatusAll(char &sys, char &acc, char &mag, char &gyro){
    char rv = ctrl->rr(0, BNO055P0_CALIB_STAT);
    sys = (((rv & 0xC0) >> 6) * 34);
    gyro = (((rv & 0x30) >> 4) * 34);
    acc = (((rv & 0x0C) >> 2) * 34);
    mag = ((rv & 0x03) * 34);
    sys -= (sys == 0) ? 0 : 2;
    gyro -= (gyro == 0) ? 0 : 2;
    acc -= (acc == 0) ? 0 : 2;
    mag -= (mag == 0) ? 0 : 2;
}
 
/* ==================================================================
 * キャリブレーション(補正)の状態(systemのみ)を取得する
 * ------------------------------------------------------------------
 * 各補正情報は0 - 100[%]の百分率で返答される。それぞれ2bitであるため
 * 値は0[%],32[%],66[%],100[%]のいずれか
 */
char BOARDC_BNO055::getCalibStatusSys(){
    char ret = (((ctrl->rr(0, BNO055P0_CALIB_STAT) & 0xC0) >> 6) * 34) - 2;
    return (ret < 0) ? 0 : ret;
}
 
/* ==================================================================
 * キャリブレーション(補正)の状態(加速度センサーのみ)を取得する
 * ------------------------------------------------------------------
 * 各補正情報は0 - 100[%]の百分率で返答される。それぞれ2bitであるため
 * 値は0[%],32[%],66[%],100[%]のいずれか
 */
char BOARDC_BNO055::getCalibStatusAcc(){
    char ret = (((ctrl->rr(0, BNO055P0_CALIB_STAT) & 0x0C) >> 2) * 34) - 2;
    return (ret < 0) ? 0 : ret;
}
 
/* ==================================================================
 * キャリブレーション(補正)の状態(地磁気センサーのみ)を取得する
 * ------------------------------------------------------------------
 * 各補正情報は0 - 100[%]の百分率で返答される。それぞれ2bitであるため
 * 値は0[%],32[%],66[%],100[%]のいずれか
 */
char BOARDC_BNO055::getCalibStatusMag(){
    char ret = ((ctrl->rr(0, BNO055P0_CALIB_STAT) & 0x03) * 34) - 2;
    return (ret < 0) ? 0 : ret;
}
 
/* ==================================================================
 * キャリブレーション(補正)の状態(角速度センサーのみ)を取得する
 * ------------------------------------------------------------------
 * 各補正情報は0 - 100[%]の百分率で返答される。それぞれ2bitであるため
 * 値は0[%],32[%],66[%],100[%]のいずれか
 */
char BOARDC_BNO055::getCalibStatusGyro(){
    char ret = (((ctrl->rr(0, BNO055P0_CALIB_STAT) & 0x30) >> 4) * 34) - 2;
    return (ret < 0) ? 0 : ret;
}
 
/* ==================================================================
 * システムおよびセンサーのセルフテストの実行結果を取得する
 * ------------------------------------------------------------------
 * returns:
 * 1bit目:加速度センサーのセルフテストの実行結果(0:failed(異常), 1:passed(正常))
 * 2bit目:地磁気センサーのセルフテストの実行結果(0:failed(異常), 1:passed(正常))
 * 3bit目:角速度センサーのセルフテストの実行結果(0:failed(異常), 1:passed(正常))
 * 4bit目:内部マイコンのセルフテストの実行結果(0:failed(異常), 1:passed(正常))
 * ------------------------------------------------------------------
 * 例:戻り値 0x0D (0b00001101)
 * >>加速度センサー:正常(1)
 * >>地磁気センサー:異常(0)
 * >>角速度センサー：正常(1)
 * >>内部マイコン:正常(1)
 */
char BOARDC_BNO055::getSelfTestResultAll(){
    return ctrl->rr(0, BNO055P0_ST_RESULT);
}
 
/* ==================================================================
 * 内部マイコン(BNO055のMCU)のセルフテストの実行結果を取得する
 * ------------------------------------------------------------------
 * returns:
 * false:failed(異常)
 * true:passed(正常)
 */
bool BOARDC_BNO055::getSelfTestResultMCU(){
    return (((ctrl->rr(0, BNO055P0_ST_RESULT) & 0x08) >> 3) == 1);
}
 
/* ==================================================================
 * 加速度センサーのセルフテストの実行結果を取得する
 * ------------------------------------------------------------------
 * returns:
 * false:failed(異常)
 * true:passed(正常)
 */
bool BOARDC_BNO055::getSelfTestResultAcc(){
    return ((ctrl->rr(0, BNO055P0_ST_RESULT) & 0x01) == 1);
}
 
/* ==================================================================
 * 地磁気センサーのセルフテストの実行結果を取得する
 * ------------------------------------------------------------------
 * returns:
 * false:failed(異常)
 * true:passed(正常)
 */
bool BOARDC_BNO055::getSelfTestResultMag(){
    return (((ctrl->rr(0, BNO055P0_ST_RESULT) & 0x02) >> 1) == 1);
}
 
/* ==================================================================
 * 地磁気センサーのセルフテストの実行結果を取得する
 * ------------------------------------------------------------------
 * returns:
 * false:failed(異常)
 * true:passed(正常)
 */
bool BOARDC_BNO055::getSelfTestResultGyro(){
    return (((ctrl->rr(0, BNO055P0_ST_RESULT) & 0x04) >> 2) == 1);
}
 
/* ==================================================================
 * 発生している割り込みステータス情報を取得する
 * ------------------------------------------------------------------
 * returns:
 * 3bit目:GYRO_AM(角速度AnyMotion発生)(1:発生中, 0:なし)
 * 4bit目:GYR_HIGH_RATE(角速度ハイレート発生)(1:発生中, 0:なし)
 * 6bit目:ACC_HIGH_G(加速度急加速発生)(1:発生中, 0:なし)
 * 7bit目:ACC_AM(角速度AnyMotion発生)(1:発生中, 0:なし)
 * 8bit目:ACC_NM(角速度NoMotion発生)(1:発生中, 0:なし)
 */
char BOARDC_BNO055::triggeredIntALL(){
    return ctrl->rr(0, BNO055P0_INT_STA);
}
 
/* ==================================================================
 * 発生している割り込みステータス情報(ACC_NM)を取得する
 * ------------------------------------------------------------------
 * returns:
 * true:ACC_NMトリガー発生中
 * false:ACC_NMトリガーなし
 */
bool BOARDC_BNO055::triggeredACC_NM(){
    return (((ctrl->rr(0, BNO055P0_INT_STA) & 0x80) >> 7) == 1);
}
 
/* ==================================================================
 * 発生している割り込みステータス情報(ACC_AM)を取得する
 * ------------------------------------------------------------------
 * returns:
 * true:ACC_AMトリガー発生中
 * false:ACC_AMトリガーなし
 */
bool BOARDC_BNO055::triggeredACC_AM(){
    return (((ctrl->rr(0, BNO055P0_INT_STA) & 0x40) >> 6) == 1);
}
 
/* ==================================================================
 * 発生している割り込みステータス情報(ACC_HIGH_G)を取得する
 * ------------------------------------------------------------------
 * returns:
 * true:ACC_HIGH_Gトリガー発生中
 * false:ACC_HIGH_Gトリガーなし
 */
bool BOARDC_BNO055::triggeredACC_HIGH_G(){
    return (((ctrl->rr(0, BNO055P0_INT_STA) & 0x20) >> 5) == 1);
}
 
/* ==================================================================
 * 発生している割り込みステータス情報(GYR_HIGH_RATE)を取得する
 * ------------------------------------------------------------------
 * returns:
 * true:GYR_HIGH_RATEトリガー発生中
 * false:GYR_HIGH_RATEトリガーなし
 */
bool BOARDC_BNO055::triggeredGYR_HIGH_RATE(){
    return (((ctrl->rr(0, BNO055P0_INT_STA) & 0x08) >> 3) == 1);
}
 
/* ==================================================================
 * 発生している割り込みステータス情報(GYRO_AM)を取得する
 * ------------------------------------------------------------------
 * returns:
 * true:GYRO_AMトリガー発生中
 * false:GYRO_AMトリガーなし
 */
bool BOARDC_BNO055::triggeredGYRO_AM(){
    return (((ctrl->rr(0, BNO055P0_INT_STA) & 0x04) >> 2) == 1);
}
 
/* ==================================================================
 * BNO055のシステムクロック固定情報を取得する
 * ------------------------------------------------------------------
 * returns:
 * true:設定によって固定されている(SYS_TRIGGEレジスタのCLK_SEL)
 * false:設定されていない(内部か外部選択可)
 * ------------------------------------------------------------------
 * setSys_ExternalCrystal(bool)で設定可
 */
bool BOARDC_BNO055::isSystemClockFixed(){
    return (ctrl->rr(0, BNO055P0_SYS_CLK_STATUS) == 1);
}
 
/* ==================================================================
 * BNO055のシステムステータスを取得する
 * ------------------------------------------------------------------
 * returns:
 * 0: システム待機状態
 * 1: システムエラー
 * 2: ペリフェラル初期化中
 * 3: システム初期化中
 * 4: セルフテスト実行中
 * 5: 起動中(Fusionアルゴリズム起動中)
 * 6: 起動中(Fusionアルゴリズムなし)
 */
char BOARDC_BNO055::getSystemStatus(){
    return ctrl->rr(0, BNO055P0_SYS_STATUS);
}
 
/* ==================================================================
 * BNO055のシステムエラー情報を取得する
 * ------------------------------------------------------------------
 * returns:
 * 0: エラーなし
 * 1: ペリフェラル初期化エラー
 * 2: システム初期化エラー
 * 3: セルフテスト結果不調
 * 4: レジスタマップエラー(値の範囲外)
 * 5: レジスタマップエラー(アドレスの範囲外)
 * 6: レジスタマップエラー(書き込み不能)
 * 7: BNO005LowPowerモードにつき指定モード実行不可
 * 8: 加速度センサーPowerMode実行不可
 * 9: Fusionアルゴリズム設定エラー
 * 10: 各センサー設定エラー
 */
char BOARDC_BNO055::getSystemError(){
    return ctrl->rr(0, BNO055P0_SYS_ERR);
}
 
/* ==================================================================
 * システムの設定単位系を取得する
 * ------------------------------------------------------------------
 * returns:
 * 1bit目: 加速度センサー単位系(0:m/s^2, 1:mg)
 * 2bit目: 角速度センサー単位系(0:deg/s, 1:rad/s)
 * 3bit目: オイラー角単位系(0:deg, 1:rad)
 * 5bit目: 温度単位系(0:摂氏, 1:華氏)
 * 7bit目: 出力設定(0:Windows, 1:Android)
 */
char BOARDC_BNO055::getUNIT_SEL(){
    return ctrl->rr(0, BNO055P0_UNIT_SEL);
}
 
/* ==================================================================
 * システムの設定単位系を設定する
 * ------------------------------------------------------------------
 * 引数selectValue:
 * 1bit目: 加速度センサー単位系(0:m/s^2, 1:mg)
 * 2bit目: 角速度センサー単位系(0:deg/s, 1:rad/s)
 * 3bit目: オイラー角単位系(0:deg, 1:rad)
 * 5bit目: 温度単位系(0:摂氏, 1:華氏)
 * 7bit目: 出力設定(0:Windows, 1:Android)
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setUNIT_SEL(char selectValue){
    if(ctrl->wr(0, BNO055P0_UNIT_SEL, selectValue) == -1) return -1;
 
    scaleACC = ((selectValue & 0x01) == 0) ? 0.01f : 1.0f;
    scaleGYRO = ((selectValue & 0x02) == 0) ? 0.0625f : 0.001111111111f;
    scaleEuler = ((selectValue & 0x04) == 0) ? 0.0625f : 0.001111111111f;
    scaleTEMP = ((selectValue & 0x10) == 0) ? 1.0f : 2.0f;
    scaleLIA = scaleACC;
    scaleGV = scaleACC;
 
    return 1;
}
 
/* ==================================================================
 * システムの設定単位系(加速度センサー)を設定する
 * ------------------------------------------------------------------
 * 引数isMeterPerSec2:
 * true: m/s^2
 * false: mg
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setUNIT_AccUnit(bool isMeterPerSec2){
    char val = getUNIT_SEL() & 0xFE;
    if(!isMeterPerSec2) val += 1;
    return setUNIT_SEL(val);
}
 
/* ==================================================================
 * システムの設定単位系(角速度センサー)を設定する
 * ------------------------------------------------------------------
 * 引数isDps:
 * true: dps(Degrees/s)
 * false: rps(Radians/s)
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setUNIT_GyroUnit(bool isDps){
    char val = getUNIT_SEL() & 0xFD;
    if(!isDps) val += 2;
    return setUNIT_SEL(val);
}
 
/* ==================================================================
 * システムの設定単位系(オイラー角)を設定する
 * ------------------------------------------------------------------
 * 引数isDegrees:
 * true: Degrees
 * false: Radians
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setUNIT_EulerUnit(bool isDegrees){
    char val = getUNIT_SEL() & 0xFB;
    if(!isDegrees) val += 4;
    return setUNIT_SEL(val);
}
 
/* ==================================================================
 * システムの設定単位系(温度)を設定する
 * ------------------------------------------------------------------
 * 引数isCelsius:
 * true: 摂氏(Celsius)
 * false: 華氏(Fahrenheit)
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setUNIT_Temperature(bool isCelsius){
    char val = getUNIT_SEL() & 0xEF;
    if(!isCelsius) val += 16;
    return setUNIT_SEL(val);
}
 
/* ==================================================================
 * システムの設定単位系(出力設定)を設定する
 * ------------------------------------------------------------------
 * 引数ori_Android:
 * true: Android用(Pitch角:+180 to -180 反時計回り＋)
 * false: Windows用(Pitch角:-180 to +180 時計回り＋)
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setUNIT_OrientationMode(bool ori_Android){
    char val = getUNIT_SEL() & 0xBF;
    if(!ori_Android) val += 64;
    return setUNIT_SEL(val);
}
 
/* ==================================================================
 * システムの動作モードを取得する
 * ------------------------------------------------------------------
 * returns:
 * 1            ACCONLY(NO FUSION)加速度のみ
 * 2            MAGONLY(NO FUSION)地磁気のみ
 * 3            GYROONLY(NO FUSION)角速度のみ
 * 4            ACCMAG(NO FUSION)加速度と地磁気
 * 5            ACCGYRO(NO FUSION)加速度と角速度
 * 6            MAGGYRO(NO FUSION)地磁気と角速度
 * 7            AMG(NO FUSION)加速度、地磁気、角速度
 * 8            IMU
 * 9            COMPASS
 * 10           M4G
 * 11           NDOF_FMC_OFF
 * 12           NDOF
 */
char BOARDC_BNO055::getOperationMode(){
    return ctrl->rr(0, BNO055P0_OPR_MODE);
}
 
/* ==================================================================
 * システムの動作モードを設定する
 * ------------------------------------------------------------------
 * 引数modeValue:
 * 1            ACCONLY(NO FUSION)加速度のみ
 * 2            MAGONLY(NO FUSION)地磁気のみ
 * 3            GYROONLY(NO FUSION)角速度のみ
 * 4            ACCMAG(NO FUSION)加速度と地磁気
 * 5            ACCGYRO(NO FUSION)加速度と角速度
 * 6            MAGGYRO(NO FUSION)地磁気と角速度
 * 7            AMG(NO FUSION)加速度、地磁気、角速度
 * 8            IMU
 * 9            COMPASS
 * 10           M4G
 * 11           NDOF_FMC_OFF
 * 12           NDOF
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setOperationMode(char modeValue){
    if(modeValue < 0 || modeValue > 12) modeValue = 7;
    return ctrl->wr(0, BNO055P0_OPR_MODE, modeValue);
}
 
/* ==================================================================
 * システムの動作モードを設定モードに設定する
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setOperation_CONFIG(){
    return ctrl->wr(0, BNO055P0_OPR_MODE, 0);
}
 
/* ==================================================================
 * システムの動作モードを加速度モードに設定する
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setOperation_ACCONRY(){
    return ctrl->wr(0, BNO055P0_OPR_MODE, 1);
}
 
/* ==================================================================
 * システムの動作モードを地磁気モードに設定する
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setOperation_MAGONRY(){
    return ctrl->wr(0, BNO055P0_OPR_MODE, 2);
}
 
/* ==================================================================
 * システムの動作モードを角速度モードに設定する
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setOperation_GYROONRY(){
    return ctrl->wr(0, BNO055P0_OPR_MODE, 3);
}
 
/* ==================================================================
 * システムの動作モードを加速度地磁気モードに設定する
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setOperation_ACCMAG(){
    return ctrl->wr(0, BNO055P0_OPR_MODE, 4);
}
 
/* ==================================================================
 * システムの動作モードを加速度角速度モードに設定する
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setOperation_ACCGYRO(){
    return ctrl->wr(0, BNO055P0_OPR_MODE, 5);
}
 
/* ==================================================================
 * システムの動作モードを地磁気角速度モードに設定する
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setOperation_MAGGYRO(){
    return ctrl->wr(0, BNO055P0_OPR_MODE, 6);
}
 
/* ==================================================================
 * システムの動作モードをFusionなし9軸モードに設定する
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setOperation_AMG(){
    return ctrl->wr(0, BNO055P0_OPR_MODE, 7);
}
 
/* ==================================================================
 * システムの動作モードを6軸(加速度、角速度)Fusionモードに設定する
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setOperation_Fusion_IMU(){
    return ctrl->wr(0, BNO055P0_OPR_MODE, 8);
}
 
/* ==================================================================
 * システムの動作モードを6軸(加速度、地磁気)Fusionモード(相対系)に設定する
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setOperation_Fusion_COMPASS(){
    return ctrl->wr(0, BNO055P0_OPR_MODE, 9);
}
 
/* ==================================================================
 * システムの動作モードを6軸(加速度、地磁気)Fusionモード(絶対系)に設定する
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setOperation_Fusion_M4G(){
    return ctrl->wr(0, BNO055P0_OPR_MODE, 10);
}
 
/* ==================================================================
 * システムの動作モードをNDOFモード(地磁気短時間補正OFF)に設定する
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setOperation_Fusion_NDOF_FMC_OFF(){
    return ctrl->wr(0, BNO055P0_OPR_MODE, 11);
}
 
/* ==================================================================
 * システムの動作モードをNDOFモードに設定する
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setOperation_Fusion_NDOF(){
    return ctrl->wr(0, BNO055P0_OPR_MODE, 12);
}
 
/* ==================================================================
 * システムの電源モードを取得する
 * ------------------------------------------------------------------
 * returns:
 * 0: Normal
 * 1: LowPower
 * 2: Suspend
 */
char BOARDC_BNO055::getPowerMode(){
    return ctrl->rr(0, BNO055P0_PWR_MODE);
}
 
/* ==================================================================
 * システムの電源モードを設定する
 * ------------------------------------------------------------------
 * modeValue:
 * 0: Normal
 * 1: LowPower
 * 2: Suspend
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setPowerMode(unsigned char modeValue){
    if(modeValue > 2) return -1;
    return ctrl->wr(0, BNO055P0_PWR_MODE, modeValue);
}
 
/* ==================================================================
 * システムの電源モードを通常モードに設定する
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setPowerMode_Normal(){
    return ctrl->wr(0, BNO055P0_PWR_MODE, 0);
}
 
/* ==================================================================
 * システムの電源モードを低消費電力モードに設定する
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setPowerMode_LowPower(){
    return ctrl->wr(0, BNO055P0_PWR_MODE, 1);
}
 
/* ==================================================================
 * システムの電源モードをスリープモードに設定する
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setPowerMode_Suspend(){
    return ctrl->wr(0, BNO055P0_PWR_MODE, 2);
}
 
/* ==================================================================
 * システムのトリガー設定を変更する
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 0            変更なし
 * 1            成功
 */
char BOARDC_BNO055::setSysTrigger(char regVal){
    clkExt = regVal >> 7;
    return ctrl->wr(0, BNO055P0_SYS_TRIGGER, regVal & 0xE1);
}
 
/* ==================================================================
 * システムのクロック発振元を設定する
 * 外部を指定する場合は、設定前にXIN,XOUTが発振子に結線されている必要がある
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 0            変更なし
 * 1            成功
 */
char BOARDC_BNO055::setSys_ExternalCrystal(bool isExternal){
    if(clkExt == isExternal) return 0;
 
    clkExt = isExternal;
    return ctrl->wr(0, BNO055P0_SYS_TRIGGER, (clkExt) ? 0x80 : 0x00);
}
 
/* ==================================================================
 * システムの割り込み発生をすべてリセットする
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::resetInterrupt(){
    return ctrl->wr(0, BNO055P0_SYS_TRIGGER, ((clkExt) ? 0x80 : 0x00) | 0x40);
}
 
/* ==================================================================
 * システムをリセットする
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::soft_reset(){
    return ctrl->wr(0, BNO055P0_SYS_TRIGGER, ((clkExt) ? 0x80 : 0x00) | 0x20);
}
 
/* ==================================================================
 * セルフテストを実行する
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::execSelfTest(){
    return ctrl->wr(0, BNO055P0_SYS_TRIGGER, ((clkExt) ? 0x80 : 0x00) | 0x01);
}
 
/* ==================================================================
 * システムの温度計測に使用するセンサーを取得する
 * ------------------------------------------------------------------
 * returns:
 * 0            温度計測に加速度センサーを使用している
 * 1            温度計測に角速度センサーを使用している
 */
char BOARDC_BNO055::getTempSource(){
    return ctrl->rr(0, BNO055P0_TEMP_SOURCE);
}
 
/* ==================================================================
 * システムの温度計測に使用するセンサーを選択する
 * ------------------------------------------------------------------
 * 引数Accelerometer:
 * true:加速度センサーを温度計測に使用する
 * false:角速度センサーを温度計測に使用する
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setTempSource(bool Accelerometer){
    return ctrl->wr(0, BNO055P0_TEMP_SOURCE, (Accelerometer) ? 0 : 1);
}
 
/* ==================================================================
 * センサー出力軸交換情報の取得
 * ------------------------------------------------------------------
 * returns:
 * AXIS_MAP_CONFIGの設定情報:BNO055データシート [3.4 Axis remap] を参照
 */
char BOARDC_BNO055::getAxisMapConfig(){
    char ret = ctrl->rr(0, BNO055P0_AXIS_MAP_CONFIG);
    if(axisRemap != ret) axisRemap = ret;
    return axisRemap;
}
 
/* ==================================================================
 * センサー出力軸交換の設定
 * ------------------------------------------------------------------
 * 引数val:
 * X,Y,Zの出力交換の情報
 * (BNO055データシート [3.4 Axis remap] を参照
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 0            変更なし
 * 1            成功
 */
char BOARDC_BNO055::setAxisMapConfig(char val){
    if(axisRemap == (val & 0x3F)) return 0;
    else axisRemap = (val & 0x3F);
    return ctrl->wr(0, BNO055P0_AXIS_MAP_CONFIG, axisRemap);
}
 
/* ==================================================================
 * センサー出力軸符号情報の取得
 * ------------------------------------------------------------------
 * returns:
 * AXIS_MAP_SIGNの設定情報:BNO055データシート [3.4 Axis remap] を参照
 */
char BOARDC_BNO055::getAxisMapSign(){
    char ret = ctrl->rr(0, BNO055P0_AXIS_MAP_SIGN);
    if(axisSign != ret) axisSign = ret;
    return axisSign;
}
 
/* ==================================================================
 * センサー出力軸符号の設定
 * ------------------------------------------------------------------
 * 引数val:
 * X,Y,Zの出力符号の情報
 * (BNO055データシート [3.4 Axis remap] を参照
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 0            変更なし
 * 1            成功
 */
char BOARDC_BNO055::setAxisMapSign(char val){
    if(axisSign == (val & 0x07)) return 0;
    else axisSign = (val & 0x07);
    return ctrl->wr(0, BNO055P0_AXIS_MAP_SIGN, axisSign);
}
 
/* ==================================================================
 * センサー出力軸の符号と交換を直感的に設定する[1pinが表側左上]
 * ------------------------------------------------------------------
 * (BNO055データシート [3.4 Axis remap] を参照
 * ------------------------------------------------------------------
 * returns:
 * -1           AXIS_MAP_CONFIG設定の失敗
 * -2           AXIS_MAP_SIGN設定の失敗
 * 1            成功
 */
char BOARDC_BNO055::setAxisRemap_topview_topleft(){
    axisRemap = 0x21;
    axisSign = 0x04;
    char ret = 0;
    ret = ctrl->wr(0, BNO055P0_AXIS_MAP_CONFIG, axisRemap);
    if(ret == -1) return -1;
    ret = ctrl->wr(0, BNO055P0_AXIS_MAP_SIGN, axisSign);
    if(ret == -1) return -2;
    return 1;
}
 
/* ==================================================================
 * センサー出力軸の符号と交換を直感的に設定する[1pinが表側右上]
 * ------------------------------------------------------------------
 * (BNO055データシート [3.4 Axis remap] を参照
 * ------------------------------------------------------------------
 * returns:
 * -1           AXIS_MAP_CONFIG設定の失敗
 * -2           AXIS_MAP_SIGN設定の失敗
 * 1            成功
 */
char BOARDC_BNO055::setAxisRemap_topview_topright(){
    axisRemap = 0x24;
    axisSign = 0x00;
    char ret = 0;
    ret = ctrl->wr(0, BNO055P0_AXIS_MAP_CONFIG, axisRemap);
    if(ret == -1) return -1;
    ret = ctrl->wr(0, BNO055P0_AXIS_MAP_SIGN, axisSign);
    if(ret == -1) return -2;
    return 1;
}
 
/* ==================================================================
 * センサー出力軸の符号と交換を直感的に設定する[1pinが表側左下]
 * ------------------------------------------------------------------
 * (BNO055データシート [3.4 Axis remap] を参照
 * ------------------------------------------------------------------
 * returns:
 * -1           AXIS_MAP_CONFIG設定の失敗
 * -2           AXIS_MAP_SIGN設定の失敗
 * 1            成功
 */
char BOARDC_BNO055::setAxisRemap_topview_bottomleft(){
    axisRemap = 0x24;
    axisSign = 0x06;
    char ret = 0;
    ret = ctrl->wr(0, BNO055P0_AXIS_MAP_CONFIG, axisRemap);
    if(ret == -1) return -1;
    ret = ctrl->wr(0, BNO055P0_AXIS_MAP_SIGN, axisSign);
    if(ret == -1) return -2;
    return 1;
}
 
/* ==================================================================
 * センサー出力軸の符号と交換を直感的に設定する[1pinが表側右下]
 * ------------------------------------------------------------------
 * (BNO055データシート [3.4 Axis remap] を参照
 * ------------------------------------------------------------------
 * returns:
 * -1           AXIS_MAP_CONFIG設定の失敗
 * -2           AXIS_MAP_SIGN設定の失敗
 * 1            成功
 */
char BOARDC_BNO055::setAxisRemap_topview_bottomright(){
    axisRemap = 0x21;
    axisSign = 0x02;
    char ret = 0;
    ret = ctrl->wr(0, BNO055P0_AXIS_MAP_CONFIG, axisRemap);
    if(ret == -1) return -1;
    ret = ctrl->wr(0, BNO055P0_AXIS_MAP_SIGN, axisSign);
    if(ret == -1) return -2;
    return 1;
}
 
/* ==================================================================
 * センサー出力軸の符号と交換を直感的に設定する[1pinが表側から見たとき裏側左上]
 * ------------------------------------------------------------------
 * (BNO055データシート [3.4 Axis remap] を参照
 * ------------------------------------------------------------------
 * returns:
 * -1           AXIS_MAP_CONFIG設定の失敗
 * -2           AXIS_MAP_SIGN設定の失敗
 * 1            成功
 */
char BOARDC_BNO055::setAxisRemap_bottomview_topleft(){
    axisRemap = 0x24;
    axisSign = 0x03;
    char ret = 0;
    ret = ctrl->wr(0, BNO055P0_AXIS_MAP_CONFIG, axisRemap);
    if(ret == -1) return -1;
    ret = ctrl->wr(0, BNO055P0_AXIS_MAP_SIGN, axisSign);
    if(ret == -1) return -2;
    return 1;
}
 
/* ==================================================================
 * センサー出力軸の符号と交換を直感的に設定する[1pinが表側から見たとき裏側右上]
 * ------------------------------------------------------------------
 * (BNO055データシート [3.4 Axis remap] を参照
 * ------------------------------------------------------------------
 * returns:
 * -1           AXIS_MAP_CONFIG設定の失敗
 * -2           AXIS_MAP_SIGN設定の失敗
 * 1            成功
 */
char BOARDC_BNO055::setAxisRemap_bottomview_topright(){
    axisRemap = 0x21;
    axisSign = 0x01;
    char ret = 0;
    ret = ctrl->wr(0, BNO055P0_AXIS_MAP_CONFIG, axisRemap);
    if(ret == -1) return -1;
    ret = ctrl->wr(0, BNO055P0_AXIS_MAP_SIGN, axisSign);
    if(ret == -1) return -2;
    return 1;
}
 
/* ==================================================================
 * センサー出力軸の符号と交換を直感的に設定する[1pinが表側から見たとき裏側左下]
 * ------------------------------------------------------------------
 * (BNO055データシート [3.4 Axis remap] を参照
 * ------------------------------------------------------------------
 * returns:
 * -1           AXIS_MAP_CONFIG設定の失敗
 * -2           AXIS_MAP_SIGN設定の失敗
 * 1            成功
 */
char BOARDC_BNO055::setAxisRemap_bottomview_bottomleft(){
    axisRemap = 0x21;
    axisSign = 0x07;
    char ret = 0;
    ret = ctrl->wr(0, BNO055P0_AXIS_MAP_CONFIG, axisRemap);
    if(ret == -1) return -1;
    ret = ctrl->wr(0, BNO055P0_AXIS_MAP_SIGN, axisSign);
    if(ret == -1) return -2;
    return 1;
}
 
/* ==================================================================
 * センサー出力軸の符号と交換を直感的に設定する[1pinが表側から見たとき裏側右下]
 * ------------------------------------------------------------------
 * (BNO055データシート [3.4 Axis remap] を参照
 * ------------------------------------------------------------------
 * returns:
 * -1           AXIS_MAP_CONFIG設定の失敗
 * -2           AXIS_MAP_SIGN設定の失敗
 * 1            成功
 */
char BOARDC_BNO055::setAxisRemap_bottomview_bottomright(){
    axisRemap = 0x24;
    axisSign = 0x05;
    char ret = 0;
    ret = ctrl->wr(0, BNO055P0_AXIS_MAP_CONFIG, axisRemap);
    if(ret == -1) return -1;
    ret = ctrl->wr(0, BNO055P0_AXIS_MAP_SIGN, axisSign);
    if(ret == -1) return -2;
    return 1;
}
 
/* ==================================================================
 * センサー出力軸の符号と交換の情報を直感的な形で取得する
 * ------------------------------------------------------------------
 * (BNO055データシート [3.4 Axis remap] を参照
 * ------------------------------------------------------------------
 * returns:
 * 0            [P0]表側から見たとき、BNO055の1pinが表側左上にある状態
 * 1            [P1]表側から見たとき、BNO055の1pinが表側右上にある状態
 * 2            [P2]表側から見たとき、BNO055の1pinが表側左下にある状態
 * 3            [P3]表側から見たとき、BNO055の1pinが表側右下にある状態
 * 4            [P4]表側から見たとき、BNO055の1pinが裏側左上にある状態
 * 5            [P5]表側から見たとき、BNO055の1pinが裏側右上にある状態
 * 6            [P6]表側から見たとき、BNO055の1pinが裏側左下にある状態
 * 7            [P7]表側から見たとき、BNO055の1pinが裏側右下にある状態
 * -1           それ以外の設定
 */
char BOARDC_BNO055::getAxisRemap_type(){
    getAxisMapConfig(); //axisRemapに最新の値を格納
    getAxisMapSign(); //axisSignに最新の値を格納
 
    if(axisRemap == 0x21){
        switch(axisSign){
            case 0x04:
                return 0;
            case 0x02:
                return 3;
            case 0x01:
                return 5;
            case 0x07:
                return 6;
        }
    }else if(axisRemap == 0x24){
        switch(axisSign){
            case 0x00:
                return 1;
            case 0x06:
                return 2;
            case 0x03:
                return 4;
            case 0x05:
                return 7;
        }
    }
 
    return -1;
}
 
/* ==================================================================
 * 加速度センサーの補正値を取得する
 * ------------------------------------------------------------------
 * &offsetX: アドレス参照引数:関数実行後、この変数にX軸補正値が格納される
 * &offsetY: アドレス参照引数:関数実行後、この変数にY軸補正値が格納される
 * &offsetZ: アドレス参照引数:関数実行後、この変数にZ軸補正値が格納される
 */
void BOARDC_BNO055::getAccOffsetAll(float &offsetX, float &offsetY, float &offsetZ){
    //連続6byte読み取り
    unsigned char rsv[6];
    ctrl->rrc(0, BNO055P0_ACC_OFFSET_X_LSB, rsv, 6);
 
    short offX = (rsv[1] << 8) | rsv[0];
    short offY = (rsv[3] << 8) | rsv[2];
    short offZ = (rsv[5] << 8) | rsv[4];
    offsetX = (1.0f * offX) * scaleACC;
    offsetY = (1.0f * offY) * scaleACC;
    offsetZ = (1.0f * offZ) * scaleACC;
}
 
/* ==================================================================
 * 加速度センサーの補正値(X軸のみ)を取得する
 * ------------------------------------------------------------------
 * returns:
 * X軸補正値(スケール乗算済み)
 */
float BOARDC_BNO055::getAccOffsetX(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_ACC_OFFSET_X_LSB, rsv, 2);
 
    short offX = (rsv[1] << 8) | rsv[0];
    return (1.0f * offX) * scaleACC;
}
 
/* ==================================================================
 * 加速度センサーの補正値(Y軸のみ)を取得する
 * ------------------------------------------------------------------
 * returns:
 * Y軸補正値(スケール乗算済み)
 */
float BOARDC_BNO055::getAccOffsetY(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_ACC_OFFSET_Y_LSB, rsv, 2);
 
    short offX = (rsv[1] << 8) | rsv[0];
    return (1.0f * offX) * scaleACC;
}
 
/* ==================================================================
 * 加速度センサーの補正値(Z軸のみ)を取得する
 * ------------------------------------------------------------------
 * returns:
 * Z軸補正値(スケール乗算済み)
 */
float BOARDC_BNO055::getAccOffsetZ(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_ACC_OFFSET_Z_LSB, rsv, 2);
 
    short offX = (rsv[1] << 8) | rsv[0];
    return (1.0f * offX) * scaleACC;
}
 
/* ==================================================================
 * 加速度センサーの補正値を設定する
 * ------------------------------------------------------------------
 * offsetX: スケール乗算済みのX軸の補正値
 * offsetY: スケール乗算済みのY軸の補正値
 * offsetZ: スケール乗算済みのZ軸の補正値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setAccOffsetAll(float offsetX, float offsetY, float offsetZ){
    short offX = (short)((offsetX / scaleACC) + 0.5f);
    short offY = (short)((offsetY / scaleACC) + 0.5f);
    short offZ = (short)((offsetZ / scaleACC) + 0.5f);
    char msg[6];
    msg[0] = offX & 0xFF;
    msg[1] = offX >> 8;
    msg[2] = offY & 0xFF;
    msg[3] = offY >> 8;
    msg[4] = offZ & 0xFF;
    msg[5] = offZ >> 8;
 
    //連続6byte書き込み
    return ctrl->wrc(0, BNO055P0_ACC_OFFSET_X_LSB, msg, 6);
}
 
char BOARDC_BNO055::setAccOffsetX(float offset){
    short offX = (short)((offset / scaleACC) + 0.5f);
    char msg[2];
    msg[0] = offX & 0xFF;
    msg[1] = offX >> 8;
 
    //連続2byte書き込み
    return ctrl->wrc(0, BNO055P0_ACC_OFFSET_X_LSB, msg, 2);
}
 
char BOARDC_BNO055::setAccOffsetY(float offset){
    short offY = (short)((offset / scaleACC) + 0.5f);
    char msg[2];
    msg[0] = offY & 0xFF;
    msg[1] = offY >> 8;
 
    //連続2byte書き込み
    return ctrl->wrc(0, BNO055P0_ACC_OFFSET_Y_LSB, msg, 2);
}
 
char BOARDC_BNO055::setAccOffsetZ(float offset){
    short offZ = (short)((offset / scaleACC) + 0.5f);
    char msg[2];
    msg[0] = offZ & 0xFF;
    msg[1] = offZ >> 8;
 
    //連続2byte書き込み
    return ctrl->wrc(0, BNO055P0_ACC_OFFSET_Z_LSB, msg, 2);
}
 
/* ==================================================================
 * 地磁気センサーの補正値を取得する
 * ------------------------------------------------------------------
 * &offsetX: アドレス参照引数:関数実行後、この変数にX軸補正値が格納される
 * &offsetY: アドレス参照引数:関数実行後、この変数にY軸補正値が格納される
 * &offsetZ: アドレス参照引数:関数実行後、この変数にZ軸補正値が格納される
 */
void BOARDC_BNO055::getMagOffsetAll(float &offsetX, float &offsetY, float &offsetZ){
    //連続6byte読み取り
    unsigned char rsv[6];
    ctrl->rrc(0, BNO055P0_MAG_OFFSET_X_LSB, rsv, 6);
 
    short offX = (rsv[1] << 8) | rsv[0];
    short offY = (rsv[3] << 8) | rsv[2];
    short offZ = (rsv[5] << 8) | rsv[4];
    offsetX = (1.0f * offX) * scaleMAG;
    offsetY = (1.0f * offY) * scaleMAG;
    offsetZ = (1.0f * offZ) * scaleMAG;
}
 
/* ==================================================================
 * 地磁気センサーの補正値(X軸のみ)を取得する
 * ------------------------------------------------------------------
 * returns:
 * X軸補正値(スケール乗算済み)
 */
float BOARDC_BNO055::getMagOffsetX(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_MAG_OFFSET_X_LSB, rsv, 2);
 
    short offX = (rsv[1] << 8) | rsv[0];
    return (1.0f * offX) * scaleMAG;
}
 
/* ==================================================================
 * 地磁気センサーの補正値(Y軸のみ)を取得する
 * ------------------------------------------------------------------
 * returns:
 * Y軸補正値(スケール乗算済み)
 */
float BOARDC_BNO055::getMagOffsetY(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_MAG_OFFSET_Y_LSB, rsv, 2);
 
    short offY = (rsv[1] << 8) | rsv[0];
    return (1.0f * offY) * scaleMAG;
}
 
/* ==================================================================
 * 地磁気センサーの補正値(Z軸のみ)を取得する
 * ------------------------------------------------------------------
 * returns:
 * Z軸補正値(スケール乗算済み)
 */
float BOARDC_BNO055::getMagOffsetZ(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_MAG_OFFSET_Z_LSB, rsv, 2);
 
    short offZ = (rsv[1] << 8) | rsv[0];
    return (1.0f * offZ) * scaleMAG;
}
 
/* ==================================================================
 * 地磁気センサーの補正値を設定する
 * ------------------------------------------------------------------
 * offsetX: スケール乗算済みのX軸の補正値
 * offsetY: スケール乗算済みのY軸の補正値
 * offsetZ: スケール乗算済みのZ軸の補正値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setMagOffsetAll(float offsetX, float offsetY, float offsetZ){
    short offX = (short)((offsetX / scaleMAG) + 0.5f);
    short offY = (short)((offsetY / scaleMAG) + 0.5f);
    short offZ = (short)((offsetZ / scaleMAG) + 0.5f);
    char msg[6];
    msg[0] = offX & 0xFF;
    msg[1] = offX >> 8;
    msg[2] = offY & 0xFF;
    msg[3] = offY >> 8;
    msg[4] = offZ & 0xFF;
    msg[5] = offZ >> 8;
 
    //連続6byte書き込み
    return ctrl->wrc(0, BNO055P0_MAG_OFFSET_X_LSB, msg, 6);
}
 
/* ==================================================================
 * 地磁気センサーの補正値(X軸のみ)を設定する
 * ------------------------------------------------------------------
 * offset: スケール乗算済みの補正値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setMagOffsetX(float offset){
    short offX = (short)((offset / scaleMAG) + 0.5f);
    char msg[2];
    msg[0] = offX & 0xFF;
    msg[1] = offX >> 8;
 
    //連続2byte書き込み
    return ctrl->wrc(0, BNO055P0_MAG_OFFSET_X_LSB, msg, 2);
}
 
/* ==================================================================
 * 地磁気センサーの補正値(Y軸のみ)を設定する
 * ------------------------------------------------------------------
 * offset: スケール乗算済みの補正値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setMagOffsetY(float offset){
    short offY = (short)((offset / scaleMAG) + 0.5f);
    char msg[2];
    msg[0] = offY & 0xFF;
    msg[1] = offY >> 8;
 
    //連続2byte書き込み
    return ctrl->wrc(0, BNO055P0_MAG_OFFSET_Y_LSB, msg, 2);
}
 
/* ==================================================================
 * 地磁気センサーの補正値(Z軸のみ)を設定する
 * ------------------------------------------------------------------
 * offset: スケール乗算済みの補正値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setMagOffsetZ(float offset){
    short offZ = (short)((offset / scaleMAG) + 0.5f);
    char msg[2];
    msg[0] = offZ & 0xFF;
    msg[1] = offZ >> 8;
 
    //連続2byte書き込み
    return ctrl->wrc(0, BNO055P0_MAG_OFFSET_Z_LSB, msg, 2);
}
 
/* ==================================================================
 * 角速度センサーの補正値を取得する
 * ------------------------------------------------------------------
 * &offsetX: アドレス参照引数:関数実行後、この変数にX軸補正値が格納される
 * &offsetY: アドレス参照引数:関数実行後、この変数にY軸補正値が格納される
 * &offsetZ: アドレス参照引数:関数実行後、この変数にZ軸補正値が格納される
 */
void BOARDC_BNO055::getGyroOffsetAll(float &offsetX, float &offsetY, float &offsetZ){
    //連続6byte読み取り
    unsigned char rsv[6];
    ctrl->rrc(0, BNO055P0_GYR_OFFSET_X_LSB, rsv, 6);
 
    short offX = (rsv[1] << 8) | rsv[0];
    short offY = (rsv[3] << 8) | rsv[2];
    short offZ = (rsv[5] << 8) | rsv[4];
    offsetX = (1.0f * offX) * scaleGYRO;
    offsetY = (1.0f * offY) * scaleGYRO;
    offsetZ = (1.0f * offZ) * scaleGYRO;
}
 
/* ==================================================================
 * 角速度センサーの補正値(X軸のみ)を取得する
 * ------------------------------------------------------------------
 * returns:
 * X軸補正値(スケール乗算済み)
 */
float BOARDC_BNO055::getGyroOffsetX(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_GYR_OFFSET_X_LSB, rsv, 2);
 
    short offX = (rsv[1] << 8) | rsv[0];
    return (1.0f * offX) * scaleGYRO;
}
 
/* ==================================================================
 * 角速度センサーの補正値(Y軸のみ)を取得する
 * ------------------------------------------------------------------
 * returns:
 * Y軸補正値(スケール乗算済み)
 */
float BOARDC_BNO055::getGyroOffsetY(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_GYR_OFFSET_Y_LSB, rsv, 2);
 
    short offY = (rsv[1] << 8) | rsv[0];
    return (1.0f * offY) * scaleGYRO;
}
 
/* ==================================================================
 * 角速度センサーの補正値(Z軸のみ)を取得する
 * ------------------------------------------------------------------
 * returns:
 * Z軸補正値(スケール乗算済み)
 */
float BOARDC_BNO055::getGyroOffsetZ(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_GYR_OFFSET_Z_LSB, rsv, 2);
 
    short offZ = (rsv[1] << 8) | rsv[0];
    return (1.0f * offZ) * scaleGYRO;
}
 
/* ==================================================================
 * 角速度センサーの補正値を設定する
 * ------------------------------------------------------------------
 * offsetX: スケール乗算済みのX軸の補正値
 * offsetY: スケール乗算済みのY軸の補正値
 * offsetZ: スケール乗算済みのZ軸の補正値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setGyroOffsetAll(float offsetX, float offsetY, float offsetZ){
    short offX = (short)((offsetX / scaleGYRO) + 0.5f);
    short offY = (short)((offsetY / scaleGYRO) + 0.5f);
    short offZ = (short)((offsetZ / scaleGYRO) + 0.5f);
    char msg[6];
    msg[0] = offX & 0xFF;
    msg[1] = offX >> 8;
    msg[2] = offY & 0xFF;
    msg[3] = offY >> 8;
    msg[4] = offZ & 0xFF;
    msg[5] = offZ >> 8;
 
    //連続6byte書き込み
    return ctrl->wrc(0, BNO055P0_GYR_OFFSET_X_LSB, msg, 6);
}
 
/* ==================================================================
 * 角速度センサーの補正値(X軸のみ)を設定する
 * ------------------------------------------------------------------
 * offset: スケール乗算済みの補正値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setGyroOffsetX(float offset){
    short offX = (short)((offset / scaleGYRO) + 0.5f);
    char msg[2];
    msg[0] = offX & 0xFF;
    msg[1] = offX >> 8;
 
    //連続2byte書き込み
    return ctrl->wrc(0, BNO055P0_GYR_OFFSET_X_LSB, msg, 2);
}
 
/* ==================================================================
 * 角速度センサーの補正値(Y軸のみ)を設定する
 * ------------------------------------------------------------------
 * offset: スケール乗算済みの補正値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setGyroOffsetY(float offset){
    short offY = (short)((offset / scaleGYRO) + 0.5f);
    char msg[2];
    msg[0] = offY & 0xFF;
    msg[1] = offY >> 8;
 
    //連続2byte書き込み
    return ctrl->wrc(0, BNO055P0_GYR_OFFSET_Y_LSB, msg, 2);
}
 
/* ==================================================================
 * 角速度センサーの補正値(Z軸のみ)を設定する
 * ------------------------------------------------------------------
 * offset: スケール乗算済みの補正値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setGyroOffsetZ(float offset){
    short offZ = (short)((offset / scaleGYRO) + 0.5f);
    char msg[2];
    msg[0] = offZ & 0xFF;
    msg[1] = offZ >> 8;
 
    //連続2byte書き込み
    return ctrl->wrc(0, BNO055P0_GYR_OFFSET_Z_LSB, msg, 2);
}
 
/* ==================================================================
 * 加速度センサーのデータ出力範囲[単位:LSB]を取得する
 * ------------------------------------------------------------------
 * returns:
 * +-データ計測範囲[単位:LSB]
 */
short BOARDC_BNO055::getAccRadius(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_ACC_RADIUS_LSB, rsv, 2);
 
    return (rsv[1] << 8) | rsv[0];
}
 
/* ==================================================================
 * 加速度センサーのデータ出力範囲[単位:LSB]を設定する
 * ------------------------------------------------------------------
 * 引数r: データ計測範囲の最大値および最低値(-r から +r　の範囲が計測範囲)
 * 最大値は1000[LSB]
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setAccRadius(short LSB){
    if(LSB > 1000) LSB = 1000;
 
    char msg[2];
    msg[0] = LSB & 0xFF;
    msg[1] = LSB >> 8;
 
    //連続2byte書き込み
    return ctrl->wrc(0, BNO055P0_ACC_RADIUS_LSB, msg, 2);
}
 
/* ==================================================================
 * 地磁気センサーのデータ出力範囲[単位:LSB]を取得する
 * ------------------------------------------------------------------
 * returns:
 * +-データ計測範囲
 */
short BOARDC_BNO055::getMagRadius(){
    //連続2byte読み取り
    unsigned char rsv[2];
    ctrl->rrc(0, BNO055P0_MAG_RADIUS_LSB, rsv, 2);
 
    return (rsv[1] << 8) | rsv[0];
}
 
/* ==================================================================
 * 加速度センサーのデータ出力範囲[単位:LSB]を設定する
 * ------------------------------------------------------------------
 * 引数r: データ計測範囲の最大値および最低値(-r から +r　の範囲が計測範囲)
 * 最大値は960[LSB]
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setMagRadius(short LSB){
    if(LSB > 960) LSB = 960;
 
    char msg[2];
    msg[0] = LSB & 0xFF;
    msg[1] = LSB >> 8;
 
    //連続2byte書き込み
    return ctrl->wrc(0, BNO055P0_MAG_RADIUS_LSB, msg, 2);
}
 
/* ==================================================================
 * 加速度センサー設定のレジスタ値を取得する
 * ------------------------------------------------------------------
 * returns:
 * 加速度センサー設定のレジスタ値
 */
char BOARDC_BNO055::getAccConfig(){
    return ctrl->rr(1, BNO055P1_ACC_CONFIG);
}
 
/* ==================================================================
 * 加速度センサー設定のレジスタ値を設定する
 * ------------------------------------------------------------------
 * 引数regVal: 加速度センサー設定のレジスタ値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setAccConfig(char regVal){
    return ctrl->wr(1, BNO055P1_ACC_CONFIG, regVal);
}
 
/* ==================================================================
 * 加速度センサー設定のレジスタ値を設定する
 * ------------------------------------------------------------------
 * 引数gRange: 加速度センサーの計測範囲
 * 引数bandWidth: 加速度センサー出力レート(Fusion時は自動設定)
 * 引数powMode: 加速度センサー電源設定(Fusion時は自動設定)
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setAccConfig(char gRange, char bandWidth, char powMode){
    char regVal = (powMode << 5) | (bandWidth << 2) | gRange;
    return ctrl->wr(1, BNO055P1_ACC_CONFIG, regVal);
}
 
/* ==================================================================
 * 加速度センサーの計測範囲を設定する
 * ------------------------------------------------------------------
 * 引数G: 加速度センサーの計測範囲(+-2, 4, 8, 16Gのいずれか)
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setAccRange(unsigned char G){
    char val = 0;
    switch(G){
        case 2:
            val = 0x00;
            break;
        case 4:
            val = 0x01;
            break;
        case 8:
            val = 0x02;
            break;
        case 16:
            val = 0x03;
            break;
        default:
            val = 0x02;
    }
 
    char regVal = ctrl->rr(1, BNO055P1_ACC_CONFIG);
    regVal = (regVal & 0xFC) | val;
 
    return ctrl->wr(1, BNO055P1_ACC_CONFIG, regVal);
}
 
/* ==================================================================
 * 地磁気センサー設定のレジスタ値を取得する
 * ------------------------------------------------------------------
 * returns:
 * 地磁気センサー設定のレジスタ値
 */
char BOARDC_BNO055::getMagConfig(){
    return ctrl->rr(1, BNO055P1_MAG_CONFIG);
}
 
/* ==================================================================
 * 地磁気センサー設定のレジスタ値を設定する
 * ------------------------------------------------------------------
 * 引数regVal: 地磁気センサー設定のレジスタ値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setMagConfig(char regVal){
    return ctrl->wr(1, BNO055P1_MAG_CONFIG, regVal);
}
 
/* ==================================================================
 * 地磁気センサー設定のレジスタ値を設定する
 * ------------------------------------------------------------------
 * 引数rate: 地磁気センサー出力レート(Fusion時は自動設定)
 * 引数oprMode: 地磁気センサー出力モード(Fusion時は自動設定)
 * 引数powMode: 地磁気センサー電源設定(Fusion時は自動設定)
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setMagConfig(char rate, char oprMode, char powMode){
    char regVal = powMode << 5 | oprMode << 3 | rate;
    return ctrl->wr(1, BNO055P1_MAG_CONFIG, regVal);
}
 
/* ==================================================================
 * 角速度センサー設定(0)のレジスタ値を取得する
 * ------------------------------------------------------------------
 * returns:
 * 角速度センサー設定(0)のレジスタ値
 */
char BOARDC_BNO055::getGyroConfig_0(){
    return ctrl->rr(1, BNO055P1_GYR_CONFIG_0);
}
 
/* ==================================================================
 * 角速度センサー設定(0)のレジスタ値を設定する
 * ------------------------------------------------------------------
 * 引数regVal: 角速度センサー設定(0)のレジスタ値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setGyroConfig_0(char regVal){
    return ctrl->wr(1, BNO055P1_GYR_CONFIG_0, regVal);
}
 
/* ==================================================================
 * 角速度センサー設定(0)のレジスタ値を設定する
 * ------------------------------------------------------------------
 * 引数range: 角速度センサーの計測範囲(Fusion時は自動設定)
 * 引数bandWidth: 角速度センサー出力レート(Fusion時は自動設定)
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setGyroConfig_0(char range, char bandWidth){
    char regVal = bandWidth << 3 | range;
    return ctrl->wr(1, BNO055P1_GYR_CONFIG_0, regVal);
}
 
/* ==================================================================
 * 角速度センサー設定(1)のレジスタ値を取得する
 * ------------------------------------------------------------------
 * returns:
 * 角速度センサー設定(1)のレジスタ値
 */
char BOARDC_BNO055::getGyroConfig_1(){
    return ctrl->rr(1, BNO055P1_GYR_CONFIG_0);
}
 
/* ==================================================================
 * 角速度センサー設定(1)のレジスタ値を設定する
 * ------------------------------------------------------------------
 * 引数powMode: 角速度センサーの電源設定(Fusion時は自動設定)
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setGyroConfig_1(char powMode){
    return ctrl->wr(1, BNO055P1_GYR_CONFIG_0, powMode & 0x07);
}
 
/* ==================================================================
 * 角速度センサーの計測範囲を設定する
 * ------------------------------------------------------------------
 * 引数G: 角速度センサーの計測範囲(2000, 1000, 500, 250, 125dpsのいずれか)
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setGyroRange(unsigned short dps){
    char val = 0;
    switch(dps){
        case 2000:
            val = 0;
            break;
        case 1000:
            val = 1;
            break;
        case 500:
            val = 2;
            break;
        case 250:
            val = 3;
            break;
        case 125:
            val = 4;
            break;
        default:
            val = 1;
    }
 
    char regVal = ctrl->rr(1, BNO055P1_GYR_CONFIG_0);
    regVal = (regVal & 0xF8) + val;
 
    return ctrl->wr(1, BNO055P1_GYR_CONFIG_0, regVal);
}
 
/* ==================================================================
 * 加速度センサーのスリープモードのレジスタ値を取得する
 * ------------------------------------------------------------------
 * returns:
 * 加速度センサーのスリープモードのレジスタ値
 */
char BOARDC_BNO055::getAccSleepConfig(){
    return ctrl->rr(1, BNO055P1_ACC_SLEEP_CONFIG);
}
 
/* ==================================================================
 * 加速度センサーのスリープモードのレジスタ値を設定する
 * ------------------------------------------------------------------
 * 引数regVal: 加速度センサーのスリープモードのレジスタ値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setAccSleepConfig(char regVal){
    return ctrl->wr(1, BNO055P1_ACC_SLEEP_CONFIG, regVal);
}
 
/* ==================================================================
 * 加速度センサーのスリープモードのレジスタ値を設定する
 * ------------------------------------------------------------------
 * 引数duration: 加速度センサーのスリープモードの持続閾値
 * 引数mode: 加速度センサーのスリープモード選択
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setAccSleepConfig(char duration, char mode){
    char regVal = duration << 1 | mode;
    return ctrl->wr(1, BNO055P1_ACC_SLEEP_CONFIG, regVal);
}
 
/* ==================================================================
 * 角速度センサーのスリープモードのレジスタ値を取得する
 * ------------------------------------------------------------------
 * returns:
 * 角速度センサーのスリープモードのレジスタ値
 */
char BOARDC_BNO055::getGyroSleepConfig(){
    return ctrl->rr(1, BNO055P1_GYR_SLEEP_CONFIG);
}
 
/* ==================================================================
 * 角速度センサーのスリープモードのレジスタ値を設定する
 * ------------------------------------------------------------------
 * 引数regVal: 角速度センサーのスリープモードのレジスタ値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setGyroSleepConfig(char regVal){
    return ctrl->wr(1, BNO055P1_GYR_SLEEP_CONFIG, regVal);
}
 
/* ==================================================================
 * 角速度センサーのスリープモードのレジスタ値を設定する
 * ------------------------------------------------------------------
 * 引数duration: 角速度センサーの自動スリープモードの持続閾値
 * 引数mode: 角速度センサーのスリープモードの持続閾値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setGyroSleepConfig(char autoSleepDuration, char duration){
    char regVal = autoSleepDuration << 3 | duration;
    return ctrl->wr(1, BNO055P1_GYR_SLEEP_CONFIG, regVal);
}
 
/* ==================================================================
 * 各センサーの割り込みフラグ発生のINTピン出力許可設定を取得する
 * ------------------------------------------------------------------
 * returns:
 * 各センサーの割り込みフラグ有効無効設定のレジスタ値
 * 2bit目        角速度センサーAnyMotion割り込み(0:無効, 1:許可)
 * 3bit目        角速度センサーHighRate割り込み(0:無効, 1:許可)
 * 5bit目        加速度センサーHIGH_G割り込み(0:無効, 1:許可)
 * 6bit目        加速度センサーAnyMotion割り込み(0:無効, 1:許可)
 * 7bit目        加速度センサーNoMotion(SloMo)割り込み(0:無効, 1:許可)
 */
char BOARDC_BNO055::getInterruptMask(){
    return ctrl->rr(1, BNO055P1_INT_MSK);
}
 
/* ==================================================================
 * 各センサーの割り込みフラグ発生のINTピン出力許可設定を設定する
 * ------------------------------------------------------------------
 * 引数mask: 各センサーの割り込みフラグ有効無効設定のレジスタ値
 * 2bit目        角速度センサーAnyMotion割り込み(0:無効, 1:許可)
 * 3bit目        角速度センサーHighRate割り込み(0:無効, 1:許可)
 * 5bit目        加速度センサーHIGH_G割り込み(0:無効, 1:許可)
 * 6bit目        加速度センサーAnyMotion割り込み(0:無効, 1:許可)
 * 7bit目        加速度センサーNoMotion(SloMo)割り込み(0:無効, 1:許可)
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setInterruptMask(char mask){
    return ctrl->wr(1, BNO055P1_INT_MSK, mask);
}
 
/* ==================================================================
 * 各センサーの割り込みフラグ有効無効設定を取得する
 * ------------------------------------------------------------------
 * returns:
 * 各センサーの割り込みフラグ有効無効設定のレジスタ値
 * 2bit目        角速度センサーAnyMotion割り込み(0:無効, 1:有効)
 * 3bit目        角速度センサーHighRate割り込み(0:無効, 1:有効)
 * 5bit目        加速度センサーHIGH_G割り込み(0:無効, 1:有効)
 * 6bit目        加速度センサーAnyMotion割り込み(0:無効, 1:有効)
 * 7bit目        加速度センサーNoMotion(SloMo)割り込み(0:無効, 1:有効)
 */
char BOARDC_BNO055::getInterruptEnable(){
    return ctrl->rr(1, BNO055P1_INT_EN);
}
 
/* ==================================================================
 * 各センサーの割り込みフラグ有効無効設定を設定する
 * ------------------------------------------------------------------
 * 引数mask: 各センサーの割り込みフラグ有効無効設定のレジスタ値
 * 2bit目        角速度センサーAnyMotion割り込み(0:無効, 1:有効)
 * 3bit目        角速度センサーHighRate割り込み(0:無効, 1:有効)
 * 5bit目        加速度センサーHIGH_G割り込み(0:無効, 1:有効)
 * 6bit目        加速度センサーAnyMotion割り込み(0:無効, 1:有効)
 * 7bit目        加速度センサーNoMotion(SloMo)割り込み(0:無効, 1:有効)
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setInterruptEnable(char mask){
    return ctrl->wr(1, BNO055P1_INT_EN, mask);
}
 
/* ==================================================================
 * 加速度センサーのAnyMotion割り込み発生閾値を取得する
 * ------------------------------------------------------------------
 * 引数ismg: 出力する値の単位の選択(true: mG, false: mm/s^2)
 * ------------------------------------------------------------------
 * returns:
 * 加速度センサーのAnyMotion割り込み発生閾値
 */
float BOARDC_BNO055::getAccAnyMotionThreashold(bool ismg){
    char rc = getAccConfig();
    float scale = 0.0f;
 
    switch(rc & 0x03){
        case 0:
            scale = 3.91;
            break;
        case 1:
            scale = 7.81;
            break;
        case 2:
            scale = 15.63;
            break;
        case 3:
            scale = 31.25;
            break;
    }
 
    scale *= (ismg) ? 1.0 : 9.80665;
 
    return (1.0 * ctrl->rr(1, BNO055P1_ACC_AM_THRES)) * scale;
}
 
/* ==================================================================
 * 加速度センサーのAnyMotion割り込み発生閾値を設定する
 * ------------------------------------------------------------------
 * 引数ismg: 設定する値の単位の選択(true: mG, false: mm/s^2)
 * 引数threashold: 単位変換済みの閾値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setAccAnyMotionThreashold(bool ismg, float threashold){
    char rc = getAccConfig();
    float scale = 0.0f;
 
    switch(rc & 0x03){
        case 0:
            scale = 3.91;
            break;
        case 1:
            scale = 7.81;
            break;
        case 2:
            scale = 15.63;
            break;
        case 3:
            scale = 31.25;
            break;
    }
 
    scale *= (ismg) ? 1.0 : 9.8;
 
    char cTh = (char)((threashold / scale) + 0.5);
 
    return ctrl->wr(1, BNO055P1_ACC_AM_THRES, cTh);
}
 
/* ==================================================================
 * 加速度センサーの割り込み使用設定を取得する
 * ------------------------------------------------------------------
 * returns:
 * 0bit目,1bit目  連続発生閾値([1bit目+0bit目+1]回の発生でフラグON)
 * 2bit目        AM/NM_X_AXISトリガー使用(0:使用しない, 1:使用する)
 * 3bit目        AM/NM_Y_AXISトリガー使用(0:使用しない, 1:使用する)
 * 4bit目        AM/NM_Z_AXISトリガー使用(0:使用しない, 1:使用する)
 * 5bit目        HG_X_AXISトリガー使用(0:使用しない, 1:使用する)
 * 6bit目        HG_Y_AXISトリガー使用(0:使用しない, 1:使用する)
 * 7bit目        HG_Z_AXISトリガー使用(0:使用しない, 1:使用する)
 */
char BOARDC_BNO055::getAccInterruptSettings(){
    return ctrl->rr(1, BNO055P1_ACC_INT_SETTINGS);
}
 
/* ==================================================================
 * 加速度センサーの割り込み使用設定を設定する
 * ------------------------------------------------------------------
 * 引数settings:
 * 0bit目,1bit目  連続発生閾値([1bit目+0bit目+1]回の発生でフラグON)
 * 2bit目        AM/NM_X_AXISトリガー使用(0:使用しない, 1:使用する)
 * 3bit目        AM/NM_Y_AXISトリガー使用(0:使用しない, 1:使用する)
 * 4bit目        AM/NM_Z_AXISトリガー使用(0:使用しない, 1:使用する)
 * 5bit目        HG_X_AXISトリガー使用(0:使用しない, 1:使用する)
 * 6bit目        HG_Y_AXISトリガー使用(0:使用しない, 1:使用する)
 * 7bit目        HG_Z_AXISトリガー使用(0:使用しない, 1:使用する)
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setAccInterruptSettings(char settings){
    return ctrl->wr(1, BNO055P1_ACC_INT_SETTINGS, settings);
}
 
/* ==================================================================
 * 加速度センサーのHighG割り込み閾値を取得する
 * ------------------------------------------------------------------
 * returns:
 * HighG割り込み発生閾値[単位:ミリ秒](2ms - 512ms)
 */
unsigned short BOARDC_BNO055::getAccHighGduration(){
    return (ctrl->rr(1, BNO055P1_ACC_HG_DURATION) + 1) << 1;
}
 
/* ==================================================================
 * 加速度センサーのHighG割り込み継続発生閾値を設定する
 * ------------------------------------------------------------------
 * 引数ms: HighG割り込み継続発生閾値[単位:ミリ秒](2ms - 512ms)
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setAccHighGduration(short ms){
    if(ms > 512 || ms < 2) return -1;
 
    ms = (ms >> 1) - 1;
 
    return ctrl->wr(1, BNO055P1_ACC_HG_DURATION, ms & 0xFF);
}
 
/* ==================================================================
 * 加速度センサーのHighG割り込み発生閾値を取得する
 * ------------------------------------------------------------------
 * 引数ismg: 出力する値の単位の選択(true: mG, false: mm/s^2)
 * ------------------------------------------------------------------
 * returns:
 * HighG割り込み発生閾値
 */
float BOARDC_BNO055::getAccHighGThreashold(bool ismg){
    char rc = getAccConfig();
    float scale = 0.0f;
 
    switch(rc & 0x03){
        case 0:
            scale = 7.81;
            break;
        case 1:
            scale = 15.63;
            break;
        case 2:
            scale = 31.25;
            break;
        case 3:
            scale = 62.5;
            break;
    }
 
    scale *= (ismg) ? 1.0 : 9.8;
 
    return (1.0 * ctrl->rr(1, BNO055P1_ACC_HG_THRES)) * scale;
}
 
/* ==================================================================
 * 加速度センサーのHighG割り込み発生閾値を設定する
 * ------------------------------------------------------------------
 * 引数ismg: 設定する値の単位の選択(true: mG, false: mm/s^2)
 * 引数threashold: 単位変換済みの閾値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setAccHighGThreashold(bool ismg, float threashold){
    char rc = getAccConfig();
    float scale = 0.0f;
 
    switch(rc & 0x03){
        case 0:
            scale = 7.81;
            break;
        case 1:
            scale = 15.63;
            break;
        case 2:
            scale = 31.25;
            break;
        case 3:
            scale = 62.5;
            break;
    }
 
    scale *= (ismg) ? 1.0 : 9.8;
 
    char cTh = (char)((threashold / scale) + 0.5);
 
    return ctrl->wr(1, BNO055P1_ACC_HG_THRES, cTh);
}
 
/* ==================================================================
 * 加速度センサーのNoMotion割り込み発生閾値を取得する
 * ------------------------------------------------------------------
 * 引数ismg: 設定する値の単位の選択(true: mG, false: mm/s^2)
 * ------------------------------------------------------------------
 * returns:
 * NoMotion割り込み発生閾値
 */
float BOARDC_BNO055::getAccNMThreashold(bool ismg){
    char rc = getAccConfig();
    float scale = 0.0f;
 
    switch(rc & 0x03){
        case 0:
            scale = 3.91;
            break;
        case 1:
            scale = 7.81;
            break;
        case 2:
            scale = 15.63;
            break;
        case 3:
            scale = 31.25;
            break;
    }
 
    scale *= (ismg) ? 1.0 : 9.8;
 
    return (1.0 * ctrl->rr(1, BNO055P1_ACC_NM_THRES)) * scale;
}
 
/* ==================================================================
 * 加速度センサーのNoMotion割り込み発生閾値を設定する
 * ------------------------------------------------------------------
 * 引数ismg: 設定する値の単位の選択(true: mG, false: mm/s^2)
 * 引数threashold: 単位変換済みの閾値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setAccNMThreashold(bool ismg, float threashold){
    char rc = getAccConfig();
    float scale = 0.0f;
 
    switch(rc & 0x03){
        case 0:
            scale = 3.91;
            break;
        case 1:
            scale = 7.81;
            break;
        case 2:
            scale = 15.63;
            break;
        case 3:
            scale = 31.25;
            break;
    }
 
    scale *= (ismg) ? 1.0 : 9.8;
 
    char cTh = (char)((threashold / scale) + 0.5);
 
    return ctrl->wr(1, BNO055P1_ACC_NM_THRES, cTh);
}
 
/* ==================================================================
 * 加速度センサーのNoMotion割り込み設定を取得する
 * ------------------------------------------------------------------
 * returns:
 * NoMotion割り込み設定値
 * 1bit目: スローモーション、ノーモーション選択(0:NoMotion, 1:SlowMotion)
 * 2 - 7bit目: slo_no_mot _dur設定値(データシート参照)
 */
char BOARDC_BNO055::getAccNMsetting(){
    return ctrl->rr(1, BNO055P1_ACC_NM_SET);
}
 
/* ==================================================================
 * 加速度センサーのNoMotion割り込み設定を設定する
 * ------------------------------------------------------------------
 * 引数setting: NoMotion割り込み設定値
 * 1bit目: スローモーション、ノーモーション選択(0:NoMotion, 1:SlowMotion)
 * 2 - 7bit目: slo_no_mot _dur設定値(データシート参照)
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setAccNMsetting(char setting){
    return ctrl->wr(1, BNO055P1_ACC_NM_SET, setting);
}
 
/* ==================================================================
 * 角速度センサーの割り込み設定を取得する
 * ------------------------------------------------------------------
 * returns:
 * 角速度センサーの割り込み設定レジスタ値
 */
char BOARDC_BNO055::getGyroInterruptSettings(){
    return ctrl->rr(1, BNO055P1_GYR_INT_SETING);
}
 
/* ==================================================================
 * 角速度センサーの割り込み設定を設定する
 * ------------------------------------------------------------------
 * 引数setting: 角速度センサーの割り込み設定レジスタ値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setGyroInterruptSettings(char settings){
    return ctrl->wr(1, BNO055P1_GYR_INT_SETING, settings);
}
 
/* ==================================================================
 * 角速度センサーのHighRate割り込み設定(X軸のみ)を取得する
 * ------------------------------------------------------------------
 * 引数setting: 角速度センサーの割り込み設定レジスタ値
 */
char BOARDC_BNO055::getGyroHighRateXsetting(){
    return ctrl->rr(1, BNO055P1_GYR_HR_X_SET);
}
 
/* ==================================================================
 * 角速度センサーのHighRate割り込み設定(X軸のみ、実際の値)を取得する
 * ------------------------------------------------------------------
 * 引数&hyst: アドレス参照引数:関数実行後、この変数にヒステリシスの値が格納される
 * 引数&thres: アドレス参照引数:関数実行後、この変数にスレッショルドの値が格納される
 */
void BOARDC_BNO055::getGyroHighRateXsetting_dps(float &hyst, float &thres){
    char r = getGyroConfig_0();
    hyst = 0.0f;
    thres = 0.0f;
 
    switch(r & 0x07){
        case 0:
            hyst = 62.26;
            thres = 62.5;
            break;
        case 1:
            hyst = 31.13;
            thres = 31.25;
            break;
        case 2:
            hyst = 15.56;
            thres = 15.62;
            break;
        case 3:
            hyst = 7.78;
            thres = 7.81;
            break;
        case 4:
            hyst = 3.89;
            thres = 3.90;
            break;
    }
 
    char val = ctrl->rr(1, BNO055P1_GYR_HR_X_SET);
 
    hyst *= (float)((val & 0x60) * 1.0);
    thres *= (float)((val & 0x1F) * 1.0);
}
 
/* ==================================================================
 * 角速度センサーのHighRate割り込み設定(X軸のみ)を設定する
 * ------------------------------------------------------------------
 * 引数setting: 角速度センサーのHighRate割り込み設定レジスタ値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setGyroHighRateXsetting(char setting){
    return ctrl->wr(1, BNO055P1_GYR_HR_X_SET, setting);
}
 
/* ==================================================================
 * 角速度センサーのHighRate割り込み設定(X軸のみ)を設定する
 * ------------------------------------------------------------------
 * 引数hystVal: ヒステリシス
 * 引数thresVal: スレッショルド
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setGyroHighRateXsetting_dps(float hystVal, float thresVal){
    char r = getGyroConfig_0();
    float hyst = 0.0f;
    float thres = 0.0f;
 
    switch(r & 0x07){
        case 0:
            hyst = 62.26;
            thres = 62.5;
            break;
        case 1:
            hyst = 31.13;
            thres = 31.25;
            break;
        case 2:
            hyst = 15.56;
            thres = 15.62;
            break;
        case 3:
            hyst = 7.78;
            thres = 7.81;
            break;
        case 4:
            hyst = 3.89;
            thres = 3.90;
            break;
    }
 
    char hystChar = (char)((hystVal / hyst) + 0.5);
    char thresChar = (char)((thresVal / thres) + 0.5);
 
    return ctrl->wr(1, BNO055P1_GYR_HR_X_SET, ((hystChar << 5) | thresChar));
}
 
/* ==================================================================
 * 角速度センサーのHighRate割り込み継続発生閾値(X軸のみ)を取得する
 * ------------------------------------------------------------------
 * returns:
 * HighRate割り込み継続発生閾値[単位:ミリ秒]
 */
float BOARDC_BNO055::getGyroHighRateXduration(){
    return (float)(1 + ctrl->rr(1, BNO055P1_GYR_DUR_X)) * 2.5;
}
 
/* ==================================================================
 * 角速度センサーのHighRate割り込み継続発生閾値(X軸のみ)を設定する
 * ------------------------------------------------------------------
 * 引数duration: HighRate割り込み継続発生閾値[単位:ミリ秒]
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setGyroHighRateXduration(float duration){
    return ctrl->wr(1, BNO055P1_GYR_HR_X_SET, (char)(((duration / 2.5) - 1.0) + 0.5));
}
 
/* ==================================================================
 * 角速度センサーのHighRate割り込み設定(Y軸のみ)を取得する
 * ------------------------------------------------------------------
 * returns:
 * 角速度センサーのHighRate割り込み設定
 */
char BOARDC_BNO055::getGyroHighRateYsetting(){
    return ctrl->rr(1, BNO055P1_GYR_HR_Y_SET);
}
 
/* ==================================================================
 * 角速度センサーのHighRate割り込み設定(Y軸のみ、実際の値)を取得する
 * ------------------------------------------------------------------
 * 引数&hyst: アドレス参照引数:関数実行後、この変数にヒステリシスの値が格納される
 * 引数&thres: アドレス参照引数:関数実行後、この変数にスレッショルドの値が格納される
 */
void BOARDC_BNO055::getGyroHighRateYsetting_dps(float &hyst, float &thres){
    char r = getGyroConfig_0();
    hyst = 0.0f;
    thres = 0.0f;
 
    switch(r & 0x07){
        case 0:
            hyst = 62.26;
            thres = 62.5;
            break;
        case 1:
            hyst = 31.13;
            thres = 31.25;
            break;
        case 2:
            hyst = 15.56;
            thres = 15.62;
            break;
        case 3:
            hyst = 7.78;
            thres = 7.81;
            break;
        case 4:
            hyst = 3.89;
            thres = 3.90;
            break;
    }
 
    char val = ctrl->rr(1, BNO055P1_GYR_HR_Y_SET);
 
    hyst *= (float)((val & 0x60) * 1.0);
    thres *= (float)((val & 0x1F) * 1.0);
}
 
/* ==================================================================
 * 角速度センサーのHighRate割り込み設定(Y軸のみ)を設定する
 * ------------------------------------------------------------------
 * 引数setting: 角速度センサーのHighRate割り込み設定レジスタ値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setGyroHighRateYsetting(char setting){
    return ctrl->wr(1, BNO055P1_GYR_HR_Y_SET, setting);
}
 
/* ==================================================================
 * 角速度センサーのHighRate割り込み設定(Y軸のみ)を設定する
 * ------------------------------------------------------------------
 * 引数hystVal: ヒステリシス
 * 引数thresVal: スレッショルド
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setGyroHighRateYsetting_dps(float hystVal, float thresVal){
    char r = getGyroConfig_0();
    float hyst = 0.0f;
    float thres = 0.0f;
 
    switch(r & 0x07){
        case 0:
            hyst = 62.26;
            thres = 62.5;
            break;
        case 1:
            hyst = 31.13;
            thres = 31.25;
            break;
        case 2:
            hyst = 15.56;
            thres = 15.62;
            break;
        case 3:
            hyst = 7.78;
            thres = 7.81;
            break;
        case 4:
            hyst = 3.89;
            thres = 3.90;
            break;
    }
 
    char hystChar = (char)((hystVal / hyst) + 0.5);
    char thresChar = (char)((thresVal / thres) + 0.5);
 
    return ctrl->wr(1, BNO055P1_GYR_HR_Y_SET, ((hystChar << 5) | thresChar));
}
 
/* ==================================================================
 * 角速度センサーのHighRate割り込み継続発生閾値(Y軸のみ)を取得する
 * ------------------------------------------------------------------
 * returns:
 * HighRate割り込み継続発生閾値[単位:ミリ秒]
 */
float BOARDC_BNO055::getGyroHighRateYduration(){
    return (float)(1 + ctrl->rr(1, BNO055P1_GYR_DUR_Y)) * 2.5;
}
 
/* ==================================================================
 * 角速度センサーのHighRate割り込み継続発生閾値(Y軸のみ)を設定する
 * ------------------------------------------------------------------
 * 引数duration: HighRate割り込み継続発生閾値[単位:ミリ秒]
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setGyroHighRateYduration(float duration){
    return ctrl->wr(1, BNO055P1_GYR_HR_Y_SET, (char)(((duration / 2.5) - 1.0) + 0.5));
}
 
/* ==================================================================
 * 角速度センサーのHighRate割り込み設定(Z軸のみ)を取得する
 * ------------------------------------------------------------------
 * returns:
 * 角速度センサーのHighRate割り込み設定レジスタ値
 */
char BOARDC_BNO055::getGyroHighRateZsetting(){
    return ctrl->rr(1, BNO055P1_GYR_HR_Z_SET);
}
 
/* ==================================================================
 * 角速度センサーのHighRate割り込み設定(Z軸のみ、実際の値)を取得する
 * ------------------------------------------------------------------
 * 引数&hyst: アドレス参照引数:関数実行後、この変数にヒステリシスの値が格納される
 * 引数&thres: アドレス参照引数:関数実行後、この変数にスレッショルドの値が格納される
 */
void BOARDC_BNO055::getGyroHighRateZsetting_dps(float &hyst, float &thres){
    char r = getGyroConfig_0();
    hyst = 0.0f;
    thres = 0.0f;
 
    switch(r & 0x07){
        case 0:
            hyst = 62.26;
            thres = 62.5;
            break;
        case 1:
            hyst = 31.13;
            thres = 31.25;
            break;
        case 2:
            hyst = 15.56;
            thres = 15.62;
            break;
        case 3:
            hyst = 7.78;
            thres = 7.81;
            break;
        case 4:
            hyst = 3.89;
            thres = 3.90;
            break;
    }
 
    char val = ctrl->rr(1, BNO055P1_GYR_HR_Z_SET);
 
    hyst *= (float)((val & 0x60) * 1.0);
    thres *= (float)((val & 0x1F) * 1.0);
}
 
/* ==================================================================
 * 角速度センサーのHighRate割り込み設定(Z軸のみ)を設定する
 * ------------------------------------------------------------------
 * 引数setting: 角速度センサーのHighRate割り込み設定レジスタ値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setGyroHighRateZsetting(char setting){
    return ctrl->wr(1, BNO055P1_GYR_HR_Z_SET, setting);
}
 
/* ==================================================================
 * 角速度センサーのHighRate割り込み設定(Z軸のみ)を設定する
 * ------------------------------------------------------------------
 * 引数hystVal: ヒステリシス
 * 引数thresVal: スレッショルド
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setGyroHighRateZsetting_dps(float hystVal, float thresVal){
    char r = getGyroConfig_0();
    float hyst = 0.0f;
    float thres = 0.0f;
 
    switch(r & 0x07){
        case 0:
            hyst = 62.26;
            thres = 62.5;
            break;
        case 1:
            hyst = 31.13;
            thres = 31.25;
            break;
        case 2:
            hyst = 15.56;
            thres = 15.62;
            break;
        case 3:
            hyst = 7.78;
            thres = 7.81;
            break;
        case 4:
            hyst = 3.89;
            thres = 3.90;
            break;
    }
 
    char hystChar = (char)((hystVal / hyst) + 0.5);
    char thresChar = (char)((thresVal / thres) + 0.5);
 
    return ctrl->wr(1, BNO055P1_GYR_HR_Z_SET, ((hystChar << 5) | thresChar));
}
 
/* ==================================================================
 * 角速度センサーのHighRate割り込み継続発生閾値(Z軸のみ)を取得する
 * ------------------------------------------------------------------
 * returns:
 * HighRate割り込み継続発生閾値[単位:ミリ秒]
 */
float BOARDC_BNO055::getGyroHighRateZduration(){
    return (float)(1 + ctrl->rr(1, BNO055P1_GYR_DUR_Z)) * 2.5;
}
 
/* ==================================================================
 * 角速度センサーのHighRate割り込み継続発生閾値(Z軸のみ)を設定する
 * ------------------------------------------------------------------
 * 引数duration: HighRate割り込み継続発生閾値[単位:ミリ秒]
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setGyroHighRateZduration(float duration){
    return ctrl->wr(1, BNO055P1_GYR_HR_Z_SET, (char)(((duration / 2.5) - 1.0) + 0.5));
}
 
/* ==================================================================
 * 角速度センサーのAnyMotion割り込み閾値を取得する
 * ------------------------------------------------------------------
 * returns:
 * AnyMotion割り込み閾値[単位:dps]
 */
float BOARDC_BNO055::getGyroAnyMotionThreashold(){
    char r = getGyroConfig_0();
    float scale = 0.0f;
 
    switch(r & 0x07){
        case 0:
            scale = 1.0f;
            break;
        case 1:
            scale = 0.5;
            break;
        case 2:
            scale = 0.25;
            break;
        case 3:
            scale = 0.125;
            break;
        case 4:
            scale = 0.0625;
            break;
    }
 
    return (1.0f * ctrl->rr(1, BNO055P1_GYR_AM_THRES)) * scale;
}
 
/* ==================================================================
 * 角速度センサーのAnyMotion割り込み閾値を設定する
 * ------------------------------------------------------------------
 * 引数threashold: AnyMotion割り込み閾値[単位:dps]
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setGyroAnyMotionThreashold(float threashold){
    char r = getGyroConfig_0();
    float scale = 0.0f;
 
    switch(r & 0x07){
        case 0:
            scale = 1.0f;
            break;
        case 1:
            scale = 0.5;
            break;
        case 2:
            scale = 0.25;
            break;
        case 3:
            scale = 0.125;
            break;
        case 4:
            scale = 0.0625;
            break;
    }
 
    return ctrl->wr(1, BNO055P1_GYR_AM_THRES, (char)((threashold / scale) + 0.5));
}
 
/* ==================================================================
 * 加速度センサーのAnyMotion割り込み閾値を取得する
 * ------------------------------------------------------------------
 * returns:
 * AnyMotion割り込み設定レジスタ値
 */
char BOARDC_BNO055::getAccAnyMotionSetting(){
    return ctrl->rr(1, BNO055P1_GYR_AM_SET);
}
 
/* ==================================================================
 * 加速度センサーのAnyMotion割り込み閾値を設定する
 * ------------------------------------------------------------------
 * 引数setting: AnyMotion割り込み設定レジスタ値
 * ------------------------------------------------------------------
 * returns:
 * -1           失敗
 * 1            成功
 */
char BOARDC_BNO055::setAccAnyMotionSetting(char setting){
    return ctrl->wr(1, BNO055P1_GYR_AM_SET, setting);
}