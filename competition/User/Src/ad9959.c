//#include "ad9959.h"
//
///* ========== DWT 微秒延时（适用于 STM32H7） ========== */
//static int dwt_initialized = 0;
//
//static void DWT_Init(void)
//{
//    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
//    {
//        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//        DWT->CYCCNT = 0;
//        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
//        dwt_initialized = 1;
//    }
//}
//
//static void DWT_Delay_us(uint32_t us)
//{
//    if (!dwt_initialized) DWT_Init();
//    uint32_t start = DWT->CYCCNT;
//    uint32_t ticks = us * (SystemCoreClock / 1000000);
//    while ((DWT->CYCCNT - start) < ticks);
//}
//
///* ========== IO 映射 ========== */
//driverIO SDIO0 = {SDIO0_GPIO_Port, SDIO0_Pin};
//driverIO SDIO1 = {SDIO1_GPIO_Port, SDIO1_Pin};
//driverIO SDIO2 = {SDIO2_GPIO_Port, SDIO2_Pin};
//driverIO SDIO3 = {SDIO3_GPIO_Port, SDIO3_Pin};
//driverIO PDC   = {PDC_GPIO_Port, PDC_Pin};
//driverIO RST   = {RST_GPIO_Port, RST_Pin};
//driverIO SCLK  = {SCLK_GPIO_Port, SCLK_Pin};
//driverIO CS    = {CS_GPIO_Port, CS_Pin};
//driverIO UPDATE = {UPDATE_GPIO_Port, UPDATE_Pin};
//driverIO PS0   = {PS0_GPIO_Port, PS0_Pin};
//driverIO PS1   = {PS1_GPIO_Port, PS1_Pin};
//driverIO PS2   = {PS2_GPIO_Port, PS2_Pin};
//driverIO PS3   = {PS3_GPIO_Port, PS3_Pin};
//
///* ========== 通道使能数据 ========== */
//uint8_t CSR_DATA0[1] = {0x10};
//uint8_t CSR_DATA1[1] = {0x20};
//uint8_t CSR_DATA2[1] = {0x40};
//uint8_t CSR_DATA3[1] = {0x80};
//uint8_t CSR_DATA[4] = {0x10, 0x20, 0x40, 0x80};
//
//uint8_t FR2_DATA[2] = {0x00, 0x00};
//
//uint32_t SinFre[4] = {1000, 1000, 1000, 1000};
//uint32_t SinAmp[4] = {1023, 1023, 1023, 1023};
//uint32_t SinPhr[4] = {0, 0, 0, 0};
//
///* ========== 微秒延时（对用户可见） ========== */
//void delay_9959(uint32_t length)
//{
//    DWT_Delay_us(length);
//}
//
///* ========== 初始化 IO ========== */
//void InitIO_9959(void)
//{
//    WRT(PDC, 0);
//    WRT(CS, 1);
//    WRT(SCLK, 0);
//    WRT(UPDATE, 0);
//    WRT(PS0, 0);
//    WRT(PS1, 0);
//    WRT(PS2, 0);
//    WRT(PS3, 0);
//    WRT(SDIO0, 0);
//    WRT(SDIO1, 0);
//    WRT(SDIO2, 0);
//    WRT(SDIO3, 0);
//}
//
///* ========== 复位 ========== */
//void InitReset(void)
//{
//    WRT(RST, 0);
//    delay_9959(1);
//    WRT(RST, 1);
//    delay_9959(30);
//    WRT(RST, 0);
//}
//
///* ========== IO 更新脉冲 ========== */
//void AD9959_IO_Update(void)
//{
//    WRT(UPDATE, 1);
//    delay_9959(1);
//    WRT(UPDATE, 0);
//}
//
///* ========== 写寄存器 ========== */
//void WriteData_AD9959(uint8_t RegisterAddress, uint8_t NumberofRegisters, uint8_t *RegisterData)
//{
//    uint8_t ControlValue = RegisterAddress;
//    uint8_t i, RegisterIndex, ValueToWrite;
//
//    WRT(SCLK, 0);
//    WRT(CS, 0);
//    // 发送地址
//    for (i = 0; i < 8; i++)
//    {
//        WRT(SCLK, 0);
//        WRT(SDIO0, (ControlValue & 0x80) ? 1 : 0);
//        WRT(SCLK, 1);
//        ControlValue <<= 1;
//        delay_9959(2);
//    }
//    WRT(SCLK, 0);
//    // 发送数据
//    for (RegisterIndex = 0; RegisterIndex < NumberofRegisters; RegisterIndex++)
//    {
//        ValueToWrite = RegisterData[RegisterIndex];
//        for (i = 0; i < 8; i++)
//        {
//            WRT(SCLK, 0);
//            WRT(SDIO0, (ValueToWrite & 0x80) ? 1 : 0);
//            WRT(SCLK, 1);
//            ValueToWrite <<= 1;
//            delay_9959(2);
//        }
//        WRT(SCLK, 0);
//    }
//    WRT(CS, 1);
//}
//
///* ========== 频率/相位/幅度字转换 ========== */
//void Freq2Word(double f, uint8_t *fWord)
//{
//    uint32_t Temp = (uint32_t)(f * 8.589934592);
//    fWord[3] = (uint8_t)Temp;
//    fWord[2] = (uint8_t)(Temp >> 8);
//    fWord[1] = (uint8_t)(Temp >> 16);
//    fWord[0] = (uint8_t)(Temp >> 24);
//}
//
//void Amp2Word(uint16_t A, uint8_t *AWord)
//{
//    uint16_t Temp = A | 0x1000;
//    AWord[2] = (uint8_t)Temp;
//    AWord[1] = (uint8_t)(Temp >> 8);
//    AWord[0] = 0x00;
//}
//
//void Phase2Word(uint16_t Phase, uint8_t *PWord)
//{
//    uint16_t Temp = (uint16_t)(45.511111111 * Phase);
//    PWord[1] = (uint8_t)Temp;
//    PWord[0] = (uint8_t)(Temp >> 8);
//}
//
///* ========== 通道选择 ========== */
//void Channel_Select(uint8_t Channel)
//{
//    if (Channel > 3) return;
//    WriteData_AD9959(CSR_ADD, 1, &CSR_DATA[Channel]);
//}
//
///* ========== 公共 API ========== */
//void Write_Frequence(uint8_t Channel, uint32_t Freq)
//{
//    if (Freq > 500000000 || Freq < 1) Freq = 114514;
//    uint8_t fWord[4];
//    Freq2Word((double)Freq, fWord);
//    Channel_Select(Channel);
//    WriteData_AD9959(CFTW0_ADD, 4, fWord);
//}
//
//void Write_Amplitude(uint8_t Channel, uint16_t Ampli)
//{
//    if (Ampli > 1023) Ampli = 114;
//    uint8_t AWord[3];
//    Amp2Word(Ampli, AWord);
//    Channel_Select(Channel);
//    WriteData_AD9959(ACR_ADD, 3, AWord);
//}
//
//void Write_Phase(uint8_t Channel, uint16_t Phase)
//{
//    if (Phase > 359) Phase = 0;
//    uint8_t PWord[2];
//    Phase2Word(Phase, PWord);
//    Channel_Select(Channel);
//    WriteData_AD9959(CPOW0_ADD, 2, PWord);
//}
//
///* ========== 初始化 ========== */
//void Init_AD9959(void)
//{
//    DWT_Init();   // 关键！启用 DWT 延时
//
//    uint8_t FR1_DATA[3] = {0xD3, 0x00, 0x00};
//    uint8_t CFR_DATA[3] = {0x00, 0x03, 0x00};
//
//    InitIO_9959();
//    InitReset();
//
//    WriteData_AD9959(FR1_ADD, 3, FR1_DATA);
//    WriteData_AD9959(CFR_ADD, 3, CFR_DATA);
//
//    // 初始化四个通道
//    Write_Phase(0, SinPhr[0]);
//    Write_Phase(1, SinPhr[1]);
//    Write_Phase(2, SinPhr[2]);
//    Write_Phase(3, SinPhr[3]);
//
//    Write_Frequence(0, SinFre[0]);
//    Write_Frequence(1, SinFre[1]);
//    Write_Frequence(2, SinFre[2]);
//    Write_Frequence(3, SinFre[3]);
//
//    Write_Amplitude(0, SinAmp[0]);
//    Write_Amplitude(1, SinAmp[1]);
//    Write_Amplitude(2, SinAmp[2]);
//    Write_Amplitude(3, SinAmp[3]);
//}
//
///* ========== 以下为扩展功能（未使用，保留） ========== */
//void ReadData_AD9959(uint8_t RegisterAddress, uint8_t NumberofRegisters, uint8_t *RegisterData)
//{
//    // ... 原有实现（此处省略，因当前未用，不影响编译） ...
//    (void)RegisterAddress; (void)NumberofRegisters; (void)RegisterData;
//}
//
//void AD9959_error(void) {}
//void Stop_AD9959(void) {}
//void Sweep_Frequency(uint8_t Channel, uint32_t Start_Freq, uint32_t Stop_Freq, uint32_t Step, uint32_t time, uint8_t NO_DWELL)
//{
//    (void)Channel; (void)Start_Freq; (void)Stop_Freq; (void)Step; (void)time; (void)NO_DWELL;
//}
//uint32_t Get_Freq(void) { return 0; }
//uint8_t Get_Amp(void) { return 0; }
//void SET_2FSK(uint8_t Channel, double f_start, double f_stop) { (void)Channel; (void)f_start; (void)f_stop; }
//void SET_2ASK(uint8_t Channel, double f, uint16_t A_start, uint16_t A_stop) { (void)Channel; (void)f; (void)A_start; (void)A_stop; }
