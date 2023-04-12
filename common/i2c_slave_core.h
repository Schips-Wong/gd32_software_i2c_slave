
///////////////////////////////////////////////////////////////////////////////
//
// \brief
//  sw_slave_i2c.h
//
// \version
// v0.0.1: 2022.05.01, Initial version.
///////////////////////////////////////////////////////////////////////////////

#ifndef __SW_SLAVE_I2C_H_
#define __SW_SLAVE_I2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef void (*voidFun)(void);
typedef void (*setIntFun)(int);
typedef uint8_t (*regWillChangeFun)(uint8_t, uint8_t);
typedef int (*checkFun)(void);

#define I2C_STA_IDLE            0
#define I2C_STA_START           1
#define I2C_STA_DATA            2
#define I2C_STA_ACK             3
#define I2C_STA_NACK            4
#define I2C_STA_STOP            5

#define I2C_READ                1
#define I2C_WRITE               0

#define GPIO_DIR_IN             0
#define GPIO_DIR_OUT            1
#define PIN_LOW  RESET
#define PIN_HIGH SET

struct SwSlaveI2C
{
    uint8_t Addr;                   // I2C设备地址
    uint8_t State;                  // I2C通信状态
    uint8_t Rw;                     // I2C读写标志：0-写，1-读
    uint8_t SclFallCnt;             // SCL下降沿计数
    uint8_t Flag;                   // I2C状态标志，BIT0：0-地址无效，1-地址匹配
    uint32_t StartMs;               // I2C通信起始时间，单位ms，用于判断通信是否超时
    uint8_t* RxBuf;                 // 指向接收缓冲区的指针
    uint8_t* TxBuf;                 // 指向发送缓冲区的指针
    uint8_t RxIdx;                  // 接收缓冲区数据写入索引，最大值255
    uint8_t TxIdx;                  // 发送缓冲区数据读取索引，最大值255
    uint16_t MaxBufSize;
    uint32_t TickCnt;
    // 回调函数
    voidFun init;
    regWillChangeFun reg_will_changed;
    setIntFun set_sda_dir;
    checkFun is_scl_trig;
    voidFun clr_scl_rtig_it;
    checkFun is_sda_trig;
    voidFun clr_sda_rtig_it;
    checkFun get_scl_state;
    checkFun get_sda_state;
    voidFun set_sda_pin;
    voidFun clr_sda_pin;
    //setIntFun set_sda_output_value;
};

#if 0

本例程中，对于400kbps速率的I2C通信，在进行代码编译链接时，需要使用-Ofast的优化方式，以提高中断处理函数的执行速度，使程序能正确执行。

如果使用默认的无优化配置，会造成程序无法正确运行。

对于主频比较低的MCU，使用这里提供的软件模拟I2C从机进行I2C通信时，
建议使用100kpbs以下的通信速率，并且注意使用可以提高代码执行速度的代码优化配置。
#endif

/**
  * @brief  Configures slave I2C
  * @param  None
  * @retval None
  */
void i2c_sw_slave_init(struct SwSlaveI2C *pSwSlaveI2C);

void i2c_tick_inc(struct SwSlaveI2C *pSwSlaveI2C);

void i2c_sw_gpio_exti_isr(struct SwSlaveI2C *SwSlaveI2C);

// 为了确保模拟I2C从机通信的可靠性，额外设计了I2C通信超时处理函数。
// 在I2C通信进行的过程中，如果通信出现了中断，则通过超时判断来重置I2C从机状态，确保出现通信异常时可以从异常状态中自动恢复。
// 该函数需要在主流程中调用。
void i2c_sw_check_timeout(struct SwSlaveI2C *pSwSlaveI2C);

#if 0
#include <stdio.h>

int main(int argc, char * argv[])
{
    int i;
    printf("enum I2C_REG_INDEX {\n");
    for(i = 0; i <= 0xff; i++)
    {
        printf("    REG_INDEX_0x%02X, /* %3d */\n", i, i);
    }
    printf("};");
    return 0;
}
#endif
enum I2C_REG_INDEX {
    REG_INDEX_0x00, /*   0 */
    REG_INDEX_0x01, /*   1 */
    REG_INDEX_0x02, /*   2 */
    REG_INDEX_0x03, /*   3 */
    REG_INDEX_0x04, /*   4 */
    REG_INDEX_0x05, /*   5 */
    REG_INDEX_0x06, /*   6 */
    REG_INDEX_0x07, /*   7 */
    REG_INDEX_0x08, /*   8 */
    REG_INDEX_0x09, /*   9 */
    REG_INDEX_0x0A, /*  10 */
    REG_INDEX_0x0B, /*  11 */
    REG_INDEX_0x0C, /*  12 */
    REG_INDEX_0x0D, /*  13 */
    REG_INDEX_0x0E, /*  14 */
    REG_INDEX_0x0F, /*  15 */
    REG_INDEX_0x10, /*  16 */
    REG_INDEX_0x11, /*  17 */
    REG_INDEX_0x12, /*  18 */
    REG_INDEX_0x13, /*  19 */
    REG_INDEX_0x14, /*  20 */
    REG_INDEX_0x15, /*  21 */
    REG_INDEX_0x16, /*  22 */
    REG_INDEX_0x17, /*  23 */
    REG_INDEX_0x18, /*  24 */
    REG_INDEX_0x19, /*  25 */
    REG_INDEX_0x1A, /*  26 */
    REG_INDEX_0x1B, /*  27 */
    REG_INDEX_0x1C, /*  28 */
    REG_INDEX_0x1D, /*  29 */
    REG_INDEX_0x1E, /*  30 */
    REG_INDEX_0x1F, /*  31 */
    REG_INDEX_0x20, /*  32 */
    REG_INDEX_0x21, /*  33 */
    REG_INDEX_0x22, /*  34 */
    REG_INDEX_0x23, /*  35 */
    REG_INDEX_0x24, /*  36 */
    REG_INDEX_0x25, /*  37 */
    REG_INDEX_0x26, /*  38 */
    REG_INDEX_0x27, /*  39 */
    REG_INDEX_0x28, /*  40 */
    REG_INDEX_0x29, /*  41 */
    REG_INDEX_0x2A, /*  42 */
    REG_INDEX_0x2B, /*  43 */
    REG_INDEX_0x2C, /*  44 */
    REG_INDEX_0x2D, /*  45 */
    REG_INDEX_0x2E, /*  46 */
    REG_INDEX_0x2F, /*  47 */
    REG_INDEX_0x30, /*  48 */
    REG_INDEX_0x31, /*  49 */
    REG_INDEX_0x32, /*  50 */
    REG_INDEX_0x33, /*  51 */
    REG_INDEX_0x34, /*  52 */
    REG_INDEX_0x35, /*  53 */
    REG_INDEX_0x36, /*  54 */
    REG_INDEX_0x37, /*  55 */
    REG_INDEX_0x38, /*  56 */
    REG_INDEX_0x39, /*  57 */
    REG_INDEX_0x3A, /*  58 */
    REG_INDEX_0x3B, /*  59 */
    REG_INDEX_0x3C, /*  60 */
    REG_INDEX_0x3D, /*  61 */
    REG_INDEX_0x3E, /*  62 */
    REG_INDEX_0x3F, /*  63 */
    REG_INDEX_0x40, /*  64 */
    REG_INDEX_0x41, /*  65 */
    REG_INDEX_0x42, /*  66 */
    REG_INDEX_0x43, /*  67 */
    REG_INDEX_0x44, /*  68 */
    REG_INDEX_0x45, /*  69 */
    REG_INDEX_0x46, /*  70 */
    REG_INDEX_0x47, /*  71 */
    REG_INDEX_0x48, /*  72 */
    REG_INDEX_0x49, /*  73 */
    REG_INDEX_0x4A, /*  74 */
    REG_INDEX_0x4B, /*  75 */
    REG_INDEX_0x4C, /*  76 */
    REG_INDEX_0x4D, /*  77 */
    REG_INDEX_0x4E, /*  78 */
    REG_INDEX_0x4F, /*  79 */
    REG_INDEX_0x50, /*  80 */
    REG_INDEX_0x51, /*  81 */
    REG_INDEX_0x52, /*  82 */
    REG_INDEX_0x53, /*  83 */
    REG_INDEX_0x54, /*  84 */
    REG_INDEX_0x55, /*  85 */
    REG_INDEX_0x56, /*  86 */
    REG_INDEX_0x57, /*  87 */
    REG_INDEX_0x58, /*  88 */
    REG_INDEX_0x59, /*  89 */
    REG_INDEX_0x5A, /*  90 */
    REG_INDEX_0x5B, /*  91 */
    REG_INDEX_0x5C, /*  92 */
    REG_INDEX_0x5D, /*  93 */
    REG_INDEX_0x5E, /*  94 */
    REG_INDEX_0x5F, /*  95 */
    REG_INDEX_0x60, /*  96 */
    REG_INDEX_0x61, /*  97 */
    REG_INDEX_0x62, /*  98 */
    REG_INDEX_0x63, /*  99 */
    REG_INDEX_0x64, /* 100 */
    REG_INDEX_0x65, /* 101 */
    REG_INDEX_0x66, /* 102 */
    REG_INDEX_0x67, /* 103 */
    REG_INDEX_0x68, /* 104 */
    REG_INDEX_0x69, /* 105 */
    REG_INDEX_0x6A, /* 106 */
    REG_INDEX_0x6B, /* 107 */
    REG_INDEX_0x6C, /* 108 */
    REG_INDEX_0x6D, /* 109 */
    REG_INDEX_0x6E, /* 110 */
    REG_INDEX_0x6F, /* 111 */
    REG_INDEX_0x70, /* 112 */
    REG_INDEX_0x71, /* 113 */
    REG_INDEX_0x72, /* 114 */
    REG_INDEX_0x73, /* 115 */
    REG_INDEX_0x74, /* 116 */
    REG_INDEX_0x75, /* 117 */
    REG_INDEX_0x76, /* 118 */
    REG_INDEX_0x77, /* 119 */
    REG_INDEX_0x78, /* 120 */
    REG_INDEX_0x79, /* 121 */
    REG_INDEX_0x7A, /* 122 */
    REG_INDEX_0x7B, /* 123 */
    REG_INDEX_0x7C, /* 124 */
    REG_INDEX_0x7D, /* 125 */
    REG_INDEX_0x7E, /* 126 */
    REG_INDEX_0x7F, /* 127 */
    REG_INDEX_0x80, /* 128 */
    REG_INDEX_0x81, /* 129 */
    REG_INDEX_0x82, /* 130 */
    REG_INDEX_0x83, /* 131 */
    REG_INDEX_0x84, /* 132 */
    REG_INDEX_0x85, /* 133 */
    REG_INDEX_0x86, /* 134 */
    REG_INDEX_0x87, /* 135 */
    REG_INDEX_0x88, /* 136 */
    REG_INDEX_0x89, /* 137 */
    REG_INDEX_0x8A, /* 138 */
    REG_INDEX_0x8B, /* 139 */
    REG_INDEX_0x8C, /* 140 */
    REG_INDEX_0x8D, /* 141 */
    REG_INDEX_0x8E, /* 142 */
    REG_INDEX_0x8F, /* 143 */
    REG_INDEX_0x90, /* 144 */
    REG_INDEX_0x91, /* 145 */
    REG_INDEX_0x92, /* 146 */
    REG_INDEX_0x93, /* 147 */
    REG_INDEX_0x94, /* 148 */
    REG_INDEX_0x95, /* 149 */
    REG_INDEX_0x96, /* 150 */
    REG_INDEX_0x97, /* 151 */
    REG_INDEX_0x98, /* 152 */
    REG_INDEX_0x99, /* 153 */
    REG_INDEX_0x9A, /* 154 */
    REG_INDEX_0x9B, /* 155 */
    REG_INDEX_0x9C, /* 156 */
    REG_INDEX_0x9D, /* 157 */
    REG_INDEX_0x9E, /* 158 */
    REG_INDEX_0x9F, /* 159 */
    REG_INDEX_0xA0, /* 160 */
    REG_INDEX_0xA1, /* 161 */
    REG_INDEX_0xA2, /* 162 */
    REG_INDEX_0xA3, /* 163 */
    REG_INDEX_0xA4, /* 164 */
    REG_INDEX_0xA5, /* 165 */
    REG_INDEX_0xA6, /* 166 */
    REG_INDEX_0xA7, /* 167 */
    REG_INDEX_0xA8, /* 168 */
    REG_INDEX_0xA9, /* 169 */
    REG_INDEX_0xAA, /* 170 */
    REG_INDEX_0xAB, /* 171 */
    REG_INDEX_0xAC, /* 172 */
    REG_INDEX_0xAD, /* 173 */
    REG_INDEX_0xAE, /* 174 */
    REG_INDEX_0xAF, /* 175 */
    REG_INDEX_0xB0, /* 176 */
    REG_INDEX_0xB1, /* 177 */
    REG_INDEX_0xB2, /* 178 */
    REG_INDEX_0xB3, /* 179 */
    REG_INDEX_0xB4, /* 180 */
    REG_INDEX_0xB5, /* 181 */
    REG_INDEX_0xB6, /* 182 */
    REG_INDEX_0xB7, /* 183 */
    REG_INDEX_0xB8, /* 184 */
    REG_INDEX_0xB9, /* 185 */
    REG_INDEX_0xBA, /* 186 */
    REG_INDEX_0xBB, /* 187 */
    REG_INDEX_0xBC, /* 188 */
    REG_INDEX_0xBD, /* 189 */
    REG_INDEX_0xBE, /* 190 */
    REG_INDEX_0xBF, /* 191 */
    REG_INDEX_0xC0, /* 192 */
    REG_INDEX_0xC1, /* 193 */
    REG_INDEX_0xC2, /* 194 */
    REG_INDEX_0xC3, /* 195 */
    REG_INDEX_0xC4, /* 196 */
    REG_INDEX_0xC5, /* 197 */
    REG_INDEX_0xC6, /* 198 */
    REG_INDEX_0xC7, /* 199 */
    REG_INDEX_0xC8, /* 200 */
    REG_INDEX_0xC9, /* 201 */
    REG_INDEX_0xCA, /* 202 */
    REG_INDEX_0xCB, /* 203 */
    REG_INDEX_0xCC, /* 204 */
    REG_INDEX_0xCD, /* 205 */
    REG_INDEX_0xCE, /* 206 */
    REG_INDEX_0xCF, /* 207 */
    REG_INDEX_0xD0, /* 208 */
    REG_INDEX_0xD1, /* 209 */
    REG_INDEX_0xD2, /* 210 */
    REG_INDEX_0xD3, /* 211 */
    REG_INDEX_0xD4, /* 212 */
    REG_INDEX_0xD5, /* 213 */
    REG_INDEX_0xD6, /* 214 */
    REG_INDEX_0xD7, /* 215 */
    REG_INDEX_0xD8, /* 216 */
    REG_INDEX_0xD9, /* 217 */
    REG_INDEX_0xDA, /* 218 */
    REG_INDEX_0xDB, /* 219 */
    REG_INDEX_0xDC, /* 220 */
    REG_INDEX_0xDD, /* 221 */
    REG_INDEX_0xDE, /* 222 */
    REG_INDEX_0xDF, /* 223 */
    REG_INDEX_0xE0, /* 224 */
    REG_INDEX_0xE1, /* 225 */
    REG_INDEX_0xE2, /* 226 */
    REG_INDEX_0xE3, /* 227 */
    REG_INDEX_0xE4, /* 228 */
    REG_INDEX_0xE5, /* 229 */
    REG_INDEX_0xE6, /* 230 */
    REG_INDEX_0xE7, /* 231 */
    REG_INDEX_0xE8, /* 232 */
    REG_INDEX_0xE9, /* 233 */
    REG_INDEX_0xEA, /* 234 */
    REG_INDEX_0xEB, /* 235 */
    REG_INDEX_0xEC, /* 236 */
    REG_INDEX_0xED, /* 237 */
    REG_INDEX_0xEE, /* 238 */
    REG_INDEX_0xEF, /* 239 */
    REG_INDEX_0xF0, /* 240 */
    REG_INDEX_0xF1, /* 241 */
    REG_INDEX_0xF2, /* 242 */
    REG_INDEX_0xF3, /* 243 */
    REG_INDEX_0xF4, /* 244 */
    REG_INDEX_0xF5, /* 245 */
    REG_INDEX_0xF6, /* 246 */
    REG_INDEX_0xF7, /* 247 */
    REG_INDEX_0xF8, /* 248 */
    REG_INDEX_0xF9, /* 249 */
    REG_INDEX_0xFA, /* 250 */
    REG_INDEX_0xFB, /* 251 */
    REG_INDEX_0xFC, /* 252 */
    REG_INDEX_0xFD, /* 253 */
    REG_INDEX_0xFE, /* 254 */
    REG_INDEX_0xFF, /* 255 */
};

#ifdef __cplusplus
}
#endif


#endif /*__SW_SLAVE_I2C_H_*/
