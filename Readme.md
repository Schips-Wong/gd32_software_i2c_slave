# software-i2c_slave in GD32

在GD32F450上实现GPIO-I2C-SLAVE，要求2个支持双边沿中断的GPIO。

## 特性

### 友好的寄存器空间管理

通过`reg_changed`接口，来快速响应主机的写入请求。

```c
void reg_changed(uint8_t reg, uint8_t value)
{
    if(reg == 0x00)
    {
        i2c_debug("REG 0x00 VALUE CHANGE to [0x%02x]\r\n", value);
    }
    if(reg == 0x01)
    {
        i2c_debug("REG 0x01 VALUE CHANGE to [0x%02x]\r\n", value);
    }
}
```

### 自定义设备地址

修改`i2c_slave.h`中的`SW_SLAVE_ADDR_7BIT`即可。

### 支持连续读写

支持连续写、连续读；在Linux平台验证结果如下：

```bash
# i2ctransfer -f -y 0 w4@0x51 0x00 0xa1 0xa2 0xa3
# i2ctransfer -f -y 0 w1@0x51 0x00 r1
0xa1
# i2ctransfer -f -y 0 w1@0x51 0x00 r3
0xa1 0xa2 0xa3
# i2ctransfer -f -y 0 w1@0x51 0x01 r2
0xa2 0xa3
```

## 注意事项

1、如果为了做移植，请修改`i2c_slave.c`中所有带`CHECK_PORTING`的函数接口。

2、可以在`systick`中添加`i2c_tick_inc`来做异常处理。实测，可以不调用`i2c_sw_check_timeout`；如果需要调用，需要确保`i2c_sw_check_timeout`的执行间隔大于10ms。

3、速率建议不超过100K左右；不支持SCL展频。

4、目前SDA、SCL都使用同一个ISR，请在你的实际项目中，检查对应的ISR
