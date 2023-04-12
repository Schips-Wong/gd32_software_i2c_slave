# software-i2c_slave in GD32

在GD32F450上实现GPIO-I2C-SLAVE，要求2个支持双边沿中断的GPIO。

| 测试平台 | CPU主频 | I2C速率 | 状态 |
| -------- | ------- | ------- | ---- |
| GD32F450 | 200M    | 100K    | OK   |
| GD32F150 | 72M     | 10K     | OK   |
## 特性

### 允许多例

核心接口和板级代码已经做分离，一个单片机可以支持n个I2C从设备。

具体见`i2c_slave_0.c`的有关实现以快速拓展n个从设备处理。


### 友好的寄存器空间管理

提供接口支持主机写入寄存器时，对值进行检查，具体见`i2c0_reg_will_changed`的实现。

提供接口支持从机主动更新寄存器（参考`i2c0_reg_update_poll`的实现和调用方式）


### 自定义设备地址

修改`i2c_slave_0.h`中的`SW_SLAVE_ADDR_7BIT`即可。

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

