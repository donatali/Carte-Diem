
/*
    REGISTERS AND BIT FIELDS FOR ICM20948 - IMU
        IN C
*/

#define ICM20948_I2C_ADDR 0x69 

/*
REGISTERS

Bank0 : sensor data + some ctrl registers
Bank1 : gyro + accel config registers
Bank2 : more sensor config + FIFO + interrupts
Bank3 : Advanced features
*/
#define ICM20948_REG_BANK_SEL    0x7F

//bank0
#define ICM20948_WHO_AM_I        0x00 //should be 0xEA
#define ICM20948_USER_CTRL       0x03
#define ICM20948_LP_CONFIG       0x05
#define ICM20948_PWR_MGMT_1      0x06
#define ICM20948_PWR_MGMT_2      0x07
#define ICM20948_INT_PIN_CFG     0x0F
#define ICM20948_INT_ENABLE      0x10
#define ICM20948_INT_ENABLE1     0x11
#define ICM20948_INT_ENABLE2     0x12
#define ICM20948_INT_ENABLE3     0x13
#define ICM20948_I2C_MST_STATUS  0x17
#define ICM20948_INT_STATUS      0x19
#define ICM20948_INT_STATUS1     0x1A
#define ICM20948_INT_STATUS2     0x1B
#define ICM20948_INT_STATUS3     0x1C
#define ICM20948_DELAY_TIMEH     0x28
#define ICM20948_DELAY_TIMEH     0x29
#define ICM20948_DATA_RDY_STATUS 0x74

#define ICM20948_ACCEL_XOUT_H    0x2D
#define ICM20948_ACCEL_XOUT_L    0x2E
#define ICM20948_ACCEL_YOUT_H    0x2F
#define ICM20948_ACCEL_YOUT_L    0x30
#define ICM20948_ACCEL_ZOUT_H    0x31
#define ICM20948_ACCEL_ZOUT_L    0x32

#define ICM20948_GYRO_XOUT_H     0x33
#define ICM20948_GYRO_XOUT_L     0x34
#define ICM20948_GYRO_YOUT_H     0x35
#define ICM20948_GYRO_YOUT_L     0x36
#define ICM20948_GYRO_ZOUT_H     0x37
#define ICM20948_GYRO_ZOUT_L     0x38

#define ICM20948_TEMP_OUT_H      0x39
#define ICM20948_TEMP_OUT_L      0x3A

#define ICM20948_FIFO_EN_1       0x66
#define ICM20948_FIFO_EN_2       0x67
#define ICM20948_FIFO_RST        0x68
#define ICM20948_FIFO_MODE       0x69
#define ICM20948_FIFO_COUNTH     0x70
#define ICM20948_FIFO_COUNTL     0x71
#define ICM20948_FIFO_R_W        0x72
#define ICM20948_FIFO_CFG        0x76

//bank1. unneeded these are values to test GYRO/ACCEL and put in offsets into registers. reset after power cycle.
#define ICM20948_SELF_TEST_X_GYRO    0x02
#define ICM20948_SELF_TEST_Y_GYRO    0x03
#define ICM20948_SELF_TEST_Z_GYRO    0x04

#define ICM20948_SELF_TEST_X_ACCEL   0x0E
#define ICM20948_SELF_TEST_Y_ACCEL   0x0F
#define ICM20948_SELF_TEST_Z_ACCEL   0x10

#define ICM20948_XA_OFFS_H           0x14
#define ICM20948_XA_OFFS_L           0x15
#define ICM20948_YA_OFFS_H           0x17
#define ICM20948_YA_OFFS_L           0x18

//bank2
#define ICM20948_GYRO_SMPLRT_DIV     0x00
#define ICM20948_GYRO_CONFIG_1       0x01 //select dps
#define ICM20948_GYRO_CONFIG_2       0x02 //filter config for low-power mode Table 17

#define ICM20948_ACCEL_SMPLRT_DIV_1  0x10 //MSB
#define ICM20948_ACCEL_SMPLRT_DIV_2  0x11 //LSB
#define ICM20948_ACCEL_INTEL_CTRL    0x12 //for wake on motion interrupts
#define ICM20948_ACCEL_WOM_THR       0x13 //threshold for wake on motion
#define ICM20948_ACCEL_CONFIG        0x14 //sellect g
#define ICM20948_ACCEL_CONFIG_2      0x15 //self test enable and # samples averaged

#define ICM20948_FSYNC_CONFIG    0x52 // for syncing with other sensors
#define ICM20948_MOD_CTRL_USR    0x54 //enable DMP in low power    

//bank3 not used. for other I2C periphs talking to IMU
#define ICM20948_I2C_MST_ODR_CONFIG 0x00
#define ICM20948_I2C_MST_CTRL       0x01
#define ICM20948_I2C_MST_DELAY_CTRL 0x02
#define ICM20948_I2C_SLV0_ADDR      0x03
#define ICM20948_I2C_SLV0_REG       0x04
#define ICM20948_I2C_SLV0_CTRL      0x05
#define ICM20948_I2C_SLV0_DO        0x06
#define ICM20948_I2C_SLV1_ADDR      0x07
#define ICM20948_I2C_SLV1_REG       0x08
#define ICM20948_I2C_SLV1_CTRL      0x09
#define ICM20948_I2C_SLV1_DO        0x0A
#define ICM20948_I2C_SLV2_ADDR      0x0B
#define ICM20948_I2C_SLV2_REG       0x0C
#define ICM20948_I2C_SLV2_CTRL      0x0D
#define ICM20948_I2C_SLV2_DO        0x0E
#define ICM20948_I2C_SLV3_ADDR      0x0F
#define ICM20948_I2C_SLV3_REG       0x10
#define ICM20948_I2C_SLV3_CTRL      0x11
#define ICM20948_I2C_SLV3_DO        0x12
#define ICM20948_I2C_SLV4_ADDR      0x13
#define ICM20948_I2C_SLV4_REG       0x14
#define ICM20948_I2C_SLV4_CTRL      0x15
#define ICM20948_I2C_SLV4_DO        0x16
#define ICM20948_I2C_SLV4_DI        0x17

//magnetometer registers
#define ICM20948_MAGNETO_WIA2    0x01 //who am i
#define ICM20948_MAGNETO_ST1     0x10 //data status 1
#define ICM20948_MAGNETO_HXL     0x11 //xaxis data L
#define ICM20948_MAGNETO_HXH     0x12 //xaxis data H
#define ICM20948_MAGNETO_HYL     0x13 //yaxis data L
#define ICM20948_MAGNETO_HYH     0x14 //yaxis data H
#define ICM20948_MAGNETO_HZL     0x15 //zaxis data L
#define ICM20948_MAGNETO_HZH     0x16 //zaxis data H
#define ICM20948_MAGNETO_ST2     0x18 //data status 2
#define ICM20948_MAGNETO_CNTL2   0x31 //control settings 2
#define ICM20948_MAGNETO_CNTL3   0x32 //control settings 3

/*
BIT FIELDS

*/
// -------BANK0-------
//USER_CTRL
#define USER_CTRL_DMP_EN_MASK       0x80
#define USER_CTRL_DMP_EN_SHIFT      7u
#define USER_CTRL_DMP_EN(x)         (((uint8_t)(((uint8_t)(x)) << USER_CTRL_DMP_EN_SHIFT)) & USER_CTRL_DMP_EN_MASK)
#define USER_CTRL_FIFO_EN_MASK      0x40
#define USER_CTRL_FIFO_EN_SHIFT     6u
#define USER_CTRL_FIFO_EN(x)        (((uint8_t)(((uint8_t)(x)) << USER_CTRL_FIFO_EN_SHIFT)) & USER_CTRL_FIFO_EN_MASK)
#define USER_CTRL_I2C_MST_EN_MASK   0x20
#define USER_CTRL_I2C_MST_EN_SHIFT  5u
#define USER_CTRL_I2C_MST_EN(x)     (((uint8_t)(((uint8_t)(x)) << USER_CTRL_I2C_MST_EN_SHIFT)) & USER_CTRL_I2C_MST_EN_MASK)
#define USER_CTRL_I2C_IF_DIS_MASK   0x10
#define USER_CTRL_I2C_IF_DIS_SHIFT  4u
#define USER_CTRL_I2C_IF_DIS(X)     (((uint8_t)(((uint8_t)(x)) << USER_CTRL_I2C_IF_DIS_SHIFT)) & USER_CTRL_I2C_IF_DIS_MASK)
#define USER_CTRL_DMP_RST_MASK      0x08
#define USER_CTRL_DMP_RST_SHIFT     3u
#define USER_CTRL_DMP_RST(x)        (((uint8_t)(((uint8_t)(x)) << USER_CTRL_DMP_RST_SHIFT)) & USER_CTRL_DMP_RST_MASK)
#define USER_CTRL_SRAM_RST_MASK     0x04
#define USER_CTRL_SRAM_RST_SHIFT    2u
#define USER_CTRL_SRAM_RST(x)       (((uint8_t)(((uint8_t)(x)) << USER_CTRL_SRAM_RST_SHIFT)) & USER_CTRL_SRAM_RST_MASK)
#define USER_CTRL_I2C_MST_RST_MASK  0x10
#define USER_CTRL_I2C_MST_RST_SHIFT 1u
#define USER_CTRL_I2C_MST_RST(x)    (((uint8_t)(((uint8_t)(x)) << USER_CTRL_I2C_MST_RST_SHIFT)) & USER_CTRL_I2C_MST_RST_MASK)
//LP_CONFIG
#define LP_CFG_I2C_MST_CYCLE_MASK   0x40
#define LP_CFG_I2C_MST_CYCLE_SHIFT  6u
#define LP_CFG_I2C_MST_CYCLE(x)     (((uint8_t)(((uint8_t)(x)) << LP_CFG_I2C_MST_CYCLE_SHIFT)) & LP_CFG_I2C_MST_CYCLE_MASK)
#define LP_CFG_ACCEL_CYCLE_MASK     0x20
#define LP_CFG_ACCEL_CYCLE_SHIFT    5u
#define LP_CFG_ACCEL_CYCLE(x)       (((uint8_t)(((uint8_t)(x)) << LP_CFG_ACCEL_CYCLE_SHIFT)) & LP_CFG_ACCEL_CYCLE_MASK)
#define LP_CFG_GYRO_CYCLE_MASK      0x10
#define LP_CFG_GYRO_CYCLE_SHIFT     4u
#define LP_CFG_GYRO_CYCLE(x)        (((uint8_t)(((uint8_t)(x)) << LP_CFG_GYRO_CYCLE_SHIFT)) & LP_CFG_GYRO_CYCLE_MASK)
//PWR_MGMT_1
#define PWR_MGMT1_DEV_RST_MASK      0x80
#define PWR_MGMT1_DEV_RST_SHIFT     7u
#define PWR_MGMT1_DEV_RST(x)        (((uint8_t)(((uint8_t)(x)) << PWR_MGMT1_DEV_RST_SHIFT)) & PWR_MGMT1_DEV_RST_MASK)
#define PWR_MGMT1_SLEEP_MASK        0x40
#define PWR_MGMT1_SLEEP_SHIFT       6u
#define PWR_MGMT1_SLEEP(x)          (((uint8_t)(((uint8_t)(x)) << PWR_MGMT1_SLEEP_SHIFT)) & PWR_MGMT1_SLEEP_MASK)
#define PWR_MGMT1_LP_EN_MASK        0x20
#define PWR_MGMT1_LP_EN_SHIFT       5u
#define PWR_MGMT1_LP_EN(x)          (((uint8_t)(((uint8_t)(x)) << PWR_MGMT1_LP_EN_SHIFT)) & PWR_MGMT1_LP_EN_MASK)
#define PWR_MGMT1_TEMP_DIS_MASK     0x08
#define PWR_MGMT1_TEMP_DIS_SHIFT    3u
#define PWR_MGMT1_TEMP_DIS(x)       (((uint8_t)(((uint8_t)(x)) << PWR_MGMT1_TEMP_DIS_SHIFT)) & PWR_MGMT1_TEMP_DIS_MASK)
#define PWR_MGMT1_CLKSEL_MASK       0x07
#define PWR_MGMT1_CLKSEL_SHIFT      0u
#define PWR_MGMT1_CLKSEL(x)         (((uint8_t)(((uint8_t)(x)) << PWR_MGMT1_CLKSEL_SHIFT)) & PWR_MGMT1_CLKSEL_MASK)
//PWR_MGMT_2
#define PWR_MGMT2_DIS_ACCEL_MASK    0x38
#define PWR_MGMT2_DIS_ACCEL_SHIFT   3u
#define PWR_MGMT2_DIS_ACCEL(x)      (((uint8_t)(((uint8_t)(x)) << PWR_MGMT2_DIS_ACCEL_SHIFT)) & PWR_MGMT2_DIS_ACCEL_MASK)
#define PWR_MGMT2_DIS_GYRO_MASK     0x07
#define PWR_MGMT2_DIS_GYRO_SHIFT    0u
#define PWR_MGMT2_DIS_GYRO(x)       (((uint8_t)(((uint8_t)(x)) << PWR_MGMT2_DIS_GYRO_SHIFT)) & PWR_MGMT2_DIS_GYRO_MASK)
//INT_PIN_CFG
#define INT_CFG_INT1_ACTL_MASK      0x80
#define INT_CFG_INT1_ACTL_SHIFT     7u
#define INT_CFG_INT1_ACTL(x)        (((uint8_t)(((uint8_t)(x)) << INT_CFG_INT1_ACTL_SHIFT)) & INT_CFG_INT1_ACTL_MASK)
#define INT_CFG_INT1_OPEN_MASK      0x40
#define INT_CFG_INT1_OPEN_SHIFT     6u
#define INT_CFG_INT1_OPEN(x)        (((uint8_t)(((uint8_t)(x)) << INT_CFG_INT1_OPEN_SHIFT)) & INT_CFG_INT1_OPEN_MASK)
#define INT_CFG_INT1_LATCH_EN_MASK  0x20
#define INT_CFG_INT1_LATCH_EN_SHIFT 5u
#define INT_CFG_INT1_LATCH_EN(x)    (((uint8_t)(((uint8_t)(x)) << INT_CFG_INT1_LATCH_EN_SHIFT)) & INT_CFG_INT1_LATCH_EN_MASK)
#define INT_CFG_ANYRD_CLEAR_MASK    0x10
#define INT_CFG_ANYRD_CLEAR_SHIFT   4u
#define INT_CFG_ANYRD_CLEAR(x)      (((uint8_t)(((uint8_t)(x)) << INT_CFG_ANYRD_CLEAR_SHIFT)) & INT_CFG_ANYRD_CLEAR_MASK)
#define INT_CFG_ACTL_FSYNC_MASK     0x08
#define INT_CFG_ACTL_FSYNC_SHIFT    3u
#define INT_CFG_ACTL_FSYNC(x)       (((uint8_t)(((uint8_t)(x)) << INT_CFG_ACTL_FSYNC_SHIFT)) & INT_CFG_ACTL_FSYNC_MASK)
#define INT_CFG_FSYNC_MODE_EN_MASK  0x04
#define INT_CFG_FSYNC_MODE_EN_SHIFT 2u
#define INT_CFG_FSYNC_MODE_EN(x)    (((uint8_t)(((uint8_t)(x)) << INT_CFG_FSYNC_MODE_EN_SHIFT)) & INT_CFG_FSYNC_MODE_EN_MASK)
#define INT_CFG_BYPASS_EN_MASK      0x02
#define INT_CFG_BYPASS_EN_SHIFT     1u
#define INT_CFG_BYPASS_EN(x)        (((uint8_t)(((uint8_t)(x)) << INT_CFG_BYPASS_EN_SHIFT)) & INT_CFG_BYPASS_EN_MASK)
//INT_ENABLE
#define INT_EN_REG_WOF_EN_MASK      0x80
#define INT_EN_REG_WOF_EN_SHIFT     7u
#define INT_EN_REG_WOF_EN(x)        (((uint8_t)(((uint8_t)(x)) << INT_EN_REG_WOF_EN_SHIFT)) & INT_EN_REG_WOF_EN_MASK)
#define INT_EN_WOM_INT_EN_MASK      0x08
#define INT_EN_WOM_INT_EN_SHIFT     3u
#define INT_EN_WOM_INT_EN(x)        (((uint8_t)(((uint8_t)(x)) << INT_EN_WOM_INT_EN_SHIFT)) & INT_EN_WOM_INT_EN_MASK)
#define INT_EN_PLL_RDY_EN_MASK      0x04
#define INT_EN_PLL_RDY_EN_SHIFT     2u
#define INT_EN_PLL_RDY_EN(x)        (((uint8_t)(((uint8_t)(x)) << INT_EN_PLL_RDY_EN_SHIFT)) & INT_EN_PLL_RDY_EN_MASK)
#define INT_EN_DMP_INT1_EN_MASK     0x02
#define INT_EN_DMP_INT1_EN_SHIFT    1u
#define INT_EN_DMP_INT1_EN(x)       (((uint8_t)(((uint8_t)(x)) << INT_EN_DMP_INT1_EN_SHIFT)) & INT_EN_DMP_INT1_EN_MASK)
#define INT_EN_I2C_MST_INT_EN_MASK  0x01
#define INT_EN_I2C_MST_INT_EN_SHIFT 0u
#define INT_EN_I2C_MST_INT_EN(x)    (((uint8_t)(((uint8_t)(x)) << INT_EN_I2C_MST_INT_EN_SHIFT)) & INT_EN_I2C_MST_INT_EN_MASK)
//INT_ENABLE1
#define INT_EN1_RAW_RDY_EN_MASK     0x01
#define INT_EN1_RAW_RDY_EN_SHIFT    0u
#define INT_EN1_RAW_RDY_EN(x)       (((uint8_t)(((uint8_t)(x)) << INT_EN1_RAW_RDY_EN_SHIFT)) & INT_EN1_RAW_RDY_EN_MASK)
//INT_ENABLE2
#define INT_EN2_FIFO_OF_EN_MASK     0x1F
#define INT_EN2_FIFO_OF_EN_SHIFT    0u
#define INT_EN2_FIFO_OF_EN(x)       (((uint8_t)(((uint8_t)(x)) << INT_EN2_FIFO_OF_EN_SHIFT)) & INT_EN2_FIFO_OF_EN_MASK)
//INT_ENABLE3
#define INT_EN3_FIFO_WM_EN_MASK     0x1F
#define INT_EN3_FIFO_WM_EN_SHIFT    0u
#define INT_EN3_FIFO_WM_EN(x)       (((uint8_t)(((uint8_t)(x)) << INT_EN3_FIFO_WM_EN_SHIFT)) & INT_EN3_FIFO_WM_EN_MASK)
//FIFO_EN1
#define FIFO_EN1_SLV3_EN_MASK       0x08
#define FIFO_EN1_SLV3_EN_SHIFT      3u
#define FIFO_EN1_SLV3_EN(x)         (((uint8_t)(((uint8_t)(x)) << FIFO_EN1_SLV3_EN_SHIFT)) & FIFO_EN1_SLV3_EN_MASK)
#define FIFO_EN1_SLV2_EN_MASK       0x04
#define FIFO_EN1_SLV2_EN_SHIFT      2u
#define FIFO_EN1_SLV2_EN(x)         (((uint8_t)(((uint8_t)(x)) << FIFO_EN1_SLV2_EN_SHIFT)) & FIFO_EN1_SLV2_EN_MASK)
#define FIFO_EN1_SLV1_EN_MASK       0x02
#define FIFO_EN1_SLV1_EN_SHIFT      1u
#define FIFO_EN1_SLV1_EN(x)         (((uint8_t)(((uint8_t)(x)) << FIFO_EN1_SLV1_EN_SHIFT)) & FIFO_EN1_SLV1_EN_MASK)
#define FIFO_EN1_SLV0_EN_MASK       0x01
#define FIFO_EN1_SLV0_EN_SHIFT      0u
#define FIFO_EN1_SLV0_EN(x)         (((uint8_t)(((uint8_t)(x)) << FIFO_EN1_SLV0_EN_SHIFT)) & FIFO_EN1_SLV0_EN_MASK)
//FIFO_EN2
#define FIFO_EN2_ACCEL_EN_MASK      0x08
#define FIFO_EN2_ACCEL_EN_SHIFT     3u
#define FIFO_EN2_ACCEL_EN(x)        (((uint8_t)(((uint8_t)(x)) << FIFO_EN2_ACCEL_EN_SHIFT)) & FIFO_EN2_ACCEL_EN_MASK)
#define FIFO_EN2_GYRO_Z_EN_MASK     0x04
#define FIFO_EN2_GYRO_Z_EN_SHIFT    2u
#define FIFO_EN2_GYRO_Z_EN(x)       (((uint8_t)(((uint8_t)(x)) << FIFO_EN2_GYRO_Z_EN_SHIFT)) & FIFO_EN2_GYRO_Z_EN_MASK)
#define FIFO_EN2_GYRO_Y_EN_MASK     0x02
#define FIFO_EN2_GYRO_Y_EN_SHIFT    1u
#define FIFO_EN2_GYRO_Y_EN(x)       (((uint8_t)(((uint8_t)(x)) << FIFO_EN2_GYRO_Y_EN_SHIFT)) & FIFO_EN2_GYRO_Y_EN_MASK)
#define FIFO_EN2_GYRO_X_EN_MASK     0x01
#define FIFO_EN2_GYRO_X_EN_SHIFT    0u
#define FIFO_EN2_GYRO_X_EN(x)       (((uint8_t)(((uint8_t)(x)) << FIFO_EN2_GYRO_X_EN_SHIFT)) & FIFO_EN2_GYRO_X_EN_MASK)
//FIFO_RST
#define FIFO_RESET_MASK             0x1F
#define FIFO_RESET_SHIFT            0u
#define FIFO_RESET(x)               (((uint8_t)(((uint8_t)(x)) << FIFO_RESET_SHIFT)) & FIFO_RESET_MASK)
//FIFO_MODE
#define FIFO_MODE_MASK              0x1F
#define FIFO_MODE_SHIFT             0u
#define FIFO_MODE(x)                (((uint8_t)(((uint8_t)(x)) << FIFO_MODE_SHIFT)) & FIFO_MODE_MASK)
//FIFO_CFG
#define FIFO_CFG_MASK               0x01
#define FIFO_CFG_SHIFT              0u
#define FIFO_CFG(x)                 (((uint8_t)(((uint8_t)(x)) << FIFO_CFG_SHIFT)) & FIFO_CFG_MASK)
//REG_BANK_SEL
#define REG_BANK_SEL_UB_MASK        0x30
#define REG_BANK_SEL_UB_SHIFT       4u
#define REG_BANK_SEL_UB(x)          (((uint8_t)(((uint8_t)(x)) << REG_BANK_SEL_UB_SHIFT)) & REG_BANK_SEL_UB_MASK)

// -------BANK 1-------
//XA_OFFS_L
#define XA_OFFS_L_MASK              0xFE
#define XA_OFFS_L_SHIFT             1u
#define XA_OFFS_L(x)                (((uint8_t)(((uint8_t)(x)) << XA_OFFS_L_SHIFT)) & XA_OFFS_L_MASK)
//YA_OFFS_L
#define YA_OFFS_L_MASK              0xFE
#define YA_OFFS_L_SHIFT             1u
#define YA_OFFS_L(x)                (((uint8_t)(((uint8_t)(x)) << YA_OFFS_L_SHIFT)) & YA_OFFS_L_MASK)
//ZA_OFFS_L
#define ZA_OFFS_L_MASK              0xFE
#define ZA_OFFS_L_SHIFT             1u
#define ZA_OFFS_L(x)                (((uint8_t)(((uint8_t)(x)) << ZA_OFFS_L_SHIFT)) & ZA_OFFS_L_MASK)

// -------BANK 2-------
//GYRO_CONFIG_1
#define GYRO_CFG1_DLPFCFG_MASK      0x38
#define GYRO_CFG1_DLPFCFG_SHIFT     3u
#define GYRO_CFG1_DLPFCFG(x)        (((uint8_t)(((uint8_t)(x)) << GYRO_CFG1_DLPFCFG_SHIFT)) & GYRO_CFG1_DLPFCFG_MASK)
#define GYRO_CFG1_FS_SEL_MASK       0x06
#define GYRO_CFG1_FS_SEL_SHIFT      1u
#define GYRO_CFG1_FS_SEL(x)         (((uint8_t)(((uint8_t)(x)) << GYRO_CFG1_FS_SEL_SHIFT)) & GYRO_CFG1_FS_SEL_MASK)
#define GYRO_CFG1_FCHOICE_MASK      0x01
#define GYRO_CFG1_FCHOICE_SHIFT     0u
#define GYRO_CFG1_FCHOICE(x)        (((uint8_t)(((uint8_t)(x)) << GYRO_CFG1_FCHOICE_SHIFT)) & GYRO_CFG1_FCHOICE_MASK)
//GYRO_CONFIG_2
#define GYRO_CFG2_XGYRO_CTEN_MASK   0x20
#define GYRO_CFG2_XGYRO_CTEN_SHIFT  5u
#define GYRO_CFG2_XGYRO_CTEN(x)     (((uint8_t)(((uint8_t)(x)) << GYRO_CFG2_XGYRO_CTEN_SHIFT)) & GYRO_CFG2_XGYRO_CTEN_MASK)
#define GYRO_CFG2_YGYRO_CTEN_MASK   0x10
#define GYRO_CFG2_YGYRO_CTEN_SHIFT  4u
#define GYRO_CFG2_YGYRO_CTEN(x)     (((uint8_t)(((uint8_t)(x)) << GYRO_CFG2_YGYRO_CTEN_SHIFT)) & GYRO_CFG2_YGYRO_CTEN_MASK)
#define GYRO_CFG2_ZGYRO_CTEN_MASK   0x08
#define GYRO_CFG2_ZGYRO_CTEN_SHIFT  3u
#define GYRO_CFG2_ZGYRO_CTEN(x)     (((uint8_t)(((uint8_t)(x)) << GYRO_CFG2_ZGYRO_CTEN_SHIFT)) & GYRO_CFG2_ZGYRO_CTEN_MASK)
#define GYRO_CFG2_AVGCFG_MASK       0x07
#define GYRO_CFG2_AVGCFG_SHIFT      0u
#define GYRO_CFG2_AVGCFG(x)         (((uint8_t)(((uint8_t)(x)) << GYRO_CFG2_AVGCFG_SHIFT)) & GYRO_CFG2_AVGCFG_MASK)
//ODR_ALIGN_EN
#define ODR_ALIGN_EN_MASK           0x01
#define ODR_ALIGN_EN_SHIFT          0u
#define ODR_ALIGN_EN(x)             (((uint8_t)(((uint8_t)(x)) << ODR_ALIGN_EN_SHIFT)) & ODR_ALIGN_EN_MASK)
//ACCEL_SMPLRT_DIV_1
#define ACCEL_SMPLRT_DIV1_MASK      0x0F
#define ACCEL_SMPLRT_DIV1_SHIFT     0u
#define ACCEL_SMPLRT_DIV1(x)        (((uint8_t)(((uint8_t)(x)) << ACCEL_SMPLRT_DIV1_SHIFT)) & ACCEL_SMPLRT_DIV1_MASK)
//ACCEL_INTEL_CTRL
#define ACCEL_INTEL_EN_MASK         0x02
#define ACCEL_INTEL_EN_SHIFT        1u
#define ACCEL_INTEL_EN(x)           (((uint8_t)(((uint8_t)(x)) << ACCEL_INTEL_EN_SHIFT)) & ACCEL_INTEL_EN_MASK)
#define ACCEL_INTEL_MODE_MASK       0x01
#define ACCEL_INTEL_MODE_SHIFT      0u
#define ACCEL_INTEL_MODE(x)         (((uint8_t)(((uint8_t)(x)) << ACCEL_INTEL_MODE_SHIFT)) & ACCEL_INTEL_MODE_MASK)
//ACCEL_CONFIG
#define ACCEL_CFG_DLPFCFG_MASK      0x38
#define ACCEL_CFG_DLPFCFG_SHIFT     3u
#define ACCEL_CFG_DLPFCFG(x)        (((uint8_t)(((uint8_t)(x)) << ACCEL_CFG_DLPFCFG_SHIFT)) & ACCEL_CFG_DLPFCFG_MASK)
#define ACCEL_CFG_FS_SEL_MASK       0x06
#define ACCEL_CFG_FS_SEL_SHIFT      1u
#define ACCEL_CFG_FS_SEL(x)         (((uint8_t)(((uint8_t)(x)) << ACCEL_CFG_FS_SEL_SHIFT)) & ACCEL_CFG_FS_SEL_MASK)
#define ACCEL_CFG_FCHOICE_MASK      0x01
#define ACCEL_CFG_FCHOICE_SHIFT     0u
#define ACCEL_CFG_FCHOICE(x)        (((uint8_t)(((uint8_t)(x)) << ACCEL_CFG_FCHOICE_SHIFT)) & ACCEL_CFG_FCHOICE_MASK)
//ACCEL_CONFIG_2
#define ACCEL_CFG2_AX_ST_EN_MASK    0x10
#define ACCEL_CFG2_AX_ST_EN_SHIFT   4u
#define ACCEL_CFG2_AX_ST_EN(x)      (((uint8_t)(((uint8_t)(x)) << ACCEL_CFG2_AX_ST_EN_SHIFT)) & ACCEL_CFG2_AX_ST_EN_MASK)
#define ACCEL_CFG2_AY_ST_EN_MASK    0x08
#define ACCEL_CFG2_AY_ST_EN_SHIFT   3u
#define ACCEL_CFG2_AY_ST_EN(x)      (((uint8_t)(((uint8_t)(x)) << ACCEL_CFG2_AY_ST_EN_SHIFT)) & ACCEL_CFG2_AY_ST_EN_MASK)
#define ACCEL_CFG2_AZ_ST_EN_MASK    0x04
#define ACCEL_CFG2_AZ_ST_EN_SHIFT   2u
#define ACCEL_CFG2_AZ_ST_EN(x)      (((uint8_t)(((uint8_t)(x)) << ACCEL_CFG2_AZ_ST_EN_SHIFT)) & ACCEL_CFG2_AZ_ST_EN_MASK)
#define ACCEL_CFG2_DEC3_CFG_MASK    0x03
#define ACCEL_CFG2_DEC3_CFG_SHIFT   0u
#define ACCEL_CFG2_DEC3_CFG(x)      (((uint8_t)(((uint8_t)(x)) << ACCEL_CFG2_DEC3_CFG_SHIFT)) & ACCEL_CFG2_DEC3_CFG_MASK)
//FSYNC_CONFIG
#define FSYNC_CFG_DELAY_EN_MASK     0x80
#define FSYNC_CFG_DELAY_EN_SHIFT    7u
#define FSYNC_CFG_DELAY_EN(x)       (((uint8_t)(((uint8_t)(x)) << FSYNC_CFG_DELAY_EN_SHIFT)) & FSYNC_CFG_DELAY_EN_MASK)
#define FSYNC_CFG_WOF_DG_EN_MASK    0x20
#define FSYNC_CFG_WOF_DG_EN_SHIFT   5u
#define FSYNC_CFG_WOF_DG_EN(x)      (((uint8_t)(((uint8_t)(x)) << FSYNC_CFG_WOF_DG_EN_SHIFT)) & FSYNC_CFG_WOF_DG_EN_MASK)
#define FSYNC_CFG_WOF_EDGE_MASK     0x10
#define FSYNC_CFG_WOF_EDGE_SHIFT    4u
#define FSYNC_CFG_WOF_EDGE(x)       (((uint8_t)(((uint8_t)(x)) << FSYNC_CFG_WOF_EDGE_SHIFT)) & FSYNC_CFG_WOF_EDGE_MASK)
#define FSYNC_CFG_EXT_SYNC_MASK     0x0F
#define FSYNC_CFG_EXT_SYNC_SHIFT    0u
#define FSYNC_CFG_EXT_SYNC(x)       (((uint8_t)(((uint8_t)(x)) << FSYNC_CFG_EXT_SYNC_SHIFT)) & FSYNC_CFG_EXT_SYNC_MASK)
//TEMP_CONFIG
#define TEMP_CFG_DLPFCFG_MASK       0x07
#define TEMP_CFG_DLPFCFG_SHIFT      0u
#define TEMP_CFG_DLPFCFG(x)         (((uint8_t)(((uint8_t)(x)) << TEMP_CFG_DLPFCFG_SHIFT)) & TEMP_CFG_DLPFCFG_MASK)
//MOD_CTRL_USR
#define MOD_CTRL_LP_DMP_EN_MASK     0x01
#define MOD_CTRL_LP_DMP_EN_SHIFT    0u
#define MOD_CTRL_LP_DMP_EN(x)       (((uint8_t)(((uint8_t)(x)) << MOD_CTRL_LP_DMP_EN_SHIFT)) & MOD_CTRL_LP_DMP_EN_MASK)

// -------BANK 3-------
//I2C_MST_ODR_CONFIG
#define I2C_MST_ODR_CFG_MASK        0x0F
#define I2C_MST_ODR_CFG_SHIFT       0u
#define I2C_MST_ODR_CFG(x)          (((uint8_t)(((uint8_t)(x)) << I2C_MST_ODR_CFG_SHIFT)) & I2C_MST_ODR_CFG_MASK)
//I2C_MST_CTRL
#define I2C_CTRL_MULT_MST_EN_MASK   0x80
#define I2C_CTRL_MULT_MST_EN_SHIFT  7u
#define I2C_CTRL_MULT_MST_EN(x)     (((uint8_t)(((uint8_t)(x)) << I2C_CTRL_MULT_MST_EN_SHIFT)) & I2C_CTRL_MULT_MST_EN_MASK)
#define I2C_CTRL_MST_P_NSR_MASK     0x10
#define I2C_CTRL_MST_P_NSR_SHIFT    4u
#define I2C_CTRL_MST_P_NSR(x)       (((uint8_t)(((uint8_t)(x)) << I2C_CTRL_MST_P_NSR_SHIFT)) & I2C_CTRL_MST_P_NSR_MASK)
#define I2C_CTRL_MST_CLK_MASK       0x0F
#define I2C_CTRL_MST_CLK_SHIFT      0u
#define I2C_CTRL_MST_CLK(x)         (((uint8_t)(((uint8_t)(x)) << I2C_CTRL_MST_CLK_SHIFT)) & I2C_CTRL_MST_CLK_MASK)
//I2C_MST_DELAY_CTRL
#define I2C_DELAY_ES_SHDW_MASK      0x80
#define I2C_DELAY_ES_SHDW_SHIFT     7u
#define I2C_DELAY_ES_SHDW(x)        (((uint8_t)(((uint8_t)(x)) << I2C_DELAY_ES_SHDW_SHIFT)) & I2C_DELAY_ES_SHDW_MASK)
#define I2C_DELAY_SLV4_EN_MASK      0x10
#define I2C_DELAY_SLV4_EN_SHIFT     4u
#define I2C_DELAY_SLV4_EN(x)        (((uint8_t)(((uint8_t)(x)) << I2C_DELAY_SLV4_EN_SHIFT)) & I2C_DELAY_SLV4_EN_MASK)
#define I2C_DELAY_SLV3_EN_MASK      0x08
#define I2C_DELAY_SLV3_EN_SHIFT     3u
#define I2C_DELAY_SLV3_EN(x)        (((uint8_t)(((uint8_t)(x)) << I2C_DELAY_SLV3_EN_SHIFT)) & I2C_DELAY_SLV3_EN_MASK)
#define I2C_DELAY_SLV2_EN_MASK      0x04
#define I2C_DELAY_SLV2_EN_SHIFT     2u
#define I2C_DELAY_SLV2_EN(x)        (((uint8_t)(((uint8_t)(x)) << I2C_DELAY_SLV2_EN_SHIFT)) & I2C_DELAY_SLV2_EN_MASK)
#define I2C_DELAY_SLV1_EN_MASK      0x02
#define I2C_DELAY_SLV1_EN_SHIFT     1u
#define I2C_DELAY_SLV1_EN(x)        (((uint8_t)(((uint8_t)(x)) << I2C_DELAY_SLV1_EN_SHIFT)) & I2C_DELAY_SLV1_EN_MASK)
#define I2C_DELAY_SLV0_EN_MASK      0x01
#define I2C_DELAY_SLV0_EN_SHIFT     0u
#define I2C_DELAY_SLV0_EN(x)        (((uint8_t)(((uint8_t)(x)) << I2C_DELAY_SLV0_EN_SHIFT)) & I2C_DELAY_SLV0_EN_MASK)
//I2C_SLV0_ADDR
#define I2C_SLV0_ADDR_RNW_MASK      0x80
#define I2C_SLV0_ADDR_RNW_SHIFT     7u
#define I2C_SLV0_ADDR_RNW(x)        (((uint8_t)(((uint8_t)(x)) << I2C_SLV0_ADDR_RNW_SHIFT)) & I2C_SLV0_ADDR_RNW_MASK)
#define I2C_SLV0_ADDR_ID0_MASK      0x7F
#define I2C_SLV0_ADDR_ID0_SHIFT     0u
#define I2C_SLV0_ADDR_ID0(x)        (((uint8_t)(((uint8_t)(x)) << I2C_SLV0_ADDR_ID0_SHIFT)) & I2C_SLV0_ADDR_ID0_MASK)
//I2C_SLV0_CTRL
#define I2C_SLV0_CTRL_EN_MASK       0x80
#define I2C_SLV0_CTRL_EN_SHIFT      7u
#define I2C_SLV0_CTRL_EN(x)         (((uint8_t)(((uint8_t)(x)) << I2C_SLV0_CTRL_EN_SHIFT)) & I2C_SLV0_CTRL_EN_MASK)
#define I2C_SLV0_CTRL_BYTE_SW_MASK  0x40
#define I2C_SLV0_CTRL_BYTE_SW_SHIFT 6u
#define I2C_SLV0_CTRL_BYTE_SW(x)    (((uint8_t)(((uint8_t)(x)) << I2C_SLV0_CTRL_BYTE_SW_SHIFT)) & I2C_SLV0_CTRL_BYTE_SW_MASK)
#define I2C_SLV0_CTRL_REG_DIS_MASK  0x20
#define I2C_SLV0_CTRL_REG_DIS_SHIFT 5u
#define I2C_SLV0_CTRL_REG_DIS(x)    (((uint8_t)(((uint8_t)(x)) << I2C_SLV0_CTRL_REG_DIS_SHIFT)) & I2C_SLV0_CTRL_REG_DIS_MASK)
#define I2C_SLV0_CTRL_GRP_MASK      0x10
#define I2C_SLV0_CTRL_GRP_SHIFT     4u
#define I2C_SLV0_CTRL_GRP(x)        (((uint8_t)(((uint8_t)(x)) << I2C_SLV0_CTRL_GRP_SHIFT)) & I2C_SLV0_CTRL_GRP_MASK)
#define I2C_SLV0_CTRL_LENG_MASK     0x0F
#define I2C_SLV0_CTRL_LENG_SHIFT    0u
#define I2C_SLV0_CTRL_LENG(x)       (((uint8_t)(((uint8_t)(x)) << I2C_SLV0_CTRL_LENG_SHIFT)) & I2C_SLV0_CTRL_LENG_MASK)
//I2C_SLV1_ADDR
#define I2C_SLV1_ADDR_RNW_MASK      0x80
#define I2C_SLV1_ADDR_RNW_SHIFT     7u
#define I2C_SLV1_ADDR_RNW(x)        (((uint8_t)(((uint8_t)(x)) << I2C_SLV1_ADDR_RNW_SHIFT)) & I2C_SLV1_ADDR_RNW_MASK)
#define I2C_SLV1_ADDR_ID1_MASK      0x7F
#define I2C_SLV1_ADDR_ID1_SHIFT     0u
#define I2C_SLV1_ADDR_ID1(x)        (((uint8_t)(((uint8_t)(x)) << I2C_SLV1_ADDR_ID1_SHIFT)) & I2C_SLV1_ADDR_ID1_MASK)
//I2C_SLV1_CTRL
#define I2C_SLV1_CTRL_EN_MASK       0x80
#define I2C_SLV1_CTRL_EN_SHIFT      7u
#define I2C_SLV1_CTRL_EN(x)         (((uint8_t)(((uint8_t)(x)) << I2C_SLV1_CTRL_EN_SHIFT)) & I2C_SLV1_CTRL_EN_MASK)
#define I2C_SLV1_CTRL_BYTE_SW_MASK  0x40
#define I2C_SLV1_CTRL_BYTE_SW_SHIFT 6u
#define I2C_SLV1_CTRL_BYTE_SW(x)    (((uint8_t)(((uint8_t)(x)) << I2C_SLV1_CTRL_BYTE_SW_SHIFT)) & I2C_SLV1_CTRL_BYTE_SW_MASK)
#define I2C_SLV1_CTRL_REG_DIS_MASK  0x20
#define I2C_SLV1_CTRL_REG_DIS_SHIFT 5u
#define I2C_SLV1_CTRL_REG_DIS(x)    (((uint8_t)(((uint8_t)(x)) << I2C_SLV1_CTRL_REG_DIS_SHIFT)) & I2C_SLV1_CTRL_REG_DIS_MASK)
#define I2C_SLV1_CTRL_GRP_MASK      0x10
#define I2C_SLV1_CTRL_GRP_SHIFT     4u
#define I2C_SLV1_CTRL_GRP(x)        (((uint8_t)(((uint8_t)(x)) << I2C_SLV1_CTRL_GRP_SHIFT)) & I2C_SLV1_CTRL_GRP_MASK)
#define I2C_SLV1_CTRL_LENG_MASK     0x0F
#define I2C_SLV1_CTRL_LENG_SHIFT    0u
#define I2C_SLV1_CTRL_LENG(x)       (((uint8_t)(((uint8_t)(x)) << I2C_SLV1_CTRL_LENG_SHIFT)) & I2C_SLV1_CTRL_LENG_MASK)
//I2C_SLV2_ADDR
#define I2C_SLV2_ADDR_RNW_MASK      0x80
#define I2C_SLV2_ADDR_RNW_SHIFT     7u
#define I2C_SLV2_ADDR_RNW(x)        (((uint8_t)(((uint8_t)(x)) << I2C_SLV2_ADDR_RNW_SHIFT)) & I2C_SLV2_ADDR_RNW_MASK)
#define I2C_SLV2_ADDR_ID0_MASK      0x7F
#define I2C_SLV2_ADDR_ID0_SHIFT     0u
#define I2C_SLV2_ADDR_ID0(x)        (((uint8_t)(((uint8_t)(x)) << I2C_SLV2_ADDR_ID0_SHIFT)) & I2C_SLV2_ADDR_ID0_MASK)
//I2C_SLV2_CTRL
#define I2C_SLV2_CTRL_EN_MASK       0x80
#define I2C_SLV2_CTRL_EN_SHIFT      7u
#define I2C_SLV2_CTRL_EN(x)         (((uint8_t)(((uint8_t)(x)) << I2C_SLV2_CTRL_EN_SHIFT)) & I2C_SLV2_CTRL_EN_MASK)
#define I2C_SLV2_CTRL_BYTE_SW_MASK  0x40
#define I2C_SLV2_CTRL_BYTE_SW_SHIFT 6u
#define I2C_SLV2_CTRL_BYTE_SW(x)    (((uint8_t)(((uint8_t)(x)) << I2C_SLV2_CTRL_BYTE_SW_SHIFT)) & I2C_SLV2_CTRL_BYTE_SW_MASK)
#define I2C_SLV2_CTRL_REG_DIS_MASK  0x20
#define I2C_SLV2_CTRL_REG_DIS_SHIFT 5u
#define I2C_SLV2_CTRL_REG_DIS(x)    (((uint8_t)(((uint8_t)(x)) << I2C_SLV2_CTRL_REG_DIS_SHIFT)) & I2C_SLV2_CTRL_REG_DIS_MASK)
#define I2C_SLV2_CTRL_GRP_MASK      0x10
#define I2C_SLV2_CTRL_GRP_SHIFT     4u
#define I2C_SLV2_CTRL_GRP(x)        (((uint8_t)(((uint8_t)(x)) << I2C_SLV2_CTRL_GRP_SHIFT)) & I2C_SLV2_CTRL_GRP_MASK)
#define I2C_SLV2_CTRL_LENG_MASK     0x0F
#define I2C_SLV2_CTRL_LENG_SHIFT    0u
#define I2C_SLV2_CTRL_LENG(x)       (((uint8_t)(((uint8_t)(x)) << I2C_SLV2_CTRL_LENG_SHIFT)) & I2C_SLV2_CTRL_LENG_MASK)
//I2C_SLV3_ADDR
#define I2C_SLV3_ADDR_RNW_MASK      0x80
#define I2C_SLV3_ADDR_RNW_SHIFT     7u
#define I2C_SLV3_ADDR_RNW(x)        (((uint8_t)(((uint8_t)(x)) << I2C_SLV3_ADDR_RNW_SHIFT)) & I2C_SLV3_ADDR_RNW_MASK)
#define I2C_SLV3_ADDR_ID0_MASK      0x7F
#define I2C_SLV3_ADDR_ID0_SHIFT     0u
#define I2C_SLV3_ADDR_ID0(x)        (((uint8_t)(((uint8_t)(x)) << I2C_SLV3_ADDR_ID0_SHIFT)) & I2C_SLV3_ADDR_ID0_MASK)
//I2C_SLV3_CTRL
#define I2C_SLV3_CTRL_EN_MASK       0x80
#define I2C_SLV3_CTRL_EN_SHIFT      7u
#define I2C_SLV3_CTRL_EN(x)         (((uint8_t)(((uint8_t)(x)) << I2C_SLV3_CTRL_EN_SHIFT)) & I2C_SLV3_CTRL_EN_MASK)
#define I2C_SLV3_CTRL_BYTE_SW_MASK  0x40
#define I2C_SLV3_CTRL_BYTE_SW_SHIFT 6u
#define I2C_SLV3_CTRL_BYTE_SW(x)    (((uint8_t)(((uint8_t)(x)) << I2C_SLV3_CTRL_BYTE_SW_SHIFT)) & I2C_SLV3_CTRL_BYTE_SW_MASK)
#define I2C_SLV3_CTRL_REG_DIS_MASK  0x20
#define I2C_SLV3_CTRL_REG_DIS_SHIFT 5u
#define I2C_SLV3_CTRL_REG_DIS(x)    (((uint8_t)(((uint8_t)(x)) << I2C_SLV3_CTRL_REG_DIS_SHIFT)) & I2C_SLV3_CTRL_REG_DIS_MASK)
#define I2C_SLV3_CTRL_GRP_MASK      0x10
#define I2C_SLV3_CTRL_GRP_SHIFT     4u
#define I2C_SLV3_CTRL_GRP(x)        (((uint8_t)(((uint8_t)(x)) << I2C_SLV3_CTRL_GRP_SHIFT)) & I2C_SLV3_CTRL_GRP_MASK)
#define I2C_SLV3_CTRL_LENG_MASK     0x0F
#define I2C_SLV3_CTRL_LENG_SHIFT    0u
#define I2C_SLV3_CTRL_LENG(x)       (((uint8_t)(((uint8_t)(x)) << I2C_SLV3_CTRL_LENG_SHIFT)) & I2C_SLV3_CTRL_LENG_MASK)
//I2C_SLV4_ADDR
#define I2C_SLV4_ADDR_RNW_MASK      0x80
#define I2C_SLV4_ADDR_RNW_SHIFT     7u
#define I2C_SLV4_ADDR_RNW(x)        (((uint8_t)(((uint8_t)(x)) << I2C_SLV4_ADDR_RNW_SHIFT)) & I2C_SLV4_ADDR_RNW_MASK)
#define I2C_SLV4_ADDR_ID0_MASK      0x7F
#define I2C_SLV4_ADDR_ID0_SHIFT     0u
#define I2C_SLV4_ADDR_ID0(x)        (((uint8_t)(((uint8_t)(x)) << I2C_SLV4_ADDR_ID0_SHIFT)) & I2C_SLV4_ADDR_ID0_MASK)
//I2C_SLV4_CTRL
#define I2C_SLV4_CTRL_EN_MASK       0x80
#define I2C_SLV4_CTRL_EN_SHIFT      7u
#define I2C_SLV4_CTRL_EN(x)         (((uint8_t)(((uint8_t)(x)) << I2C_SLV4_CTRL_EN_SHIFT)) & I2C_SLV4_CTRL_EN_MASK)
#define I2C_SLV4_CTRL_BYTE_SW_MASK  0x40
#define I2C_SLV4_CTRL_BYTE_SW_SHIFT 6u
#define I2C_SLV4_CTRL_BYTE_SW(x)    (((uint8_t)(((uint8_t)(x)) << I2C_SLV4_CTRL_BYTE_SW_SHIFT)) & I2C_SLV4_CTRL_BYTE_SW_MASK)
#define I2C_SLV4_CTRL_REG_DIS_MASK  0x20
#define I2C_SLV4_CTRL_REG_DIS_SHIFT 5u
#define I2C_SLV4_CTRL_REG_DIS(x)    (((uint8_t)(((uint8_t)(x)) << I2C_SLV4_CTRL_REG_DIS_SHIFT)) & I2C_SLV4_CTRL_REG_DIS_MASK)
#define I2C_SLV4_CTRL_GRP_MASK      0x10
#define I2C_SLV4_CTRL_GRP_SHIFT     4u
#define I2C_SLV4_CTRL_GRP(x)        (((uint8_t)(((uint8_t)(x)) << I2C_SLV4_CTRL_GRP_SHIFT)) & I2C_SLV4_CTRL_GRP_MASK)
#define I2C_SLV4_CTRL_LENG_MASK     0x0F
#define I2C_SLV4_CTRL_LENG_SHIFT    0u
#define I2C_SLV4_CTRL_LENG(x)       (((uint8_t)(((uint8_t)(x)) << I2C_SLV4_CTRL_LENG_SHIFT)) & I2C_SLV4_CTRL_LENG_MASK)

// -------Magnetometer bit fields-------
//CNTL2
#define MAGNETO_CNTL2_MPDE4_MASK    0x10
#define MAGNETO_CNTL2_MPDE4_SHIFT   4u
#define MAGNETO_CNTL2_MPDE4(x)      (((uint8_t)(((uint8_t)(x)) << MAGNETO_CNTL2_MPDE4_SHIFT)) & MAGNETO_CNTL2_MPDE4_MASK)
#define MAGNETO_CNTL2_MODE3_MASK    0x08
#define MAGNETO_CNTL2_MODE3_SHIFT   3u
#define MAGNETO_CNTL2_MODE3(x)      (((uint8_t)(((uint8_t)(x)) << MAGNETO_CNTL2_MODE3_SHIFT)) & MAGNETO_CNTL2_MODE3_MASK)
#define MAGNETO_CNTL2_MODE2_MASK    0x04
#define MAGNETO_CNTL2_MODE2_SHIFT   2u
#define MAGNETO_CNTL2_MODE2(x)      (((uint8_t)(((uint8_t)(x)) << MAGNETO_CNTL2_MODE2_SHIFT)) & MAGNETO_CNTL2_MODE2_MASK)
#define MAGNETO_CNTL2_MODE1_MASK    0x02
#define MAGNETO_CNTL2_MODE1_SHIFT   1u
#define MAGNETO_CNTL2_MODE1(x)      (((uint8_t)(((uint8_t)(x)) << MAGNETO_CNTL2_MODE1_SHIFT)) & MAGNETO_CNTL2_MODE1_MASK)
#define MAGNETO_CNTL2_MODE0_MASK    0x01
#define MAGNETO_CNTL2_MODE0_SHIFT   0u
#define MAGNETO_CNTL2_MODE0(x)      (((uint8_t)(((uint8_t)(x)) << MAGNETO_CNTL2_MODE0_SHIFT)) & MAGNETO_CNTL2_MODE0_MASK)
//CNTL3
#define MAGNETO_CNTL3_SRST_MASK     0x01
#define MAGNETO_CNTL3_SRST_SHIFT    0u
#define MAGNETO_CNTL3_SRST(x)       (((uint8_t)(((uint8_t)(x)) << MAGNETO_CNTL3_SRST_SHIFT)) & MAGNETO_CNTL3_SRST_MASK)