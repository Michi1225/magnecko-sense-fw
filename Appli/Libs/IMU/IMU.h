#ifndef __IMU_H_
#define __IMU_H_

#include "main.h"
#include "lsm6dsv16x_reg.h"
#include "platform.h"
#include "imu_config.h"
#include "stm32n6xx_hal_def.h"
#include <string.h>

#define IMU_SPI_HANDLE (&hspi4)

#define BOOT_TIME      (10)
#define FIFO_WATERMARK (32)

#define IMU_INIT_TIMEOUT_MS (1000)
#define IMU_DATA_ACC  (1U << 0)
#define IMU_DATA_GYRO (1U << 1)
#define IMU_DATA_SFLP (1U << 2)
#define IMU_DATA_ALL  (IMU_DATA_ACC | IMU_DATA_GYRO | IMU_DATA_SFLP)

typedef struct 
{
    uint8_t whoamI;
    uint8_t tx_buf[128];
    stmdev_ctx_t dev_ctx;
    lsm6dsv16x_fifo_sflp_raw_t fifo_sflp;
    lsm6dsv16x_fifo_status_t fifo_status;
    lsm6dsv16x_fifo_out_raw_t f_data;

    float_t quat[4];
    uint64_t timestamp_quat;
    float_t acc_mg[3];
    uint64_t timestamp_acc;
    float_t gyro_mdps[3];
    uint64_t timestamp_gyro;
    /* Timestamp tag and sensor entries may straddle FIFO drain calls. */
    uint32_t fifo_timestamp, raw_acc, raw_gyro, raw_quat;
    uint8_t fifo_count, fifo_timestamp_valid, timestamp_pending, fresh_timestamps;
    uint8_t valid_data;
    imu_config_t config;
} IMU;

/** @brief Initialize the IMU transport, sampling, FIFO, and SFLP game rotation output. */
HAL_StatusTypeDef imu_init(IMU *imu);
/** @brief Change sampling/ranges and restart FIFO/SFLP without mixing sample scales. */
HAL_StatusTypeDef imu_configure(IMU *imu, const imu_config_t *config);
/** @brief Drain available FIFO entries into the latest acceleration, gyro, and quaternion samples. */
HAL_StatusTypeDef imu_get_data_blocking(IMU *imu);
/** @brief Flush the IMU FIFO and clear sample validity before resuming streaming. */
HAL_StatusTypeDef imu_reset_fifo(IMU *imu);


#endif //__IMU_H_
