#include "IMU.h"
#include "lsm6dsv16x_reg.h"
#include "platform.h"
#include "sensor_clock.h"
#include "stm32n6xx_hal_def.h"
#include <math.h>

/** @brief Convert an IMU half-precision sample into a single-precision value. */
static float_t npy_half_to_float(uint16_t h)
{
    union { float_t ret; uint32_t retbits; } conv;
    conv.retbits = lsm6dsv16x_from_f16_to_f32(h);
    return conv.ret;
}

/** @brief Expand the SFLP rotation vector into a normalized xyzw quaternion. */
static void sflp2q(float_t quat[4], uint16_t sflp[3])
{
  float_t sumsq = 0;

  quat[0] = npy_half_to_float(sflp[0]);
  quat[1] = npy_half_to_float(sflp[1]);
  quat[2] = npy_half_to_float(sflp[2]);

  for (uint8_t i = 0; i < 3; i++)
    sumsq += quat[i] * quat[i];

  if (sumsq > 1.0f) {
    float_t n = sqrtf(sumsq);
    quat[0] /= n;
    quat[1] /= n;
    quat[2] /= n;
    sumsq = 1.0f;
  }

  quat[3] = sqrtf(1.0f - sumsq);
}

#define IMU_INIT_CALL(call) do { if ((call) != 0) return HAL_ERROR; } while (0)

/** @brief Reinitialize the fusion algorithm and restore the normal register bank on failure. */
static int32_t reset_fusion(stmdev_ctx_t *ctx)
{
    lsm6dsv16x_emb_func_init_a_t init = {.sflp_game_init = 1};
    int32_t result = lsm6dsv16x_mem_bank_set(ctx, LSM6DSV16X_EMBED_FUNC_MEM_BANK);
    if (!result) result = lsm6dsv16x_write_reg(ctx, LSM6DSV16X_EMB_FUNC_INIT_A, (uint8_t *)&init, 1);
    int32_t restore = lsm6dsv16x_mem_bank_set(ctx, LSM6DSV16X_MAIN_MEM_BANK);
    return result ? result : restore;
}

/** @brief Initialize the IMU transport, sampling, FIFO, and SFLP game rotation output. */
HAL_StatusTypeDef imu_init(IMU *imu)
{
    memset(imu, 0, sizeof(*imu));

    stmdev_ctx_t *dev_ctx = &(imu->dev_ctx);
    dev_ctx->write_reg = platform_write;
    dev_ctx->read_reg  = platform_read;
    dev_ctx->mdelay    = platform_delay;
    dev_ctx->handle    = IMU_SPI_HANDLE;

    // Init platform
    platform_init(dev_ctx->handle);

    platform_delay(BOOT_TIME);

    /* A failed embedded-function transaction can leave the alternate bank
     * selected. Reset must recover that state before reading WHO_AM_I. */
    IMU_INIT_CALL(lsm6dsv16x_mem_bank_set(dev_ctx, LSM6DSV16X_MAIN_MEM_BANK));

    uint16_t timeout = 0;
    do {
        if (lsm6dsv16x_device_id_get(dev_ctx, &(imu->whoamI)) != 0)
            return HAL_ERROR;
        platform_delay(1);
        ++timeout;
        if(timeout > IMU_INIT_TIMEOUT_MS) return HAL_TIMEOUT;
    }while (imu->whoamI != LSM6DSV16X_ID);

    // Reset Device
    IMU_INIT_CALL(lsm6dsv16x_sw_por(dev_ctx));

    // Output registers are not updated until LSB and MSB have been read
    IMU_INIT_CALL(lsm6dsv16x_block_data_update_set(dev_ctx, PROPERTY_ENABLE));

    IMU_INIT_CALL(lsm6dsv16x_fifo_watermark_set(dev_ctx, FIFO_WATERMARK));
    IMU_INIT_CALL(lsm6dsv16x_fifo_compress_algo_set(dev_ctx, LSM6DSV16X_CMP_DISABLE));

    imu->fifo_sflp.game_rotation = 1;
    imu->fifo_sflp.gbias = 0;
    imu->fifo_sflp.gravity = 0;
    IMU_INIT_CALL(lsm6dsv16x_fifo_sflp_batch_set(dev_ctx, imu->fifo_sflp));

    const imu_config_t config = IMU_CONFIG_DEFAULT;
    return imu_configure(imu, &config);
}

/** @brief Restart sampling with a validated profile, committing conversion scales only on success. */
HAL_StatusTypeDef imu_configure(IMU *imu, const imu_config_t *config)
{
    if (!imu_config_valid(config)) return HAL_ERROR;
    stmdev_ctx_t *dev_ctx = &imu->dev_ctx;
    const lsm6dsv16x_data_rate_t rates[] = {LSM6DSV16X_ODR_AT_30Hz, LSM6DSV16X_ODR_AT_60Hz,
        LSM6DSV16X_ODR_AT_120Hz, LSM6DSV16X_ODR_AT_240Hz, LSM6DSV16X_ODR_AT_480Hz};
    const lsm6dsv16x_fifo_xl_batch_t xl_rates[] = {LSM6DSV16X_XL_BATCHED_AT_30Hz, LSM6DSV16X_XL_BATCHED_AT_60Hz,
        LSM6DSV16X_XL_BATCHED_AT_120Hz, LSM6DSV16X_XL_BATCHED_AT_240Hz, LSM6DSV16X_XL_BATCHED_AT_480Hz};
    const lsm6dsv16x_fifo_gy_batch_t gy_rates[] = {LSM6DSV16X_GY_BATCHED_AT_30Hz, LSM6DSV16X_GY_BATCHED_AT_60Hz,
        LSM6DSV16X_GY_BATCHED_AT_120Hz, LSM6DSV16X_GY_BATCHED_AT_240Hz, LSM6DSV16X_GY_BATCHED_AT_480Hz};
    const lsm6dsv16x_sflp_data_rate_t sflp_rates[] = {LSM6DSV16X_SFLP_30Hz, LSM6DSV16X_SFLP_60Hz,
        LSM6DSV16X_SFLP_120Hz, LSM6DSV16X_SFLP_240Hz, LSM6DSV16X_SFLP_480Hz};
    unsigned rate = 0;
    for (uint32_t hz = 30; hz != config->sample_hz; hz *= 2) ++rate;
    lsm6dsv16x_xl_full_scale_t acc = config->acc_g == 2 ? LSM6DSV16X_2g :
        config->acc_g == 4 ? LSM6DSV16X_4g : config->acc_g == 8 ? LSM6DSV16X_8g : LSM6DSV16X_16g;
    lsm6dsv16x_gy_full_scale_t gyro = config->gyro_dps == 250 ? LSM6DSV16X_250dps :
        config->gyro_dps == 500 ? LSM6DSV16X_500dps : config->gyro_dps == 1000 ? LSM6DSV16X_1000dps : LSM6DSV16X_2000dps;
    /* Flush old-scale FIFO entries before changing conversion factors. Sampling
     * and SFLP restart together; the owner withholds packets until all are valid. */
    imu->valid_data = 0;
    imu->fifo_count = 0xff;
    imu->fifo_timestamp_valid = imu->timestamp_pending = 0;
    IMU_INIT_CALL(lsm6dsv16x_sflp_game_rotation_set(dev_ctx, PROPERTY_DISABLE));
    IMU_INIT_CALL(lsm6dsv16x_xl_data_rate_set(dev_ctx, LSM6DSV16X_ODR_OFF));
    IMU_INIT_CALL(lsm6dsv16x_gy_data_rate_set(dev_ctx, LSM6DSV16X_ODR_OFF));
    IMU_INIT_CALL(lsm6dsv16x_fifo_mode_set(dev_ctx, LSM6DSV16X_BYPASS_MODE));
    IMU_INIT_CALL(lsm6dsv16x_timestamp_set(dev_ctx, PROPERTY_ENABLE));
    IMU_INIT_CALL(lsm6dsv16x_fifo_timestamp_batch_set(dev_ctx, LSM6DSV16X_TMSTMP_DEC_1));
    IMU_INIT_CALL(lsm6dsv16x_xl_full_scale_set(dev_ctx, acc));
    IMU_INIT_CALL(lsm6dsv16x_gy_full_scale_set(dev_ctx, gyro));
    IMU_INIT_CALL(lsm6dsv16x_fifo_xl_batch_set(dev_ctx, xl_rates[rate]));
    IMU_INIT_CALL(lsm6dsv16x_fifo_gy_batch_set(dev_ctx, gy_rates[rate]));
    IMU_INIT_CALL(lsm6dsv16x_sflp_data_rate_set(dev_ctx, sflp_rates[rate]));
    IMU_INIT_CALL(lsm6dsv16x_fifo_mode_set(dev_ctx, LSM6DSV16X_STREAM_MODE));
    IMU_INIT_CALL(lsm6dsv16x_xl_data_rate_set(dev_ctx, rates[rate]));
    IMU_INIT_CALL(lsm6dsv16x_gy_data_rate_set(dev_ctx, rates[rate]));
    IMU_INIT_CALL(lsm6dsv16x_sflp_game_rotation_set(dev_ctx, PROPERTY_ENABLE));
    IMU_INIT_CALL(reset_fusion(dev_ctx));
    imu->config = *config;
    return HAL_OK;
}

#undef IMU_INIT_CALL

/** @brief Flush the IMU FIFO and clear sample validity before resuming streaming. */
HAL_StatusTypeDef imu_reset_fifo(IMU *imu)
{
    if (lsm6dsv16x_fifo_mode_set(&imu->dev_ctx, LSM6DSV16X_BYPASS_MODE) != 0)
        return HAL_ERROR;
    imu->valid_data = 0;
    imu->fifo_count = 0xff;
    imu->fifo_timestamp_valid = imu->timestamp_pending = 0;
    if (lsm6dsv16x_fifo_mode_set(&imu->dev_ctx, LSM6DSV16X_STREAM_MODE) != 0)
        return HAL_ERROR;
    return HAL_OK;
}

/* AN5763 section 9.11: timestamp every FIFO time slot. Correlate by TAG_CNT,
 * never by SPI read time; tolerate either ordering within a slot and partial
 * groups at the boundary of a drain. Ignore data until a timestamp is present. */
static void timestamp_pending(IMU *imu)
{
    if (!imu->fifo_timestamp_valid) return;
    uint8_t pending = imu->timestamp_pending;
    if (pending & IMU_DATA_ACC) imu->raw_acc = imu->fifo_timestamp;
    if (pending & IMU_DATA_GYRO) imu->raw_gyro = imu->fifo_timestamp;
    if (pending & IMU_DATA_SFLP) imu->raw_quat = imu->fifo_timestamp;
    imu->fresh_timestamps |= pending;
    imu->valid_data |= pending;
    imu->timestamp_pending = 0;
}

/* Read the live IMU clock AFTER draining, so every FIFO sample precedes the
 * anchor. Bracket the SPI read on TIM2 and use its midpoint. Reanchor each drain
 * to avoid accumulating oscillator-rate error; 21.75 us is the nominal IMU tick.
 * Unsigned subtraction handles the IMU's 32-bit timestamp rollover. */
static HAL_StatusTypeDef map_timestamps(IMU *imu)
{
    if (!imu->fresh_timestamps) return HAL_OK;
    uint32_t raw_now;
    uint64_t before = sensor_clock_now_us();
    if (lsm6dsv16x_timestamp_raw_get(&imu->dev_ctx, &raw_now)) return HAL_ERROR;
    uint64_t after = sensor_clock_now_us();
    if (after - before > 1000U) {
        /* Preempted SPI reads do not establish a useful synchronization anchor.
         * Drop these samples and wait for a fresh slot rather than stamp them. */
        imu->valid_data &= ~imu->fresh_timestamps;
        return HAL_OK;
    }
    uint64_t anchor = before + (after - before) / 2U;
    uint32_t raw[] = {imu->raw_acc, imu->raw_gyro, imu->raw_quat};
    uint64_t *out[] = {&imu->timestamp_acc, &imu->timestamp_gyro, &imu->timestamp_quat};
    for (unsigned i = 0; i < 3; ++i) {
        if (!(imu->fresh_timestamps & (1U << i))) continue;
        uint64_t age = ((uint64_t)(uint32_t)(raw_now - raw[i]) * 87U + 2U) / 4U;
        if (age > anchor || age > 5000000U) {
            imu->valid_data &= ~(1U << i);
            continue;
        }
        *out[i] = anchor - age;
    }
    return HAL_OK;
}

/** @brief Drain available FIFO entries into the latest acceleration, gyro, and quaternion samples. */
HAL_StatusTypeDef imu_get_data_blocking(IMU *imu)
{
    int32_t errorcode = 0;

    stmdev_ctx_t *dev_ctx = &(imu->dev_ctx);
    lsm6dsv16x_fifo_out_raw_t * f_data = &(imu->f_data);
    errorcode |= lsm6dsv16x_fifo_status_get(dev_ctx, &(imu->fifo_status));
    if (errorcode != 0) return HAL_ERROR;
    /* Discontinuity requires a FIFO flush, rather than stopping sampling. */
    if (imu->fifo_status.fifo_ovr) return HAL_BUSY;

    uint16_t fifo_level = imu->fifo_status.fifo_level;
    imu->fresh_timestamps = 0;

    while(fifo_level > 0)
    {
        errorcode |= lsm6dsv16x_fifo_out_raw_get(dev_ctx, f_data);
        if (errorcode != 0) return HAL_ERROR;
        if (f_data->cnt != imu->fifo_count) {
            imu->fifo_count = f_data->cnt;
            imu->fifo_timestamp_valid = imu->timestamp_pending = 0;
        }
        int16_t raw[3];
        memcpy(raw, f_data->data, sizeof(raw));

        switch(f_data->tag)
        {
            case LSM6DSV16X_TIMESTAMP_TAG:
                memcpy(&imu->fifo_timestamp, f_data->data, 4);
                imu->fifo_timestamp_valid = 1;
                break;
            case LSM6DSV16X_XL_NC_TAG:
                for (unsigned i = 0; i < 3; ++i)
                    imu->acc_mg[i] = imu->config.acc_g == 2 ? lsm6dsv16x_from_fs2_to_mg(raw[i]) :
                        imu->config.acc_g == 4 ? lsm6dsv16x_from_fs4_to_mg(raw[i]) :
                        imu->config.acc_g == 8 ? lsm6dsv16x_from_fs8_to_mg(raw[i]) : lsm6dsv16x_from_fs16_to_mg(raw[i]);
                imu->valid_data &= ~IMU_DATA_ACC;
                imu->timestamp_pending |= IMU_DATA_ACC;
                break;
            case LSM6DSV16X_GY_NC_TAG:
                for (unsigned i = 0; i < 3; ++i)
                    imu->gyro_mdps[i] = imu->config.gyro_dps == 250 ? lsm6dsv16x_from_fs250_to_mdps(raw[i]) :
                        imu->config.gyro_dps == 500 ? lsm6dsv16x_from_fs500_to_mdps(raw[i]) :
                        imu->config.gyro_dps == 1000 ? lsm6dsv16x_from_fs1000_to_mdps(raw[i]) : lsm6dsv16x_from_fs2000_to_mdps(raw[i]);
                imu->valid_data &= ~IMU_DATA_GYRO;
                imu->timestamp_pending |= IMU_DATA_GYRO;
                break;
            case LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG:
                {
                    uint16_t rotation[3];
                    memcpy(rotation, f_data->data, sizeof(rotation));
                    sflp2q(imu->quat, rotation);
                }
                imu->valid_data &= ~IMU_DATA_SFLP;
                imu->timestamp_pending |= IMU_DATA_SFLP;
                break;
            default:
                break;
        }

        timestamp_pending(imu);
        --fifo_level;
    }

    return map_timestamps(imu);
}
