#pragma once

#include <memory>

#include <hal/i2c_types.h>
#include <freertos/FreeRTOS.h>
#include <bme68x_defs.h>

enum class BME68xOperationMode : uint8_t
{
  Sleep = BME68X_SLEEP_MODE,
  Forced = BME68X_FORCED_MODE,
  Parallel = BME68X_PARALLEL_MODE,
};

class BME68x
{
private:
  class BME68xImpl;
  std::unique_ptr<BME68xImpl> impl_;
public:
  explicit BME68x(i2c_port_t num, int sda_num, int scl_num, uint8_t dev_addr, uint32_t clk_speed,
                  TickType_t wait_ticks);
  ~BME68x();

  bme68x_conf &get_conf();
  bool set_conf(bme68x_conf &conf);

  bme68x_heatr_conf &get_heatr_conf();
  bool set_heatr_conf(bme68x_heatr_conf &heatr_conf);

  uint8_t get_op_mode();
  bool set_op_mode(uint8_t mode);

  uint32_t get_meas_dur();

  bme68x_data *get_data(uint8_t &n_data);
};