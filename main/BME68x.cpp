#include "BME68x.hpp"

#include <cstring>
#include <stdexcept>

#include <rom/ets_sys.h>
#include <esp_log.h>
#include <driver/i2c.h>
#include <bme68x.h>
#include <bme68x_defs.h>

struct BME68xCommon
{

  explicit BME68xCommon() = default;
  virtual ~BME68xCommon() = default;

  virtual bool read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len) = 0;
  virtual bool write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len) = 0;
};

class BME68xCommonI2C final : public BME68xCommon
{
private:
  static constexpr uint8_t I2C_WRITE_BUFFER = 0xFF;

  i2c_port_t num_;
  uint8_t dev_addr_;
  TickType_t wait_ticks_;
  uint8_t buffer_[I2C_WRITE_BUFFER];

public:
  explicit BME68xCommonI2C(i2c_port_t num, int sda_num, int scl_num, uint8_t dev_addr, uint32_t clk_speed,
                           TickType_t wait_ticks)
    : num_(num), dev_addr_(dev_addr), wait_ticks_(wait_ticks), buffer_()
  {
    // I2Cの初期化
    i2c_config_t i2c_config = {};
    i2c_config.mode = I2C_MODE_MASTER;
    i2c_config.sda_io_num = sda_num;
    i2c_config.scl_io_num = scl_num;
    i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.master.clk_speed = clk_speed;

    i2c_param_config(num_, &i2c_config);
    ESP_ERROR_CHECK(i2c_driver_install(num_, i2c_config.mode, 0, 0, 0));
  }

  ~BME68xCommonI2C() override
  {
    i2c_driver_delete(num_);
  }

  bool read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len) override
  {
    return i2c_master_write_read_device(num_, dev_addr_, &reg_addr, 1, reg_data, len, wait_ticks_) == ESP_OK;
  }

  bool write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len) override
  {
    buffer_[0] = reg_addr;
    memcpy(buffer_ + 1, reg_data, len);
    return i2c_master_write_to_device(num_, dev_addr_, buffer_, len + 1, wait_ticks_) == ESP_OK;
  }

};

class BME68x::BME68xImpl
{
private:
  std::unique_ptr<BME68xCommon> common_;
  bme68x_dev dev_;
  bme68x_conf conf_;
  bme68x_heatr_conf heatr_conf_;
  bme68x_data data_[3];
  uint8_t mode_;

  // 0: success, 1: error
  static BME68X_INTF_RET_TYPE bme68x_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
  {
    auto this_ptr = reinterpret_cast<BME68x::BME68xImpl *>(intf_ptr);
    return this_ptr->common_->read(reg_addr, reg_data, len) ? 0 : 1;
  }
  static BME68X_INTF_RET_TYPE bme68x_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
  {
    auto this_ptr = reinterpret_cast<BME68x::BME68xImpl *>(intf_ptr);
    return this_ptr->common_->write(reg_addr, reg_data, len) ? 0 : 1;
  }
  static void bme68x_delay_us(uint32_t period, void *)
  {
    ets_delay_us(period);
  }

  // https://github.com/boschsensortec/BME68x-Sensor-API/blob/v4.4.8/examples/common/common.c#L84
  static bool bme68x_check(const char api_name[], int8_t result)
  {
    const char *bme68x_error[] = {
      "",
      "BME68X_E_NULL_PTR",
      "BME68X_E_COM_FAIL",
      "BME68X_E_DEV_NOT_FOUND",
      "BME68X_E_INVALID_LENGTH",
      "BME68X_E_SELF_TEST",
      };

    const char *bme68x_warn[] = {
      "",
      "BME68X_W_DEFINE_OP_MODE",
      "BME68X_W_NO_NEW_DATA",
      "BME68X_W_DEFINE_SHD_HEATR_DUR",
      };

    switch (result)
    {
      case BME68X_OK:
        return true;

      case BME68X_E_NULL_PTR:
      case BME68X_E_COM_FAIL:
      case BME68X_E_DEV_NOT_FOUND:
      case BME68X_E_INVALID_LENGTH:
      case BME68X_E_SELF_TEST:
        ESP_LOGE(api_name, "%s (%d)", bme68x_error[-result], result);
        break;

     case BME68X_W_NO_NEW_DATA:
        ESP_LOGW(api_name, "%s (%d)", bme68x_warn[result], result);
        break;

      default:
        ESP_LOGW(api_name, "Unknown error (%d)", result);
        break;
    }

    return false;
  }

  void init()
  {
    dev_.read = BME68xImpl::bme68x_read;
    dev_.write = BME68xImpl::bme68x_write;
    dev_.delay_us = BME68xImpl::bme68x_delay_us;
    dev_.intf = BME68X_I2C_INTF;
    dev_.intf_ptr = reinterpret_cast<void *>(this);
    dev_.amb_temp = 25;
    bme68x_check("bme68x_init()", bme68x_init(&dev_));
  }

public:
  /* I2C */
  BME68xImpl(i2c_port_t num, int sda_num, int scl_num, uint8_t dev_addr, uint32_t clk_speed, TickType_t wait_ticks)
    : common_(new BME68xCommonI2C(num, sda_num, scl_num, dev_addr, clk_speed, wait_ticks)),
      dev_(), conf_(), heatr_conf_(), data_(), mode_()
  {
    init();
    bme68x_check("bme68x_get_conf()", bme68x_get_conf(&conf_, &dev_));
  }
  /* SPI */
  // BME68xImpl() {}

  ~BME68xImpl() = default;

  void delay(uint32_t period)
  {
    dev_.delay_us(period, reinterpret_cast<void *>(this));
  }

  bme68x_conf &get_conf()
  {
    return conf_;
  }
  bool set_conf(bme68x_conf &conf)
  {
    if (!bme68x_check("bme68x_set_conf()", bme68x_set_conf(&conf, &dev_)))
    {
      return false;
    }
    conf_ = conf;
    return true;
  }

  bme68x_heatr_conf &get_heatr_conf()
  {
    return heatr_conf_;
  }
  bool set_heatr_conf(bme68x_heatr_conf &heatr_conf)
  {
    if (!bme68x_check("bme68x_set_conf()", bme68x_set_heatr_conf(mode_, &heatr_conf, &dev_)))
    {
      return false;
    }
    heatr_conf_ = heatr_conf;
    return true;
  }

  uint8_t get_op_mode()
  {
    bme68x_check("bme68x_get_op_mode()", bme68x_get_op_mode(&mode_, &dev_));
    return mode_;
  }
  bool set_op_mode(uint8_t mode)
  {
    if (!bme68x_check("bme68x_set_op_mode()", bme68x_set_op_mode(mode, &dev_)))
    {
      return false;
    }
    mode_ = mode;
    return true;
  }

  uint32_t get_meas_dur()
  {
    return bme68x_get_meas_dur(mode_, &conf_, &dev_);
  }

  bme68x_data *get_data(uint8_t &n_data)
  {
    delay(get_meas_dur());
    if (!bme68x_check("bme68x_get_data()", bme68x_get_data(mode_, data_, &n_data, &dev_)))
    {
      return nullptr;
    }

    return data_;
  }
};

BME68x::BME68x(i2c_port_t num, int sda_num, int scl_num, uint8_t dev_addr, uint32_t clk_speed, TickType_t wait_ticks)
  : impl_(new BME68xImpl(num, sda_num, scl_num, dev_addr, clk_speed, wait_ticks))
{}

BME68x::~BME68x() = default;

bme68x_conf &BME68x::get_conf()
{
  return impl_->get_conf();
}
bool BME68x::set_conf(bme68x_conf &conf)
{
  return impl_->set_conf(conf);
}

bme68x_heatr_conf &BME68x::get_heatr_conf()
{
  return impl_->get_heatr_conf();
}
bool BME68x::set_heatr_conf(bme68x_heatr_conf &heatr_conf)
{
  return impl_->set_heatr_conf(heatr_conf);
}

uint8_t BME68x::get_op_mode()
{
  return impl_->get_op_mode();
}
bool BME68x::set_op_mode(uint8_t mode)
{
  return impl_->set_op_mode(mode);
}

uint32_t BME68x::get_meas_dur()
{
  return impl_->get_meas_dur();
}

bme68x_data *BME68x::get_data(uint8_t &n_data)
{
  return impl_->get_data(n_data);
}