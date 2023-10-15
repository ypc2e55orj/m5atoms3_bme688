#include <M5Unified.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include "BME68x.hpp"

[[noreturn]] static void mainTask(void *)
{
  M5Canvas canvas(&M5.Display);
  BME68x bme688(I2C_NUM_1, GPIO_NUM_38, GPIO_NUM_39, 0x76, 400'000, pdMS_TO_TICKS(10));

  bme688.set_op_mode(BME68X_PARALLEL_MODE);

  auto conf = bme688.get_conf();
  conf.filter = BME68X_FILTER_OFF;
  conf.odr = BME68X_ODR_NONE;
  conf.os_hum = BME68X_OS_1X;
  conf.os_pres = BME68X_OS_16X;
  conf.os_temp = BME68X_OS_2X;
  bme688.set_conf(conf);

  uint16_t heatr_temp_prof[] = {320, 100, 100, 100, 200, 200, 200, 320, 320, 320};
  uint16_t heatr_mul_prof[] = {5, 2, 10, 30, 5, 5, 5, 5, 5, 5};
  auto heatr_conf = bme688.get_heatr_conf();
  heatr_conf.enable = BME68X_ENABLE;
  heatr_conf.heatr_temp_prof = heatr_temp_prof;
  heatr_conf.heatr_dur_prof = heatr_mul_prof;
  heatr_conf.shared_heatr_dur = static_cast<uint16_t>(140 - bme688.get_meas_dur() / 1000);
  heatr_conf.profile_len = 10;
  bme688.set_heatr_conf(heatr_conf);

  bme688.set_op_mode(BME68X_PARALLEL_MODE);

  const auto width = M5.Display.width();
  const auto height = M5.Display.height();

  auto xLastWakeTime = xTaskGetTickCount();
  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    M5.update();
    canvas.createSprite(width, height);
    canvas.setTextColor(WHITE, BLACK);
    canvas.fillScreen(BLACK);
    canvas.setCursor(0, height / 2, &Font4);
    canvas.printf("Hello, world");
    canvas.pushSprite(0, 0);

    uint8_t n;
    auto data = bme688.get_data(n);
    for (uint8_t i = 0; i < n; i++)
    {
      if (data[i].status == 0xB0)
      {
        printf("%.2f, %.2f, %.2f, %.2f, 0x%x, %d, %d\n",
               data[i].temperature,
               data[i].pressure,
               data[i].humidity,
               data[i].gas_resistance,
               data[i].status,
               data[i].gas_index,
               data[i].meas_index);
      }
    }
  }
}

extern "C" [[maybe_unused]] void app_main(void)
{
  M5.begin();
  xTaskCreatePinnedToCore(mainTask, "mainTask", 8192, nullptr, 25, nullptr, 1);
}
