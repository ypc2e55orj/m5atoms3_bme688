#include <M5Unified.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

[[noreturn]] static void mainTask(void *)
{
  M5Canvas canvas(&M5.Display);
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
    canvas.printf("%lld", esp_timer_get_time());
    canvas.pushSprite(0, 0);
  }
}

extern "C" [[maybe_unused]] void app_main(void)
{
  M5.begin();
  xTaskCreatePinnedToCore(mainTask, "mainTask", 8192, nullptr, 25,  nullptr, 1);
}
