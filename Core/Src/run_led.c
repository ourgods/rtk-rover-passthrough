#include "main.h"
#include "cmsis_os.h"


void play_note(unsigned int on_ms, unsigned int off_ms)
{
    BEEP = 0;
    osDelay(on_ms);   // 发声长短
    BEEP = 0;
    osDelay(off_ms);  // 间隔
}

void play_music(void)
{
    // 一闪一闪亮晶晶
    play_note(50, 50);   // 1
    play_note(50, 50);   // 1
    play_note(100, 50);   // 5
    play_note(100, 50);   // 5
    play_note(150, 50);   // 6
    play_note(150, 50);   // 6
    play_note(100, 200);   // 5 —

}
//用于定期切换led的状态  运行状态指示灯？？ pb9
void RunLed_task(void const * argument)
{
    /* USER CODE BEGIN run_led */
    /* Infinite loop */
    uint32_t led_counter;
    
    play_music();
    
    for(;;)
    {
        led_counter++;
        RUN_LED = !(led_counter & 4);
        //    BEEP = led_counter &1;
        osDelay(100);
    }
    /* USER CODE END run_led */
}