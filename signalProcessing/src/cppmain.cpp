


// todo moc hal so this is testable cross platform
#include "cppmain.h"
#include "main.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include <stm32f4xx.h>
#include <string>
#include <array>
#include <variant>
#include <algorithm>
namespace {
    struct p_HW global_hw;
    constexpr void set_led_bit(uint8_t led_bit) {
//    const GPIO_TypeDef * port = GPIOE;
        for (int i = 0; i < 7; ++i) {
            if (led_bit & (1 << i)) {
                //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0<<i, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 << i, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 << i, GPIO_PIN_RESET);
            }
        }
    }

    // class is templated to all possible Callable types
    class Executor{
        // variant is a union of all possible types
        std::variant<int> callable;
        // execute a sequence of callables
        template<class Callable>
        void add(Callable c){
            // add callable to a variant queue

        }
    };
    class SerialLogger{
       void  operator()(){
            std::string msg = "hello\n";
            // send to usb serial
        }
    };
    class LedPatern {
    private:
        uint8_t led_count;
        uint8_t last_gray_code = 0;
    public:
        void operator()(){
            led_count++;
            led_count %= 0x80;
            //calculate grey code of led_count
            const auto grey_code = (led_count >> 1) ^ led_count;
            const auto pattern = grey_code ^ last_gray_code;
            set_led_bit(~pattern);
            last_gray_code = grey_code;
        }
    }ledPatern;

    class Pwm {
    };

    class serialCommander {

    };

} // namespace


/**
 * this is the entry point for the c++ program
 * the hardware is initialized in main.c
 *
 */

extern "C" {
    // function to call at each system tick
    void HAL_SYSTICK_Callback(void) {

//        HAL_IncTick();
//        HAL_SYSTICK_IRQHandler();
        ledPatern();

    }

void cppmain(struct p_HW hw)
{
        global_hw = hw;
    set_led_bit(0x7);
    // TODO: Add your code here
    LedPatern ledPatern;
    while (1) {
        HAL_IWDG_Refresh(global_hw.iwdg);

        HAL_Delay(400);
      std::array<uint8_t, 14> hello{"hello\n"};
      // find length of null terminated string in hello
      const auto length = std::string(hello.begin(), hello.end()).find('\0');
      CDC_Transmit_HS(hello.data(), hello.size());
        //ledPatern();
       HAL_Delay(0);
    }
}
}