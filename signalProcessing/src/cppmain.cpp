
/** motor reference systems
 * rotor referenced: d,q
 * stator frame referenced: alpha, beta
 * stator coil referenced: a,b,c
 *
 * clark transform:
 *  a,b,c => alpha, beta, homopolar component of current measured.
 *
 *  park transform:
 *  alpha, beta, theta => d,q
 *
 *  i_d = alpha*cos(theta) + beta*sin(theta)
 *
 * d,
 */

// todo moc hal so this is testable cross platform
#include "cppmain.h"
#include "main.h"
//#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include <stm32f4xx.h>
#include <stm32f4xx_hal_adc.h>
#include <stm32f4xx_hal_adc_ex.h>
#include <string>
#include <array>
#include <variant>
#include <algorithm>
#include <functional>
namespace constants{
    constexpr float pi = 3.14159265358979323846;
    constexpr float two_pi = 2*pi;
    constexpr float half_pi = pi/2;
    constexpr float quarter_pi = pi/4;
    constexpr float three_quarter_pi = 3*pi/4;
    constexpr float one_over_pi = 1/pi;
}
namespace math{
    namespace detail {
        template<typename Real>
        constexpr Real wrap(Real x) {
            // standardize the angle so that -pi <= x < pi
            // clang-format off
            return (x <= -constants::pi) ? wrap(x + 2 * constants::pi) :
                   (x > constants::pi) ? wrap(x - 2 * constants::pi) :
                   (true) ? x : 0;
            // clang-format on
        }
        template <typename Real>
        constexpr Real sin_cfrac(Real x2, int k = 2, int n = 40) {
            return (n == 0) ? k * (k + 1) - x2
                            : k * (k + 1) - x2 +
                              (k * (k + 1) * x2) / sin_cfrac(x2, k + 2, n - 1);
        }
//        template <typename Real>
//        constexpr Real sqrt_newton(Real x, Real guess) {
//            return is_close(guess * guess, x)
//                   ? guess
//                   : sqrt_newton(x, (guess + x / guess) / static_cast<Real>(2));
//        }

        template <typename Real>
        constexpr Real sqr(Real x) {
            return x * x;
        }
    }// namespace detail
//    template <typename Real>
//    constexpr Real sqrt(Real x) {
//        return (x < 0) ? throw std::domain_error("x >= 0")
//                       : detail::sqrt_newton(x, x);
//    }
    template <typename Real>
    constexpr Real sin(Real x) {
        return detail::wrap(x) /
               (1 + detail::sqr(detail::wrap(x)) / detail::sin_cfrac(detail::sqr(detail::wrap(x))));
    }
    template <typename Real>
    constexpr Real cos(Real x) {
        return sin(constants::half_pi - x);
    }
}

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
    class Executor {
        // variant is a union of all possible types
        //std::variant<int> callable;
        // execute a sequence of callables
        template<class Callable>
        constexpr void add(Callable c) {
            // add callable to a variant queue

        }
    };

    class SerialLogger {
        constexpr void operator()() {
            //std::string msg = "hello\n";
            // send to usb serial
        }
    };

    class LedPatern {
    private:
        uint8_t led_count;
        uint8_t last_gray_code = 0;
    public:
        constexpr void operator()() {
            led_count++;
            led_count %= 0x80;
            //calculate grey code of led_count
            const auto grey_code = (led_count >> 1) ^ led_count;
            const auto pattern = grey_code ^ last_gray_code;
            set_led_bit(~pattern);
            last_gray_code = grey_code;
        }
    };

    class ThreePhasePwm {
        using Numeric = float;
    private:
        //handle to the timer
        TIM_HandleTypeDef m_timer;
        TIM_OC_InitTypeDef m_sConfig1;
        TIM_OC_InitTypeDef m_sConfig2;
        TIM_OC_InitTypeDef m_sConfig3;
        TIM_OC_InitTypeDef m_sConfig4;
    public:
        ThreePhasePwm(TIM_HandleTypeDef timer) : m_timer(timer) {
            // init for advanced pwm count up and down 20khz
            HAL_TIM_PWM_Start(&m_timer, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&m_timer, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&m_timer, TIM_CHANNEL_3);
            // set pwm time base
            m_timer.Init.Prescaler = 0;
            m_timer.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
            m_timer.Init.Period = 0xFFFF;
            m_timer.Init.RepetitionCounter = 0x080;
            HAL_TIM_Base_Init(&m_timer);

//          TIM_Base_SetConfig(&m_timer, &m_timer.Init);
            // SET COUNT TO 4096 FOR 20KHZ

            // initialize pwm config
            m_sConfig1 = {
                    .OCMode = TIM_OCMODE_PWM1,
                    .Pulse = 0xFFFF,
                    .OCPolarity = TIM_OCPOLARITY_HIGH,
                    .OCNPolarity = TIM_OCNPOLARITY_HIGH,
                    .OCFastMode = TIM_OCFAST_DISABLE,
                    .OCIdleState = TIM_OCIDLESTATE_RESET,
                    .OCNIdleState = TIM_OCNIDLESTATE_RESET
            };
            m_sConfig2 = {
                    .OCMode = TIM_OCMODE_PWM1,
                    .Pulse = 0xFFFF,
                    .OCPolarity = TIM_OCPOLARITY_HIGH,
                    .OCNPolarity = TIM_OCNPOLARITY_HIGH,
                    .OCFastMode = TIM_OCFAST_DISABLE,
                    .OCIdleState = TIM_OCIDLESTATE_RESET,
                    .OCNIdleState = TIM_OCNIDLESTATE_RESET
            };
            m_sConfig3 = {
                    .OCMode = TIM_OCMODE_PWM1,
                    .Pulse = 0xFFFF,
                    .OCPolarity = TIM_OCPOLARITY_HIGH,
                    .OCNPolarity = TIM_OCNPOLARITY_HIGH,
                    .OCFastMode = TIM_OCFAST_DISABLE,
                    .OCIdleState = TIM_OCIDLESTATE_RESET,
                    .OCNIdleState = TIM_OCNIDLESTATE_RESET
            };
            m_sConfig4 = {
                    .OCMode = TIM_OCMODE_PWM1,
                    .Pulse = 0xFFFF,
                    .OCPolarity = TIM_OCPOLARITY_HIGH,
                    .OCNPolarity = TIM_OCNPOLARITY_HIGH,
                    .OCFastMode = TIM_OCFAST_DISABLE,
                    .OCIdleState = TIM_OCIDLESTATE_RESET,
                    .OCNIdleState = TIM_OCNIDLESTATE_RESET
            };

            //m_sConfig1.OCPolarity = TIM_OCPOLARITY_HIGH;
            HAL_TIM_PWM_ConfigChannel(&m_timer, &m_sConfig1, TIM_CHANNEL_1);
            HAL_TIM_PWM_ConfigChannel(&m_timer, &m_sConfig2, TIM_CHANNEL_2);
            HAL_TIM_PWM_ConfigChannel(&m_timer, &m_sConfig3, TIM_CHANNEL_3);
            HAL_TIM_PWM_ConfigChannel(&m_timer, &m_sConfig4, TIM_CHANNEL_4);
            // set channel 4 to 100% duty cycle

            HAL_TIM_PWM_Start(&m_timer, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&m_timer, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&m_timer, TIM_CHANNEL_3);
            HAL_TIM_PWM_Start(&m_timer, TIM_CHANNEL_4);
        }
        void update_from_phasePQ(Numeric  p, Numeric q,Numeric theta){
            // todo update to constexpr funnction
            const auto alpha = p*math::cos(theta) - q*math::sin(theta);
            const auto beta = p*math::sin(theta) + q*math::cos(theta);
            // inverse clark transform converts 90 degree phase alpha, beta to three phase
            // square root of 3 is 1.732050807568877
            constexpr auto sqr3Over2 = 1.732050807568877/2;
            const auto a = alpha;
            const auto b= -1/2*alpha + sqr3Over2*beta;
            const auto c = -1/2*alpha - sqr3Over2*beta;
        }
    };

    class serialCommander {

    };

    class ADCDriver {
    private:
        ADC_HandleTypeDef m_hadc;
        //buffer to store adc values
        std::array<uint32_t, 8> m_adc_buffer;
        std::function<void(uint32_t)> m_callback;
        uint32_t m_count = 0;
        bool m_ready = false;
    public:
        ADCDriver(ADC_HandleTypeDef hAdc) :
                m_hadc{hAdc} {
            HAL_ADC_Start_DMA(&m_hadc, m_adc_buffer.data(), m_adc_buffer.size());
            // conversion on all channels, no trigger
            HAL_ADC_Start_IT(&m_hadc);
            // call callback when conversion is done
            m_ready = true;
        }

        bool isReady() {
            return m_ready;
        }

        uint32_t get_adc_value(uint8_t channel) {
            m_ready = false;
            return m_adc_buffer[channel];
        }

        void set_callback(std::function<void(uint32_t)> callback) {
            m_callback = callback;
        }
    };

    class NokiaLcd{
        /**
         * reset, PG1
         * cs, PF14
         * command, PF13
         * mosi, PC12
         * sclk PC10
         */
    private:
        SPI_HandleTypeDef m_hspi;
    public:
         void send_bitmap48_64(std::array<uint8_t,504> bitmap) {
             // reset
             HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
             HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
             // cs
             HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);
             // data mode
             HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);
             // send bitmap over spi one byte at a time
            HAL_SPI_Transmit(&m_hspi, bitmap.data(), bitmap.size(), 100);
             // cs
             HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);
         }
         NokiaLcd(SPI_HandleTypeDef spi):
             m_hspi(spi){
             // init spi
             HAL_SPI_Init(&m_hspi);
         }
        // output
    };

    class AdcDriver {
        // use static function as callback.
        // this is a hack to get around the fact that function pointers are needed to be passed to HAL
        // conversion complete
    private:
        ADC_HandleTypeDef m_hadc;
        //buffer to store adc values
        std::array<uint32_t, 8> m_adc_buffer;
        std::function<void(uint32_t)> m_callback;
        static uint32_t m_count;
    public:

        static void conversion_complete(ADC_HandleTypeDef *hadc) {
            // get adc driver from static member
            //auto adc_driver = static_cast<AdcDriver*>(hadc->UserData);
            // call callback
            //adc_driver->conversion_complete();
            m_count++;
        }

    };

} // namespace


/**
 * this is the entry point for the c++ program
 * the hardware is initialized in main.c
 *
 */

extern "C" {
// function to call at each system tick
void my_systick_Callback(void) {

//        HAL_IncTick();
//        HAL_SYSTICK_IRQHandler();
    //ledPattern();

}

[[noreturn]] void cppmain(struct p_HW hw) {
    global_hw = hw;
    set_led_bit(0x7);
    // TODO: Add your code here
    LedPatern ledPattern;
    NokiaLcd lcd(*hw.spi3);
    lcd.send_bitmap48_64({});
    ThreePhasePwm pwm1(* (hw.tim1));
    ThreePhasePwm pwm2(* (hw.tim2));
    ThreePhasePwm pwm3(* (hw.tim3));
    ThreePhasePwm pwm4(* (hw.tim4));
    for(;;) {
        HAL_IWDG_Refresh(global_hw.iwdg);
        ledPattern();
        HAL_Delay(0);
        std::array<uint32_t, 8> adc_buffer={1,2,3};
        // convert buffer to json array.
        std::string json_string = "[";
        // iterate over adc buffer
        std::for_each_n(std::begin(adc_buffer), adc_buffer.size()-1, [&json_string](uint32_t value) {
            json_string += std::to_string(value) + ",";
        });
        if (adc_buffer.size() > 0) {
            json_string += std::to_string(adc_buffer.back()) + "]";
        }else{
            json_string += "]";
        }
        // send json string to usb serial
        json_string = "[1,2,3]";
        std::array<uint8_t, 1000> hello{"[5,6,7,8]"};
        std::copy(std::begin(json_string), std::end(json_string), std::begin(hello));
        // find length of null terminated string in hello
        const auto length = std::string(hello.begin(), hello.end()).find('\0');

        //CDC_Transmit_HS(hello.data(), length-1);
        //CDC_Transmit_HS( (uint8_t*) json_string.c_str(), json_string.size());
    }
}
}