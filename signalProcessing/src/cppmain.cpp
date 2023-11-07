
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
 *
 *
 * performance notes runs led RelWithDebInfo at 476 khz
 *
 */
#include <dsp/controller_functions.h>
// todo moc hal so this is testable cross platform
#include "cppmain.h"
#include "main.h"
//#include "usbd_cdc.h"
#include "signal.h"
#include "usbd_cdc_if.h"
#include <stm32f4xx.h>
#include <stm32f4xx_hal_adc.h>
#include <stm32f4xx_hal_adc_ex.h>
#include <string>
#include <array>
#include <variant>
#include <algorithm>
#include <functional>
#include <valarray>
#include <numeric>
#include <cmath>
namespace constants{
    constexpr float pi = 3.14159265358979323846;
    constexpr float two_pi = 2*pi;
    constexpr float half_pi = pi/2;
    constexpr float quarter_pi = pi/4;
    constexpr float three_quarter_pi = 3*pi/4;
    constexpr float one_over_pi = 1/pi;
}
//input an angle in radians, and track in absolute rotation. if the new angle is more that 2pi
//from the old angle, it will be wrapped around to the other side of the circle.
class AngleTracker{
    float angle;
    float pi = constants::pi;
public:
    explicit AngleTracker(float angle=0.0f):angle(angle){}
    float get(){return angle;}
    float set(float new_sensor_value){
        auto equivilent_angle = new_sensor_value;
        auto delta = equivilent_angle - angle;
        //make delta between -pi and pi by adding or subtracting 2pi with
        // floating point remainder
        auto remainder = std::fmod(delta+pi,constants::two_pi);
        if(remainder < 0) remainder += constants::two_pi;
        angle += remainder - pi;
        return angle;
    }
};

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

        template <typename Real>
        constexpr Real sqr(Real x) {
            return x * x;
        }
    }
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
//        constexpr uint16_t led_mask = 0x00ff;
#if 0
        for(int i=0;i<1;++i) {
            // the HAL methed requires two calls since the upper 16 bits of bssr,
            // and the lower 16 bits of bssr can not be set in the same HAL call
            HAL_GPIO_WritePin(GPIOE, led_bit, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOE, ~led_bit, GPIO_PIN_RESET);
        }
#else
            GPIOE->BSRR = (uint32_t)(~led_bit) << 16 | led_bit;
#endif
    }

    // class is templated to all possible Callable types
    class Executor {
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
            //led_count %= 0x80;
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
        arm_pid_instance_f32 m_thetaPid{};
        float m_raw_theta{};
        float m_thetaSin{};
        float m_thetaCos{};
        //handle to the timer
        TIM_HandleTypeDef m_timer;
        TIM_OC_InitTypeDef m_sConfig1;
        TIM_OC_InitTypeDef m_sConfig2;
        TIM_OC_InitTypeDef m_sConfig3;
        TIM_OC_InitTypeDef m_sConfig4;
        // todo set APB1 clock, and ABP2 clock to same frequency
        static constexpr uint32_t m_period = 42000/40; // divide by desired frequency in khz
    public:
        ThreePhasePwm(TIM_HandleTypeDef timer) : m_timer(timer) {
            m_thetaPid.Kp = 0.5;
            m_thetaPid.Ki = 0.0;
            m_thetaPid.Kd = 0.0;
            m_thetaSin = math::sin(m_thetaPid.state[2]);
            m_thetaCos = math::cos(m_thetaPid.state[2]);
            //arm_sin_cos_f32(0, &m_thetaSin, &m_thetaCos);
            arm_pid_init_f32(&m_thetaPid, true);
            const auto pulse_width = m_period / 2;
            // set pwm time base
            m_timer.Init.Prescaler = 0;
            m_timer.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
            m_timer.Init.Period = m_period;
            m_timer.Init.RepetitionCounter = 0x00;
            HAL_TIM_PWM_DeInit(&m_timer);
            HAL_TIM_PWM_Init(&m_timer);
            HAL_TIM_Base_Init(&m_timer);
            // initialize pwm config
            m_sConfig1 = {};
            m_sConfig1.OCMode = TIM_OCMODE_PWM1;
            m_sConfig1.Pulse = pulse_width;
            m_sConfig1.OCPolarity = TIM_OCPOLARITY_HIGH;
            m_sConfig1.OCNPolarity = TIM_OCNPOLARITY_HIGH;
            m_sConfig1.OCFastMode = TIM_OCFAST_DISABLE;
            m_sConfig1.OCIdleState = TIM_OCIDLESTATE_RESET;
            m_sConfig1.OCNIdleState = TIM_OCNIDLESTATE_RESET;

            m_sConfig2 = m_sConfig1;
            m_sConfig3 = m_sConfig1;
            // set m_sConfig4 to enabled.
            m_sConfig4 = m_sConfig1;
            m_sConfig4.Pulse = 0x00;

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
        template <class Numeric>
        constexpr Numeric truncate1(Numeric value) {
            constexpr Numeric upper = 1.0;
            constexpr Numeric lower = -1.0;
            return std::min(std::max(value, lower), upper);
        }
        void updateCurrentAngle(float _a, float _b, float motorTorque){
            const auto a = truncate1(_a);
            const auto b = truncate1(_b);
            float alpha{}, beta{};
            arm_clarke_f32(a, b, &alpha, &beta);
//            float theta = m_thetaPid.
            float d{}, q{};
            arm_park_f32(alpha, beta, &d, &q,m_thetaSin, m_thetaCos);
            m_raw_theta = std::atan2(d, q); // todo check ordering
            arm_pid_f32(&m_thetaPid, m_raw_theta);
            const auto new_theta = m_thetaPid.state[2];
//            arm_sin_cos_f32(new_theta, &m_thetaSin, &m_thetaCos);
            m_thetaSin = std::sin(new_theta);
            m_thetaCos = std::cos(new_theta);
            update_from_phasePQ(motorTorque, 0);
            //setabc(m_thetaCos*motorTorque, m_thetaCos*motorTorque, motorTorque*new_theta);
        }
        void enable_output(){
            //m_period
            m_sConfig4.Pulse = 0xffff;
            __HAL_TIM_SET_COMPARE(&m_timer, TIM_CHANNEL_4, 0xffff);
        }
        void disable_output(){
            m_sConfig4.Pulse = 0;
            __HAL_TIM_SET_COMPARE(&m_timer, TIM_CHANNEL_4, 0);
        }
        void update_from_phasePQ(Numeric d, Numeric q, Numeric theta){
            m_thetaCos = std::cos(theta);
            m_thetaSin = std::sin(theta);
            return update_from_phasePQ(d, q);
        }

        void update_from_phasePQ(Numeric  d, Numeric q){
            // todo update to constexpr funnction
            // todo use q.
            (void) q;

#if 0 // trying to make this faster.
            const auto alpha = p*math::cos(theta) - q*math::sin(theta);
            const auto beta = p*math::sin(theta) + q*math::cos(theta);
            // inverse clark transform converts 90 degree phase alpha, beta to three phase
            // square root of 3 is 1.732050807568877
            constexpr Numeric sqr3Over2 = 1.732050807568877/2;
            const auto a = alpha;
            const auto b= -1.0F/2*alpha + sqr3Over2*beta;
            const auto c = -1.0F/2*alpha - sqr3Over2*beta;
#else
            float alpha{}, beta{};
             auto id = d;
            constexpr auto iq = 0;
            arm_inv_park_f32(id, iq, & alpha, &beta, m_thetaSin, m_thetaCos);
            float a{},b{};
            arm_inv_clarke_f32(alpha, beta, &a, &b);
            // solve for c where a+b+c = 0
            const auto c = -(a+b);
#endif
            setabc(a*d, b*d, c*d);
        }
        template<typename Numeric>
        static constexpr Numeric envelope(Numeric input, Numeric lowerLimit, Numeric upperLimit){
            return std::min(upperLimit, std::max(input, lowerLimit));
        }
        template<typename Numeric, uint32_t upper>
        constexpr uint32_t scaleToRange(Numeric a){
            // take a number in the range -1, 1, and scale between 0, and upper.
            auto unbounded = static_cast<uint32_t>((a/2+.5)*upper);
            return envelope<uint32_t>(unbounded, 0, m_period);

        }
        /**
         * this is a duplicate of update_from_phasePQ, but with a different interface.
         * todo is it d, or q the torque?
         * @tparam Numeric
         * @param d
         * @param theta
         * @return
         */
        //template<typename Numeric>
        //using Numeric = float;
#if 0
        void setPhase(const float d, const float theta){
            constexpr auto mySin = math::sin(theta);
            constexpr auto myCos = math::cos(theta);
            constexpr auto alpha = d*myCos;
            constexpr auto beta = d*mySin;
            // inverse clark transform converts 90 degree phase alpha, beta to three phase
            // square root of 3 is 1.732050807568877
            constexpr auto sqr3Over2 = 1.732050807568877/2;
            const auto a = alpha;
            const auto b= -1/2*alpha + sqr3Over2*beta;
            const auto c = -1/2*alpha - sqr3Over2*beta;
            setabc(a,b,c);
        }
#endif
        // three phase version is called by polar rotor referenced version
        constexpr void setabc(const float a,const float b,const float c){
            // take a,b, c between 0 and 1 and convert to pwm duty cycle
            // todo make this constexpr
//            constexpr auto max_pwm = m_period;
//            constexpr auto min_pwm = 0;
//            constexpr auto max_duty = 1;
//            constexpr auto min_duty = -1;
//            m_sConfig1.Pulse = scaleToRange<float, m_period>(a);
//            m_sConfig2.Pulse = scaleToRange<float, m_period>(b);
//            m_sConfig3.Pulse = scaleToRange<float, m_period>(c);
            // truncate inputs
            float a_ = truncate1(a);
            float b_ = truncate1(b);
            float c_ = truncate1(c);
            // scale to pwm period length. 0 input becomes fifty percent duty cycle
            m_sConfig1.Pulse = static_cast<uint32_t>((a_+1) * m_period)/2;
            m_sConfig2.Pulse = static_cast<uint32_t>((b_+1) * m_period)/2;
            m_sConfig3.Pulse = static_cast<uint32_t>((c_+1) * m_period)/2;
            const auto compare1 = m_sConfig1.Pulse;
            const auto compare2 = m_sConfig2.Pulse;
            const auto compare3 = m_sConfig3.Pulse;
#ifdef USE_PWM_HAL
            // this does not work with modern g++.
            __HAL_TIM_SET_COMPARE(&m_timer, TIM_CHANNEL_1, compare1);
            __HAL_TIM_SET_COMPARE(&m_timer, TIM_CHANNEL_2, compare2);
            __HAL_TIM_SET_COMPARE(&m_timer, TIM_CHANNEL_3, compare3);
#else
            m_timer.Instance->CCR1 = compare1;
            m_timer.Instance->CCR2 = compare2;
            m_timer.Instance->CCR3 = compare3;
#endif
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
             (void)hadc;
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
    void push_message(enum Event e){
        // todo
            (void)e;
    }
[[noreturn]] void cppmain(struct p_HW hw) {
    global_hw = hw;
    set_led_bit(0x7);
    // TODO: Add your code here
    LedPatern ledPattern{};
    NokiaLcd lcd(*hw.spi3);
    lcd.send_bitmap48_64({});
    ThreePhasePwm pwm1(* (hw.tim1));
    ThreePhasePwm pwm2(* (hw.tim2));
    ThreePhasePwm pwm3(* (hw.tim3));
    ThreePhasePwm pwm4(* (hw.tim4));
    pwm1.update_from_phasePQ(0, 0,0);
    pwm2.update_from_phasePQ(0, 0,0);
    pwm3.update_from_phasePQ(0, 0,0);
    pwm4.update_from_phasePQ(0, 0,0);
    pwm1.disable_output();
    pwm2.disable_output();
    pwm3.disable_output();
    pwm4.enable_output();
    pwm2.setabc(0,0,0);
    ledPattern();
//    float angle = 0;
    constexpr float d = .05; // the magnitude of the sinusoid
    for(;;) {
//        std::array<uint32_t, 8> adc_buffer={1,2,3};
        // iterate from 0 to 300
        for (auto i = 0u; i < currentSignal.size(); i++) {
            for(auto stepPerSample=0; stepPerSample<3;stepPerSample++) {
                constexpr auto phase_offset = currentSignal.size() / 3;
                const auto a_signal = currentSignal.at(i);
                const auto b_signal = currentSignal.at((i + phase_offset) % currentSignal.size());
                const auto c_signal = currentSignal.at((i + phase_offset + phase_offset) % currentSignal.size());
                // do it for all 4 pwms
                pwm1.updateCurrentAngle(a_signal, b_signal, d);
                pwm2.updateCurrentAngle(b_signal, c_signal, d);
                pwm3.updateCurrentAngle(c_signal, a_signal, d);
                pwm4.updateCurrentAngle(a_signal, b_signal, d);
                //pwm4.setabc(0,0,0);
                HAL_IWDG_Refresh(global_hw.iwdg);
                // update led before and after delay to find how much processing time we are spending.
                ledPattern();
                HAL_Delay(0);
                ledPattern();
            }
        }

        //ledPattern();
        //Error_Handler();
        // todo: move pwm update into a timer callback
//        constexpr float twopi = 6.283185307179586476925286766559;
//        if(angle > twopi) {
//            angle -= twopi;
//        }
        //pwm2.setabc(angle,(angle),(angle));

        // find length of null terminated string in hello
        //const auto length = std::string(hello.begin(), hello.end()).find('\0');

        //CDC_Transmit_HS(hello.data(), length-1);
        //CDC_Transmit_HS( (uint8_t*) json_string.c_str(), json_string.size());
    }
}
}