#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/timer.h>
#include <FreeRTOS.h>
#include <task.h>
#include <string.h>
#include <stdlib.h>
#include <libopencm3/cm3/nvic.h>
#include "data_packet.h"
#include <assert.h>

#define ADC_CHANNELS 1, 2, 3, 4
#define THRESHOLD 50
#define U_PORT USART1

struct RadioDataPacket rdp = {0};
struct ControlDataPacket cdp = {0};
static uint16_t adc_dat[4];
static TaskHandle_t saw_task_hdl;

#define RECV_BUF_SIZE	10		/* Arbitrary buffer size */
char recv_buf[RECV_BUF_SIZE];
volatile uint8_t recv_ndx_nxt;		/* Next place to store */
volatile uint8_t recv_ndx_cur;		/* Next place to read */

static void adc_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(ADC1, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1 | GPIO2 | GPIO3 | GPIO4);

    // init ADC
    rcc_periph_clock_enable(RCC_ADC1);
    adc_power_off(ADC1);

    adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);
    adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
    adc_set_left_aligned(ADC1);

    uint8_t channels[] = {ADC_CHANNELS};
    adc_set_regular_sequence(ADC1, sizeof(channels), channels);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);

    adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
    adc_set_continuous_conversion_mode(ADC1);

    // init DMA
    rcc_periph_clock_enable(RCC_DMA1);

    dma_channel_reset(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t) &ADC_DR(ADC1));
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t) adc_dat);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
    dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_LOW);
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, sizeof(channels));
    dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
    dma_enable_channel(DMA1, DMA_CHANNEL1);

    adc_enable_dma(ADC1);

    uint8_t i = 255; // TODO: check registers
    while (i--);
    adc_power_on(ADC1);
}

#define PWM_TIM          TIM3
#define PWM_RCC          RCC_TIM3
#define PWM_PORT         GPIOA
#define PWM_PIN          GPIO6
#define TIM_PSC_DIV      8000

static void pwm_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(PWM_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_PIN);
    gpio_set_af(PWM_PORT, GPIO_AF1, PWM_PIN);

    rcc_periph_clock_enable(PWM_RCC);
    rcc_periph_reset_pulse(RST_TIM3);
    timer_set_mode(PWM_TIM, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(PWM_TIM, rcc_apb1_frequency);
    timer_disable_preload(PWM_TIM);
    timer_continuous_mode(PWM_TIM);
    timer_set_period(PWM_TIM, TIM_PSC_DIV);

    // PWM mode 1 on OC1 (output compare 1)
    timer_set_oc_mode(PWM_TIM, TIM_OC1, TIM_OCM_PWM2);
    timer_enable_oc_output(PWM_TIM, TIM_OC1);
    timer_set_oc_value(PWM_TIM, TIM_OC1, TIM_PSC_DIV);

    // Enable the timer peripheral and start the PWM
    timer_enable_break_main_output(PWM_TIM);
//    timer_enable_counter(PWM_TIM);
}

static void send_packet(void) {
    char buf[27];
    sprintf(buf, "#%05d%05d%05d%05d%01d%01d%02d\n", rdp.ch1, rdp.ch2, rdp.ch3, rdp.ch4, rdp.gen_on, rdp.force_gen, rdp.hash);

    uint8_t len = strlen(buf);
    char *ptr = buf;
    while (len--) {
        usart_send_blocking(U_PORT, *(ptr++));
    }
}

_Noreturn static void adc_analyze_task(void *args) {
    while (1) {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_HTIF);
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TEIF);
        adc_start_conversion_regular(ADC1);
        while (!(dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF) ||
                 dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TEIF)));

        if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TEIF)) { //TEIF - error flag
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        if (adc_dat[0] * 100 / 65536 > THRESHOLD || adc_dat[1] * 100 / 65536 > THRESHOLD ||
            adc_dat[2] * 100 / 65536 > THRESHOLD || adc_dat[3] * 100 / 65536 > THRESHOLD) {
            rdp.gen_on = true;
            if (!rdp.force_gen) {
                timer_enable_counter(PWM_TIM);
                vTaskResume(saw_task_hdl);
            }
        } else {
            rdp.gen_on = false;
            if (!rdp.force_gen) {
                vTaskSuspend(saw_task_hdl);
                timer_set_oc_value(PWM_TIM, TIM_OC1, TIM_PSC_DIV);
                timer_disable_counter(PWM_TIM);
            }
        }

        memcpy(rdp.channels, adc_dat, sizeof(uint16_t) * 4);
        rdp_gen_hash(&rdp);
        send_packet();

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

_Noreturn static void saw_task(void *args) {
    while (1) {
        for (int i = 0; i < TIM_PSC_DIV; ++i)
            timer_set_oc_value(PWM_TIM, TIM_OC1, i);

        vTaskDelay(1);
    }
}

void usart1_isr(void) {
    do {
        if (usart_get_flag(U_PORT, USART_FLAG_RXNE)) {
            recv_buf[recv_ndx_nxt] = usart_recv(U_PORT);

            /* Check for "overrun" */
            uint16_t i = (recv_ndx_nxt + 1) % RECV_BUF_SIZE;
            if (i != recv_ndx_cur) {
                recv_ndx_nxt = i;
            }
        }
    } while (usart_get_flag(U_PORT, USART_FLAG_RXNE)); /* can read back-to-back interrupts */
}

static char usart_getc(bool wait) {
    char c = 0;

    while (wait && recv_ndx_cur == recv_ndx_nxt);
    if (recv_ndx_cur != recv_ndx_nxt) {
        c = recv_buf[recv_ndx_cur];
        recv_ndx_cur = (recv_ndx_cur + 1) % RECV_BUF_SIZE;
    }
    return c;
}

static void usart_setup(void) {
    static_assert(U_PORT == USART1, "U_PORT isn't USART1!");
    /* Setup GPIO pins for USART transmit. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO9 | GPIO10);

    /* Setup USART parameters. */
    usart_set_baudrate(U_PORT, 115200);
    usart_set_databits(U_PORT, 8);
    usart_set_parity(U_PORT, USART_PARITY_NONE);
    usart_set_stopbits(U_PORT, USART_CR2_STOPBITS_1);
    usart_set_mode(U_PORT, USART_MODE_TX_RX);
    usart_set_flow_control(U_PORT, USART_FLOWCONTROL_NONE);

    /* Enable interrupts from the USART */
    nvic_enable_irq(NVIC_USART1_IRQ);

    /* Specifically enable recieve interrupts */
    usart_enable_rx_interrupt(U_PORT);

    /* Finally enable the USART. */
    usart_enable(U_PORT);
}

_Noreturn static void usart_task(void *args) {
    while (1) {
        vTaskDelay(20 / portTICK_PERIOD_MS);
        char ch = usart_getc(false);
        if (ch == '#') { //#101
            char buf[5];
            char *ptr = buf;

            *(ptr++) = usart_getc(1);
            *(ptr++) = ' ';
            *(ptr++) = usart_getc(1);
            *(ptr++) = usart_getc(1);
            *(ptr++) = 0;

            char *end;
            cdp.force_gen = strtol(buf, &end, 10);
            cdp.hash = strtol(end, &end, 10);

            if (cdp_check_hash(&cdp)) {
                rdp.force_gen = cdp.force_gen;
                if (cdp.force_gen) {
                    timer_enable_counter(PWM_TIM);
                    vTaskResume(saw_task_hdl);
                } else {
                    vTaskSuspend(saw_task_hdl);
                    timer_set_oc_value(PWM_TIM, TIM_OC1, TIM_PSC_DIV);
                    timer_disable_counter(PWM_TIM);
                }
            }
        }
    }
}

int main(void) {
    rcc_clock_setup_in_hse_8mhz_out_48mhz();

    adc_setup();
    pwm_setup();
    usart_setup();

//    {
//        char *str = "Con!\n";
//        size_t len = strlen(str);
//        while (len--)
//            usart_send_blocking(U_PORT, *str++);
//    }

    if (xTaskCreate(&usart_task, "u", 127, NULL, 8, NULL) != pdPASS) {
        configASSERT(0);
    }
    if (xTaskCreate(&adc_analyze_task, "a", 127, NULL, 8, NULL) != pdPASS) {
        configASSERT(0);
    }
    if (xTaskCreate(&saw_task, "s", 127, NULL, 8, &saw_task_hdl) != pdPASS) {
        configASSERT(0);
    }
    vTaskSuspend(saw_task_hdl);

    vTaskStartScheduler();

    while (1) {}
    return 0;
}