#include <inttypes.h>

void spin(volatile uint32_t count)
{
    while (count--)
        (void)0;
}

struct gpio
{
    volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
};

#define GPIO_MODE_INPUT 0
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_ALTERNATE 2
#define GPIO_MODE_ANALOG 3

#define GPIO_BASE_ADDRESS (0x40020000)
#define GPIO_PORT(port_num) ((struct gpio *)(GPIO_BASE_ADDRESS + (0x400 * port_num)))

int gpio_set_mode(struct gpio *port, uint8_t mode, uint8_t pin)
{
    port->MODER = (port->MODER & (~(0b11U << (uint32_t)pin * 2U))) | ((uint32_t)mode << (uint32_t)pin * 2U);
    return 0;
}

int gpio_set_alternate_function(struct gpio *port, uint8_t function, uint8_t pin)
{
    port->AFR[pin / 8] = (port->AFR[pin / 8] & ~(0b1111U << ((uint32_t)pin & 7U) * 4U)) | ((uint32_t)function << ((uint32_t)pin & 7U) * 4U);
    return 0;
}

int gpio_write(struct gpio *port, uint8_t pin, uint8_t state)
{
    port->BSRR = (1 << (pin + (1 - state) * 16));
    return 0;
}

struct rcc
{
    volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, RESERVED0[2], APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, RESERVED2[2], APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR, RESERVED4[2], APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR, RESERVED6[2], SSCGR, PLLI2SCFGR, RESERVED7, DCKCFGR;
};

#define RCC ((struct rcc *)0x40023800)

int rcc_enable_gpio(struct rcc *r)
{
    r->AHB1ENR = r->AHB1ENR | 0b10011111;
    return 0;
}

int rcc_enable_usart1(struct rcc *r)
{
    r->AHB2ENR = r->AHB2ENR | 0b10000;
    return 0;
}

int rcc_enable_usart2(struct rcc *r)
{
    r->APB1ENR = r->APB1ENR | (1 << 17);
    return 0;
}

struct systick
{
    volatile uint32_t CTRL, LOAD, VAL, CALIB;
};
#define SYSTICK ((struct systick *)0xe000e010)

struct usart
{
    volatile uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
};
#define USART1 ((struct usart *)(0x40011000))
#define USART2 ((struct usart *)(0x40004400))
#define FREQ 16000000
int usart_set_baud_rate(struct usart *uart, unsigned long baud)
{
    uart->BRR = FREQ / baud;
    return 0;
}
int usart1_init()
{
    rcc_enable_usart1(RCC);
    struct gpio *portB = GPIO_PORT(1);
    gpio_set_mode(portB, GPIO_MODE_ALTERNATE, 6);
    gpio_set_alternate_function(portB, 7, 6); // Set TX
    gpio_set_mode(portB, GPIO_MODE_ALTERNATE, 7);
    gpio_set_alternate_function(portB, 7, 7); // Set RX

    struct usart *usart1 = USART1;
    usart_set_baud_rate(usart1, 115200);
    usart1->CR1 = 0;
    usart1->CR1 = 0b1100 | (1 << 13);

    return 0;
}

int usart2_init()
{
    rcc_enable_usart2(RCC);
    struct gpio *portA = GPIO_PORT(0);
    gpio_set_mode(portA, GPIO_MODE_ALTERNATE, 2);
    gpio_set_alternate_function(portA, 7, 2); // Set TX
    gpio_set_mode(portA, GPIO_MODE_ALTERNATE, 3);
    gpio_set_alternate_function(portA, 7, 3); // Set RX

    struct usart *usart2 = USART2;
    usart_set_baud_rate(usart2, 115200);
    usart2->CR1 = 0;
    usart2->CR1 = 0b1100 | (1 << 13);

    return 0;
}

int uart_read_ready(struct usart *uart)
{
    return uart->SR & (1 << 5);
}

uint8_t uart_read_byte(struct usart *uart)
{
    return (uint8_t)(uart->DR);
}

int uart_write_byte(struct usart *uart, uint8_t byte)
{
    uart->DR = byte;
    while ((uart->SR & (1 << 7)) == 0)
    {
        spin(1);
    }
    return 0;
}

int uart_write_buf(struct usart *uart, uint8_t *buf, uint32_t len)
{
    while (len--)
    {
        uart_write_byte(uart, *(buf++));
    }
    return 0;
}

int main(void)
{
    int err = 0;
    /*

    struct rcc *r = RCC;
    err = rcc_enable_gpio(r);

    usart2_init();
    for (uint32_t i = 0; i < 1000; ++i)
    {
        uart_write_buf(USART2, (uint8_t *)"chicken\r\n", 10);
        spin(100);
    }
    */

    struct rcc *r = RCC;
    err = rcc_enable_gpio(r);

    gpio_set_mode(GPIO_PORT(0), GPIO_MODE_OUTPUT, 5);
    gpio_set_mode(GPIO_PORT(1), GPIO_MODE_OUTPUT, 13);

    for (;;)
    {
        gpio_write(GPIO_PORT(1), 13, 1);
        gpio_write(GPIO_PORT(0), 5, 1);
        spin(999999);
        gpio_write(GPIO_PORT(1), 13, 0);
        gpio_write(GPIO_PORT(0), 5, 0);
        spin(999999);
    }

    for (;;)
        (void)0;
    return err;
}

// Startup code
__attribute__((naked, noreturn)) void _reset(void)
{
    // memset .bss to zero, and copy .data section to RAM region
    extern long _sbss, _ebss, _sdata, _edata, _sidata;
    for (long *dst = &_sbss; dst < &_ebss; dst++)
        *dst = 0;
    for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;)
        *dst++ = *src++;

    main(); // Call main()
    for (;;)
        (void)0; // Infinite loop in the case if main() returns
}