#include <inttypes.h>
#include <stdint.h>

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
    port->BSRR = (1 << ((uint32_t)pin + (1U - (uint32_t)state) * 16U));
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

int morse_init()
{
    RCC->AHB1ENR = RCC->AHB1ENR | 1;
    return gpio_set_mode(GPIO_PORT(0), GPIO_MODE_OUTPUT, 5);
}

#define MORSE_DOT 0
#define MORSE_DASH 1
#define MORSE_TERMINATOR 2

int morse_char(char letter)
{
    static const uint8_t morse_table[][4] = {
        {MORSE_DOT, MORSE_DASH, MORSE_TERMINATOR, MORSE_TERMINATOR},        // A
        {MORSE_DASH, MORSE_DOT, MORSE_DOT, MORSE_DOT},                      // B
        {MORSE_DASH, MORSE_DOT, MORSE_DASH, MORSE_DOT},                     // C
        {MORSE_DASH, MORSE_DOT, MORSE_DOT, MORSE_TERMINATOR},               // D
        {MORSE_DOT, MORSE_TERMINATOR, MORSE_TERMINATOR, MORSE_TERMINATOR},  // E
        {MORSE_DOT, MORSE_DOT, MORSE_DASH, MORSE_DOT},                      // F
        {MORSE_DASH, MORSE_DASH, MORSE_DOT, MORSE_TERMINATOR},              // G
        {MORSE_DOT, MORSE_DOT, MORSE_DOT, MORSE_DOT},                       // H
        {MORSE_DOT, MORSE_DOT, MORSE_TERMINATOR, MORSE_TERMINATOR},         // I
        {MORSE_DOT, MORSE_DASH, MORSE_DASH, MORSE_DASH},                    // J
        {MORSE_DASH, MORSE_DOT, MORSE_DASH, MORSE_TERMINATOR},              // K
        {MORSE_DOT, MORSE_DASH, MORSE_DOT, MORSE_DOT},                      // L
        {MORSE_DASH, MORSE_DASH, MORSE_TERMINATOR, MORSE_TERMINATOR},       // M
        {MORSE_DASH, MORSE_DOT, MORSE_TERMINATOR, MORSE_TERMINATOR},        // N
        {MORSE_DASH, MORSE_DASH, MORSE_DASH, MORSE_TERMINATOR},             // O
        {MORSE_DOT, MORSE_DASH, MORSE_DASH, MORSE_DOT},                     // P
        {MORSE_DASH, MORSE_DASH, MORSE_DOT, MORSE_DASH},                    // Q
        {MORSE_DOT, MORSE_DASH, MORSE_DOT, MORSE_TERMINATOR},               // R
        {MORSE_DOT, MORSE_DOT, MORSE_DOT, MORSE_TERMINATOR},                // S
        {MORSE_DASH, MORSE_TERMINATOR, MORSE_TERMINATOR, MORSE_TERMINATOR}, // T
        {MORSE_DOT, MORSE_DOT, MORSE_DASH, MORSE_TERMINATOR},               // U
        {MORSE_DOT, MORSE_DOT, MORSE_DOT, MORSE_DASH},                      // V
        {MORSE_DOT, MORSE_DASH, MORSE_DASH, MORSE_TERMINATOR},              // W
        {MORSE_DASH, MORSE_DOT, MORSE_DOT, MORSE_DASH},                     // X
        {MORSE_DASH, MORSE_DOT, MORSE_DASH, MORSE_DASH},                    // Y
        {MORSE_DASH, MORSE_DASH, MORSE_DOT, MORSE_DOT}                      // Z
    };

    if (letter > 'Z')
    {
        letter = letter - 'a';
    }
    else
    {
        letter = letter - 'A';
    }

    uint8_t index = 0;
    while (morse_table[(uint8_t)letter][index++] != MORSE_TERMINATOR)
    {
        gpio_write(GPIO_PORT(0), 5, 1);
        spin(morse_table[(uint8_t)letter][index] * 50000U * 3U + 50000U * 3U);
        gpio_write(GPIO_PORT(0), 5, 0);
    }
    return 0;
}

int morse_word(char *message)
{
    while (*(message))
    {
        morse_char(*(message++));
        spin(300000 * 3);
    }
    return 0;
}

int main(void)
{
    int err = 0;
    err = morse_init();

    // Infinite loop
    for (;;)
    {
        err = morse_word("SOS");
        (void)0;
    }

    return err;
}

extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _estack;

// Entry
__attribute__((naked, noreturn)) void _reset(void)
{

    /* Copy Data from FLASH to SRAM */
    uint32_t *pSRC = (uint32_t *)&_sidata;
    uint32_t *pDST = (uint32_t *)&_sdata;

    for (uint32_t *dataptr = (uint32_t *)pDST; dataptr < &_edata;)
    {

        *dataptr++ = *pSRC++;
    }
    /** Initialize BSS with 0 **/
    for (uint32_t *bss_ptr = (uint32_t *)&_sbss; bss_ptr < &_ebss;)
    {
        *bss_ptr++ = 0;
    }
    main();
}

__attribute__((section(".isr_vector"))) void (*const fpn_vector[])(void) = {
    (void (*)(void))(&_estack),
    _reset};
