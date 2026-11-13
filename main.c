//header files stdint --- give s speacial integer datatypes uint etc  and stdbool allows to use true and false
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/eeprom.h"




#define UART_BAUD           (115200U)
#define TICK_MS             (100U)                     /* main loop period      */
#define TICKS_PER_SEC       (1000U / TICK_MS)
#define LOG_PERIOD_S        (10U)                      /* EEPROM write interval */

#define ADC_SEQ             (3U)         /*sequencer 3=1 step meaning it only takes one input which is sufficient*/
#define ADC_OVERSAMPLE      (64U)

#define EEPROM_BYTES        (2048U)
#define REC_WORDS           (2U)  // two words i.e the index and the temperature
#define REC_BYTES           (8U)
#define REC_COUNT           (EEPROM_BYTES / REC_BYTES) // 256 slots for recording the data
#define EMPTY_WORD          (0xFFFFFFFFU)


// the following variables that are most important and are being used by multiple variables hence declared globally
static uint32_t g_clock_hz;
static uint32_t g_next_slot;   //slot the next record goes int
static uint32_t g_next_seq;    //sequence number of next record
static uint32_t g_stored;      //number of valid records in EEPROM


// UART helping functions
static void uart_puts(const char *s) // function used to transmit the string on the uart
{
    uint32_t i = 0U;

    while (s[i] != '\0')
    {
        UARTCharPut(UART0_BASE, s[i]);
        i++;
    }
}

static void uart_put_uint(uint32_t value)
{
    char     buf[10];
    uint32_t i = 0U;
    uint32_t v = value;

    do
    {
        buf[i] = (char)((uint32_t)'0' + (v % 10U)); //extracts the last digit and the value is added to the ascii value of zero so that it is stored as a character and prints as a character
        i++;
        v /= 10U; // removes the last digit
    } while (v != 0U);

    while (i > 0U)
    {
        i--;
        UARTCharPut(UART0_BASE, buf[i]);
    }
}

static void uart_put_temp(int32_t t_x10)
{
    uint32_t mag;

    if (t_x10 < 0)
    {
        UARTCharPut(UART0_BASE, '-'); // if negative then just put a negative sign in the begining
        mag = (uint32_t)(-t_x10);
    }
    else
    {
        mag = (uint32_t)t_x10;
    }

    uart_put_uint(mag / 10U); // each and every character transmitter through uart
    UARTCharPut(UART0_BASE, '.');
    uart_put_uint(mag % 10U);
    uart_puts(" C");
}


// Peripheral initialisation

static void uart_init(void) // setting up the uart  so that it gets initialized
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)) { }
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) { }

    /* PA0 = U0RX, PA1 = U0TX -> routed to the ICDI virtual COM port */
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, g_clock_hz, UART_BAUD,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
}

static void adc_init(void) // adc initialization
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) { }


    ADCHardwareOversampleConfigure(ADC0_BASE, ADC_OVERSAMPLE); //doing oversampling x64 so as to reduce noise and it takes avergage of 64 readings


    ADCSequenceConfigure(ADC0_BASE, ADC_SEQ, ADC_TRIGGER_PROCESSOR, 0U);
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQ, 0U,
                             (ADC_CTL_TS | ADC_CTL_IE | ADC_CTL_END));
    ADCSequenceEnable(ADC0_BASE, ADC_SEQ);  // ADC is sequenceer 3 is enabled because only 1 step is needed
    ADCIntClear(ADC0_BASE, ADC_SEQ);
}

static bool eeprom_init(void) // eeprom initialisation
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0)) { }

    // EEPROMInit must be called after enabling;
    return (EEPROMInit() == EEPROM_INIT_OK); // checking if the eeprom is initialised fine and free from any interrupted writes
}

//Temperature measurement
static int32_t temp_read_x10(void)
{
    uint32_t raw = 0U;

    ADCIntClear(ADC0_BASE, ADC_SEQ);  // clears the adc completion flag
    ADCProcessorTrigger(ADC0_BASE, ADC_SEQ);

    while (ADCIntStatus(ADC0_BASE, ADC_SEQ, false) == 0U) { } //waits for adc to finish --- POLLING   we could use the ADC interrupt service routines to handle the completion

    (void)ADCSequenceDataGet(ADC0_BASE, ADC_SEQ, &raw);

    /* Datasheet: TEMP = 147.5 - (75 * (VREFP - VREFN) * ADCCODE) / 4096
       With VREFP - VREFN = 3.3 V and scaled by 10 for integer maths:
       TEMP_x10 = 1475 - (2475 * ADCCODE) / 4096                        */
    return 1475 - (int32_t)((2475U * raw) / 4096U);
}


// EEPROM logging

static void log_scan(void)
{
    uint32_t slot;  // examines which eeprom slot is being examined
    uint32_t rec[REC_WORDS];  // array for reading eeprom record  (2 element of 32 bits each)
    uint32_t max_seq = 0U;    // stores the highest sequence number encountered during the scan
    bool     found   = false;

    g_next_slot = 0U;  // the next record in which the storage would take place
    g_stored    = 0U;  // number of stored records

    for (slot = 0U; slot < REC_COUNT; slot++)     // scanning all 256 slots ( REC_COUNT =256 iyk)
    {
        EEPROMRead(rec, slot * REC_BYTES, REC_BYTES);

        if (rec[0] != EMPTY_WORD) // checking if the record is valid according to the empty-slot convention (if not this value then the slot is occupied)
        {
            g_stored++;
            if ((!found) || (rec[0] > max_seq))
            {
                max_seq     = rec[0];
                g_next_slot = (slot + 1U) % REC_COUNT;
                found       = true;
            }
        }
    }

    g_next_seq = found ? (max_seq + 1U) : 0U;  // max sequence value incremented
}

static bool log_write(int32_t temp_x10)
{
    uint32_t rec[REC_WORDS];
    uint32_t status;

    rec[0] = g_next_seq;
    rec[1] = (uint32_t)temp_x10;

    status = EEPROMProgram(rec, g_next_slot * REC_BYTES, REC_BYTES);

    if (status == 0U) // if status is zero then it means succesfull writing
    {
        g_next_seq++;
        g_next_slot = (g_next_slot + 1U) % REC_COUNT;// like circular buffer
        if (g_stored < REC_COUNT) // this is just for satisfying the ending index problem
        {
            g_stored++;
        }
    }

    return (status == 0U);
}

static void log_dump(void)
{
    uint32_t i;
    uint32_t slot;
    uint32_t rec[REC_WORDS];

    uart_puts("\r\n--- EEPROM log (oldest first) ---\r\n");

    /* Start just after the newest record so output is chronological */
    for (i = 0U; i < REC_COUNT; i++)
    {
        slot = (g_next_slot + i) % REC_COUNT;
        EEPROMRead(rec, slot * REC_BYTES, REC_BYTES);

        if (rec[0] != EMPTY_WORD)
        {
            uart_puts("#");
            uart_put_uint(rec[0]);
            uart_puts("  ");
            uart_put_temp((int32_t)rec[1]);
            uart_puts("\r\n");
        }
    }

    uart_puts("--- ");
    uart_put_uint(g_stored);
    uart_puts(" record(s) ---\r\n");
}

static void log_erase(void)
{
    if (EEPROMMassErase() == 0U)
    {
        g_next_slot = 0U;
        g_next_seq  = 0U;
        g_stored    = 0U;
        uart_puts("\r\nLog erased.\r\n");
    }
    else
    {
        uart_puts("\r\nERROR: erase failed.\r\n");
    }
}

/* ------------------------------------------------------------------ */
/* Misc                                                               */
/* ------------------------------------------------------------------ */
static void delay_ms(uint32_t ms)
{
    /* SysCtlDelay burns 3 CPU cycles per loop count */
    SysCtlDelay((g_clock_hz / 3000U) * ms);
}

static void print_help(void)
{
    uart_puts("Commands: d = dump log, e = erase log, h = help\r\n");
}

static void handle_command(int32_t ch)
{
    switch (ch)
    {
        case 'd':
        case 'D':
            log_dump();
            break;
        case 'e':
        case 'E':
            log_erase();
            break;
        case 'h':
        case 'H':
            print_help();
            break;
        default:
            /* ignore other keys */
            break;
    }
}

/* ------------------------------------------------------------------ */
/* main                                                               */
/* ------------------------------------------------------------------ */
int main(void)
{
    uint32_t ticks = 0U;
    int32_t  temp_x10;

    /* 16 MHz crystal -> PLL (200 MHz) / 5 = 40 MHz system clock */
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL |
                   SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    g_clock_hz = SysCtlClockGet();

    uart_init();
    adc_init();

    uart_puts("\r\n=== Tiva C Temperature Logger ===\r\n");

    if (!eeprom_init())
    {
        uart_puts("ERROR: EEPROM init failed. Halting.\r\n");
        for (;;) { }
    }

    log_scan();
    uart_puts("Records found in EEPROM: ");
    uart_put_uint(g_stored);
    uart_puts("\r\n");
    print_help();

    for (;;)
    {
        if (UARTCharsAvail(UART0_BASE))
        {
            handle_command(UARTCharGetNonBlocking(UART0_BASE));
        }

        if ((ticks % TICKS_PER_SEC) == 0U)
        {
            temp_x10 = temp_read_x10();
            uart_puts("Live: ");
            uart_put_temp(temp_x10);

            if ((ticks % (TICKS_PER_SEC * LOG_PERIOD_S)) == 0U)
            {
                if (log_write(temp_x10))
                {
                    uart_puts("  [logged #");
                    uart_put_uint(g_next_seq - 1U);
                    uart_puts("]");
                }
                else
                {
                    uart_puts("  [EEPROM write FAILED]");
                }
            }
            uart_puts("\r\n");
        }

        delay_ms(TICK_MS);
        ticks++;
    }
}
