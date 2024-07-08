/*
Simple program to demonstrate communicating with a BMI270 over SPI on an MSP430.
This example uses the MSP-EXP430FR6989 Launchpad and its EUSCI_B0 SPI interface,
which has the following pinout:

P1.4: UCB0CLK (serial clock)
P1.5: CSB (chip select bar)
P1.6: UCB0SIMO (peripheral in, controller out)
P1.7: UCB0SOMI (peripheral out, controller in)
*/

#include <stdio.h>
#include <driverlib.h>
#include "BMI270_SensorAPI/bmi270.h"

#define SPI_BASE EUSCI_B0_BASE

volatile static const uint8_t* tx_data;
volatile static uint32_t tx_len;
volatile static uint32_t tx_count;
volatile static uint8_t* rx_data;
volatile static uint32_t rx_len;
volatile static uint32_t rx_count;

static uint32_t mclk_uhz;

enum RwState { NONE, TRANSMITTING, RECEIVING };
volatile static enum RwState rw_state = NONE;

/* Delay a specified number of microseconds -- function to be passed to the BMI270 library */
void bmi2_delay_us(uint32_t period, void* intf_ptr) {
    uint32_t i = period * mclk_uhz;
    while (i) {
        __delay_cycles(1);
        i -= 1;
    }
}

/* Read len bytes from the device at its register reg_addr into reg_data --
function to be passed to the BMI270 library */
BMI2_INTF_RETURN_TYPE bmi2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    rx_data = reg_data;
    rx_len = len;
    rx_count = 0;
    rw_state = RECEIVING;

    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);    // Set CSB low to indicate transmission
    EUSCI_B_SPI_clearInterrupt(SPI_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT);
    EUSCI_B_SPI_enableInterrupt(SPI_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT | EUSCI_B_SPI_TRANSMIT_INTERRUPT);
    EUSCI_B_SPI_transmitData(SPI_BASE, 0x80 | reg_addr);    // MSB=1 indicates a read to the device

    // Enter LPM0, with interrupts enabled, and wait for transmit interrupt
    __bis_SR_register(LPM0_bits + GIE);

    EUSCI_B_SPI_disableInterrupt(SPI_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT | EUSCI_B_SPI_TRANSMIT_INTERRUPT);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);   // Set CSB high to indicate end of transmission
    return 0;
}

/* Write len bytes from reg_data into the device at its register reg_addr --
function to be passed to the BMI270 library */
BMI2_INTF_RETURN_TYPE bmi2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    tx_data = reg_data;
    tx_len = len;
    tx_count = 0;
    rw_state = TRANSMITTING;

    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);    // Set CSB low to indicate transmission
    EUSCI_B_SPI_clearInterrupt(SPI_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT);
    EUSCI_B_SPI_enableInterrupt(SPI_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT);
    EUSCI_B_SPI_transmitData(SPI_BASE, reg_addr);

    // Enter LPM0, with interrupts enabled, and wait for transmit interrupt
    __bis_SR_register(LPM0_bits + GIE);

    EUSCI_B_SPI_disableInterrupt(SPI_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);   // Set CSB high to indicate end of transmission
    return 0;
}

void init_spi() {
    // Set pins P1.6 and P1.4 as UCB0SIMO and UCB0CLK respectively
    GPIO_setAsPeripheralModuleFunctionOutputPin(
        GPIO_PORT_P1,
        GPIO_PIN6 + GPIO_PIN4,
        GPIO_PRIMARY_MODULE_FUNCTION
    );

    // Set pin P1.7 as UCB0SOMI
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P1,
        GPIO_PIN7,
        GPIO_PRIMARY_MODULE_FUNCTION
    );

    // While it is possible to set this as an SPI chip select pin (UCB0STE), it should instead
    // be set just as a normal GPIO output, so that it doesn't get driven low after every write.
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
    __delay_cycles(100);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();

    EUSCI_B_SPI_initMasterParam param = {
        .selectClockSource = EUSCI_B_SPI_CLOCKSOURCE_SMCLK,
        .clockSourceFrequency = CS_getSMCLK(),
        .desiredSpiClock = 1000000,
        // Per the datasheet, the BMI270 supports either 00 (the current setting) or 11 for clockPhase and clockPolarity.
        // This is automatically detected by the BMI270.
        .clockPhase = EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,
        .clockPolarity = EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW,
        .msbFirst = EUSCI_B_SPI_MSB_FIRST,
        .spiMode = EUSCI_B_SPI_4PIN_UCxSTE_ACTIVE_LOW
    };
    EUSCI_B_SPI_initMaster(SPI_BASE, &param);
    // Honestly I have no idea what this next line does, it might do nothing
    EUSCI_B_SPI_select4PinFunctionality(SPI_BASE, EUSCI_B_SPI_ENABLE_SIGNAL_FOR_4WIRE_SLAVE);
    EUSCI_B_SPI_enable(SPI_BASE);
}

void init_bmi_device(struct bmi2_dev* bmi) {
    bmi->intf = BMI2_SPI_INTF;

    bmi->read = bmi2_spi_read;
    bmi->write = bmi2_spi_write;
    bmi->delay_us = bmi2_delay_us;

    // i am not using this, you can if you want
    bmi->intf_ptr = NULL;

    // Configure max read/write length (in bytes) ( Supported length depends on target machine)
    bmi->read_write_len = 46;

    // Assign to NULL to load the default config file.
    bmi->config_file_ptr = NULL;
}

void init_clk() {
    // Set DCO Frequency to 4 MHz
    CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_3);

    // Configure MCLK, SMCLK to be sourced by DCOCLK
    CS_initClockSignal(CS_MCLK,  CS_DCOCLK_SELECT,  CS_CLOCK_DIVIDER_1); // 4 MHz
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT,  CS_CLOCK_DIVIDER_1); // 4 MHz

    // Store MCLK frequency in uHz for delay calculation
    mclk_uhz = CS_getMCLK() / 1000000;
}

int main(void) {
    struct bmi2_dev bmi;

    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    init_clk();
    init_spi();
    init_bmi_device(&bmi);

    printf("test\n");

    int result = bmi270_init(&bmi);
    printf("bmi270_init result: %d\n", result);
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_B0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_B0_VECTOR)))
#endif
void USCI_B0_ISR (void)
{
    switch (__even_in_range(UCB0IV, USCI_SPI_UCTXIFG))
    {
        case USCI_NONE: break;
        case USCI_SPI_UCRXIFG:
            switch (rw_state) {
                case TRANSMITTING:
                    // this shouldn't happen, but if it does it's probably ok to ignore it
                    break;
                case RECEIVING:
                    // receive the next byte in the sequence
                    rx_data[rx_count] = EUSCI_B_SPI_receiveData(SPI_BASE);
                    rx_count += 1;
                    if (rx_count == rx_len) {
                        // we're done receiving data
                        rw_state = NONE;
                        __bic_SR_register_on_exit(LPM0_bits); // leave low power mode
                    }
                    break;
                default:
                    break;
            }
            break;
        case USCI_SPI_UCTXIFG:
            switch (rw_state) {
                case TRANSMITTING:
                    // transmit the next byte in the sequence
                    EUSCI_B_SPI_transmitData(SPI_BASE, tx_data[tx_count]);
                    tx_count += 1;
                    if (tx_count == tx_len) {
                        // we're done transmitting data
                        rw_state = NONE;
                        __bic_SR_register_on_exit(LPM0_bits); // leave low power mode
                    }
                    break;
                case RECEIVING:
                    // for every byte received, we need to transmit a dummy byte to get
                    // the clock going for 8 cycles
                    // the BMI270 will transfer data to us during those cycles, and then the UCRXIFG
                    // interrupt will fire and we can read it
                    EUSCI_B_SPI_transmitData(SPI_BASE, 0);
                    break;
                default:
                    break;
            }
            break;
        default: break;
  }
}
