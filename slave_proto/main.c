// C standard library
#include <stdint.h>
#include <stdbool.h>

// necessary for initialization
#include "MIMX8ML8_cm7.h"
#include "core_cm7.h"
#include "mpu_armv7.h"
#include "fsl_device_registers.h"
#include "fsl_common.h"
#include "fsl_rdc.h"
#include "fsl_clock.h"
#include "fsl_iomuxc.h"

#define BOARD_NAME        "DART-MX8M-PLUS"
#define MANUFACTURER_NAME "Variscite"
#define BOARD_DOMAIN_ID   (1U)

void init_memory();
void init_rdc();
void init_pinmux();
void run_clocks();

#define TIMEOUT_US 1200U // 1.2ms timeout
#define MAX_DIFFERING_READS 3U // 3 differing reads until fail
#define FAULT_RESET_TIME_US 1000000U // 1s target reset time
uint32_t loop_time_us = 0U;
uint32_t differing_reads = 0U;
bool in_fault_state = false;
uint32_t last_fault_time_us = 0U;
void gpt1_init();
bool gpt1_timeout_reached(uint32_t loop_start);

#define IN1_GPIO_IN GPIO3, 19U
#define OUT1_GPIO_OUT GPIO4, 1U

uint32_t gpio_read(GPIO_Type *base, uint32_t pin);
void gpio_write(GPIO_Type *base, uint32_t pin, uint8_t output);
void gpio_init(GPIO_Type *base, uint32_t pin, bool is_output, uint8_t output_initial_level);
void gpio_init_all();

#define SPI_BURST_WORDS 3U
#define SPI_BURST_LENGTH (SPI_BURST_WORDS * 32U)
void spi2_slave_init();
void spi2_clear_buffers();
bool spi2_slave_start_transfer(uint32_t tx_buf[SPI_BURST_WORDS], bool check_timeout, uint32_t loop_start);
bool spi2_slave_finish_transfer(uint32_t rx_buf[SPI_BURST_WORDS], bool check_timeout, uint32_t loop_start);
bool spi2_slave_transfer_blocking(uint32_t tx_buf[SPI_BURST_WORDS], uint32_t rx_buf[SPI_BURST_WORDS], bool check_timeout, uint32_t loop_start);

// fatal state
void enter_fatal_state() {
    gpio_write(OUT1_GPIO_OUT, 0);
    while (true) { }
}

// fault state
void enter_fault_state() {
    gpio_write(OUT1_GPIO_OUT, 0);
    in_fault_state = true;
    last_fault_time_us = GPT1->CNT;
}

// reset fault state
void reset_fault_state() {
    gpio_write(OUT1_GPIO_OUT, 1);
    in_fault_state = false;
}

int main() {
    // initialization
    init_memory();
    init_rdc();
    init_pinmux();
    run_clocks();

    gpio_init_all();
    spi2_slave_init();

    uint32_t tx_buf[SPI_BURST_WORDS] = {};
    uint32_t rx_buf[SPI_BURST_WORDS] = {};

    // pong
    bool got_ping = false;
    while (!got_ping) {
        tx_buf[0] = 0xAC;
        tx_buf[1] = 0xDC;
        tx_buf[2] = 0xBE;
        spi2_slave_transfer_blocking(tx_buf, rx_buf, false, 0);
        if (rx_buf[0] == 0xAC && rx_buf[1] == 0xDC && rx_buf[2] == 0xBE) {
            got_ping = true;
        }
        else {
            SDK_DelayAtLeastUs(100000, DEFAULT_SYSTEM_CLOCK); // delay 100ms if ping not received correctly
        }
    }

    gpt1_init();

    gpio_write(OUT1_GPIO_OUT, 1);

    uint32_t last_in = 0;
    uint32_t had_fault = 0;

    while (1) {
        uint32_t loop_start = GPT1->CNT;
        bool success = true;

        // start SPI transfer
        tx_buf[0] = 0xB8;
        tx_buf[1] = last_in;
        tx_buf[2] = had_fault;
        // blocks until master is ready to send or timeout is reached
        success = spi2_slave_start_transfer(tx_buf, true, loop_start);
        if (!success) {
            enter_fault_state();
        }
        else {
            // on time, continue
        }

        // measure input
        uint32_t in = gpio_read(IN1_GPIO_IN);

        // finish SPI transfer
        success = spi2_slave_finish_transfer(rx_buf, true, loop_start);
        if (!success) {
            enter_fault_state();
        }
        else {
            // on time, continue
        }
        uint32_t master_crc = rx_buf[0];
        uint32_t master_last_in = rx_buf[1];
        uint32_t master_had_fault = rx_buf[2];

        // evaluate
        if (had_fault == 1 || had_fault != master_had_fault) {
            // if last eval was 0 or the two processors don't agree, fault state
            enter_fault_state();
        }
        else {
            reset_fault_state();
        }

        if (differing_reads > MAX_DIFFERING_READS) {
            // if 3 input reads differ, enter fault state
            had_fault = 1;
        }
        else if ((master_crc != 0xB8) || (last_in != master_last_in)) {
            // if input reads differ, add to differing reads
            ++differing_reads;
        }
        else if (differing_reads != 0) {
            // reset differing loops if the input reads do not differ
            differing_reads = 0;
        }
        else {
            // do nothing
        }

        last_in = in;

        // check timing
        if (gpt1_timeout_reached(loop_start)) {
            // loop took too long
            enter_fault_state();
        }
        else {
            // continue
        }

        if (in_fault_state) {
            spi2_clear_buffers();
            while ((GPT1->CNT - last_fault_time_us) < FAULT_RESET_TIME_US) {
                // wait for the fault reset time to run out
            }

            got_ping = false;
            while (!got_ping) {
                spi2_clear_buffers();
                // pong to establish communication again
                tx_buf[0] = 0xAC;
                tx_buf[1] = 0xDC;
                tx_buf[2] = 0xBE;
                spi2_slave_transfer_blocking(tx_buf, rx_buf, false, 0);
                if (rx_buf[0] == 0xAC && rx_buf[1] == 0xDC && rx_buf[2] == 0xBE) {
                    got_ping = true;
                }
                else {
                    SDK_DelayAtLeastUs(100000, DEFAULT_SYSTEM_CLOCK); // delay 100ms if ping not received correctly
                }
            }

            // reset fault state when communication established again
            last_in = 0;
            had_fault = 0;
            differing_reads = 0;
        }
    }
}

void gpt1_init() {
    CLOCK_SetRootMux(kCLOCK_RootGpt1, kCLOCK_GptRootmuxOsc24M);
    CLOCK_SetRootDivider(kCLOCK_RootGpt1, 3U, 8U);
    CLOCK_EnableClock(kCLOCK_Gpt1);

    GPT1->CR = 0; // stop timer before configuration
    GPT1->PR = 0; // no prescaler
    GPT1->OCR[0] = 0xFFFFFFFF; // not used for interrupts
    GPT1->CR =
        GPT_CR_CLKSRC(1) | // peripheral clock
        GPT_CR_ENMOD_MASK | // enable on setting EN
        GPT_CR_FRR_MASK |   // free-run mode
        GPT_CR_EN_MASK;     // enable GPT
}

bool gpt1_timeout_reached(uint32_t loop_start) {
    return (GPT1->CNT - loop_start) > TIMEOUT_US;
}

uint32_t gpio_read(GPIO_Type *base, uint32_t pin) {
    assert(pin < 32U);
    return (base->DR >> pin) & 1U;
}

void gpio_write(GPIO_Type *base, uint32_t pin, uint8_t output) {
    assert(pin < 32U);
    if (output == 0U) {
        base->DR &= ~(1UL << pin);
    }
    else {
        base->DR |= (1UL << pin);
    }
}

void gpio_init(GPIO_Type *base, uint32_t pin, bool is_output, uint8_t output_initial_level) {
    base->IMR &= ~(1UL << pin);
    if (is_output) {
        gpio_write(base, pin, output_initial_level);
        base->GDIR |= (1UL << pin);
    }
    else {
        base->GDIR &= (1UL << pin);
    }
}

void gpio_init_all() {
    CLOCK_EnableClock(kCLOCK_Gpio3);
    CLOCK_EnableClock(kCLOCK_Gpio4);
    gpio_init(IN1_GPIO_IN, false, 0U); // set GPIO3 IO19 as input (safe digital input)
    gpio_init(OUT1_GPIO_OUT, true, 0U); // set GPIO4 IO01 as output (safe digital output)
}

void spi2_slave_init() {
    CLOCK_EnableClock(kCLOCK_Ecspi2);
    ECSPI2->CONREG &= ~ECSPI_CONREG_EN_MASK; // reset the spi controller
    ECSPI2->CONREG = 0U; // clear the control register
    ECSPI2->CONREG =
        ECSPI_CONREG_CHANNEL_MODE(0U) | // slave mode on channel 0
        ECSPI_CONREG_CHANNEL_SELECT(0U) | // select channel 0
        ECSPI_CONREG_BURST_LENGTH(SPI_BURST_LENGTH - 1U); // set burst length to SPI_BURST_LENGTH
    ECSPI2->CONFIGREG =
        ECSPI_CONFIGREG_SCLK_PHA(1U) | // phase 1
        ECSPI_CONFIGREG_SCLK_POL(0U) | // active high clock polarity
        ECSPI_CONFIGREG_SS_POL(0U) | // active low chip select polarity
        ECSPI_CONFIGREG_DATA_CTL(0U) | // data line stays high when inactive
        ECSPI_CONFIGREG_SCLK_CTL(0U); // clock line stays low when inactive
    ECSPI2->DMAREG |=
        ECSPI_DMAREG_TX_THRESHOLD(1U) | // threshold of data entries in TXFIFO to trigger a TX DMA/INT request
        ECSPI_DMAREG_RX_THRESHOLD(0U); // threshold of data entries in RXFIFO to trigger a RX DMA/INT request
    ECSPI2->CONREG |= ECSPI_CONREG_EN(1U); // enable SPI
}

bool spi2_slave_start_transfer(uint32_t tx_buf[SPI_BURST_WORDS], bool check_timeout, uint32_t loop_start) {
    for (uint32_t i = 0; i < SPI_BURST_WORDS; ++i) {
        ECSPI2->TXDATA = tx_buf[i];
        while ((ECSPI2->STATREG & ECSPI_STATREG_TE_MASK) == 0U) {
            // wait until txdata is empty
            if (check_timeout && gpt1_timeout_reached(loop_start)) {
                return false;
            }
            else {
                // continue if timeout not reached
            }
        }
    }
    return true;
}

bool spi2_slave_finish_transfer(uint32_t rx_buf[SPI_BURST_WORDS], bool check_timeout, uint32_t loop_start) {
    for (uint32_t i = 0; i < SPI_BURST_WORDS; ++i) {
        while ((ECSPI2->STATREG & ECSPI_STATREG_RR_MASK) == 0U) {
            // wait until receive buffer has a word
            if (check_timeout && gpt1_timeout_reached(loop_start)) {
                return false;
            }
            else {
                // continue if timeout not reached
            }
        }
        rx_buf[i] = ECSPI2->RXDATA;
    }
    return true;
}

bool spi2_slave_transfer_blocking(uint32_t tx_buf[SPI_BURST_WORDS], uint32_t rx_buf[SPI_BURST_WORDS], bool check_timeout, uint32_t loop_start) {
    if (!spi2_slave_start_transfer(tx_buf, check_timeout, loop_start)) {
        return false;
    }
    if (!spi2_slave_finish_transfer(rx_buf, check_timeout, loop_start)) {
        return false;
    }
    return true;
}

void spi2_clear_buffers() {
    while ((ECSPI2->STATREG & ECSPI_STATREG_RR_MASK) != 0U) {
        volatile uint32_t dummy = ECSPI2->RXDATA;
    }
}

void init_memory() {
    /* __CACHE_REGION_START and __CACHE_REGION_SIZE are defined in the linker file */
    extern uint32_t __CACHE_REGION_START[];
    extern uint32_t __CACHE_REGION_SIZE[];
    uint32_t cacheStart = (uint32_t)__CACHE_REGION_START;
    uint32_t size       = (uint32_t)__CACHE_REGION_SIZE;
    uint32_t i          = 0;

    /* Disable I cache and D cache */
    if (SCB_CCR_IC_Msk == (SCB_CCR_IC_Msk & SCB->CCR))
    {
        SCB_DisableICache();
    }
    if (SCB_CCR_DC_Msk == (SCB_CCR_DC_Msk & SCB->CCR))
    {
        SCB_DisableDCache();
    }

    /* Disable MPU */
    ARM_MPU_Disable();

    /* MPU configure:
     * Use ARM_MPU_RASR(DisableExec, AccessPermission, TypeExtField, IsShareable, IsCacheable, IsBufferable,
     * SubRegionDisable, Size)
     * API in mpu_armv7.h.
     * param DisableExec       Instruction access (XN) disable bit,0=instruction fetches enabled, 1=instruction fetches
     * disabled.
     * param AccessPermission  Data access permissions, allows you to configure read/write access for User and
     * Privileged mode.
     *      Use MACROS defined in mpu_armv7.h:
     * ARM_MPU_AP_NONE/ARM_MPU_AP_PRIV/ARM_MPU_AP_URO/ARM_MPU_AP_FULL/ARM_MPU_AP_PRO/ARM_MPU_AP_RO
     * Combine TypeExtField/IsShareable/IsCacheable/IsBufferable to configure MPU memory access attributes.
     *  TypeExtField  IsShareable  IsCacheable  IsBufferable   Memory Attribute    Shareability        Cache
     *     0             x           0           0             Strongly Ordered    shareable
     *     0             x           0           1              Device             shareable
     *     0             0           1           0              Normal             not shareable   Outer and inner write
     * through no write allocate
     *     0             0           1           1              Normal             not shareable   Outer and inner write
     * back no write allocate
     *     0             1           1           0              Normal             shareable       Outer and inner write
     * through no write allocate
     *     0             1           1           1              Normal             shareable       Outer and inner write
     * back no write allocate
     *     1             0           0           0              Normal             not shareable   outer and inner
     * noncache
     *     1             1           0           0              Normal             shareable       outer and inner
     * noncache
     *     1             0           1           1              Normal             not shareable   outer and inner write
     * back write/read acllocate
     *     1             1           1           1              Normal             shareable       outer and inner write
     * back write/read acllocate
     *     2             x           0           0              Device              not shareable
     *  Above are normal use settings, if your want to see more details or want to config different inner/outter cache
     * policy.
     *  please refer to Table 4-55 /4-56 in arm cortex-M7 generic user guide <dui0646b_cortex_m7_dgug.pdf>
     * param SubRegionDisable  Sub-region disable field. 0=sub-region is enabled, 1=sub-region is disabled.
     * param Size              Region size of the region to be configured. use ARM_MPU_REGION_SIZE_xxx MACRO in
     * mpu_armv7.h.
     */

    /* Region 0 [0x0000_0000 - 0x4000_0000] : Memory with Device type, not executable, not shareable, non-cacheable. */
    MPU->RBAR = ARM_MPU_RBAR(0, 0x00000000U);
    MPU->RASR = ARM_MPU_RASR(1, ARM_MPU_AP_FULL, 0, 0, 0, 1, 0, ARM_MPU_REGION_SIZE_1GB);

    /* Region 1 TCML[0x0000_0000 - 0x0001_FFFF]: Memory with Normal type, not shareable, non-cacheable */
    MPU->RBAR = ARM_MPU_RBAR(1, 0x00000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 1, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_128KB);

    /* Region 2 QSPI[0x0800_0000 - 0x0FFF_FFFF]: Memory with Normal type, not shareable, cacheable */
    MPU->RBAR = ARM_MPU_RBAR(2, 0x08000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 1, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_128MB);

    /* Region 3 TCMU[0x2000_0000 - 0x2002_0000]: Memory with Normal type, not shareable, non-cacheable */
    MPU->RBAR = ARM_MPU_RBAR(3, 0x20000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 1, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_128KB);

    /* Region 4 DDR[0x4000_0000 - 0x8000_0000]: Memory with Normal type, not shareable, non-cacheable */
    MPU->RBAR = ARM_MPU_RBAR(4, 0x40000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 1, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_1GB);

    /*
    Non-cacheable area is provided in DDR memory, the DDR region [0x80000000 ~ 0x81000000](please see the
    imx8mp-var-common-m7.dtsi) totally 16MB is revserved for CM7 core. You can put global or static uninitialized
    variables in NonCacheable section(initialized variables in NonCacheable.init section) to make them uncacheable.
    Since the base address of MPU region should be multiples of region size, to make it simple, the MPU region 5 set
    the address space 0x80000000 ~ 0xBFFFFFFF to be non-cacheable. Then MPU region 6 set the text and data section to
    be cacheable if the program running on DDR. The cacheable area base address should be multiples of its size in
    linker file, they can be modified per your needs.
    */
    /* Region 5 DDR[0x8000_0000 - 0xBFFFFFFF]: Memory with Normal type, not shareable, non-cacheable */
    MPU->RBAR = ARM_MPU_RBAR(5, 0x80000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 1, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_1GB);

    while ((size >> i) > 0x1U)
    {
        i++;
    }
    /* If run on DDR, configure text and data section to be cacheable */
    if (i != 0)
    {
        /* The MPU region size should be 2^N, 5<=N<=32, region base should be multiples of size. */
        assert((size & (size - 1)) == 0);
        assert(!(cacheStart % size));
        assert(size == (uint32_t)(1 << i));
        assert(i >= 5);

        /* Region 6 DDR[cacheStart]: Memory with Normal type, not shareable, cacheable */
        MPU->RBAR = ARM_MPU_RBAR(6, cacheStart);
        MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 1, 0, 1, 1, 0, (i - 1));
    }

    /*
     * Enable MPU and HFNMIENA feature
     * HFNMIENA ensures that M7 core uses MPU configuration when in hard fault, NMI, and FAULTMASK handlers,
     * otherwise all memory regions are accessed without MPU protection, which has high risks of cacheable,
     * especially for AIPS systems.
     */
    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_HFNMIENA_Msk);

    /* Enable I cache and D cache */
    SCB_EnableICache();
    SCB_EnableDCache();
}

void init_rdc(void)
{
    /* Move M7 core to specific RDC domain 1 */
    rdc_domain_assignment_t assignment = {0};
    uint8_t domainId                   = 0U;

    domainId = RDC_GetCurrentMasterDomainId(RDC);
    /* Only configure the RDC if RDC peripheral write access allowed. */
    if ((0x1U & RDC_GetPeriphAccessPolicy(RDC, kRDC_Periph_RDC, domainId)) != 0U)
    {
        assignment.domainId = BOARD_DOMAIN_ID;
        RDC_SetMasterDomainAssignment(RDC, kRDC_Master_M7, &assignment);
    }

    /*
     * The M7 core is running at domain 1, now enable the clock gate of the following IP/BUS/PLL in domain 1 in the CCM.
     * In this way, to ensure the clock of the peripherals used by M core not be affected by A core which is running at
     * domain 0.
     */
    CLOCK_EnableClock(kCLOCK_Iomux);

    CLOCK_EnableClock(kCLOCK_Ipmux1);
    CLOCK_EnableClock(kCLOCK_Ipmux2);
    CLOCK_EnableClock(kCLOCK_Ipmux3);

#if defined(FLASH_TARGET)
    CLOCK_EnableClock(kCLOCK_Qspi);
#endif

    CLOCK_ControlGate(kCLOCK_SysPll1Gate, kCLOCK_ClockNeededAll);   /* Enable the CCGR gate for SysPLL1 in Domain 1 */
    CLOCK_ControlGate(kCLOCK_SysPll2Gate, kCLOCK_ClockNeededAll);   /* Enable the CCGR gate for SysPLL2 in Domain 1 */
    CLOCK_ControlGate(kCLOCK_SysPll3Gate, kCLOCK_ClockNeededAll);   /* Enable the CCGR gate for SysPLL3 in Domain 1 */
    CLOCK_ControlGate(kCLOCK_AudioPll1Gate, kCLOCK_ClockNeededAll); /* Enable the CCGR gate for AudioPLL1 in Domain 1 */
    CLOCK_ControlGate(kCLOCK_AudioPll2Gate, kCLOCK_ClockNeededAll); /* Enable the CCGR gate for AudioPLL2 in Domain 1 */
    CLOCK_ControlGate(kCLOCK_VideoPll1Gate, kCLOCK_ClockNeededAll); /* Enable the CCGR gate for VideoPLL1 in Domain 1 */
}

void init_pinmux() {
    // SPI #2
    IOMUXC_SetPinMux(IOMUXC_ECSPI2_MISO_ECSPI2_MISO, 0U);
    IOMUXC_SetPinConfig(IOMUXC_ECSPI2_MISO_ECSPI2_MISO,
        IOMUXC_SW_PAD_CTL_PAD_DSE(2U));
    IOMUXC_SetPinMux(IOMUXC_ECSPI2_MOSI_ECSPI2_MOSI, 0U);
    IOMUXC_SetPinConfig(IOMUXC_ECSPI2_MOSI_ECSPI2_MOSI,
        IOMUXC_SW_PAD_CTL_PAD_DSE(2U));
    IOMUXC_SetPinMux(IOMUXC_ECSPI2_SCLK_ECSPI2_SCLK, 0U);
    IOMUXC_SetPinConfig(IOMUXC_ECSPI2_SCLK_ECSPI2_SCLK,
        IOMUXC_SW_PAD_CTL_PAD_DSE(2U));
    IOMUXC_SetPinMux(IOMUXC_ECSPI2_SS0_ECSPI2_SS0, 0U);
    IOMUXC_SetPinConfig(IOMUXC_ECSPI2_SS0_ECSPI2_SS0,
        IOMUXC_SW_PAD_CTL_PAD_DSE(2U));
    // Safe Input #1
    IOMUXC_SetPinMux(IOMUXC_SAI5_RXFS_GPIO3_IO19, 0U); // input
    IOMUXC_SetPinConfig(IOMUXC_SAI5_RXFS_GPIO3_IO19,
        IOMUXC_SW_PAD_CTL_PAD_PE(0));
    /*
    IOMUXC_SetPinMux(IOMUXC_SAI5_RXC_GPIO3_IO20, 0U); // output
    IOMUXC_SetPinConfig(IOMUXC_SAI5_RXC_GPIO3_IO20,
        IOMUXC_SW_PAD_CTL_PAD_PE(0));
    */
}

void run_clocks(void)
{
    /* * The following steps just show how to configure the PLL clock sources using the clock driver on M7 core side .
     * Please note that the ROM has already configured the SYSTEM PLL1 to 800Mhz when power up the SOC, meanwhile A core
     * would enable SYSTEM PLL1, SYSTEM PLL2 and SYSTEM PLL3 by U-Boot.
     * Therefore, there is no need to configure the system PLL again on M7 side, otherwise it would have a risk to make
     * the SOC hang.
     */

    /* switch AHB NOC root to 24M first in order to configure the SYSTEM PLL1. */
    CLOCK_SetRootMux(kCLOCK_RootAhb, kCLOCK_AhbRootmuxOsc24M);

    /* switch AXI M7 root to 24M first in order to configure the SYSTEM PLL2. */
    CLOCK_SetRootMux(kCLOCK_RootM7, kCLOCK_M7RootmuxOsc24M);

    // CLOCK_InitSysPll2(&g_sysPll2Config); /* init SYSTEM PLL2 run at 1000MHZ */
    // CLOCK_InitSysPll3(&g_sysPll3Config); /* init SYSTEM PLL3 run at 600MHZ */

    CLOCK_SetRootDivider(kCLOCK_RootM7, 1U, 1U);              /* Set root clock to 800M */
    CLOCK_SetRootMux(kCLOCK_RootM7, kCLOCK_M7RootmuxSysPll1); /* switch cortex-m7 to SYSTEM PLL1 */
    
    // CLOCK_SetRootDivider(kCLOCK_RootQspi, 1U, 2U);              /* Set root clock to 800M */
    // CLOCK_SetRootMux(kCLOCK_RootM7, kCLOCK_M7RootmuxSysPll1); /* switch QSPI to SYSTEM PLL1 */

    CLOCK_SetRootDivider(kCLOCK_RootAhb, 1U, 1U);                   /* Set root clock freq to 133M / 1= 133MHZ */
    CLOCK_SetRootMux(kCLOCK_RootAhb, kCLOCK_AhbRootmuxSysPll1Div6); /* switch AHB to SYSTEM PLL1 DIV6 */

    CLOCK_SetRootDivider(kCLOCK_RootAudioAhb, 1U, 2U);                    /* Set root clock freq to 800MHZ/ 2= 400MHZ*/
    CLOCK_SetRootMux(kCLOCK_RootAudioAhb, kCLOCK_AudioAhbRootmuxSysPll1); /* switch AUDIO AHB to SYSTEM PLL1 */

    CLOCK_SetRootMux(kCLOCK_RootUart2, kCLOCK_UartRootmuxSysPll1Div10); /* Set UART source to SysPLL1 Div10 80MHZ */
    CLOCK_SetRootDivider(kCLOCK_RootUart2, 1U, 1U);                     /* Set root clock to 80MHZ/ 1= 80MHZ */

    CLOCK_EnableClock(kCLOCK_Rdc);   /* Enable RDC clock */
    CLOCK_EnableClock(kCLOCK_Ocram); /* Enable Ocram clock */
    
    /* The purpose to enable the following modules clock is to make sure the M7 core could work normally when A53 core
     * enters the low power status.*/
    CLOCK_EnableClock(kCLOCK_Sim_m);
    CLOCK_EnableClock(kCLOCK_Sim_main);
    CLOCK_EnableClock(kCLOCK_Sim_s);
    CLOCK_EnableClock(kCLOCK_Sim_wakeup);
    CLOCK_EnableClock(kCLOCK_Debug);
    CLOCK_EnableClock(kCLOCK_Dram);
    CLOCK_EnableClock(kCLOCK_Sec_Debug);

    /* Update core clock */
    SystemCoreClockUpdate();
}
