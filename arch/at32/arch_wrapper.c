/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "cd_utils.h"
#include "arch_wrapper.h"

#define BITS_SET(val, set)          ((val) |= (set))
#define BITS_CLR(val, clr)          ((val) &= ~(clr))
#define BITS_SET_CLR(val, set, clr) ((val) = ((val) | (set)) & ~(clr))

#define CCR             ctrl
#define CNDTR           dtcnt
#define CMAR            maddr
#define CPAR            paddr
#define ISR             sts
#define IFCR            clr
#define DMA_CCR_EN      (1 << 0)
#define DMA_CCR_TCIE    (1 << 1)

#define SR              sts
#define DR              dt
#define CR1             ctrl1
#define CR2             ctrl2
#define SPI_FLAG_BSY    (1 << 7)
#define SPI_CR1_SPE     (1 << 6)
#define SPI_CR2_RXDMAEN (1 << 0)
#define SPI_CR2_TXDMAEN (1 << 1)

#define CD_DMA_MASK     (2 << 0) // DMA_ISR.TCIF1
#define CD_SS_HIGH()    {CD_SS_GPIO_PORT->scr = CD_SS_PIN;}
#define CD_SS_LOW()     {CD_SS_GPIO_PORT->clr = CD_SS_PIN;}
#define CD_INT_RD()     (CD_INT_GPIO_PORT->idt & CD_INT_PIN)
#define CD_IRQ          EXINT9_5_IRQn

#ifdef CD_ARCH_SPI_DMA

void spi_wr(spi_t *dev, const uint8_t *w_buf, uint8_t *r_buf, int len)
{
    dev->dma_ch_rx->CCR &= ~(DMA_CCR_EN | DMA_CCR_TCIE);
    dev->dma_ch_rx->CNDTR = len;
    dev->dma_ch_rx->CMAR = (uint32_t)r_buf;
    dev->dma_ch_rx->CCR |= DMA_CCR_EN;

    dev->dma_ch_tx->CCR &= ~DMA_CCR_EN;
    dev->dma_ch_tx->CNDTR = len;
    dev->dma_ch_tx->CMAR = (uint32_t)w_buf;
    dev->dma_ch_tx->CCR |= DMA_CCR_EN;

    while (!(dev->dma_rx->ISR & CD_DMA_MASK));
    dev->dma_rx->IFCR = CD_DMA_MASK;
}

void spi_wr_it(spi_t *dev, const uint8_t *w_buf, uint8_t *r_buf, int len)
{
    dev->dma_ch_rx->CCR &= ~DMA_CCR_EN;
    dev->dma_ch_rx->CNDTR = len;
    dev->dma_ch_rx->CMAR = (uint32_t)r_buf;
    dev->dma_ch_rx->CCR |= DMA_CCR_TCIE | DMA_CCR_EN;

    dev->dma_ch_tx->CCR &= ~DMA_CCR_EN;
    dev->dma_ch_tx->CNDTR = len;
    dev->dma_ch_tx->CMAR = (uint32_t)w_buf;
    dev->dma_ch_tx->CCR |= DMA_CCR_EN;
}


/* isr template:
void spi_wr_isr(void)
{
    //uint32_t flag_it = CD_DMA->ISR;
    //if (flag_it & CD_DMA_MASK) {
        CD_DMA->IFCR = CD_DMA_MASK;
        cdctl_spi_isr();
    //}
}
*/

void spi_wr_init(spi_t *dev)
{
    BITS_SET(dev->spi->CR1, SPI_CR1_SPE); // enable spi
    BITS_SET(dev->spi->CR2, SPI_CR2_RXDMAEN);
    BITS_SET(dev->spi->CR2, SPI_CR2_TXDMAEN);
    dev->dma_ch_rx->CCR &= ~DMA_CCR_EN;
    dev->dma_ch_tx->CCR &= ~DMA_CCR_EN;
    dev->dma_ch_tx->CPAR = (uint32_t)&dev->spi->DR;
    dev->dma_ch_rx->CPAR = (uint32_t)&dev->spi->DR;
}

#endif


#ifdef CD_ARCH_CRC_HW

uint16_t crc16_hw_sub(const uint8_t *data, uint32_t length, uint16_t crc_val)
{
    uint16_t ret_val;
#ifdef CD_CRC_HW_IRQ_SAFE // not recommended, avoid large critical sections
    uint32_t flags;
    local_irq_save(flags);
#endif
    CRC->INIT = crc_val;
    CRC->CR = 0xe9;
    CRC->INIT = CRC->DR; // bit-reverse crc_val

    while (((unsigned)data & 3) && length) {
        *(volatile uint8_t *)&CRC->DR = *data++;
        length--;
    }

    unsigned cnt = length >> 2;
    while (cnt--) {
        CRC->DR = *(uint32_t *)data;
        data += 4;
    }

    length &= 3;
    while (length--)
        *(volatile uint8_t *)&CRC->DR = *data++;

    ret_val = CRC->DR;
#ifdef CD_CRC_HW_IRQ_SAFE
    local_irq_restore(flags);
#endif
    return ret_val;
}

#endif


void delay_us(uint32_t us)
{
    uint32_t cnt_1ms = SysTick->LOAD + 1;
    uint32_t last = SysTick->VAL;
    uint32_t total = 0;
    uint32_t target = cnt_1ms / 1000 * us;

    while (total < target) {
        uint32_t cur = SysTick->VAL;
        int32_t diff = last - cur;
        if (diff < 0)
            diff += cnt_1ms;
        total += diff;
        last = cur;
    }
}

