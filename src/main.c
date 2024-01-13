/*
 * Zephyr I2C slave/target example
 *
 * Copyright (c) 2023 Aaron Mohtar & Co Pty Ltd
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Nordic TWIS reference: https://infocenter.nordicsemi.com/index.jsp?topic=%2Fdrivers_nrfx_v2.3.0%2Fgroup__nrfx__twis.html
 */

#include <stdio.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>

#include <nrfx_twis.h>


LOG_MODULE_REGISTER(i2c_slave);

#define I2C_SLAVE_ADDRESS       0x55
#define I2C_SDA_PIN     26
#define I2C_SCL_PIN     27

#define I2C_RX_BUFFER_SIZE      10

uint8_t i2c_rx_buffer[I2C_RX_BUFFER_SIZE];

const nrfx_twis_t i2c = NRFX_TWIS_INSTANCE(0);

// Register defines
#define REGISTER_WHO_AM_I       0xF0
#define REGISTER_CONFIG         0x01
#define REGISTER_STATUS         0x10

#define REGISTER_SIZE_WHO_AM_I  0x01
#define REGISTER_SIZE_CONFIG  0x01
#define REGISTER_SIZE_STATUS  0x01

uint8_t register_who_am_i[REGISTER_SIZE_WHO_AM_I];
uint8_t register_config[REGISTER_SIZE_CONFIG];
uint8_t register_status[REGISTER_SIZE_STATUS];
uint8_t register_unknown[] = {0xff};    // Register for when the incorrect register contents are requested

uint8_t last_written_register = 0x00;   // Stores the next register to read from

/*
* @brief i2c event handler
*/
void i2c_event_handler(nrfx_twis_evt_t const *p_event)
{

        switch (p_event->type)
        {
                case NRFX_TWIS_EVT_READ_REQ:
                {
                        switch(last_written_register)
                        {
                                case REGISTER_WHO_AM_I:
                                {
                                        nrfx_twis_tx_prepare(&i2c,register_who_am_i,REGISTER_SIZE_WHO_AM_I);
                                        break;
                                }
                                case REGISTER_CONFIG:
                                {
                                        nrfx_twis_tx_prepare(&i2c,register_config,REGISTER_SIZE_CONFIG);
                                        break;
                                }
                                case REGISTER_STATUS:
                                {
                                        nrfx_twis_tx_prepare(&i2c,register_status,REGISTER_SIZE_STATUS);
                                        break;
                                }
                                default:
                                {
                                        nrfx_twis_tx_prepare(&i2c,register_unknown,1);
                                        //LOG_INF("Read request for unknown register.\n");
                                }
                        }
                        break;
                }
                case NRFX_TWIS_EVT_READ_DONE:
                {
                        break;
                }
                case NRFX_TWIS_EVT_READ_ERROR:
                {
                        LOG_INF("Read error\n");
                        break;
                }
                case NRFX_TWIS_EVT_WRITE_REQ:
                {
                        nrfx_twis_rx_prepare(&i2c,i2c_rx_buffer,I2C_RX_BUFFER_SIZE);
                        //LOG_INF("Write request.\n");
                        break;
                }
                case NRFX_TWIS_EVT_WRITE_DONE:
                {
                        uint8_t len = p_event->data.rx_amount;
                        //LOG_INF("Write done. Len = %d\n", len);
                        if(len > 0)
                        {
                                last_written_register = i2c_rx_buffer[0];
                                if(len > 1)
                                {
                                        // Register write request - only 1 byte write implemented.
                                        switch (last_written_register)
                                        {
                                                case REGISTER_CONFIG:
                                                {
                                                        register_config[0] = i2c_rx_buffer[1];
                                                        LOG_INF("Register %02X changed to %02X.\n", i2c_rx_buffer[0],i2c_rx_buffer[1]);    
                                                        break;
                                                }
                                                default:
                                                {
                                                        LOG_INF("Unknown register %02X\n", i2c_rx_buffer[0]);
                                                }
                                        }   
                                }
                        }
                        break;
                }
                case NRFX_TWIS_EVT_WRITE_ERROR:
                {
                        LOG_INF("Write error\n");
                        break;
                }
                case NRFX_TWIS_EVT_GENERAL_ERROR:
                {
                        LOG_INF("General error\n");
                        break;
                }
                default:
                {
                }
        }
}



int main(void)
{
        printk("I2C slave/target example! \nBoard: %s\n", CONFIG_BOARD);

        // default register values
        register_who_am_i[0] = 0xAA;
        register_status[0] = 0x93;
        register_config[0] = 0x00;

        const nrfx_twis_config_t config = {
            .scl = I2C_SCL_PIN,
            .sda = I2C_SDA_PIN,
            .addr[0] = I2C_SLAVE_ADDRESS,
            .addr[1] = 0x00,
            .scl_pull = NRF_GPIO_PIN_PULLUP,    // update if pullups are externally set.
            .sda_pull = NRF_GPIO_PIN_PULLUP,
        };   
        // Initialise the I2C peripherial and interrupt
        nrfx_twis_init(&i2c, &config, i2c_event_handler);
        nrfx_twis_enable(&i2c);

        IRQ_CONNECT(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, 3, nrfx_isr, nrfx_twis_0_irq_handler, 0);

        printk("Entering main loop.\n");

        while (1)
        {
                k_sleep(K_MSEC(1000));
        }
        return 0;
}
