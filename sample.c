#include "f_nrf24l01p.h"

// Set pins for communication
sbit NRF_CE at       PORTC.B6;
sbit NRF_CE_DIR at   TRISC.B6;
sbit NRF_CSN at      PORTC.B7;
sbit NRF_CSN_DIR at  TRISC.B7;
sbit NRF_IRQ at      PORTA.B0;
sbit NRF_IRQ_DIR at  TRISA.B0;

// Define packet payload size (see datasheet)
// aka "how many bytes per packet"
#define PAYLOAD_SIZE 10

void config()
{
	// This will be the module's address
	uint8 moduleAddress[5] = {1, 2, 3, 4, 5};

	// Initialize SPI
    SPI1_Init_Advanced
    (
        _SPI_MASTER_OSC_DIV4,
        _SPI_DATA_SAMPLE_MIDDLE,
        _SPI_CLK_IDLE_LOW,
        _SPI_LOW_2_HIGH
    );

	// Initialize module
    nrf_init();
	
    nrf_set_crc                 (NRFCFG_CRC_1B);		// configure to use 1 byte CRC checksum
    nrf_set_channel             (2);					// operate on channel #2
    nrf_set_interrupts          (NRFCFG_NO_INTERRUPTS);	// do not enable interrupts on IRQ line
    nrf_set_direction           (NRFCFG_DIRECTION_TX);	// this module will act as transmitter
														// 	alternatively, use NRFCFG_DIRECTION_RX to set as RX
    nrf_enable_auto_acks        (0b000001);				// enable auto-ack on pipe #0
    nrf_enable_data_pipes       (0b000001);				// enable data pipe #0
    nrf_set_retransmit_delay    (6);					// set retransmission delay
    nrf_set_max_retransmit      (15);					// set max retransmission attempts
    nrf_set_data_rate           (NRFCFG_DATARATE_250k);	// work at 250kbps
    nrf_set_output_power        (NRFCFG_POWER_m18dbm);	// set tx power at -18 dBm
    nrf_set_rx_address          (0, 5, moduleAddress);	// set address as RX of pipe #0
    nrf_set_tx_address          (5, moduleAddress);		// set address as TX
    nrf_set_payload_size        (0, PAYLOAD_SIZE);		// specify payload size
    nrf_powerdown();									// go into standby mode	

	// Delay to let the module settle
    Delay_ms(100);
}

void test_as_tx()
{
	// This buffer will contain payload for outbound packets
	uint8 buffer[PAYLOAD_SIZE] = {0};
	
	config();
	
    while (1)
    {
		// set content of buffer
        buffer[0] ++ ;
        
		// wake up module
		nrf_powerup();
		
		// try to send a packet
		if (! nrf_tx_send_packet(PAYLOAD_SIZE, buffer, 1)) 
		{
			// an error occured, retransmit failed too (if enabled)
			nrf_flush_tx();
		}
		else
		{
			// packet transmitted
			// if enabled, confirm from RX has been received
		}
		
		// return module in standby
		nrf_powerdown();
        
		// Repeat every 5 seconds
		Delay_ms(5000);
    }
}

void test_as_rx()
{
	// This buffer will contain received payload
	uint8 buffer[PAYLOAD_SIZE] = {0};
	
	config();
	
	// Start waiting for packets
	nrf_rx_start_listening();
	
	while (1)
    {
        // Wait until a packet is received
		while (! nrf_rx_packet_ready())
		{
			Delay_ms(1);
		}

		// Get the packet data
        nrf_rx_read_packet(PAYLOAD_SIZE, buffer);
		
		// if using interrupt, clear
        nrf_clear_interrupts(NRFCFG_RX_INTERRUPT);
		
		// get back to listening status
        nrf_rx_start_listening();

        // now process data in "buffer"
		// process(buffer);
    }
}