#ifndef _f_nrf24l01_h
#define _f_nrf24l01_h

#include <built_in.h>

extern sbit NRF_CE;
extern sbit NRF_CE_DIR;
extern sbit NRF_CSN;
extern sbit NRF_CSN_DIR;
extern sbit NRF_IRQ;
extern sbit NRF_IRQ_DIR;

typedef struct
{
    uint8 rx_data_ready;
    uint8 tx_data_sent;
    uint8 max_retries_reached;
    uint8 available_pipe;
    uint8 tx_full;
    uint8 rx_empty;

} nrf_status;

typedef struct
{
    uint8 tx_reuse;
    uint8 tx_full;
    uint8 tx_empty;
    uint8 rx_full;
    uint8 rx_empty;

} nrf_fifo_status;

/*
Default settings at startup:
Power                  down
interrupts             all enabled
channel                2
data rate              2 Mbps
power                  0 dBm
CRC                    1 byte
Data pipe              0 and 1 enabled
Auto_Ack               enabled on all pipes
AutoRetransmitDelay    250 us
AutoRetransmit count   3
per payload            0
Dynamic payload length deactivated
ACK with payload       deactivated
TX_NO_ACK              not allowed
address width          5 bytes
TX address             0xE7 0xE7 0xE7 0xE7 0xE7
RX address pipe 0      0xE7 0xE7 0xE7 0xE7 0xE7
RX address pipe 1      0xC2 0xC2 0xC2 0xC2 0xC2
RX address pipe 2      0xC2 0xC2 0xC2 0xC2 0xC3
RX address pipe 3      0xC2 0xC2 0xC2 0xC2 0xC4
RX address pipe 4      0xC2 0xC2 0xC2 0xC2 0xC5
RX address pipe 5      0xC2 0xC2 0xC2 0xC2 0xC6
*/

#define NRFCMD_R_REGISTER           0b00000000
#define NRFCMD_W_REGISTER           0b00100000
#define NRFCMD_R_RX_PAYLOAD         0b01100001
#define NRFCMD_W_TX_PAYLOAD         0b10100000
#define NRFCMD_FLUSH_TX             0b11100001
#define NRFCMD_FLUSH_RX             0b11100010
#define NRFCMD_REUSE_TX_PL          0b11100011
#define NRFCMD_R_RX_PL_WID          0b01100000
#define NRFCMD_W_ACK_PAYLOAD        0b10101000
#define NRFCMD_W_TX_PAYLOAD_NOACK   0b10110000
#define NRFCMD_NOP                  0b11111111

#define NRFREG_CONFIG               0x00
#define NRFREG_EN_AA                0x01
#define NRFREG_EN_RXADDR            0x02
#define NRFREG_SETUP_AW             0x03
#define NRFREG_SETUP_RETR           0x04
#define NRFREG_RF_CH                0x05
#define NRFREG_RF_SETUP             0x06
#define NRFREG_STATUS               0x07
#define NRFREG_OBSERVE_TX           0x08
#define NRFREG_RPD                  0x09
#define NRFREG_RX_ADDR_P0           0x0A
#define NRFREG_RX_ADDR_P1           0x0B
#define NRFREG_RX_ADDR_P2           0x0C
#define NRFREG_RX_ADDR_P3           0x0D
#define NRFREG_RX_ADDR_P4           0x0E
#define NRFREG_RX_ADDR_P5           0x0F
#define NRFREG_TX_ADDR              0x10
#define NRFREG_RX_PW_P0             0x11
#define NRFREG_RX_PW_P1             0x12
#define NRFREG_RX_PW_P2             0x13
#define NRFREG_RX_PW_P3             0x14
#define NRFREG_RX_PW_P4             0x15
#define NRFREG_RX_PW_P5             0x16
#define NRFREG_FIFO_STATUS          0x17
#define NRFREG_DYNPD                0x1C
#define NRFREG_FEATURE              0x1D

#define NRFCFG_RX_INTERRUPT         0b001
#define NRFCFG_TX_INTERRUPT         0b010
#define NRFCFG_MAXRETRIES_INTERRUPT 0b100
#define NRFCFG_ALL_INTERRUPTS       0b111
#define NRFCFG_NO_INTERRUPTS        0b000
#define NRFCFG_NO_CRC               0b001
#define NRFCFG_CRC_1B               0b010
#define NRFCFG_CRC_2B               0b100
#define NRFCFG_POWER_UP             1
#define NRFCFG_POWER_DOWN           0
#define NRFCFG_DIRECTION_RX         1
#define NRFCFG_DIRECTION_TX         0
#define NRFCFG_DATARATE_250k        0b00
#define NRFCFG_DATARATE_1M          0b01
#define NRFCFG_DATARATE_2M          0b10
#define NRFCFG_POWER_0dbm           0b00
#define NRFCFG_POWER_m6dbm          0b01
#define NRFCFG_POWER_m12dbm         0b10
#define NRFCFG_POWER_m18dbm         0b11
#define NRFCFG_ALL_PIPES            0b00111111;

#define NRF_BUFFERSIZE_DYNAMIC      0

#define ___nrf_delay_poweron()      Delay_ms(120)
#define ___nrf_delay_post_select()  Delay_us(10)
#define ___nrf_delay_post_cmd()     Delay_us(10)
#define ___nrf_delay_pre_deselect() Delay_us(10)
#define ___nrf_delay_pd2stby()      Delay_ms(5)
#define ___nrf_delay_stby2a()       Delay_us(150)
#define ___nrf_delay_CE_high()      Delay_us(12)
#define ___nrf_delay_peCE2csn()     Delay_us(6)
#define ___nrf_delay_active2pd()    Delay_us(100)
#define ___nrf_select()             { NRF_CSN = 0; ___nrf_delay_post_select(); }
#define ___nrf_deselect()           { ___nrf_delay_pre_deselect(); NRF_CSN = 1; }
#define ___nrf_ce_on()              { NRF_CE = 1; ___nrf_delay_peCE2csn(); }
#define ___nrf_ce_off()             { NRF_CE = 0; }
#define ___nrf_ce_pulse()           { NRF_CE = 1; ___nrf_delay_CE_high(); NRF_CE = 0; }

uint8   ___nrf_send                 (uint8 value);
uint8   ___nrf_send_command         (uint8 cmd, uint8 numbytes, uint8 * buffer);
uint8   ___nrf_read_command         (uint8 cmd, uint8 numbytes, uint8 * buffer);

// Generic functions
void    nrf_init                    ();
uint8   nrf_set_interrupts          (uint8 interrupts);
uint8   nrf_set_crc                 (uint8 crc);
uint8   nrf_set_power               (uint8 power);
uint8   nrf_set_direction           (uint8 direction);
uint8   nrf_enable_auto_acks        (uint8 channels);
uint8   nrf_enable_data_pipes       (uint8 channels);
uint8   nrf_set_address_width       (uint8 numbytes);
uint8   nrf_set_retransmit_delay    (uint8 delayPer250us);
uint8   nrf_set_max_retransmit      (uint8 maxattempts);
uint8   nrf_set_channel             (uint8 channel);
uint8   nrf_set_data_rate           (uint8 datarate);
uint8   nrf_set_output_power        (uint8 power);
uint8   nrf_get_status              (nrf_status * dest);
uint8   nrf_get_current_retransmit_count        ();
uint8   nrf_get_total_retransmit_count          ();
uint8   nrf_signal_detected         ();
uint8   nrf_set_rx_pipe             (uint8 pipenum, uint8 num, uint8 * buffer);
uint8   nrf_set_rx_address          (uint8 pipenum, uint8 num, uint8 * buffer);
uint8   nrf_set_tx_address          (uint8 num, uint8 * buffer);
uint8   nrf_set_payload_size        (uint8 pipenum, uint8 size);
uint8   nrf_get_fifo_status         (nrf_fifo_status * dest);
uint8   nrf_enable_dynamic_payload  (uint8 channels);
uint8   nrf_enable_dynamic_payload_length       (uint8 enable);
uint8   nrf_enable_ack_payload      (uint8 enable);
uint8   nrf_enable_no_ack_command   (uint8 enable);
#define nrf_powerdown()             nrf_set_power(NRFCFG_POWER_DOWN)
#define nrf_powerup()               nrf_set_power(NRFCFG_POWER_UP)

// General command functions
uint8   nrf_read_payload            (uint8 size, uint8 * buffer);
uint8   nrf_write_payload           (uint8 size, uint8 * buffer);
uint8   nrf_flush_tx                ();
uint8   nrf_flush_rx                ();
uint8   nrf_reuse_tx_payload        ();
uint8   nrf_clear_interrupts        (uint8 interrupts);

// RX functions
void    nrf_rx_start_listening      ();
void    nrf_rx_stop_listening       ();
uint8   nrf_rx_packet_ready         ();
uint8   nrf_rx_get_sender_pipe      ();
#define nrf_rx_read_packet(size, buffer) nrf_read_payload(size, buffer)

// TX functions
uint8   nrf_tx_send_packet          (uint8 size, uint8 * buffer, uint8 wait);
uint8   nrf_tx_send_current         (uint8 wait);
uint8   nrf_tx_packet_sent          ();
uint8   nrf_tx_packet_failed        ();
#define nrf_tx_wait_packet_send(size, buffer) nrf_tx_send_packet(size, buffer, 1)
#define nrf_tx_start_packet_send(size, buffer) nrf_tx_send_packet(size, buffer, 0)

// Implementations

void nrf_init()
{
    NRF_IRQ_DIR = 1;

    NRF_CE = 0;
    NRF_CE_DIR =  0;

    NRF_CSN = 1;
    NRF_CSN_DIR = 0;

    ___nrf_delay_poweron();
    nrf_set_power(NRFCFG_POWER_DOWN);
    nrf_flush_tx();
    nrf_flush_rx();
}

uint8 nrf_tx_send_packet(uint8 size, uint8 * buffer, uint8 wait)
{
    nrf_write_payload(size, buffer);
    return nrf_tx_send_current(wait);
}

uint8 nrf_tx_send_current(uint8 wait)
{
    uint8 status;

    ___nrf_ce_pulse();
    if (! wait) return 1;

    while (1)
    {
        status = ___nrf_read_command(NRFCMD_NOP, 0, 0);
        if (status.B5)
        {
            nrf_clear_interrupts(NRFCFG_TX_INTERRUPT);
            status = 1;
            break;
        }
        if (status.B4)
        {
            nrf_clear_interrupts(NRFCFG_MAXRETRIES_INTERRUPT);
            status = 0;
            break;
        }
    }
    return status;
}

uint8 nrf_tx_packet_failed()
{
    uint8 status = ___nrf_read_command(NRFCMD_NOP, 0, 0);
    if (status.B4)
    {
        nrf_clear_interrupts(NRFCFG_MAXRETRIES_INTERRUPT);
        return 1;
    }
    return 0;
}

uint8 nrf_tx_packet_sent()
{
    uint8 status = ___nrf_read_command(NRFCMD_NOP, 0, 0);
    if (status.B5)
    {
        nrf_clear_interrupts(NRFCFG_TX_INTERRUPT);
        return 1;
    }
    return 0;
}

uint8 nrf_rx_get_sender_pipe()
{
    return ((___nrf_read_command(NRFCMD_NOP, 0, 0) & 0b00001110) >> 1);
}

uint8 nrf_rx_packet_ready()
{
    uint8 fifo = 0;
    ___nrf_read_command(NRFCMD_R_REGISTER | NRFREG_FIFO_STATUS, 1, &fifo);
    return (! fifo.B0);
}

void nrf_rx_start_listening()
{
    ___nrf_ce_on();
    ___nrf_delay_stby2a();
}

void nrf_rx_stop_listening()
{
    ___nrf_ce_off();
}

uint8 nrf_set_rx_pipe(uint8 pipenum, uint8 num, uint8 * buffer)
{
    nrf_set_rx_address(pipenum, num, buffer);
    return nrf_set_payload_size(pipenum, num);
}

uint8 nrf_clear_interrupts(uint8 interrupts)
{
    uint8 actual = ___nrf_read_command(NRFCMD_NOP, 0, 0);

    actual.B6 = (interrupts & NRFCFG_RX_INTERRUPT ? 1 : 0);
    actual.B5 = (interrupts & NRFCFG_TX_INTERRUPT ? 1 : 0);
    actual.B4 = (interrupts & NRFCFG_MAXRETRIES_INTERRUPT ? 1 : 0);

    return ___nrf_send_command(NRFCMD_W_REGISTER | NRFREG_STATUS, 1, &actual);
}

uint8 nrf_reuse_tx_payload()
{
    return ___nrf_send_command(NRFCMD_REUSE_TX_PL, 0, 0);
}

uint8 nrf_flush_rx()
{
    return ___nrf_send_command(NRFCMD_FLUSH_RX, 0, 0);
}

uint8 nrf_flush_tx()
{
    return ___nrf_send_command(NRFCMD_FLUSH_TX, 0, 0);
}

uint8 nrf_write_payload(uint8 size, uint8 * buffer)
{
    uint8 i, status;
    if (size < 1) size = 1;
    if (size > 32) size = 32;

    ___nrf_select();
    status = ___nrf_send(NRFCMD_W_TX_PAYLOAD);
    for (i = 0; i < size; i ++) ___nrf_send(buffer[i]);
    ___nrf_deselect();

    return status;
}

uint8 nrf_read_payload(uint8 size, uint8 * buffer)
{
    uint8 i;
    ___nrf_select();
    if (size == NRF_BUFFERSIZE_DYNAMIC || size > 32)
    {
        ___nrf_send(NRFCMD_R_RX_PL_WID);
        size = ___nrf_send(NRFCMD_NOP);

        if (size > 32)
        {
            ___nrf_send(NRFCMD_FLUSH_RX);
            ___nrf_deselect();
            return 0;
        }
    }
    
    ___nrf_send(NRFCMD_R_RX_PAYLOAD);
    for (i = 0; i < size; i ++)
        buffer[i] = ___nrf_send(NRFCMD_NOP);

    ___nrf_deselect();
    return size;
}

uint8 nrf_enable_no_ack_command(uint8 enable)
{
    uint8 actual = 0;
    ___nrf_read_command(NRFCMD_R_REGISTER | NRFREG_FEATURE, 1, &actual);
    actual.B0 = enable ? 1 : 0;
    return ___nrf_send_command(NRFCMD_W_REGISTER | NRFREG_FEATURE, 1, &actual);
}

uint8 nrf_enable_ack_payload(uint8 enable)
{
    uint8 actual = 0;
    ___nrf_read_command(NRFCMD_R_REGISTER | NRFREG_FEATURE, 1, &actual);
    actual.B1 = enable ? 1 : 0;
    return ___nrf_send_command(NRFCMD_W_REGISTER | NRFREG_FEATURE, 1, &actual);
}

uint8 nrf_enable_dynamic_payload_length(uint8 enable)
{
    uint8 actual = 0;
    ___nrf_read_command(NRFCMD_R_REGISTER | NRFREG_FEATURE, 1, &actual);
    actual.B2 = enable ? 1 : 0;
    return ___nrf_send_command(NRFCMD_W_REGISTER | NRFREG_FEATURE, 1, &actual);
}

uint8 nrf_enable_dynamic_payload(uint8 channels)
{
    channels &= 0b00111111;
    return ___nrf_send_command(NRFCMD_W_REGISTER | NRFREG_DYNPD, 1, &channels);
}

uint8 nrf_get_fifo_status(nrf_fifo_status * dest)
{
    uint8 fstat = 0;
    ___nrf_read_command(NRFCMD_R_REGISTER | NRFREG_FIFO_STATUS, 1, &fstat);
    
    if (dest)
    {
        dest -> tx_reuse = fstat.B6;
        dest -> tx_full = fstat.B5;
        dest -> tx_empty = fstat.B4;
        dest -> rx_full = fstat.B1;
        dest -> rx_empty = fstat.B0;
    }
    return fstat;
}

uint8 nrf_set_payload_size(uint8 pipenum, uint8 size)
{
    if (pipenum > 5) pipenum = 5;
    if (size < 1) size = 1;
    if (size > 32) size = 32;

    size &= 0b00111111;
    return ___nrf_send_command(NRFCMD_W_REGISTER | (NRFREG_RX_PW_P0 + pipenum), 1, &size);
}

uint8 nrf_set_tx_address(uint8 num, uint8 * buffer)
{
    if (num < 3) num = 3;
    if (num > 5) num = 5;
    return ___nrf_send_command(NRFCMD_W_REGISTER | NRFREG_TX_ADDR, num, buffer);
}

uint8 nrf_set_rx_address(uint8 pipenum, uint8 num, uint8 * buffer)
{
    if (pipenum > 5) pipenum = 5;
    if (pipenum < 2)
    {
        if (num < 3) num = 3;
        if (num > 5) num = 5;
    }
    else num = 1;
    
    return ___nrf_send_command(NRFCMD_W_REGISTER | (NRFREG_RX_ADDR_P0 + pipenum), num, buffer);
}

uint8 nrf_signal_detected()
{
    uint8 actual = 0;
    ___nrf_read_command(NRFCMD_R_REGISTER | NRFREG_RPD, 1, &actual);
    return actual.B0;
}

uint8 nrf_get_total_retransmit_count()
{
    uint8 actual = 0;
    ___nrf_read_command(NRFCMD_R_REGISTER | NRFREG_OBSERVE_TX, 1, &actual);
    return actual >> 4;
}

uint8 nrf_get_current_retransmit_count()
{
    uint8 actual = 0;
    ___nrf_read_command(NRFCMD_R_REGISTER | NRFREG_OBSERVE_TX, 1, &actual);
    return actual & 0b00001111;
}

uint8 nrf_get_status(nrf_status * dest)
{
    uint8 status = ___nrf_read_command(NRFCMD_NOP, 0, 0);
    
    if (dest)
    {
        dest -> rx_data_ready = status.B6;
        dest -> tx_data_sent = status.B5;
        dest -> max_retries_reached = status.B4;
        
        dest -> available_pipe = (status & 0b00001110) >> 1;
        dest -> tx_full = status.B0;
        dest -> rx_empty = (dest -> available_pipe == 0b111 ? 1 : 0);
    }
    return status;
}

uint8 nrf_set_output_power(uint8 power)
{
    uint8 actual = 0;
    ___nrf_read_command(NRFCMD_R_REGISTER | NRFREG_RF_SETUP, 1, &actual);

    switch (power)
    {
        case NRFCFG_POWER_0dbm:
            actual.B2 = 1;
            actual.B1 = 1;
            break;

        case NRFCFG_POWER_m6dbm:
            actual.B2 = 1;
            actual.B1 = 0;
            break;

        case NRFCFG_POWER_m12dbm:
            actual.B2 = 0;
            actual.B1 = 1;
            break;
            
        default:
            actual.B2 = 0;
            actual.B1 = 0;
    }

    return ___nrf_send_command(NRFCMD_W_REGISTER | NRFREG_RF_SETUP, 1, &actual);
}

uint8 nrf_set_data_rate(uint8 datarate)
{
    uint8 actual = 0;
    ___nrf_read_command(NRFCMD_R_REGISTER | NRFREG_RF_SETUP, 1, &actual);

    switch (datarate)
    {
        case NRFCFG_DATARATE_2M:
            actual.B3 = 1;
            actual.B5 = 0;
            break;

        case NRFCFG_DATARATE_1M:
            actual.B3 = 0;
            actual.B5 = 0;
            break;

        default:
            actual.B3 = 0;
            actual.B5 = 1;
    }

    return ___nrf_send_command(NRFCMD_W_REGISTER | NRFREG_RF_SETUP, 1, &actual);
}

uint8 nrf_set_channel(uint8 channel)
{
    if (channel > 127) channel = 127;
    channel &= 0b01111111;
    return ___nrf_send_command(NRFCMD_W_REGISTER | NRFREG_RF_CH, 1, &channel);
}

uint8 nrf_set_max_retransmit(uint8 maxattempts)
{
    uint8 actual = 0;
    ___nrf_read_command(NRFCMD_R_REGISTER | NRFREG_SETUP_RETR, 1, &actual);

    actual &= 0b11110000;
    if (maxattempts > 15) maxattempts = 15;
    actual |= maxattempts;

    return ___nrf_send_command(NRFCMD_W_REGISTER | NRFREG_SETUP_RETR, 1, &actual);
}

uint8 nrf_set_retransmit_delay(uint8 delayPer250us)
{
    uint8 actual = 0;
    
    if (delayPer250us < 1)  delayPer250us = 1;
    if (delayPer250us > 16) delayPer250us = 16;

    ___nrf_read_command(NRFCMD_R_REGISTER | NRFREG_SETUP_RETR, 1, &actual);

    actual &= 0b00001111;
    actual |= ((delayPer250us - 1) << 4);
    
    return ___nrf_send_command(NRFCMD_W_REGISTER | NRFREG_SETUP_RETR, 1, &actual);
}

uint8 nrf_set_address_width(uint8 numbytes)
{
    if (numbytes < 3) numbytes = 3;
    if (numbytes > 5) numbytes = 5;
    
    numbytes = ((numbytes - 2) & 0b00000011);
    return ___nrf_send_command(NRFCMD_W_REGISTER | NRFREG_SETUP_AW, 1, &numbytes);
}

uint8 nrf_enable_data_pipes(uint8 channels)
{
    channels &= 0b00111111;
    return ___nrf_send_command(NRFCMD_W_REGISTER | NRFREG_EN_RXADDR, 1, &channels);
}

uint8 nrf_enable_auto_acks(uint8 channels)
{
    channels &= 0b00111111;
    return ___nrf_send_command(NRFCMD_W_REGISTER | NRFREG_EN_AA, 1, &channels);
}

uint8 nrf_set_direction(uint8 direction)
{
    uint8 actual = 0;
    ___nrf_read_command(NRFCMD_R_REGISTER | NRFREG_CONFIG, 1, &actual);
    actual.B0 = (direction ? 1 : 0);
    return ___nrf_send_command(NRFCMD_W_REGISTER | NRFREG_CONFIG, 1, &actual);
}

uint8 nrf_set_power(uint8 power)
{
    uint8 actual = 0;
    uint8 result;
    
    ___nrf_ce_off();
    
    ___nrf_read_command(NRFCMD_R_REGISTER | NRFREG_CONFIG, 1, &actual);
    actual.B1 = (power ? 1 : 0);
    result = ___nrf_send_command(NRFCMD_W_REGISTER | NRFREG_CONFIG, 1, &actual);
    if (power) ___nrf_delay_pd2stby();
    else       ___nrf_delay_active2pd();
    return result;
}

uint8 nrf_set_crc(uint8 crc)
{
    uint8 actual = 0;
    ___nrf_read_command(NRFCMD_R_REGISTER | NRFREG_CONFIG, 1, &actual);
    
    if (crc & NRFCFG_NO_CRC)        actual.B3 = 0;
    else if (crc & NRFCFG_CRC_1B) { actual.B3 = 1; actual.B2 = 0; }
    else                          { actual.B3 = 1; actual.B2 = 1; }
    
    return ___nrf_send_command(NRFCMD_W_REGISTER | NRFREG_CONFIG, 1, &actual);
}

uint8 nrf_set_interrupts(uint8 interrupts)
{
    uint8 actual = 0;
    ___nrf_read_command(NRFCMD_R_REGISTER | NRFREG_CONFIG, 1, &actual);
    
    actual.B6 = (interrupts & NRFCFG_RX_INTERRUPT) ? 0 : 1;
    actual.B5 = (interrupts & NRFCFG_TX_INTERRUPT) ? 0 : 1;
    actual.B4 = (interrupts & NRFCFG_MAXRETRIES_INTERRUPT) ? 0 : 1;

    return ___nrf_send_command(NRFCMD_W_REGISTER | NRFREG_CONFIG, 1, &actual);
}

uint8 ___nrf_send_command(uint8 cmd, uint8 numbytes, uint8 * buffer)
{
    uint8 status, i;

    ___nrf_select();
    status = ___nrf_send(cmd);

    for (i = 0; i < numbytes; i ++)
        ___nrf_send(buffer[i]);

    ___nrf_deselect();

    return status;
}

uint8 ___nrf_read_command(uint8 cmd, uint8 numbytes, uint8 * buffer)
{
    uint8 status, i;

    ___nrf_select();
    status = ___nrf_send(cmd);

    for (i = 0; i < numbytes; i ++)
        buffer[i] = ___nrf_send(NRFCMD_NOP);

    ___nrf_deselect();

    return status;
}

uint8 ___nrf_send(uint8 value)
{
    uint8 result = SPI_NRF_FUNCTION(value);
    ___nrf_delay_post_cmd();
    return result;
}

#endif