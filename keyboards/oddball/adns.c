#include <avr/io.h>
#include <avr/interrupt.h>
#include "quantum.h"
#include "pointing_device.h"
#include "adns9800_srom_A4.h"
#include "../../lib/lufa/LUFA/Drivers/Peripheral/SPI.h"

// registers
#define Product_ID  0x00
#define Revision_ID 0x01
#define Motion  0x02
#define Delta_X_L 0x03
#define Delta_X_H 0x04
#define Delta_Y_L 0x05
#define Delta_Y_H 0x06
#define SQUAL 0x07
#define Raw_Data_Sum  0x08
#define Maximum_Raw_data  0x09
#define Minimum_Raw_data  0x0A
#define Shutter_Lower 0x0B
#define Shutter_Upper 0x0C
#define Control 0x0D
#define Config1 0x0F
#define Config2 0x10
#define Angle_Tune  0x11
#define Frame_Capture 0x12
#define SROM_Enable 0x13
#define Run_Downshift 0x14
#define Rest1_Rate_Lower  0x15
#define Rest1_Rate_Upper  0x16
#define Rest1_Downshift 0x17
#define Rest2_Rate_Lower  0x18
#define Rest2_Rate_Upper  0x19
#define Rest2_Downshift 0x1A
#define Rest3_Rate_Lower  0x1B
#define Rest3_Rate_Upper  0x1C
#define Observation 0x24
#define Data_Out_Lower  0x25
#define Data_Out_Upper  0x26
#define Raw_Data_Dump 0x29
#define SROM_ID 0x2A
#define Min_SQ_Run  0x2B
#define Raw_Data_Threshold  0x2C
#define Config5 0x2F
#define Power_Up_Reset  0x3A
#define Shutdown  0x3B
#define Inverse_Product_ID  0x3F
#define LiftCutoff_Tune3  0x41
#define Angle_Snap  0x42
#define LiftCutoff_Tune1  0x4A
#define Motion_Burst  0x50
#define LiftCutoff_Tune_Timeout 0x58
#define LiftCutoff_Tune_Min_Length  0x5A
#define SROM_Load_Burst 0x62
#define Lift_Config 0x63
#define Raw_Data_Burst  0x64
#define LiftCutoff_Tune2  0x65

// pins
#define NCS 0

extern const uint16_t firmware_length;
extern const uint8_t firmware_data[];

enum motion_burst_property{
    motion = 0,
    observation,
    delta_x_l,
    delta_x_h,
    delta_y_l,
    delta_y_h,
    squal,
    pixel_sum,
    maximum_pixel,
    minimum_pixel,
    shutter_upper,
    shutter_lower,
    frame_period_upper,
    frame_period_lower,
    end_data
};

// used to track the motion delta between updates
volatile int8_t delta_x;
volatile int8_t delta_y;

void adns_begin(void){
    PORTB &= ~ (1 << NCS);
}

void adns_end(void){
    PORTB |= (1 << NCS);
}

void adns_write(uint8_t reg_addr, uint8_t data){

    adns_begin();

    //send address of the register, with MSBit = 1 to indicate it's a write
    SPI_TransferByte(reg_addr | 0x80 );
    SPI_TransferByte(data);

    // tSCLK-NCS for write operation
    wait_us(20);

    adns_end();

    // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound
    wait_us(100);
}

uint8_t adns_read(uint8_t reg_addr){

    adns_begin();

    // send adress of the register, with MSBit = 0 to indicate it's a read
    SPI_TransferByte(reg_addr & 0x7f );
    uint8_t data = SPI_TransferByte(0);

    // tSCLK-NCS for read operation is 120ns
    wait_us(1);

    adns_end();

    //  tSRW/tSRR (=20us) minus tSCLK-NCS
    wait_us(19);

    return data;
}

void pointing_device_init(void) {

    if(!is_keyboard_master())
        return;

    // interrupt 2
    EICRA &= ~(1 << 4);
    EICRA |= (1 << 5);
    EIMSK |= (1<<INT2);

    // mode 3
    SPI_Init(
        SPI_SPEED_FCPU_DIV_128 |
        SPI_ORDER_MSB_FIRST |
        SPI_SCK_LEAD_FALLING |
        SPI_SAMPLE_TRAILING |
        SPI_MODE_MASTER);

    // set B0 output
    DDRB |= (1 << 0);

    // reset serial port
    adns_end();
    adns_begin();
    adns_end();

    // reboot
    adns_write(Power_Up_Reset, 0x5a);
    wait_ms(60);

    // read registers and discard
    adns_read(Motion);
    adns_read(Delta_X_L);
    adns_read(Delta_X_H);
    adns_read(Delta_Y_L);
    adns_read(Delta_Y_H);

    // upload firmware

    // set the configuration_IV register in 3k firmware mode
    // bit 1 = 1 for 3k mode, other bits are reserved
    adns_write(Config2, 0x20);

    // write 0x1d in SROM_enable reg for initializing
    adns_write(SROM_Enable, 0x1d);

    // wait for more than one frame period
    // assume that the frame rate is as low as 100fps... even if it should never be that low
    wait_ms(10);

    // write 0x18 to SROM_enable to start SROM download
    adns_write(SROM_Enable, 0x18);

    // write the SROM file (=firmware data)
    adns_begin();

    // write burst destination adress
    SPI_TransferByte(SROM_Load_Burst | 0x80);
    wait_us(15);

    // send all bytes of the firmware
    unsigned char c;
    for(int i = 0; i < firmware_length; i++){
        c = (unsigned char)pgm_read_byte(firmware_data + i);
        SPI_TransferByte(c);
        wait_us(15);
    }

    //Read the SROM_ID register to verify the ID before any other register reads or writes.
    adns_read(SROM_ID);

    //Write 0x00 to Config2 register for wired mouse or 0x20 for wireless mouse design.
    adns_write(Config2, 0x00);

    // set initial CPI resolution
    adns_write(Config1, 0x15);

    adns_end();

    wait_ms(10);

//    // enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
//    // reading the actual value of the register is important because the real
//    // default value is different from what is said in the datasheet, and if you
//    // change the reserved bytes (like by writing 0x00...) it would not work.
//    uint8_t laser_ctrl0 = adns_read(REG_LASER_CTRL0);
//    adns_write(REG_LASER_CTRL0, laser_ctrl0 & 0xf0);

    wait_ms(1);

    // set the configuration_I register to set the CPI
    // 0x01 = 50, minimum
    // 0x44 = 3400, default
    // 0x8e = 7100
    // 0xA4 = 8200, maximum
    adns_write(Config1, 0x04);

    wait_ms(100);
}

void tap_tb(int16_t delta, uint16_t keycode0, uint16_t keycode1) {
	if(delta == 0) {
		return;
	}
	if(delta > 0) {
		tap_code(keycode0);
	} else {
		tap_code(keycode1);
	}
}

uint8_t trackMode = 0; // 0 Mousecursor; 1 arrowkeys; 2 scrollwheel
void pointing_device_task(void) {
    if(!is_keyboard_master())
        return;
    report_mouse_t report = pointing_device_get_report();
    // clamp deltas from -127 to 127
	delta_x = delta_x < -127 ? -127 : delta_x > 127 ? 127 : delta_x;
    delta_y = delta_y < -127 ? -127 : delta_y > 127 ? 127 : delta_y;
    delta_y = -delta_y;
	if (trackMode == 0){
		report.x = delta_x;
		report.y = delta_y;
    } else if (trackMode == 1) {
		tap_tb(delta_x, KC_RIGHT, KC_LEFT);
		tap_tb(delta_y, KC_UP   , KC_DOWN);
    } else {
		report.h = delta_x;
		report.v = delta_y;
    }
    // reset deltas
    delta_x = 0;
    delta_y = 0;
    pointing_device_set_report(report);
    pointing_device_send();
}

int16_t convertDeltaToInt(uint8_t high, uint8_t low){
    // join bytes into twos compliment
    uint16_t twos_comp = (high << 8) | low;
    // convert twos comp to int
    if (twos_comp & 0x8000)
        return -1 * ((twos_comp ^ 0xffff) + 1);
    return twos_comp;
}

ISR(INT2_vect) {
    // called on interrupt 2 when sensed motion
    // copy burst data from the respective registers
    adns_begin();
    // send adress of the register, with MSBit = 1 to indicate it's a write
    SPI_TransferByte(Motion_Burst & 0x7f);
    uint8_t burst_data[pixel_sum];
    for (int i = 0; i < pixel_sum; ++i) {
        burst_data[i] = SPI_TransferByte(0);
    }
    delta_x += convertDeltaToInt(burst_data[delta_x_h], burst_data[delta_x_l]);
    delta_y += convertDeltaToInt(burst_data[delta_y_h], burst_data[delta_y_l]);
    adns_end();
}
