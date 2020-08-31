#ifndef HCMS_H_ /* include guard */
#define HCMS_H_

#define CTRL_WORD_0_MASK 	0x7F
#define CTRL_WORD_1_MASK 	0xFF

#define SLEEP_MODE_MASK		0xBF
#define NORM_MODE_MASK		0x40

#define PEAK_BRT_31			0x20
#define PEAK_BRT_50			0x10
#define PEAK_BRT_73			0x00
#define PEAK_BRT_100		0x30



SPI_Bus SPI_Bus_1;
GPIO_InitTypeDef gResetPin;
GPIO_InitTypeDef gRSPin;
GPIO_InitTypeDef gBlankPin;

void init_reset_pin();
void init_reg_pin();
void init_blank_pin();
void init_display();

void write_display(uint16_t bytes);
void update_display();
void set_brightness(float percent);

#endif /* HCMS_H_ */
