
void write_to_lcd_4bit(uint8_t rs, uint8_t data)
{
	uint8_t i;
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, rs);
	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, 0);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 1);

	HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, ((data >> 3) & 0x01));
	HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, ((data >> 2) & 0x01));
	HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, ((data >> 1) & 0x01));
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, ((data >> 0) & 0x01));

	for (i = 0; i < 48; i++)
	{
		__asm("NOP");
	}
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 0);
	HAL_Delay(2);
}

void lcd_send_command(uint8_t data)
{
	write_to_lcd_4bit(0, ((data >> 4) & 0x0f));
	write_to_lcd_4bit(0, ((data >> 0) & 0x0f));
}

void lcd_display_char(uint8_t data)
{
	write_to_lcd_4bit(1, ((data >> 4) & 0x0f));
	write_to_lcd_4bit(1, ((data >> 0) & 0x0f));
}

void lcd_display_string(uint8_t *data, uint8_t len)
{
	for (uint8_t j = 0; j < len; j++)
	{
		lcd_display_char(data[j]);
	}
}

void lcd_goto_xy(uint8_t row, uint8_t col)
{
	uint8_t temp;
	switch (row)
	{
	case 0:
		temp = col | 0x80;
		break;
	case 1:
		temp = col | 0xc0;
		break;
	default:
		temp = col | 0x80;
	}
	lcd_send_command(temp);
}

void lcd_clear()
{
	lcd_send_command(0x01);
	HAL_Delay(20);
}

void lcd_init(void)
{

	lcd_send_command(0x28); // gui che do 4 bit 2line
	HAL_Delay(50);
	lcd_send_command(0x28); // gui che do 4 bit 2line
	HAL_Delay(50);
	lcd_send_command(0x0C); // hien thi lcd tat con tro
	HAL_Delay(1);
	lcd_send_command(0x06); // che do tu tang con tro
	HAL_Delay(1);
	lcd_send_command(0x01); // xoa man hinh
	HAL_Delay(20);
}

void ghiKiTuDacBiet(uint8_t *data, uint8_t vitri) {
	send_command(0x40 + (vitri * 8));
	HAL_Delay(1);
	for (uint8_t x = 0; x < 8; x++) {
		display_lcd(data[x]);
	}
}