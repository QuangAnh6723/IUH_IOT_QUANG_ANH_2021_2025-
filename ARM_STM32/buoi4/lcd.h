void write_to_lcd_4bit(uint8_t rs, uint8_t data);
void lcd_send_command(uint8_t data);
void lcd_display_char(uint8_t data);
void lcd_display_string(uint8_t *data, uint8_t len);
void lcd_goto_xy(uint8_t row, uint8_t col);
void lcd_clear();
void lcd_init(void);
