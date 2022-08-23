#ifdef USE_LCD
#   include <U8g2lib.h>

void initLCD(int contrast,int brightness);
void updateLCDScreen(); 
void setLCDBrightness(int brightness);
void setLCDContrast(int contrast);

enum LCD_WIFI_STATUS { LCD_WIFI_DISABLED, LCD_WIFI_DISCONNECTED, LCD_WIFI_CONNECTED_LOW, LCD_WIFI_CONNECTED_HIGH };
enum LCD_AP_STATUS { LCD_AP_OFF, LCD_AP_ON };


U8G2_ST7565_ERC12864_ALT_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ LCD_CS, /* dc=*/ LCD_DC, /* reset=*/ LCD_RESET);  // contrast improved version for ERC12864

LCD_WIFI_STATUS lcd_wifi_status=LCD_WIFI_DISCONNECTED;
LCD_AP_STATUS lcd_ap_status=LCD_AP_OFF;

char lcd_text[20]="";
char lcd_hvb_SoH[10]="";
char lcd_hvb_SoC[10]="";
char lcd_hvb_coolant_temp[10]="";
char lcd_hvb_v[10];
char lcd_hvb_a[10]="";
char lcd_hvb_w[10]="";
char lcd_gear_pos[5]="";
char lcd_dc_dc_temp[10]="";
char lcd_dc_dc_v[10]="";
char lcd_dc_dc_a[10]="";
char lcd_dc_dc_w[10]="";
char lcd_motor_temp[10]="";

#endif
