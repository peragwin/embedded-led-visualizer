#include "stm32f7xx_hal.h"
#include "gpio.h"
#include "tim.h"
#include "fmc.h"
#include <ssd1289.h>
#include <string.h>
#include <math.h>

#define BYTE_SWAP(a) ((a << 8) | (a >> 8))

static void delay(uint16_t d) {
    while(d--) asm volatile ("nop");
}

uint16_t RGB565(uint8_t R,uint8_t G,uint8_t B) {
	return ((R >> 3) << 11) | ((G >> 2) << 5) | (B >> 3);
}

void LCD_write_command(uint16_t cmd) {
	LCD_CMD->REG = BYTE_SWAP(cmd);
	delay(10);
}

void LCD_write_data(uint16_t data) {
	LCD_DATA->REG = BYTE_SWAP(data);
	delay(10);
}

void LCD_write_data_fast(uint16_t data, uint32_t count) {
    while (count--) {
        LCD_DATA->REG = BYTE_SWAP(data);
        //delay(10);
    }
}

uint16_t LCD_read_data(void) {
	uint16_t ret = LCD_DATA->REG;
    return BYTE_SWAP(ret);
}

void LCD_WriteReg(uint16_t reg, uint16_t data) {
	LCD_write_command(reg);
	LCD_write_data(data);
}

uint16_t LCD_ReadReg(uint16_t reg) {
	LCD_write_command(reg);
	return LCD_read_data();
}

void LCD_Reset(void) {
	HAL_GPIO_WritePin(LCD_GPIO, LCD_RST, GPIO_PIN_SET);
	HAL_Delay(3); // 1ms by datasheet
	HAL_GPIO_WritePin(LCD_GPIO, LCD_RST, GPIO_PIN_RESET);
	HAL_Delay(3); // 1ms by datasheet
	HAL_GPIO_WritePin(LCD_GPIO, LCD_RST, GPIO_PIN_SET);
	HAL_Delay(3); // 1ms by datasheet
}

void LCD_Backlight(uint8_t pct) {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void LCD_Init(void) {
	LCD_Reset();
    LCD_Backlight(100);

	HAL_Delay(20);

	// power supply setting
    // set R07h at 0021h (GON=1,DTE=0,D[1:0]=01)
    LCD_WriteReg(0x0007, 0x0021);
    // set R00h at 0001h (OSCEN=1)
    LCD_WriteReg(0x0000, 0x0001);
    // set R07h at 0023h (GON=1, DTE=0, D[1:0]=11)
    LCD_WriteReg(0x0007, 0x0023);
    // set R10h at 0000h (Exit sleep mode)
    LCD_WriteReg(0x0010, 0x0000);
    // Wait 30ms
    HAL_Delay(30);
    // set R07h at 0033h (GON=1, DTE=1, D[1:0]=11)
    LCD_WriteReg(0x0007, 0x0033);
    // Entry mode setting (R11h)
    // R11H Entry mode
    // vsmode DFM1 DFM0 TRANS OEDef WMode DMode1 DMode0 TY1 TY0 ID1 ID0 AM LG2 LG2 LG0
    //   0     1    1     0     0     0     0      0     0   1   1   1  *   0   0   0
    // 14..13.DFM[1, 0] = (11 => 65k Color; 10 => 256k Color)
	// 07..06.TY[1, 0] = mode for 264k over 16 wire
    // 05..04.ID[1, 0] = (00 = hdec, vdec; 01 = hinc, vdec; 10 = hdec, vinc; 11 = hinc, vinc
    // 03.AM = 0 for horizontal; AM = 1 for vertical
    LCD_WriteReg(0x0011, (1<<14)|(0<<13)|(0<<6)|(1<<5)|(1<<4)|(1<<3));
    // LCD driver AC setting (R02h)
    LCD_WriteReg(0x0002, 0x0600);
    // power control 1
    // DCT3 DCT2 DCT1 DCT0 BT2 BT1 BT0 0 DC3 DC2 DC1 DC0 AP2 AP1 AP0 0
    // 1     0    1    0    1   0   0  0  1   0   1   0   0   1   0  0
    // DCT[3:0] fosc/4 BT[2:0]  DC{3:0] fosc/4
    LCD_WriteReg(0x0003, 0xA8A4);//0xA8A4 // i had 0804??
    LCD_WriteReg(0x000C, 0x0000);//
    LCD_WriteReg(0x000D, 0x080C);// 0x080C --> 0x0808
    // power control 4
    // 0 0 VCOMG VDV4 VDV3 VDV2 VDV1 VDV0 0 0 0 0 0 0 0 0
    // 0 0   1    0    1    0    1    1   0 0 0 0 0 0 0 0
    LCD_WriteReg(0x000E, 0x2900);
    LCD_WriteReg(0x001E, 0x00B8);
    // Driver output control
    // 0 RL REV CAD BGR SM TB MUX8 MUX7 MUX6 MUX5 MUX4 MUX3 MUX2 MUX1 MUX0
    // 0 0  1    0   1  0  1  1     0    0    1    1    1    1    1    1
    // 14.RL  = Output shift direction of Source driver (1 = S0->S719; 0 = S719->S0)
    // 13.REV = Grayscale mode (1 = normal; 0 = inverted)
    // 11.BGR = Components order(1 = BGR; 0 = RGB)
    // 09.TB  = Output shift direction of Gate driver (1 = G0->G319; 0 = G319->G0)
    // 08..00.MUX = Number of lines (0..319)
    LCD_WriteReg(0x0001,(1<<14)|(1<<13)|(1<<11)|(1<<9)|(1<<8)|(1<<5)|(1<<4)|(1<<3)|(1<<2)|(1<<1)|(1<<0));
    LCD_WriteReg(0x0010, 0x0000);
    LCD_WriteReg(0x0005, 0x0000);
    LCD_WriteReg(0x0006, 0x0000);
    LCD_WriteReg(0x0016, 0xEF1C);
    LCD_WriteReg(0x0017, 0x0003);
    LCD_WriteReg(0x0007, 0x0233);
    LCD_WriteReg(0x000B, 0x0000|(3<<6));
    LCD_WriteReg(0x000F, 0x0000);
    LCD_WriteReg(0x0041, 0x0000);
    LCD_WriteReg(0x0042, 0x0000);
    LCD_WriteReg(0x0048, 0x0000);
    LCD_WriteReg(0x0049, 0x013F);
    LCD_WriteReg(0x004A, 0x0000);
    LCD_WriteReg(0x004B, 0x0000);
    LCD_WriteReg(0x0044, 0xEF00);
    LCD_WriteReg(0x0045, 0x0000);
    LCD_WriteReg(0x0046, 0x013F);
    // Gamma control
    LCD_WriteReg(0x0030, 0x0707);
    LCD_WriteReg(0x0031, 0x0204);
    LCD_WriteReg(0x0032, 0x0204);
    LCD_WriteReg(0x0033, 0x0502);
    LCD_WriteReg(0x0034, 0x0507);
    LCD_WriteReg(0x0035, 0x0204);
    LCD_WriteReg(0x0036, 0x0204);
    LCD_WriteReg(0x0037, 0x0502);
    LCD_WriteReg(0x003A, 0x0302);
    LCD_WriteReg(0x003B, 0x0302);
    // Gamma control end
    LCD_WriteReg(0x0023, 0x0000);
    LCD_WriteReg(0x0024, 0x0000);
    LCD_WriteReg(0x0025, 0xE000);   // 65hz = 0x8000, 80hz = 0xE000
    LCD_WriteReg(0x004f, 0);
    LCD_WriteReg(0x004e, 0);
}

void LCD_SetCursor(uint16_t X, uint16_t Y) {
	LCD_WriteReg(0x004e,Y);
	LCD_WriteReg(0x004f,X);
}

void LCD_Pixel(uint16_t X, uint16_t Y, uint16_t C) {
	LCD_SetCursor(X,Y);
	LCD_WriteReg(0x0022,C);
}

void LCD_SetWindow(uint16_t X, uint16_t Y, uint16_t W, uint16_t H) {
	uint16_t XW = X + W - 1;
	uint16_t YH = Y + H - 1;
	LCD_WriteReg(0x0045,X);           // Top RAM address pos
	LCD_WriteReg(0x0046,XW);          // Bottom RAM address pos
	LCD_WriteReg(0x0044,(YH << 8) + (Y & 0x00ff)); // Left and Right RAM address pos
	LCD_SetCursor(X,Y);
}

void LCD_Clear(uint16_t C) {
	LCD_SetWindow(0,0,320,240);
	LCD_write_command(0x0022);
	//for (i = 0; i < 320*240; i++) { LCD_write_data(C); }
	LCD_write_data_fast(C, 320*240);
}

void LCD_HLine(uint16_t X0, uint16_t X1, uint16_t Y, uint16_t Color) {
	uint16_t W = X1 - X0 + 1;
    uint16_t i;

    LCD_SetWindow(X0,Y,W,1);
	LCD_write_command(0x0022);
	for (i = 0; i < W; i++) { LCD_write_data(Color); }
}

void LCD_VLine(uint16_t X, uint16_t Y0, uint16_t Y1, uint16_t Color) {
	uint16_t H = Y1 - Y0 + 1;
	uint16_t i;

	LCD_SetWindow(X,Y0,1,H);
	LCD_write_command(0x0022);
	for (i = 0; i < H; i++) { LCD_write_data(Color); }
}

void LCD_Line(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint16_t Color) {
	int16_t dX = X2 - X1;
	int16_t dY = Y2 - Y1;
	int16_t dXsym = (dX > 0) ? 1 : -1;
	int16_t dYsym = (dY > 0) ? 1 : -1;

	if (dX == 0) {
		if (Y2>Y1) LCD_VLine(X1,Y1,Y2,Color); else LCD_VLine(X1,Y2,Y1,Color);
		return;
	}
	if (dY == 0) {
		if (X2>X1) LCD_HLine(X1,X2,Y1,Color); else LCD_HLine(X2,X1,Y1,Color);
		return;
	}

	LCD_SetWindow(0,0,320,240);

	dX *= dXsym;
	dY *= dYsym;
	int16_t dX2 = dX << 1;
	int16_t dY2 = dY << 1;
	int16_t di;

	if (dX >= dY) {
		di = dY2 - dX;
		while (X1 != X2) {
			LCD_Pixel(X1,Y1,Color);
			X1 += dXsym;
			if (di < 0) {
				di += dY2;
			} else {
				di += dY2 - dX2;
				Y1 += dYsym;
			}
		}
	} else {
		di = dX2 - dY;
		while (Y1 != Y2) {
			LCD_Pixel(X1,Y1,Color);
			Y1 += dYsym;
			if (di < 0) {
				di += dX2;
			} else {
				di += dX2 - dY2;
				X1 += dXsym;
			}
		}
	}
	LCD_Pixel(X1,Y1,Color);
}

#define ipart_(X) ((int)(X))
#define round_(X) ((int)(((float)(X))+0.5))
#define fpart_(X) (((float)(X))-(float)ipart_(X))
#define rfpart_(X) (1.0-fpart_(X))
#define swap_(a, b) do{ __typeof__(a) tmp;  tmp = a; a = b; b = tmp; }while(0)

static void dla_plot(int x, int y, uint8_t r, uint8_t g, uint8_t b, float br) {
	r = (uint8_t)round_(br*r);
	g = (uint8_t)round_(br*g);
	b = (uint8_t)round_(br*b);

	LCD_Pixel(x,y,RGB565(r,g,b));
}

void LCD_LineAA(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint16_t Color) {
	float dX = (float)X2 - (float)X1;
	float dY = (float)Y2 - (float)Y1;

	uint8_t R,G,B;

	R = (uint8_t)(Color >> 11) << 3;
	G = (uint8_t)((Color >> 5) & 0x3f) << 2;
	B = (uint8_t)(Color & 0x1f) << 3;

	if (dX == 0) { if (Y2>Y1) LCD_VLine(X1,Y1,Y2,Color); else LCD_VLine(X1,Y2,Y1,Color); return; }
	if (dY == 0) { if (X2>X1) LCD_HLine(X1,X2,Y1,Color); else LCD_HLine(X2,X1,Y1,Color); return; }

	LCD_SetWindow(0,0,320,240);

	float dx = (float)X2 - (float)X1;
	float dy = (float)Y2 - (float)Y1;
	if (fabs(dx) > fabs(dy)) {
		if (X2 < X1) { swap_(X1,X2); swap_(Y1,Y2); }
		float gradient = dy / dx;
		float xend = round_(X1);
		float yend = Y1 + gradient*(xend - X1);
		float xgap = rfpart_(X1 + 0.5f);
		int xpxl1 = xend;
		int ypxl1 = ipart_(yend);
		dla_plot(xpxl1,ypxl1,R,G,B,rfpart_(yend)*xgap);
		dla_plot(xpxl1,ypxl1+1,R,G,B,fpart_(yend)*xgap);
		float intery = yend + gradient;

		xend = round_(X2);
		yend = Y2 + gradient*(xend - X2);
		xgap = fpart_(X2+0.5f);
		int xpxl2 = xend;
		int ypxl2 = ipart_(yend);
		dla_plot(xpxl2,ypxl2,R,G,B,rfpart_(yend)*xgap);
		dla_plot(xpxl2,ypxl2 + 1,R,G,B,fpart_(yend)*xgap);

		int x;
		for (x = xpxl1+1; x <= (xpxl2-1); x++) {
			dla_plot(x,ipart_(intery),R,G,B,rfpart_(intery));
			dla_plot(x,ipart_(intery)+1,R,G,B,fpart_(intery));
			intery += gradient;
		}
	} else {
		if ( Y2 < Y1 ) { swap_(X1,X2); swap_(Y1,Y2); }
		float gradient = dx / dy;
		float yend = round_(Y1);
		float xend = X1 + gradient*(yend - Y1);
		float ygap = rfpart_(Y1+0.5f);
		int ypxl1 = yend;
		int xpxl1 = ipart_(xend);
		dla_plot(xpxl1,ypxl1,R,G,B,rfpart_(xend)*ygap);
		dla_plot(xpxl1,ypxl1+1,R,G,B,fpart_(xend)*ygap);
		float interx = xend + gradient;

		yend = round_(Y2);
		xend = X2 + gradient*(yend - Y2);
		ygap = fpart_(Y2+0.5f);
		int ypxl2 = yend;
		int xpxl2 = ipart_(xend);
		dla_plot(xpxl2,ypxl2,R,G,B,rfpart_(xend)*ygap);
		dla_plot(xpxl2,ypxl2+1,R,G,B,fpart_(xend)*ygap);

		int y;
		for(y=ypxl1+1; y <= (ypxl2-1); y++) {
			dla_plot(ipart_(interx),y,R,G,B,rfpart_(interx));
			dla_plot(ipart_(interx)+1,y,R,G,B,fpart_(interx));
			interx += gradient;
		}
	}
}

void LCD_Rect(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, uint16_t Color) {
	LCD_HLine(X,X+W-1,Y,Color);
	LCD_HLine(X,X+W-1,Y+H-1,Color);
	LCD_VLine(X,Y,Y+H-1,Color);
	LCD_VLine(X+W-1,Y,Y+H-1,Color);
}

void LCD_FillRect(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, uint16_t Color) {
	uint32_t i = 0;
	LCD_SetWindow(X,Y,W,H);
	LCD_write_command(0x0022);
	for (i = 0; i < W*H; i++) { LCD_write_data(Color); }
}

void LCD_FillRect264(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, color_t color) {
	uint32_t i = 0;
	LCD_SetWindow(X,Y,W,H);
	uint32_t len = W*H;
	len = len / 2;
	LCD_write_command(0x0022);
	uint16_t even_write;
	uint16_t odd_write;
	uint16_t third_write;
	for (i = 0; i < len; i++) {
		//if (i%2 == 0) {
			even_write = (color.r << 10) | ((color.g <<2) & 0xff);
			odd_write = (color.b << 10) | ((color.r <<2) & 0xff);
		//} else {
			third_write = (color.g << 10) | ((color.b <<2) & 0xff);
		//	odd_write = color.r << 10 | ((color.g <<2) & 0xff);
		//}
		LCD_write_data(even_write);
		LCD_write_data(odd_write);
		LCD_write_data(third_write);
	}
}

void LCD_Ellipse(uint16_t X, uint16_t Y, uint16_t A, uint16_t B, uint16_t Color) {
	LCD_SetWindow(0,0,320,240);

	int16_t Xc = 0, Yc = B;
	long A2 = (long)A*A, B2 = (long)B*B;
	long C1 = -(A2/4 + A % 2 + B2);
	long C2 = -(B2/4 + B % 2 + A2);
	long C3 = -(B2/4 + B % 2);
	long t = -A2*Yc;
	long dXt = B2*Xc*2, dYt = -A2*Yc*2;
	long dXt2 = B2*2, dYt2 = A2*2;
	while (Yc >= 0 && Xc <= A) {
		LCD_Pixel(X+Xc,Y+Yc,Color);
		LCD_Pixel(X-Xc,Y+Yc,Color);
		LCD_Pixel(X+Xc,Y-Yc,Color);
		LCD_Pixel(X-Xc,Y-Yc,Color);
		if (t + Xc*B2 <= C1 || t + Yc*A2 <= C3) {
			Xc++;
			dXt += dXt2;
			t   += dXt;
		} else if (t - Yc*A2 > C2) {
			Yc--;
			dYt += dYt2;
			t   += dYt;
		} else {
			Xc++;
			Yc--;
			dXt += dXt2;
			dYt += dYt2;
			t   += dXt;
			t   += dYt;
		}
	}
}

void LCD_FillEllipse(uint16_t X, uint16_t Y, uint16_t A, uint16_t B, uint16_t Color) {
	LCD_SetWindow(0,0,320,240);

	int16_t Xc = 0, Yc = B;
	long A2 = (long)A*A, B2 = (long)B*B;
	long C1 = -(A2/4 + A % 2 + B2);
	long C2 = -(B2/4 + B % 2 + A2);
	long C3 = -(B2/4 + B % 2);
	long t = -A2*Yc;
	long dXt = B2*Xc*2, dYt = -A2*Yc*2;
	long dXt2 = B2*2, dYt2 = A2*2;
	while (Yc >= 0 && Xc <= A) {
		LCD_HLine(X-Xc,X+Xc,Y+Yc,Color);
		LCD_HLine(X-Xc,X+Xc,Y-Yc,Color);
		if (t + Xc*B2 <= C1 || t + Yc*A2 <= C3) {
			Xc++;
			dXt += dXt2;
			t   += dXt;
		} else if (t - Yc*A2 > C2) {
			Yc--;
			dYt += dYt2;
			t   += dYt;
		} else {
			Xc++;
			Yc--;
			dXt += dXt2;
			dYt += dYt2;
			t   += dXt;
			t   += dYt;
		}
	}
}

void LCD_PutChar(uint16_t X, uint16_t Y, uint8_t Char, uint16_t Color) {
	uint16_t i,j;
	uint8_t buffer[16],tmpCh;

    LCD_SetWindow(X,Y,8,16);
    memcpy(buffer,AsciiLib[Char-32],16);

    for (i = 0; i < 16; i++) {
    	tmpCh = buffer[i];
    	for (j = 0; j < 8; j++) {
    		if (((tmpCh >> (7-j)) & 0x01) == 0x01) LCD_Pixel(X+j,Y+i,Color);
    	}
    }
}

void LCD_PutCharO(uint16_t X, uint16_t Y, uint8_t Char, uint16_t Color, uint16_t bgColor) {
	uint16_t i,j;
	uint8_t buffer[16],tmpCh;

    LCD_SetWindow(X,Y,8,16);
    memcpy(buffer,AsciiLib[Char-32],16);

    for (i = 0; i < 16; i++) {
    	tmpCh = buffer[i];
    	for (j = 0; j < 8; j++) LCD_Pixel(X+j,Y+i,(((tmpCh >> (7-j)) & 0x01) == 0x01) ? Color : bgColor);
    }
}

void LCD_PutStr(uint16_t X, uint16_t Y, char *str, uint16_t Color) {
    while (*str) {
        LCD_PutChar(X,Y,*str++,Color);
        if (X < 320-8) { X += 8; } else if (Y < 240-16) { X = 0; Y += 16; } else { X = 0; Y = 0; }
    };
}

void LCD_PutStrO(uint16_t X, uint16_t Y, char *str, uint16_t Color, uint16_t bgColor) {
    while (*str) {
        LCD_PutCharO(X,Y,*str++,Color,bgColor);
        if (X < 320-8) { X += 8; } else if (Y < 240-16) { X = 0; Y += 16; } else { X = 0; Y = 0; }
    }
}

void LCD_PutInt(uint16_t X, uint16_t Y, uint32_t num, uint16_t Color) {
	char str[11]; // 10 chars max for UINT32_MAX
	int i = 0;
	do { str[i++] = num % 10 + '0'; } while ((num /= 10) > 0);
	int strLen = i;
	for (i--; i>=0; i--) LCD_PutChar(X+(strLen << 3)-(i << 3),Y,str[i],Color);
}

void LCD_PutIntO(uint16_t X, uint16_t Y, uint32_t num, uint16_t Color, uint16_t bgColor) {
	char str[11]; // 10 chars max for UINT32_MAX
	int i = 0;
	do { str[i++] = num % 10 + '0'; } while ((num /= 10) > 0);
	int strLen = i;
	for (i--; i>=0; i--) LCD_PutCharO(X+(strLen << 3)-(i << 3),Y,str[i],Color,bgColor);
}

void LCD_PutHex(uint16_t X, uint16_t Y, uint32_t num, uint16_t Color) {
	char str[11]; // 10 chars max for UINT32_MAX
	int i = 0;
	do { str[i++] = "0123456789ABCDEF"[num % 0x10]; } while ((num /= 0x10) > 0);
	str[i++] = 'x';
	str[i++] = '0';
	int strLen = i;
	for (i--; i>=0; i--) LCD_PutChar(X+(strLen << 3)-(i << 3),Y,str[i],Color);
}

void LCD_PutHexO(uint16_t X, uint16_t Y, uint32_t num, uint16_t Color, uint16_t bgColor) {
	char str[11]; // 10 chars max for UINT32_MAX
	int i = 0;
	do { str[i++] = "0123456789ABCDEF"[num % 0x10]; } while ((num /= 0x10) > 0);
	str[i++] = 'x';
	str[i++] = '0';
	int strLen = i;
	for (i--; i>=0; i--) LCD_PutCharO(X+(strLen << 3)-(i << 3),Y,str[i],Color,bgColor);
}

void LCD_BMPMono(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, const uint8_t* pBMP, uint16_t Color) {
	uint16_t i,j,k;
	uint8_t buffer[W];

	LCD_SetWindow(X,Y,W*8,H);
	for (i = 0; i < H; i++) {
		memcpy(buffer,&pBMP[i*W],W);
		for (j = 0; j < W; j++) {
			for (k = 0; k < 8; k++) if ((buffer[j] >> (7-k)) & 0x01) LCD_Pixel(X+(j << 3)+k,Y+i,Color);
		}
	}
}

void LCD_BMPMonoO(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, const uint8_t* pBMP, uint16_t Color, uint16_t bgColor) {
	uint16_t i,j,k;
	uint8_t buffer[W];

	LCD_SetWindow(X,Y,W*8,H);
	for (i = 0; i < H; i++) {
		memcpy(buffer,&pBMP[i*W],W);
		for (j = 0; j < W; j++) {
			for (k = 0; k < 8; k++) LCD_Pixel(X+(j << 3)+k,Y+i,((buffer[j] >> (7-k)) & 0x01) ? Color : bgColor);
		}
	}
}

void LCD_BMP(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, const uint16_t* pBMP) {
	uint16_t i,j;
	uint16_t buffer[W];

	LCD_SetWindow(X,Y,W,H);
	LCD_write_command(0x0022);
	for (i = 0; i < H; i++) {
		memcpy(buffer,&pBMP[i*W],W*2);
		for (j = 0; j < W; j++) LCD_write_data(buffer[j]);
	}
}

// void LCD_SetBuffer(buffer_t *buffer, uint16_t x, uint16_t y, color_t color) {
// 	int i = 3*(320*y + x);
// 	buffer[i] = color.r;
// 	buffer[i+1] = color.g;
// 	buffer[i+2] = color.b;
// }

//  void LCD_SetBuffer(buffer_t *buffer, uint16_t x, uint16_t y, color_t color) {
// 	int i = 320*y + x;
// 	int is_even = i % 2 == 0;

// 	if (is_even) {
// 		// RRRRRRxx | GGGGGGxx
// 		// BBBBBBxx |
// 		i = 3*i / 2;
// 		buffer[i]   = (color.r << 8) | color.g;
// 		buffer[i+1] = (color.b << 8) | ((buffer[i+1] & 0xFF));
// 	} else {
// 		//          | RRRRRRxx
// 		// GGGGGGxx | BBBBBBxx
// 		i = 3*i / 2 + 1;
// 		buffer[i-1] = (buffer[i-1] & 0xFF00) | (color.r);
// 		buffer[i]   = (color.g << 8)         | (color.b);
// 	}
// }

// char String1[50];

// void LCD_Buffer_DMA(uint8_t *buffer, uint16_t X, uint16_t Y, uint16_t W, uint16_t H) {
// 	LCD_SetWindow(X, Y, W, H);
// 	LCD_write_command(0x0022);

// 	uint32_t length = (uint32_t) W * (uint32_t) H;
// 	length = 3*length/2 + (length%2);

// 	uint32_t start = 320*Y + X;
	
// 	uint8_t *frame = buffer + 3*start;

	
// 	DMA_Transfer(frame, length);
	

// // 	uint32_t cnt = htimy.Instance->CNT;
// //   sprintf(String1, "TIMy CNT is: %8lx", cnt);
// //   LCD_FillRect(10+8*12, 88, 8*10, 18, RGB565(127, 0, 192));
// //   LCD_PutStr(10, 90, String1, RGB565(255, 0, 0));
// //   if (et == 1)
// //     LCD_PutStr(10, 110, "DMA Transfer SUCESS!!", RGB565(92, 255, 92));
// //   else if (et == 2)
// //     LCD_PutStr(10, 110, "DMA Transfer FAILURE!! :(", RGB565(255, 92, 92));
// }