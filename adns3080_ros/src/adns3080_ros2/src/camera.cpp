#include "adns3080_ros2/camera.h"
#include <bcm2835.h>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <cmath>

// Motion data structure (should also be in camera.h)
struct MD {
    int motion;
    signed char dx, dy;
    int squal;
    int shutter;
    uint8_t max_pix;
};

double computeConversionFactor() {
    // Convert FOV to radians
    double fov_rad = FOV * M_PI / 180.0;

    // Compute the conversion factor in mm/pixel
    double cf = (2.0 * HEIGHT * std::tan(fov_rad / 2.0)) / (RESOLUTION * MOTION_SCALE);

    return cf;  // in mm/pixel
}


void mousecam_reset()
{
    bcm2835_gpio_write(PIN_MOUSECAM_RESET, HIGH);
    bcm2835_delayMicroseconds(1000); // >10us pulse
    bcm2835_gpio_write(PIN_MOUSECAM_RESET, LOW);
    bcm2835_delayMicroseconds(35000); // 35ms reset to functional
    printf("Mousecam reset\n");
    std::cout<<"MOUSCAM INIT"<<std::endl<<std::flush;
}

int mousecam_read_reg(int reg)
{
    bcm2835_gpio_write(PIN_MOUSECAM_CS, LOW);
    bcm2835_spi_transfer(reg);
    bcm2835_delayMicroseconds(75);
    int ret = bcm2835_spi_transfer(0xff);
    bcm2835_gpio_write(PIN_MOUSECAM_CS, HIGH);
    bcm2835_delayMicroseconds(1);
    return ret;
}

void mousecam_write_reg(int reg, int val)
{
    bcm2835_gpio_write(PIN_MOUSECAM_CS, LOW);
    bcm2835_spi_transfer(reg | 0x80);  // Set MSB for write
    bcm2835_spi_transfer(val);
    bcm2835_gpio_write(PIN_MOUSECAM_CS, HIGH);
    bcm2835_delayMicroseconds(50);
}

int mousecam_init()
{
    //bcm2835_gpio_fsel(PIN_MOUSECAM_RESET, BCM2835_GPIO_FSEL_OUTP);
    //bcm2835_gpio_fsel(PIN_MOUSECAM_CS, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(PIN_MOUSECAM_RESET, 0b001);
  	bcm2835_gpio_fsel(PIN_MOUSECAM_CS, 0b001);
    bcm2835_gpio_write(PIN_MOUSECAM_CS, HIGH);  // Deselect

    mousecam_reset();

    int pid = mousecam_read_reg(ADNS3080_PRODUCT_ID);
    //printf("Mousecam PID: %d\n", pid);
    std::cout<<"Mousecam PID:"<<pid <<std::endl<<std::flush;

    if (pid != ADNS3080_PRODUCT_ID_VAL)
        return -1;

    // Enable sensitive mode
    mousecam_write_reg(ADNS3080_CONFIGURATION_BITS, 0x10);

    return 0;
}

void mousecam_read_motion(struct MD *p)
{
    bcm2835_gpio_write(PIN_MOUSECAM_CS, LOW);
    bcm2835_spi_transfer(ADNS3080_MOTION_BURST);
    bcm2835_delayMicroseconds(75);

    p->motion  = bcm2835_spi_transfer(0xff);
    p->dx      = bcm2835_spi_transfer(0xff);
    p->dy      = bcm2835_spi_transfer(0xff);
    p->squal   = bcm2835_spi_transfer(0xff);
    p->shutter = bcm2835_spi_transfer(0xff) << 8;
    p->shutter |= bcm2835_spi_transfer(0xff);
    p->max_pix = bcm2835_spi_transfer(0xff);

    

    bcm2835_gpio_write(PIN_MOUSECAM_CS, HIGH);
    bcm2835_delayMicroseconds(5);
}


int setup()
{
	printf("Start setup\n");

  	if(!bcm2835_init())
  	{
    	printf("bcm2835_init failed. Are you running as root??\n");
    	return 1;
  	}

  	if (!bcm2835_spi_begin())
    	{
      		printf("bcm2835_spi_begin failed. Are you running as root??\n");
      		return 1 ;
    	}
  	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  	bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);
  	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_512);
  	bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
	
	mousecam_write_reg(0x7F, 0x00); // Max speed
	mousecam_write_reg(ADNS3080_CONFIGURATION_BITS, 0x10);
    
    mousecam_init();

  return 0 ;
}