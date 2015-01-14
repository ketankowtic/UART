/*******************************************************************************
 * 1. Inserting the driver module in to the kernel by giving its .ko file
 *    (i.e) insmod ulk_fpga_uart.ko will inserts the kernel module (.ko) 
 *    in to the kernel.
 * 
 * 2. Removing the module form the kernel can be done by giving the module name 
 *    loaded (i.e) rmmod ulk_fpga_uart will unloads the ulk_fpga_uart
 *    module form the kernel.
 * 
 * 3. Run the Application and do the operation
*******************************************************************************/

/*=========================================================================================================================*/


#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>	
#include <linux/module.h>	
#include <linux/fs.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <linux/blkdev.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#include <linux/delay.h>
#include "include/ulk_fpga_gpmc.h" 
#include "include/ulk_fpga_uart.h" 

/*=========================================================================================================================*/

#define  UART_BAUD_RATE	 		 1
#define  UART_FIFO_CONTROL               2
#define  UART_SETTING_DATA_PARITY        3
#define	 UART_READ		  	 4
#define	 UART_WRITE   		  	 5

#define  GPMC_START_ADDRESS        0x04000000
#define  GPMC_SIZE                 0x01000000
#define  GPMC_CHIP_SELECT          0x30

#define  FPGA_UART_DEVICE_NAME     "fpga_uart"

#define  MAKE_POLARITY_LOW         (1 << 11)
#define  ENABLE                    (1 << 1)

#define  SUCCESS 	           0 
#define  CALIBRATED_DELAY          5000

/*=========================================================================================================================*/

static int fpga_uart_driver = 0;
static struct class *fpga_uart_class;
int bauddiv;
int copy;
char ulk_status = 'n';
struct ioctl_parameters{
       unsigned char buff;
		 int data_bits;
		 int stop_bits;
		char parity_enable;
		char parity_type;
		char fifo_enable;
		char ulk_connected;
		int  data;
		int  baud_rate;
};


static int ulk_fpga_uart_device_open(struct inode *, struct file *);
static int ulk_fpga_uart_device_release(struct inode *, struct file *);
int ulk_fpga_uart_device_ioctl(struct inode *inode, struct file *file, unsigned int ioctl_num, struct ioctl_parameters *);

/*=========================================================================================================================*/

/*Initialize the FPGA UART */
unsigned char ulk_fpga_uart_init( int );   /*use default configuration */

/*Get a character from FPGA UART */
unsigned char ulk_fpga_uart_getc(void);

/*Write a character from FPGA UART */
unsigned char ulk_fpga_uart_putc(unsigned char val);

/*Get a character from FPGA UART */
unsigned char  NS16550_putc_fpga_uart ( char );

/*Write a character from FPGA UART */
unsigned char NS16550_getc_fpga_uart (void);

/* Set clock and baud rate */
static int calc_divisor ( int );

/* Initialize the module  */
int ulk_init_fpga_uart_module(void);

/* Clean up the module */
void ulk_cleanup_fpga_uart_module(void);

/*GPMC initialization for FPGA */
void ulk_uart_gpmc_init (void);

/*GPMC configure for FPGA */
void ulk_uart_gpmc_config (u8 );

/*GPMC read version */
void ulk_uart_gpmc_read_version (u8 );

/*=========================================================================================================================*/

static ssize_t ulk_fpga_uart_device_read(struct file *filp,
   char *buffer,    /* The buffer to fill with data */
   size_t length,   /* The length of the buffer     */
   loff_t *offset);  /* Our offset in the file       */

static ssize_t ulk_fpga_uart_device_write(struct file *filp,
   const char *buff,
   size_t len,
   loff_t *off);

static int device_opened = 0;

extern void __iomem *gpmc_cs_address;
void __iomem *gpmc_cs_uart_address;

/*=========================================================================================================================*/
 
/*******************************************************************************
 * Module Declarations 
 * 
 * This structure will hold the functions to be calfpga_uart when a process does
 * something to the device we  created. Since a pointer to this structure is 
 * kept in  the devices table, it can't be local to  init_module. NULL is for 
 * unimplemented functions. 
 *
 ******************************************************************************/
struct file_operations Fops_uart = {
    .ioctl = ulk_fpga_uart_device_ioctl,
    .open = ulk_fpga_uart_device_open,
    .read = ulk_fpga_uart_device_read,
    .write = ulk_fpga_uart_device_write,
    .release = ulk_fpga_uart_device_release,	/* a.k.a. close */
};

/*=========================================================================================================================*/

/*******************************************************************************
 * 
 * Function Name        : ulk_init_module                            
 * Input Arguments      : None                                    
 * Return Value         : Zero for sucess                                    
 * Description          : This functin will call the initializes the 
 *                        module -Register the character device and
 *                        gpmc   
 *
 ******************************************************************************/
int __init ulk_init_fpga_uart_module()
{
	struct class_device *temp_class;
	int  err=0;

#if 0
	if (!request_mem_region(GPMC_START_ADDRESS, GPMC_SIZE, FPGA_UART_DEVICE_NAME)) 
	{
#ifdef FPGA_UART_DEBUG
	printk("gpmc_cs_uart_address mapped\n");
#endif
	}

	/*Remap adddress */
	gpmc_cs_uart_address = ioremap(GPMC_START_ADDRESS, GPMC_SIZE);
#endif
	
	gpmc_cs_uart_address = gpmc_cs_address;

	if (!gpmc_cs_uart_address) 
		return 1; //Failure

	/* 
	 * Register the character device (atleast try) 
	 * Passing the major number as 0 to get the device its own free major number
	 */

	fpga_uart_driver = register_chrdev(fpga_uart_driver, FPGA_UART_DEVICE_NAME, &Fops_uart);

	/* 
	 * Negative values signify an error 
	 */
	if (fpga_uart_driver < 0) {
		printk("%s failed with %d\n",
				"Sorry, registering the character device ", fpga_uart_driver);
		return fpga_uart_driver;
	}

	fpga_uart_class = class_create(THIS_MODULE, FPGA_UART_DEVICE_NAME);

	if (IS_ERR(fpga_uart_class)) {
		printk("ERROR wlile creating class create\n");
		err = PTR_ERR(fpga_uart_class);
	}

	temp_class = device_create(fpga_uart_class, NULL, MKDEV(fpga_uart_driver, 0), NULL, FPGA_UART_DEVICE_NAME);

	if (IS_ERR(temp_class)) {

		printk("ERROR wlile creating device create\n");
		err = PTR_ERR(temp_class);
	}

	/* GPMC Initialization */
	ulk_uart_gpmc_init ();


	return 0;
}

/*=========================================================================================================================*/

/*******************************************************************************
 * 
 * Function Name        : ulk_cleanup_module                            
 * Input Arguments      : None                                    
 * Return Value         : None                                 
 * Description          : Cleanup unregister the device  from  /proc/devices  
 *
 ******************************************************************************/

void __exit ulk_cleanup_fpga_uart_module()
{
	/* Unregister the device */

	if (fpga_uart_driver > 0) {

		device_destroy(fpga_uart_class, MKDEV(fpga_uart_driver, 0));
		class_destroy(fpga_uart_class);

#ifdef FPGA_UART_DEBUG
		printk (" Unregisters the fpga_uartment driver \r\n");
#endif
		unregister_chrdev(fpga_uart_driver, FPGA_UART_DEVICE_NAME);
		fpga_uart_driver = 0;
	}
}

/*=========================================================================================================================*/

/*******************************************************************************
 * 
 * Function Name        : ulk_fpga_uart_device_open                            
 * Input Arguments      : Node and file                                   
 * Return Value         : SUCCESS                                 
 * Description          : Called when a process tries to open the device file
 *
 ******************************************************************************/

static int ulk_fpga_uart_device_open(struct inode *inode, struct file *file)
{
	if (device_opened)
	{
		printk ("Device already opened and it is busy \n");
		return -EBUSY;
	}

	device_opened++;
	try_module_get(THIS_MODULE);

	return SUCCESS;
}

/*=========================================================================================================================*/

/*******************************************************************************
 *
 * Function Name        : ulk_fpga_uart_device_release                            
 * Input Arguments      : Node and file                                   
 * Return Value         : Zero                                 
 * Description          : Called when a process closes the device file
 *
 ******************************************************************************/

static int ulk_fpga_uart_device_release(struct inode *inode, struct file *file)
{
	device_opened--;	/* We're now ready for our next caller */

	/* 
	 * Decrement the usage count, or else once you opened the file, you'll
	 * never get get rid of the module. 
	 */
	module_put(THIS_MODULE);

	return 0;
}

/*=========================================================================================================================*/

/*******************************************************************************
 * 
 * Function Name        : ulk_fpga_uart_device_ioctl                            
 * Input Arguments      : Node and file                                   
 * Return Value         : Output of the function                                 
 * Description          : This function is called whenever a process tries to do 
 *        		  an ioctl on our device file. We get two extra 
 *			  parameters(additional to the inode and file structures
 *			  which all device functions get): the number of the 
 *		          ioctl called and the parameter given to the ioctl 
 *			  function.
 *
 ******************************************************************************/

int ulk_fpga_uart_device_ioctl(struct inode *inode,	
		 struct file *file,	
		 unsigned int ioctl_num,	
		 struct ioctl_parameters *ioctl)
{
	unsigned char buff = '\0';
	ulk_status = ioctl->ulk_connected;
	switch (ioctl_num) 
	{
		case UART_BAUD_RATE:  /* Setting the Baud Rate of the Data Stream to be Transmitted */
						
						ulk_fpga_uart_init(ioctl->baud_rate);

						break;

		case UART_FIFO_CONTROL: /* Setting the FIFO control Registers */

				if( ioctl->fifo_enable == 'y')
					{
						/* Clear and enable the FIFO control register */
						*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_FCR_REG) = FCRVAL;
					}
				else
					{
						/* Clear and disable the FIFO control register */
						*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_FCR_REG) = 0x00 ;
					}
					break;


		case UART_SETTING_DATA_PARITY : /* Setting the parity of the Data Stream to be Transmitted */

						*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_LCR_REG) = 0x00;

				switch(ioctl-> data_bits)
					{

						case 5: *(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_LCR_REG) = 0x00 ;  
							break;
								
						case 6: *(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_LCR_REG) = 0x01 ;
							break;

						case 7:	*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_LCR_REG) = 0x02 ;  	
							break;

						case 8:	*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_LCR_REG) = 0x03 ;  
							break;

						default:
							break;
					}

				if(ioctl->stop_bits == 1)
					{
						/* Clear 2nd bit of Line control register */
						*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_LCR_REG) &= 0xfb;
						//Clearing the LCR 2nd bit	
					}
			       else
					{
						/* Set 2nd bit of Line control register */
						*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_LCR_REG) &= 0xfb;  
						*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_LCR_REG) |= 0x04;
						//Setting the LCR 2nd bit
					}


				if(ioctl->parity_enable == 'y')
					{
						/* Set 3rd bit of Line control register */
						*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_LCR_REG) &= 0xf7;
						*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_LCR_REG) |= 0x08; 
						//Setting the LCR 3rd bit	
					}
			       else
					{
						/* Clear 3rd bit of Line control register */
						*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_LCR_REG) &= 0xf7;
						//Clearing the LCR 3rd bit	
					}

				if(ioctl->parity_type == 'e')
					{
						/* Set 4th bit of Line control register */
						*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_LCR_REG) &= 0xef;
						*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_LCR_REG) |= 0x10; 
						//Setting the LCR 4th bit	
					}
			       else
					{
						/* Clear 4th bit of Line control register */
						*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_LCR_REG) &= 0xef;
						//Clearing the LCR 4th bit	
					}

						break;

		case UART_READ: /* Read from fpga uart single character at a time */
			buff = ulk_fpga_uart_getc();
#ifdef FPGA_UART_DEBUG
			printk("Driver: Read char =%c::Hex = 0x%x int = %d\r\n",buff,buff,buff);
#endif
			copy = ioctl->data;
		
			ioctl->data = (unsigned char)buff;

						break;

		case UART_WRITE: /* Write to fpga uart single character at a time */

			buff = (char) ioctl->data + 0x30;			
#ifdef FPGA_UART_DEBUG
			printk("Driver: Entered char =%c::Hex = 0x%x int = %d\r\n",buff,buff,buff);
#endif
			ulk_fpga_uart_putc(buff);
		
			
						break;
		
		default:			
						break;
	}

				return buff;
}

/*=========================================================================================================================*/

/* Called when a process, which already opened the dev file, attempts to
   read from it.
*/
static ssize_t ulk_fpga_uart_device_read(struct file *filp,
   char *buffer,    /* The buffer to fill with data */
   size_t length,   /* The length of the buffer     */
   loff_t *offset)  /* Our offset in the file       */
{
	unsigned char ch;
	/* Number of bytes actually written to the buffer */
	int i = 0;
		/* Clear and enable the FIFO control register */
		*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_FCR_REG) = FCRVAL;
	for ( i=0;i<length;i++)
	{
       		ch = NS16550_getc_fpga_uart();
		if(ch == '\r')
			break;
                buffer[i] = ch;
        }
        buffer[i] = '\0';

	/* Most read functions return the number of bytes put into the buffer */
	return i;

}

/*=========================================================================================================================*/

/*  Called when a process writes to FPGA uart */
static ssize_t ulk_fpga_uart_device_write(struct file *filp,
   const char *buff,
   size_t len,
   loff_t *off)
{
	unsigned int i = 0;

	while((i < len) && (buff[i] != NULL))
	{
		ulk_fpga_uart_putc(buff[i]);
		i++;
	}
	ulk_fpga_uart_putc('\r');
	ulk_fpga_uart_putc('\n');
	/* Clear and enable the FIFO control register */
	*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_FCR_REG) = FCRVAL;
	if( ulk_status == 'n')
	{
	ulk_fpga_uart_putc('\n');			
	}	
	return i;
}

/*=========================================================================================================================*/

/*******************************************************************************
 * 
 * Function Name        : ulk_uart_gpmc_init                            
 * Input Arguments      : None                               
 * Return Value         : None                             
 * Description          : This function used to intialise the GPMC 
 *
 ******************************************************************************/

void ulk_uart_gpmc_init ()
{

    /* configure GPMC */
    ulk_uart_gpmc_config (4);
    ulk_uart_gpmc_read_version(4);

}

/*=========================================================================================================================*/

/*******************************************************************************
 *
 * Function Name        : ulk_uart_gpmc_config                          
 * Input Arguments      : None                               
 * Return Value         : None                             
 * Description          : This function used to configure the GPMC 
 *
 ******************************************************************************/

void 
ulk_uart_gpmc_config (u8 chip_sel)
{
    /*a26- A11 are not modified during external access */

    *(u32 *)GPMC_CONFIG |= ENABLE;
    *(u32 *)GPMC_CONFIG  &= ~MAKE_POLARITY_LOW; /*clearing the bit 11 the\
			polarity is active low*/
    *(u32 *)((GPMC_CONFIG1_i) + (GPMC_CHIP_SELECT * chip_sel)) = ( 1 << 22) |\
			(1 << 21) | ( 0x3 << 16) | (1 << 12);
    *(u32 *)((GPMC_CONFIG2_i) + (GPMC_CHIP_SELECT * chip_sel)) = (CSONTIME |\
			 CSRDOFFTIME | CSWROFFTIME) ;
    *(u32 *)((GPMC_CONFIG4_i) + (GPMC_CHIP_SELECT * chip_sel)) = (WEONTIME |\
			 WEOFFTIME | OEONTIME | OEOFFTIME);
    *(u32 *)((GPMC_CONFIG5_i) + (GPMC_CHIP_SELECT * chip_sel)) = \
			(RDACCESSTIME  | RDCYCLETIME | WRITECYCLETIME);
    *(u32 *)((GPMC_CONFIG6_i) + (GPMC_CHIP_SELECT * chip_sel)) =\
			 (WRITEACCESSTIME);
    *(u32 *)((GPMC_CONFIG7_i) + (GPMC_CHIP_SELECT * chip_sel)) = \
			(((OMAP34XX_GPMC_CS0_SIZE & 0xF)<<8) |\
			( (CS4_BASE_GPMC >> 24) & 0x3F) | CSVALID );
}

/*=========================================================================================================================*/

/*******************************************************************************
 *
 * Function Name        : ulk_uart_gpmc_read_version                          
 * Input Arguments      : chip Number                              
 * Return Value         : None                             
 * Description          : This function used to read the gpmc version
 *
 ******************************************************************************/

void ulk_uart_gpmc_read_version (u8 chip_sel)
{
	u16 fpga_ver = 0;    

	fpga_ver = *(u16 *)(gpmc_cs_uart_address + FPGA_VER_INFO);

#ifdef FPGA_UART_DEBUG
	printk(" The FPGA Version is address  = 0x%x \r\n",fpga_ver);
#endif
}

/*=========================================================================================================================*/

/***********************************************************************
 * Function Name        : FPGA_init                            
 * Input Arguments      : baud_divisor                                 
 * Return Value         : none                                    
 * Description          : This functin will initialize the FPGA uart reg
                          
 ***********************************************************************/

void FPGA_init (int baud_divisor)
{

	/* Clear the Inturrupt enable register */
	*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_IER_REG) = 0x00;

	/* Set the 7th bit of Line control register */
	*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_LCR_REG) = 1 << 7 ;

	/* Set baud rate Inturrupt enable register */
	*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_IER_REG) = (baud_divisor >> 8) & 0xff;

	/* Clear the Inturrupt enable register */
	*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_RX_REG) = baud_divisor & 0xff;

	/* Clear 7th bit of Line control register */
	*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_LCR_REG) &= ~(1 << 7) ;  //Clearing the LCR 7thbit

	/* Set RX fifo as zero */
	*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_RX_REG) = 0;

	/* Clear the Inturrupt enable register */
	*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_IER_REG) = 0;

	/* Set  RTS/DTR for MCR  */
	*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_LCR_REG) = LCRVAL;

	/* Clear the Inturrupt enable register */
	*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_MCR_REG) = MCRVAL;

	/* Clear and enable the FIFO control register */
	*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_FCR_REG) = FCRVAL;
}

/*=========================================================================================================================*/

/***********************************************************************
 * Function Name        : ulk_fpga_uart_init                            
 * Input Arguments      : none                                
 * Return Value         : Zero                                    
 * Description          : This functin will call the initializes the 
                          gpmc and FPGA uart reg
                          
 ***********************************************************************/

unsigned char ulk_fpga_uart_init(int baud)
{
    int clock_divisor;

    mdelay(100);
    ulk_uart_gpmc_read_version (4);

    clock_divisor = calc_divisor(baud);
    FPGA_init(clock_divisor);
    return (0);
}

/*=========================================================================================================================*/

/**********************************************************************
 * Function Name        : ulk_fpga_uart_getc                            
 * Input Arguments      : none                                    
 * Return Value         : NS16550_getc_fpga_uart () value                                    
 * Description          : This function used to get the value    
        
 **********************************************************************/

unsigned char ulk_fpga_uart_getc()
{
#ifdef FPGA_UART_DEBUG
        printk("  Entered in function getc \r\n");
#endif
        return NS16550_getc_fpga_uart();
}

/*=========================================================================================================================*/

/**********************************************************************
 * Function Name        : ulk_fpga_uart_putc                            
 * Input Arguments      : unsigned char val                                    
 * Return Value         : none                                    
 * Description          : This function used to put the value    
        
 **********************************************************************/

unsigned char ulk_fpga_uart_putc( unsigned char val)
{
    /* Should we need this check ?? */
    if (val == '\n')
        NS16550_putc_fpga_uart( '\r');
#ifdef FPGA_UART_DEBUG
    printk(" The val is %c int %d  \r\n",val,val);
#endif
    return NS16550_putc_fpga_uart(val);
}

/*=========================================================================================================================*/

/**********************************************************************
 * Function Name        : ulk_fpga_uart_gets 
 * Input Arguments      : none                                    
 * Return Value         : NS16550_getc_fpga_uart () value                                    
 * Description          : This function used to get the string    
        
 **********************************************************************/

/*unsigned char * ulk_fpga_uart_gets()
{
	unsigned char buff[100];
	unsigned char ch;
	unsigned i =0;
#ifdef FPGA_UART_DEBUG
        printk("  Entered in function gets \r\n");
#endif

        while((ch = NS16550_getc_fpga_uart()) != '\r');
		buff[i++] = ch;

	buff[i] = '\0';
	bytes_read = i;

	return buff;
}
*/

/*=========================================================================================================================*/

/**********************************************************************
 * Function Name        : calc_divisor                            
 * Input Arguments      : none                                    
 * Return Value         : divisor value                                     
 * Description          : This function used to calculate the divisor
                          value    
                
 **********************************************************************/
static int calc_divisor (int div)
{
        return (CFG_NS16550_CLK / (16 * div));
}

/*=========================================================================================================================*/

/**********************************************************************
 * Function Name        : NS16550_putc_fpga_uart 
 * Input Arguments      : char c                                    
 * Return Value         : zero                                     
 * Description          : This function used to write the value in 
                           FPGA transmit reg   
                
 **********************************************************************/

unsigned char  NS16550_putc_fpga_uart (char c)
{

	long int time_out = 0;
	/* wait until Transmit buffer gets empty */
	while ((*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_LSR_REG) & 0x20) == 0)
	{
		time_out++;
		
		if (time_out == 0x1FFFFFF){
			printk(" Transmit Time Out !! \r\n");
			return (-1);
		}
	}
    *(u16 *)(gpmc_cs_uart_address + FPGA_UART_TX_REG) = c;
    return(0);
}

/*=========================================================================================================================*/

/**********************************************************************
 * Function Name        : NS16550_getc_fpga_uart 
 * Input Arguments      : none                                  
 * Return Value         : read_char                                     
 * Description          : This function used to read the value in 
                           FPGA recevice reg   
                
 **********************************************************************/
unsigned char NS16550_getc_fpga_uart ()
{

	long int time_out = 0;
	
	/* variable is to store the read value from UART */
	u16 read_char;

	/* wait until a character is entered in UART (check Reciever FIFO)*/
	while ((*(vushort16 *) (gpmc_cs_uart_address + FPGA_UART_LSR_REG) & LSR_DR) == 0)
	{
		time_out++;
		
		if (time_out == 0x1FFFFFF){
			read_char = '\0';
			return read_char;
		}
	}
	/* read a char from UART */
	read_char = *(u16 *)(gpmc_cs_uart_address + FPGA_UART_RX_REG);

#ifdef FPGA_UART_DEBUG
	printk("%c %d %x is the Read char \r\n", read_char,read_char,read_char);
#endif

	/* return a char read from UART */
	return read_char ;
}

/*=========================================================================================================================*/

MODULE_DESCRIPTION("FPGAUART_driver");

MODULE_LICENSE("GPL");


module_init(ulk_init_fpga_uart_module);
module_exit(ulk_cleanup_fpga_uart_module);

/*=========================================================================================================================*/




