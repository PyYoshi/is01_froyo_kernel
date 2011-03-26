

typedef struct GPIO_DEF {
	unsigned int no;
	int direction;
	int out_val;
	int init_done;
} stGPIO_DEF;

#define DirctionIn (0)
#define DirctionOut (1)

#define GPIO_PWRDWN_PORTNO		(33)
#define GPIO_GTDION_PORTNO		(34)
#define GPIO_LDO_PORTNO		(152)
#define GPIO_SRDT_TUNER_CTL	(107)
#define GPIO_PBVAL_TUNER	(108)
#define GPIO_SRCK_TUNER		(109)

typedef struct __ioctl_cmd{
	unsigned int no;
	unsigned int val;
} ioctl_cmd;

#define IOC_GPIO_VAL_SET	0x0001
#define IOC_GPIO_VAL_GET	0x0002
#define IOC_VREG_ENABLE		0x0003
#define IOC_VREG_DISABLE	0x0004


