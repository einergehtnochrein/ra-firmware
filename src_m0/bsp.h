
#if (BOARD_RA == 1)
#  define GPIO_RX_CLK                       GPIO_0_11
#  define GPIO_RX_DATA                      GPIO_0_12
#  define PIN_SSEL_D                        PIO0_4
#  define SSEL_D_ASSERT_VALUE               0x8A
#endif

#if (BOARD_RA == 2)
#  define GPIO_RX_CLK                       GPIO_0_14
#  define GPIO_RX_DATA                      GPIO_0_29
#  define PIN_SSEL_D                        PIO0_23
#  define SSEL_D_ASSERT_VALUE               0xA1

#endif
