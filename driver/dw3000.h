#ifndef _DW3000_H
#define _DW3000_H

// -------------------------------------------------------------------------------------------------------------------
// Macros and Enumerations for SPI & CLock blocks
//
#define DW3000_SPI_FAC      (0<<6 | 1<<0)
#define DW3000_SPI_FARW     (0<<6 | 0<<0)
#define DW3000_SPI_EAMRW    (1<<6)

// Defines for enable_clocks function
#define FORCE_CLK_SYS_TX        (1)
#define FORCE_CLK_AUTO          (5)
//SYSCLK
#define FORCE_SYSCLK_PLL        (2)
#define FORCE_SYSCLK_FOSCDIV4   (1)
#define FORCE_SYSCLK_FOSC       (3)
//RX and TX CLK
#define FORCE_CLK_PLL           (2)

#define SEL_CHANNEL5            (5)
#define SEL_CHANNEL9            (9)

typedef enum {
    DW3000_SPI_RD_BIT    = 0x0000U,
    DW3000_SPI_WR_BIT    = 0x8000U,
    DW3000_SPI_AND_OR_8  = 0x0001U,
    DW3000_SPI_AND_OR_16 = 0x0002U,
    DW3000_SPI_AND_OR_32 = 0x0003U,
}spi_modes_e;

/*This Enum holds the index for factor calculation.*/
typedef enum
{
    DWT_STS_LEN_32  =0,
    DWT_STS_LEN_64  =1,
    DWT_STS_LEN_128 =2,
    DWT_STS_LEN_256 =3,
    DWT_STS_LEN_512 =4,
    DWT_STS_LEN_1024=5,
    DWT_STS_LEN_2048=6
} dwt_sts_lengths_e;

//DW3000 IDLE/INIT mode definitions
#define DWT_DW_INIT      0x0
#define DWT_DW_IDLE      0x1
#define DWT_DW_IDLE_RC   0x2

//DW3000 OTP operating parameter set selection
#define DWT_OPSET_LONG   (0x0<<11)
#define DWT_OPSET_SCP    (0x1<<11)
#define DWT_OPSET_SHORT  (0x2<<11)

//Reset options
#define DWT_RESET_ALL          0x00
#define DWT_RESET_CTRX         0x0F
#define DWT_RESET_RX           0xEF
#define DWT_RESET_CLEAR        0xFF

// OTP addresses definitions
#define LDOTUNELO_ADDRESS (0x04)
#define LDOTUNEHI_ADDRESS (0x05)
#define PARTID_ADDRESS  (0x06)
#define LOTID_ADDRESS   (0x07)
#define VBAT_ADDRESS    (0x08)
#define VTEMP_ADDRESS   (0x09)
#define XTRIM_ADDRESS   (0x1E)
#define OTPREV_ADDRESS  (0x1F)
#define BIAS_TUNE_ADDRESS (0xA)
#define DGC_TUNE_ADDRESS (0x20)

// LDO and BIAS tune kick
#define LDO_BIAS_KICK (0x0180)  // Writing to bit 7 and 8

//DW3000 interrupt events

#define DWT_INT_SCRC            0x00000004          // SPI write CRC error event
#define DWT_INT_TFRS            0x00000080          // frame sent
#define DWT_INT_LDED            0x00000400          // micro-code has finished execution
#define DWT_INT_RSFDD           0x00000200          // detected the SFD sequence
#define DWT_INT_RFCG            0x00004000          // frame received with good CRC
#define DWT_INT_RPHE            0x00001000          // receiver PHY header error
#define DWT_INT_RFCE            0x00008000          // receiver CRC error
#define DWT_INT_RFSL            0x00010000          // receiver sync loss error
#define DWT_INT_RFTO            0x00020000          // frame wait timeout
#define DWT_INT_LDEERR          0x00040000          // CIA error
#define DWT_INT_RXOVRR          0x00100000          // receiver overrun
#define DWT_INT_RXPTO           0x00200000          // preamble detect timeout
#define DWT_INT_LCSSERR         0x00400000          // LCSS set up error
#define DWT_INT_SFDT            0x04000000          // SFD timeout
#define DWT_INT_HPDWARN         0x08000000          // HPDWARN timeout
#define DWT_INT_CPERR           0x10000000          // STS Error
#define DWT_INT_ARFE            0x20000000          // frame rejected (due to frame filtering configuration)
#define DWT_INT_ALL             0x3FFFFFFF
#define DWT_INT_RX              (DWT_INT_LDED | DWT_INT_RSFDD | DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_RFTO | DWT_INT_LDEERR | DWT_INT_RXPTO | DWT_INT_SFDT | DWT_INT_ARFE)

//! fast commands
#define CMD_DB_TOGGLE     0x13   //!< Toggle double buffer pointer
#define CMD_CLR_IRQS      0x12   //!< Clear all events/clear interrupt
#define CMD_CCA_TX_W4R    0x11   //!< Check if channel clear prior to TX, enable RX when TX done
#define CMD_DTX_REF_W4R   0x10   //!< Start delayed TX (as DTX_REF below), enable RX when TX done
#define CMD_DTX_RS_W4R    0xF    //!< Start delayed TX (as DTX_RS below), enable RX when TX done
#define CMD_DTX_TS_W4R    0xE    //!< Start delayed TX (as DTX_TS below), enable RX when TX done
#define CMD_DTX_W4R       0xD    //!< Start delayed TX (as DTX below), enable RX when TX done
#define CMD_TX_W4R        0xC    //!< Start TX (as below), enable RX when TX done
#define CMD_CCA_TX        0xB    //!< Check if channel clear prior to TX
#define CMD_DRX_REF       0xA    //!< Enable RX @ time = DREF_TIME + DX_TIME
#define CMD_DTX_REF       0x9    //!< Start delayed TX (RMARKER will be @ time = DREF_TIME + DX_TIME)
#define CMD_DRX_RS        0x8    //!< Enable RX @ time = RX_TIME + DX_TIME
#define CMD_DTX_RS        0x7    //!< Start delayed TX (RMARKER will be @ time = RX_TIME + DX_TIME)
#define CMD_DRX_TS        0x6    //!< Enable RX @ time = TX_TIME + DX_TIME
#define CMD_DTX_TS        0x5    //!< Start delayed TX (RMARKER will be @ time = TX_TIME + DX_TIME)
#define CMD_DRX           0x4    //!< Enable RX @ time specified in DX_TIME register
#define CMD_DTX           0x3    //!< Start delayed TX (RMARKER will be @ time set in DX_TIME register)
#define CMD_RX            0x2    //!< Enable RX
#define CMD_TX            0x1    //!< Start TX
#define CMD_TXRXOFF       0x0    //!< Turn off TX or RX, clear any TX/RX events and put DW3000 into IDLE

//! constants for specifying TX Preamble length in symbols
//! These are defined to allow them be directly written into byte 2 of the TX_FCTRL register
//! (i.e. a four bit value destined for bits 20..18 but shifted left by 2 for byte alignment)
#define DWT_PLEN_4096   0x03    //! Standard preamble length 4096 symbols
#define DWT_PLEN_2048   0x0A    //! Non-standard preamble length 2048 symbols
#define DWT_PLEN_1536   0x06    //! Non-standard preamble length 1536 symbols
#define DWT_PLEN_1024   0x02    //! Standard preamble length 1024 symbols
#define DWT_PLEN_512    0x0d    //! Non-standard preamble length 512 symbols
#define DWT_PLEN_256    0x09    //! Non-standard preamble length 256 symbols
#define DWT_PLEN_128    0x05    //! Non-standard preamble length 128 symbols
#define DWT_PLEN_64     0x01    //! Standard preamble length 64 symbols
#define DWT_PLEN_32     0x04    //! Non-standard length 32
#define DWT_PLEN_72     0x07    //! Non-standard length 72

//! constants for specifying Preamble Acquisition Chunk (PAC) Size in symbols
#define DWT_PAC8        0   //!< PAC  8 (recommended for RX of preamble length  128 and below
#define DWT_PAC16       1   //!< PAC 16 (recommended for RX of preamble length  256
#define DWT_PAC32       2   //!< PAC 32 (recommended for RX of preamble length  512
#define DWT_PAC4        3   //!< PAC  4 (recommended for RX of preamble length  < 127

//! constants for selecting the bit rate for data TX (and RX)
//! These are defined for write (with just a shift) the TX_FCTRL register
#define DWT_BR_850K     0   //!< UWB bit rate 850 kbits/s
#define DWT_BR_6M8      1   //!< UWB bit rate 6.8 Mbits/s
#define DWT_BR_NODATA   2   //!< No data (SP3 packet format)

//! constants for specifying the (Nominal) mean Pulse Repetition Frequency
//! These are defined for direct write (with a shift if necessary) to CHAN_CTRL and TX_FCTRL regs
#define DWT_PRF_16M     1   //!< UWB PRF 16 MHz
#define DWT_PRF_64M     2   //!< UWB PRF 64 MHz
#define DWT_PRF_SCP     3   //!< SCP UWB PRF ~100 MHz

#define DWT_SFDTOC_DEF          129  // default SFD timeout value

#define DWT_PHRMODE_STD         0x0     // standard PHR mode
#define DWT_PHRMODE_EXT         0x1     // DW proprietary extended frames PHR mode

#define DWT_PHRRATE_STD         0x0     // standard PHR rate
#define DWT_PHRRATE_DTA         0x1     // PHR at data rate (6M81)

// Define DW3000 PDOA modes
#define DWT_PDOA_M0           0x0     // DW PDOA mode is off
#define DWT_PDOA_M1           0x1     // DW PDOA mode  mode 1
#define DWT_PDOA_M3           0x3     // DW PDOA mode  mode 3

// Define DW3000 STS modes
#define DWT_STS_MODE_OFF         0x0     // STS is off
#define DWT_STS_MODE_1           0x1     // STS mode 1
#define DWT_STS_MODE_2           0x2     // STS mode 2
#define DWT_STS_MODE_ND          0x3     // STS with no data
#define DWT_STS_MODE_SDC         0x8     // Enable Super Deterministic Codes
#define DWT_STS_CONFIG_MASK      0xB

//DW3000 SLEEP and WAKEUP configuration parameters
#define DWT_PGFCAL       0x0800
#define DWT_GOTORX       0x0200
#define DWT_GOTOIDLE     0x0100
#define DWT_SEL_OPS3     0x00C0
#define DWT_SEL_OPS2     0x0080                     // Short OPS table
#define DWT_SEL_OPS1     0x0040                     // SCP
#define DWT_SEL_OPS0     0x0000                     // Long OPS table
#define DWT_ALT_OPS      0x0020
#define DWT_LOADLDO      0x0010
#define DWT_LOADDGC      0x0008
#define DWT_LOADBIAS     0x0004
#define DWT_RUNSAR       0x0002
#define DWT_CONFIG       0x0001                     // download the AON array into the HIF (configuration download)

/* Enum used for selecting location to load DGC data from */
typedef enum
{
    DWT_DGC_LOAD_FROM_SW=0,
    DWT_DGC_LOAD_FROM_OTP
} dwt_dgc_load_location;

/* Enum used for selecting channel for DGC on-wake kick. */
typedef enum
{
    DWT_DGC_SEL_CH5=0,
    DWT_DGC_SEL_CH9
} dwt_dgc_chan_sel;

/*
	Lookup table default values for channel 5
*/
typedef enum
{
    CH5_DGC_LUT_0 = 0x1c0fd,
    CH5_DGC_LUT_1 = 0x1c43e,
    CH5_DGC_LUT_2 = 0x1c6be,
    CH5_DGC_LUT_3 = 0x1c77e,
    CH5_DGC_LUT_4 = 0x1cf36,
    CH5_DGC_LUT_5 = 0x1cfb5,
    CH5_DGC_LUT_6 = 0x1cff5
} dwt_configmrxlut_ch5_e;

/*
	Lookup table default values for channel 9
*/
typedef enum
{
    CH9_DGC_LUT_0 = 0x2a8fe,
    CH9_DGC_LUT_1 = 0x2ac36,
    CH9_DGC_LUT_2 = 0x2a5fe,
    CH9_DGC_LUT_3 = 0x2af3e,
    CH9_DGC_LUT_4 = 0x2af7d,
    CH9_DGC_LUT_5 = 0x2afb5,
    CH9_DGC_LUT_6 = 0x2afb5
} dwt_configmrxlut_ch9_e;

#define DBL_BUFF_OFF             0x0
#define DBL_BUFF_ACCESS_BUFFER_0 0x1
#define DBL_BUFF_ACCESS_BUFFER_1 0x3

/*! ------------------------------------------------------------------------------------------------------------------
 * Structure typedef: dwt_config_t
 *
 * Structure for setting device configuration via dwt_configure() function
 *
 */
struct dw3000_config_t
{
    u8 chan ;           //!< Channel number (5 or 9)
    u8 txPreambLength ; //!< DWT_PLEN_64..DWT_PLEN_4096
    u8 rxPAC ;          //!< Acquisition Chunk Size (Relates to RX preamble length)
    u8 txCode ;         //!< TX preamble code (the code configures the PRF, e.g. 9 -> PRF of 64 MHz)
    u8 rxCode ;         //!< RX preamble code (the code configures the PRF, e.g. 9 -> PRF of 64 MHz)
    u8 sfdType;         //!< SFD type (0 for short IEEE 8-bit standard, 1 for DW 8-bit, 2 for DW 16-bit, 3 for 4z BPRF)
    u8 dataRate ;       //!< Data rate {DWT_BR_850K or DWT_BR_6M8}
    u8 phrMode ;        //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
    u8 phrRate;         //!< PHR rate {0x0 - standard DWT_PHRRATE_STD, 0x1 - at datarate DWT_PHRRATE_DTA}
    u16 sfdTO ;         //!< SFD timeout value (in symbols)
    u8 stsMode;         //!< STS mode (no STS, STS before PHR or STS after data)
    dwt_sts_lengths_e stsLength;    //!< STS length (the allowed values are listed in dwt_sts_lengths_e
    u8 pdoaMode;        //!< PDOA mode
};

#define GET_STS_REG_SET_VALUE(x)     ((uint16_t)1<<((x)+2))    /* Returns the value to set in CP_CFG0_ID for STS length. The x is the enum value from dwt_sts_lengths_e */

#endif /* !_DW3000_H */