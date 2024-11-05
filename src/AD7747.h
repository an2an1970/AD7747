#ifndef AD7747_h
#define AD7747_h

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>




#define _AD7747_ADDRESS_read 0x91
#define _AD7747_ADDRESS_write 0x90

#define __STATUS__ 0x00    // 00000111
#define CAP_DATA_H 0x01    // (R) Capacitive channel data—high byte, 0x00
#define CAP_DATA_M 0x02    // (R) Capacitive channel data—middle byte, 0x00
#define CAP_DATA_L 0x03    // (R) Capacitive channel data—low byte, 0x00
#define VT_DATA_H 0x04     // (R) Voltage/temperature channel data—high byte, 0x00
#define VT_DATA_M 0x05     // (R) Voltage/temperature channel data—middle byte, 0x00
#define VT_DATA_L 0x06     // (R) Voltage/temperature channel data—low byte, 0x00
#define CAP_SETUP 0x07     // (R/W) CAPEN – CAPDIFF – – – – –
#define VT_SETUP 0x08      // (R/W) VTEN VTMD1 VTMD0 EXTREF – - VTSHORT VTCHOP
#define EXC_SETUP 0x09     // (R/W) – – – – EXCDAC EXCEN EXCLVL1 EXCLVL0
#define CONFIGURATION 0x0A // (R/W) VTFS1 VTFS0 CAPFS2 CAPFS1 CAPFS0 MD2 MD1 MD0
#define CAP_DAC_A 0x0B     // (R/W) DACAENA – DACA—6-Bit Value
#define CAP_DAC_B 0x0C     // (R/W) DACBENB – DACB—6-Bit Value
#define CAP_OFFSET_H 0x0D  // (R/W) Capacitive offset calibration—high byte, 0x80
#define CAP_OFFSET_L 0x0E  // (R/W) Capacitive offset calibration—low byte, 0x00
#define CAP_GAIN_H 0x0F    // (R/W) Capacitive gain calibration—high byte, factory calibrated
#define CAP_GAIN_L 0x10    // (R/W) Capacitive gain calibration—low byte, factory calibrated
#define VOLT_GAIN_H 0x11   // (R/W) Voltage gain calibration—high byte, factory calibrated
#define VOLT_GAIN_L 0x12   // (R/W) Voltage gain calibration—low byte, factory calibrated

#define CAP_DATA_START 0x01
#define VT_DATA_START 0x04

  //------------------- register config --------------------------

#define CAP_SETUP_REG_VALUE B10100000 // 0XA0 -> CAPEN=1, CAPDIFF=1
#define VT_SETUP_REG_VALUE B10000001  // 0X81 -> VTEN=1, VTMD1=0, VTMD0=0, EXTREF=0, VSHORT=0, VTCHOP=1
#define EXC_SETUP_REG_VALUE B00001111 // 0X07 -> EXCDAC=1, EXCEN=1, EXCLVL1=1, EXCLVL0=0
// TODO: RECHECK DATASHEET FOR CONFIGURATION_REG_VALUE
#define CONFIGURATION_REG_VALUE B01111001 // 0X01 -> {VTFS1=0 , VTFS0=0 == CONVERSATION TIME :20.1 ms}
                                          //  {CAPFS2/CAFPFS1/CAPFS0=000 == DSP CONVERSATION TIME :22.0 }
                                          //  {MD2/MD1/MD0=001 === COUNTINUOUS MODE }
                                          //  {REMEMBER :: OFFSET U GAIN CALIBRATION }
// #define CAP_DAC_A_REG_VALUE B10000000 // 0X80  -> DACAENA=1 , DACA=000000
#define CAP_DAC_A_REG_VALUE B00000000 // 0X80  -> DACAENA=1 , DACA=000000
#define CAP_DAC_B_REG_VALUE B10000000 // 0X80  -> DACBENB=1 , DBCB=000000
// TODO: RECHECK DATASHEET FOR CAP_OFFSET_H_REG_VALUE
#define CAP_OFFSET_H_REG_VALUE B10000000 // 0X8000 DEFAULT VALUE
#define CAP_OFFSET_L_REG_VALUE B00000000
// TODO: RECHECK DATASHEET FOR CAP_OFFSET_H_REG_VALUE
#define CAP_GAIN_H_REG_VALUE B00000000 // 0Xxxx NO DEFAULT VALUE
#define CAP_GAIN_L_REG_VALUE B00000000
// TODO: RECHECK DATASHEET FOR VOLT_GAIN_H_REG_VALUE
#define VOLT_GAIN_H_REG_VALUE B00000000 // 0Xxxx NO DEFAULT VALUE
#define VOLT_GAIN_L_REG_VALUE B00000000

  // bool    ee24_isConnected(void);
  // bool    ee24_write(uint16_t address, uint8_t *data, size_t lenInBytes, uint32_t timeout);
  // bool    ee24_read(uint16_t address, uint8_t *data, size_t lenInBytes, uint32_t timeout);
  // bool    ee24_eraseChip(void);

  class AD7747
  {

  public:
    AD7747(uint8_t i2caddrRead, uint8_t i2caddrWrite)
    {

      _i2caddrRead = i2caddrRead >> 1;
      _i2caddrWrite = i2caddrWrite >> 1;
    }

    uint8_t read8_AD77(uint8_t _address);
    uint8_t read8_debug_AD77(uint8_t _address);
    void write8_AD77(uint8_t _address, uint8_t _data);
    uint16_t read16_AD77(uint16_t _address);
    void write16_AD77(uint8_t _address, uint8_t _dataHIGH, uint8_t _dataLOW);
    void loadSettings(void);
    uint8_t getSetupCap(uint8_t address);
    uint32_t getCap(uint8_t addressStart);
    uint32_t getVT(uint8_t vt_start);
    bool isDeviceConnected(void);
    void waitUntilDeviceConnected(void);
    double capPF(uint32_t rawCap);
    double TempCentigrade(uint32_t rawTemp);
    bool debug_mode = true;

  private:
    uint8_t _i2caddrRead;
    uint8_t _i2caddrWrite;

    uint8_t cap_dataH(uint8_t address);
    uint8_t cap_dataM(uint8_t address);
    uint8_t cap_dataL(uint8_t address);

    uint8_t vt_dataH(uint8_t address);
    uint8_t vt_dataM(uint8_t address);
    uint8_t vt_dataL(uint8_t address);

    void setSetupCap(uint8_t address, uint8_t data);
    void setSetupVT(uint8_t address, uint8_t data);
    void setSetupEXC(uint8_t address, uint8_t data);
    void setConfiguration(uint8_t address, uint8_t data);

    void setCapDacA(uint8_t address, uint8_t data);
    void setCapDacB(uint8_t address, uint8_t data);

    void setOffsetH(uint8_t address, uint8_t dataHigh);
    void setOffsetL(uint8_t address, uint8_t dataLow);

    void setGainH(uint8_t address, uint8_t dataHigh);
    void setGainL(uint8_t address, uint8_t dataLow);

    void setOffset(uint8_t address, uint8_t dataHigh, uint8_t dataLow);
    void setGain(uint8_t address, uint8_t dataHigh, uint8_t dataLow);

    bool isDataReadyCap(uint8_t address);
    bool isDataReadyTemp(uint8_t address);
    uint8_t check_status(uint8_t address);
    uint8_t *read24_AD77(uint8_t address);
    void __debugger__(char *title, uint8_t arr_input[], uint8_t size_arr);
    void __debugger__(char *title, double arr_input[], uint8_t size_arr);
  };
#endif
