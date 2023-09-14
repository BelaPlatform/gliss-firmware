#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

enum I2cPinsMode {
	kI2cPinsModeI2c,
	kI2cPinsModeExternal,
	kI2cPinsModeProgramming,
};

void i2cPinsMode(enum I2cPinsMode mode);
void psocPower(uint8_t power);

#ifdef __cplusplus
};
#endif // __cplusplus
