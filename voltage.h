class Voltage {
private:
    int low_voltage_flag;
    unsigned long vol_measure_time;
    double voltage;

public:
  Voltage();
  double measure();
  bool isLowVoltage();
};