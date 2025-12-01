#ifndef AS201_H
#define AS201_H

#include <Arduino.h>

typedef struct
{
    float ax, ay, az;
    float gx, gy, gz;
    float roll, pitch, yaw;
    float mx, my, mz;
    float q0, q1, q2, q3;
    float temperature;
    float pressure;
    float height;
} SensorData;

class AS201
{
public:
    AS201(int rxPin, int txPin, HardwareSerial &uart = Serial2);

    void begin(uint32_t baud = 115200);
    bool update();          // 调用后自动读取 + 解析
    SensorData &getData();  // 返回最新数据引用

private:
    HardwareSerial *serial;
    int rx, tx;

    static const int BUFFER_SIZE = 256;
    uint8_t buffer[BUFFER_SIZE];
    int bufferIndex = 0;

    bool parseFrame(uint8_t *buf, int len);
    int findSequence(uint8_t *arr, int arrLen, const uint8_t *seq, int seqLen);

    SensorData data;
};

#endif
