#include "AS201.h"

// 帧头帧尾
const uint8_t FRAME_START[2] = {0xFA, 0xFB};
const uint8_t FRAME_END[2]   = {0xFC, 0xFD};

AS201::AS201(int rxPin, int txPin, HardwareSerial &uart)
{
    rx = rxPin;
    tx = txPin;
    serial = &uart;
}

void AS201::begin(uint32_t baud)
{
    serial->begin(baud, SERIAL_8N1, rx, tx);
}

// 找序列
int AS201::findSequence(uint8_t *arr, int arrLen, const uint8_t *seq, int seqLen)
{
    for (int i = 0; i < arrLen - seqLen + 1; i++)
    {
        bool found = true;
        for (int j = 0; j < seqLen; j++)
        {
            if (arr[i + j] != seq[j])
            {
                found = false;
                break;
            }
        }
        if (found)
            return i;
    }
    return -1;
}

// 更新一次数据
bool AS201::update()
{
    while (serial->available())
    {
        buffer[bufferIndex++] = serial->read();
        if (bufferIndex >= BUFFER_SIZE) bufferIndex = 0;

        int s = findSequence(buffer, bufferIndex, FRAME_START, 2);
        int e = findSequence(buffer, bufferIndex, FRAME_END, 2);

        if (s != -1 && e != -1 && e > s)
        {
            int len = (e + 2) - s;
            if (len < 5 || len > BUFFER_SIZE) { bufferIndex = 0; return false; }

            uint8_t frame[BUFFER_SIZE];
            memcpy(frame, buffer + s, len);

            parseFrame(frame, len);
            bufferIndex = 0;
            return true;
        }
    }
    return false;
}

// 数据解析
bool AS201::parseFrame(uint8_t *buffer, int len)
{
    if (buffer[0] != 0xFA || buffer[1] != 0xFB) return false;
    uint8_t blen = buffer[2];

    if (buffer[len - 2] != 0xFC || buffer[len - 1] != 0xFD) return false;

    uint8_t checksum = 0;
    for (int i = 3; i < len - 3; i++)
        checksum += buffer[i];

    if (checksum != buffer[len - 3]) return false;

    uint8_t cmd = buffer[3];
    int idx = 5;

    if (cmd != 0) return false;

    auto c16 = [&](int &i) -> float {
        int16_t v = (buffer[i+1] << 8) | buffer[i];
        i += 2;
        return (float)v;
    };

    data.ax = c16(idx) * 0.00478515625f;
    data.ay = c16(idx) * 0.00478515625f;
    data.az = c16(idx) * 0.00478515625f;

    data.gx = c16(idx) * 0.0625f;
    data.gy = c16(idx) * 0.0625f;
    data.gz = c16(idx) * 0.0625f;

    data.roll  = c16(idx) * 0.0054931640625f;
    data.pitch = c16(idx) * 0.0054931640625f;
    data.yaw   = c16(idx) * 0.0054931640625f;

    data.mx = c16(idx) * 0.006103515625f;
    data.my = c16(idx) * 0.006103515625f;
    data.mz = c16(idx) * 0.006103515625f;

    data.q0 = c16(idx) * 0.000030517578125f;
    data.q1 = c16(idx) * 0.000030517578125f;
    data.q2 = c16(idx) * 0.000030517578125f;
    data.q3 = c16(idx) * 0.000030517578125f;

    data.temperature = c16(idx) * 0.01f;

    uint32_t pb = (buffer[idx+3]<<24)|(buffer[idx+2]<<16)|(buffer[idx+1]<<8)|buffer[idx];
    idx += 4;
    data.pressure = pb * 0.0002384185791f;

    uint32_t hb = (buffer[idx+3]<<24)|(buffer[idx+2]<<16)|(buffer[idx+1]<<8)|buffer[idx];
    idx += 4;
    data.height = hb * 0.0010728836f;

    return true;
}

SensorData &AS201::getData()
{
    return data;
}
