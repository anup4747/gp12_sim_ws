#include <SPI.h>
#include <mcp2515.h>

// ESP32 SPI pins
#define CAN_CS 5
#define CAN_INT 4
#define CAN_SCK 18
#define CAN_MISO 19
#define CAN_MOSI 23

MCP2515 mcp2515(CAN_CS);

struct can_frame canMsg;

static const uint16_t CAN_ID_QUATERNION = 0x100;
static const uint16_t CAN_ID_FLEX = 0x101;

static int16_t readInt16LE(const uint8_t *data)
{
    uint16_t raw = (static_cast<uint16_t>(data[1]) << 8) | data[0];
    return static_cast<int16_t>(raw);
}

static void printUnknownFrame(const struct can_frame &frame)
{
    Serial.print("Unknown CAN ID: 0x");
    Serial.println(frame.can_id, HEX);
    Serial.print("DLC: ");
    Serial.println(frame.can_dlc);
    Serial.print("DATA: ");
    for (uint8_t i = 0; i < frame.can_dlc && i < 8; i++)
    {
        if (frame.data[i] < 0x10)
            Serial.print("0");
        Serial.print(frame.data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

static void printFlexPacket(const struct can_frame &frame)
{
    uint16_t adcRaw = static_cast<uint16_t>(frame.data[0]) |
                      (static_cast<uint16_t>(frame.data[1]) << 8);
    uint8_t bendPercent = frame.data[2];
    uint32_t voltageMv = (static_cast<uint32_t>(adcRaw) * 3300UL) / 4095UL;

    Serial.println("=== Flex Sensor Packet (CAN ID 0x101) ===");
    Serial.print("ADC raw: ");
    Serial.println(adcRaw);
    Serial.print("Bend: ");
    Serial.print(bendPercent);
    Serial.println("%");
    Serial.print("Voltage: ");
    Serial.print(voltageMv / 1000UL);
    Serial.print(".");
    if ((voltageMv % 1000UL) < 100UL)
        Serial.print("0");
    if ((voltageMv % 1000UL) < 10UL)
        Serial.print("0");
    Serial.print(voltageMv % 1000UL);
    Serial.println(" V");
}

static void printQuaternionPacket(const struct can_frame &frame)
{
    float i = readInt16LE(&frame.data[0]) / 10000.0f;
    float j = readInt16LE(&frame.data[2]) / 10000.0f;
    float k = readInt16LE(&frame.data[4]) / 10000.0f;
    float real = readInt16LE(&frame.data[6]) / 10000.0f;

    Serial.println("=== BNO085 Quaternion Packet (CAN ID 0x100) ===");
    Serial.print("i: ");
    Serial.print(i, 4);
    Serial.print("  j: ");
    Serial.print(j, 4);
    Serial.print("  k: ");
    Serial.print(k, 4);
    Serial.print("  real: ");
    Serial.println(real, 4);
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("ESP32 MCP2515 CAN Receiver");

    // Start SPI
    SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS);

    // Reset MCP2515
    mcp2515.reset();

    // Set CAN bitrate
    // IMPORTANT: Must be the same as transmitter
    // MCP_8MHZ assumes an 8 MHz MCP2515 crystal
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);

    // Set normal CAN mode
    mcp2515.setNormalMode();

    Serial.println("CAN receiver ready");
}

void loop()
{
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
    {
        if (canMsg.can_dlc != 8)
        {
            Serial.print("CAN ID 0x");
            Serial.print(canMsg.can_id, HEX);
            Serial.print(" has unexpected DLC: ");
            Serial.println(canMsg.can_dlc);
        }
        else if (canMsg.can_id == CAN_ID_FLEX)
        {
            printFlexPacket(canMsg);
        }
        else if (canMsg.can_id == CAN_ID_QUATERNION)
        {
            printQuaternionPacket(canMsg);
        }
        else
        {
            printUnknownFrame(canMsg);
        }

        Serial.println("----------------------");
    }
}