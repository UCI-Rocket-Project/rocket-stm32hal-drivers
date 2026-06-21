#pragma once

#include <cstdint>

#pragma pack(push, 1)

// https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf

struct UBX_ACK_NAK {
    const uint8_t packetClass = 0x05;
    const uint8_t packetId = 0x00;
    const uint16_t payloadLength = sizeof(UBX_ACK_NAK) - 4;
    uint8_t clsID;
    uint8_t msgID;
};

struct UBX_ACK_ACK {
    const uint8_t packetClass = 0x05;
    const uint8_t packetId = 0x01;
    const uint16_t payloadLength = sizeof(UBX_ACK_ACK) - 4;
    uint8_t clsID;
    uint8_t msgID;
};

struct UBX_NAV_PVT {
    const uint8_t packetClass = 0x01;
    const uint8_t packetId = 0x07;
    const uint16_t payloadLength = sizeof(UBX_NAV_PVT) - 4;
    uint32_t iTOW;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint16_t flags3;
    uint8_t reserved1[4];
    int32_t headVeh;
    int16_t magDec;
    uint16_t magAcc;
};

struct UBX_CFG_PRT {
    const uint8_t packetClass = 0x06;
    const uint8_t packetId = 0x00;
    const uint16_t payloadLength = sizeof(UBX_CFG_PRT) - 4;
    uint8_t portID;
    uint8_t reserved1 = 0;
    uint16_t txReady;
    uint32_t mode;
    uint32_t baudRate;
    uint16_t inProtoMask;
    uint16_t outProtoMask;
    uint16_t flags;
    uint8_t reserved2[2] = {0};
};

struct UBX_CFG_PRT_READ {
    const uint8_t packetClass = 0x06;
    const uint8_t packetId = 0x00;
    const uint16_t payloadLength = sizeof(UBX_CFG_PRT_READ) - 4;
    uint8_t portID;
};

struct UBX_CFG_MSG {
    const uint8_t packetClass = 0x06;
    const uint8_t packetId = 0x01;
    const uint16_t payloadLength = sizeof(UBX_CFG_MSG) - 4;
    uint8_t msgClass;
    uint8_t msgID;
    uint8_t rate;
};

struct UBX_CFG_RST {
    const uint8_t packetClass = 0x06;
    const uint8_t packetId = 0x04;
    const uint16_t payloadLength = sizeof(UBX_CFG_RST) - 4;
    uint16_t navBbrMask;
    uint8_t resetMode;
    uint8_t reserved1 = 0;
};

struct UBX_CFG_RATE {
    const uint8_t packetClass = 0x06;
    const uint8_t packetId = 0x08;
    const uint16_t payloadLength = sizeof(UBX_CFG_RATE) - 4;
    uint16_t measRate;
    uint16_t navRate;
    uint16_t timeRef;
};

struct UBX_CFG_NAV5 {
    const uint8_t packetClass = 0x06;
    const uint8_t packetId = 0x24;
    const uint16_t payloadLength = sizeof(UBX_CFG_NAV5) - 4;
    uint16_t mask;
    uint8_t dynModel;
    uint8_t fixMode;
    int32_t fixedAlt;
    uint32_t fixedAltVar;
    int8_t minElev;
    uint8_t drLimit = 0;
    uint16_t pDop;
    uint16_t tDop;
    uint16_t pAcc;
    uint16_t tAcc;
    uint8_t staticHoldThresh;
    uint8_t dgnssTimeout;
    uint8_t cnoThreshNumSVs;
    uint8_t cnoThresh;
    uint8_t reserved1[2] = {0};
    uint16_t staticHoldMaxDist;
    uint8_t utcStandard;
    uint8_t reserved2[5] = {0};
};

// Save/reset/load settings from non-volatile memory
struct UBX_CFG_CFG {
    const uint8_t packetClass = 0x06;
    const uint8_t packetId = 0x09;
    const uint16_t payloadLength = sizeof(UBX_CFG_CFG) - 4;
    // Mask definitions: Page 209 of proto specs (linked @ top of this file)
    uint32_t clearMask;
    uint32_t saveMask;
    uint32_t loadMask;
};

struct UBX_MON_VER {
    const uint8_t packetClass = 0x0A;
    const uint8_t packetId = 0x04;
    const uint16_t payloadLength = sizeof(UBX_MON_VER) - 4;
};

struct UBX_NAV_RESETODO {
    const uint8_t packetClass = 0x01;
    const uint8_t packetId = 0x10;
    const uint16_t payloadLength = sizeof(UBX_MON_VER) - 4;
};

#pragma pack(pop)
