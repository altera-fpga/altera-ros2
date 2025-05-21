// Copyright (C) 2025 Altera Corporation
// SPDX-License-Identifier: Apache-2.0
//

#include "include/doc_device.hpp"

int DoCDevice::activate() {
    fd = open(path.c_str(), O_RDWR);
    if (fd == -1) {
        return 1;
    }

    map = (int32_t*) mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

    if (map == MAP_FAILED) {
        close(fd);
        return 2;
    }

    // Update config
    map[DOC_DBG_WAVE_DEMO_MODE + offset / 4] = (int32_t)1;
    map[DOC_DBG_POS_SPEED_LIMIT + offset / 4] = (int32_t)1000;
    map[DOC_DBG_WAVE_DEMO_PERIOD + offset / 4] = (int32_t)200;
    map[DOC_DBG_WAVE_DEMO_AMP_F + offset / 4] = (int32_t)0;
    map[DOC_DBG_WAVE_DEMO_OFFSET + offset / 4] = (int32_t)0;
    map[DOC_DBG_WAVE_DEMO_WAVEFORM + offset / 4] = (int32_t)4;
    map[DOC_DBG_DEMO_UPDATE] = (int32_t)1;

    sleep(1);

    map[DOC_DBG_I_PI_KP + offset / 4] = (int32_t)90000;
    map[DOC_DBG_I_PI_KI + offset / 4] = (int32_t)10000;
    map[DOC_DBG_SPEED_PI_KP + offset / 4] = (int32_t)90000;
    map[DOC_DBG_SPEED_PI_KI + offset / 4] = (int32_t)0;
    map[DOC_DBG_POS_PI_KP + offset / 4] = (int32_t)5000;
    map[DOC_DBG_DEMO_UPDATE] = (int32_t)1;

    map[DOC_DBG_POS_SETP0 + offset / 4] = (int32_t)0;
    map[DOC_DBG_DEMO_UPDATE] = (int32_t)1;

    return 0;
}

void DoCDevice::deactivate() {
    munmap(map, size);
    close(fd);
}

int32_t DoCDevice::read(const int addr) {
    return map[addr + offset / 4];
}

void DoCDevice::write(const int addr, const int32_t value) {
    map[addr + offset / 4] = value;
}
