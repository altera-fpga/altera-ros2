// Copyright (C) 2025 Altera Corporation
// SPDX-License-Identifier: Apache-2.0
//

#include <string>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

#include "drive_on_chip.h"

using namespace std;


class DoCDevice
{
public:
  DoCDevice();

  DoCDevice(std::string p, int o, int s) {
    path = p;
    offset = o;
    size = s;
  }

  int activate();

  void deactivate();

  int32_t read(const int addr);

  void write(const int addr, const int32_t value);

protected:
  std::string path;
  int offset;
  int size;

  int fd;
  int32_t *map;
};