# Agilex™ 5 FPGA E-Series Modular Development Kit
Below you will find instructions for setting up the [Drive-On-Chip 6 Axis Design](https://github.com/altera-fpga/agilex-ed-drive-on-chip/tree/main/HPS_NIOSVg_DoC_3x2_axis) on the [Agilex™ 5 FPGA E-Series Modular Development Kit](https://www.intel.com/content/www/us/en/products/details/fpga/development-kits/agilex/a5e065b-modular.html).

These instructions assume you are familiar with the board. If this isn't the case please refer to the [Quick Start Guide](https://www.intel.com/content/www/us/en/products/docs/programmable/agilex5e-065b-modular-dev-kit-quick-start-guide.html) before continuing.

## Dependencies

* Altera® Quartus® Prime Pro Edition Version 24.3
* SD Card (at least 8GB)


## Required Files

* Linux Image Files: [wic.gz](https://github.com/altera-fpga/agilex-ed-drive-on-chip/releases/download/rel-crc-24.3/core-image-minimal-agilex5_mk_a5e065bb32aes1.wic.gz), [wic.bmap](https://github.com/altera-fpga/agilex-ed-drive-on-chip/releases/download/rel-crc-24.3/core-image-minimal-agilex5_mk_a5e065bb32aes1.wic.bmap)
* QSPI Image: [top.hps.jic](https://github.com/altera-fpga/agilex-ed-drive-on-chip/releases/download/rel-crc-24.3/top.hps.jic)

## Flashing The SD Card

Download both the `wic.gz` image and the `wic.bmap` file to the same directory.

Follow the instructions [HERE](flash_sd_card.md) to write the image to SD card.

## Flashing The QSPI

Download the `.jic` file to the device with Quartus® Prime Pro installed before proceeding.

1. Power down the board
2. Set `MSEL` dipswitch `S4` on the SOM to JTAG: **`OFF-OFF`**
3. Power on the board

4. Run the following command to begin programming the QSPI
```
quartus_pgm -c 1 -m jtag -o "pvi;top.hps.jic"
```

5. Once programming is complete power down the board
6. Set `MSEL` dipswitch `S4` on the SOM to ASX4: **`ON-ON`**

## Finishing

You can now insert the SD card and power on the board which should proceed to boot Linux.