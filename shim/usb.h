#pragma once

/*
 * ROSEMARY USB host bring-up (MT6785 / Synopsys DWC3), shim side.
 *
 * stage 0: read-only report of what the bootloader left behind.
 * stage 1: stage 0, then force host mode (GCTL.PrtCapDir) + port power and
 *          wait for a device to connect (PORTSC.CCS).
 *
 * Every step prints a marker *before* it touches the block, so if a register
 * access hangs the last line on screen names the exact address.
 */
void usb_probe(int stage);
