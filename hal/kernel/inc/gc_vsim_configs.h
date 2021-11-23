/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2021 Vivante Corporation
*
*    Permission is hereby granted, free of charge, to any person obtaining a
*    copy of this software and associated documentation files (the "Software"),
*    to deal in the Software without restriction, including without limitation
*    the rights to use, copy, modify, merge, publish, distribute, sublicense,
*    and/or sell copies of the Software, and to permit persons to whom the
*    Software is furnished to do so, subject to the following conditions:
*
*    The above copyright notice and this permission notice shall be included in
*    all copies or substantial portions of the Software.
*
*    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
*    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
*    DEALINGS IN THE SOFTWARE.
*
*****************************************************************************
*
*    The GPL License (GPL)
*
*    Copyright (C) 2014 - 2021 Vivante Corporation
*
*    This program is free software; you can redistribute it and/or
*    modify it under the terms of the GNU General Public License
*    as published by the Free Software Foundation; either version 2
*    of the License, or (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not, write to the Free Software Foundation,
*    Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*****************************************************************************
*
*    Note: This software is released under dual MIT and GPL licenses. A
*    recipient may use this file under the terms of either the MIT license or
*    GPL License. If you wish to use only one license not the other, you can
*    indicate your decision by deleting one of the above license notices in your
*    version of this file.
*
*****************************************************************************/


VSimulatorConfig vsimConfigs[] = {
        {"VIP7000", 0x7000, 0x6100, 0x5070003, 0x0, 0x0000},
        {"VIP8000ULF", 0x8000, 0x6212, 0x5080003, 0x0, 0x21},
        {"VIP8000ULS", 0x8000, 0x7000, 0x15080003, 0x0, 0x25},
        {"VIP8000ULQ", 0x8000, 0x7000, 0x45080003, 0x0, 0x26},
        {"VIP8000NANOD", 0x8000, 0x7003, 0x25080001, 0x0, 0x2a},
        {"VIP8000NANOQ", 0x8000, 0x7100, 0x45080001, 0x0, 0x24},
        {"VIP8000NANOQI", 0x8000, 0x7004, 0x45080009, 0x0, 0x7d},
        {"VIP8000NANOS", 0x8000, 0x7003, 0x15080001, 0x0, 0x23},
        {"VIP8000LO", 0x8000, 0x7000, 0x85080002, 0x0, 0x2f},
        {"VIP8000NANODI", 0x8000, 0x7000, 0x25080009, 0x0, 0x7e},
        {"VIPNANODI_PID0X7F", 0x00008000, 0x00007110, 0x25080009, 0x00000000, 0x7f},
        {"VIPNANOQ_PID0X24", 0x00008000, 0x0007100, 0x45080001, 0x00000000, 0x24},

        {"VIP8000LO_PID0X2F", 0x8000, 0x7000, 0x85080002, 0x0, 0x2f},
        {"VIP8000ULF_PID0X21", 0x00008000, 0x00006212, 0x05080003, 0x00000000, 0x21},
        {"VIP8000ULQ_PID0X26", 0x00008000, 0x0007000, 0x45080003, 0x00000000, 0x26},
        {"VIP8000ULS_PID0X25", 0x00008000, 0x0007000, 0x15080003, 0x00000000, 0x25},
        {"VIPNANODI_PID0X7E", 0x00008000, 0x0007000, 0x25080009, 0x00000000, 0x7e},
        {"VIPNANOD_PID0X2A", 0x00008000, 0x0007003, 0x25080001, 0x00000000, 0x2a},
        {"VIPNANOQI_PID0X7D", 0x00008000, 0x00007004, 0x45080009, 0x00000001, 0x7d},
        {"VIPNANOS_PID0X23", 0x00008000, 0x00007003, 0x15080001, 0x00000000, 0x23},
        {"VIPNANOD_V7_0_1", 0x8000, 0x7010, 0x25080001, 0x0, 0x0},
        {"VIPNANOD_V7_1", 0x8000, 0x7100, 0x25080001, 0x0, 0x0},
        {"VIPNANOO_V7_0_1", 0x8000, 0x7010, 0x85080001, 0x0, 0x0},
        {"VIPNANOO_V7_1", 0x8000, 0x7100, 0x85080001, 0x0, 0x0},
        {"VIPNANOQ_V7_0_1", 0x8000, 0x7010, 0x45080001, 0x0, 0x0},
        {"VIPNANOQ_V7_1", 0x8000, 0x7100, 0x45080001, 0x0, 0x0},
        {"VIPNANOS_V7_0_1", 0x8000, 0x7010, 0x15080001, 0x0, 0x0},
        {"VIPNANOS_V7_1", 0x8000, 0x7100, 0x15080001, 0x0, 0x0},
        {"VIPNANOQ_PID0X82", 0x00008000, 0x00007100, 0x45080001, 0x00000000, 0x82},
        {"VIPNANODI_PID0X84", 0x00008000, 0x00007131, 0x25080009, 0x00000000, 0x84},
        {"VIPNANOSI_PID0X80", 0x00008000, 0x00007120, 0x15080009, 0x00000000, 0x80},
        {"VIP8000ULS_PID0X83", 0x00008000, 0x00007005, 0x15080003, 0x00000000, 0x83},

        {"VIPNANOD_V7_2", 0x8000, 0x7200, 0x25080001, 0x0, 0x0},
        {"VIPNANOO_V7_2", 0x8000, 0x7200, 0x85080001, 0x0, 0x0},
        {"VIPNANOQ_V7_2", 0x8000, 0x7200, 0x45080001, 0x0, 0x0},
        {"VIPNANOS_V7_2", 0x8000, 0x7200, 0x15080001, 0x0, 0x0},
        {"VIPNANOD_V7_3", 0x8000, 0x7300, 0x25080001, 0x0, 0x0},
        {"VIPNANOO_V7_3", 0x8000, 0x7300, 0x85080001, 0x0, 0x0},
        {"VIPNANOQ_V7_3", 0x8000, 0x7300, 0x45080001, 0x0, 0x0},
        {"VIPNANOS_V7_3", 0x8000, 0x7300, 0x15080001, 0x0, 0x0},
        {"VIP8000LQI_PID0X85", 0x00008000, 0x0007200, 0x4508000a, 0x00000000, 0x85},

        {"VIPNANOD_PLUS_V7_1", 0x8000, 0x7100, 0x05080001, 0x06000000, 0x0},
        {"VIPNANOD_PLUS_V7_2", 0x8000, 0x7200, 0x05080001, 0x06000000, 0x0},
        {"VIPNANOD_PLUS_V7_3", 0x8000, 0x7300, 0x05080001, 0x06000000, 0x0},
        {"VIPNANOO_PLUS_V7_1", 0x8000, 0x7100, 0x05080001, 0x18000000, 0x0},
        {"VIPNANOO_PLUS_V7_2", 0x8000, 0x7200, 0x05080001, 0x18000000, 0x0},
        {"VIPNANOO_PLUS_V7_3", 0x8000, 0x7300, 0x05080001, 0x18000000, 0x0},
        {"VIPNANOQ_PLUS_V7_1", 0x8000, 0x7100, 0x05080001, 0x0C000000, 0x0},
        {"VIPNANOQ_PLUS_V7_2", 0x8000, 0x7200, 0x05080001, 0x0C000000, 0x0},
        {"VIPNANOQ_PLUS_V7_3", 0x8000, 0x7300, 0x05080001, 0x0C000000, 0x0},
        {"VIPNANOS_PLUS_V7_1", 0x8000, 0x7100, 0x05080001, 0x03000000, 0x0},
        {"VIPNANOS_PLUS_V7_2", 0x8000, 0x7200, 0x05080001, 0x03000000, 0x0},
        {"VIPNANOS_PLUS_V7_3", 0x8000, 0x7300, 0x05080001, 0x03000000, 0x0},

        {"VIPNANOQI_PID0X88", 0x00008000, 0x00007120, 0x45080009, 0x00000000, 0x88},
        {"VIPNANOD_PID0X89", 0x8000, 0x7110, 0x25080001, 0x0, 0x89},

        {"VIP8000OI_MP_PID0X86", 0x00008000, 0x00007300, 0x05080008, 0x10000000, 0x86},
        {"VIP8000ULDI_PID0X92", 0x00008000, 0x00007120, 0x0508000b, 0x06000000, 0x92},
        {"VIPNANOSI_PID0X9", 0x00008000, 0x00007131, 0x05000009, 0x02000000, 0x9},
        {"VIPNANOSI_PID0X9", 0x00008000, 0x00007131, 0x05000009, 0x02000000, 0x9},

        {"VIP8000LH_MP_V8_0", 0x8000, 0x8000, 0x05080002, 0x20000000, 0x0},
        {"VIP8000ULO_MP_V8_0", 0x8000, 0x8000, 0x05080003, 0x10000000, 0x0},
        {"VIP8000LH_PLUS_MP_V8_0", 0x8000, 0x8000, 0x05080002, 0x30000000, 0x0},
        {"VIP8000ULO_PLUS_MP_V8_0", 0x8000, 0x8000, 0x05080003, 0x18000000, 0x0},
        {"VIPNANOD_V8_0", 0x8000, 0x8000, 0x05080001, 0x04000000, 0x0},
        {"VIPNANOO_V8_0", 0x8000, 0x8000, 0x05080001, 0x10000000, 0x0},
        {"VIPNANOQ_V8_0", 0x8000, 0x8000, 0x05080001, 0x08000000, 0x0},
        {"VIPNANOS_V8_0", 0x8000, 0x8000, 0x05080001, 0x02000000, 0x0},
        {"VIPNANOD_PLUS_V8_0", 0x8000, 0x8000, 0x05080001, 0x06000000, 0x0},
        {"VIPNANOO_PLUS_V8_0", 0x8000, 0x8000, 0x05080001, 0x18000000, 0x0},
        {"VIPNANOQ_PLUS_V8_0", 0x8000, 0x8000, 0x05080001, 0x0C000000, 0x0},
        {"VIPNANOS_PLUS_V8_0", 0x8000, 0x8000, 0x05080001, 0x03000000, 0x0},

        {"VIPPICO_V1_PID0X87", 0x8000, 0x7120, 0x08000001, 0x01000000, 0x87},
        {"VIPPICO_V2_PID0X93", 0x8000, 0x7130, 0x8000001, 0x1000000, 0x93},
        {"VIPPICO_V3_PID0X99", 0x00008000, 0x00007131, 0x08000001, 0x02000000, 0x99},
        {"VIPPICO_V8_0", 0x8000, 0x8000, 0x08000001, 0x02000000, 0x0},

        {"VIP8000ULSI_PID0X98", 0x00008000, 0x00007121, 0x0508000b, 0x02000000, 0x98},
        {"VIP8000OI_MP_PID0X9B", 0x8000, 0x7300, 0x508000a, 0x8000000, 0x9b},
        {"VIPNANOSI_PID0X9", 0x00008000, 0x00007131, 0x05000009, 0x02000000, 0x9},
        {"VIP8000NANOSI_PLUS_PID0X9F", 0x00008000, 0x00008002, 0x05080009, 0x06000000, 0x9f},

        {"VIPNANOQI_PID0XA1", 0x00008000, 0x00007131, 0x05000009, 0x08000000, 0xa1},
        {"VIP8000NANOQI_PLUS_PID0XA3", 0x8000, 0x8000, 0x5080009, 0x16000000, 0xa3},
        {"VIP8000OI_PID0XA4", 0x00008000, 0x00008102, 0x05080008, 0x20000000, 0xa4},
        {"VIPNANOSI_PID0XA5", 0x00008000, 0x00008003, 0x05080009, 0x04000000, 0xa5},
        {"VIPNANONI_PID0XA2", 0x00008000, 0x00007121, 0x05000009, 0x01000000, 0xa2},
        {"VIP8000NANOSI_PLUS_PID0XA6", 0x00008000, 0x00008002, 0x05080009, 0x07000000, 0xa6},
        {"VIP9000PICO4C_PID0XAA", 0x00008000, 0x00008000, 0x08000001, 0x01000000, 0xaa},
        {"VIP8000NANOQI_PLUS_PID0XA9", 0x00008000, 0x00008001, 0x05080009, 0x18000000, 0xa9},
        {"VIP8000NANOSI_PID0XAC", 0x00008000, 0x00008000, 0x05080009, 0x04000000, 0xac},
        {"VIP8000NANOSI_PLUS_PID0XAE", 0x00008000, 0x00008020, 0x05080009, 0x07000000, 0xae},
        {"VIP8000NANONI_PID0XAD", 0x00008000, 0x00008020, 0x05080009, 0x02000000, 0xad},

        {"VIP8000NANOQI_PLUS_PID0XB1", 0x00008000, 0x00008004, 0x05080009, 0x18000000, 0xb1},
        {"VIP8000NANOSI_PLUS_PID0XB2", 0x00008000, 0x00008000, 0x05080009, 0x06000000, 0xb2},
        {"VIP8000NANOSI_PLUS_PID0XB3", 0x00008000, 0x00008003, 0x05080009, 0x06000000, 0xb3},
        {"VIP8000LSI_PLUS_PID0XB4", 0x00008000, 0x00008003, 0x0508000a, 0x06000000, 0xb4},
        {"VIP8000NANOSI_PID0XB5", 0x00008000, 0x00008003, 0x05080009, 0x04000000, 0xb5},
        {"VIP8000NANOSI_PLUS_PID0XA0", 0x00008000, 0x00008002, 0x05080009, 0x06000000, 0xa0},
        {"VIP9000NANODI_PID0XB6", 0x00009000, 0x00008101, 0x05090009, 0x08000000, 0xb6},
        {"VIP9000NANOD_PID0XB7", 0x9000, 0x0, 0x5090001, 0x8000000, 0xb7},
        {"VIP9000NANOSI_PLUS_PID0XB8", 0x00009000, 0x00008101, 0x05090009, 0x06000000, 0xb8},

        {"VIP9000NANOSI_PID0XB9", 0x00009000, 0x00008101, 0x05090009, 0x04000000, 0xb9},
        {"VIP9000ULSI_PID0XBA", 0x00009000, 0x00008301, 0x0509000b, 0x04000000, 0xba},
        {"VIP9000NANOSI_PID0XBB", 0x00009000, 0x0008101, 0x05090009, 0x04000000, 0xbb},
        {"VIP9000NANOSI_PLUS_PID0XBD", 0x9000, 0x0, 0x5090009, 0x7000000, 0xbd},
        {"VIP9000NANODI_PID0XBE", 0x00009000, 0x00008102, 0x05090009, 0x08000000, 0xbe},

        {"VIP9000NANOSI_PID0XC0", 0x00009000, 0x0008101, 0x05090009, 0x04000000, 0xc0},
        {"VIP9000NANOQI_PLUS_PID0XC1", 0x00009000, 0x00008203, 0x05090009, 0x16000000, 0xc1},
        {"VIP9000NANOSI_PID0XC2", 0x00009000, 0x00008202, 0x05090009, 0x04000000, 0xc2},
        {"VIPPICO_PID0XC3", 0x00009000, 0x00009000, 0x05090009, 0x02000000, 0xc3},
        {"VIPPICO_PID0XC4", 0x00008000, 0x00007005, 0x08080008, 0x01000000, 0xc4},
        {"VIP9000NANOSI_PLUS_PID0XC8", 0x00009000, 0x00008202, 0x05090009, 0x06000000, 0xc8},
        {"VIP9000NANOS_PID0XC9", 0x9000, 0x0, 0x5090001, 0x4000000, 0xc9},
        {"VIP9000NANOS_PID0XC5", 0x00009000, 0x00008203, 0x05090001, 0x04000000, 0xc5},
        {"VIP9000NANODI_PLUS_PID0XC7", 0x9000, 0x0, 0x5090009, 0xb000000, 0xc7},
        {"VIP9000NANOSI_PID0XCD", 0x00009000, 0x00008301, 0x05090009, 0x04000000, 0xcd},
        {"VIP9000NANOQI_PLUS_PID0XCE", 0x00009000, 0x00008203, 0x05090009, 0x16000000, 0xce},
        {"VIP9000NANOSI_PID0XCC", 0x00009000, 0x00008205, 0x05090009, 0x04000000, 0xcc},
        {"VIP9000NANOS_PID0XCB", 0x00009000, 0x00008204, 0x05090001, 0x04000000, 0xcb},
        {"VIP9000ULD_PLUS_MP4_PID0XC6", 0x00009000, 0x00008205, 0x05090003, 0x30000000, 0xc6},

        {"VIP9000NANOO_PID0XCF", 0x00009000, 0x00008300, 0x05090001, 0x20000000, 0xcf},
        {"VIP9000NANOO_MP4_PID0XD0", 0x00009000, 0x00008300, 0x05090001, 0x20000000, 0xd0},
        {"VIP8000NANOSI_PLUS_PID0XD1", 0x00008000, 0x00008020, 0x05080009, 0x07000000, 0xd1},
        {"VIP9000LQ_PID0XD2", 0x00009000, 0x00008205, 0x05090002, 0x10000000, 0xd2},
        {"VIP9000NANOSI_PID0XD3", 0x00009000, 0x00008204, 0x05090009, 0x04000000, 0xd3},
        {"VIP9000NANODI_PID0XD4", 0x00009000, 0x00008205, 0x05090009, 0x08000000, 0xd4},
        {"VIP9400O_PID0XD9", 0x00009400, 0x00009003, 0x05094000, 0x80000000, 0xd9},
        {"VIP9000O_MP4", 0x9400, 0x0, 0x0, 0x0, 0x0},
        {"VIP9000NANOS_PID0XDA", 0x00009000, 0x00008205, 0x05090001, 0x04000000, 0xda},
        {"VIP9000NANODI_PID0XDB", 0x00009000, 0x00008203, 0x05090001, 0x08000000, 0xdb},
        {"VIP9000NANONI_PID0XDC", 0x00009000, 0x00008205, 0x05090009, 0x02000000, 0xdc},
        {"VIP9000Q_PID0XDD", 0x00009000, 0x00008205, 0x05090000, 0x10000000, 0xdd},
        {"VIP9000Q_MP2_PID0XDE", 0x00009000, 0x00008206, 0x05090000, 0x20000000, 0xde},

        {"VIP9000PICO_PID0XE0", 0x00009000, 0x00008206, 0x0809000c, 0x01000000, 0xe0},
        {"VIP9000NANOD_MP4_PID0XE1", 0x00009000, 0x00008205, 0x05090001, 0x20000000, 0xe1},
        {"VIP9300O_PID0XE2", 0x00009300, 0x00009000, 0x05093000, 0x60000000, 0xe2},
        {"VIP9000NANODI_PLUS_MP2_PID0XE3", 0x00009000, 0x00008303, 0x05090009, 0x14000000, 0xe3},
        {"VIP9000NANODI_PLUS_V2_PID0XE4", 0x00009000, 0x00008301, 0x05090009, 0x0a000000, 0xe4},
        {"VIP9000NANODI_PID0XE5", 0x00009000, 0x00008302, 0x05090009, 0x08000000, 0xe5},
        {"VIP9000NANOQ_PID0XE6", 0x00009000, 0x00008302, 0x05090001, 0x10000000, 0xe6},
        {"VIP9000NANOSI_PID0XE7", 0x00009000, 0x00008302, 0x05090009, 0x04000000, 0xe7},
        {"VIP9000NANODI_PID0XE8", 0x00009000, 0x00008302, 0x05090009, 0x08000000, 0xe8},
        {"VIP9000PICO_PID0XEC", 0x00009000, 0x0, 0x0809000c, 0x01000000, 0xec},
        {"VIP9000PICO_PID0XED", 0x00009000, 0x00008302, 0x0809000c, 0x03000000, 0xed},

        {"VIP9400O_V9_0", 0x9400, 0x9000, 0x0, 0x0, 0xF9},

        {"VIPNANOS_V8_2", 0x9000, 0x8200, 0x5090001, 0x4000000, 0x0},
        {"VIPNANOS_PLUS_V8_2", 0x9000, 0x8200, 0x5090001, 0x6000000, 0x0},
        {"VIPNANOD_V8_2", 0x9000, 0x8200, 0x5090001, 0x8000000, 0x0},
        {"VIPNANOD_PLUS_V8_2", 0x9000, 0x8200, 0x5090001, 0xc000000, 0x0},
        {"VIPNANOQ_V8_2", 0x9000, 0x8200, 0x5090001, 0x10000000, 0x0},
        {"VIPNANOQ_PLUS_V8_2", 0x9000, 0x8200, 0x5090001, 0x18000000, 0x0},
        {"VIPNANOO_V8_2", 0x9000, 0x8200, 0x5090001, 0x20000000, 0x0},
        {"VIPNANOO_PLUS_V8_2", 0x9000, 0x8200, 0x5090001, 0x30000000, 0x0},

        {"VIPNANOS_V8_3", 0x9000, 0x8300, 0x5090001, 0x4000000, 0x0},
        {"VIPNANOS_PLUS_V8_3", 0x9000, 0x8300, 0x5090001, 0x6000000, 0x0},
        {"VIPNANOD_V8_3", 0x9000, 0x8300, 0x5090001, 0x8000000, 0x0},
        {"VIPNANOD_PLUS_V8_3", 0x9000, 0x8300, 0x5090001, 0xc000000, 0x0},
        {"VIPNANOQ_V8_3", 0x9000, 0x8300, 0x5090001, 0x10000000, 0x0},
        {"VIPNANOQ_PLUS_V8_3", 0x9000, 0x8300, 0x5090001, 0x18000000, 0x0},
        {"VIPNANOO_V8_3", 0x9000, 0x8300, 0x5090001, 0x20000000, 0x0},
        {"VIPNANOO_PLUS_V8_3", 0x9000, 0x8300, 0x5090001, 0x30000000, 0x0},

        {"VIPNANOS_V9_0", 0x9000, 0x9000, 0x0000000, 0x0000000, 0xF1},
        {"VIPNANOS_PLUS_V9_0", 0x9000, 0x9000, 0x0000000, 0x0000000, 0xF2},
        {"VIPNANOD_V9_0", 0x9000, 0x9000, 0x0000000, 0x0000000, 0xF3},
        {"VIPNANOD_PLUS_V9_0", 0x9000, 0x9000, 0x0000000, 0x0000000, 0xF4},
        {"VIPNANOQ_V9_0", 0x9000, 0x9000, 0x0000000, 0x00000000, 0xF5},
        {"VIPNANOQ_PLUS_V9_0", 0x9000, 0x9000, 0x0000000, 0x00000000, 0xF6},
        {"VIPNANOO_V9_0", 0x9000, 0x9000, 0x0000000, 0x00000000, 0xF7},
        {"VIPNANOO_PLUS_V9_0", 0x9000, 0x9000, 0x0000000, 0x00000000, 0xF8},

        {"GCNANOULTRA31_VIP2_PID0X15", 0x00008000, 0x00006205, 0x00080003, 0x00000000, 0x15},

        {"GC7000_BG4CT", 0x7000, 0x6008, 0x70004, 0x0, 0x0},
        {"GC7000XSVX_IMX8QM", 0x7000, 0x6009, 0x70008, 0x0, 0x0},
        {"GC7000L_IMX8QXP", 0x7000, 0x6214, 0x70002, 0x0, 0x30},
        {"GC7000L_IMX8MQ", 0x7000, 0x6214, 0x70002, 0x0, 0x30},
        {"GC7000NANOULTRA_IMX8MM", 0x00000600, 0x00004653, 0x00070005, 0x00000000, 0x102},
        {"GC7000NANOULTRA_IMX7ULP", 0x600, 0x4653, 0x70005, 0x0, 0x0},
        {"GC7000NANO_PID0X100", 0x400, 0x4652, 0x70001, 0x0, 0x100},
        {"GC2000_PLUS_IMX6QP", 0x2000, 0xffff5450, 0x0, 0x0, 0x0},
        {"GC2000_IMX6Q", 0x2000, 0x5108, 0x0, 0x0, 0x0},
        {"GC880_IMX6DL", 0x880, 0x5106, 0x0, 0x0, 0x0},
        {"GC400_IMX6SX", 0x400, 0x4645, 0x0, 0x0, 0x0},
        {"GC7000NANOULTRA_A5", 0x600, 0x4652, 0x70005, 0x0, 0x0},
        {"GC7000NANO_WILDCAT", 0x00000400, 0x00004652, 0x00070001, 0x00000000, 0x100},
        {"GC7000ULVX_ARTPEC7", 0x00007000, 0x00006203, 0x0007000f, 0x00000000, 0x60},
        {"GC8000ULVX_ARTPEC8", 0x8000, 0x6205, 0x8000F, 0x1, 0x3},
        {"GC7000NANOULTRA_M20", 0x00000600, 0x00004653, 0x00070005, 0x00000000, 0x104},
        {"CC8000_GINVP", 0x00008000, 0x00006331, 0x06080000, 0x00000000, 0x51},
        {"CC8400_PID0X52", 0x00008400, 0x00006302, 0x06084000, 0x00000000, 0x52},
        {"GC7000UL_IMX8MP", 0x00007000, 0x00006204, 0x00070003, 0x00000001, 0x11},
        {"GC7000UL_IMX8MN", 0x00007000, 0x00006203, 0x00070003, 0x00000000, 0x4},
        {"GC8000XS_PID0XD", 0x00008000, 0x00006212, 0x00080004, 0x00000000, 0xd},
        {"GC8000UL_PID0X18", 0x00008000, 0x00006206, 0x00080003, 0x00000000, 0x18},
        {"GC8400XS_PID0X41", 0x00008400, 0x00006304, 0x00084004, 0x00000000, 0x41},
        {"GC8400XS_PID0X49", 0x00008400, 0x00006304, 0x00084004, 0x00000000, 0x49},
        {"CC8400_MP4_PID0X54", 0x00008400, 0x00006305, 0x06084000, 0x00000000, 0x54},
        {"VIP9000PICO_PID0XEE", 0x00009000, 0x00008303, 0x0809000c, 0x03000000, 0xee},
        {"VIP9000LD_MP4_PID0X10000001", 0x00009000, 0x00008303, 0x05090002, 0x20000000, 0x10000001},
        {"VIP9000LD_MP4_PID0X10000017", 0x00009000, 0x00008303, 0x05090002, 0x20000000, 0x10000017},
        {"VIP9000NANOD_PID0X10000002", 0x00009000, 0x00009000, 0x05090001, 0x08000000, 0x10000002},
        {"VIP9000NANOSIPLUS_PID0X10000003", 0x00009000, 0x00008302, 0x05090009, 0x06000000, 0x10000003},
        {"GC8400XS_PID0X4B", 0x00008400, 0x00006306, 0x00084004, 0x00000000, 0x4b},
        {"VIP9000NANODIPLUS_PID0X10000005", 0x00009000, 0x00009000, 0x0809000c, 0x0c000000, 0x10000005},
        {"VIP9000NANOSI_PID0X10000007", 0x00009000, 0x00008303, 0x05090009, 0x04000000, 0x10000007},
        {"VIP9000NANOSPLUS_PID0X10000008", 0x00009000, 0x00008303, 0x05090001, 0x06000000, 0x10000008},
        {"VIP9000NANOQIPLUS_PID0X10000004", 0x00009000, 0x00008206, 0x05090009, 0x16000000, 0x10000004},
        {"VIP9000NANODI_PID0X10000009", 0x00009000, 0x00009002, 0x05090009, 0x08000000, 0x10000009},
        {"VIP9400LO_PID0X1000000A", 0x00009400, 0x00009003, 0x05094002, 0x80000000, 0x1000000a},
        {"CC8400_PID0X56", 0x00008400, 0x00006305, 0x06084000, 0x00000000, 0x56},
        {"VIV_GPU_IMX8ULP", 0x7000, 0x6205, 0x70007, 0x0, 0x12},
        {"VIP9000NANODIPLUS_PID0X1000000B", 0x00009000, 0x00009002, 0x05090009, 0x0c000000, 0x1000000b},
        {"GC9100_PID0X81", 0x00009100, 0x00008000, 0x00091000, 0x00000000, 0x81},
        {"GC8000L_V2_PID0X32", 0x00008000, 0x00006214, 0x00080002, 0x00000000, 0x32},
        {"VIP9000NANOS_PID0XD8", 0x9000, 0x9000, 0x5090002, 0x20000000, 0xd8},
        {"VIP9000NANOS_PID0X1000000E", 0x00009000, 0x00009001, 0x05090001, 0x04000000, 0x1000000e},
        {"VIP9000ULSIPLUS_PID0X1000000F", 0x00009000, 0x00009001, 0x0509000b, 0x06000000, 0x1000000f},
        {"VIP9000NANONI_PID0XEF", 0x00009000, 0x00008302, 0x05090009, 0x02000000, 0xef},
        {"VIP9000ULSIPLUS_PID0X10000010", 0x00009000, 0x00009003, 0x0509000b, 0x06000000, 0x10000010},
        {"VIP9000ULSIPLUS_MP2_PID0X10000011", 0x00009000, 0x00009003, 0x0509000b, 0x0c000000, 0x10000011},
        {"VIP9000NANOSIPLUS_PID0X10000012", 0x00009000, 0x00009001, 0x05090009, 0x06000000, 0x10000012},
        {"VIP9000NANONI_PID0X10000013", 0x00009000, 0x00009002, 0x05090009, 0x01000000, 0x10000013},
        {"VIP9000ULSI_PID0X10000014", 0x00009000, 0x00008303, 0x0509000b, 0x04000000, 0x10000014},
        {"VIP9000ULSI_PID0XBC", 0x00009000, 0x00009000, 0x0509000b, 0x05000000, 0xbc},
        {"VIP9000NANOSI_PID0X10000018", 0x00009000, 0x00009000, 0x05090009, 0x04000000, 0x10000018},
        {"VIP9000ULDI_PID0X10000015", 0x00009000, 0x00009000, 0x0509000b, 0x08000000, 0x10000015},
        {"VIP9000PICO_PID0X1000001C", 0x00009000, 0x00009001, 0x0809000c, 0x03000000, 0x1000001c},
        {"VIPNANOD_PID0X1000100A", 0x00009100, 0x00009110, 0x05090001, 0x08000000, 0x1000100a},
        {"VIP9000NANOSIPLUS_PID0X10000016", 0x00009000, 0x00009003, 0x05090009, 0x06000000, 0x10000016},
        {"VIP9000LDPLUS_PID0X1000001D", 0x00009000, 0x00009003, 0x05090002, 0x0c000000, 0x1000001d},
        {"VIP9000NANODI_PID0X1000001E", 0x9000, 0x9110, 0x05090009, 0x08000000, 0x1000001E},
        {"VIP9000NANODI_PLUS_PID0XDF", 0x00009000, 0x00008302, 0x05090009, 0x0c000000, 0xdf},
        {"GC8000L_PID0X31", 0x00008000, 0x00006213, 0x00080002, 0x00000000, 0x31},
        {"VIPNANOSI_PID0X9", 0x00008000, 0x00007131, 0x05000009, 0x02000000, 0x9},
        {"CC8000L_PID0X55", 0x00008000, 0x00006332, 0x06080002, 0x00000000, 0x55},
        {"VIPNANOQI_PLUS_PID0XA3", 0x00008000, 0x00008000, 0x05080009, 0x16000000, 0xa3},
        {"VIPNANOSI_PID0X9", 0x00008000, 0x00007131, 0x05000009, 0x02000000, 0x9},
        {"VIPNANOSI_PID0X96", 0x00008000, 0x00007005, 0x05000009, 0x02000000, 0x96},
        {"VIPNANOSI_PID0X97", 0x00008000, 0x00007121, 0x05000009, 0x02000000, 0x97},
        {"VIP9400NANOO_V9_1_0", 0x00009400, 0x00009100, 0x0, 0x0, 0x10001010},
        {"VIP9400NANOO_V9_1_1", 0x00009400, 0x00009110, 0x0, 0x0, 0x10001011},
        {"VIPNANOS_V9_1_0", 0x9000, 0x00009100, 0x0, 0x0, 0x10001000},
        {"VIPNANOSPLUS_V9_1_0", 0x9000, 0x00009100, 0x0, 0x0, 0x10001001},
        {"VIPNANOD_V9_1_0", 0x9000, 0x00009100, 0x0, 0x0, 0x10001002},
        {"VIPNANODPLUS_V9_1_0", 0x9000, 0x00009100, 0x0, 0x0, 0x10001003},
        {"VIPNANOQ_V9_1_0", 0x9000, 0x00009100, 0x0, 0x0, 0x10001004},
        {"VIPNANOQPLUS_V9_1_0", 0x9000, 0x00009100, 0x0, 0x0, 0x10001005},
        {"VIPNANOO_V9_1_0", 0x9000, 0x00009100, 0x0, 0x0, 0x10001006},
        {"VIPNANOOPLUS_V9_1_0", 0x9000, 0x00009100, 0x0, 0x0, 0x10001007},
        {"VIPNANOS_V9_1_1", 0x9000, 0x00009110, 0x0, 0x0, 0x10001008},
        {"VIPNANOSPLUS_V9_1_1", 0x9000, 0x00009110, 0x0, 0x0, 0x10001009},
        {"VIPNANOD_V9_1_1", 0x9100, 0x00009110, 0x05090001, 0x08000000, 0x1000100a},
        {"VIPNANODPLUS_V9_1_1", 0x9000, 0x00009110, 0x0, 0x0, 0x1000100b},
        {"VIPNANOQ_V9_1_1", 0x9000, 0x00009110, 0x0, 0x0, 0x1000100c},
        {"VIPNANOQPLUS_V9_1_1", 0x9000, 0x00009110, 0x0, 0x0, 0x1000100d},
        {"VIPNANOO_V9_1_1", 0x9000, 0x00009110, 0x0, 0x0, 0x1000100e},
        {"VIPNANOOPLUS_V9_1_1", 0x9000, 0x00009110, 0x0, 0x0, 0x1000100f},

};


