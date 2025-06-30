#!/usr/bin/env python3
import minimalmodbus
import serial
import struct
import time
import binascii

instrument = minimalmodbus.Instrument('/dev/cu.usbserial-124330', 10) 
instrument.serial.port                     
instrument.serial.baudrate = 19200         
instrument.serial.bytesize = 8
instrument.serial.parity   = serial.PARITY_NONE
instrument.serial.stopbits = 1
instrument.serial.timeout  = 100       
instrument.address                         
instrument.mode = minimalmodbus.MODE_RTU 
instrument.clear_buffers_before_each_transaction = True


BOOT_MODE_COIL = 1

FLASH_PAGE_NUMBER_ADDRESS = 40000
FLASH_PAGE_START_REGISTER = 41000

FLASH_PAGE_CRC_REGISTER = 30000

PERFORM_UPDATE_COIL = 1000
VALIDATE_UPDATE_COIL = 1001
BUILD_DATE_REGISTER = 39000
BUILD_TIME_REGISTER = 39050

MSG_SIZE = 128             
PACKETS_PER_BLOCK = 16
FLASH_PAGE_SIZE = MSG_SIZE * PACKETS_PER_BLOCK  

def bytes_to_registers(data_chunk):
    if len(data_chunk) % 2 != 0:
        data_chunk += b'\x00'
    return list(struct.unpack('>' + 'H' * (len(data_chunk) // 2), data_chunk))


def crc32mpeg2(buf, crc=0xffffffff):
    for val in buf:
        crc ^= val << 24
        for _ in range(8):
            crc = crc << 1 if (crc & 0x80000000) == 0 else (crc << 1) ^ 0x104c11db7
    return crc

def crc32_to_registers(crc):
    high = (crc >> 16) & 0xFFFF
    low = crc & 0xFFFF
    return [high, low]

def get_build_date():
    data = instrument.read_registers(BUILD_DATE_REGISTER, 30, functioncode=3)
    text = bytes(data).decode('utf-8').rstrip('\x00')
    print("Build date:", text)

def get_version():
    data = instrument.read_registers(BUILD_TIME_REGISTER, 30, functioncode=3)
    text = bytes(data).decode('utf-8').rstrip('\x00')
    print("Version:", text)

def app_info():
    print("======================== APP ========================")
    get_build_date()
    get_version()
    print("=====================================================")

def bootloader_info():
    print("===================== BOOTLOADER =====================")
    get_build_date()
    get_version()
    print("======================================================")







if(instrument.read_bit(BOOT_MODE_COIL, functioncode=1) == 1):
    app_info()
    print("BDCD in APP, switching to BOOTLOADER")
    instrument.write_bit(BOOT_MODE_COIL, 0, functioncode=5)
    time.sleep(3)
else:
    print("BDCD already in BOOTLOADER")


bootloader_info()

with open("./app/Debug/app.bin", "rb") as f:
    offset = 0
    block_index = 0

    while True:
        block = f.read(FLASH_PAGE_SIZE)
        if not block:
            print("Flashing done.")
            break

        # Padding with zeros up to flash page size
        if len(block) < FLASH_PAGE_SIZE:
            print(f"Flash page {block_index}: has less than 1024 bytes ({len(block)} B), filling with zeros...")
            block += b'\x00' * (FLASH_PAGE_SIZE - len(block))    
        try:
            instrument.write_register(FLASH_PAGE_NUMBER_ADDRESS, block_index, 0)
        except Exception as e:
            print(f"Error writing PAGE number to PAGE register: {e}")
            exit(1)

        # Send flash page - 16 packages(128 bytes each) 
        for i in range(PACKETS_PER_BLOCK):
            chunk = block[i * MSG_SIZE:(i + 1) * MSG_SIZE]
            registers = bytes_to_registers(chunk)
            register_address = FLASH_PAGE_START_REGISTER + offset
            print(f"[{i+1}/{PACKETS_PER_BLOCK}] Sending {len(registers)} register to address {register_address}")
            try:
                instrument.write_registers(register_address, registers)
            except Exception as e:
                print(f"Error sending flash page: {e}")
                exit(1)
            offset += len(registers)

        # Calculate CRC32 (STM32-style) and write to CRC registers
        crc = crc32mpeg2(block)
        crc_registers = crc32_to_registers(crc)
        print(f"Flash page {block_index}: CRC32 = {crc:08X}, writing to register {FLASH_PAGE_CRC_REGISTER}")

        try:
            data = instrument.read_registers(FLASH_PAGE_CRC_REGISTER, 2, functioncode=3)
            if(data == crc_registers):
                print("CRC Valid, sending next flash page")
            else:
                print(f"CRC invalid for:{block_index} block, exiting");
                exit(1)
        except Exception as e:
            print(f"Error writing to CRC register: {e}")
            exit(1)
        offset = 0
        block_index += 1


print(f"Sending request to update")     
try:
    instrument.write_bit(PERFORM_UPDATE_COIL,1, functioncode=5)
except Exception as e:
    print(f"Error writing request to update: {e}")
    exit(1)

print("Waiting for APP to boot")
time.sleep(5)

if(instrument.read_bit(BOOT_MODE_COIL, functioncode=1) == 1):
    instrument.write_bit(VALIDATE_UPDATE_COIL,1, functioncode=5)
    print("BDCD booted, OTW successful")
else:
    print("ERROR - BDCD in BOOTLOADER")


app_info()