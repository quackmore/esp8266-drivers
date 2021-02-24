# ESP8266 digital I/O and common sensors drivers for NON_OS SDK

## Summary

Drivers for ESP8266 digital I/O and compatible sensors.
Based on Espressif NON-OS SDK (<https://github.com/espressif/ESP8266_NONOS_SDK>) and ESPBOT_2.0 (<https://github.com/quackmore/espbot_2.0>).

Every sensor is accessible through a single interface with class Esp8266_Sensor (drivers_sensor.hpp).

    class Esp8266_Sensor
    {
      public:
        Esp8266_Sensor() {}
        virtual ~Esp8266_Sensor() {}
    
        virtual void getSensor(sensor_t *) = 0;
        virtual int get_max_events_count(void) = 0;
        virtual void getEvent(sensors_event_t *, int idx = 0) = 0; // idx = 0 => latest event
                                                                   // idx = 1 => previous event
                                                                   // ...
        virtual void force_reading(void (*callback)(void *), void *param) = 0;
    };

The repository contains a full environment to compile, run and test the drivers.

To import the drivers in your project as a library use the following file:

+ lib/libdrivers.a

To import the drivers source files use files into src/drivers.

The drivers include files are:

+ esp8266_io.h
+ drivers_common_types.hpp
+ drivers_dht.hpp
+ drivers_di_sequence.h
+ drivers_dio_task.h
+ drivers_do_sequence.h
+ drivers_event_codes.h
+ drivers_max6675.hpp
+ drivers_sensor.hpp
+ drivers.h
+ drivers.hpp

Features:

+ digital output pulse sequences
+ digital input pulse sequence acquisition
+ DHT temperature and humidity sensors
+ MAX6675 temperature sensor

more to come ...

## Building the binaries and flashing ESP8266

Needed:

+ [Espressif NON-OS SDK](https://github.com/espressif/ESP8266_NONOS_SDK) in a separate repository.
+ [esp-open-sdk toolchain](https://github.com/pfalcon/esp-open-sdk) in a separate repository; build the bare Xtensa toolchain and leave ESP8266 SDK separate using:

      make STANDALONE=n

Build steps (linux)

+ Clone the repository.
+ Customize build variables according to your ESP8266 module and environment:

      cd <your path>/drivers
      ./gen_env.sh

      this will generate a env.sh file
      for instance a WEMOS D1 mini file will look like this:
      
      export SDK_DIR=<your path to ESP8266_NONOS_SDK>
      export COMPILE=gcc
      export BOOT=new
      export APP=1
      export SPI_SPEED=40
      export SPI_MODE=DIO
      export SPI_SIZE_MAP=4
      export COMPILE=gcc
      export COMPORT=<your COM port>
      export CC_DIR=<your path to compiler>
      export PATH=$PATH:<your path to compiler>
      export SDK_DIR=<your path to ESP8266_NONOS_SDK>
      export BOOT=new
      export APP=1
      export SPI_SPEED=40
      export FREQDIV=0
      export SPI_MODE=dio
      export MODE=2
      export SPI_SIZE_MAP=6
      export FLASH_SIZE=4096
      export LD_REF=2048
      export FLASH_OPTIONS=" write_flash -fm dio -fs 32m-c1 -ff 40m "
      export FLASH_INIT="0x3FB000 <your path to ESP8266_NONOS_SDK>/bin/blank.bin 0x3FC000 <your path to ESP8266_NONOS_SDK>/bin/esp_init_data_default_v08.bin 0x3FE000 <your path to ESP8266_NONOS_SDK>/blank.bin"

+ Building (commands available as tasks in case you are using Visual Studio)
  
  Clean project
  
      source ${workspaceFolder}/env.sh && make clean

  Building current user#.bin

      source ${workspaceFolder}/env.sh && make all

  Building user1.bin
  
      source ${workspaceFolder}/env.sh && make -e APP=1 all

  Building user2.bin
  
      source ${workspaceFolder}/env.sh && make -e APP=2 all

  Building both user1.bin and user2.bin
  
      source ${workspaceFolder}/env.sh && make -e APP=1 all && make -e APP=2 all

+ Flashing ESP8266 using esptool.py (checkout your distribution packages or [github repository](https://github.com/espressif/esptool)) (commands available as tasks in case you are using Visual Studio)
  
  Erase flash
  
      source ${workspaceFolder}/env.sh && make flash_erase

  Flash the bootloader
  
      source ${workspaceFolder}/env.sh && make flash_boot

  Flash init
  
      source ${workspaceFolder}/env.sh && make flash_init

  Flash current user#.bin
  
      source ${workspaceFolder}/env.sh && make flash

  Flash user1.bin
  
      source ${workspaceFolder}/env.sh && make -e APP=1 flash

  Flash user2.bin
  
      source ${workspaceFolder}/env.sh && make -e APP=2 flash

## License

The drivers comes with a [BEER-WARE] license.

Enjoy.
