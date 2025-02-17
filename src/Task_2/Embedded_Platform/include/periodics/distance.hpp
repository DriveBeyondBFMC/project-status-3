#ifndef DISTANCE_HPP
#define DISTANCE_HPP

// TODO: Add your code here

#include <mbed.h>
#include <drivers/vl53l1x.hpp>
#include <utils/task.hpp>
#include <brain/globalsv.hpp>
#include <chrono>


namespace periodics
{
   /**
    * @brief Class distance
    *
    */
    class CDistance: public utils::CTask
    {
        public:
            /* Constructor */
            CDistance(
                std::chrono::milliseconds f_period,
                UnbufferedSerial& f_serial,
                PinName SDA,
                PinName SCL
            );
            /* Destructor */
            ~CDistance();
            /* The API is used as SPI bus write */
            static s8 VL53L1X_I2C_bus_write(u8 dev_addr, u16 reg_addr, u8 reg_data, u8 cnt);

            static s8 VL53L1X_I2C_bus_write_byte(u8 dev_addr, u16 reg_addr, u8 reg_data);

            static s8 VL53L1X_I2C_bus_write_word(u8 dev_addr, u16 reg_addr, u16 reg_data);

            static s8 VL53L1X_I2C_bus_read(u8 dev_addr, u16 reg_addr, u8 *reg_data, u16 length);

            static s8 VL53L1X_I2C_bus_read_byte(u8 dev_addr, u16 reg_addr, u8 *reg_data);
            
            static s8 VL53L1X_I2C_bus_read_word(u8 dev_addr, u16 reg_addr, u16 *reg_data);

            static s8 VL53L1X_I2C_bus_read_double_word(u8 dev_addr, u16 reg_addr, u32 *reg_data);

            static void VL53L1X_delay_msek(u32 msek);
            /* Serial callback implementation */
            void serialCallbackDISTANCEcommand(char const * a, char * b);
        private:
            static I2C* i2c_instance;
            /* private variables & method member */
            virtual void I2C_routine(void);
            /* Run method */
            virtual void        _run();

            /** @brief Active flag  */
            bool m_isActive;
            /* @brief Serial communication obj.  */
            UnbufferedSerial&   m_serial;
            uint16_t m_id;
            uint64_t m_delta_time;
            struct vl53l1x_t vl53l1x;
            char buffer_1[100];

        protected:

    }; // class CDistance
}; // namespace periodics

#endif // DISTANCE_HPP
