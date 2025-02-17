#include "periodics/distance.hpp"
#include "distance.hpp"
// TODO: Add your code here
#define _100_chars                      100
namespace periodics
{
    /**
     * @brief Class constructor distance
     *
     */

    I2C *periodics::CDistance::i2c_instance = nullptr;

    CDistance::CDistance(
            std::chrono::milliseconds f_period,
            UnbufferedSerial &f_serial,
            PinName SDA,
            PinName SCL)
        : utils::CTask(f_period)
        , m_id()
        , m_delta_time(f_period.count())
        , m_isActive(false)
        , m_serial(f_serial)
    {
        if (m_delta_time < 150)
        {
            setNewPeriod(150);
            m_delta_time = 150;
        }
        s32 comres = VL53L1X_ERROR;
        i2c_instance = new I2C(SDA, SCL);
        i2c_instance->frequency(400000);
        ThisThread::sleep_for(chrono::milliseconds(300));
        
        I2C_routine();
        vl53l1x_init(&vl53l1x);

    }

    /** @brief  CDistance class destructor
     */
    CDistance::~CDistance()
    {
        if(i2c_instance != nullptr)
        {
            delete i2c_instance;
            i2c_instance = nullptr;
        }
    }

    void CDistance::serialCallbackDISTANCEcommand(char const * a, char * b) {
        uint8_t l_isActivate=0;
        uint8_t l_res = sscanf(a,"%hhu",&l_isActivate);

        if(1 == l_res){
            if(uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30)
            {
                m_isActive=(l_isActivate>=1);
                bool_globalsV_distance_isActive = (l_isActivate>=1);
                sprintf(b,"1");
            }
            else{
                sprintf(b,"kl 15/30 is required!!");
            }
            
        }else{
            sprintf(b,"syntax error");
        }
    }



    s8 CDistance::VL53L1X_I2C_bus_write(u8 dev_addr, u16 reg_addr, u8 reg_data, u8 cnt)
    {
        s32 VL53L1X_iERROR = VL53L1X_INIT_VALUE;
        u8 buffer[2];
        buffer[0] = reg_addr >> 8;
        buffer[1] = reg_addr & 0xFF;

        if (i2c_instance->write(dev_addr, (const char*)buffer, 2) != 0) {
            VL53L1X_iERROR = VL53L1X_ERROR; 
            return (s8)VL53L1X_iERROR;
        }

        u8 data_buffer[1];  // Tạo buffer trung gian
        for (u8 index = 0; index < cnt; index++) {
            data_buffer[0] = reg_data;  // Lưu reg_data vào buffer trước khi gửi
            if (i2c_instance->write(dev_addr, (const char*)data_buffer, 1) != 0) {
                VL53L1X_iERROR = VL53L1X_ERROR; 
                return (s8)VL53L1X_iERROR;
            }
        }
        return (s8)VL53L1X_iERROR;
    }


    s8 CDistance::VL53L1X_I2C_bus_write_byte(u8 dev_addr, u16 reg_addr, u8 reg_data)
    {
        s32 VL53L1X_iERROR = VL53L1X_INIT_VALUE;
        VL53L1X_iERROR = VL53L1X_I2C_bus_write(dev_addr, reg_addr, reg_data, 1);
        
        return (s8)VL53L1X_iERROR;
    }

    s8 CDistance::VL53L1X_I2C_bus_write_word(u8 dev_addr, u16 reg_addr, u16 reg_data)
    {
        s32 VL53L1X_iERROR = VL53L1X_INIT_VALUE;
        u8 buffer[2];
        buffer[0] = reg_data >> 8;
        buffer[1] = reg_data & 0x00FF;
        VL53L1X_iERROR = VL53L1X_I2C_bus_write(dev_addr, reg_addr, reg_data, 1);
        
        return (s8)VL53L1X_iERROR;
    }


    s8 CDistance::VL53L1X_I2C_bus_read(u8 dev_addr, u16 reg_addr, u8 *reg_data, u16 length) {
        s32 VL53L1X_iERROR = VL53L1X_INIT_VALUE;
        uint8_t regBuffer[2];

        regBuffer[0] = reg_addr >> 8;
        regBuffer[1] = reg_addr & 0xFF;

        if (i2c_instance->write(dev_addr, (const char*)regBuffer, 2) != 0) {
            VL53L1X_iERROR = VL53L1X_ERROR;
            return (s8)VL53L1X_ERROR;
        }
        if (i2c_instance->read(dev_addr, (char*)reg_data, length) != 0) {
            VL53L1X_iERROR = VL53L1X_ERROR;
            return (s8)VL53L1X_ERROR;
        }
        return VL53L1X_iERROR; 
    }

    s8 CDistance::VL53L1X_I2C_bus_read_byte(u8 dev_addr, u16 reg_addr, u8 *reg_data) {
        s32 VL53L1X_iERROR = VL53L1X_INIT_VALUE;
        uint8_t regBuffer[2];
        VL53L1X_iERROR = VL53L1X_I2C_bus_read(dev_addr, reg_addr, reg_data, 1);
        return VL53L1X_iERROR; 
    }

    s8 CDistance::VL53L1X_I2C_bus_read_word(u8 dev_addr, u16 reg_addr, u16 *reg_data) {
        s32 VL53L1X_iERROR = VL53L1X_INIT_VALUE;
        uint8_t buffer[2] = {0, 0};
        // Check for error correctly
        if (VL53L1X_I2C_bus_read(dev_addr, reg_addr, buffer, 2) == 0) {
            *reg_data = (buffer[0] << 8) | buffer[1];
            VL53L1X_iERROR = VL53L1X_SUCCESS;
        }
        return VL53L1X_iERROR; 
    }


     s8 CDistance::VL53L1X_I2C_bus_read_double_word(u8 dev_addr, u16 reg_addr, u32 *reg_data) {
        s32 VL53L1X_iERROR = VL53L1X_INIT_VALUE;
        uint8_t buffer[4] = {0, 0, 0, 0};
        // Check for error correctly
        if (VL53L1X_I2C_bus_read(dev_addr, reg_addr, buffer, 4) == 0) {
            *reg_data = (buffer[0] << 8) | buffer[1];
            VL53L1X_iERROR = VL53L1X_SUCCESS;
        }
        return VL53L1X_iERROR; 
    }



    void CDistance::VL53L1X_delay_msek(u32 msek)
    {
        /*Here you can write your own delay routine*/
        ThisThread::sleep_for(chrono::milliseconds(msek));
    }



    void CDistance::I2C_routine(void)
    {
        //Asigns function pointer for I2C write operation
        vl53l1x.bus_write = VL53L1X_I2C_bus_write;
        vl53l1x.bus_write_byte = VL53L1X_I2C_bus_write_byte;
        vl53l1x.bus_write_word = VL53L1X_I2C_bus_write_word;

        //Asigns function pointer for I2C read operations
        vl53l1x.bus_read = VL53L1X_I2C_bus_read;
        vl53l1x.bus_read_byte = VL53L1X_I2C_bus_read_byte;
        vl53l1x.bus_read_word = VL53L1X_I2C_bus_read_word;
        vl53l1x.bus_read_double_word = VL53L1X_I2C_bus_read_double_word;


        //Asign the delay function for timming operations
        vl53l1x.delay_msec = VL53L1X_delay_msek;

        //Set the I2C device address 
        vl53l1x.dev_addr = VL53L1X_I2C_ADDR << 1;

        ThisThread::sleep_for(chrono::milliseconds(300));
    }

    void CDistance::_run(){
        if(!m_isActive) return;

        char buffer[_100_chars];  
        u16 m_distance = VL53L1X_INIT_VALUE;
        u16 m_id = VL53L1X_INIT_VALUE;
        u8 value = VL53L1X_INIT_VALUE;

        vl53l1x_get_distance(&m_distance);
        vl53l1x_get_sensor_id(&m_id);
        vl53l1x_clear_interupt();
        vl53l1x_stop_ranging();

        snprintf(buffer, sizeof(buffer), "@distance: %d \n", m_distance);
        m_serial.write(buffer, strlen(buffer));
    }

};


    
