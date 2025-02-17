/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*/


#include <periodics/imu.hpp>
#include "imu.hpp"

#define _100_chars                      100
#define BNO055_EULER_DIV_DEG_int        16
#define BNO055_LINEAR_ACCEL_DIV_MSQ_int 100
#define precision_scaling_factor        1000

namespace periodics{
    /** \brief  Class constructor
     *
     *  It initializes the task and the state of the led. 
     *
     *  \param f_period       Toggling period of LED
     *  \param f_led          Digital output line to LED
     */

    /*--------------------------------------------------------------------------------------------------*
    *  Before initializiting with another value, the i2c_instance static pointer variable should be
    *  initialized with a nullptr.
    *---------------------------------------------------------------------------------------------------*/
    I2C* periodics::CImu::i2c_instance = nullptr;

    CImu::CImu(
            std::chrono::milliseconds    f_period, 
            UnbufferedSerial& f_serial,
            PinName SDA,
            PinName SCL)
        : utils::CTask(f_period)
        , m_isActive(false)
        , m_serial(f_serial)
        , m_velocityX(0)
        , m_velocityY(0)
        , m_velocityZ(0)
        , m_velocityStationaryCounter(0)
        , m_delta_time(f_period.count())
    {
        if(m_delta_time < 150){
            setNewPeriod(150);
            m_delta_time = 150;
        }
        
        s32 comres = BNO055_ERROR;
        /* variable used to set the power mode of the sensor*/
        u8 power_mode = BNO055_INIT_VALUE;

        /*---------------------------------------------------------------------------*
        *********************** START INITIALIZATION ************************
        *--------------------------------------------------------------------------*/

        /*--------------------------------------------------------------------------------------------------*
        *  i2c_instance variable member will be initialized with the actual I2C of the target board.
        *---------------------------------------------------------------------------------------------------*/      
        i2c_instance = new I2C(SDA, SCL);
        i2c_instance->frequency(400000);

        ThisThread::sleep_for(chrono::milliseconds(300));

        /*  Based on the user need configure I2C interface.
        *  It is example code to explain how to use the bno055 API*/
        I2C_routine();

        /*--------------------------------------------------------------------------*
        *  This API used to assign the value/reference of
        *  the following parameters
        *  I2C address
        *  Bus Write
        *  Bus read
        *  Chip id
        *  Page id
        *  Accel revision id
        *  Mag revision id
        *  Gyro revision id
        *  Boot loader revision id
        *  Software revision id
        *-------------------------------------------------------------------------*/
        comres = bno055_init(&bno055);

        /*  For initializing the BNO sensor it is required to the operation mode
        * of the sensor as NORMAL
        * Normal mode can set from the register
        * Page - page0
        * register - 0x3E
        * bit positions - 0 and 1*/
        power_mode = BNO055_POWER_MODE_NORMAL;

        /* set the power mode as NORMAL*/
        comres += bno055_set_power_mode(power_mode);

        /************************* START READ RAW DATA ********
        * operation modes of the sensor
        * operation mode can set from the register
        * page - page0
        * register - 0x3D
        * bit - 0 to 3
        * for sensor data read following operation mode have to set
        * SENSOR MODE
        * 0x01 - BNO055_OPERATION_MODE_ACCONLY
        * 0x02 - BNO055_OPERATION_MODE_MAGONLY
        * 0x03 - BNO055_OPERATION_MODE_GYRONLY
        * 0x04 - BNO055_OPERATION_MODE_ACCMAG
        * 0x05 - BNO055_OPERATION_MODE_ACCGYRO
        * 0x06 - BNO055_OPERATION_MODE_MAGGYRO
        * 0x07 - BNO055_OPERATION_MODE_AMG
        * based on the user need configure the operation mode*/

        /************************* START READ RAW FUSION DATA ********
         * For reading fusion data it is required to set the
         * operation modes of the sensor
         * operation mode can set from the register
         * page - page0
         * register - 0x3D
         * bit - 0 to 3
         * for sensor data read following operation mode have to set
         * FUSION MODE
         * 0x08 - BNO055_OPERATION_MODE_IMUPLUS
         * 0x09 - BNO055_OPERATION_MODE_COMPASS
         * 0x0A - BNO055_OPERATION_MODE_M4G
         * 0x0B - BNO055_OPERATION_MODE_NDOF_FMC_OFF
         * 0x0C - BNO055_OPERATION_MODE_NDOF
         * based on the user need configure the operation mode*/
        comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

        /*----------------------------------------------------------------*
        ************************* END INITIALIZATION *************************
        *-----------------------------------------------------------------*/
        u8 euler_unit_u8 = BNO055_INIT_VALUE;

        /* Read the current Euler unit and set the
        * unit as degree if the unit is in radians */
        comres = bno055_get_euler_unit(&euler_unit_u8);
        if (euler_unit_u8 != BNO055_EULER_UNIT_DEG)
        {
            comres += bno055_set_euler_unit(BNO055_EULER_UNIT_DEG);
        }

    }

    /** @brief  CImu class destructor
     */
    CImu::~CImu()
    {
        /*-----------------------------------------------------------------------*
        ************************* START DE-INITIALIZATION ***********************
        *-------------------------------------------------------------------------*/
        s32 comres = BNO055_ERROR;
        /* variable used to set the power mode of the sensor*/
        u8 power_mode = BNO055_INIT_VALUE;
        /*  For de - initializing the BNO sensor it is required
        * to the operation mode of the sensor as SUSPEND
        * Suspend mode can set from the register
        * Page - page0
        * register - 0x3E
        * bit positions - 0 and 1*/
        power_mode = BNO055_POWER_MODE_SUSPEND;

        /* set the power mode as SUSPEND*/
        comres += bno055_set_power_mode(power_mode);

        if(i2c_instance != nullptr)
        {
            delete i2c_instance;
            i2c_instance = nullptr;
        }
        /*---------------------------------------------------------------------*
        ************************* END DE-INITIALIZATION **********************
        *---------------------------------------------------------------------*/
    };

    /** \brief  Serial callback method to activate or deactivate the publisher. 
     * When the received integer value is bigger or equal to 1, then the publisher become 
     * active and send messages, otherwise is deactivated. 
     *
     * @param a                   input received string
     * @param b                   output reponse message
     * 
     */
    void CImu::serialCallbackIMUcommand(char const * a, char * b) {
        uint8_t l_isActivate=0;
        uint8_t l_res = sscanf(a,"%hhu",&l_isActivate);

        if(1 == l_res){
            if(uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30)
            {
                m_isActive=(l_isActivate>=1);
                bool_globalsV_imu_isActive = (l_isActivate>=1);
                sprintf(b,"1");
            }
            else{
                sprintf(b,"kl 15/30 is required!!");
            }
            
        }else{
            sprintf(b,"syntax error");
        }
    }



    /**
    * \brief Writes data to the device over the I2C bus.
    * 
    * This function serves as a low-level I2C write routine tailored for the BNO055 sensor. 
    * It packages the register address and data to be written into a single buffer and then 
    * dispatches the write operation.
    * 
    * \param dev_addr   : I2C address of the BNO055 sensor.
    * \param reg_addr   : Address of the target register where the data needs to be written.
    * \param reg_data   : Pointer to an array containing the data bytes to be written to the sensor.
    * \param cnt        : Number of data bytes to write from the \a reg_data array.
    * 
    * \return BNO055_SUCCESS (0) on successful write operation.
    * \return BNO055_ERROR (-1) if the write operation encounters an error.
    * 
    * \note The actual data writing starts from the second position in the array, as the 
    *       first position is reserved for the register address.
    */
    s8 CImu::BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
    {
        s32 BNO055_iERROR = BNO055_INIT_VALUE;
        u8 array[I2C_BUFFER_LEN];
        u8 stringpos = BNO055_INIT_VALUE;

        array[BNO055_INIT_VALUE] = reg_addr;
        for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
        {
            array[stringpos + BNO055_I2C_BUS_WRITE_ARRAY_INDEX] = *(reg_data + stringpos);
        }

        if (i2c_instance->write(dev_addr, (const char*)array, cnt + 1) == 0)
        {
            BNO055_iERROR = BNO055_SUCCESS; // Return success (0)
        }
        else
        {
            BNO055_iERROR = BNO055_ERROR; // Return error (-1)
        }

        return (s8)BNO055_iERROR;
    }

    /**
    * \brief Reads data from the device over the I2C bus.
    * 
    * This function facilitates reading data from a specific register of the BNO055 sensor 
    * over the I2C communication protocol. It sends the desired register address to the sensor, 
    * then reads back the requested amount of data bytes into a provided buffer.
    * 
    * \param dev_addr   : I2C address of the BNO055 sensor.
    * \param reg_addr   : Address of the target register from which the data needs to be read.
    * \param reg_data   : Pointer to an array where the read data bytes will be stored.
    * \param cnt        : Number of data bytes to read into the \a reg_data array.
    * 
    * \return BNO055_SUCCESS (0) on successful read operation.
    * \return BNO055_ERROR (-1) if the read operation encounters an error.
    * 
    * \note The function first writes the register address to the sensor to set the pointer 
    *       to the desired location and then initiates the I2C read operation.
    */
    s8 CImu::BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
    {
        s32 BNO055_iERROR = BNO055_INIT_VALUE;
        u8 array[I2C_BUFFER_LEN] = { BNO055_INIT_VALUE };
        u8 stringpos = BNO055_INIT_VALUE;

        array[BNO055_INIT_VALUE] = reg_addr;

        for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
        {
            *(reg_data + stringpos) = array[stringpos];
        }

        // Write the register address to set the pointer for reading
        if (i2c_instance->write(dev_addr, (const char*)&reg_addr, 1) != 0)
        {
            BNO055_iERROR = BNO055_ERROR; // Return error (-1)
            return (s8)BNO055_iERROR;
        }

        // Read the data from the specified register address
        if (i2c_instance->read(dev_addr, (char*)reg_data, cnt) == 0)
        {
            BNO055_iERROR = BNO055_SUCCESS; // Return success (0)
        }
        else
        {
            BNO055_iERROR = BNO055_ERROR; // Return error (-1)
        }
        return (s8)BNO055_iERROR;
    }
    
    /*-------------------------------------------------------------------------*
     *  By using bno055 the following structure parameter can be accessed
     *  Bus write function pointer: BNO055_WR_FUNC_PTR
     *  Bus read function pointer: BNO055_RD_FUNC_PTR
     *  Delay function pointer: delay_msec
     *  I2C address: dev_addr
     *--------------------------------------------------------------------------*/
    void CImu::I2C_routine(void)
    {
        bno055.bus_write = BNO055_I2C_bus_write;
        bno055.bus_read = BNO055_I2C_bus_read;
        bno055.delay_msec = BNO055_delay_msek;
        bno055.dev_addr = BNO055_I2C_ADDR1 << 1;
        // bno055.dev_addr = BNO055_I2C_ADDR1 << 1;

        ThisThread::sleep_for(chrono::milliseconds(300));
    }

    /**
    * \brief Introduces a delay for the specified duration in milliseconds.
    * 
    * This function relies on the `ThisThread::sleep_for` method to create 
    * a delay, making the current thread sleep for the specified duration.
    * 
    * \param msek The delay duration in milliseconds.
    */
    void CImu::BNO055_delay_msek(u32 msek)
    {
        /*Here you can write your own delay routine*/
        ThisThread::sleep_for(chrono::milliseconds(msek));
    }

    /** 
    * \brief  Periodically retrieves and processes IMU sensor values.
    * 
    * This method is invoked periodically and handles:
    * 1. Reading Euler angles (roll, pitch, yaw) from the BNO055 sensor.
    * 2. Reading linear acceleration values in the x, y, and z axes from the sensor.
    * 3. Based on the linear acceleration, it updates the current velocity of the device.
    * 4. If the device appears to be stationary (based on x and y acceleration thresholds), 
    *    a counter is incremented. If the device remains stationary for a certain duration 
    *    (15 cycles in this case), the velocity is reset.
    * 5. Formats and sends the acquired data over a serial connection.
    * 
    * \note If there are any issues reading from the BNO055 sensor, the method will exit early without sending data.
    */
    void CImu::_run()
    {
        if(!m_isActive) return;
        
        char buffer[_100_chars];
        s8 comres = BNO055_SUCCESS;

        s16 s16_euler_h_raw = BNO055_INIT_VALUE;
        s16 s16_euler_p_raw = BNO055_INIT_VALUE;
        s16 s16_euler_r_raw = BNO055_INIT_VALUE;

        s16 s16_linear_accel_x_raw = BNO055_INIT_VALUE;
        s16 s16_linear_accel_y_raw = BNO055_INIT_VALUE;
        s16 s16_linear_accel_z_raw = BNO055_INIT_VALUE;

        comres += bno055_read_euler_h(&s16_euler_h_raw);

        if(comres != BNO055_SUCCESS) return;

        comres += bno055_read_euler_p(&s16_euler_p_raw);

        if(comres != BNO055_SUCCESS) return;

        comres += bno055_read_euler_r(&s16_euler_r_raw);

        if(comres != BNO055_SUCCESS) return;

        s16 s16_euler_h_deg = (s16_euler_h_raw * precision_scaling_factor) / BNO055_EULER_DIV_DEG_int;
        s16 s16_euler_p_deg = (s16_euler_p_raw * precision_scaling_factor) / BNO055_EULER_DIV_DEG_int;
        s16 s16_euler_r_deg = (s16_euler_r_raw * precision_scaling_factor) / BNO055_EULER_DIV_DEG_int;

        comres = bno055_read_linear_accel_x(&s16_linear_accel_x_raw);

        if(comres != BNO055_SUCCESS) return;

        comres = bno055_read_linear_accel_y(&s16_linear_accel_y_raw);

        if(comres != BNO055_SUCCESS) return;

        comres = bno055_read_linear_accel_z(&s16_linear_accel_z_raw);

        if(comres != BNO055_SUCCESS) return;

        s32 s16_linear_accel_x_msq = (s16_linear_accel_x_raw * precision_scaling_factor) / BNO055_LINEAR_ACCEL_DIV_MSQ_int;
        s32 s16_linear_accel_y_msq = (s16_linear_accel_y_raw * precision_scaling_factor) / BNO055_LINEAR_ACCEL_DIV_MSQ_int;
        s32 s16_linear_accel_z_msq = (s16_linear_accel_z_raw * precision_scaling_factor) / BNO055_LINEAR_ACCEL_DIV_MSQ_int;

        if((-110 <= s16_linear_accel_x_msq && s16_linear_accel_x_msq <= 110) && (-110 <= s16_linear_accel_y_msq && s16_linear_accel_y_msq <= 110))
        {
            m_velocityX += 0 * m_delta_time; // Δt = m_delta_time
            m_velocityY += 0 * m_delta_time;
            m_velocityZ += 0 * m_delta_time;
            m_velocityStationaryCounter += 1;
            if (m_velocityStationaryCounter == 10)
            {
                m_velocityX = 0;
                m_velocityY = 0;
                m_velocityZ = 0;
                m_velocityStationaryCounter = 0;
            }
            
        }
        else{
            m_velocityX += (s16_linear_accel_x_msq * (uint16_t)m_delta_time) / 1000; // Δt = m_delta_time
            m_velocityY += (s16_linear_accel_y_msq * (uint16_t)m_delta_time) / 1000;
            m_velocityZ += (s16_linear_accel_z_msq * (uint16_t)m_delta_time) / 1000;
            m_velocityStationaryCounter = 0;
        }

        if(comres != BNO055_SUCCESS) return;

        snprintf(buffer, sizeof(buffer), "@imu:%d.%03d;%d.%03d;%d.%03d;%d.%03d;%d.%03d;%d.%03d;;\r\n",
            s16_euler_r_deg/1000, abs(s16_euler_r_deg%1000),
            s16_euler_p_deg/1000, abs(s16_euler_p_deg%1000),
            s16_euler_h_deg/1000, abs(s16_euler_h_deg%1000),
            m_velocityX/1000, abs(m_velocityX%1000),
            m_velocityY/1000, abs(m_velocityY%1000),
            m_velocityZ/1000, abs(m_velocityZ%1000));
        m_serial.write(buffer,strlen(buffer));
    }

}; // namespace periodics