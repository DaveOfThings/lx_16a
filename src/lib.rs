use std::{io::{Error, ErrorKind, Read, Write}, sync::Mutex};
use byteorder::{ByteOrder, LittleEndian};

// TODO-DW : Add support for direction control?
pub struct Lx16aBus<T: Read+Write> {
    port: Mutex<T>,
}

impl<'a, T: Read+Write> Lx16aBus<T> {
    // Create an Lx16a bus from anything that implements Read+Write
    pub fn new(port: T) -> Lx16aBus<T> {
        Lx16aBus { port: Mutex::new(port) }
    }

    // Get the "broadcast" servo
    pub fn broadcast(&'a self) -> Lx16a<'a, T> {
        const BROADCAST_ID: u8 = 254;

        Lx16a::new(BROADCAST_ID, self)
    }

    // Get an individual servo
    pub fn servo(&'a self, id: u8) -> Lx16a<'a, T> {
        Lx16a::new(id, self)
    }

    // Write data
    pub fn write(&self, out_data: &[u8])  -> Result<(), Error> {
        // Get exclusive access to the bus
        let mut port = self.port.lock().unwrap();

        // write the request
        port.write(out_data)?;
        port.flush()?;

        Ok(())
    }

    // Write data then read expected response.
    pub fn write_read(&self, out_data: &[u8], rx_data: &mut[u8]) -> Result<usize, Error> {
        // Get exclusive access to the bus
        let mut port = self.port.lock().unwrap();

        // write the request
        port.write(out_data)?;
        port.flush()?;

        // read the response
        port.read_exact(rx_data)?;

        Ok(rx_data.len())
    }
}

pub enum Lx16aMode {
    Servo,       // Servo (position control) mode
    Speed(i16),  // Speed mode with speed parameter
}

// Implement interface to one LX-16a servo on a bus.
pub struct Lx16a<'a, T: Read+Write> {
    id: u8, 
    bus: &'a Lx16aBus<T>,
}

impl<'a, T: Read+Write> Lx16a<'a, T> {
    // Private new method, used by Lx16aBus.
    // To create a servo, first create an Lx16aBus, then use servo() factory method.
    fn new(id: u8, bus: &'a Lx16aBus<T>) -> Lx16a<'a, T> {
        Lx16a { id, bus }
    }

    // Get the ID associated with this servo
    pub fn get_id(&self) -> u8 {
        self.id
    }

    // --- Public operations corresponding to servo commands

    pub fn move_time(&self, pos: u16, time_ms: u16) -> Result<(), Error> {
        const SERVO_MOVE_TIME_WRITE: u8 = 1;

        let mut params = [0; 4];
        LittleEndian::write_u16(&mut params[0..2], pos);
        LittleEndian::write_u16(&mut params[2..4], time_ms);
        self.write(self.id, SERVO_MOVE_TIME_WRITE, &params)?;

        Ok(())
    }

    pub fn read_move_time(&self) -> Result<(u16, u16), Error> {
        const SERVO_MOVE_TIME_READ: u8 = 2;

        let mut rx_buf = [0; 32];
        let params = [];
        let response = self.read(self.id, SERVO_MOVE_TIME_READ, &params, &mut rx_buf, 4)?;

        let pos = LittleEndian::read_u16(&response[0..2]);
        let time_ms = LittleEndian::read_u16(&response[2..4]);
        Ok((pos, time_ms))
    }
    
    pub fn move_wait(&self, pos: u16, time_ms: u16) -> Result<(), Error> {
        const SERVO_MOVE_TIME_WAIT_WRITE: u8 = 7;

        let mut params = [0; 4];
        LittleEndian::write_u16(&mut params[0..2], pos);
        LittleEndian::write_u16(&mut params[2..4], time_ms);
        self.write(self.id, SERVO_MOVE_TIME_WAIT_WRITE, &params)?;

        Ok(())
    }

    // read_move_wait
    pub fn read_move_wait(&self) -> Result<(u16, u16), Error> {
        const SERVO_MOVE_TIME_WAIT_READ: u8 = 8;

        let mut rx_buf = [0; 32];
        let params = [];
        let response = self.read(self.id, SERVO_MOVE_TIME_WAIT_READ, &params, &mut rx_buf, 4)?;

        let pos = LittleEndian::read_u16(&response[0..2]);
        let time_ms = LittleEndian::read_u16(&response[2..4]);
        Ok((pos, time_ms))
    }

    pub fn move_start(&self) -> Result<(), Error> {
        const SERVO_MOVE_START: u8 = 11;

        let params = [];
        self.write(self.id, SERVO_MOVE_START, &params)?;

        Ok(())
    }

    pub fn move_stop(&self) -> Result<(), Error> {
        const SERVO_MOVE_STOP: u8 = 12;

        let params = [];
        self.write(self.id, SERVO_MOVE_STOP, &params)?;

        Ok(())
    }

    pub fn set_servo_id(&self, id: u8) -> Result<(), Error> {
        const SERVO_ID_WRITE: u8 = 13;

        let params = [id];
        self.write(self.id, SERVO_ID_WRITE, &params)?;

        Ok(())
    }

    pub fn read_servo_id(&self) -> Result<u8, Error> {
        const SERVO_ID_READ: u8 = 14;

        let mut rx_buf = [0; 32];
        let params = [];
        let response = self.read(self.id, SERVO_ID_READ, &params, &mut rx_buf, 1)?;

        Ok(response[0])
    }

    pub fn set_angle_offset(&self, offset: i8) -> Result<(), Error> {
        const SERVO_ANGLE_OFFSET_ADJUST: u8 = 17;

        if (offset < -125) || (offset > 125) {
            return Err(Error::new(ErrorKind::InvalidData, "Offset out of range -125 to 125"))
        }

        let params = [offset as u8];
        self.write(self.id, SERVO_ANGLE_OFFSET_ADJUST, &params)?;

        Ok(())
    }

    pub fn save_angle_offset(&self) -> Result<(), Error> {
        const SERVO_ANGLE_OFFSET_WRITE: u8 = 18;

        let params = [];
        self.write(self.id, SERVO_ANGLE_OFFSET_WRITE, &params)?;

        Ok(())
    }

    pub fn read_angle_offset(&self) -> Result<u8, Error> {
        const SERVO_ANGLE_OFFSET_READ: u8 = 19;

        let mut rx_buf = [0; 32];
        let params = [];
        let response = self.read(self.id, SERVO_ANGLE_OFFSET_READ, &params, &mut rx_buf, 1)?;

        Ok(response[0])
    }

    pub fn set_angle_limit(&self, minimum: u16, maximum: u16) -> Result<(), Error> {
        const SERVO_ANGLE_LIMIT_WRITE: u8 = 20;

        let mut params = [0_u8; 4];
        LittleEndian::write_u16(&mut params[0..2], minimum);
        LittleEndian::write_u16(&mut params[2..4], maximum);
        self.write(self.id, SERVO_ANGLE_LIMIT_WRITE, &params)?;

        Ok(())
    }

    pub fn read_angle_limit(&self) -> Result<(u16, u16), Error> {
        const SERVO_ANGLE_LIMIT_READ: u8 = 21;

        let mut rx_buf = [0; 32];
        let params = [];
        let response = self.read(self.id, SERVO_ANGLE_LIMIT_READ, &params, &mut rx_buf, 4)?;
        let minimum = LittleEndian::read_u16(&response[0..2]);
        let maximum = LittleEndian::read_u16(&response[2..4]);

        Ok((minimum, maximum))
    }

    pub fn set_vin_limit_mv(&self, minimum: u16, maximum: u16) -> Result<(), Error> {
        const SERVO_VIN_LIMIT_WRITE: u8 = 22;

        if (minimum < 4500) || (minimum > 12000) {
            return Err(Error::new(ErrorKind::InvalidInput, "minimum must be in range 4500-12000 mV"));
        }
        if (maximum < 4500) || (maximum > 12000) {
            return Err(Error::new(ErrorKind::InvalidInput, "maximum must be in range 4500-12000 mV"));
        }
        if minimum >= maximum {
            return Err(Error::new(ErrorKind::InvalidInput, "minimum must be less than maximum"));
        }

        let mut params = [0_u8; 4];
        LittleEndian::write_u16(&mut params[0..2], minimum);
        LittleEndian::write_u16(&mut params[2..4], maximum);
        self.write(self.id, SERVO_VIN_LIMIT_WRITE, &params)?;

        Ok(())
    }

    pub fn read_vin_limit_mv(&self) -> Result<(u16, u16), Error> {
        const SERVO_VIN_LIMIT_READ: u8 = 23;

        let mut rx_buf = [0; 32];
        let params = [];
        let response = self.read(self.id, SERVO_VIN_LIMIT_READ, &params, &mut rx_buf, 4)?;
        let minimum = LittleEndian::read_u16(&response[0..2]);
        let maximum = LittleEndian::read_u16(&response[2..4]);

        Ok((minimum, maximum))
    }

    pub fn set_temp_limit_c(&self, limit_c: u8) -> Result<(), Error> {
        const SERVO_TEMP_MAX_LIMIT_WRITE: u8 = 24;

        if (limit_c < 50) || (limit_c > 85) {
            return Err(Error::new(ErrorKind::InvalidInput, "temp limit must be in range 50-85 mV"));
        }

        let params = [limit_c];
        self.write(self.id, SERVO_TEMP_MAX_LIMIT_WRITE, &params)?;

        Ok(())
    }

    pub fn read_temp_limit_c(&self) -> Result<u8, Error> {
        const SERVO_TEMP_MAX_LIMIT_READ: u8 = 25;

        let mut rx_buf = [0; 32];
        let params = [];
        let response = self.read(self.id, SERVO_TEMP_MAX_LIMIT_READ, &params, &mut rx_buf, 1)?;
        let limit = response[0];

        Ok(limit)
    }

    pub fn read_temp_c(&self) -> Result<i8, Error> {
        const SERVO_TEMP_READ: u8 = 26;

        let mut rx_buf = [0; 32];
        let params = [];
        let response = self.read(self.id, SERVO_TEMP_READ, &params, &mut rx_buf, 1)?;

        Ok(response[0] as i8)
    }

    pub fn read_vin_mv(&self) -> Result<u16, Error> {
        const SERVO_VIN_READ: u8 = 27;

        let mut rx_buf = [0; 32];
        let params = [];
        let response = self.read(self.id, SERVO_VIN_READ, &params, &mut rx_buf, 2)?;

        Ok(LittleEndian::read_u16(&response[0..2]))
    }

    pub fn read_pos(&self) -> Result<u16, Error> {
        const SERVO_POS_READ: u8 = 28;

        let mut rx_buf = [0; 32];
        let params = [];
        let response = self.read(self.id, SERVO_POS_READ, &params, &mut rx_buf, 2)?;

        Ok(LittleEndian::read_u16(&response[0..2]))
    }

    pub fn set_mode(&self, mode: Lx16aMode) -> Result<(), Error> {
        const SERVO_OR_MOTOR_MODE_WRITE: u8 = 29;

        let mut params = [0_u8; 4];
        if let Lx16aMode::Speed(speed) = mode {
            params[0] = 1;  // motor control mode
            LittleEndian::write_i16(&mut params[2..4], speed);
        }
        else {
            // For Servo mode, params should all be 0, which they already are.
        }

        println!("Writing mode as {:02x} {:02x} {:02x} {:02x}", params[0], params[1], params[2], params[3]);
        self.write(self.id, SERVO_OR_MOTOR_MODE_WRITE, &params)?;

        Ok(())
    }

    pub fn read_mode(&self) -> Result<Lx16aMode, Error> {
        const SERVO_OR_MOTOR_MODE_READ: u8 = 30;

        let mut rx_buf = [0; 32];
        let params = [];
        let response = self.read(self.id, SERVO_OR_MOTOR_MODE_READ, &params, &mut rx_buf, 4)?;

        match response[0] {
            0 => Ok(Lx16aMode::Servo),
            1 => {
                let speed = LittleEndian::read_i16(&response[2..4]);
                Ok(Lx16aMode::Speed(speed))
            }
            _ => {
                Err(Error::new(std::io::ErrorKind::InvalidData, format!("Invalid mode, {}", response[0])))
            }
        }
    }

    pub fn set_powered(&self, powered: bool) -> Result<(), Error> {
        const SERVO_LOAD_OR_UNLOAD_WRITE: u8 = 31;

        let powered_u8: u8 = match powered { 
            false => 0,
            true => 1,
        };

        let params = [powered_u8];
        self.write(self.id, SERVO_LOAD_OR_UNLOAD_WRITE, &params)?;

        Ok(())
    }

    pub fn read_powered(&self) -> Result<bool, Error> {
        const SERVO_LOAD_OR_UNLOAD_READ: u8 = 32;

        let mut rx_buf = [0; 32];
        let params = [];
        let response = self.read(self.id, SERVO_LOAD_OR_UNLOAD_READ, &params, &mut rx_buf, 1)?;

        match response[0] {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(Error::new(std::io::ErrorKind::InvalidData, format!("Invalid powered state {}", response[0])))
        }
    }

    // Note: The LX-16A protocol uses 0 to represent LED ON and 1 to represent OFF.
    // But this API uses true for on, false for off.
    pub fn set_led(&self, on: bool) -> Result<(), Error> {
        const SERVO_LED_CTRL_WRITE: u8 = 33;

        let led_state: u8 = match on { 
            false => 1,
            true => 0,
        };

        let params = [led_state];
        self.write(self.id, SERVO_LED_CTRL_WRITE, &params)?;

        Ok(())
    }

    pub fn read_led(&self) -> Result<bool, Error> {
        const SERVO_LED_CTRL_READ: u8 = 34;

        let mut rx_buf = [0; 32];
        let params = [];
        let response = self.read(self.id, SERVO_LED_CTRL_READ, &params, &mut rx_buf, 1)?;

        match response[0] {
            0 => Ok(true),   // 0 is on
            1 => Ok(true),   // 1 is off
            _ => Err(Error::new(std::io::ErrorKind::InvalidData, format!("Invalid led state {}", response[0])))
        }
    }

    pub fn set_led_err(&self, over_temp: bool, over_voltage: bool, locked_rotor: bool) -> Result<(), Error> {
        const SERVO_LED_ERROR_WRITE: u8 = 35;

        let mut led_err_state: u8 = 0;
        if over_temp { 
            led_err_state |= 1; 
        }
        if over_voltage { 
            led_err_state |= 2; 
        }
        if locked_rotor { 
            led_err_state |= 4; 
        }

        let params = [led_err_state];
        self.write(self.id, SERVO_LED_ERROR_WRITE, &params)?;

        Ok(())
    }

    pub fn read_led_err(&self) -> Result<(bool, bool, bool), Error> {
        const SERVO_LED_ERROR_READ: u8 = 36;

        let mut rx_buf = [0; 32];
        let params = [];
        let response = self.read(self.id, SERVO_LED_ERROR_READ, &params, &mut rx_buf, 1)?;

        let over_temp = (response[0] & 1) == 1;
        let over_voltage = (response[0] & 2) == 2;
        let locked_rotor = (response[0] & 4) == 4;

        Ok((over_temp, over_voltage, locked_rotor))
    }

    // --- Utility methods --------------------------------------------
    
    fn checksum(msg: &[u8]) -> u8 {
        // Add the payload bytes together, mod 256, and negate.
        let sum = msg.iter()
            .fold(0_u8, |acc, &x| { 
                acc.wrapping_add(x)
            }) ^ 0xFF;

        // Stuff the result into the last byte of the message.
        sum
    }

    fn form_packet(tx_data: &mut[u8], id: u8, cmd: u8, params: &[u8]) -> usize {
        let mut len;
        let len_field = 3_u8 + params.len() as u8;

        // Format contents of packet in tx_data
        tx_data[0] = 0x55;
        tx_data[1] = 0x55;
        tx_data[2] = id;
        tx_data[3] = len_field;
        tx_data[4] = cmd;
        len = 5;
        params.iter()
            .for_each(|b| { 
                tx_data[len] = *b; 
                len += 1; 
            });
        tx_data[len] = Self::checksum(&tx_data[2..len]);
        len += 1;

        len
    }

    fn write(&self, id: u8, cmd: u8, params: &[u8]) -> Result<(), Error> {
        let mut tx_data: [u8; 32] = [0; 32];
        let tx_data_len = Self::form_packet(&mut tx_data, id, cmd, params);

        self.bus.write(&tx_data[0..tx_data_len])?;

        Ok(())
    }

    fn read(&self, id: u8, cmd: u8, params: &[u8], rx_buf: &'a mut[u8], params_len: usize) -> Result<&'a [u8], Error> {
        let mut tx_data: [u8; 32] = [0; 32];
        let tx_data_len = Self::form_packet(&mut tx_data, id, cmd, params);
        let resp_len = params_len + 6;

        self.bus.write_read(&tx_data[0..tx_data_len], &mut rx_buf[0..resp_len])?;

        // Validate response: header, id, length, cmd, checksum
        if Self::checksum(&rx_buf[2..resp_len]) != 0_u8 {
            Err(Error::new(ErrorKind::InvalidData, "Checksum error"))
        }
        else if (rx_buf[0] != 0x55) ||
           (rx_buf[1] != 0x55) ||
           (rx_buf[2] != id) ||
           (rx_buf[3] != 3_u8 + params_len as u8) ||
           (rx_buf[4] != cmd) {
            Err(Error::new(ErrorKind::InvalidData, "Bad response"))
        }
        else {
            Ok(&rx_buf[5..5+params_len])
        }
    }
}

// TODO: Real tests

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = 2+2;
        assert_eq!(result, 4);
    }
}
