use crate::reader::Error::{ChecksumMismatch, ReadError};
use crate::START;
use defmt::debug;
use embedded_io_async::BufRead;
use log::warn;

pub enum MessageType {
    ReadToken,
    WriteToken,
    Other,
}

pub struct ModbusMessage {
    pub length: usize,
    buf: [u8; 2048],
}

impl ModbusMessage {
    pub fn raw_frame(&self) -> &[u8] {
        &self.buf[..self.length]
    }
    pub fn address(&self) -> u16 {
        ((self.buf[1] as u16) << 8) | self.buf[2] as u16
    }
    pub fn message_type(&self) -> MessageType {
        let command_and_length = ((self.buf[3] as u16) << 8) | self.buf[4] as u16;
        debug!("command_and_length: {=u16:04x}", command_and_length);
        match command_and_length {
            0x6900 => MessageType::ReadToken,
            0x6b00 => MessageType::WriteToken,
            _ => MessageType::Other,
        }
    }
}

struct Buffer {
    data: [u8; 2048],
    pos: usize,
    len: Option<usize>,
}

impl Buffer {
    const fn new() -> Self {
        Self {
            data: [0; 2048],
            pos: 0,
            len: None,
        }
    }
}

pub enum Error<R>
where
    R: BufRead,
{
    ChecksumMismatch,
    ReadError(R::Error),
}

pub struct AsyncReader<R> {
    reader: R,
    buffer: Option<Buffer>,
}

impl<R> AsyncReader<R>
where
    R: BufRead,
{
    /// Construct a new AsyncReader from a byte reader.
    pub fn new(reader: R) -> Self {
        Self {
            reader,
            buffer: None,
        }
    }

    /// Read message from reader
    ///
    /// This function is cancel-safe.
    pub async fn next_message(&mut self) -> Result<Option<ModbusMessage>, Error<R>> {
        loop {
            match self.buffer {
                Some(ref mut buffer) => 'fill_buf: loop {
                    let buf = self.reader.fill_buf().await.map_err(|e| ReadError(e))?;
                    let n = buf.len();

                    if n == 0 {
                        return Ok(None);
                    }

                    for (i, &b) in buf.iter().enumerate() {
                        //debug!("{=u8:x}", b);
                        if buffer.pos >= buffer.data.len() {
                            self.reader.consume(i + 1);
                            self.buffer = None;
                            warn!("Buffer overflow");
                            break 'fill_buf;
                        }

                        buffer.data[buffer.pos] = b;

                        if buffer.pos == 4 {
                            buffer.len = Some(b as usize + 6); // 5 byte header + checksum
                        }

                        if buffer.len.is_some_and(|len| buffer.pos >= len - 1) {
                            self.reader.consume(i + 1);
                            debug!("Read message: {=[u8]:02x}", &buffer.data[..buffer.pos + 1]);
                            if !is_valid_checksum(&buffer.data[1..buffer.pos], b) {
                                self.buffer = None;
                                return Err(ChecksumMismatch);
                            }
                            let readout = ModbusMessage {
                                length: buffer.len.unwrap(),
                                buf: buffer.data,
                            };
                            self.buffer = None;
                            return Ok(Some(readout));
                        }

                        buffer.pos += 1;
                    }

                    self.reader.consume(n);
                },
                None => match scan_to_next(&mut self.reader)
                    .await
                    .map_err(|e| ReadError(e))?
                {
                    Some(buffer) => {
                        debug!("new buffer");
                        self.buffer = Some(buffer);
                    }
                    None => return Ok(None),
                },
            }
        }
    }
}

pub fn compute_checksum(data: &[u8]) -> u8 {
    data.iter().fold(0, |acc, b| acc ^ b)
}

fn is_valid_checksum(data: &[u8], message_checksum: u8) -> bool {
    let computed_checksum = compute_checksum(data);
    if computed_checksum == message_checksum {
        return true;
    }
    // if checksum is 0x5c (start character), heat pump seems to send 0xc5 checksum
    if computed_checksum == 0x5c && message_checksum == 0xc5 {
        return true;
    }
    debug!(
        "Computed checksum {=u8:x} != {=u8:x}",
        computed_checksum, message_checksum
    );
    false
}

async fn scan_to_next<R>(reader: &mut R) -> Result<Option<Buffer>, R::Error>
where
    R: BufRead,
{
    loop {
        let buf = reader.fill_buf().await?;
        let n = buf.len();
        if n == 0 {
            return Ok(None);
        }

        if let Some(start) = buf.iter().position(|b| *b == START) {
            reader.consume(start);
            return Ok(Some(Buffer::new()));
        } else {
            debug!("Skip {=[u8]:02x}", buf);
            reader.consume(n);
        }
    }
}
