use defmt::debug;
use embedded_io_async::BufRead;

use crate::reader::Error::{ChecksumMismatch, ReadError};

pub struct Message {
    pub length: usize,
    pub address: u16,
    buf: [u8; 2048],
}

impl Message {
    pub fn raw_frame(&self) -> &[u8] {
        &self.buf[..self.length]
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

pub enum Error<R> where R: BufRead {
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
    pub async fn next_message(&mut self) -> Result<Option<Message>, Error<R>> {
        loop {
            match self.buffer {
                Some(ref mut buffer) => 'fill_buf: loop {
                    let buf = self.reader.fill_buf().await
                        .map_err(|e| ReadError(e))?;
                    let n = buf.len();

                    if n == 0 {
                        return Ok(None);
                    }

                    for (i, &b) in buf.iter().enumerate() {
                        if buffer.pos >= buffer.data.len() {
                            self.reader.consume(i);
                            self.buffer = None;
                            break 'fill_buf; // buffer overflow
                        }

                        buffer.data[buffer.pos] = b;

                        if buffer.pos == 4 {
                            buffer.len = Some(b as usize + 6); // 5 byte header + checksum
                        }

                        if buffer.len.is_some_and(|len| buffer.pos >= len) {
                            self.reader.consume(i);
                            let checksum = compute_checksum(&buffer.data[1..buffer.pos]);
                            debug!("Read message: {=[?]}", &buffer.data[..buffer.pos+1]);
                            if checksum != b {
                                debug!("Computed checksum {} != {}", checksum, b);
                                return Err(ChecksumMismatch);
                            }
                            let readout = Message {
                                length: buffer.len.unwrap(),
                                address: ((buffer.data[1] as u16) << 8) | buffer.data[2] as u16,
                                buf: buffer.data,
                            };
                            self.buffer = None;
                            return Ok(Some(readout));
                        }

                        buffer.pos += 1;
                    }

                    self.reader.consume(n);
                },
                None => match scan_to_next(&mut self.reader).await
                    .map_err(|e| ReadError(e))? {
                    Some(buffer) => {
                        self.buffer = Some(buffer);
                    }
                    None => return Ok(None),
                },
            }
        }
    }
}

fn compute_checksum(data: &[u8]) -> u8 {
    data.iter().fold(0, |acc, b| acc ^ b)
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

        if let Some(start) = buf.iter().position(|b| *b == 0x05) {
            reader.consume(start);
            return Ok(Some(Buffer::new()));
        } else {
            reader.consume(n);
        }
    }
}