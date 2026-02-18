//! Line-by-line G-code reader for `no_std` environments.
//!
//! Accumulates bytes from chunked SD card reads and yields
//! complete lines one at a time via a callback.

/// Maximum G-code line length (RepRapFirmware uses 256, most lines < 96 chars).
const MAX_LINE_LEN: usize = 256;

/// Incremental line reader that works with chunked byte input.
pub struct LineReader {
    line_buf: [u8; MAX_LINE_LEN],
    line_len: usize,
}

impl Default for LineReader {
    fn default() -> Self {
        Self::new()
    }
}

impl LineReader {
    pub const fn new() -> Self {
        Self {
            line_buf: [0u8; MAX_LINE_LEN],
            line_len: 0,
        }
    }

    /// Feed a chunk of bytes (from an SD card read) into the reader.
    /// Calls `on_line` for each complete line found.
    pub fn feed<F>(&mut self, data: &[u8], mut on_line: F)
    where
        F: FnMut(&str),
    {
        for &byte in data {
            match byte {
                b'\n' => {
                    // Complete line — deliver it
                    if self.line_len > 0 {
                        if let Ok(line) = core::str::from_utf8(&self.line_buf[..self.line_len]) {
                            on_line(line);
                        }
                    }
                    self.line_len = 0;
                }
                b'\r' => {
                    // Skip carriage return (handle both \r\n and \r)
                }
                _ => {
                    // Accumulate byte
                    if self.line_len < MAX_LINE_LEN {
                        self.line_buf[self.line_len] = byte;
                        self.line_len += 1;
                    }
                    // Silently truncate lines longer than MAX_LINE_LEN
                }
            }
        }
    }

    /// Flush any remaining partial line (call after EOF).
    pub fn flush<F>(&mut self, mut on_line: F)
    where
        F: FnMut(&str),
    {
        if self.line_len > 0 {
            if let Ok(line) = core::str::from_utf8(&self.line_buf[..self.line_len]) {
                on_line(line);
            }
            self.line_len = 0;
        }
    }

    /// Reset the reader state.
    pub fn reset(&mut self) {
        self.line_len = 0;
    }
}
