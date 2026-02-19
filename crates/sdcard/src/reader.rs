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

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;

    fn collect_lines(data: &[u8]) -> std::vec::Vec<std::string::String> {
        let mut reader = LineReader::new();
        let mut lines = std::vec::Vec::new();
        reader.feed(data, |line| lines.push(std::string::String::from(line)));
        reader.flush(|line| lines.push(std::string::String::from(line)));
        lines
    }

    #[test]
    fn test_single_line() {
        let lines = collect_lines(b"G1 X10\n");
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0], "G1 X10");
    }

    #[test]
    fn test_multiple_lines() {
        let lines = collect_lines(b"G1 X10\nG1 Y20\nG1 Z5\n");
        assert_eq!(lines.len(), 3);
        assert_eq!(lines[0], "G1 X10");
        assert_eq!(lines[1], "G1 Y20");
        assert_eq!(lines[2], "G1 Z5");
    }

    #[test]
    fn test_crlf() {
        let lines = collect_lines(b"G1 X10\r\nG1 Y20\r\n");
        assert_eq!(lines.len(), 2);
        assert_eq!(lines[0], "G1 X10");
        assert_eq!(lines[1], "G1 Y20");
    }

    #[test]
    fn test_partial_line_flushed() {
        let lines = collect_lines(b"G1 X10");
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0], "G1 X10");
    }

    #[test]
    fn test_chunked_input() {
        let mut reader = LineReader::new();
        let mut lines = std::vec::Vec::new();
        reader.feed(b"G1 X", |line| lines.push(std::string::String::from(line)));
        reader.feed(b"10\nG1 Y20\n", |line| {
            lines.push(std::string::String::from(line))
        });
        assert_eq!(lines.len(), 2);
        assert_eq!(lines[0], "G1 X10");
        assert_eq!(lines[1], "G1 Y20");
    }

    #[test]
    fn test_empty_lines_skipped() {
        let lines = collect_lines(b"\n\nG1 X10\n\n");
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0], "G1 X10");
    }

    #[test]
    fn test_reset() {
        let mut reader = LineReader::new();
        let mut lines = std::vec::Vec::new();
        reader.feed(b"G1 X", |_| {});
        reader.reset();
        reader.feed(b"G1 Y20\n", |line| {
            lines.push(std::string::String::from(line))
        });
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0], "G1 Y20");
    }
}
