//! Portable configuration loader.
//!
//! Reads CONFIG.G (and optionally CONFIGO.G) from any `FileSystem` impl.
//! Falls back to built-in defaults when no file is found.
//! This lets a single binary per board work out of the box, while still
//! allowing users to customize via config files on storage.

use crate::reader::LineReader;
use printer_hal::FileSystem;

/// Read buffer size for chunked file reads.
const READ_BUF_SIZE: usize = 128;

/// Result of a config load operation.
#[derive(Clone, Debug, defmt::Format)]
pub struct ConfigResult {
    /// Number of G-code commands executed.
    pub commands_executed: u32,
    /// Whether the config came from a file (true) or built-in defaults (false).
    pub from_file: bool,
}

/// Load configuration by reading a G-code file from the filesystem.
///
/// Opens `path` (e.g. "CONFIG.G"), reads it line by line, and calls
/// `on_line` for each G-code command found. Returns the number of
/// commands executed.
///
/// Returns `None` if the file does not exist or cannot be opened.
pub fn load_config_file<F, L>(fs: &mut F, path: &str, on_line: &mut L) -> Option<u32>
where
    F: FileSystem,
    L: FnMut(&str),
{
    if !fs.open(path) {
        return None;
    }

    let mut reader = LineReader::new();
    let mut count: u32 = 0;

    loop {
        let mut buf = [0u8; READ_BUF_SIZE];
        let n = fs.read(&mut buf);
        if n == 0 {
            break; // EOF
        }

        reader.feed(&buf[..n], |line| {
            let trimmed = line.trim();
            // Skip empty lines and comments
            if !trimmed.is_empty() && !trimmed.starts_with(';') {
                on_line(trimmed);
                count += 1;
            }
        });
    }

    // Flush any partial line at EOF
    reader.flush(|line| {
        let trimmed = line.trim();
        if !trimmed.is_empty() && !trimmed.starts_with(';') {
            on_line(trimmed);
            count += 1;
        }
    });

    fs.close();
    Some(count)
}

/// Load configuration with fallback to built-in defaults.
///
/// Tries to read `CONFIG.G` from the filesystem. If the file doesn't exist
/// (or the filesystem is unavailable), falls back to `defaults`.
/// Then optionally loads `CONFIGO.G` (saved overrides) on top.
///
/// This is the primary entry point for firmware startup config.
pub fn load_config_with_fallback<F, L>(
    fs: &mut F,
    defaults: &[&str],
    mut on_line: L,
) -> ConfigResult
where
    F: FileSystem,
    L: FnMut(&str),
{
    // Try CONFIG.G first
    let (base_count, from_file) = if let Some(n) = load_config_file(fs, "CONFIG.G", &mut on_line) {
        (n, true)
    } else {
        // Fall back to compiled-in defaults
        let mut count: u32 = 0;
        for line in defaults {
            on_line(line);
            count += 1;
        }
        (count, false)
    };

    // Try CONFIGO.G (saved overrides) on top of base config
    let override_count = load_config_file(fs, "CONFIGO.G", &mut on_line).unwrap_or(0);

    ConfigResult {
        commands_executed: base_count + override_count,
        from_file,
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::string::String;
    use std::vec;
    use std::vec::Vec;

    /// Test filesystem backed by in-memory files.
    struct MemFs {
        files: Vec<(&'static str, &'static [u8])>,
        open_idx: Option<usize>,
        read_pos: usize,
    }

    impl MemFs {
        fn new(files: Vec<(&'static str, &'static [u8])>) -> Self {
            Self {
                files,
                open_idx: None,
                read_pos: 0,
            }
        }

        fn empty() -> Self {
            Self::new(Vec::new())
        }
    }

    impl FileSystem for MemFs {
        fn exists(&mut self, path: &str) -> bool {
            self.files.iter().any(|(name, _)| *name == path)
        }

        fn open(&mut self, path: &str) -> bool {
            for (i, (name, _)) in self.files.iter().enumerate() {
                if *name == path {
                    self.open_idx = Some(i);
                    self.read_pos = 0;
                    return true;
                }
            }
            false
        }

        fn read(&mut self, buf: &mut [u8]) -> usize {
            if let Some(idx) = self.open_idx {
                let data = self.files[idx].1;
                if self.read_pos >= data.len() {
                    return 0;
                }
                let remaining = &data[self.read_pos..];
                let n = remaining.len().min(buf.len());
                buf[..n].copy_from_slice(&remaining[..n]);
                self.read_pos += n;
                n
            } else {
                0
            }
        }

        fn close(&mut self) {
            self.open_idx = None;
            self.read_pos = 0;
        }
    }

    #[test]
    fn test_load_config_file_reads_lines() {
        let mut fs = MemFs::new(std::vec![("CONFIG.G", b"G90\nM92 X80\nM203 X6000\n")]);
        let mut lines = Vec::new();
        let count = load_config_file(&mut fs, "CONFIG.G", &mut |line: &str| {
            lines.push(String::from(line));
        });
        assert_eq!(count, Some(3));
        assert_eq!(lines[0], "G90");
        assert_eq!(lines[1], "M92 X80");
        assert_eq!(lines[2], "M203 X6000");
    }

    #[test]
    fn test_load_config_file_skips_comments() {
        let mut fs = MemFs::new(std::vec![(
            "CONFIG.G",
            b"; Machine config\nG90\n; Speed\nM203 X6000\n",
        )]);
        let mut lines = Vec::new();
        let count = load_config_file(&mut fs, "CONFIG.G", &mut |line: &str| {
            lines.push(String::from(line));
        });
        assert_eq!(count, Some(2));
        assert_eq!(lines[0], "G90");
        assert_eq!(lines[1], "M203 X6000");
    }

    #[test]
    fn test_load_config_file_missing_returns_none() {
        let mut fs = MemFs::empty();
        let count = load_config_file(&mut fs, "CONFIG.G", &mut |_| {});
        assert_eq!(count, None);
    }

    #[test]
    fn test_fallback_uses_defaults_when_no_file() {
        let mut fs = MemFs::empty();
        let defaults = &["G90", "M92 X80"];
        let mut lines = Vec::new();
        let result = load_config_with_fallback(&mut fs, defaults, |line| {
            lines.push(String::from(line));
        });
        assert_eq!(result.commands_executed, 2);
        assert!(!result.from_file);
        assert_eq!(lines[0], "G90");
        assert_eq!(lines[1], "M92 X80");
    }

    #[test]
    fn test_fallback_prefers_file_over_defaults() {
        let mut fs = MemFs::new(std::vec![("CONFIG.G", b"M92 X200\n")]);
        let defaults = &["M92 X80"];
        let mut lines = Vec::new();
        let result = load_config_with_fallback(&mut fs, defaults, |line| {
            lines.push(String::from(line));
        });
        assert_eq!(result.commands_executed, 1);
        assert!(result.from_file);
        assert_eq!(lines[0], "M92 X200");
    }

    #[test]
    fn test_override_applied_on_top_of_config() {
        let mut fs = MemFs::new(std::vec![
            ("CONFIG.G", b"M92 X80\nG90\n"),
            ("CONFIGO.G", b"M92 X100\n"),
        ]);
        let defaults = &[];
        let mut lines = Vec::new();
        let result = load_config_with_fallback(&mut fs, defaults, |line| {
            lines.push(String::from(line));
        });
        assert_eq!(result.commands_executed, 3);
        assert!(result.from_file);
        assert_eq!(lines, vec!["M92 X80", "G90", "M92 X100"]);
    }

    #[test]
    fn test_override_applied_on_top_of_defaults() {
        let mut fs = MemFs::new(std::vec![("CONFIGO.G", b"M92 X100\n")]);
        let defaults = &["M92 X80", "G90"];
        let mut lines = Vec::new();
        let result = load_config_with_fallback(&mut fs, defaults, |line| {
            lines.push(String::from(line));
        });
        // 2 defaults + 1 override
        assert_eq!(result.commands_executed, 3);
        assert!(!result.from_file); // base config was from defaults
        assert_eq!(lines, vec!["M92 X80", "G90", "M92 X100"]);
    }

    #[test]
    fn test_chunked_read_works() {
        // File larger than READ_BUF_SIZE to test chunked reading
        let long_config = b"M92 X80\nM92 Y80\nM92 Z400\nM92 E420\nM203 X6000\nM203 Y6000\nM203 Z600\nM203 E3600\nM201 X500\nM201 Y500\nM201 Z100\nM201 E500\nM204 P500 T1000\nM906 X800\nM906 Y800\nM906 Z800\nM906 E800\nG90\n";
        let mut fs = MemFs::new(std::vec![("CONFIG.G", long_config.as_slice())]);
        let mut lines = Vec::new();
        let count = load_config_file(&mut fs, "CONFIG.G", &mut |line: &str| {
            lines.push(String::from(line));
        });
        assert_eq!(count, Some(18));
        assert_eq!(lines[0], "M92 X80");
        assert_eq!(lines[17], "G90");
    }
}
