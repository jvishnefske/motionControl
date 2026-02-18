//! Messages for the SD card actor.

/// Commands sent TO the SD card reader.
#[derive(Clone, Debug, defmt::Format)]
pub enum SdCardCommand {
    /// Load and execute config.g from SD card.
    LoadConfig,

    /// Load and execute config-override.g from SD card.
    LoadConfigOverride,

    /// Start executing a G-code file.
    StartJob {
        /// 8.3 filename (e.g., "PART1.GCO")
        filename: heapless::String<12>,
    },

    /// Pause the current job.
    PauseJob,

    /// Resume the current job.
    ResumeJob,

    /// Cancel the current job.
    CancelJob,
}

/// Status updates FROM the SD card reader.
#[derive(Clone, Debug, defmt::Format)]
pub enum SdCardStatus {
    /// Config file loaded successfully, N commands executed.
    ConfigLoaded {
        commands_executed: u32,
    },

    /// Job progress update.
    JobProgress {
        lines_processed: u32,
        percent_complete: u8,
    },

    /// Job completed.
    JobComplete,

    /// Error reading from SD card.
    Error(SdCardError),
}

#[derive(Clone, Debug, defmt::Format)]
pub enum SdCardError {
    CardNotFound,
    FileNotFound,
    ReadError,
    FilesystemError,
}
