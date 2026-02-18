//! # Actor Framework for Embedded Async Systems
//!
//! Zero-allocation actor infrastructure built on Embassy.
//! Provides typed mailboxes, system event bus, and priority-aware scheduling
//! so that control is always responsive — even during long-running motor moves.

#![no_std]

pub mod event_bus;
pub mod mailbox;
pub mod select;

pub use event_bus::{EventBus, EventPublisher, EventSubscriber};
pub use mailbox::{Mailbox, MailboxReceiver, MailboxSender};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

/// Default mutex type: safe from both thread and interrupt context on single-core Cortex-M.
pub type Mutex = CriticalSectionRawMutex;
